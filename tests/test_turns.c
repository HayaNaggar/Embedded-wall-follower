/**
 * TEST: 90-degree turn calibration using encoders + IR trigger.
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │  TUNE THESE until the turn is exactly 90°               │
 * │                                                         │
 * │  TURN_90_TICKS    — encoder ticks (L+R abs) for 90°    │
 * │  FRONT_ALIGN_CM   — how close to front wall before      │
 * │                     pivot starts (~10 cm)               │
 * │  TURN_OUTER / TURN_INNER / TURN_L_INNER                 │
 * └─────────────────────────────────────────────────────────┘
 *
 * Flow:
 *   FORWARD      → front-US speed scaling
 *   APPROACH     → df < APPROACH_CM, watch IR
 *   CREEP_CENTER → IR opened → creep forward until front ≤ FRONT_ALIGN_CM
 *   TURN_L/R     → 90° pivot using encoders only (IR ignored)
 *   POST_TURN    → straight burst, log result, repeat
 *
 * Track: 45 cm wide.  Car: 20 cm wide, 24 cm long.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#include "timer.h"
#include "uart.h"
#include "motor.h"

/* =========================================================================
 * TUNE THESE
 * ========================================================================= */
#define TURN_90_TICKS_L      400u   /* LEFT  90° ticks — calibrated           */
#define TURN_90_TICKS_R      280u   /* RIGHT 90° ticks — calibrated           */
#define FRONT_ALIGN_CM        10u   /* creep until front wall is this close   */
#define TURN_OUTER           230u
#define TURN_INNER           110u
#define TURN_TIMEOUT_MS     1500u   /* safety: abort pivot after this        */
#define CREEP_TIMEOUT_MS    3000u   /* safety: abort creep if no front wall  */

/* =========================================================================
 * Front ultrasonic speed scaling
 * ========================================================================= */
#define FRONT_TRIG      0u          /* PC0 */
#define FRONT_ECHO      1u          /* PC1 */
#define BASE_SPEED     180u
#define SLOW_SPEED      80u
#define CREEP_SPEED     60u   /* must be above motor stall — tune if needed */
#define SLOW_CM         70u
#define CREEP_CM        20u
#define APPROACH_CM     45u         /* enter approach zone below this        */

/* =========================================================================
 * IR sensors  (active-LOW: LOW = wall present → returns 1)
 *   IR Left  = D3 = PD3  (physical LEFT  side)
 *   IR Right = D4 = PD4  (physical RIGHT side)
 * ========================================================================= */
#define IR_LEFT_BIT   3u   /* D3 — LEFT  side */
#define IR_RIGHT_BIT  4u   /* D4 — RIGHT side */

static inline uint8_t ir_left_wall(void)  { return (PIND & (1u<<IR_LEFT_BIT))  ? 0u : 1u; }
static inline uint8_t ir_right_wall(void) { return (PIND & (1u<<IR_RIGHT_BIT)) ? 0u : 1u; }

/* =========================================================================
 * Encoder
 *   Enc A LEFT  = D7  = PD7  → g_enc_left   (PCINT23)
 *   Enc B LEFT  = D2  = PD2
 *   Enc A RIGHT = D12 = PB4  → g_enc_right  (PCINT4)
 *   Enc B RIGHT = D13 = PB5
 *
 * NOTE: Motor_TurnLeft/Right function names are physically SWAPPED.
 *       Motor_TurnRight → robot physically turns LEFT.
 *       Motor_TurnLeft  → robot physically turns RIGHT.
 * ========================================================================= */
#define ENC_L_A_BIT  7u   /* D7  — LEFT  wheel */
#define ENC_L_B_BIT  2u   /* D2  — LEFT  wheel direction */
#define ENC_R_A_BIT  4u   /* D12 — RIGHT wheel */
#define ENC_R_B_BIT  5u   /* D13 — RIGHT wheel direction */

static volatile int32_t g_enc_left  = 0;
static volatile int32_t g_enc_right = 0;
static volatile uint8_t g_prev_pind = 0u;
static volatile uint8_t g_prev_pinb = 0u;

ISR(PCINT2_vect) {
    uint8_t cur = PIND, changed = cur ^ g_prev_pind;
    g_prev_pind = cur;
    if (changed & (1u << ENC_L_A_BIT))
        if (cur & (1u << ENC_L_A_BIT))
            g_enc_left += ((cur >> ENC_L_B_BIT) & 1u) ? 1 : -1;
}

ISR(PCINT0_vect) {
    uint8_t cur = PINB, changed = cur ^ g_prev_pinb;
    g_prev_pinb = cur;
    if (changed & (1u << ENC_R_A_BIT))
        if (cur & (1u << ENC_R_A_BIT))
            g_enc_right += ((cur >> ENC_R_B_BIT) & 1u) ? -1 : 1;
}

static void encoder_init(void) {
    DDRD  &= ~((1u<<ENC_L_A_BIT)|(1u<<ENC_L_B_BIT));
    PORTD |=   (1u<<ENC_L_A_BIT)|(1u<<ENC_L_B_BIT);
    DDRB  &= ~((1u<<ENC_R_A_BIT)|(1u<<ENC_R_B_BIT));
    PORTB |=   (1u<<ENC_R_A_BIT)|(1u<<ENC_R_B_BIT);
    PCMSK2 |= (1u << PCINT23);
    PCMSK0 |= (1u << PCINT4);
    PCICR  |= (1u << PCIE2) | (1u << PCIE0);
    g_prev_pind = PIND;
    g_prev_pinb = PINB;
}

static void enc_reset(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_enc_left = 0; g_enc_right = 0; }
}

static int32_t enc_total_abs(void) {
    int32_t l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { l = g_enc_left; r = g_enc_right; }
    return (l < 0 ? -l : l) + (r < 0 ? -r : r);
}

/* =========================================================================
 * Front ultrasonic
 * ========================================================================= */
static uint32_t us_now(void) {
    uint32_t ms; uint16_t tc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tc = TCNT1;
        ms = timer_get_ms();
        if ((TIFR1 & (1u << OCF1A)) && tc < 4u) ms++;
    }
    return ms * 1000UL + (uint32_t)(tc >> 1u);
}

static uint16_t us_read_once(void) {
    uint32_t t0 = timer_get_ms();
    while ((PINC & (1u<<FRONT_ECHO)) && (timer_get_ms()-t0) < 10u);
    PORTC |= (1u<<FRONT_TRIG);
    uint32_t t_trig = us_now();
    while ((us_now()-t_trig) < 10u);
    PORTC &= ~(1u<<FRONT_TRIG);
    t0 = timer_get_ms();
    while (!(PINC & (1u<<FRONT_ECHO))) {
        if ((timer_get_ms()-t0) >= 4u) return 0u;
    }
    uint32_t echo_start = us_now();
    while (PINC & (1u<<FRONT_ECHO)) {
        if ((us_now()-echo_start) >= 8000u) return 0u;
    }
    uint32_t dur = us_now() - echo_start;
    return (dur == 0u) ? 0u : (uint16_t)(dur / 58u);
}

static uint16_t us_read(void) {
    uint16_t s[3], tmp;
    s[0] = us_read_once(); timer_delay_ms(30u);
    s[1] = us_read_once(); timer_delay_ms(30u);
    s[2] = us_read_once();
    if (s[0]>s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    if (s[1]>s[2]) { tmp=s[1]; s[1]=s[2]; s[2]=tmp; }
    if (s[0]>s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    return s[1];
}

/* =========================================================================
 * UART helpers
 * ========================================================================= */
static void print_u32(uint32_t v) {
    char buf[11]; uint8_t i = 0;
    if (v == 0u) { UART_SendChar('0'); return; }
    while (v > 0u) { buf[i++] = (char)('0' + v % 10u); v /= 10u; }
    while (i > 0u) UART_SendChar(buf[--i]);
}

/* =========================================================================
 * Front-distance speed
 * ========================================================================= */
static uint8_t front_speed(uint16_t df) {
    if (df == 0u) df = 255u;
    if (df >= SLOW_CM)  return BASE_SPEED;
    if (df <= CREEP_CM) return CREEP_SPEED;
    uint16_t range = SLOW_CM - CREEP_CM;
    uint16_t gap   = df     - CREEP_CM;
    return (uint8_t)(SLOW_SPEED + ((uint16_t)(BASE_SPEED - SLOW_SPEED) * gap) / range);
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void) {
    timer_init();
    UART_Init(9600UL);
    encoder_init();
    Motor_Init();

    DDRC |=  (1u<<FRONT_TRIG);
    DDRC &= ~(1u<<FRONT_ECHO);
    PORTC &= ~(1u<<FRONT_TRIG);

    DDRD  &= ~((1u<<IR_LEFT_BIT)|(1u<<IR_RIGHT_BIT));
    PORTD &= ~((1u<<IR_LEFT_BIT)|(1u<<IR_RIGHT_BIT));

    sei();
    UART_SendString("test_turns ready\r\n");
    UART_SendString("TICKS_L="); print_u32(TURN_90_TICKS_L);
    UART_SendString(" TICKS_R="); print_u32(TURN_90_TICKS_R);
    UART_SendString(" FRONT_ALIGN_CM="); print_u32(FRONT_ALIGN_CM);
    UART_SendString("\r\n");

    typedef enum {
        S_FORWARD,
        S_APPROACH,
        S_CREEP_CENTER,   /* creep forward until front wall close enough */
        S_TURN_LEFT,
        S_TURN_RIGHT,
        S_POST_TURN
    } state_t;

    state_t  state       = S_FORWARD;
    uint32_t phase_start = 0u;
    uint8_t  turn_num    = 0u;
    uint8_t  turn_dir    = 0u;   /* 0 = LEFT, 1 = RIGHT */

    while (1) {

        /* ================================================================
         * FORWARD — speed-scale by front US, watch for approach zone
         * ================================================================ */
        if (state == S_FORWARD) {
            uint16_t df  = us_read();
            uint8_t  spd = front_speed(df);
            Motor_Forward(spd, spd);

            UART_SendString("FWD D="); print_u32(df);
            UART_SendString(" S=");    print_u32(spd);
            UART_SendString("\r\n");

            if (df != 0u && df <= APPROACH_CM) {
                state = S_APPROACH;
                UART_SendString("Approach\r\n");
            }
            continue;
        }

        /* ================================================================
         * APPROACH — slow down, watch IR for junction opening
         * IL=0 → left wall gone  → turn LEFT
         * IR=0 → right wall gone → turn RIGHT
         * ================================================================ */
        if (state == S_APPROACH) {
            uint16_t df  = us_read();
            uint8_t  spd = front_speed(df);
            Motor_Forward(spd, spd);

            uint8_t il = ir_left_wall();
            uint8_t ir = ir_right_wall();

            UART_SendString("APR D="); print_u32(df);
            UART_SendString(" IL=");   UART_SendChar('0' + il);
            UART_SendString(" IR=");   UART_SendChar('0' + ir);
            UART_SendString("\r\n");

            if (il == 0u && ir == 1u) {
                turn_dir    = 0u;   /* LEFT */
                phase_start = timer_get_ms();
                state       = S_CREEP_CENTER;
                UART_SendString("IR: left open -> creep to align (LEFT)\r\n");
            } else if (ir == 0u && il == 1u) {
                turn_dir    = 1u;   /* RIGHT */
                phase_start = timer_get_ms();
                state       = S_CREEP_CENTER;
                UART_SendString("IR: right open -> creep to align (RIGHT)\r\n");
            }
            continue;
        }

        /* ================================================================
         * CREEP_CENTER — keep going forward slowly until front wall is
         * FRONT_ALIGN_CM away, then start the pivot.
         * IR is ignored here — we committed to the turn direction already.
         * ================================================================ */
        if (state == S_CREEP_CENTER) {
            Motor_Forward(CREEP_SPEED, CREEP_SPEED);

            uint16_t df      = us_read();
            uint32_t elapsed = timer_get_ms() - phase_start;

            UART_SendString("CRP D="); print_u32(df);
            UART_SendString(" t=");    print_u32(elapsed);
            UART_SendString("\r\n");

            uint8_t aligned = (df != 0u && df <= FRONT_ALIGN_CM);
            uint8_t timeout = (elapsed >= CREEP_TIMEOUT_MS);

            if (aligned || timeout) {
                Motor_Stop();
                if (timeout && !aligned)
                    UART_SendString("Creep timeout — pivoting anyway\r\n");
                else
                    UART_SendString("Aligned — pivoting\r\n");

                enc_reset();
                phase_start = timer_get_ms();
                turn_num++;
                state = (turn_dir == 0u) ? S_TURN_LEFT : S_TURN_RIGHT;
                if (turn_dir == 0u)
                    UART_SendString(">>> Pivot LEFT\r\n");
                else
                    UART_SendString(">>> Pivot RIGHT\r\n");
            }
            continue;
        }

        /* ================================================================
         * TURN LEFT — full pivot, encoders only, IR ignored.
         * Motor_TurnRight used because motors are physically swapped:
         * Motor_TurnRight → physical LEFT turn.
         * ================================================================ */
        if (state == S_TURN_LEFT) {
            Motor_TurnRight(TURN_OUTER, TURN_INNER);

            int32_t  ticks   = enc_total_abs();
            uint32_t elapsed = timer_get_ms() - phase_start;
            uint8_t  done    = (ticks >= (int32_t)TURN_90_TICKS_L);
            uint8_t  timeout = (elapsed >= TURN_TIMEOUT_MS);

            if (done || timeout) {
                Motor_Stop();
                UART_SendString("Turn #"); print_u32(turn_num);
                UART_SendString(" LEFT  ticks="); print_u32((uint32_t)ticks);
                UART_SendString(" time=");        print_u32(elapsed);
                UART_SendString("ms");
                if (timeout && !done) UART_SendString(" [TIMEOUT]");
                UART_SendString("\r\n");
                state = S_POST_TURN;
            }
            continue;
        }

        /* ================================================================
         * TURN RIGHT — full pivot, encoders only, IR ignored.
         * Motor_TurnLeft used because motors are physically swapped:
         * Motor_TurnLeft → physical RIGHT turn.
         * ================================================================ */
        if (state == S_TURN_RIGHT) {
            Motor_TurnLeft(TURN_INNER, TURN_OUTER);

            int32_t  ticks   = enc_total_abs();
            uint32_t elapsed = timer_get_ms() - phase_start;
            uint8_t  done    = (ticks >= (int32_t)TURN_90_TICKS_R);
            uint8_t  timeout = (elapsed >= TURN_TIMEOUT_MS);

            if (done || timeout) {
                Motor_Stop();
                UART_SendString("Turn #"); print_u32(turn_num);
                UART_SendString(" RIGHT ticks="); print_u32((uint32_t)ticks);
                UART_SendString(" time=");        print_u32(elapsed);
                UART_SendString("ms");
                if (timeout && !done) UART_SendString(" [TIMEOUT]");
                UART_SendString("\r\n");
                state = S_POST_TURN;
            }
            continue;
        }

        /* ================================================================
         * POST_TURN — short straight burst so you can observe alignment,
         * then back to FORWARD
         * ================================================================ */
        if (state == S_POST_TURN) {
            UART_SendString("Post-turn burst\r\n");
            Motor_Forward(BASE_SPEED, BASE_SPEED);
            timer_delay_ms(600u);
            Motor_Stop();
            timer_delay_ms(300u);
            state = S_FORWARD;
            UART_SendString("--- ready for next ---\r\n");
            continue;
        }
    }

    return 0;
}
