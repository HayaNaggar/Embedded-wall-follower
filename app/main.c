/**
 * @file    main.c
 * @brief   Wall-following robot – ATmega328P / Arduino Nano
 *
 * PIN ASSIGNMENTS
 *   US Left  TRIG → PC2 (A2)    US Left  ECHO → PC3 (A3)
 *   US Right TRIG → PC4 (A4)    US Right ECHO → PC5 (A5)
 *   US Front TRIG → PC0 (A0)    US Front ECHO → PC1 (A1)
 *   Motor L IN1   → PB0 (D8)    Motor L IN2   → PB1 (D9)   [physical RIGHT motor]
 *   Motor R IN3   → PB2 (D10)   Motor R IN4   → PB3 (D11)  [physical LEFT motor]
 *   PWM Left ENA  → PD5 (D5)    PWM Right ENB → PD6 (D6)
 *   Enc A Left    → PB4 (D12)   PCINT4  (physical left wheel)
 *   Enc B Left    → PB5 (D13)   direction sense (read only)
 *   Enc A Right   → PD7 (D7)    PCINT23 (physical right wheel)
 *   Enc B Right   → PD2 (D2)    direction sense
 *   IR Left       → PD3 (D3)    [physical RIGHT side sensor]
 *   IR Right      → PD4 (D4)    [physical LEFT  side sensor]
 *   UART TX       → PD1 (D1)    9600 baud
 *
 * PID SIGN (verified)
 *   error = dl - dr  (positive = robot right-of-centre, dl large, dr small)
 *   Positive output → code-"left" motor faster → physical RIGHT motor faster
 *   → robot steers LEFT → corrects rightward drift  ✓
 *   Motors physically swapped so the sign works out correctly.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#include "timer.h"
#include "uart.h"
#include "motor.h"

/* =========================================================================
 * Ultrasonic – side walls
 * ========================================================================= */
#define US_LEFT_TRIG  2u
#define US_LEFT_ECHO  3u
#define US_RIGHT_TRIG 4u
#define US_RIGHT_ECHO 5u
#define US_MIN_CM     2u
#define US_MAX_CM     60u
#define JUMP_LIMIT    6u

/* =========================================================================
 * Front ultrasonic
 * ========================================================================= */
#define US_FRONT_TRIG 0u
#define US_FRONT_ECHO 1u
#define FRONT_SLOW_CM 35u

/* =========================================================================
 * IR sensors  (active-LOW: LOW = wall present, HIGH = open space)
 * ========================================================================= */
#define IR_LEFT_BIT  3u
#define IR_RIGHT_BIT 4u

static inline uint8_t ir_left_wall(void)  { return (PIND & (1u << IR_LEFT_BIT))  ? 0u : 1u; }
static inline uint8_t ir_right_wall(void) { return (PIND & (1u << IR_RIGHT_BIT)) ? 0u : 1u; }

/* =========================================================================
 * Turn / speed parameters
 * ========================================================================= */
#define SPEED_SLOW          155u   /* approach speed (df <= FRONT_SLOW_CM)  */
#define SPEED_CREEP         130u   /* creep speed   (df <= FRONT_CREEP_CM)  */
#define FRONT_CREEP_CM       18u   /* second slow-down threshold             */
#define SPEED_TURN_OUTER    230u
#define SPEED_TURN_INNER    110u
#define SPEED_TURN_L_INNER  150u
#define TURN_DURATION_MS    900u   /* safety timeout only — enc exits first  */
#define POST_TURN_MS        250u

/* Encoder ticks for a 90° pivot.
 * Tune this on your robot: run one turn, read T ticks from UART, set here. */
#define TURN_90_TICKS       55u

/* =========================================================================
 * Encoder pins
 * ========================================================================= */
#define ENC_L_A_BIT 7u   /* PD7 = D7  — physically RIGHT wheel */
#define ENC_L_B_BIT 2u
#define ENC_R_A_BIT 4u   /* PB4 = D12 — physically LEFT wheel  */
#define ENC_R_B_BIT 5u   /* PB5 = D13 — direction read only    */

/* =========================================================================
 * Drive parameters
 * ========================================================================= */
#define BASE_SPEED   210u
#define MAX_SPEED    255u
#define MIN_SPEED     60u
#define RIGHT_REDUCE    0

/* =========================================================================
 * Wall PID — PID with anti-windup
 *
 * KP lowered to 4 to prevent oscillation; KD re-added (median3 readings
 * suppress sensor noise enough to make derivative useful).
 * Anti-windup: integral does not accumulate when output is already clamped
 * in the same direction as the current error.
 * ========================================================================= */
#define WALL_KP            7.0f
#define WALL_KI            0.3f
#define WALL_KD            0.3f
#define WALL_MAX_OUT      75.0f
#define WALL_INTEGRAL_LIM 50.0f
#define WALL_DEADBAND      0.5f

/* =========================================================================
 * Post-turn single-wall realignment
 * ========================================================================= */
#define POST_TURN_REALIGN_MS  600u
#define REALIGN_TARGET_CM    12.5f
#define REALIGN_KP            4.0f
#define REALIGN_MAX_OUT      60.0f
#define REALIGN_SPEED        180u

/* =========================================================================
 * Turn tracking — project output format:
 *   Turns: 5
 *   Sequence: L, R, R, L, L
 * ========================================================================= */
#define MAX_TURNS 30u

static uint8_t turn_count = 0u;
static char    turn_sequence[MAX_TURNS];

/* =========================================================================
 * Encoder ISRs
 * ========================================================================= */
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
    DDRD  &= ~((1u << ENC_L_A_BIT) | (1u << ENC_L_B_BIT));
    PORTD |=  (1u << ENC_L_A_BIT)  | (1u << ENC_L_B_BIT);
    DDRB  &= ~((1u << ENC_R_A_BIT) | (1u << ENC_R_B_BIT));
    PORTB |=  (1u << ENC_R_A_BIT)  | (1u << ENC_R_B_BIT);
    PCMSK2 |= (1u << PCINT23);
    PCMSK0 |= (1u << PCINT4);   /* PB4 = D12 only; PB5/D13 LED excluded */
    PCICR  |= (1u << PCIE2) | (1u << PCIE0);
    g_prev_pind = PIND;
    g_prev_pinb = PINB;
}

static void enc_reset(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_enc_left  = 0;
        g_enc_right = 0;
    }
}


/* =========================================================================
 * Ultrasonic helpers
 * ========================================================================= */
static void ultrasonic_init(void) {
    DDRC |=  (1u<<US_LEFT_TRIG)|(1u<<US_RIGHT_TRIG)|(1u<<US_FRONT_TRIG);
    DDRC &= ~((1u<<US_LEFT_ECHO)|(1u<<US_RIGHT_ECHO)|(1u<<US_FRONT_ECHO));
    PORTC &= ~((1u<<US_LEFT_TRIG)|(1u<<US_RIGHT_TRIG)|(1u<<US_FRONT_TRIG));
}

static uint32_t us_now(void) {
    uint32_t ms;
    uint16_t tc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tc = TCNT1;
        ms = timer_get_ms();
        if ((TIFR1 & (1u << OCF1A)) && tc < 4u) ms++;
    }
    return ms * 1000UL + (uint32_t)(tc >> 1u);
}

static uint16_t us_read_once(uint8_t trig, uint8_t echo) {
    uint32_t t0 = timer_get_ms();
    while ((PINC & (1u << echo)) && (timer_get_ms() - t0) < 10u);

    PORTC |= (1u << trig);
    uint32_t t_trig = us_now();
    while ((us_now() - t_trig) < 10u);
    PORTC &= ~(1u << trig);

    t0 = timer_get_ms();
    while (!(PINC & (1u << echo))) {
        if ((timer_get_ms() - t0) >= 4u) return 0u;
    }

    uint32_t echo_start = us_now();
    while (PINC & (1u << echo)) {
        if ((us_now() - echo_start) >= 8000u) return 0u;
    }
    uint32_t duration_us = us_now() - echo_start;
    if (duration_us == 0u) return 0u;
    return (uint16_t)(duration_us / 58u);
}

static uint16_t us_read_median3(uint8_t trig, uint8_t echo) {
    uint16_t s[3], tmp;
    s[0] = us_read_once(trig, echo);
    s[1] = us_read_once(trig, echo);
    s[2] = us_read_once(trig, echo);
    if (s[0] > s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    if (s[1] > s[2]) { tmp=s[1]; s[1]=s[2]; s[2]=tmp; }
    if (s[0] > s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    return s[1];
}

/* =========================================================================
 * UART helpers
 * ========================================================================= */
static void uart_print_i16(int16_t v) {
    char buf[7];
    uint8_t i = 0, neg = 0;
    if (v < 0) { neg = 1; v = (int16_t)-v; }
    if (v == 0) { UART_SendChar('0'); return; }
    while (v > 0) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    if (neg) UART_SendChar('-');
    while (i > 0) UART_SendChar(buf[--i]);
}


static void uart_print_u16(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { UART_SendChar('0'); return; }
    while (v > 0) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    while (i > 0) UART_SendChar(buf[--i]);
}

/* Sends the required project report format over UART. */
static void send_turn_report(void) {
    UART_SendString("Turns: ");
    uart_print_u16(turn_count);
    UART_SendString("\r\nSequence: ");
    for (uint8_t i = 0u; i < turn_count; i++) {
        if (i > 0u) UART_SendString(", ");
        UART_SendChar(turn_sequence[i]);
    }
    UART_SendString("\r\n");
}

/* =========================================================================
 * Wall PID
 * ========================================================================= */
static float wall_integral   = 0.0f;
static float wall_prev_error = 0.0f;

static float wall_pid(float dl, float dr, float dt, float *raw_error_out) {
    float error = dl - dr;
    *raw_error_out = error;

    if      (error >  WALL_DEADBAND) error -= WALL_DEADBAND;
    else if (error < -WALL_DEADBAND) error += WALL_DEADBAND;
    else                             error  = 0.0f;

    /* Anti-windup: skip integration when already saturated in same direction */
    float pre = WALL_KP * error + WALL_KI * wall_integral;
    if (!((pre >= WALL_MAX_OUT && error > 0.0f) ||
          (pre <= -WALL_MAX_OUT && error < 0.0f)))
        wall_integral += error * dt;

    if (wall_integral >  WALL_INTEGRAL_LIM) wall_integral =  WALL_INTEGRAL_LIM;
    if (wall_integral < -WALL_INTEGRAL_LIM) wall_integral = -WALL_INTEGRAL_LIM;

    float deriv = (error - wall_prev_error) / dt;
    wall_prev_error = error;

    float out = WALL_KP * error + WALL_KI * wall_integral + WALL_KD * deriv;
    if (out >  WALL_MAX_OUT) out =  WALL_MAX_OUT;
    if (out < -WALL_MAX_OUT) out = -WALL_MAX_OUT;
    return out;
}

static int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void) {
    timer_init();
    UART_Init(9600UL);
    ultrasonic_init();
    encoder_init();
    Motor_Init();

    DDRD  &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));
    PORTD &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));

    sei();

    UART_SendString("Wall-follower ready.\r\n");

    uint16_t last_good_l = 15u;
    uint16_t last_good_r = 15u;
    UART_SendString("Calibrating...\r\n");
    for (uint8_t wi = 0; wi < 5u; wi++) {
        uint16_t wl = us_read_median3(US_LEFT_TRIG, US_LEFT_ECHO);
        uint16_t wr = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);
        if (wl >= US_MIN_CM && wl <= US_MAX_CM) last_good_l = wl;
        if (wr >= US_MIN_CM && wr <= US_MAX_CM) last_good_r = wr;
    }
    UART_SendString("Go.\r\n");

    uint8_t  sensor_valid       = 1u;
    uint32_t prev_time          = timer_get_ms();
    enc_reset();

    typedef enum {
        S_STRAIGHT = 0,
        S_APPROACH,
        S_TURN_RIGHT,
        S_TURN_LEFT,
        S_POST_TURN,
        S_REALIGN
    } state_t;

    state_t  state              = S_STRAIGHT;
    uint32_t turn_start_ms      = 0u;
    uint32_t approach_start_ms  = 0u;
    uint32_t post_turn_start_ms = 0u;
    uint8_t  last_turn_right    = 0u;

    uint32_t ir_last_change_ms  = 0u;
    uint8_t  ir_last_left       = 1u;
    uint8_t  ir_last_right      = 1u;

    uint16_t last_valid_front   = 30u;
    uint8_t  grace              = 10u;

    while (1) {
        /* ================================================================
         * TURNING — bypass PID entirely while pivoting
         * Exit condition: both IR see walls AND front clear, OR timeout.
         * Minimum 500ms guard prevents premature exit at junction entry.
         * ================================================================ */
        if (state == S_TURN_RIGHT || state == S_TURN_LEFT) {
            if (state == S_TURN_RIGHT)
                Motor_TurnRight(SPEED_TURN_OUTER, SPEED_TURN_INNER);
            else
                Motor_TurnLeft(SPEED_TURN_L_INNER, SPEED_TURN_OUTER);

            uint32_t elapsed = timer_get_ms() - turn_start_ms;

            /* Read total absolute encoder ticks since turn started */
            int32_t enc_l_snap, enc_r_snap;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                enc_l_snap = g_enc_left;
                enc_r_snap = g_enc_right;
            }
            int32_t abs_l = (enc_l_snap < 0) ? -enc_l_snap : enc_l_snap;
            int32_t abs_r = (enc_r_snap < 0) ? -enc_r_snap : enc_r_snap;
            uint8_t enc_done = ((abs_l + abs_r) >= (int32_t)TURN_90_TICKS);

            /* 200ms minimum guard — let the robot physically start pivoting */
            if (elapsed >= 200u) {
                uint8_t timed_out = (elapsed >= TURN_DURATION_MS);

                if (enc_done || timed_out) {
                    if (enc_done)   UART_SendString("Turn done (enc)\r\n");
                    else            UART_SendString("Turn done (timeout)\r\n");

                    state         = S_POST_TURN;
                    turn_start_ms = timer_get_ms();
                    enc_reset();
                }
            }
            continue;
        }

        /* ================================================================
         * POST-TURN — short straight burst to clear junction geometry
         * ================================================================ */
        if (state == S_POST_TURN) {
            Motor_Forward((uint8_t)BASE_SPEED, (uint8_t)(BASE_SPEED - RIGHT_REDUCE));
            if ((timer_get_ms() - turn_start_ms) >= POST_TURN_MS) {
                last_valid_front = 100u;
                last_good_l      = 20u;
                last_good_r      = 20u;
                sensor_valid     = 0u;
                prev_time        = timer_get_ms();
                state            = S_REALIGN;
                turn_start_ms    = timer_get_ms();
                UART_SendString("Realign\r\n");
            }
            continue;
        }

        /* ================================================================
         * REALIGN — single-wall P-only follow for POST_TURN_REALIGN_MS.
         * After right turn → near wall on LEFT  → follow dl.
         * After left  turn → near wall on RIGHT → follow dr.
         * ================================================================ */
        if (state == S_REALIGN) {
            uint16_t dl_r = us_read_once(US_LEFT_TRIG, US_LEFT_ECHO);
            uint16_t dr_r = us_read_once(US_RIGHT_TRIG, US_RIGHT_ECHO);
            if (dl_r >= US_MIN_CM && dl_r <= US_MAX_CM) last_good_l = dl_r;
            if (dr_r >= US_MIN_CM && dr_r <= US_MAX_CM) last_good_r = dr_r;

            float wall_dist = last_turn_right ? (float)last_good_l
                                              : (float)last_good_r;
            float r_err  = wall_dist - REALIGN_TARGET_CM;
            float r_corr = REALIGN_KP * r_err;
            if (r_corr >  REALIGN_MAX_OUT) r_corr =  REALIGN_MAX_OUT;
            if (r_corr < -REALIGN_MAX_OUT) r_corr = -REALIGN_MAX_OUT;
            if (!last_turn_right) r_corr = -r_corr;

            int16_t ls_r = (int16_t)REALIGN_SPEED + (int16_t)r_corr;
            int16_t rs_r = (int16_t)REALIGN_SPEED - (int16_t)r_corr;
            ls_r = clamp16(ls_r, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
            rs_r = clamp16(rs_r, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
            Motor_Forward((uint8_t)ls_r, (uint8_t)rs_r);

            if ((timer_get_ms() - turn_start_ms) >= POST_TURN_REALIGN_MS) {
                wall_integral    = 0.0f;
                wall_prev_error  = 0.0f;
                enc_reset();
                ir_last_change_ms  = timer_get_ms();
                ir_last_left       = 1u;
                ir_last_right      = 1u;
                sensor_valid       = 1u;
                prev_time          = timer_get_ms();
                state              = S_STRAIGHT;
                post_turn_start_ms = timer_get_ms();
                UART_SendString("Resuming wall-follow\r\n");
            }
            continue;
        }

        /* ================================================================
         * Sensor reads
         * ================================================================ */
        uint16_t dl_raw = us_read_once(US_LEFT_TRIG, US_LEFT_ECHO);
        uint16_t dr_raw = us_read_once(US_RIGHT_TRIG, US_RIGHT_ECHO);
        uint16_t df_raw = us_read_once(US_FRONT_TRIG, US_FRONT_ECHO);
        if (df_raw == 0u) df_raw = 255u;

        if (df_raw != 255u) last_valid_front = df_raw;
        uint16_t df = last_valid_front;

        /* Jump filter — reject side-sensor spikes > JUMP_LIMIT cm */
        uint16_t dl = last_good_l;
        uint16_t dr = last_good_r;
        if (dl_raw >= US_MIN_CM && dl_raw <= US_MAX_CM) {
            uint16_t jump = (dl_raw > last_good_l) ? (dl_raw - last_good_l)
                                                   : (last_good_l - dl_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) { last_good_l = dl_raw; dl = dl_raw; }
        }
        if (dr_raw >= US_MIN_CM && dr_raw <= US_MAX_CM) {
            uint16_t jump = (dr_raw > last_good_r) ? (dr_raw - last_good_r)
                                                   : (last_good_r - dr_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) { last_good_r = dr_raw; dr = dr_raw; }
        }
        if (last_good_l > 0u && last_good_r > 0u) sensor_valid = 1u;

        uint32_t now = timer_get_ms();
        float dt = (float)(now - prev_time) * 0.001f;
        if (dt < 0.005f) dt = 0.005f;
        prev_time = now;

        if (grace > 0u) {
            grace--;
            Motor_Forward((uint8_t)BASE_SPEED, (uint8_t)(BASE_SPEED - RIGHT_REDUCE));
            continue;
        }

        uint8_t ir_left_val  = ir_left_wall();
        uint8_t ir_right_val = ir_right_wall();

        /* ================================================================
         * STOP — open space on all sides → send report and halt
         * ================================================================ */
        uint8_t side_l_open = (dl_raw == 0u || dl_raw > US_MAX_CM);
        uint8_t side_r_open = (dr_raw == 0u || dr_raw > US_MAX_CM);
        if (!ir_left_val && !ir_right_val
            && (df > FRONT_SLOW_CM) && (df < 255u)
            && side_l_open && side_r_open)
        {
            Motor_Stop();
            send_turn_report();
            UART_SendString("STOP: open space\r\n");
            while (1);
        }

        /* ================================================================
         * Post-turn settling: suppress FRONT_CLOSE for 500ms after turn
         * ================================================================ */
        uint32_t settle_elapsed     = timer_get_ms() - post_turn_start_ms;
        uint8_t  front_close_thresh = (settle_elapsed < 500u) ? 12u : FRONT_SLOW_CM;

        /* ================================================================
         * APPROACH
         * ================================================================ */
        if (state == S_STRAIGHT && df <= front_close_thresh) {
            state             = S_APPROACH;
            approach_start_ms = timer_get_ms();
            ir_last_change_ms = timer_get_ms();
            UART_SendString("Approach\r\n");
        }

        /* ================================================================
         * IR DEBOUNCE TURN DETECTION
         * Only fires when exactly one side is open for >= 200ms.
         * ================================================================ */
        if (state == S_APPROACH) {
            uint32_t approach_elapsed = timer_get_ms() - approach_start_ms;
            uint32_t ir_stable_time   = timer_get_ms() - ir_last_change_ms;

            uint8_t ir_left_now  = ir_left_wall();
            uint8_t ir_right_now = ir_right_wall();

            if ((ir_left_now != ir_last_left) || (ir_right_now != ir_last_right)) {
                ir_last_left      = ir_left_now;
                ir_last_right     = ir_right_now;
                ir_last_change_ms = timer_get_ms();
                ir_stable_time    = 0u;
            }

            if (ir_stable_time >= 80u) {
                uint8_t l_open = (ir_left_now  == 0u);
                uint8_t r_open = (ir_right_now == 0u);

                if (r_open && !l_open) {
                    if (turn_count < MAX_TURNS) turn_sequence[turn_count++] = 'L';
                    enc_reset();
                    state           = S_TURN_LEFT;
                    turn_start_ms   = timer_get_ms();
                    last_turn_right = 0u;
                    UART_SendString("Turn L\r\n");
                    continue;
                }
                if (l_open && !r_open) {
                    if (turn_count < MAX_TURNS) turn_sequence[turn_count++] = 'R';
                    enc_reset();
                    state           = S_TURN_RIGHT;
                    turn_start_ms   = timer_get_ms();
                    last_turn_right = 1u;
                    UART_SendString("Turn R\r\n");
                    continue;
                }
            }

            /* Safety timeout (2500ms) — force turn based on current IR */
            if (approach_elapsed > 2500u) {
                uint8_t ir_l_chk = ir_left_wall();
                uint8_t ir_r_chk = ir_right_wall();

                if (ir_r_chk == 0u) {
                    if (turn_count < MAX_TURNS) turn_sequence[turn_count++] = 'L';
                    enc_reset();
                    state             = S_TURN_LEFT;
                    turn_start_ms     = timer_get_ms();
                    last_turn_right   = 0u;
                    ir_last_change_ms = timer_get_ms();
                    UART_SendString("TIMEOUT Turn L\r\n");
                    continue;
                } else if (ir_l_chk == 0u) {
                    if (turn_count < MAX_TURNS) turn_sequence[turn_count++] = 'R';
                    enc_reset();
                    state             = S_TURN_RIGHT;
                    turn_start_ms     = timer_get_ms();
                    last_turn_right   = 1u;
                    ir_last_change_ms = timer_get_ms();
                    UART_SendString("TIMEOUT Turn R\r\n");
                    continue;
                } else {
                    /* Both walls still present — creep forward slowly */
                    Motor_Forward(50u, 50u);
                    UART_SendString("TIMEOUT BACKUP\r\n");
                    state             = S_APPROACH;
                    approach_start_ms = timer_get_ms();
                    continue;
                }
            }
        }

        /* ================================================================
         * Wall PID
         * ================================================================ */
        float raw_err = 0.0f;
        float w_corr  = 0.0f;
        if (sensor_valid) {
            w_corr = wall_pid((float)dl, (float)dr, dt, &raw_err);
        }

        /* ================================================================
         * Drive
         * ================================================================ */
        uint8_t base;
        if (state != S_APPROACH)
            base = (uint8_t)BASE_SPEED;
        else if (df <= FRONT_CREEP_CM)
            base = SPEED_CREEP;   /* very close — creep */
        else
            base = SPEED_SLOW;    /* approach zone      */
        int16_t ls = (int16_t)base + (int16_t)w_corr;
        int16_t rs = (int16_t)(base - RIGHT_REDUCE) - (int16_t)w_corr;
        ls = clamp16(ls, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        rs = clamp16(rs, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        Motor_Forward((uint8_t)ls, (uint8_t)rs);

        /* ================================================================
         * Debug UART — minimal fields to keep loop fast at 9600 baud
         * ================================================================ */
        UART_SendString("L=");  uart_print_i16((int16_t)dl);
        UART_SendString(" R="); uart_print_i16((int16_t)dr);
        UART_SendString(" e="); uart_print_i16((int16_t)raw_err);
        UART_SendString(" W="); uart_print_i16((int16_t)w_corr);
        UART_SendString(" F="); uart_print_i16((int16_t)df);
        UART_SendString(" S="); uart_print_i16((int16_t)state);
        UART_SendString(" T="); uart_print_u16(turn_count);
        UART_SendString("\r\n");
    }

    return 0;
}
