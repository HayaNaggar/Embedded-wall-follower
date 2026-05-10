/**
 * @file    main.c
 * @brief   Autonomous wall-following maze robot — ATmega328P / Arduino Nano
 *
 * Hardware wiring
 * ---------------
 *   US Front  TRIG→PC0(A0)  ECHO→PC1(A1)   front approach + creep alignment
 *   US Left   TRIG→PC2(A2)  ECHO→PC3(A3)   left wall distance
 *   US Right  TRIG→PC4(A4)  ECHO→PC5(A5)   right wall distance
 *   Motor code-L  IN1→PB0  IN2→PB1          physical RIGHT motor (swapped)
 *   Motor code-R  IN3→PB2  IN4→PB3          physical LEFT  motor (swapped)
 *   PWM  ENA → PD5 (OC0B)                   enables code-L / physical RIGHT
 *   PWM  ENB → PD6 (OC0A)                   enables code-R / physical LEFT
 *   Encoder L  A→PD7(PCINT23)  B→PD2        physical right wheel
 *   Encoder R  A→PB4(PCINT4)   B→PB5        physical left  wheel
 *   IR Left  → PD3                           physical LEFT  side sensor
 *   IR Right → PD4                           physical RIGHT side sensor
 *   UART TX  → PD1   9600 baud              PC turn report
 *
 * Motor swap note
 * ---------------
 *   PB0/PB1 are labelled "code-left" in motor.c but drive the PHYSICAL RIGHT
 *   motor.  Turn states compensate by calling the opposite Motor_Turn* function:
 *     S_TURN_LEFT  → Motor_TurnRight(OUTER, INNER)  → physical LEFT  pivot
 *     S_TURN_RIGHT → Motor_TurnLeft (INNER, OUTER)  → physical RIGHT pivot
 *   (Verified correct in the test_turns calibration sketch.)
 *
 * Wall PID sign convention
 * ------------------------
 *   error = dl - dr  (positive → robot drifted RIGHT of centre)
 *   Positive output → code-left motor faster → physical RIGHT faster
 *   → robot steers LEFT → corrects rightward drift  ✓
 *
 * FSM states
 * ----------
 *   S_STRAIGHT    Dual-wall PID + encoder straight-line correction
 *   S_APPROACH    Front ≤ FRONT_SLOW_CM; slow; watch IR sensors
 *   S_CREEP_CENTER IR committed direction; creep until front ≤ FRONT_ALIGN_CM
 *   S_TURN_LEFT   Encoder-controlled 90° left pivot
 *   S_TURN_RIGHT  Encoder-controlled 90° right pivot
 *   S_POST_TURN   Brief straight burst to clear junction geometry
 *   S_REALIGN     Single-wall P-only follow before dual-PID resumes
 *
 * No _delay_ms or _delay_us are used anywhere in this file.
 * All timing uses timer_get_ms() and timer_get_us() from the timer driver.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#include "timer.h"
#include "uart.h"
#include "motor.h"
#include "ultrasonic.h"

/* =========================================================================
 * IR sensors — active-LOW (LOW pin = wall present → function returns 1)
 * ========================================================================= */
#define IR_LEFT_BIT   3u   /* PD3 = D3  — physical LEFT  side */
#define IR_RIGHT_BIT  4u   /* PD4 = D4  — physical RIGHT side */

static inline uint8_t ir_left_wall(void)  { return (PIND & (1u<<IR_LEFT_BIT))  ? 0u : 1u; }
static inline uint8_t ir_right_wall(void) { return (PIND & (1u<<IR_RIGHT_BIT)) ? 0u : 1u; }

/* =========================================================================
 * Turn parameters  (tuned via test_turns calibration sketch)
 *
 *   TURN_90_TICKS_L / _R — measure actual ticks from UART output of the
 *   test sketch and paste them here.  400/280 are reasonable starting points.
 * ========================================================================= */
#define TURN_OUTER          230u
#define TURN_INNER          110u
#define TURN_90_TICKS_L     420u   /* sum of |left| + |right| encoder ticks for left  90° */
#define TURN_90_TICKS_R     380u   /* sum of |left| + |right| encoder ticks for right 90° */
#define FRONT_ALIGN_CM       18u   /* creep until front wall ≤ this before pivoting (cm)  */
#define CREEP_SPEED          50u   /* PWM during S_CREEP_CENTER (above stall threshold)    */
#define TURN_TIMEOUT_MS    1500u   /* safety: abort pivot after this (ms)                  */
#define CREEP_TIMEOUT_MS   3000u   /* safety: abort creep if no front wall found (ms)      */
#define POST_TURN_MS        200u   /* straight burst after pivot to clear junction (ms)     */

/* =========================================================================
 * Drive parameters
 * ========================================================================= */
#define BASE_SPEED   210u   /* normal forward PWM (0–255)              */
#define SPEED_SLOW   80u   /* maximum speed cap while in S_APPROACH   */
#define MAX_SPEED    255u
#define MIN_SPEED     55u
#define RIGHT_REDUCE   0    /* trim if one physical motor runs faster   */

/* =========================================================================
 * Front-distance speed ramp (active in S_STRAIGHT)
 *
 *   df ≥ FRONT_FULL_CM  →  BASE_SPEED
 *   df ≤ FRONT_STOP_CM  →  SPEED_MIN_FWD
 *   in between          →  linear interpolation
 *
 * In S_APPROACH the ramp output is capped at SPEED_SLOW so the robot
 * always arrives at a junction slowly regardless of front distance.
 * ========================================================================= */
#define FRONT_SLOW_CM    50u   /* enter S_APPROACH when front ≤ this (cm)  */
#define FRONT_FULL_CM   80u   /* ramp starts reducing speed below this     */
#define FRONT_STOP_CM    12u   /* minimum approach distance                 */
#define SPEED_MIN_FWD    40u   /* minimum drive speed (above motor stall)   */

/* =========================================================================
 * Wall-centering PID
 *   Gains: start with KI = 0; enable only after KP and KD are stable.
 * ========================================================================= */
#define WALL_KP            5.5f   /* raised from 4.5 for smoother/ceramic floor */
#define WALL_KI            0.0f
#define WALL_KD            1.5f   /* raised from 1.0 — more damping to prevent overshoot on low-grip surface */
#define WALL_MAX_OUT      95.0f   /* raised from 85 — more correction authority */
#define WALL_INTEGRAL_LIM 30.0f
#define WALL_DEADBAND      0.3f
#define WALL_DERIV_ALPHA   0.15f  /* reduced from 0.3 — faster derivative response on ceramic */

/* =========================================================================
 * Post-turn single-wall realignment
 *   Right turn → new close wall on LEFT  → follow left  US at REALIGN_TARGET.
 *   Left  turn → new close wall on RIGHT → follow right US at REALIGN_TARGET.
 * ========================================================================= */
#define POST_TURN_REALIGN_MS  600u
#define REALIGN_TARGET_CM    12.5f
#define REALIGN_KP            4.0f
#define REALIGN_MAX_OUT      60.0f
#define REALIGN_SPEED        180u

/* =========================================================================
 * Encoder straight-line PID (active only when wall error < deadband)
 * ========================================================================= */
#define ENC_KP       0.8f
#define ENC_KD       0.1f
#define ENC_MAX_OUT 12.0f

/* =========================================================================
 * Side US jump filter — rejects single-sample spikes
 * ========================================================================= */
#define US_MIN_CM    2u
#define US_MAX_CM   60u
#define JUMP_LIMIT   6u

/* =========================================================================
 * Turn log (sent over UART when robot stops)
 * ========================================================================= */
#define MAX_TURNS 30u

/* =========================================================================
 * Encoder ISRs
 *
 *   Left  encoder: A → PD7 (PCINT23),  B → PD2   (physical right wheel)
 *   Right encoder: A → PB4 (PCINT4),   B → PB5   (physical left  wheel)
 *
 * The ISRs live here (not in a separate driver) because interrupt vectors
 * must be defined in exactly one translation unit linked into the image.
 * ========================================================================= */
#define ENC_L_A_BIT 7u
#define ENC_L_B_BIT 2u
#define ENC_R_A_BIT 4u
#define ENC_R_B_BIT 5u

static volatile int32_t g_enc_left  = 0;
static volatile int32_t g_enc_right = 0;
static volatile uint8_t g_prev_pind = 0u;
static volatile uint8_t g_prev_pinb = 0u;

ISR(PCINT2_vect)
{
    uint8_t cur     = PIND;
    uint8_t changed = cur ^ g_prev_pind;
    g_prev_pind     = cur;
    if (changed & (1u << ENC_L_A_BIT))
        if (cur & (1u << ENC_L_A_BIT))
            g_enc_left += ((cur >> ENC_L_B_BIT) & 1u) ? 1 : -1;
}

ISR(PCINT0_vect)
{
    uint8_t cur     = PINB;
    uint8_t changed = cur ^ g_prev_pinb;
    g_prev_pinb     = cur;
    if (changed & (1u << ENC_R_A_BIT))
        if (cur & (1u << ENC_R_A_BIT))
            g_enc_right += ((cur >> ENC_R_B_BIT) & 1u) ? -1 : 1;
}

static void encoder_init(void)
{
    /* Inputs with pull-ups */
    DDRD  &= ~((1u << ENC_L_A_BIT) | (1u << ENC_L_B_BIT));
    PORTD |=   (1u << ENC_L_A_BIT) | (1u << ENC_L_B_BIT);
    DDRB  &= ~((1u << ENC_R_A_BIT) | (1u << ENC_R_B_BIT));
    PORTB |=   (1u << ENC_R_A_BIT) | (1u << ENC_R_B_BIT);

    /* Enable pin-change interrupts */
    PCMSK2 |= (1u << PCINT23);   /* PD7 */
    PCMSK0 |= (1u << PCINT4);    /* PB4 */
    PCICR  |= (1u << PCIE2) | (1u << PCIE0);

    g_prev_pind = PIND;
    g_prev_pinb = PINB;
}

static void enc_reset(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_enc_left  = 0;
        g_enc_right = 0;
    }
}

static void enc_snapshot(int32_t *l, int32_t *r)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *l = g_enc_left;
        *r = g_enc_right;
    }
}

static int32_t enc_total_abs(void)
{
    int32_t l, r;
    enc_snapshot(&l, &r);
    return (l < 0 ? -l : l) + (r < 0 ? -r : r);
}

/* =========================================================================
 * UART integer print helpers (no stdio / no float formatting)
 * ========================================================================= */
static void print_u16(uint16_t v)
{
    char    buf[6];
    uint8_t i = 0;
    if (v == 0u) { UART_SendChar('0'); return; }
    while (v)    { buf[i++] = (char)('0' + v % 10u); v /= 10u; }
    while (i)    UART_SendChar(buf[--i]);
}

static void print_i16(int16_t v)
{
    if (v < 0) { UART_SendChar('-'); v = (int16_t)-v; }
    print_u16((uint16_t)v);
}

static void send_turn_report(uint8_t count, const char *seq, uint32_t elapsed_ms)
{
    uint32_t secs = elapsed_ms / 1000UL;
    uint16_t ms   = (uint16_t)(elapsed_ms % 1000UL);

    UART_SendString("Time: ");
    print_u16((uint16_t)secs);
    UART_SendChar('.');
    if (ms < 100u) UART_SendChar('0');
    if (ms <  10u) UART_SendChar('0');
    print_u16(ms);
    UART_SendString("s\r\nTurns: ");
    print_u16(count);
    UART_SendString("\r\nSeq: ");
    for (uint8_t i = 0u; i < count; i++) {
        if (i) UART_SendChar(',');
        UART_SendChar(seq[i]);
    }
    UART_SendString("\r\n");
}

/* =========================================================================
 * Wall-centering PID
 * ========================================================================= */
static float wall_integral   = 0.0f;
static float wall_prev_error = 0.0f;
static float wall_deriv_filt = 0.0f;

static float wall_pid(float dl, float dr, float dt, float *err_out)
{
    float err  = dl - dr;
    *err_out   = err;

    /* Deadband — ignore small errors, bleed integral when inside it */
    if      (err >  WALL_DEADBAND) err -= WALL_DEADBAND;
    else if (err < -WALL_DEADBAND) err += WALL_DEADBAND;
    else { err = 0.0f; wall_integral *= 0.9f; }

    /* Anti-windup: only accumulate integral when output is not saturated */
    float pre = WALL_KP * err + WALL_KI * wall_integral;
    if (pre < WALL_MAX_OUT && pre > -WALL_MAX_OUT)
        wall_integral += err * dt;
    if (wall_integral >  WALL_INTEGRAL_LIM) wall_integral =  WALL_INTEGRAL_LIM;
    if (wall_integral < -WALL_INTEGRAL_LIM) wall_integral = -WALL_INTEGRAL_LIM;

    /* Low-pass filtered derivative — suppresses noise spikes */
    float raw_d      = (err - wall_prev_error) / dt;
    wall_deriv_filt  = WALL_DERIV_ALPHA * wall_deriv_filt
                     + (1.0f - WALL_DERIV_ALPHA) * raw_d;
    wall_prev_error  = err;

    float out = WALL_KP * err + WALL_KI * wall_integral + WALL_KD * wall_deriv_filt;
    if (out >  WALL_MAX_OUT) out =  WALL_MAX_OUT;
    if (out < -WALL_MAX_OUT) out = -WALL_MAX_OUT;
    return out;
}

/* =========================================================================
 * Encoder straight-line PID
 * ========================================================================= */
static float enc_prev_error = 0.0f;

static float enc_pid(int32_t l, int32_t r, float dt)
{
    float err   = (float)l - (float)r;
    float deriv = (err - enc_prev_error) / dt;
    enc_prev_error = err;
    float out   = ENC_KP * err + ENC_KD * deriv;
    if (out >  ENC_MAX_OUT) out =  ENC_MAX_OUT;
    if (out < -ENC_MAX_OUT) out = -ENC_MAX_OUT;
    return out;
}

/* =========================================================================
 * Utility
 * ========================================================================= */
static int16_t clamp16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void)
{
    /* ── Driver initialisation ───────────────────────────────────────────── */
    timer_init();
    UART_Init(9600UL);
    US_Init();
    encoder_init();
    Motor_Init();

    /* IR sensor pins: inputs, no pull-up (sensors have their own output) */
    DDRD  &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));
    PORTD &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));

    sei();
    UART_SendString("Robot ready.\r\n");

    /* ── Sensor warm-up: prime the jump filter with real readings ─────────── */
    uint16_t last_good_l = 15u;
    uint16_t last_good_r = 15u;
    for (uint8_t i = 0u; i < 5u; i++) {
        uint16_t wl = US_ReadCm(US_LEFT_TRIG,  US_LEFT_ECHO);
        uint16_t wr = US_ReadCm(US_RIGHT_TRIG, US_RIGHT_ECHO);
        if (wl >= US_MIN_CM && wl <= US_MAX_CM) last_good_l = wl;
        if (wr >= US_MIN_CM && wr <= US_MAX_CM) last_good_r = wr;
    }
    UART_SendString("Go.\r\n");
    uint32_t maze_start_ms = timer_get_ms();
    enc_reset();

    /* ── FSM definition ──────────────────────────────────────────────────── */
    typedef enum {
        S_STRAIGHT = 0,   /* dual-wall PID centering + encoder correction */
        S_APPROACH,       /* front ≤ FRONT_SLOW_CM; slow; watch IR        */
        S_CREEP_CENTER,   /* direction committed; creep to align           */
        S_TURN_LEFT,      /* encoder-controlled left pivot                 */
        S_TURN_RIGHT,     /* encoder-controlled right pivot                */
        S_POST_TURN,      /* straight burst to clear junction              */
        S_REALIGN         /* single-wall P-only follow after turn          */
    } state_t;

    state_t  state              = S_STRAIGHT;
    uint32_t turn_start_ms      = 0u;
    uint32_t approach_start_ms  = 0u;
    uint32_t post_turn_start_ms = 0u;
    uint8_t  last_turn_right    = 0u;

    /* IR debounce — microsecond-resolution stability timer */
    uint32_t ir_stable_us   = 0u;   /* timer_get_us() when IR became stable */
    uint8_t  ir_last_left   = 1u;
    uint8_t  ir_last_right  = 1u;

    uint16_t last_valid_front = 100u;
    uint8_t  sensor_valid     = 1u;
    uint8_t  us_rr            = 0u;   /* round-robin: 0=left, 1=right, 2=front */

    /* Stop detection — ms timestamp of last valid wall echo per side.
     * When both sides have been silent for >= WALL_ABSENT_MS the robot has
     * exited the maze.  Using time (not a count) makes this immune to the
     * round-robin pattern and HC-SR04 timeout noise. */
    uint32_t l_wall_ms = timer_get_ms();
    uint32_t r_wall_ms = timer_get_ms();
#define WALL_ABSENT_MS 400u

    uint8_t  turn_count       = 0u;
    char     turn_seq[MAX_TURNS];

    uint32_t prev_time        = timer_get_ms();
    uint32_t grace_start_ms   = timer_get_ms();   /* 200 ms boot grace */
    uint8_t  print_cnt        = 0u;

    /* ── Main loop ────────────────────────────────────────────────────────── */
    while (1) {

        /* ================================================================
         * S_TURN_LEFT
         * 90° pivot physically LEFT.  Motor_TurnRight used because
         * code-left = physical right (motors are physically swapped).
         * Exit: encoder ticks ≥ TURN_90_TICKS_L  OR  safety timeout.
         * ================================================================ */
        if (state == S_TURN_LEFT) {
            Motor_TurnRight(TURN_OUTER, TURN_INNER);
            int32_t  ticks   = enc_total_abs();
            uint32_t elapsed = timer_get_ms() - turn_start_ms;
            if (ticks >= (int32_t)TURN_90_TICKS_L || elapsed >= TURN_TIMEOUT_MS) {
                Motor_Stop();
                UART_SendString("TL ticks="); print_u16((uint16_t)ticks);
                UART_SendString(elapsed >= TURN_TIMEOUT_MS ? " TO\r\n" : " ok\r\n");
                state         = S_POST_TURN;
                turn_start_ms = timer_get_ms();
                enc_reset();
            }
            continue;
        }

        /* ================================================================
         * S_TURN_RIGHT
         * 90° pivot physically RIGHT.  Motor_TurnLeft used because
         * code-right = physical left (motors are physically swapped).
         * ================================================================ */
        if (state == S_TURN_RIGHT) {
            Motor_TurnLeft(TURN_INNER, TURN_OUTER);
            int32_t  ticks   = enc_total_abs();
            uint32_t elapsed = timer_get_ms() - turn_start_ms;
            if (ticks >= (int32_t)TURN_90_TICKS_R || elapsed >= TURN_TIMEOUT_MS) {
                Motor_Stop();
                UART_SendString("TR ticks="); print_u16((uint16_t)ticks);
                UART_SendString(elapsed >= TURN_TIMEOUT_MS ? " TO\r\n" : " ok\r\n");
                state         = S_POST_TURN;
                turn_start_ms = timer_get_ms();
                enc_reset();
            }
            continue;
        }

        /* ================================================================
         * S_CREEP_CENTER
         * IR has committed the turn direction.  Creep forward at low speed
         * until the front US reads ≤ FRONT_ALIGN_CM, then begin the pivot.
         * IR is ignored here — direction was already decided.
         * ================================================================ */
        if (state == S_CREEP_CENTER) {
            Motor_Forward(CREEP_SPEED, CREEP_SPEED);
            uint16_t df_c    = US_ReadCm(US_FRONT_TRIG, US_FRONT_ECHO);
            uint32_t elapsed = timer_get_ms() - turn_start_ms;
            uint8_t  aligned = (df_c != 0u && df_c <= (uint16_t)FRONT_ALIGN_CM);
            uint8_t  timeout = (elapsed >= (uint32_t)CREEP_TIMEOUT_MS);
            if (aligned || timeout) {
                Motor_Stop();
                UART_SendString(aligned ? "Aligned\r\n" : "CrpTO\r\n");
                enc_reset();
                turn_start_ms = timer_get_ms();
                state = last_turn_right ? S_TURN_RIGHT : S_TURN_LEFT;
            }
            continue;
        }

        /* ================================================================
         * S_POST_TURN
         * Drive straight at full speed for POST_TURN_MS to pull the robot
         * clear of the junction corner before sensors are trusted again.
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
         * S_REALIGN
         * Single-wall P-only controller for POST_TURN_REALIGN_MS.
         * Brings the robot parallel to the new corridor wall before the
         * full dual-wall PID re-engages.
         *   Right turn → near wall is on LEFT  → follow left  US
         *   Left  turn → near wall is on RIGHT → follow right US
         * ================================================================ */
        if (state == S_REALIGN) {
            /* Read ONLY the relevant wall sensor — firing both simultaneously
             * causes HC-SR04 cross-talk (both return 0) and leaves last_good
             * at the reset default (20 cm), which generates a large spurious
             * correction that drives the robot the wrong way for 600 ms. */
            float r_corr = 0.0f;
            if (last_turn_right) {
                uint16_t d = US_ReadCm(US_LEFT_TRIG, US_LEFT_ECHO);
                if (d >= US_MIN_CM && d <= US_MAX_CM) last_good_l = d;
                if (last_good_l >= US_MIN_CM && last_good_l <= US_MAX_CM)
                    r_corr = REALIGN_KP * ((float)last_good_l - REALIGN_TARGET_CM);
            } else {
                uint16_t d = US_ReadCm(US_RIGHT_TRIG, US_RIGHT_ECHO);
                if (d >= US_MIN_CM && d <= US_MAX_CM) last_good_r = d;
                if (last_good_r >= US_MIN_CM && last_good_r <= US_MAX_CM)
                    r_corr = REALIGN_KP * ((float)last_good_r - REALIGN_TARGET_CM);
                r_corr = -r_corr;
            }
            if (r_corr >  REALIGN_MAX_OUT) r_corr =  REALIGN_MAX_OUT;
            if (r_corr < -REALIGN_MAX_OUT) r_corr = -REALIGN_MAX_OUT;

            int16_t ls_r = clamp16((int16_t)REALIGN_SPEED + (int16_t)r_corr,
                                   (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
            int16_t rs_r = clamp16((int16_t)REALIGN_SPEED - (int16_t)r_corr,
                                   (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
            Motor_Forward((uint8_t)ls_r, (uint8_t)rs_r);

            if ((timer_get_ms() - turn_start_ms) >= POST_TURN_REALIGN_MS) {
                /* Reset all PID and IR state before handing off to S_STRAIGHT */
                wall_integral    = 0.0f;
                wall_prev_error  = 0.0f;
                wall_deriv_filt  = 0.0f;
                enc_prev_error   = 0.0f;
                enc_reset();
                ir_last_left     = 1u;
                ir_last_right    = 1u;
                ir_stable_us     = 0u;
                l_wall_ms        = timer_get_ms();
                r_wall_ms        = timer_get_ms();
                sensor_valid     = 1u;
                prev_time        = timer_get_ms();
                post_turn_start_ms = timer_get_ms();
                grace_start_ms   = timer_get_ms();
                state            = S_STRAIGHT;
                UART_SendString("Wall-follow\r\n");
            }
            continue;
        }

        /* ================================================================
         * SENSOR READS — strict round-robin, one ping per loop.
         *
         * Firing two HC-SR04 sensors back-to-back causes acoustic cross-talk:
         * the first sensor's sound wave arrives at the second sensor's echo
         * pin, returning a false 0.  Rotating through left → right → front
         * (one per loop, ~10 ms apart) eliminates this entirely.  Each side
         * still updates every ~30 ms — sufficient for 12 cm/s wall following.
         * ================================================================ */
        uint8_t  rr_now = us_rr;
        uint16_t dl_raw = last_good_l;       /* default: last cached reading */
        uint16_t dr_raw = last_good_r;
        uint16_t df_raw = last_valid_front;

        if      (rr_now == 0u) dl_raw = US_ReadCm(US_LEFT_TRIG,  US_LEFT_ECHO);
        else if (rr_now == 1u) dr_raw = US_ReadCm(US_RIGHT_TRIG, US_RIGHT_ECHO);
        else                   df_raw = US_ReadCm(US_FRONT_TRIG, US_FRONT_ECHO);

        us_rr = (uint8_t)((us_rr + 1u) % 3u);

        /* Update wall-presence timestamps on fresh valid side reads only */
        if (rr_now == 0u && dl_raw >= US_MIN_CM && dl_raw <= US_MAX_CM)
            l_wall_ms = timer_get_ms();
        if (rr_now == 1u && dr_raw >= US_MIN_CM && dr_raw <= US_MAX_CM)
            r_wall_ms = timer_get_ms();

        if (df_raw == 0u) df_raw = 255u;                /* timeout → treat as far */
        if (df_raw != 255u) last_valid_front = df_raw;
        uint16_t df = last_valid_front;

        /* Jump filter — reject outward spikes only; always accept inward jumps
         * so the PID reacts immediately when the robot drifts into a wall. */
        uint16_t dl = last_good_l;
        uint16_t dr = last_good_r;
        if (dl_raw >= US_MIN_CM && dl_raw <= US_MAX_CM) {
            uint16_t j_out = (dl_raw > last_good_l) ? (dl_raw - last_good_l) : 0u;
            if (j_out <= JUMP_LIMIT || !sensor_valid) { last_good_l = dl_raw; dl = dl_raw; }
        }
        if (dr_raw >= US_MIN_CM && dr_raw <= US_MAX_CM) {
            uint16_t j_out = (dr_raw > last_good_r) ? (dr_raw - last_good_r) : 0u;
            if (j_out <= JUMP_LIMIT || !sensor_valid) { last_good_r = dr_raw; dr = dr_raw; }
        }
        if (last_good_l > 0u && last_good_r > 0u) sensor_valid = 1u;

        /* dt for PID — clamped to avoid zero-division or huge derivative spikes */
        uint32_t now = timer_get_ms();
        float    dt  = (float)(now - prev_time) * 0.001f;
        if (dt < 0.005f) dt = 0.005f;
        prev_time = now;

        /* Encoder delta for straight-line PID (reset each loop) */
        int32_t el, er;
        enc_snapshot(&el, &er);
        enc_reset();

        /* Boot grace: drive straight for 200 ms before trusting any sensor */
        if ((timer_get_ms() - grace_start_ms) < 200u) {
            Motor_Forward((uint8_t)BASE_SPEED, (uint8_t)(BASE_SPEED - RIGHT_REDUCE));
            continue;
        }

        /* IR readings — sampled once per loop for stop check, debug, and S_APPROACH */
        uint8_t il_now = ir_left_wall();
        uint8_t ir_now = ir_right_wall();

        /* ================================================================
         * STOP — maze exit detection.
         *
         * Primary: BOTH IR sensors see no wall — the most reliable signal
         * that the robot has left the maze (in any corridor at least one
         * IR always sees a wall).
         *
         * Secondary guard: at least ONE side US has been silent for
         * >= WALL_ABSENT_MS — prevents stopping in a momentary wide section
         * where both IRs briefly see nothing.  Only one side required because
         * exit walls often extend along one side beyond the maze boundary.
         *
         * Checked in S_STRAIGHT and S_APPROACH (the stop zone can be reached
         * while the front US has already triggered approach mode).
         * Timestamps are reset after every turn so the US guard cannot be
         * pre-satisfied from a previous corridor.
         * ================================================================ */
        {
            uint8_t no_l_us  = (timer_get_ms() - l_wall_ms) >= WALL_ABSENT_MS;
            uint8_t no_r_us  = (timer_get_ms() - r_wall_ms) >= WALL_ABSENT_MS;
            uint8_t no_l_ir  = (ir_left_wall()  == 0u);
            uint8_t no_r_ir  = (ir_right_wall() == 0u);
            if ((state == S_STRAIGHT || state == S_APPROACH)
                && (no_l_us || no_r_us) && no_l_ir && no_r_ir) {
                Motor_Stop();
                send_turn_report(turn_count, turn_seq,
                                 timer_get_ms() - maze_start_ms);
                while (1);
            }
        }

        /* Suppress approach re-entry for 500 ms after a turn exits */
        uint32_t settle  = timer_get_ms() - post_turn_start_ms;
        uint8_t  f_thresh = (settle < 500u) ? 12u : (uint8_t)FRONT_SLOW_CM;

        /* ================================================================
         * S_APPROACH entry — triggered by front US crossing threshold.
         * ================================================================ */
        if (state == S_STRAIGHT && df <= f_thresh) {
            state             = S_APPROACH;
            approach_start_ms = timer_get_ms();
            ir_stable_us      = 0u;
            UART_SendString("App F="); print_u16(df); UART_SendString("\r\n");
        }

        /* ================================================================
         * TURN DETECTION (runs only in S_APPROACH)
         *
         * Primary trigger — IR debounce (80 ms stable):
         *   Left  IR open  → turn LEFT  (left  wall disappeared → new corridor left)
         *   Right IR open  → turn RIGHT (right wall disappeared → new corridor right)
         *   Both commit direction via S_CREEP_CENTER before the pivot starts.
         *
         * Fallback — raw side US (fires when front ≤ 18 cm and IR was missed).
         *
         * Safety timeout — 2500 ms in S_APPROACH without a decision.
         * ================================================================ */
        if (state == S_APPROACH) {
            uint32_t app_elapsed = timer_get_ms() - approach_start_ms;
            uint32_t now_us      = timer_get_us();

            /* Update debounce timestamp whenever IR state changes */
            if ((il_now != ir_last_left) || (ir_now != ir_last_right)) {
                ir_last_left  = il_now;
                ir_last_right = ir_now;
                ir_stable_us  = 0u;
            } else if (ir_stable_us == 0u) {
                ir_stable_us  = now_us;
            }

            /* --- Primary: IR stable for ≥ 80 ms ---------------------------------------- */
            if (ir_stable_us != 0u && (now_us - ir_stable_us) >= 80000UL) {
                if (il_now == 0u && ir_now != 0u) {          /* left  open only → LEFT  turn */
                    if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'L';
                    last_turn_right = 0u;
                    state           = S_CREEP_CENTER;
                    turn_start_ms   = timer_get_ms();
                    UART_SendString("L->crp\r\n");
                    continue;
                }
                if (ir_now == 0u && il_now != 0u) {          /* right open only → RIGHT turn */
                    if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'R';
                    last_turn_right = 1u;
                    state           = S_CREEP_CENTER;
                    turn_start_ms   = timer_get_ms();
                    UART_SendString("R->crp\r\n");
                    continue;
                }
                if (il_now == 0u && ir_now == 0u) {          /* both open → use side US to pick */
                    uint8_t go_right = (dr >= dl);           /* farther US side = more open space */
                    if (go_right) {
                        if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'R';
                        last_turn_right = 1u;
                        UART_SendString("R(US-both)->crp\r\n");
                    } else {
                        if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'L';
                        last_turn_right = 0u;
                        UART_SendString("L(US-both)->crp\r\n");
                    }
                    state         = S_CREEP_CENTER;
                    turn_start_ms = timer_get_ms();
                    continue;
                }
            }

            /* --- Fallback: raw side US when front is very close -------------------------*/
            /* IR must agree with the US reading to avoid false turns caused by a
             * stale last_good_l/r (reset to 20 cm in S_POST_TURN, which can read
             * > 25 cm after a wide turn and trigger a phantom open-side decision). */
            if (df <= 18u) {
                uint8_t l_open = (dl_raw == 0u || dl_raw > 25u);
                uint8_t r_open = (dr_raw == 0u || dr_raw > 25u);
                if (l_open && !r_open && il_now == 0u) {
                    if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'L';
                    last_turn_right = 0u;
                    state           = S_CREEP_CENTER;
                    turn_start_ms   = timer_get_ms();
                    UART_SendString("L(US)->crp\r\n");
                    continue;
                }
                if (r_open && !l_open && ir_now == 0u) {
                    if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'R';
                    last_turn_right = 1u;
                    state           = S_CREEP_CENTER;
                    turn_start_ms   = timer_get_ms();
                    UART_SendString("R(US)->crp\r\n");
                    continue;
                }
            }

            /* --- Safety timeout --------------------------------------------------------- */
            if (app_elapsed > 2500u) {
                uint8_t il_c = ir_left_wall();
                uint8_t ir_c = ir_right_wall();
                if (il_c == 0u && ir_c == 0u) { /* both open → side US decides */
                    uint8_t go_right = (dr >= dl);
                    if (go_right) {
                        if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'R';
                        last_turn_right = 1u;
                        UART_SendString("TO R(US)\r\n");
                    } else {
                        if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'L';
                        last_turn_right = 0u;
                        UART_SendString("TO L(US)\r\n");
                    }
                    state         = S_CREEP_CENTER;
                    turn_start_ms = timer_get_ms();
                    ir_stable_us  = 0u;
                    continue;
                } else if (ir_c == 0u) {         /* right open only → RIGHT */
                    if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'R';
                    last_turn_right = 1u;
                    state           = S_CREEP_CENTER;
                    turn_start_ms   = timer_get_ms();
                    ir_stable_us    = 0u;
                    UART_SendString("TO R\r\n");
                    continue;
                } else if (il_c == 0u) {         /* left open only → LEFT */
                    if (turn_count < MAX_TURNS) turn_seq[turn_count++] = 'L';
                    last_turn_right = 0u;
                    state           = S_CREEP_CENTER;
                    turn_start_ms   = timer_get_ms();
                    ir_stable_us    = 0u;
                    UART_SendString("TO L\r\n");
                    continue;
                } else {
                    Motor_Forward(50u, 50u);
                    state             = S_APPROACH;
                    approach_start_ms = timer_get_ms();
                    UART_SendString("TO BK\r\n");
                    continue;
                }
            }
        }

        /* ================================================================
         * WALL PID + ENCODER STRAIGHT-LINE PID
         *
         * wall_pid: steers the robot to the centre of the corridor.
         * enc_pid:  corrects left/right motor speed imbalance when the
         *           wall error is already within deadband (straight runs).
         * ================================================================ */
        float raw_err = 0.0f;
        float w_corr  = 0.0f;
        float e_corr  = 0.0f;
        if (sensor_valid) {
            w_corr = wall_pid((float)dl, (float)dr, dt, &raw_err);
            float abs_err = (raw_err < 0.0f) ? -raw_err : raw_err;
            if (abs_err <= WALL_DEADBAND)
                e_corr = enc_pid(el, er, dt);
            else
                enc_prev_error = 0.0f;  /* reset enc PID when wall is correcting */
        }

        /* ================================================================
         * DRIVE — front-distance speed ramp + approach speed cap.
         *
         * S_STRAIGHT: linear ramp from BASE_SPEED (far) to SPEED_MIN_FWD (close).
         * S_APPROACH: ramp output capped at SPEED_SLOW for safe turn entry.
         * ================================================================ */
        uint8_t base;
        if (df >= (uint16_t)FRONT_FULL_CM) {
            base = (uint8_t)BASE_SPEED;
        } else if (df <= (uint16_t)FRONT_STOP_CM) {
            base = (uint8_t)SPEED_MIN_FWD;
        } else {
            uint16_t gap   = df - (uint16_t)FRONT_STOP_CM;
            uint16_t range = (uint16_t)(FRONT_FULL_CM - FRONT_STOP_CM);
            base = (uint8_t)((uint16_t)SPEED_MIN_FWD
                           + (uint32_t)gap * (BASE_SPEED - SPEED_MIN_FWD) / range);
        }
        if (state == S_APPROACH && base > (uint8_t)SPEED_SLOW)
            base = (uint8_t)SPEED_SLOW;

        float   total = w_corr + e_corr;
        int16_t ls    = clamp16((int16_t)base + (int16_t)total,
                                (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        int16_t rs    = clamp16((int16_t)(base - RIGHT_REDUCE) - (int16_t)total,
                                (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        Motor_Forward((uint8_t)ls, (uint8_t)rs);

        /* ================================================================
         * DEBUG — throttled to every 10th loop (~100 ms at 10 ms/loop)
         * Fields: L R F — US distances (cm)
         *         e     — raw wall error (cm)
         *         IL IR  — IR left / right (1=wall, 0=open)
         *         S     — FSM state index
         *         T     — turn count so far
         * ================================================================ */
        if ((++print_cnt % 10u) == 0u) {
            UART_SendString("L=");   print_i16((int16_t)dl);
            UART_SendString(" R=");  print_i16((int16_t)dr);
            UART_SendString(" F=");  print_i16((int16_t)df);
            UART_SendString(" e=");  print_i16((int16_t)raw_err);
            UART_SendString(" IL="); UART_SendChar(il_now ? '1' : '0');
            UART_SendString(" IR="); UART_SendChar(ir_now ? '1' : '0');
            UART_SendString(" S=");  print_i16((int16_t)state);
            UART_SendString(" T=");  print_u16(turn_count);
            UART_SendString("\r\n");
        }
    }

    return 0;
}
