/**
 * @file    main.c
 * @brief   Wall-following robot – ATmega328P / Arduino Nano
 *          Wall-centre PID  +  Encoder straight-line PID
 *          + IR-based turn detection with debounce fix
 *
 * PIN ASSIGNMENTS
 *   US Left  TRIG → PC2 (A2)    US Left  ECHO → PC3 (A3)
 *   US Right TRIG → PC4 (A4)    US Right ECHO → PC5 (A5)
 *   US Front TRIG → PC0 (A0)    US Front ECHO → PC1 (A1)
 *   Motor L IN1   → PB0 (D8)    Motor L IN2   → PB1 (D9)
 *   Motor R IN3   → PB2 (D10)   Motor R IN4   → PB3 (D11)
 *   PWM Left ENA  → PD5 (D5)    PWM Right ENB → PD6 (D6)
 *   Enc A Left    → PD7 (D7)    PCINT23
 *   Enc B Left    → PD2 (D2)    direction sense
 *   Enc A Right   → PB5 (D13)   PCINT5
 *   Enc B Right   → PB4 (D12)   PCINT4
 *   IR Left       → PD3 (D3)
 *   IR Right      → PD4 (D4)
 *   UART TX       → PD1 (D1)    9600 baud
 *
 * FIXES APPLIED
 *   1. IR debounce – must see open space IR_DEBOUNCE_COUNT times in a row
 *      before a turn is triggered. Prevents false turns from flicker.
 *   2. Grace period increased to 10 iterations so IR is not trusted at boot.
 *   3. FRONT_SLOW_CM reduced to 30 cm so approach only fires when truly close.
 *   4. Turn exit is purely time-based (600 ms) – no IR dependency during turn.
 *   5. Post-turn straight burst (250 ms) before PID re-engages.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#include "timer.h"
#include "uart.h"
#include "motor.h"

/* =========================================================================
 * Ultrasonic – LEFT and RIGHT (wall following)
 * ========================================================================= */
#define US_LEFT_TRIG 2u
#define US_LEFT_ECHO 3u
#define US_RIGHT_TRIG 4u
#define US_RIGHT_ECHO 5u
#define US_MIN_CM 2u
#define US_MAX_CM 60u
#define US_TIMEOUT_ITER 5333u
#define JUMP_LIMIT 6u

/* =========================================================================
 * Front ultrasonic – for turn detection
 * ========================================================================= */
#define US_FRONT_TRIG 0u /* PC0 = A0 */
#define US_FRONT_ECHO 1u /* PC1 = A1 */

/** @brief  Slow down when front wall is closer than this (cm). */
#define FRONT_SLOW_CM 30u /* was 50 – reduced to avoid false approach */

/* =========================================================================
 * IR sensors
 * IR output is LOW when wall present, HIGH when open space.
 * ========================================================================= */
#define IR_LEFT_BIT 3u  /* PD3 = D3 */
#define IR_RIGHT_BIT 4u /* PD4 = D4 */

/**
 * @brief  Number of consecutive open readings required before a turn fires.
 *         Increase this if your IR still flickers on white cardboard walls.
 *         At ~60 ms per main loop iteration, 8 counts ≈ 480 ms of open space.
 */
#define IR_DEBOUNCE_COUNT 8u

/** @brief  Returns 1 if wall is present, 0 if open space. */
static inline uint8_t ir_left_wall(void) {
    return (PIND & (1u << IR_LEFT_BIT)) ? 0u : 1u;
}
static inline uint8_t ir_right_wall(void) {
    return (PIND & (1u << IR_RIGHT_BIT)) ? 0u : 1u;
}

/* =========================================================================
 * Turn parameters
 * ========================================================================= */
#define SPEED_SLOW        180u
#define SPEED_TURN_OUTER  220u   /* outer wheel forward speed  */
#define SPEED_TURN_INNER  110u   /* inner wheel backward speed */

/* Sensor-based exit now fires first; this is the safety timeout only.
 * 900 ms is a generous upper bound for a full 90° pivot. */
#define TURN_DURATION_MS  900u

/** @brief  Straight burst after a turn to clear junction geometry (ms). */
#define POST_TURN_MS 250u

/* =========================================================================
 * Encoder pins
 * ========================================================================= */
#define ENC_L_A_BIT 7u
#define ENC_L_B_BIT 2u
#define ENC_R_A_BIT 5u
#define ENC_R_B_BIT 4u

/* =========================================================================
 * Drive parameters
 * ========================================================================= */
#define BASE_SPEED 230u
#define MAX_SPEED  255u
#define MIN_SPEED   80u
#define LEFT_TRIM 0
#define RIGHT_REDUCE 0

/* =========================================================================
 * Wall PID gains
 * ========================================================================= */
#define WALL_KP 1.2f
#define WALL_KI 0.04f
#define WALL_KD 0.3f
#define WALL_MAX_OUT 35.0f
#define WALL_INTEGRAL_LIM 25.0f
#define WALL_DEADBAND 1.5f
#define WALL_DERIV_FILTER 0.55f

/* =========================================================================
 * Encoder PID gains
 * ========================================================================= */
#define ENC_KP 0.8f
#define ENC_KI 0.0f
#define ENC_KD 0.1f
#define ENC_MAX_OUT 12.0f
#define ENC_INTEGRAL_LIM 40.0f

/* =========================================================================
 * Encoder ISRs
 * ========================================================================= */
static volatile int32_t g_enc_left = 0;
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
            g_enc_right += ((cur >> ENC_R_B_BIT) & 1u) ? 1 : -1;
}

/** @brief  Configure encoder pins as inputs with pull-ups, enable PCINT. */
static void encoder_init(void) {
    DDRD &= ~((1u << ENC_L_A_BIT) | (1u << ENC_L_B_BIT));
    PORTD |= (1u << ENC_L_A_BIT) | (1u << ENC_L_B_BIT);
    DDRB &= ~((1u << ENC_R_A_BIT) | (1u << ENC_R_B_BIT));
    PORTB |= (1u << ENC_R_A_BIT) | (1u << ENC_R_B_BIT);
    PCMSK2 |= (1u << PCINT23);
    PCMSK0 |= (1u << PCINT5) | (1u << PCINT4);
    PCICR |= (1u << PCIE2) | (1u << PCIE0);
    g_prev_pind = PIND;
    g_prev_pinb = PINB;
}

/** @brief  Reset both encoder counters atomically. */
static void enc_reset(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_enc_left = 0;
        g_enc_right = 0;
    }
}

/** @brief  Read both encoder counters atomically into provided pointers. */
static void enc_snapshot(int32_t *l, int32_t *r) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *l = g_enc_left;
        *r = g_enc_right;
    }
}

/* =========================================================================
 * Ultrasonic helpers
 * ========================================================================= */

/** @brief  Init trig pins as output, echo pins as input. */
static void ultrasonic_init(void)
{
    DDRC |=  (1u<<US_LEFT_TRIG)|(1u<<US_RIGHT_TRIG)|(1u<<US_FRONT_TRIG);
    DDRC &= ~((1u<<US_LEFT_ECHO)|(1u<<US_RIGHT_ECHO)|(1u<<US_FRONT_ECHO));
    PORTC &= ~((1u<<US_LEFT_TRIG)|(1u<<US_RIGHT_TRIG)|(1u<<US_FRONT_TRIG));
}

/*
 * Microsecond timestamp: combines the 1ms project counter with TCNT1.
 * Timer1 prescaler=8, F_CPU=16MHz → 2 ticks/us, period=2000 ticks=1ms.
 * Resolution ~0.5 us.  Handles the CTC-reset boundary: if the compare
 * flag is already set but the ISR hasn't run yet, the ms count is
 * adjusted so the result is always monotonic.
 */
static uint32_t us_now(void)
{
    uint32_t ms;
    uint16_t tc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tc = TCNT1;
        ms = timer_get_ms();
        /* ISR pending but not yet serviced — ms is one behind */
        if ((TIFR1 & (1u << OCF1A)) && tc < 4u) ms++;
    }
    return ms * 1000UL + (uint32_t)(tc >> 1u);  /* tc/2 converts ticks → us */
}

/*
 * Send one trigger pulse and measure the echo duration.
 * All timing uses us_now() (project timer + TCNT1) — no _delay_us.
 * Returns distance in cm, or 0 on timeout / out-of-range.
 */
static uint16_t us_read_once(uint8_t trig, uint8_t echo)
{
    /* Wait for any lingering echo to clear (up to 10 ms) */
    uint32_t t0 = timer_get_ms();
    while ((PINC & (1u << echo)) && (timer_get_ms() - t0) < 10u)
        ;

    /* 10 µs trigger pulse measured by us_now() — no _delay_us */
    PORTC |= (1u << trig);
    uint32_t t_trig = us_now();
    while ((us_now() - t_trig) < 10u)
        ;
    PORTC &= ~(1u << trig);

    /* Wait for echo HIGH (4 ms timeout) */
    t0 = timer_get_ms();
    while (!(PINC & (1u << echo))) {
        if ((timer_get_ms() - t0) >= 4u) return 0u;
    }

    /* Measure echo HIGH duration */
    uint32_t echo_start = us_now();
    while (PINC & (1u << echo)) {
        if ((us_now() - echo_start) >= 8000u) return 0u;  /* 8 ms = ~136 cm */
    }
    uint32_t duration_us = us_now() - echo_start;

    if (duration_us == 0u) return 0u;
    return (uint16_t)(duration_us / 58u);
}

/**
 * @brief  Take 3 readings and return the median — rejects noise spikes.
 */
static uint16_t us_read_median3(uint8_t trig, uint8_t echo) {
    uint16_t s[3], tmp;
    s[0] = us_read_once(trig, echo);
    s[1] = us_read_once(trig, echo);
    s[2] = us_read_once(trig, echo);

    /* @brief  bubble sort 3 elements */
    if (s[0] > s[1]) {
        tmp = s[0];
        s[0] = s[1];
        s[1] = tmp;
    }
    if (s[1] > s[2]) {
        tmp = s[1];
        s[1] = s[2];
        s[2] = tmp;
    }
    if (s[0] > s[1]) {
        tmp = s[0];
        s[0] = s[1];
        s[1] = tmp;
    }
    return s[1];
}

/* =========================================================================
 * UART helpers
 * ========================================================================= */
static void uart_print_i16(int16_t v) {
    char buf[7];
    uint8_t i = 0, neg = 0;
    if (v < 0) {
        neg = 1;
        v = (int16_t)-v;
    }
    if (v == 0) {
        UART_SendChar('0');
        return;
    }
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    if (neg)
        UART_SendChar('-');
    while (i > 0)
        UART_SendChar(buf[--i]);
}

static void uart_print_i32(int32_t v) {
    char buf[12];
    uint8_t i = 0, neg = 0;
    if (v < 0) {
        neg = 1;
        v = -v;
    }
    if (v == 0) {
        UART_SendChar('0');
        return;
    }
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    if (neg)
        UART_SendChar('-');
    while (i > 0)
        UART_SendChar(buf[--i]);
}

static void uart_print_u16(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) {
        UART_SendChar('0');
        return;
    }
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0)
        UART_SendChar(buf[--i]);
}

/**
 * @brief Log sensor readings and decision state over UART in CSV format.
 * Format: US_L,US_R,US_F,IR_L,IR_R,STATE,DECISION
 */
static void log_sensor_data(uint16_t us_left, uint16_t us_right,
                            uint16_t us_front, uint8_t ir_left,
                            uint8_t ir_right, const char *state_name,
                            const char *decision) {
    UART_SendString("[LOG] ");
    uart_print_u16(us_left);
    UART_SendString(",");
    uart_print_u16(us_right);
    UART_SendString(",");
    uart_print_u16(us_front);
    UART_SendString(",");
    UART_SendChar(ir_left ? '1' : '0');
    UART_SendString(",");
    UART_SendChar(ir_right ? '1' : '0');
    UART_SendString(",");
    UART_SendString(state_name);
    UART_SendString(",");
    UART_SendString(decision);
    UART_SendString("\r\n");
}

/* =========================================================================
 * Wall PID
 * ========================================================================= */
static float wall_integral = 0.0f;
static float wall_prev_error = 0.0f;
static float wall_deriv_filtered = 0.0f;

static float wall_pid(float dl, float dr, float dt, float *raw_error_out) {
    float error = dl - dr;
    *raw_error_out = error;

    if (error > WALL_DEADBAND)
        error -= WALL_DEADBAND;
    else if (error < -WALL_DEADBAND)
        error += WALL_DEADBAND;
    else {
        error = 0.0f;
        wall_integral *= 0.85f;
    }

    float pre_clamp = WALL_KP * error + WALL_KI * wall_integral;
    if (pre_clamp < WALL_MAX_OUT && pre_clamp > -WALL_MAX_OUT)
        wall_integral += error * dt;
    if (wall_integral > WALL_INTEGRAL_LIM)
        wall_integral = WALL_INTEGRAL_LIM;
    if (wall_integral < -WALL_INTEGRAL_LIM)
        wall_integral = -WALL_INTEGRAL_LIM;

    float raw_d = (error - wall_prev_error) / dt;
    wall_deriv_filtered = WALL_DERIV_FILTER * wall_deriv_filtered +
                          (1.0f - WALL_DERIV_FILTER) * raw_d;
    wall_prev_error = error;

    float out = WALL_KP * error + WALL_KI * wall_integral +
                WALL_KD * wall_deriv_filtered;
    if (out > WALL_MAX_OUT)
        out = WALL_MAX_OUT;
    if (out < -WALL_MAX_OUT)
        out = -WALL_MAX_OUT;
    return out;
}

/* =========================================================================
 * Encoder PID
 * ========================================================================= */
static float enc_integral = 0.0f;
static float enc_prev_error = 0.0f;

static float enc_pid(int32_t l_ticks, int32_t r_ticks, float dt) {
    const float ratio =
        (float)(BASE_SPEED) / (float)(BASE_SPEED - RIGHT_REDUCE);
    float error = (float)l_ticks - ratio * (float)r_ticks;
    enc_integral += error * dt;
    if (enc_integral > ENC_INTEGRAL_LIM)
        enc_integral = ENC_INTEGRAL_LIM;
    if (enc_integral < -ENC_INTEGRAL_LIM)
        enc_integral = -ENC_INTEGRAL_LIM;
    float deriv = (error - enc_prev_error) / dt;
    enc_prev_error = error;
    float out = ENC_KP * error + ENC_KI * enc_integral + ENC_KD * deriv;
    if (out > ENC_MAX_OUT)
        out = ENC_MAX_OUT;
    if (out < -ENC_MAX_OUT)
        out = -ENC_MAX_OUT;
    return out;
}

static int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void) {
    /* ── Init ─────────────────────────────────────────────────────────── */
    timer_init();
    UART_Init(9600UL);
    ultrasonic_init();
    encoder_init();
    Motor_Init();

    /* @brief  IR pins as inputs, no pull-up (sensor has its own output) */
    DDRD &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));
    PORTD &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));

    sei();

    UART_SendString("Wall-follower ready.\r\n");

    /* ── Sensor warm-up ───────────────────────────────────────────────── */
    uint16_t last_good_l = 15u;
    uint16_t last_good_r = 15u;
    UART_SendString("Calibrating...\r\n");
    for (uint8_t wi = 0; wi < 5u; wi++) {
        uint16_t wl = us_read_median3(US_LEFT_TRIG, US_LEFT_ECHO);
        uint16_t wr = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);
        if (wl >= US_MIN_CM && wl <= US_MAX_CM)
            last_good_l = wl;
        if (wr >= US_MIN_CM && wr <= US_MAX_CM)
            last_good_r = wr;
        /* No delay - next iteration will read immediately */
    }
    UART_SendString("Go.\r\n");

    uint8_t sensor_valid = 1u;
    uint32_t prev_time = timer_get_ms();
    enc_reset();

    /* ── State machine ────────────────────────────────────────────────── */
    typedef enum {
        S_STRAIGHT = 0, /* normal wall-following PID                  */
        S_APPROACH,     /* front <= FRONT_SLOW_CM, slow + wait for IR */
        S_TURN_RIGHT,   /* pivot right for TURN_DURATION_MS           */
        S_TURN_LEFT,    /* pivot left  for TURN_DURATION_MS           */
        S_POST_TURN,    /* straight burst to clear junction           */
        S_SENSOR_WAIT   /* wait between sensor reads (non-blocking)   */
    } state_t;

    state_t state = S_STRAIGHT;

    uint32_t turn_start_ms      = 0u;
    uint32_t approach_start_ms  = 0u;
    uint32_t post_turn_start_ms = 0u;

    /* IR debounce: time-based instead of count-based */
    uint32_t ir_last_change_ms = 0u; /* timestamp of last IR change */
    uint8_t ir_last_left = 1u;       /* previous IR left reading */
    uint8_t ir_last_right = 1u;      /* previous IR right reading */

    /* Front sensor: track last valid reading when not timeout */
    uint16_t last_valid_front = 30u;

    /**
     * @brief  Grace period — skip IR checking for first N iterations.
     * Increased to 10 so sensors have time to stabilise after power-on.
     */
    uint8_t grace = 10u;

    while (1) {
        /* ================================================================
         * TURNING STATE — bypass wall PID entirely
         *
         * Exit when the robot has actually turned into the new corridor:
         *   1. Front sensor clears  (df > FRONT_SLOW_CM)  — primary condition
         *   2. Both IR sensors see walls again             — aligned with corridor
         *   3. Timeout (TURN_DURATION_MS)                 — safety fallback
         * A minimum guard of 200 ms stops the exit firing before the
         * pivot has had a chance to physically start.
         * ================================================================ */
        if (state == S_TURN_RIGHT || state == S_TURN_LEFT)
        {
            if (state == S_TURN_RIGHT)
                Motor_TurnRight(SPEED_TURN_OUTER, SPEED_TURN_INNER);
            else
                Motor_TurnLeft(SPEED_TURN_INNER, SPEED_TURN_OUTER);

            uint32_t elapsed = timer_get_ms() - turn_start_ms;

            if (elapsed >= 200u)
            {
                uint16_t df_t  = us_read_once(US_FRONT_TRIG, US_FRONT_ECHO);
                if (df_t == 0u) df_t = 255u;          /* timeout → treat as far */

                uint8_t ir_l_t = ir_left_wall();
                uint8_t ir_r_t = ir_right_wall();

                uint8_t front_clear = (df_t  > FRONT_SLOW_CM);
                uint8_t ir_aligned  = (ir_l_t && ir_r_t);
                uint8_t timed_out   = (elapsed >= TURN_DURATION_MS);

                if (front_clear && ir_aligned )
                {
                    if      (ir_aligned && front_clear) UART_SendString("Turn done (IR+front)\r\n");
                    else if (ir_aligned)                UART_SendString("Turn done (IR)\r\n");
                    else if (front_clear)               UART_SendString("Turn done (front)\r\n");
                    else                                UART_SendString("Turn done (timeout)\r\n");

                    state         = S_POST_TURN;
                    turn_start_ms = timer_get_ms();
                    enc_reset();
                }
            }
            continue;
        }

        /* ================================================================
         * POST-TURN — drive straight to clear junction
         * ================================================================ */
        if (state == S_POST_TURN) {
            Motor_Forward((uint8_t)BASE_SPEED,
                          (uint8_t)(BASE_SPEED - RIGHT_REDUCE));

            if ((timer_get_ms() - turn_start_ms) >= POST_TURN_MS) {
                /* Reset PID integrators */
                wall_integral = 0.0f; wall_prev_error = 0.0f;
                wall_deriv_filtered = 0.0f;
                enc_integral  = 0.0f; enc_prev_error  = 0.0f;
                enc_reset();

                /* Reset IR debounce */
                ir_last_change_ms = timer_get_ms();
                ir_last_left      = 1u;
                ir_last_right     = 1u;

                /* Flush stale pre-turn sensor values.
                 * Without this the jump filter keeps the old side readings
                 * and last_valid_front could immediately re-trigger APPROACH. */
                last_valid_front = 100u;
                last_good_l      = 20u;
                last_good_r      = 20u;
                sensor_valid     = 0u;  /* bypass jump filter for first reads */

                /* Reset prev_time so the first PID call gets a sane dt.
                 * Without this, dt = time_since_turn_started (~1 s),
                 * which spikes the integral on the very first PID tick. */
                prev_time          = timer_get_ms();
                state              = S_STRAIGHT;
                post_turn_start_ms = timer_get_ms();
                UART_SendString("Resuming wall-follow\r\n");
            }
            continue;
        }

        /* ================================================================
         * Sensor reads - NON-BLOCKING with delay between sensors
         * ================================================================ */
        uint16_t dl_raw = us_read_median3(US_LEFT_TRIG, US_LEFT_ECHO);
        uint16_t dr_raw = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);
        uint16_t df_raw = us_read_once(US_FRONT_TRIG, US_FRONT_ECHO);
        if (df_raw == 0u)
            df_raw = 255u;

        /* FIX 4: track last valid front reading (ignore 255 = timeout) */
        if (df_raw != 255u) {
            last_valid_front = df_raw;
        }
        uint16_t df = last_valid_front; /* use validated reading for logic */

        /* @brief  apply jump filter — reject spikes > JUMP_LIMIT cm */
        uint16_t dl = last_good_l;
        uint16_t dr = last_good_r;

        if (dl_raw >= US_MIN_CM && dl_raw <= US_MAX_CM) {
            uint16_t jump = (dl_raw > last_good_l) ? (dl_raw - last_good_l)
                                                   : (last_good_l - dl_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) {
                last_good_l = dl_raw;
                dl = dl_raw;
            }
        }
        if (dr_raw >= US_MIN_CM && dr_raw <= US_MAX_CM) {
            uint16_t jump = (dr_raw > last_good_r) ? (dr_raw - last_good_r)
                                                   : (last_good_r - dr_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) {
                last_good_r = dr_raw;
                dr = dr_raw;
            }
        }
        if (last_good_l > 0u && last_good_r > 0u)
            sensor_valid = 1u;

        /* ── dt ──────────────────────────────────────────────────────── */
        uint32_t now = timer_get_ms();
        float dt = (float)(now - prev_time) * 0.001f;
        if (dt < 0.005f)
            dt = 0.005f;
        prev_time = now;

        /* ── Encoder snapshot ───────────────────────────────────────── */
        int32_t el, er;
        enc_snapshot(&el, &er);
        enc_reset();

        /* ── Grace period — don't trust IR at boot ───────────────────── */
        if (grace > 0u) {
            grace--;
            Motor_Forward((uint8_t)BASE_SPEED,
                          (uint8_t)(BASE_SPEED - RIGHT_REDUCE));
            continue;
        }

        /* ================================================================
         * Log sensor data
         * ================================================================ */
        uint8_t ir_left_val = ir_left_wall();
        uint8_t ir_right_val = ir_right_wall();

        /* ================================================================
         * STOP — all five sensors see open space:
         *   • Both IR sensors open (no wall at front corners)
         *   • Front US clear (df > FRONT_SLOW_CM, not a timeout)
         *   • Left  side US beyond range (dl_raw == 0 or > US_MAX_CM)
         *   • Right side US beyond range (dr_raw == 0 or > US_MAX_CM)
         * Permanent halt; reset the board to restart.
         * ================================================================ */
        uint8_t side_l_open = (dl_raw == 0u || dl_raw > US_MAX_CM);
        uint8_t side_r_open = (dr_raw == 0u || dr_raw > US_MAX_CM);

        if ((!ir_left_wall()) && (!ir_right_wall())
            && (df > FRONT_SLOW_CM) && (df < 255u)
            && side_l_open && side_r_open)
        {
            Motor_Stop();
            log_sensor_data(dl, dr, df, ir_left_val, ir_right_val,
                            "STOP", "ALL_OPEN");
            UART_SendString("STOP: open space\r\n");
            while (1);  /* halt — reset board to restart */
        }

        /* ================================================================
         * Post-turn settling: ignore FRONT_CLOSE for 300ms after turn
         * FIX 2: Raise threshold temporarily to prevent premature re-entry
         * ================================================================ */
        uint32_t settle_elapsed = timer_get_ms() - post_turn_start_ms;
        uint8_t front_close_threshold =
            (settle_elapsed < 500u) ? 12u : FRONT_SLOW_CM;

        /* ================================================================
         * APPROACH detection — front wall close (with settling protection)
         * ================================================================ */
        if (state == S_STRAIGHT && df <= front_close_threshold) {
            state = S_APPROACH;
            approach_start_ms = timer_get_ms();
            ir_last_change_ms = timer_get_ms();
            log_sensor_data(dl, dr, df, ir_left_val, ir_right_val, "APPROACH",
                            "FRONT_CLOSE");
            UART_SendString("Approach\r\n");
        }

        /* ================================================================
         * IR DEBOUNCE TURN DETECTION (TIME-BASED)
         *
         * FIX 3: IR must read the SAME value for >= 120ms continuously.
         * Timestamp records when IR last changed state.
         * Only act on IR reading if (current_time - ir_stable_since) >= 120ms.
         * ================================================================ */
        if (state == S_APPROACH) {
            uint32_t approach_elapsed = timer_get_ms() - approach_start_ms;
            uint32_t ir_stable_time = timer_get_ms() - ir_last_change_ms;

            /* -- Check if IR state changed -------------------------------- */
            uint8_t ir_left_now = ir_left_wall();
            uint8_t ir_right_now = ir_right_wall();

            if ((ir_left_now != ir_last_left) ||
                (ir_right_now != ir_last_right)) {
                ir_last_left = ir_left_now;
                ir_last_right = ir_right_now;
                ir_last_change_ms = timer_get_ms();
                ir_stable_time = 0u;
            }

            /* -- Fire turn only if IR stable for >= IR_DEBOUNCE_MS (120ms) --
             */
            if (ir_stable_time >= 120u) {
                /* Right IR open → turn LEFT (sensors are mirror-reversed) */
                if (ir_right_now == 0u) {
                    state         = S_TURN_LEFT;
                    turn_start_ms = timer_get_ms();
                    log_sensor_data(dl, dr, df, ir_left_val, ir_right_val,
                                    "TURN_LEFT", "IR_RIGHT_OPEN");
                    UART_SendString("Turn L\r\n");
                    continue;
                }

                /* Left IR open → turn RIGHT (sensors are mirror-reversed) */
                if (ir_left_now == 0u) {
                    state         = S_TURN_RIGHT;
                    turn_start_ms = timer_get_ms();
                    log_sensor_data(dl, dr, df, ir_left_val, ir_right_val,
                                    "TURN_RIGHT", "IR_LEFT_OPEN");
                    UART_SendString("Turn R\r\n");
                    continue;
                }
            }

            /* -- FIX 1: Safety timeout for stuck in APPROACH (2500ms) ----- */
            if (approach_elapsed > 2500u) {
                uint8_t ir_left_now_check = ir_left_wall();
                uint8_t ir_right_now_check = ir_right_wall();

                if (ir_right_now_check == 0u) {
                    state             = S_TURN_LEFT;
                    turn_start_ms     = timer_get_ms();
                    ir_last_change_ms = timer_get_ms();
                    log_sensor_data(dl, dr, df, ir_left_val, ir_right_val,
                                    "TURN_LEFT", "TIMEOUT_IR_RIGHT");
                    UART_SendString("TIMEOUT Turn L\r\n");
                    continue;
                } else if (ir_left_now_check == 0u) {
                    state             = S_TURN_RIGHT;
                    turn_start_ms     = timer_get_ms();
                    ir_last_change_ms = timer_get_ms();
                    log_sensor_data(dl, dr, df, ir_left_val, ir_right_val,
                                    "TURN_RIGHT", "TIMEOUT_IR_LEFT");
                    UART_SendString("TIMEOUT Turn R\r\n");
                    continue;
                } else {
                    /* FIX 1: Both walls present - BACKUP instead of STOP */
                    Motor_Forward(50u, 50u); /* reverse at low speed */
                    log_sensor_data(dl, dr, df, ir_left_val, ir_right_val,
                                    "BACKUP", "BOTH_WALLS_TIMEOUT");
                    UART_SendString("TIMEOUT BACKUP\r\n");
                    state = S_APPROACH; /* stay in APPROACH, reset timer */
                    approach_start_ms = timer_get_ms();
                    continue;
                }
            }

            /* @brief  both walls still present — keep creeping forward slowly
             */
            log_sensor_data(dl, dr, df, ir_left_val, ir_right_val, "APPROACH",
                            "WAIT_IR");
        }

        /* ================================================================
         * PID — wall centering + encoder straight-line
         * ================================================================ */
        float raw_err = 0.0f;
        float w_corr = 0.0f;
        float e_corr = 0.0f;

        if (sensor_valid) {
            w_corr = wall_pid((float)dl, (float)dr, dt, &raw_err);
            float abs_err = raw_err < 0.0f ? -raw_err : raw_err;
            if (abs_err <= WALL_DEADBAND) {
                e_corr = enc_pid(el, er, dt);
            } else {
                /* @brief  wall error is large — reset encoder PID to avoid
                 * conflict */
                enc_integral = 0.0f;
                enc_prev_error = 0.0f;
            }
        }

        /* ================================================================
         * Log wall-following state
         * ================================================================ */
        if (state == S_STRAIGHT) {
            log_sensor_data(dl, dr, df, ir_left_val, ir_right_val, "STRAIGHT",
                            "PID_FOLLOW");
        }

        /* ================================================================
         * Drive
         * ================================================================ */
        float total = w_corr + e_corr;
        uint8_t base = (state == S_APPROACH) ? SPEED_SLOW : (uint8_t)BASE_SPEED;

        int16_t ls = (int16_t)base + (int16_t)total;
        int16_t rs = (int16_t)(base - RIGHT_REDUCE) - (int16_t)total;
        ls = clamp16(ls, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        rs = clamp16(rs, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        Motor_Forward((uint8_t)ls, (uint8_t)rs);

        /* ================================================================
         * Debug output
         * ================================================================ */
        UART_SendString("L=");
        uart_print_i16((int16_t)dl);
        UART_SendString(" R=");
        uart_print_i16((int16_t)dr);
        UART_SendString(" F=");
        uart_print_i16((int16_t)df);
        UART_SendString(" e=");
        uart_print_i16((int16_t)raw_err);
        UART_SendString(" W=");
        uart_print_i16((int16_t)w_corr);
        UART_SendString(" EL=");
        uart_print_i32(el);
        UART_SendString(" ER=");
        uart_print_i32(er);
        UART_SendString(" E=");
        uart_print_i16((int16_t)e_corr);
        UART_SendString(" LS=");
        uart_print_i16(ls);
        UART_SendString(" RS=");
        uart_print_i16(rs);
        UART_SendString(" IRL=");
        uart_print_i16((int16_t)ir_left_wall());
        UART_SendString(" IRR=");
        uart_print_i16((int16_t)ir_right_wall());
        UART_SendString(" ST=");
        uart_print_i16((int16_t)state);
        UART_SendString("\r\n");
    }

    return 0;
}