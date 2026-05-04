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
#include <util/delay.h>

#include "../drivers/timer/timer.h"
#include "../drivers/uart/uart.h"
#include "../modules/motor/motor.h"

/* =========================================================================
 * Ultrasonic – LEFT and RIGHT (wall following)
 * ========================================================================= */
#define US_LEFT_TRIG    2u
#define US_LEFT_ECHO    3u
#define US_RIGHT_TRIG   4u
#define US_RIGHT_ECHO   5u
#define US_MIN_CM       2u
#define US_MAX_CM       60u
#define US_TIMEOUT_ITER 5333u
#define JUMP_LIMIT      6u

/* =========================================================================
 * Front ultrasonic – for turn detection
 * ========================================================================= */
#define US_FRONT_TRIG   0u   /* PC0 = A0 */
#define US_FRONT_ECHO   1u   /* PC1 = A1 */

/** @brief  Slow down when front wall is closer than this (cm). */
#define FRONT_SLOW_CM   30u  /* was 50 – reduced to avoid false approach */

/* =========================================================================
 * IR sensors
 * IR output is LOW when wall present, HIGH when open space.
 * ========================================================================= */
#define IR_LEFT_BIT     3u   /* PD3 = D3 */
#define IR_RIGHT_BIT    4u   /* PD4 = D4 */

/**
 * @brief  Number of consecutive open readings required before a turn fires.
 *         Increase this if your IR still flickers on white cardboard walls.
 *         At ~60 ms per main loop iteration, 8 counts ≈ 480 ms of open space.
 */
#define IR_DEBOUNCE_COUNT  8u

/** @brief  Returns 1 if wall is present, 0 if open space. */
static inline uint8_t ir_left_wall(void)
{
    return (PIND & (1u << IR_LEFT_BIT)) ? 0u : 1u;
}
static inline uint8_t ir_right_wall(void)
{
    return (PIND & (1u << IR_RIGHT_BIT)) ? 0u : 1u;
}

/* =========================================================================
 * Turn parameters
 * ========================================================================= */
#define SPEED_SLOW          90u
#define SPEED_TURN_OUTER   170u
#define SPEED_TURN_INNER    60u

/** @brief  Fixed time for a 90° turn in milliseconds. Tune for your robot. */
#define TURN_DURATION_MS   600u

/** @brief  Straight burst after a turn to clear junction geometry (ms). */
#define POST_TURN_MS       250u

/* =========================================================================
 * Encoder pins
 * ========================================================================= */
#define ENC_L_A_BIT  7u
#define ENC_L_B_BIT  2u
#define ENC_R_A_BIT  5u
#define ENC_R_B_BIT  4u

/* =========================================================================
 * Drive parameters
 * ========================================================================= */
#define BASE_SPEED    150u
#define MAX_SPEED     210u
#define MIN_SPEED      60u
#define LEFT_TRIM        0
#define RIGHT_REDUCE     0

/* =========================================================================
 * Wall PID gains
 * ========================================================================= */
#define WALL_KP             1.2f
#define WALL_KI             0.04f
#define WALL_KD             0.3f
#define WALL_MAX_OUT        35.0f
#define WALL_INTEGRAL_LIM   25.0f
#define WALL_DEADBAND       1.5f
#define WALL_DERIV_FILTER   0.55f

/* =========================================================================
 * Encoder PID gains
 * ========================================================================= */
#define ENC_KP            0.8f
#define ENC_KI            0.0f
#define ENC_KD            0.1f
#define ENC_MAX_OUT       12.0f
#define ENC_INTEGRAL_LIM  40.0f

/* =========================================================================
 * Encoder ISRs
 * ========================================================================= */
static volatile int32_t g_enc_left  = 0;
static volatile int32_t g_enc_right = 0;
static volatile uint8_t g_prev_pind = 0u;
static volatile uint8_t g_prev_pinb = 0u;

ISR(PCINT2_vect)
{
    uint8_t cur = PIND, changed = cur ^ g_prev_pind;
    g_prev_pind = cur;
    if (changed & (1u << ENC_L_A_BIT))
        if (cur & (1u << ENC_L_A_BIT))
            g_enc_left += ((cur >> ENC_L_B_BIT) & 1u) ? 1 : -1;
}

ISR(PCINT0_vect)
{
    uint8_t cur = PINB, changed = cur ^ g_prev_pinb;
    g_prev_pinb = cur;
    if (changed & (1u << ENC_R_A_BIT))
        if (cur & (1u << ENC_R_A_BIT))
            g_enc_right += ((cur >> ENC_R_B_BIT) & 1u) ? 1 : -1;
}

/** @brief  Configure encoder pins as inputs with pull-ups, enable PCINT. */
static void encoder_init(void)
{
    DDRD  &= ~((1u << ENC_L_A_BIT) | (1u << ENC_L_B_BIT));
    PORTD |=  (1u << ENC_L_A_BIT)  | (1u << ENC_L_B_BIT);
    DDRB  &= ~((1u << ENC_R_A_BIT) | (1u << ENC_R_B_BIT));
    PORTB |=  (1u << ENC_R_A_BIT)  | (1u << ENC_R_B_BIT);
    PCMSK2 |= (1u << PCINT23);
    PCMSK0 |= (1u << PCINT5) | (1u << PCINT4);
    PCICR  |= (1u << PCIE2)  | (1u << PCIE0);
    g_prev_pind = PIND;
    g_prev_pinb = PINB;
}

/** @brief  Reset both encoder counters atomically. */
static void enc_reset(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_enc_left  = 0;
        g_enc_right = 0;
    }
}

/** @brief  Read both encoder counters atomically into provided pointers. */
static void enc_snapshot(int32_t *l, int32_t *r)
{
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
    DDRC |=  (1u << US_LEFT_TRIG)  | (1u << US_RIGHT_TRIG) | (1u << US_FRONT_TRIG);
    DDRC &= ~((1u << US_LEFT_ECHO) | (1u << US_RIGHT_ECHO) | (1u << US_FRONT_ECHO));
    PORTC &= ~((1u << US_LEFT_TRIG) | (1u << US_RIGHT_TRIG) | (1u << US_FRONT_TRIG));
}

/**
 * @brief  Send 10 µs trigger and measure echo pulse.
 * @return Distance in cm, 0 on timeout.
 */
static uint16_t us_read_once(uint8_t trig, uint8_t echo)
{
    uint16_t us;

    /* @brief  wait for any previous echo to settle */
    uint16_t settle = 0u;
    while ((PINC & (1u << echo)) && settle < 10000u) {
        _delay_us(1);
        settle++;
    }

    PORTC |=  (1u << trig);
    _delay_us(10);
    PORTC &= ~(1u << trig);

    /* wait for echo HIGH */
    for (us = 0u; !(PINC & (1u << echo)); ++us) {
        _delay_us(1);
        if (us > 4000u) return 0u;
    }

    /* measure echo HIGH duration */
    for (us = 0u; PINC & (1u << echo); ++us) {
        _delay_us(1);
        if (us >= US_TIMEOUT_ITER) return 0u;
    }

    return (uint16_t)(us / 58u);
}

/**
 * @brief  Take 3 readings and return the median — rejects noise spikes.
 */
static uint16_t us_read_median3(uint8_t trig, uint8_t echo)
{
    uint16_t s[3], tmp;
    s[0] = us_read_once(trig, echo); timer_delay_ms(20u);
    s[1] = us_read_once(trig, echo); timer_delay_ms(20u);
    s[2] = us_read_once(trig, echo);

    /* @brief  bubble sort 3 elements */
    if (s[0] > s[1]) { tmp = s[0]; s[0] = s[1]; s[1] = tmp; }
    if (s[1] > s[2]) { tmp = s[1]; s[1] = s[2]; s[2] = tmp; }
    if (s[0] > s[1]) { tmp = s[0]; s[0] = s[1]; s[1] = tmp; }
    return s[1];
}

/* =========================================================================
 * UART helpers
 * ========================================================================= */
static void uart_print_i16(int16_t v)
{
    char buf[7]; uint8_t i = 0, neg = 0;
    if (v < 0) { neg = 1; v = (int16_t)-v; }
    if (v == 0) { UART_SendChar('0'); return; }
    while (v > 0) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    if (neg) UART_SendChar('-');
    while (i > 0) UART_SendChar(buf[--i]);
}

static void uart_print_i32(int32_t v)
{
    char buf[12]; uint8_t i = 0, neg = 0;
    if (v < 0) { neg = 1; v = -v; }
    if (v == 0) { UART_SendChar('0'); return; }
    while (v > 0) { buf[i++] = (char)('0' + (v % 10)); v /= 10; }
    if (neg) UART_SendChar('-');
    while (i > 0) UART_SendChar(buf[--i]);
}

/* =========================================================================
 * Wall PID
 * ========================================================================= */
static float wall_integral        = 0.0f;
static float wall_prev_error      = 0.0f;
static float wall_deriv_filtered  = 0.0f;

static float wall_pid(float dl, float dr, float dt, float *raw_error_out)
{
    float error = dl - dr;
    *raw_error_out = error;

    if      (error >  WALL_DEADBAND) error -= WALL_DEADBAND;
    else if (error < -WALL_DEADBAND) error += WALL_DEADBAND;
    else { error = 0.0f; wall_integral *= 0.85f; }

    float pre_clamp = WALL_KP * error + WALL_KI * wall_integral;
    if (pre_clamp < WALL_MAX_OUT && pre_clamp > -WALL_MAX_OUT)
        wall_integral += error * dt;
    if (wall_integral >  WALL_INTEGRAL_LIM) wall_integral =  WALL_INTEGRAL_LIM;
    if (wall_integral < -WALL_INTEGRAL_LIM) wall_integral = -WALL_INTEGRAL_LIM;

    float raw_d = (error - wall_prev_error) / dt;
    wall_deriv_filtered = WALL_DERIV_FILTER * wall_deriv_filtered +
                         (1.0f - WALL_DERIV_FILTER) * raw_d;
    wall_prev_error = error;

    float out = WALL_KP * error + WALL_KI * wall_integral +
                WALL_KD * wall_deriv_filtered;
    if (out >  WALL_MAX_OUT) out =  WALL_MAX_OUT;
    if (out < -WALL_MAX_OUT) out = -WALL_MAX_OUT;
    return out;
}

/* =========================================================================
 * Encoder PID
 * ========================================================================= */
static float enc_integral   = 0.0f;
static float enc_prev_error = 0.0f;

static float enc_pid(int32_t l_ticks, int32_t r_ticks, float dt)
{
    const float ratio = (float)(BASE_SPEED) / (float)(BASE_SPEED - RIGHT_REDUCE);
    float error = (float)l_ticks - ratio * (float)r_ticks;
    enc_integral += error * dt;
    if (enc_integral >  ENC_INTEGRAL_LIM) enc_integral =  ENC_INTEGRAL_LIM;
    if (enc_integral < -ENC_INTEGRAL_LIM) enc_integral = -ENC_INTEGRAL_LIM;
    float deriv = (error - enc_prev_error) / dt;
    enc_prev_error = error;
    float out = ENC_KP * error + ENC_KI * enc_integral + ENC_KD * deriv;
    if (out >  ENC_MAX_OUT) out =  ENC_MAX_OUT;
    if (out < -ENC_MAX_OUT) out = -ENC_MAX_OUT;
    return out;
}

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
    /* ── Init ─────────────────────────────────────────────────────────── */
    timer_init();
    UART_Init(9600UL);
    ultrasonic_init();
    encoder_init();
    Motor_Init();

    /* @brief  IR pins as inputs, no pull-up (sensor has its own output) */
    DDRD  &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));
    PORTD &= ~((1u << IR_LEFT_BIT) | (1u << IR_RIGHT_BIT));

    sei();

    timer_delay_ms(500u);
    UART_SendString("Wall-follower ready.\r\n");

    /* ── Sensor warm-up ───────────────────────────────────────────────── */
    uint16_t last_good_l = 15u;
    uint16_t last_good_r = 15u;
    UART_SendString("Calibrating...\r\n");
    for (uint8_t wi = 0; wi < 5u; wi++) {
        uint16_t wl = us_read_median3(US_LEFT_TRIG, US_LEFT_ECHO);
        uint16_t wr = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);
        if (wl >= US_MIN_CM && wl <= US_MAX_CM) last_good_l = wl;
        if (wr >= US_MIN_CM && wr <= US_MAX_CM) last_good_r = wr;
        timer_delay_ms(50u);
    }
    UART_SendString("Go.\r\n");

    uint8_t  sensor_valid = 1u;
    uint32_t prev_time    = timer_get_ms();
    enc_reset();

    /* ── State machine ────────────────────────────────────────────────── */
    typedef enum {
        S_STRAIGHT = 0,  /* normal wall-following PID                  */
        S_APPROACH,      /* front <= FRONT_SLOW_CM, slow + wait for IR */
        S_TURN_RIGHT,    /* pivot right for TURN_DURATION_MS           */
        S_TURN_LEFT,     /* pivot left  for TURN_DURATION_MS           */
        S_POST_TURN      /* straight burst to clear junction           */
    } state_t;

    state_t state = S_STRAIGHT;

    uint32_t turn_start_ms = 0u;

    /**
     * @brief  IR debounce counters.
     * Each counter increments every loop when the IR sees open space.
     * Resets to 0 the moment a wall is seen again.
     * Turn only fires when counter reaches IR_DEBOUNCE_COUNT.
     */
    uint8_t ir_right_open_cnt = 0u;
    uint8_t ir_left_open_cnt  = 0u;

    /**
     * @brief  Grace period — skip IR checking for first N iterations.
     * Increased to 10 so sensors have time to stabilise after power-on.
     */
    uint8_t grace = 10u;

    while (1)
    {
        /* ================================================================
         * TURNING STATE — bypass wall PID entirely
         * ================================================================ */
        if (state == S_TURN_RIGHT || state == S_TURN_LEFT)
        {
            if (state == S_TURN_RIGHT)
                Motor_TurnRight(SPEED_TURN_OUTER, SPEED_TURN_INNER);
            else
                Motor_TurnLeft(SPEED_TURN_INNER, SPEED_TURN_OUTER);

            /* @brief  exit turn after fixed time — no IR dependency */
            if ((timer_get_ms() - turn_start_ms) >= TURN_DURATION_MS)
            {
                UART_SendString("Turn done\r\n");
                state         = S_POST_TURN;
                turn_start_ms = timer_get_ms(); /* reuse for post-turn */
                enc_reset();
            }
            continue;
        }

        /* ================================================================
         * POST-TURN — drive straight to clear junction
         * ================================================================ */
        if (state == S_POST_TURN)
        {
            Motor_Forward((uint8_t)BASE_SPEED,
                          (uint8_t)(BASE_SPEED - RIGHT_REDUCE));

            if ((timer_get_ms() - turn_start_ms) >= POST_TURN_MS)
            {
                /* @brief  reset all PID state so stale integrals don't jerk */
                wall_integral        = 0.0f;
                wall_prev_error      = 0.0f;
                wall_deriv_filtered  = 0.0f;
                enc_integral         = 0.0f;
                enc_prev_error       = 0.0f;
                enc_reset();

                /* @brief  also reset IR debounce counters */
                ir_right_open_cnt = 0u;
                ir_left_open_cnt  = 0u;

                state = S_STRAIGHT;
                UART_SendString("Resuming wall-follow\r\n");
            }
            continue;
        }

        /* ================================================================
         * Sensor reads
         * ================================================================ */
        uint16_t dl_raw = us_read_median3(US_LEFT_TRIG,  US_LEFT_ECHO);
        timer_delay_ms(30u);   /* @brief  prevent crosstalk between sensors */
        uint16_t dr_raw = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);
        timer_delay_ms(10u);
        uint16_t df = us_read_once(US_FRONT_TRIG, US_FRONT_ECHO);
        if (df == 0u) df = 255u;

        /* @brief  apply jump filter — reject spikes > JUMP_LIMIT cm */
        uint16_t dl = last_good_l;
        uint16_t dr = last_good_r;

        if (dl_raw >= US_MIN_CM && dl_raw <= US_MAX_CM) {
            uint16_t jump = (dl_raw > last_good_l) ? (dl_raw - last_good_l)
                                                   : (last_good_l - dl_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) {
                last_good_l = dl_raw; dl = dl_raw;
            }
        }
        if (dr_raw >= US_MIN_CM && dr_raw <= US_MAX_CM) {
            uint16_t jump = (dr_raw > last_good_r) ? (dr_raw - last_good_r)
                                                   : (last_good_r - dr_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) {
                last_good_r = dr_raw; dr = dr_raw;
            }
        }
        if (last_good_l > 0u && last_good_r > 0u) sensor_valid = 1u;

        /* ── dt ──────────────────────────────────────────────────────── */
        uint32_t now = timer_get_ms();
        float dt = (float)(now - prev_time) * 0.001f;
        if (dt < 0.005f) dt = 0.005f;
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
         * ALL OPEN — stop condition
         * ================================================================ */
        uint8_t left_open  = (dl == 0u) || (dl > US_MAX_CM);
        uint8_t right_open = (dr == 0u) || (dr > US_MAX_CM);
        uint8_t front_open = (df == 0u) || (df > FRONT_SLOW_CM);
        uint8_t ir_both_open = (!ir_left_wall()) && (!ir_right_wall());

        if (left_open && right_open && front_open && ir_both_open) {
            Motor_Forward(0u, 0u);
            UART_SendString("ALL OPEN - STOP\r\n");
            continue;
        }

        /* ================================================================
         * APPROACH detection — front wall close
         * ================================================================ */
        if (state == S_STRAIGHT && df <= FRONT_SLOW_CM) {
            state = S_APPROACH;
            UART_SendString("Approach\r\n");
        }

        /* ================================================================
         * IR DEBOUNCE TURN DETECTION
         *
         * Only runs in S_APPROACH.
         * Counter increments each loop while IR sees open space.
         * Resets immediately when wall is seen again.
         * Turn fires only after IR_DEBOUNCE_COUNT consecutive open reads.
         * ================================================================ */
        if (state == S_APPROACH)
        {
            /* -- Right IR debounce -------------------------------------- */
            if (!ir_right_wall())
                ir_right_open_cnt++;
            else
                ir_right_open_cnt = 0u;   /* @brief  wall seen — reset counter */

            /* -- Left IR debounce --------------------------------------- */
            if (!ir_left_wall())
                ir_left_open_cnt++;
            else
                ir_left_open_cnt = 0u;

            /* -- Fire right turn ---------------------------------------- */
            if (ir_right_open_cnt >= IR_DEBOUNCE_COUNT)
            {
                ir_right_open_cnt = 0u;
                ir_left_open_cnt  = 0u;
                state             = S_TURN_LEFT;
                turn_start_ms     = timer_get_ms();
                UART_SendString("Turn R\r\n");
                continue;
            }

            /* -- Fire left turn ----------------------------------------- */
            if (ir_left_open_cnt >= IR_DEBOUNCE_COUNT)
            {
                ir_right_open_cnt = 0u;
                ir_left_open_cnt  = 0u;
                state             = S_TURN_RIGHT;
                turn_start_ms     = timer_get_ms();
                UART_SendString("Turn L\r\n");
                continue;
            }

            /* @brief  both walls still present — keep creeping forward slowly */
        }

        /* ================================================================
         * PID — wall centering + encoder straight-line
         * ================================================================ */
        float raw_err = 0.0f;
        float w_corr  = 0.0f;
        float e_corr  = 0.0f;

        if (sensor_valid) {
            w_corr = wall_pid((float)dl, (float)dr, dt, &raw_err);
            float abs_err = raw_err < 0.0f ? -raw_err : raw_err;
            if (abs_err <= WALL_DEADBAND) {
                e_corr = enc_pid(el, er, dt);
            } else {
                /* @brief  wall error is large — reset encoder PID to avoid conflict */
                enc_integral   = 0.0f;
                enc_prev_error = 0.0f;
            }
        }

        /* ================================================================
         * Drive
         * ================================================================ */
        float   total = w_corr + e_corr;
        uint8_t base  = (state == S_APPROACH) ? SPEED_SLOW : (uint8_t)BASE_SPEED;

        int16_t ls = (int16_t)base + (int16_t)total;
        int16_t rs = (int16_t)(base - RIGHT_REDUCE) - (int16_t)total;
        ls = clamp16(ls, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        rs = clamp16(rs, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        Motor_Forward((uint8_t)ls, (uint8_t)rs);

        /* ================================================================
         * Debug output
         * ================================================================ */
        UART_SendString("L=");    uart_print_i16((int16_t)dl);
        UART_SendString(" R=");   uart_print_i16((int16_t)dr);
        UART_SendString(" F=");   uart_print_i16((int16_t)df);
        UART_SendString(" e=");   uart_print_i16((int16_t)raw_err);
        UART_SendString(" W=");   uart_print_i16((int16_t)w_corr);
        UART_SendString(" EL=");  uart_print_i32(el);
        UART_SendString(" ER=");  uart_print_i32(er);
        UART_SendString(" E=");   uart_print_i16((int16_t)e_corr);
        UART_SendString(" LS=");  uart_print_i16(ls);
        UART_SendString(" RS=");  uart_print_i16(rs);
        UART_SendString(" IRL="); uart_print_i16((int16_t)ir_left_wall());
        UART_SendString(" IRR="); uart_print_i16((int16_t)ir_right_wall());
        UART_SendString(" ST=");  uart_print_i16((int16_t)state);
        UART_SendString("\r\n");
    }

    return 0;
}