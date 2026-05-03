/**
 * @file    main.c
 * @brief   Wall-following robot – ATmega328P / Arduino Nano
 *          Wall-centre PID  +  Encoder straight-line PID
 *
 * PIN ASSIGNMENTS
 *   US Left  TRIG → PC2 (A2)    US Left  ECHO → PC3 (A3)
 *   US Right TRIG → PC4 (A4)    US Right ECHO → PC5 (A5)
 *   Motor L IN1   → PB0 (D8)    Motor L IN2   → PB1 (D9)
 *   Motor R IN3   → PB2 (D10)   Motor R IN4   → PB3 (D11)
 *   PWM Left ENA  → PD5 (D5)    PWM Right ENB → PD6 (D6)
 *   Enc A Left    → PD7 (D7)    PCINT23
 *   Enc B Left    → PD2 (D2)    direction sense
 *   Enc A Right   → PB5 (D13)   PCINT5
 *   Enc B Right   → PB4 (D12)   PCINT4
 *   UART TX       → PD1 (D1)    9600 baud
 *
 * ==========================================================================
 * ROOT CAUSE OF WRONG DISTANCES (v11 → v12)
 * ==========================================================================
 *
 * Both sensors physically work but consistently report ~8cm when the real
 * distance is ~12cm — a 1.5× under-read on every measurement.
 *
 * The cause is in us_read_once(). We count loop iterations of _delay_us(1)
 * and divide by 58 (the standard HC-SR04 µs-to-cm constant):
 *
 *     distance_cm = iteration_count / 58
 *
 * But each iteration is NOT 1µs. At 16MHz with the compiler optimisation
 * level used, _delay_us(1) = 16 cycles, plus the loop body overhead
 * (PINC read, counter increment, compare, branch) adds ~7–8 cycles.
 * Total ≈ 23–24 cycles ≈ 1.5µs per iteration.
 *
 * So the real formula is:
 *     distance_cm = (iteration_count × 1.5µs) / 58µs_per_cm
 *                 = iteration_count / (58 / 1.5)
 *                 = iteration_count / 38.67
 *                 → divisor = 39
 *
 * Verification: real 12cm → echo = 696µs → 696/1.5 = 464 iterations
 *               464 / 39 = 11.9cm ≈ 12cm  ✓
 *
 * FIX: change the divisor from 58 to 39.
 * The timeout threshold is scaled the same way (×1.5) so range is unchanged.
 *
 * With both sensors now accurate, DUAL-SENSOR mode is restored.
 * error = dl - dr = 0 when centred — no TARGET_L constant needed,
 * the control is self-calibrating regardless of car width.
 *
 * ==========================================================================
 * OTHER ACTIVE FIXES (carried from v10/v11)
 * ==========================================================================
 *
 * JUMP FILTER (double-echo / spike rejection):
 *   Reject any reading that jumps more than JUMP_LIMIT cm from the previous
 *   confirmed reading in one loop. Catches the 2× double-echo artefact and
 *   single-sample spikes. Real wall drift at driving speed is <4cm per 90ms.
 *   JUMP_LIMIT = 6cm catches artefacts while passing real movement.
 *
 * ENCODER PID GATING:
 *   Encoder PID runs ONLY when |wall_error| ≤ WALL_DEADBAND.
 *   When the wall PID is actively steering, encoder PID fighting it was
 *   the primary cause of sinusoidal swerving. State is reset during
 *   wall corrections so it doesn't burst when re-enabled.
 *
 * LOOP SPEED:
 *   3-sample median with 20ms inter-ping gaps → ~90ms total loop.
 *   PID gains tuned for this period.
 *
 * TIMER NOTE:
 *   Timer1 is a 1ms CTC timer. TCNT1 is NOT used for echo timing here —
 *   it would wrap every 1ms and corrupt measurements (previous bug).
 *   Echo timing uses the _delay_us counting loop with divisor=39.
 */

 //el commentss de hattshalll lena for noww

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>
#include <util/delay.h>

#include "../drivers/timer/timer.h"
#include "../drivers/uart/uart.h"
#include "../modules/motor/motor.h"

/* =========================================================================
 * Ultrasonic
 * =========================================================================
 * US_DIVISOR = 39  (was 58 — see header for derivation)
 * US_TIMEOUT  scaled by same 1.5× factor: 8000µs real → 5333 iterations
 * JUMP_LIMIT  = 6cm — rejects double-echo and spikes, passes real drift
 * ========================================================================= */
#define US_LEFT_TRIG    2u
#define US_LEFT_ECHO    3u
#define US_RIGHT_TRIG   4u
#define US_RIGHT_ECHO   5u
#define US_MIN_CM       2u
#define US_MAX_CM       60u
#define US_DIVISOR      39u   /* calibrated: real_cm = iterations / 39 */
#define US_TIMEOUT_ITER 5333u /* 8000µs / 1.5µs_per_iter               */
#define JUMP_LIMIT      6u    /* cm — max valid change between loops    */

/* =========================================================================
 * Encoder pins
 * ========================================================================= */
#define ENC_L_A_BIT   7u
#define ENC_L_B_BIT   2u
#define ENC_R_A_BIT   5u
#define ENC_R_B_BIT   4u

/* =========================================================================
 * Drive parameters  –  DO NOT CHANGE
 * ========================================================================= */
#define BASE_SPEED      150u
#define MAX_SPEED       210u
#define MIN_SPEED        60u

/* Motor speed balance
 * The left motor is mechanically stronger than the right.
 * We balance by REDUCING right motor base speed instead of boosting left.
 * This keeps absolute speeds lower and reduces oscillation.
 *
 * RIGHT_REDUCE: start at 20, increase by 5 if still swerves left.
 *               decrease by 5 if swerves right.
 * LEFT_TRIM must stay 0 when using RIGHT_REDUCE.               */
#define LEFT_TRIM        0
#define RIGHT_REDUCE     0   /* right motor base = BASE_SPEED - RIGHT_REDUCE */

/* =========================================================================
 * Wall-centering PID
 * error = dl - dr  (+ve → closer to right wall → steer left)
 * Gains for ~90ms loop (3-sample median, 20ms inter-ping gaps).
 * ========================================================================= */
#define WALL_KP             1.2f  /* reduced: first-loop overcorrection was swerving left */
#define WALL_KI             0.04f
#define WALL_KD             0.3f
#define WALL_MAX_OUT        35.0f
#define WALL_INTEGRAL_LIM   25.0f
#define WALL_DEADBAND       1.5f   /* cm — above sensor noise floor      */
#define WALL_DERIV_FILTER   0.55f

/* =========================================================================
 * Encoder PID  (active only when wall error is settled)
 * ========================================================================= */
#define ENC_KP              0.8f
#define ENC_KI              0.0f
#define ENC_KD              0.1f
#define ENC_MAX_OUT         12.0f
#define ENC_INTEGRAL_LIM    40.0f

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

static void enc_reset(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_enc_left = 0; g_enc_right = 0; }
}

static void enc_snapshot(int32_t *l, int32_t *r)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *l = g_enc_left; *r = g_enc_right; }
}

/* =========================================================================
 * Ultrasonic
 * ========================================================================= */
static void ultrasonic_init(void)
{
    DDRC  |=  (1u << US_LEFT_TRIG) | (1u << US_RIGHT_TRIG);
    DDRC  &= ~((1u << US_LEFT_ECHO) | (1u << US_RIGHT_ECHO));
    PORTC &= ~((1u << US_LEFT_TRIG) | (1u << US_RIGHT_TRIG));
}

static uint16_t us_read_once(uint8_t trig, uint8_t echo)
{
    uint16_t us;

    PORTC |=  (1u << trig);
    _delay_us(10);
    PORTC &= ~(1u << trig);

    /* Wait for echo HIGH – timeout ~6ms */
    for (us = 0u; !(PINC & (1u << echo)); ++us) {
        _delay_us(1);
        if (us > 4000u) return 0u;
    }

    /* Count iterations while echo HIGH.
     * Each iteration ≈ 1.5µs (measured empirically: 12cm real → 8cm with /58,
     * corrected to /39). Timeout = US_TIMEOUT_ITER iterations ≈ 8000µs.   */
    for (us = 0u; PINC & (1u << echo); ++us) {
        _delay_us(1);
        if (us >= US_TIMEOUT_ITER) return 0u;
    }

    return (uint16_t)(us / US_DIVISOR);
}

/* 3-sample median, 20ms between pings → ~44ms per sensor, ~90ms total loop */
static uint16_t us_read_median3(uint8_t trig, uint8_t echo)
{
    uint16_t s[3], tmp;
    s[0] = us_read_once(trig, echo); timer_delay_ms(20u);
    s[1] = us_read_once(trig, echo); timer_delay_ms(20u);
    s[2] = us_read_once(trig, echo);
    /* Sort network for 3 elements */
    if (s[0] > s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    if (s[1] > s[2]) { tmp=s[1]; s[1]=s[2]; s[2]=tmp; }
    if (s[0] > s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    return s[1]; /* median */
}

/* =========================================================================
 * UART helpers
 * ========================================================================= */
static void uart_print_i16(int16_t v)
{
    char buf[7]; uint8_t i=0, neg=0;
    if (v<0){neg=1;v=(int16_t)-v;}
    if (v==0){UART_SendChar('0');return;}
    while(v>0){buf[i++]=(char)('0'+(v%10));v/=10;}
    if(neg) UART_SendChar('-');
    while(i>0) UART_SendChar(buf[--i]);
}
static void uart_print_i32(int32_t v)
{
    char buf[12]; uint8_t i=0, neg=0;
    if(v<0){neg=1;v=-v;}
    if(v==0){UART_SendChar('0');return;}
    while(v>0){buf[i++]=(char)('0'+(v%10));v/=10;}
    if(neg) UART_SendChar('-');
    while(i>0) UART_SendChar(buf[--i]);
}

/* =========================================================================
 * Wall PID  —  dual sensor, error = dl - dr
 * ========================================================================= */
static float wall_integral       = 0.0f;
static float wall_prev_error     = 0.0f;
static float wall_deriv_filtered = 0.0f;

static float wall_pid(float dl, float dr, float dt, float *raw_error_out)
{
    float error = dl - dr;
    *raw_error_out = error;

    if      (error >  WALL_DEADBAND) error -= WALL_DEADBAND;
    else if (error < -WALL_DEADBAND) error += WALL_DEADBAND;
    else {
        error = 0.0f;
        wall_integral *= 0.85f;
    }

    float pre_clamp = WALL_KP * error + WALL_KI * wall_integral;
    if (pre_clamp < WALL_MAX_OUT && pre_clamp > -WALL_MAX_OUT)
        wall_integral += error * dt;
    if (wall_integral >  WALL_INTEGRAL_LIM) wall_integral =  WALL_INTEGRAL_LIM;
    if (wall_integral < -WALL_INTEGRAL_LIM) wall_integral = -WALL_INTEGRAL_LIM;

    float raw_d = (error - wall_prev_error) / dt;
    wall_deriv_filtered = WALL_DERIV_FILTER * wall_deriv_filtered
                        + (1.0f - WALL_DERIV_FILTER) * raw_d;
    wall_prev_error = error;

    float out = WALL_KP*error + WALL_KI*wall_integral + WALL_KD*wall_deriv_filtered;
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
    float deriv    = (error - enc_prev_error) / dt;
    enc_prev_error = error;
    float out = ENC_KP*error + ENC_KI*enc_integral + ENC_KD*deriv;
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
    timer_init();
    UART_Init(9600UL);
    ultrasonic_init();
    encoder_init();
    Motor_Init();
    sei();

    timer_delay_ms(500u);
    UART_SendString("Wall-follower v13 ready.\r\n");
    UART_SendString("L= R= e= W= EL= ER= E= LS= RS=\r\n");

    /* ── Warm-up: take 5 sensor readings before starting motors ──────────
     * Discards the first few noisy readings and seeds last_good with real
     * values so the jump filter and PID start from a known-good baseline.
     * Motors stay off during this phase.                                  */
    uint16_t last_good_l = 15u;
    uint16_t last_good_r = 15u;

    UART_SendString("Calibrating sensors...\r\n");
    for (uint8_t wi = 0; wi < 5u; wi++) {
        uint16_t wl = us_read_median3(US_LEFT_TRIG,  US_LEFT_ECHO);
        uint16_t wr = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);
        if (wl >= US_MIN_CM && wl <= US_MAX_CM) last_good_l = wl;
        if (wr >= US_MIN_CM && wr <= US_MAX_CM) last_good_r = wr;
        timer_delay_ms(50u);
    }
    UART_SendString("Ready. L0="); 
    /* print initial readings */
    {
        char buf[7]; uint8_t i=0;
        uint16_t v = last_good_l;
        if(v==0){UART_SendChar('0');}
        else{while(v>0){buf[i++]=(char)('0'+(v%10));v/=10;} while(i>0)UART_SendChar(buf[--i]);}
    }
    UART_SendString(" R0=");
    {
        char buf[7]; uint8_t i=0;
        uint16_t v = last_good_r;
        if(v==0){UART_SendChar('0');}
        else{while(v>0){buf[i++]=(char)('0'+(v%10));v/=10;} while(i>0)UART_SendChar(buf[--i]);}
    }
    UART_SendString("\r\n");

    uint8_t  sensor_valid = 1u;  /* warm-up guarantees valid baseline */
    uint32_t prev_time = timer_get_ms();
    enc_reset();

    while (1)
    {
        /* --- 1. Sensor reads with jump filter --------------------------- */
        uint16_t dl_raw = us_read_median3(US_LEFT_TRIG,  US_LEFT_ECHO);
        uint16_t dr_raw = us_read_median3(US_RIGHT_TRIG, US_RIGHT_ECHO);

        uint16_t dl = last_good_l;
        uint16_t dr = last_good_r;

        if (dl_raw >= US_MIN_CM && dl_raw <= US_MAX_CM) {
            uint16_t jump = (dl_raw > last_good_l)
                          ? (dl_raw - last_good_l)
                          : (last_good_l - dl_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) {
                last_good_l = dl_raw;
                dl = dl_raw;
            }
        }

        if (dr_raw >= US_MIN_CM && dr_raw <= US_MAX_CM) {
            uint16_t jump = (dr_raw > last_good_r)
                          ? (dr_raw - last_good_r)
                          : (last_good_r - dr_raw);
            if (jump <= JUMP_LIMIT || sensor_valid == 0u) {
                last_good_r = dr_raw;
                dr = dr_raw;
            }
        }

        if (last_good_l > 0u && last_good_r > 0u) sensor_valid = 1u;

        /* --- 2. dt ------------------------------------------------------- */
        uint32_t now = timer_get_ms();
        float dt = (float)(now - prev_time) * 0.001f;
        if (dt < 0.005f) dt = 0.005f;
        prev_time = now;

        /* --- 3. Encoder snapshot ----------------------------------------- */
        int32_t el, er;
        enc_snapshot(&el, &er);
        enc_reset();

        /* --- 4 & 5. PIDs ------------------------------------------------- */
        float raw_err = 0.0f;
        float w_corr  = 0.0f;
        float e_corr  = 0.0f;

        /* Grace period: skip first 2 loops to let speeds and sensors settle */
        static uint8_t grace = 2u;
        if (grace > 0u) { grace--; Motor_Forward((uint8_t)(BASE_SPEED), (uint8_t)(BASE_SPEED - RIGHT_REDUCE)); continue; }

        if (sensor_valid)
        {
            w_corr = wall_pid((float)dl, (float)dr, dt, &raw_err);

            /* Gate encoder PID: off during active wall corrections        */
            float abs_err = raw_err < 0.0f ? -raw_err : raw_err;
            if (abs_err <= WALL_DEADBAND)
            {
                e_corr = enc_pid(el, er, dt);
            }
            else
            {
                enc_integral   = 0.0f;
                enc_prev_error = 0.0f;
            }
        }

        /* --- 6. Drive ---------------------------------------------------- */
        float total = w_corr + e_corr;

        /* Sign convention:
         *   error = L - R
         *   +ve error → closer to RIGHT wall → steer LEFT
         *              → speed up left, slow down right
         *   total > 0 → add to left, subtract from right          */
        int16_t ls = (int16_t)(BASE_SPEED)               + (int16_t)total;
        int16_t rs = (int16_t)(BASE_SPEED - RIGHT_REDUCE) - (int16_t)total;

        ls = clamp16(ls, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);
        rs = clamp16(rs, (int16_t)MIN_SPEED, (int16_t)MAX_SPEED);

        Motor_Forward((uint8_t)ls, (uint8_t)rs);

        /* --- 7. Debug ---------------------------------------------------- */
        UART_SendString("L=");  uart_print_i16((int16_t)dl);
        UART_SendString(" R="); uart_print_i16((int16_t)dr);
        UART_SendString(" e="); uart_print_i16((int16_t)raw_err);
        UART_SendString(" W="); uart_print_i16((int16_t)w_corr);
        UART_SendString(" EL=");uart_print_i32(el);
        UART_SendString(" ER=");uart_print_i32(er);
        UART_SendString(" E="); uart_print_i16((int16_t)e_corr);
        UART_SendString(" LS=");uart_print_i16(ls);
        UART_SendString(" RS=");uart_print_i16(rs);
        UART_SendString("\r\n");
    }
    return 0;
}
