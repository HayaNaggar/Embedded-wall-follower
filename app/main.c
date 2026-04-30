/*
 * Wall-centering robot — ATmega328P / Arduino Nano
 *
 * Pins:
 *   US Left  : TRIG=A2(PC2), ECHO=A3(PC3)
 *   US Right : TRIG=A4(PC4), ECHO=A5(PC5)
 *   Motor    : IN1-IN4 = PB0-PB3
 *   PWM      : D5(PD5)=left, D6(PD6)=right  (Timer0 OC0B/OC0A)
 *
 * Logic: PID on (dist_left - dist_right), target = 0 (centered).
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "../drivers/timer/timer.h"
#include "../modules/motor/motor.h"

/* =========================================================================
 * Ultrasonic pins  (PORTC)
 * ========================================================================= */
#define US_LEFT_TRIG    2   /* PC2 = A2 */
#define US_LEFT_ECHO    3   /* PC3 = A3 */
#define US_RIGHT_TRIG   4   /* PC4 = A4 */
#define US_RIGHT_ECHO   5   /* PC5 = A5 */

/* =========================================================================
 * Tuning
 * ========================================================================= */
#define BASE_SPEED      150     /* straight-line PWM (0-255)         */
#define MIN_SPEED        60     /* minimum PWM to keep motors moving  */
#define PID_OUTPUT_MAX   80.0f  /* max speed adjustment (PWM counts)  */

/* PID gains — tune Kp first, then Kd, leave Ki = 0 until stable */
#define PID_KP           2.5f
#define PID_KI           0.0f
#define PID_KD           1.2f
#define PID_INTEGRAL_MAX 60.0f

/* =========================================================================
 * Ultrasonic read
 * ========================================================================= */
static void ultrasonic_init(void)
{
    DDRC |=  (1 << US_LEFT_TRIG) | (1 << US_RIGHT_TRIG);
    DDRC &= ~((1 << US_LEFT_ECHO) | (1 << US_RIGHT_ECHO));
    PORTC &= ~((1 << US_LEFT_TRIG) | (1 << US_RIGHT_TRIG));
}

static uint16_t ultrasonic_read_cm(uint8_t trig, uint8_t echo)
{
    uint32_t count = 0;

    PORTC |=  (1 << trig);
    timer_delay_us(10);
    PORTC &= ~(1 << trig);

    count = 0;
    while (!(PINC & (1 << echo))) {
        if (++count > 30000UL) return 0;
    }

    count = 0;
    while (PINC & (1 << echo)) {
        count++;
        timer_delay_us(1);
        if (count > 30000UL) break;
    }

    return (uint16_t)(count / 58);
}

/* =========================================================================
 * PID state
 * ========================================================================= */
static float g_integral   = 0.0f;
static float g_prev_error = 0.0f;

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void)
{
    timer_init();
    ultrasonic_init();
    Motor_Init();
    sei();

    timer_delay_ms(500);    /* let sensors settle */

    while (1) {
        uint16_t dist_left  = ultrasonic_read_cm(US_LEFT_TRIG,  US_LEFT_ECHO);
        uint16_t dist_right = ultrasonic_read_cm(US_RIGHT_TRIG, US_RIGHT_ECHO);

        if (dist_left == 0 || dist_right == 0) {
            /* One sensor failed — go straight, don't update PID state */
            Motor_Forward(BASE_SPEED, BASE_SPEED);
            continue;
        }

        /* PID: error > 0 means closer to right wall → steer left */
        float error = (float)dist_left - (float)dist_right;

        g_integral += error;
        if (g_integral >  PID_INTEGRAL_MAX) g_integral =  PID_INTEGRAL_MAX;
        if (g_integral < -PID_INTEGRAL_MAX) g_integral = -PID_INTEGRAL_MAX;

        float derivative = error - g_prev_error;
        g_prev_error = error;

        float output = (PID_KP * error)
                     + (PID_KI * g_integral)
                     + (PID_KD * derivative);

        if (output >  PID_OUTPUT_MAX) output =  PID_OUTPUT_MAX;
        if (output < -PID_OUTPUT_MAX) output = -PID_OUTPUT_MAX;

        /* error > 0 → steer left: reduce left, increase right */
        int16_t ls = (int16_t)BASE_SPEED - (int16_t)output;
        int16_t rs = (int16_t)BASE_SPEED + (int16_t)output;

        if (ls < MIN_SPEED) ls = MIN_SPEED;
        if (ls > 255)       ls = 255;
        if (rs < MIN_SPEED) rs = MIN_SPEED;
        if (rs > 255)       rs = 255;

        Motor_Forward((uint8_t)ls, (uint8_t)rs);
    }

    return 0;
}
