/**
 * @file    main.c
 * @brief   Straight-line PID controller – ATmega328P / Arduino Nano
 *
 * Hardware:
 *  US Left   : TRIG = PC2, ECHO = PC3
 *  US Right  : TRIG = PC4, ECHO = PC5
 *  Motor IN1-4 : PB0-PB3
 *  PWM L / R   : PD6/PD5  (Timer0 OC0A/OC0B)
 *
 * Strategy:
 *  error = dist_left - dist_right
 *  error > 0 → closer to right wall → steer left  (right faster, left slower)
 *  error < 0 → closer to left wall  → steer right (left faster, right slower)
 *  Right motor base speed is set slightly higher than left to
 *  compensate for natural left drift.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "../drivers/uart/uart.h"
#include "../drivers/timer/timer.h"
#include "../modules/motor/motor.h"

/* =========================================================================
 * Ultrasonic pin config  (PORTC)
 * ========================================================================= */
#define US_LEFT_TRIG_PIN    2   /* PC2 */
#define US_LEFT_ECHO_PIN    3   /* PC3 */
#define US_RIGHT_TRIG_PIN   4   /* PC4 */
#define US_RIGHT_ECHO_PIN   5   /* PC5 */

/* =========================================================================
 * Drive parameters  – tune on hardware
 * ========================================================================= */
#define BASE_LEFT_SPEED     155     /* right slightly faster to offset drift  */
#define BASE_RIGHT_SPEED    160

#define PID_KP              3.0f
#define PID_KI              0.05f
#define PID_KD              1.5f

#define PID_MAX_CORRECTION  60      /* maximum per-side speed adjustment       */
#define PID_INTEGRAL_LIMIT  200.0f  /* anti-windup clamp                       */

#define MIN_MOTOR_SPEED     60      /* keep motors above stall threshold       */

#define BAUD_RATE           9600UL

/* =========================================================================
 * Ultrasonic helpers
 * ========================================================================= */
static uint16_t ultrasonic_read_cm(uint8_t trig_pin, uint8_t echo_pin)
{
    uint32_t count = 0;

    PORTC |=  (1 << trig_pin);
    timer_delay_us(10);
    PORTC &= ~(1 << trig_pin);

    count = 0;
    while (!(PINC & (1 << echo_pin))) {
        if (++count > 30000UL) return 0;
    }

    count = 0;
    while (PINC & (1 << echo_pin)) {
        count++;
        timer_delay_us(1);
        if (count > 30000UL) break;
    }

    return (uint16_t)(count / 58);
}

static void ultrasonic_init(void)
{
    DDRC |=  (1 << US_LEFT_TRIG_PIN) | (1 << US_RIGHT_TRIG_PIN);
    DDRC &= ~((1 << US_LEFT_ECHO_PIN) | (1 << US_RIGHT_ECHO_PIN));
    PORTC &= ~((1 << US_LEFT_TRIG_PIN) | (1 << US_RIGHT_TRIG_PIN));
}

/* =========================================================================
 * Helper: clamp int16_t to [lo, hi]
 * ========================================================================= */
static int16_t clamp16(int16_t val, int16_t lo, int16_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void)
{
    timer_init();
    UART_Init(BAUD_RATE);
    ultrasonic_init();
    Motor_Init();
    sei();

    timer_delay_ms(500);
    UART_SendString("Straight PID ready.\r\n");

    float    integral   = 0.0f;
    float    prev_error = 0.0f;
    uint32_t prev_time  = timer_get_ms();

    while (1) {
        uint16_t dist_left  = ultrasonic_read_cm(US_LEFT_TRIG_PIN,  US_LEFT_ECHO_PIN);
        uint16_t dist_right = ultrasonic_read_cm(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

        /* If either sensor has no valid reading, drive straight with base bias */
        if (dist_left == 0 || dist_right == 0) {
            Motor_Forward(BASE_LEFT_SPEED, BASE_RIGHT_SPEED);
            continue;
        }

        uint32_t now = timer_get_ms();
        float dt = (float)(now - prev_time) * 0.001f;
        if (dt < 0.001f) dt = 0.001f;
        prev_time = now;

        /* Positive error → robot is closer to right wall → need to steer left */
        float error = (float)dist_left - (float)dist_right;

        integral += error * dt;
        if (integral >  PID_INTEGRAL_LIMIT) integral =  PID_INTEGRAL_LIMIT;
        if (integral < -PID_INTEGRAL_LIMIT) integral = -PID_INTEGRAL_LIMIT;

        float derivative = (error - prev_error) / dt;
        prev_error = error;

        float pid_out = PID_KP * error + PID_KI * integral + PID_KD * derivative;

        int16_t correction = clamp16((int16_t)pid_out,
                                     -PID_MAX_CORRECTION,
                                      PID_MAX_CORRECTION);

        /* Steer left  (correction > 0): slow left, speed up right
         * Steer right (correction < 0): speed up left, slow right  */
        int16_t left_speed  = (int16_t)BASE_LEFT_SPEED  - correction;
        int16_t right_speed = (int16_t)BASE_RIGHT_SPEED + correction;

        left_speed  = clamp16(left_speed,  MIN_MOTOR_SPEED, 255);
        right_speed = clamp16(right_speed, MIN_MOTOR_SPEED, 255);

        Motor_Forward((uint8_t)left_speed, (uint8_t)right_speed);
    }

    return 0;
}
