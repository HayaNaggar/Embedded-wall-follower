/**
 * @file    Ultrasonic.c
 * @brief   HC-SR04 ultrasonic sensor driver — ATmega328P
 *
 * All three sensors (front, left, right) share PORTC.
 * Timing uses timer_get_ms() / timer_get_us() — no _delay_ms or _delay_us.
 * The 10 µs trigger pulse is the only busy-wait; it lasts ~160 CPU cycles.
 */

#include "ultrasonic.h"
#include "timer.h"
#include <avr/io.h>
#include <stdint.h>

/* Maximum wait for echo pin to go HIGH after trigger */
#define US_ECHO_WAIT_MS   4u

/* Maximum echo HIGH duration → matches US_MAX_CM=60 cm (60×58=3480 µs + margin) */
#define US_ECHO_MAX_US 4000u

/* -------------------------------------------------------------------------
 * US_Init
 * ------------------------------------------------------------------------- */
void US_Init(void)
{
    /* TRIG pins → output, start LOW */
    DDRC  |=  (1u << US_FRONT_TRIG) | (1u << US_LEFT_TRIG) | (1u << US_RIGHT_TRIG);
    PORTC &= ~((1u << US_FRONT_TRIG) | (1u << US_LEFT_TRIG) | (1u << US_RIGHT_TRIG));

    /* ECHO pins → input, no pull-up (sensor drives the line) */
    DDRC  &= ~((1u << US_FRONT_ECHO) | (1u << US_LEFT_ECHO) | (1u << US_RIGHT_ECHO));
}

/* -------------------------------------------------------------------------
 * US_ReadCm
 * ------------------------------------------------------------------------- */
uint16_t US_ReadCm(uint8_t trig, uint8_t echo)
{
    uint32_t t0;

    /* 1. Wait for any lingering echo to clear (10 ms max) */
    t0 = timer_get_ms();
    while ((PINC & (1u << echo)) && (timer_get_ms() - t0) < 10u)
        ;

    /* 2. Send 10 µs trigger pulse, timed with timer_get_us() */
    PORTC |= (1u << trig);
    uint32_t t_pulse = timer_get_us();
    while ((timer_get_us() - t_pulse) < 10u)
        ;
    PORTC &= ~(1u << trig);

    /* 3. Wait for ECHO to go HIGH */
    t0 = timer_get_ms();
    while (!(PINC & (1u << echo))) {
        if ((timer_get_ms() - t0) >= US_ECHO_WAIT_MS)
            return 0u;
    }

    /* 4. Measure ECHO HIGH duration in microseconds */
    uint32_t echo_start = timer_get_us();
    while (PINC & (1u << echo)) {
        if ((timer_get_us() - echo_start) >= US_ECHO_MAX_US)
            return 0u;
    }

    uint32_t duration_us = timer_get_us() - echo_start;
    if (duration_us == 0u)
        return 0u;

    /* 5. Distance (cm) = duration_µs / 58 */
    return (uint16_t)(duration_us / 58u);
}
