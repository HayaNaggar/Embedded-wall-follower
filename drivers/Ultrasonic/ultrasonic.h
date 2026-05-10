/**
 * @file    ultrasonic.h
 * @brief   HC-SR04 ultrasonic sensor driver for ATmega328P
 *
 * All three sensors share PORTC.  Each sensor has a dedicated TRIG (output)
 * and ECHO (input) pin.  The driver is non-blocking except for the mandatory
 * 10 µs trigger pulse, which is timed via timer_get_us() (no _delay_us).
 *
 * Pin assignments
 * ---------------
 *   Front TRIG → PC0 (A0)    Front ECHO → PC1 (A1)
 *   Left  TRIG → PC2 (A2)    Left  ECHO → PC3 (A3)
 *   Right TRIG → PC4 (A4)    Right ECHO → PC5 (A5)
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

/* PORTC bit numbers for each sensor */
#define US_FRONT_TRIG  0u   /* PC0 = A0 */
#define US_FRONT_ECHO  1u   /* PC1 = A1 */
#define US_LEFT_TRIG   2u   /* PC2 = A2 */
#define US_LEFT_ECHO   3u   /* PC3 = A3 */
#define US_RIGHT_TRIG  4u   /* PC4 = A4 */
#define US_RIGHT_ECHO  5u   /* PC5 = A5 */

/**
 * @brief  Configure TRIG pins as outputs and ECHO pins as inputs on PORTC.
 *         Must be called once at startup before US_ReadCm().
 */
void US_Init(void);

/**
 * @brief  Fire one trigger pulse and measure the echo return time.
 *
 * @param  trig  PORTC bit of the TRIG pin (US_FRONT_TRIG, US_LEFT_TRIG, …)
 * @param  echo  PORTC bit of the ECHO pin (US_FRONT_ECHO, US_LEFT_ECHO, …)
 * @return Distance in whole centimetres, or 0 if no echo within range.
 *
 * Timing details
 * --------------
 *   Trigger pulse: 10 µs HIGH, timed with timer_get_us() — no _delay_us.
 *   Echo wait:     up to 4 ms for echo to go HIGH (timer_get_ms timeout).
 *   Echo measure:  up to 8 ms pulse = ~136 cm maximum range.
 *   Distance:      duration_us / 58  (sound round-trip in cm).
 */
uint16_t US_ReadCm(uint8_t trig, uint8_t echo);

#endif /* ULTRASONIC_H */
