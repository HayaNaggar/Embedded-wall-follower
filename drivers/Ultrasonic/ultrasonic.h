/**
 * @file  Ultrasonic.h
 * @brief HC-SR04 driver using Timer1 CTC + ms counter for timing.
 *        All sensors assumed on PORTC. Trig/echo passed as pin bit numbers.
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>

/**
 * @brief Configure PORTC trig pins as output, echo pins as input.
 * @param trig_mask  OR of (1<<trig_pin) for all sensors
 * @param echo_mask  OR of (1<<echo_pin) for all sensors
 */
void Ultrasonic_Init(uint8_t trig_mask, uint8_t echo_mask);

/**
 * @brief Fire one trigger pulse and measure echo duration.
 * @param trig  Bit number of TRIG pin within PORTC
 * @param echo  Bit number of ECHO pin within PORTC
 * @return Distance in cm, or 0 on timeout / out of range.
 */
uint16_t Ultrasonic_ReadOnce(uint8_t trig, uint8_t echo);

/**
 * @brief Take 3 readings and return the median — rejects noise spikes.
 * @param trig  Bit number of TRIG pin within PORTC
 * @param echo  Bit number of ECHO pin within PORTC
 * @return Median distance in cm, or 0 on timeout.
 */
uint16_t Ultrasonic_ReadMedian3(uint8_t trig, uint8_t echo);

#endif /* ULTRASONIC_H_ */
