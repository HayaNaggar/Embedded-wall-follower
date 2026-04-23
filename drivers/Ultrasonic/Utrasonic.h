/**
 * @file ultrasonic.h
 * @brief Driver for HC-SR04 Ultrasonic Sensor
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>

/**
 * @brief Initializes the GPIO pins for the ultrasonic sensor.
 * @param trigger_pin The pin connected to the sensor's TRIG
 * @param echo_pin The pin connected to the sensor's ECHO
 */
void Ultrasonic_Init(void);

/**
 * @brief Triggers a pulse and measures the distance.
 * @return Distance in centimeters, or 0 if out of range/timeout.
 */
uint16_t Ultrasonic_ReadDistance(void);

#endif /* ULTRASONIC_H_ */