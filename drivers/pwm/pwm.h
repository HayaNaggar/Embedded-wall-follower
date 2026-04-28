#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>

/**
 * @brief Initialize PWM for motor speed control
 *
 * Uses Timer0 on Arduino Uno (ATmega328P)
 * OC0A -> PD6 (Pin D6)
 * OC0B -> PD5 (Pin D5)
 */
void PWM_Init(void);

/**
 * @brief Set left motor speed using PWM
 *
 * @param duty Duty cycle (0–255)
 */
void PWM_SetLeftSpeed(uint8_t duty);

/**
 * @brief Set right motor speed using PWM
 *
 * @param duty Duty cycle (0–255)
 */
void PWM_SetRightSpeed(uint8_t duty);

#endif