#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

/**
 * @brief Initialize motor driver pins
 */
void Motor_Init(void);

/**
 * @brief Move robot forward
 *
 * @param leftSpeed Left motor PWM (0–255)
 * @param rightSpeed Right motor PWM (0–255)
 */
void Motor_Forward(uint8_t leftSpeed, uint8_t rightSpeed);

/**
 * @brief Smooth right turn
 *
 * @param leftSpeed Left motor PWM
 * @param rightSpeed Right motor PWM
 */
void Motor_TurnRight(uint8_t leftSpeed, uint8_t rightSpeed);

/**
 * @brief Smooth left turn
 *
 * @param leftSpeed Left motor PWM
 * @param rightSpeed Right motor PWM
 */
void Motor_TurnLeft(uint8_t leftSpeed, uint8_t rightSpeed);

/**
 * @brief Stop both motors
 */
void Motor_Stop(void);

#endif