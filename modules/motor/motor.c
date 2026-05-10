/**
 * @file    motor.c
 * @brief   L298N motor driver for ATmega328P.
 *
 * L298N wiring:
 *   IN1 → PB0, IN2 → PB1  (code-left  = physical RIGHT motor)
 *   IN3 → PB2, IN4 → PB3  (code-right = physical LEFT  motor)
 */

#include "motor.h"
#include "pwm.h"
#include <avr/io.h>

/** @brief Initialize motor direction pins and PWM. */
void Motor_Init(void)
{
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
    PWM_Init();
    Motor_Stop();
}

/** @brief Drive both motors forward at the given speeds. */
void Motor_Forward(uint8_t leftSpeed, uint8_t rightSpeed)
{
    PORTB |=  (1 << PB0);
    PORTB &= ~(1 << PB1);
    PORTB |=  (1 << PB2);
    PORTB &= ~(1 << PB3);
    PWM_SetLeftSpeed(leftSpeed);
    PWM_SetRightSpeed(rightSpeed);
}

/**
 * @brief  Pivot right: code-left forward, code-right backward.
 * @param  leftSpeed   Forward speed of the outer (code-left) motor.
 * @param  rightSpeed  Backward speed of the inner (code-right) motor.
 */
void Motor_TurnRight(uint8_t leftSpeed, uint8_t rightSpeed)
{
    PORTB |=  (1 << PB0);
    PORTB &= ~(1 << PB1);
    PORTB &= ~(1 << PB2);
    PORTB |=  (1 << PB3);
    PWM_SetLeftSpeed(leftSpeed);
    PWM_SetRightSpeed(rightSpeed);
}

/**
 * @brief  Pivot left: code-right forward, code-left backward.
 * @param  leftSpeed   Backward speed of the inner (code-left) motor.
 * @param  rightSpeed  Forward speed of the outer (code-right) motor.
 */
void Motor_TurnLeft(uint8_t leftSpeed, uint8_t rightSpeed)
{
    PORTB &= ~(1 << PB0);
    PORTB |=  (1 << PB1);
    PORTB |=  (1 << PB2);
    PORTB &= ~(1 << PB3);
    PWM_SetLeftSpeed(leftSpeed);
    PWM_SetRightSpeed(rightSpeed);
}

/** @brief Brake both motors (all direction pins low, duty 0). */
void Motor_Stop(void)
{
    PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3));
    PWM_SetLeftSpeed(0);
    PWM_SetRightSpeed(0);
}
