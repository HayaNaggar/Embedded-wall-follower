#include "motor.h"
#include "pwm.h"
#include <avr/io.h>

/*
 * L298N Connections
 *
 * Left Motor:
 * IN1 -> PB0
 * IN2 -> PB1
 *
 * Right Motor:
 * IN3 -> PB2
 * IN4 -> PB3
 */

/**
 * @brief Initialize motor control pins
 */
void Motor_Init(void)
{
    /* Set PB0, PB1, PB2, PB3 as output */
    DDRB |=
        (1 << PB0) |
        (1 << PB1) |
        (1 << PB2) |
        (1 << PB3);

    PWM_Init();
    Motor_Stop();
}

/**
 * @brief Move both motors forward
 */
void Motor_Forward(uint8_t leftSpeed, uint8_t rightSpeed)
{
    /* Left Motor Forward */
    PORTB |= (1 << PB0);
    PORTB &= ~(1 << PB1);

    /* Right Motor Forward */
    PORTB |= (1 << PB2);
    PORTB &= ~(1 << PB3);

    PWM_SetLeftSpeed(leftSpeed);
    PWM_SetRightSpeed(rightSpeed);
}

/**
 * @brief Pivot right: left motor forward, right motor backward.
 * leftSpeed  = forward speed of outer (left) motor
 * rightSpeed = backward speed of inner (right) motor
 */
void Motor_TurnRight(uint8_t leftSpeed, uint8_t rightSpeed)
{
    /* Left motor: forward */
    PORTB |=  (1 << PB0);
    PORTB &= ~(1 << PB1);

    /* Right motor: backward */
    PORTB &= ~(1 << PB2);
    PORTB |=  (1 << PB3);

    PWM_SetLeftSpeed(leftSpeed);
    PWM_SetRightSpeed(rightSpeed);
}

/**
 * @brief Pivot left: right motor forward, left motor backward.
 * leftSpeed  = backward speed of inner (left) motor
 * rightSpeed = forward speed of outer (right) motor
 */
void Motor_TurnLeft(uint8_t leftSpeed, uint8_t rightSpeed)
{
    /* Left motor: backward */
    PORTB &= ~(1 << PB0);
    PORTB |=  (1 << PB1);

    /* Right motor: forward */
    PORTB |=  (1 << PB2);
    PORTB &= ~(1 << PB3);

    PWM_SetLeftSpeed(leftSpeed);
    PWM_SetRightSpeed(rightSpeed);
}

/**
 * @brief Stop both motors
 */
void Motor_Stop(void)
{
    /* Stop direction pins */
    PORTB &= ~(1 << PB0);
    PORTB &= ~(1 << PB1);
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);

    PWM_SetLeftSpeed(0);
    PWM_SetRightSpeed(0);
}