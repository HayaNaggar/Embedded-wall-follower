/**
 * @file    pwm.c
 * @brief   Timer0 Fast PWM driver for ATmega328P motor speed control.
 *
 * OC0A = PD6 (right motor), OC0B = PD5 (left motor)
 * Fast PWM, non-inverting, prescaler 64 → ~976 Hz PWM frequency.
 */

#include "pwm.h"
#include <avr/io.h>
#include <stdint.h>

/** @brief Initialize Timer0 Fast PWM on OC0A (PD6) and OC0B (PD5). */
void PWM_Init(void)
{
    DDRD |= (1 << PD5) | (1 << PD6);
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 0;
    OCR0B  = 0;
}

/** @brief Set left motor PWM duty cycle (0–255). */
void PWM_SetLeftSpeed(uint8_t duty)  { OCR0B = duty; }

/** @brief Set right motor PWM duty cycle (0–255). */
void PWM_SetRightSpeed(uint8_t duty) { OCR0A = duty; }
