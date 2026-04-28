#include "pwm.h"
#include <avr/io.h>

/**
 * @brief Initialize Fast PWM mode using Timer0
 */
void PWM_Init(void)
{
    /* Set PD5 (OC0B) and PD6 (OC0A) as output */
    DDRD |= (1 << PD5) | (1 << PD6);

    /*
     * Fast PWM Mode
     * Non-inverting mode
     * Prescaler = 64
     */
    TCCR0A =
        (1 << COM0A1) |
        (1 << COM0B1) |
        (1 << WGM00)  |
        (1 << WGM01);

    TCCR0B =
        (1 << CS01) |
        (1 << CS00);

    /* Start with motors stopped */
    OCR0A = 0;
    OCR0B = 0;
}

/**
 * @brief Set left motor speed
 */
void PWM_SetLeftSpeed(uint8_t duty)
{
    OCR0B = duty;   /* PD5 */
}

/**
 * @brief Set right motor speed
 */
void PWM_SetRightSpeed(uint8_t duty)
{
    OCR0A = duty;   /* PD6 */
}