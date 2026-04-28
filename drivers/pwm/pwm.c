#include "pwm.h"
#include <avr/io.h>

/**
 * @brief Initialize Fast PWM mode using Timer0
 */
void PWM_Init(void)
{
    /* Set PD5 (OC0B) and PD6 (OC0A) as output */
    DDRD |= (1 << PB3) | (1 << PD3);

    /*
     * Fast PWM Mode
     * Non-inverting mode
     * Prescaler = 64
     */
    TCCR2A =
        (1 << COM2A1) |
        (1 << COM2B1) |
        (1 << WGM21)  |
        (1 << WGM20);

    TCCR2B =
        (1 << CS22) |
        // (1 << CS20);

    /* Start with motors stopped */
    OCR2A = 0;
    OCR2B = 0;
}

/**
 * @brief Set left motor speed
 */
void PWM_SetLeftSpeed(uint8_t duty)
{
    OCR2B = duty;   /* PD5 */
}

/**
 * @brief Set right motor speed
 */
void PWM_SetRightSpeed(uint8_t duty)
{
    OCR2A = duty;   /* PD6 */
}