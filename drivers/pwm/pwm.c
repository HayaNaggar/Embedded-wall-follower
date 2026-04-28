#include "pwm.h"
#include <avr/io.h>
#include <stdint.h>

/**
 * @brief Initialize Fast PWM mode using Timer2
 */
void PWM_Init(void)
{
    /* OC2A = PD6, OC2B = PD3 */
    DDRD |= (1 << PD3) | (1 << PD6);

    /*
     * Fast PWM mode
     * Non-inverting mode
     * Prescaler = 64
     */
    TCCR2A =
        (1 << COM2A1) |
        (1 << COM2B1) |
        (1 << WGM21)  |
        (1 << WGM20);

    TCCR2B =
        (1 << CS22);   // prescaler 64

    /* Start with motors stopped */
    OCR2A = 0;
    OCR2B = 0;
}

/**
 * @brief Set left motor speed
 */
void PWM_SetLeftSpeed(uint8_t duty)
{
    OCR2B = duty;
}

/**
 * @brief Set right motor speed
 */
void PWM_SetRightSpeed(uint8_t duty)
{
    OCR2A = duty;
}