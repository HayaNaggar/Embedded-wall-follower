#include "pwm.h"
#include <avr/io.h>
#include <stdint.h>

/**
 * @brief Initialize Fast PWM mode using Timer0
 */
void PWM_Init(void) {
    /* OC0A = PD6, OC0B = PD5 */
    DDRD |= (1 << PD5) | (1 << PD6);

    /*
     * Fast PWM mode
     * Non-inverting mode
     * Prescaler = 64
     */
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);

    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64

    /* Start with motors stopped */
    OCR0A = 0;
    OCR0B = 0;
}

/**
 * @brief Set left motor speed
 */
void PWM_SetLeftSpeed(uint8_t duty) { OCR0B = duty; }

/**
 * @brief Set right motor speed
 */
void PWM_SetRightSpeed(uint8_t duty) { OCR0A = duty; }