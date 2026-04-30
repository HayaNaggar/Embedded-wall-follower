#include "pwm.h"
#include <avr/io.h>
#include <stdint.h>

/*
 * PWM uses Timer0 Fast PWM mode.
 *   OC0A -> PD6 (D6) -> PWM 2 (right motor)
 *   OC0B -> PD5 (D5) -> PWM 1 (left motor)
 *
 * Timer0 is shared with the 1ms millis counter which has been moved
 * to Timer2 in timer.c — do not reinitialise Timer0 elsewhere.
 */
void PWM_Init(void)
{
    /* D5=PD5=OC0B, D6=PD6=OC0A as outputs */
    DDRD |= (1 << PD5) | (1 << PD6);

    /* Fast PWM, non-inverting on both channels, prescaler 64 */
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01)   | (1 << CS00);

    OCR0A = 0;
    OCR0B = 0;
}

void PWM_SetLeftSpeed(uint8_t duty)
{
    OCR0B = duty;
}

void PWM_SetRightSpeed(uint8_t duty)
{
    OCR0A = duty;
}