#include "ultrasonic.h"
#include <avr/io.h>
#include <util/delay.h>

// Configuration: Adjust based on your specific wiring
#define TRIG_PIN PB1 // Digital 9
#define ECHO_PIN PB0 // Digital 8

void Ultrasonic_Init(void) {
    // Set TRIG_PIN as output
    DDRB |= (1 << TRIG_PIN);
    // Set ECHO_PIN as input
    DDRB &= ~(1 << ECHO_PIN);
    // Ensure TRIG is low initially
    PORTB &= ~(1 << TRIG_PIN);
}

uint16_t Ultrasonic_ReadDistance(void) {
    uint32_t count = 0;
    uint16_t distance = 0;

    // 1. Send 10us High pulse to Trigger pin
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    // 2. Wait for Echo to go HIGH (with timeout)
    while (!(PINB & (1 << ECHO_PIN))) {
        if (count++ > 100000)
            return 0;
    }

    // 3. Measure duration Echo is HIGH
    count = 0;
    while (PINB & (1 << ECHO_PIN)) {
        count++;
        _delay_us(1);
        if (count > 30000)
            break; // Max range timeout (~5 meters)
    }

    /**
     * @formula: Distance = (Time * Speed of Sound) / 2
     * Speed of sound is ~343 m/s or 0.0343 cm/us.
     * distance = (count * 0.0343) / 2 -> count / 58
     */
    distance = count / 58;

    return distance;
}