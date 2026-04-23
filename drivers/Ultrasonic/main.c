#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>

/*

This code assume that we use the other driver uart.h (To send data to serial
monitor)


*/

// INCLUDES
extern "C" {
#include "Ultra.h"
#include "uart.h"
}

#define LED_PIN PB5

/**
 * @brief Helper to send a number followed by a unit and newline
 */
void Send_Distance_Data(uint16_t dist) {
    char buffer[7];

    UART_SendString("Distance: ");

    if (dist == 0) {
        UART_SendString("Out of Range\r\n");
    } else {
        utoa(dist, buffer, 10); // Unsigned int to string (Base 10)
        UART_SendString(buffer);
        UART_SendString(" cm\r\n");
    }
}

int main(void) {
    // 1. Initialize Drivers
    UART_Init(9600);
    Ultrasonic_Init();

    // 2. Set LED Pin as Output
    DDRB |= (1 << LED_PIN);

    UART_SendString("--- Ultrasonic System Booted ---\r\n");

    uint16_t current_dist = 0;

    while (1) {
        // 3. Get measurement
        current_dist = Ultrasonic_ReadDistance();

        // 4. Report to Serial Monitor
        Send_Distance_Data(current_dist);

        // 5. LED Control Logic
        // If an object is closer than 15cm, turn LED ON
        if (current_dist > 0 && current_dist < 15) {
            PORTB |= (1 << LED_PIN);
        } else {
            PORTB &= ~(1 << LED_PIN);
        }

        // 6. Wait 100ms before next pulse (Sensor needs recovery time)
        _delay_ms(100);
    }

    return 0;
}