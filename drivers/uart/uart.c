#include "uart.h"
#include <avr/io.h>

/**
 * @brief Initialize UART (USART0) for ATmega328P.
 * 
 * Configures:
 * - Asynchronous mode
 * - 8 data bits
 * - 1 stop bit
 * - No parity
 * 
 * @param baud Desired baud rate (e.g., 9600)
 */
void UART_Init(uint32_t baud)
{
    uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;

    /* Set baud rate */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;

    /* Enable transmitter and receiver */
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    /* Set frame format: 8 data bits, 1 stop bit, no parity */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/**
 * @brief Send a single character over UART.
 * 
 * Waits until transmit buffer is empty, then writes data.
 * 
 * @param data Character to send
 */
void UART_SendChar(char data)
{
    /* Wait until transmit buffer is empty */
    while (!(UCSR0A & (1 << UDRE0)));

    /* Put data into buffer, sends the data */
    UDR0 = data;
}

/**
 * @brief Send a null-terminated string over UART.
 * 
 * Sends characters one by one until '\0' is reached.
 * 
 * @param str Pointer to string
 */
void UART_SendString(const char* str)
{
    while (*str)
    {
        UART_SendChar(*str++);
    }
}

/**
 * @brief Receive a single character (blocking).
 * 
 * Waits until data is received.
 * 
 * @return Received character
 */
char UART_ReceiveChar(void)
{
    /* Wait for data to be received */
    while (!(UCSR0A & (1 << RXC0)));

    /* Get and return received data */
    return UDR0;
}