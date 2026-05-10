/**
 * @file    uart.c
 * @brief   USART0 driver for ATmega328P (8N1, polling).
 */

#include "uart.h"
#include <avr/io.h>

/**
 * @brief  Initialize USART0.
 * @param  baud  Desired baud rate (e.g., 9600).
 */
void UART_Init(uint32_t baud)
{
    uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/**
 * @brief  Send a single character over UART (blocking).
 * @param  data  Character to transmit.
 */
void UART_SendChar(char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

/**
 * @brief  Send a null-terminated string over UART.
 * @param  str  Pointer to string.
 */
void UART_SendString(const char *str)
{
    while (*str)
        UART_SendChar(*str++);
}

/**
 * @brief  Receive a single character (blocking).
 * @return Received character.
 */
char UART_ReceiveChar(void)
{
    while (!(UCSR0A & (1 << RXC0)))
        ;
    return UDR0;
}
