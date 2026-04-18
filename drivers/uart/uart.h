#ifndef UART_H_
#define UART_H_

#include <stdint.h>

/**
 * @brief Initialize UART with desired baud rate.
 * 
 * @param baud Baud rate (e.g., 9600, 115200)
 */
void UART_Init(uint32_t baud);

/**
 * @brief Send a single character over UART.
 * 
 * @param data Character to send
 */
void UART_SendChar(char data);

/**
 * @brief Send a null-terminated string over UART.
 * 
 * @param str Pointer to string
 */
void UART_SendString(const char* str);

/**
 * @brief Receive a single character (blocking).
 * 
 * @return Received character
 */
char UART_ReceiveChar(void);

#endif /* UART_H_ */