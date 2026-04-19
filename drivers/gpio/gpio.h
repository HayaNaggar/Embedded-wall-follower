/**
 * @file    gpio.h
 * @brief   GPIO driver for ATmega328P (Arduino Nano).
 *
 */

#ifndef GPIO_H
#define GPIO_H

#include <avr/io.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Pin-direction constants
 * ---------------------------------------------------------------------- */
#define GPIO_INPUT    0  /**< Configure pin as input  */
#define GPIO_OUTPUT   1  /**< Configure pin as output */

/* -----------------------------------------------------------------------
 * Pull-up control (only meaningful when pin is configured as INPUT)
 * ---------------------------------------------------------------------- */
#define GPIO_PULLUP_DISABLE  0  /**< Tri-state input (floating)      */
#define GPIO_PULLUP_ENABLE   1  /**< Enable internal pull-up resistor */

/* -----------------------------------------------------------------------
 * Digital-level constants
 * ---------------------------------------------------------------------- */
#define GPIO_LOW   0  /**<  (0 V) */
#define GPIO_HIGH  1  /**< (VCC) */

/* -----------------------------------------------------------------------
 * Port identifiers – passed to every GPIO function
 * ---------------------------------------------------------------------- */
typedef enum {
    GPIO_PORT_B = 0, /**< PORTB – digital pins D8–D13 + XTAL */
    GPIO_PORT_C,     /**< PORTC – analog pins A0–A5          */
    GPIO_PORT_D      /**< PORTD – digital pins D0–D7         */
} gpio_port_t;

/* -----------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Configure the direction of a single pin.
 *
 * @param  port       One of GPIO_PORT_B, GPIO_PORT_C, GPIO_PORT_D.
 * @param  pin        Bit position within the port (0–7).
 * @param  direction  GPIO_INPUT or GPIO_OUTPUT.
 */
void gpio_set_direction(gpio_port_t port, uint8_t pin, uint8_t direction);

/**
 * @brief  Enable or disable the internal pull-up on an input pin.
 *
 * Has no effect when the pin is configured as an output.
 *
 * @param  port    Target port.
 * @param  pin     Bit position within the port (0–7).
 * @param  pullup  GPIO_PULLUP_ENABLE or GPIO_PULLUP_DISABLE.
 */
void gpio_set_pullup(gpio_port_t port, uint8_t pin, uint8_t pullup);

/**
 * @brief  Write a logic level to an output pin.
 *
 * @param  port   Target port.
 * @param  pin    Bit position within the port (0–7).
 * @param  value  GPIO_HIGH or GPIO_LOW.
 */
void gpio_write(gpio_port_t port, uint8_t pin, uint8_t value);

/**
 * @brief  Toggle the current logic level of an output pin.
 *
 * @param  port  Target port.
 * @param  pin   Bit position within the port (0–7).
 */
void gpio_toggle(gpio_port_t port, uint8_t pin);

/**
 * @brief  Read the current logic level of a pin.
 *
 * Works for both input and output pins (reads the PINx register).
 *
 * @param  port  Target port.
 * @param  pin   Bit position within the port (0–7).
 * @return       GPIO_HIGH (1) or GPIO_LOW (0).
 */
uint8_t gpio_read(gpio_port_t port, uint8_t pin);

#endif /* GPIO_H */