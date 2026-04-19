/**
 * @file  gpio.c
 * @brief Low-level GPIO driver implementation for ATmega328P.
 *
 * All operations are single-cycle register writes/reads with no blocking
 * delays, making this safe to call from an FSM tick function.
 */

#include "gpio.h"

/* -----------------------------------------------------------------------
 * Internal helpers – resolve port → register pointers at runtime.
 * Using volatile uint8_t pointers keeps the code portable across all
 * three ATmega328P ports without code duplication.
 * ---------------------------------------------------------------------- */

/** @brief Return pointer to the DDRx register for the given port. */
static volatile uint8_t *_ddr(gpio_port_t port)
{
    switch (port) {
        case GPIO_PORT_B: return &DDRB;
        case GPIO_PORT_C: return &DDRC;
        case GPIO_PORT_D: return &DDRD;
        default:          return &DDRD;   /* safe fallback */
    }
}

/** @brief Return pointer to the PORTx register for the given port. */
static volatile uint8_t *_port(gpio_port_t port)
{
    switch (port) {
        case GPIO_PORT_B: return &PORTB;
        case GPIO_PORT_C: return &PORTC;
        case GPIO_PORT_D: return &PORTD;
        default:          return &PORTD;
    }
}

/** @brief Return pointer to the PINx register for the given port. */
static volatile uint8_t *_pin(gpio_port_t port)
{
    switch (port) {
        case GPIO_PORT_B: return &PINB;
        case GPIO_PORT_C: return &PINC;
        case GPIO_PORT_D: return &PIND;
        default:          return &PIND;
    }
}

/* -----------------------------------------------------------------------
 * Public API implementations
 * ---------------------------------------------------------------------- */

void gpio_set_direction(gpio_port_t port, uint8_t pin, uint8_t direction)
{
    volatile uint8_t *ddr = _ddr(port);

    if (direction == GPIO_OUTPUT) {
        *ddr |= (1 << pin);   /* set bit   → output */
    } else {
        *ddr &= ~(1 << pin);  /* clear bit → input  */
    }
}

void gpio_set_pullup(gpio_port_t port, uint8_t pin, uint8_t pullup)
{
    volatile uint8_t *port_reg = _port(port);

    if (pullup == GPIO_PULLUP_ENABLE) {
        *port_reg |= (1 << pin);
    } else {
        *port_reg &= ~(1 << pin);
    }
}

void gpio_write(gpio_port_t port, uint8_t pin, uint8_t value)
{
    volatile uint8_t *port_reg = _port(port);

    if (value == GPIO_HIGH) {
        *port_reg |= (1 << pin);
    } else {
        *port_reg &= ~(1 << pin);
    }
}

void gpio_toggle(gpio_port_t port, uint8_t pin)
{
    /*
     * Writing a 1 to PINx toggles the corresponding PORTx bit atomically
     * (AVR hardware feature – no read-modify-write needed).
     */
    *_pin(port) = (1 << pin);
}

uint8_t gpio_read(gpio_port_t port, uint8_t pin)
{
    return (*_pin(port) & (1 << pin)) ? GPIO_HIGH : GPIO_LOW;
}