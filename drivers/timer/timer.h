/**
 * @file    timer.h
 * @brief   Timer driver interface for ATmega328P
 *
 * Timer allocation:
 *  - Timer0 (8-bit)  : Fast PWM for motor control (OC0A / OC0B)
 *  - Timer1 (16-bit) : 1 ms CTC tick ISR + timer_get_us() via TCNT1
 *  - Timer2          : unused
 *
 * Call timer_init() once at startup before sei().
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/** @brief Initialize Timer1 for 1 ms CTC tick. Must be called before sei(). */
void timer_init(void);

/** @brief Return elapsed milliseconds since timer_init() (rolls over after ~49 days). */
uint32_t timer_get_ms(void);

/** @brief Blocking delay in milliseconds. */
void timer_delay_ms(uint32_t ms);

/**
 * @brief  Blocking delay in microseconds using Timer1 busy-wait.
 * @note   Minimum reliable value: 2 µs. Interrupts are not disabled.
 */
void timer_delay_us(uint16_t us);

/**
 * @brief  Return elapsed microseconds since timer_init().
 *
 * Combines the 1 ms tick counter with TCNT1 for ~0.5 µs resolution.
 * Safe to call from main loop and from atomic sections.
 */
uint32_t timer_get_us(void);

#endif /* TIMER_H */
