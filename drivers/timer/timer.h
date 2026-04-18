/**
 * @file    timer.h
 * @brief   Timer driver interface for ATmega328P
 *
 * Provides:
 *  - Non-blocking millisecond tick counter via Timer0 CTC + ISR
 *  - Blocking delay functions (ms and us) via Timer1
 *
 * Timer allocation:
 *  - Timer0 (8-bit)  : CTC mode, 1ms tick ISR  -> millis counter
 *  - Timer1 (16-bit) : Input Capture / one-shot -> delay_us, delay_ms
 *
 * @note    Call timer_init() once before using any other function.
 *          sei() (global interrupt enable) must be called after timer_init().
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialize Timer0 for 1ms CTC tick and Timer1 for blocking delays.
 *         Must be called once at startup before sei().
 */
void timer_init(void);

/**
 * @brief  Return elapsed milliseconds since timer_init() was called.
 * @return 32-bit millisecond count (rolls over after ~49 days).
 * @note   Safe to call from main loop; value is updated by Timer0 ISR.
 */
uint32_t timer_get_ms(void);

/**
 * @brief  Blocking delay in milliseconds using the tick counter.
 * @param  ms  Number of milliseconds to wait.
 */
void timer_delay_ms(uint32_t ms);

/**
 * @brief  Blocking delay in microseconds using Timer1 busy-wait.
 * @param  us  Number of microseconds to wait (minimum reliable value: 2 us).
 * @note   Uses a tight Timer1 loop; interrupts are NOT disabled during this call.
 */
void timer_delay_us(uint16_t us);

#endif /* TIMER_H */