/**
 * @file    timer.c
 * @brief   Timer driver implementation for ATmega328P
 *
 * Timer1 (16-bit) — CTC mode, prescaler 8, OCR1A = 1999
 * Fires ISR every 1 ms at F_CPU = 16 MHz: (16 000 000 / 8 / 1000) - 1 = 1999
 * Also used for timer_get_us() via TCNT1 readout.
 */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define TIMER1_CTC_TOP    1999U
#define TIMER1_TICKS_PER_US 2U

static volatile uint32_t g_ms_ticks = 0;

ISR(TIMER1_COMPA_vect) { g_ms_ticks++; }

/** @brief Initialize Timer1 for 1 ms CTC tick. */
void timer_init(void)
{
    TCCR1B = (1 << WGM12);
    OCR1A  = TIMER1_CTC_TOP;
    TIMSK1 = (1 << OCIE1A);
    TCCR1B |= (1 << CS11);
    TCCR1A = 0x00;
}

/** @brief Return elapsed milliseconds since timer_init(). */
uint32_t timer_get_ms(void)
{
    uint32_t ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ms = g_ms_ticks; }
    return ms;
}

/** @brief Blocking millisecond delay. */
void timer_delay_ms(uint32_t ms)
{
    uint32_t start = timer_get_ms();
    while ((timer_get_ms() - start) < ms)
        ;
}

/**
 * @brief  Return elapsed microseconds since timer_init().
 *
 * Combines g_ms_ticks with TCNT1 (prescaler 8, 16 MHz → 0.5 µs/tick).
 * Handles the CTC-reset boundary: if OCF1A is set but the ISR has not
 * yet run, ms is incremented locally so the result stays monotonic.
 */
uint32_t timer_get_us(void)
{
    uint32_t ms;
    uint16_t tc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tc = TCNT1;
        ms = g_ms_ticks;
        if ((TIFR1 & (1u << OCF1A)) && tc < 4u) ms++;
    }
    return ms * 1000UL + (uint32_t)(tc >> 1u);
}

/** @brief Blocking microsecond delay using Timer1. */
void timer_delay_us(uint16_t us)
{
    uint16_t start = TCNT1;
    uint16_t ticks = (uint16_t)(us * TIMER1_TICKS_PER_US);
    while ((uint16_t)(TCNT1 - start) < ticks)
        ;
}
