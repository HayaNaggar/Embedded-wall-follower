/**
 * @file    timer.c
 * @brief   Timer driver implementation for ATmega328P
 *
 * Timer0  -> CTC mode, prescaler 64, OCR0A = 249
 *            Fires ISR every 1ms at F_CPU = 16MHz
 *            Formula: (F_CPU / prescaler / target_freq) - 1
 *                     (16,000,000 / 64 / 1000) - 1 = 249
 *
 * Timer1  -> Normal mode, prescaler 8
 *            1 tick = 0.5 us at 16MHz
 *            Used for microsecond and millisecond blocking delays
 */

#include "timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>   /* ATOMIC_BLOCK for safe 32-bit read */

/* -----------------------------------------------------------------------
 * Private defines
 * ----------------------------------------------------------------------- */

/** @brief F_CPU must be defined; default to 16MHz if missing */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/** @brief Timer0 CTC top value for 1ms tick (prescaler=64, F_CPU=16MHz) */
#define TIMER0_CTC_TOP      249U

/** @brief Timer1 prescaler = 8 -> 1 tick = 0.5us at 16MHz */
#define TIMER1_TICKS_PER_US 2U      /* 2 ticks = 1 us */

/* -----------------------------------------------------------------------
 * Private variables
 * ----------------------------------------------------------------------- */

/** @brief Millisecond counter incremented by Timer0 ISR every 1ms */
static volatile uint32_t g_ms_ticks = 0;

/* -----------------------------------------------------------------------
 * Timer0 ISR — fires every 1ms
 * ----------------------------------------------------------------------- */

/**
 * @brief  Timer0 Compare Match A ISR.
 *         Increments the global millisecond counter.
 */
ISR(TIMER0_COMPA_vect)
{
    g_ms_ticks++;
}

/* -----------------------------------------------------------------------
 * Public function implementations
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialize Timer0 (1ms tick) and Timer1 (us delay).
 */
void timer_init(void)
{
    /* --- Timer0: CTC mode, 1ms tick --- */

    /** @brief Set CTC mode: WGM01 = 1, WGM00 = 0 */
    TCCR0A = (1 << WGM01);

    /** @brief Set TOP value for 1ms period */
    OCR0A  = TIMER0_CTC_TOP;

    /** @brief Enable Timer0 Compare Match A interrupt */
    TIMSK0 = (1 << OCIE0A);

    /** @brief Start Timer0 with prescaler = 64 (CS01 | CS00) */
    TCCR0B = (1 << CS01) | (1 << CS00);


    /* --- Timer1: Normal mode, prescaler 8, used for us delays --- */

    /** @brief Normal mode (all WGM bits = 0, default after reset) */
    TCCR1A = 0x00;

    /** @brief Prescaler = 8 (CS11 = 1) — gives 0.5us per tick at 16MHz */
    TCCR1B = (1 << CS11);

    /** @brief Disable all Timer1 interrupts; we use busy-wait only */
    TIMSK1 = 0x00;
}

/**
 * @brief  Return elapsed milliseconds since timer_init().
 * @return 32-bit ms count.
 */
uint32_t timer_get_ms(void)
{
    uint32_t ms;

    /** @brief Atomic read: prevents corrupted value if ISR fires mid-read */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ms = g_ms_ticks;
    }

    return ms;
}

/**
 * @brief  Blocking delay using the ms tick counter.
 * @param  ms  Milliseconds to wait.
 */
void timer_delay_ms(uint32_t ms)
{
    /** @brief Record start time then spin until elapsed time >= ms */
    uint32_t start = timer_get_ms();

    while ((timer_get_ms() - start) < ms)
    {
        /* busy wait — yields to ISR naturally */
    }
}

/**
 * @brief  Blocking microsecond delay using Timer1.
 * @param  us  Microseconds to wait.
 *
 * Strategy:
 *  - Save current TCNT1 value as start
 *  - Calculate target tick count  = us * TIMER1_TICKS_PER_US
 *  - Spin until TCNT1 has advanced by that many ticks
 *  - Handles 16-bit overflow correctly via unsigned subtraction
 */
void timer_delay_us(uint16_t us)
{
    /** @brief Snapshot the current Timer1 counter */
    uint16_t start = TCNT1;

    /** @brief Calculate how many Timer1 ticks correspond to requested us */
    uint16_t ticks = (uint16_t)(us * TIMER1_TICKS_PER_US);

    /** @brief Spin until Timer1 has advanced by the required ticks.
     *         Unsigned subtraction handles 16-bit wraparound correctly. */
    while ((uint16_t)(TCNT1 - start) < ticks)
    {
        /* busy wait */
    }
}