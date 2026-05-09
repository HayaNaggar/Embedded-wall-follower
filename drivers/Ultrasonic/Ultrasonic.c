#include "Ultrasonic.h"
#include "timer.h"
#include <avr/io.h>
#include <util/atomic.h>

/*
 * Microsecond timestamp: combines the 1ms project counter with TCNT1.
 * Timer1 prescaler=8, F_CPU=16MHz -> 2 ticks/us, period=2000 ticks=1ms.
 * Resolution ~0.5 us. Handles the CTC-reset boundary: if the compare
 * flag is already set but the ISR hasn't run yet, the ms count is
 * adjusted so the result is always monotonic.
 */
static uint32_t us_now(void)
{
    uint32_t ms;
    uint16_t tc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tc = TCNT1;
        ms = timer_get_ms();
        if ((TIFR1 & (1u << OCF1A)) && tc < 4u) ms++;
    }
    return ms * 1000UL + (uint32_t)(tc >> 1u);
}

void Ultrasonic_Init(uint8_t trig_mask, uint8_t echo_mask)
{
    DDRC |=   trig_mask;
    DDRC &=  ~echo_mask;
    PORTC &= ~trig_mask;
}

uint16_t Ultrasonic_ReadOnce(uint8_t trig, uint8_t echo)
{
    /* Wait for any lingering echo to clear (up to 10 ms) */
    uint32_t t0 = timer_get_ms();
    while ((PINC & (1u << echo)) && (timer_get_ms() - t0) < 10u)
        ;

    /* 10 us trigger pulse measured by us_now() — no _delay_us */
    PORTC |= (1u << trig);
    uint32_t t_trig = us_now();
    while ((us_now() - t_trig) < 10u)
        ;
    PORTC &= ~(1u << trig);

    /* Wait for echo HIGH (4 ms timeout) */
    t0 = timer_get_ms();
    while (!(PINC & (1u << echo))) {
        if ((timer_get_ms() - t0) >= 4u) return 0u;
    }

    /* Measure echo HIGH duration */
    uint32_t echo_start = us_now();
    while (PINC & (1u << echo)) {
        if ((us_now() - echo_start) >= 8000u) return 0u;
    }
    uint32_t duration_us = us_now() - echo_start;

    if (duration_us == 0u) return 0u;
    return (uint16_t)(duration_us / 58u);
}

uint16_t Ultrasonic_ReadMedian3(uint8_t trig, uint8_t echo)
{
    uint16_t s[3], tmp;
    s[0] = Ultrasonic_ReadOnce(trig, echo);
    s[1] = Ultrasonic_ReadOnce(trig, echo);
    s[2] = Ultrasonic_ReadOnce(trig, echo);

    if (s[0] > s[1]) { tmp = s[0]; s[0] = s[1]; s[1] = tmp; }
    if (s[1] > s[2]) { tmp = s[1]; s[1] = s[2]; s[2] = tmp; }
    if (s[0] > s[1]) { tmp = s[0]; s[0] = s[1]; s[1] = tmp; }
    return s[1];
}
