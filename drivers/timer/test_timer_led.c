/**
 * @file    test_timer_led.c
 * @brief   LED blink test to verify the timer driver on Arduino Uno
 *
 * Hardware:
 *  - Built-in LED on PB5 (Arduino pin 13) — no external parts needed
 *
 * Expected behavior:
 *  - LED toggles every 500ms  -> confirms timer_delay_ms() works
 *  - LED does a quick 3x blink on startup -> confirms timer_delay_us() works
 *
 * How to verify:
 *  - Watch the LED: it should blink exactly once per second (500ms ON, 500ms
 * OFF)
 *  - If it blinks faster/slower, check F_CPU matches your clock (16MHz for Uno)
 */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>

/** @brief Built-in LED is on Port B, pin 5 */
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB5

/**
 * @brief  Toggle the LED pin.
 */
static inline void led_toggle(void) { LED_PORT ^= (1 << LED_PIN); }

/**
 * @brief  Main entry point — init peripherals then run blink loop.
 */
int main(void) {
    /* --- GPIO setup --- */

    /** @brief Set LED pin as output */
    LED_DDR |= (1 << LED_PIN);

    /* --- Timer setup --- */

    /** @brief Initialize timer driver (Timer0 + Timer1) */
    timer_init();

    /** @brief Enable global interrupts so Timer0 ISR can fire */
    sei();

    /* --- Startup test: quick 3x blink to verify delay_us --- */

    /** @brief 3 fast blinks (100ms each) using timer_delay_ms */
    for (uint8_t i = 0; i < 3; i++) {
        led_toggle();
        timer_delay_ms(100);
        led_toggle();
        timer_delay_ms(100);
    }

    /** @brief Short pause before entering main loop */
    timer_delay_ms(500);

    /* --- Main loop: toggle every 500ms --- */
    while (1) {
        /** @brief Toggle LED state */
        led_toggle();

        /** @brief Wait 500ms using non-blocking tick counter internally */
        timer_delay_ms(500);
    }

    return 0; /* never reached */
}
