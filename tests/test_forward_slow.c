/**
 * TEST: Forward motion with front-ultrasonic speed control.
 *
 * Behavior:
 *   > SLOW_CM  (40 cm) : full speed (BASE_SPEED)
 *   STOP_CM..SLOW_CM   : linearly scales from BASE_SPEED down to HALF_SPEED
 *   <= STOP_CM (5  cm) : stop
 *
 * Front US: TRIG = PC0 (A0), ECHO = PC1 (A1)   [same as main.c]
 * UART TX:  PD1 (D1), 9600 baud  → Bluetooth module
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#include "timer.h"
#include "uart.h"
#include "motor.h"

#define FRONT_TRIG  0u   /* PC0 */
#define FRONT_ECHO  1u   /* PC1 */

#define BASE_SPEED  250u
#define SLOW_SPEED   60u   /* speed in slow zone (~38% of full) */
#define CREEP_SPEED  40u   /* speed very close to wall          */
#define SLOW_CM      70u   /* start slowing here                */
#define CREEP_CM     30u   /* creep below this distance         */
#define STOP_CM       5u

static void us_init(void) {
    DDRC |=  (1u << FRONT_TRIG);
    DDRC &= ~(1u << FRONT_ECHO);
    PORTC &= ~(1u << FRONT_TRIG);
}

static uint32_t us_now(void) {
    uint32_t ms;
    uint16_t tc;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        tc = TCNT1;
        ms = timer_get_ms();
        if ((TIFR1 & (1u << OCF1A)) && tc < 4u) ms++;
    }
    return ms * 1000UL + (uint32_t)(tc >> 1u);
}

static uint16_t us_read_once(void) {
    uint32_t t0 = timer_get_ms();
    while ((PINC & (1u << FRONT_ECHO)) && (timer_get_ms() - t0) < 10u);

    PORTC |= (1u << FRONT_TRIG);
    uint32_t t_trig = us_now();
    while ((us_now() - t_trig) < 10u);
    PORTC &= ~(1u << FRONT_TRIG);

    t0 = timer_get_ms();
    while (!(PINC & (1u << FRONT_ECHO))) {
        if ((timer_get_ms() - t0) >= 4u) return 0u;
    }

    uint32_t echo_start = us_now();
    while (PINC & (1u << FRONT_ECHO)) {
        if ((us_now() - echo_start) >= 8000u) return 0u;
    }
    uint32_t dur = us_now() - echo_start;
    return (dur == 0u) ? 0u : (uint16_t)(dur / 58u);
}

/* 3 readings with 30ms gap between pings — median filters noise spikes */
static uint16_t us_read(void) {
    uint16_t s[3], tmp;
    s[0] = us_read_once(); timer_delay_ms(30u);
    s[1] = us_read_once(); timer_delay_ms(30u);
    s[2] = us_read_once();
    if (s[0] > s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    if (s[1] > s[2]) { tmp=s[1]; s[1]=s[2]; s[2]=tmp; }
    if (s[0] > s[1]) { tmp=s[0]; s[0]=s[1]; s[1]=tmp; }
    return s[1];
}

static void print_u16(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    if (v == 0) { UART_SendChar('0'); return; }
    while (v > 0) { buf[i++] = (char)('0' + v % 10); v /= 10; }
    while (i > 0) UART_SendChar(buf[--i]);
}

int main(void) {
    timer_init();
    UART_Init(9600UL);
    us_init();
    Motor_Init();
    sei();

    UART_SendString("test_forward_slow ready\r\n");

    while (1) {
        uint16_t dist = us_read();
        if (dist == 0u) dist = 255u;

        uint8_t spd;
        const char *state;

        if (dist <= STOP_CM) {
            Motor_Stop();
            spd   = 0;
            state = "STOP";
        } else if (dist <= CREEP_CM) {
            Motor_Forward(CREEP_SPEED, CREEP_SPEED);
            spd   = CREEP_SPEED;
            state = "CREEP";
        } else if (dist >= SLOW_CM) {
            Motor_Forward(BASE_SPEED, BASE_SPEED);
            spd   = BASE_SPEED;
            state = "FULL";
        } else {
            /* linear scale: SLOW_CM→BASE_SPEED down to CREEP_CM→SLOW_SPEED */
            uint16_t range = SLOW_CM - CREEP_CM;
            uint16_t gap   = dist   - CREEP_CM;
            spd = (uint8_t)(SLOW_SPEED + ((uint16_t)(BASE_SPEED - SLOW_SPEED) * gap) / range);
            Motor_Forward(spd, spd);
            state = "SLOW";
        }

        UART_SendString("D="); print_u16(dist);
        UART_SendString(" S="); print_u16(spd);
        UART_SendString(" ["); UART_SendString(state); UART_SendString("]\r\n");
    }

    return 0;
}
