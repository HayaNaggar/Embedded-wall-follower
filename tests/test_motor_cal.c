/**
 * TEST: Motor + encoder calibration — forward, turn left, forward, turn right, repeat.
 *
 * No IR sensors. No front ultrasonic. Just straight driving and pivots.
 * Place the robot on the floor with space, watch it trace an S/square path.
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │  WHAT EACH PARAMETER CHANGES IN REAL LIFE                        │
 * │                                                                  │
 * │  TURN_90_TICKS   Total encoder ticks (left + right absolute)     │
 * │                  to stop the pivot. Increase → more rotation.   │
 * │                  Decrease → less rotation. Target: 90°.         │
 * │                  Current estimate: 100 ticks ≈ 20°              │
 * │                  so 90° ≈ 450 ticks. Tune from there.           │
 * │                                                                  │
 * │  TURN_OUTER      PWM speed of the OUTER wheel during a pivot     │
 * │                  (0–255). Higher = faster turn overall.          │
 * │                  Too high = wheels slip → ticks are unreliable.  │
 * │                                                                  │
 * │  TURN_INNER      PWM speed of the INNER wheel during a pivot     │
 * │                  (0–255). Lower = tighter/sharper pivot.         │
 * │                  Set to 0 for a spin-on-axis pivot.              │
 * │                  Too low = inner wheel stalls → encoder only     │
 * │                  counts outer wheel → need more ticks.           │
 * │                                                                  │
 * │  FORWARD_SPEED   Speed during straight sections (0–255).         │
 * │                  Higher = faster straight line.                  │
 * │                                                                  │
 * │  STRAIGHT_MS     Milliseconds to drive forward before each turn. │
 * │                  Increase to travel farther between turns.       │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * UART output (9600 baud, Bluetooth):
 *   FWD L           — driving forward before left turn
 *   Pivot LEFT      — turning left
 *   LEFT  ticks=XX time=XXms   — result for calibration
 *   FWD R           — driving forward before right turn
 *   Pivot RIGHT     — turning right
 *   RIGHT ticks=XX time=XXms
 *
 * PIN MAP
 *   Enc A LEFT  = D7  = PD7  (PCINT23)
 *   Enc B LEFT  = D2  = PD2
 *   Enc A RIGHT = D12 = PB4  (PCINT4)
 *   Enc B RIGHT = D13 = PB5
 *   Motor IN1–IN4 = D8–D11 (PB0–PB3)
 *   ENA = D5, ENB = D6
 *
 * NOTE: Motor_TurnRight → robot physically turns LEFT.
 *       Motor_TurnLeft  → robot physically turns RIGHT.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#include "timer.h"
#include "uart.h"
#include "motor.h"

/* =========================================================================
 * TUNE THESE
 * ========================================================================= */
#define TURN_90_TICKS_L 400u    /* ticks for LEFT  90° (tune: left was 49° at 450) */
#define TURN_90_TICKS_R 200u    /* ticks for RIGHT 90° (tune: right was 135° at 450) */
#define TURN_OUTER      230u    /* outer-wheel PWM  — higher = faster turn    */
#define TURN_INNER      110u    /* inner-wheel PWM  — lower  = sharper pivot  */
#define FORWARD_SPEED   150u    /* straight-line speed                        */
#define STRAIGHT_MS    1500u    /* ms to drive forward between turns          */
#define TURN_TIMEOUT_MS 2000u   /* safety: abort if turn takes too long       */

/* =========================================================================
 * Encoder
 * ========================================================================= */
#define ENC_L_A_BIT  7u   /* D7  — LEFT  wheel */
#define ENC_L_B_BIT  2u
#define ENC_R_A_BIT  4u   /* D12 — RIGHT wheel */
#define ENC_R_B_BIT  5u

static volatile int32_t g_enc_left  = 0;
static volatile int32_t g_enc_right = 0;
static volatile uint8_t g_prev_pind = 0u;
static volatile uint8_t g_prev_pinb = 0u;

ISR(PCINT2_vect) {
    uint8_t cur = PIND, changed = cur ^ g_prev_pind;
    g_prev_pind = cur;
    if (changed & (1u << ENC_L_A_BIT))
        if (cur & (1u << ENC_L_A_BIT))
            g_enc_left += ((cur >> ENC_L_B_BIT) & 1u) ? 1 : -1;
}

ISR(PCINT0_vect) {
    uint8_t cur = PINB, changed = cur ^ g_prev_pinb;
    g_prev_pinb = cur;
    if (changed & (1u << ENC_R_A_BIT))
        if (cur & (1u << ENC_R_A_BIT))
            g_enc_right += ((cur >> ENC_R_B_BIT) & 1u) ? -1 : 1;
}

static void encoder_init(void) {
    DDRD  &= ~((1u<<ENC_L_A_BIT)|(1u<<ENC_L_B_BIT));
    PORTD |=   (1u<<ENC_L_A_BIT)|(1u<<ENC_L_B_BIT);
    DDRB  &= ~((1u<<ENC_R_A_BIT)|(1u<<ENC_R_B_BIT));
    PORTB |=   (1u<<ENC_R_A_BIT)|(1u<<ENC_R_B_BIT);
    PCMSK2 |= (1u << PCINT23);
    PCMSK0 |= (1u << PCINT4);
    PCICR  |= (1u << PCIE2) | (1u << PCIE0);
    g_prev_pind = PIND;
    g_prev_pinb = PINB;
}

static void enc_reset(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_enc_left = 0; g_enc_right = 0; }
}

/* returns |left| + |right| ticks */
static int32_t enc_total_abs(void) {
    int32_t l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { l = g_enc_left; r = g_enc_right; }
    return (l < 0 ? -l : l) + (r < 0 ? -r : r);
}

/* returns individual wheel ticks for diagnosis */
static void enc_read(int32_t *l_out, int32_t *r_out) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { *l_out = g_enc_left; *r_out = g_enc_right; }
}

/* =========================================================================
 * UART helper
 * ========================================================================= */
static void print_i32(int32_t v) {
    char buf[12]; uint8_t i = 0;
    if (v < 0) { UART_SendChar('-'); v = -v; }
    if (v == 0) { UART_SendChar('0'); return; }
    while (v > 0) { buf[i++] = (char)('0' + v % 10); v /= 10; }
    while (i > 0) UART_SendChar(buf[--i]);
}

/* =========================================================================
 * Pivot helper — runs until ticks or timeout, logs result
 * dir: 0 = LEFT (Motor_TurnRight), 1 = RIGHT (Motor_TurnLeft)
 * ========================================================================= */
static void do_pivot(uint8_t dir) {
    enc_reset();
    uint32_t t_start  = timer_get_ms();
    int32_t  tgt      = (dir == 0u) ? (int32_t)TURN_90_TICKS_L
                                    : (int32_t)TURN_90_TICKS_R;

    if (dir == 0u) {
        UART_SendString("Pivot LEFT\r\n");
        Motor_TurnRight(TURN_OUTER, TURN_INNER);   /* physically LEFT */
    } else {
        UART_SendString("Pivot RIGHT\r\n");
        Motor_TurnLeft(TURN_INNER, TURN_OUTER);    /* physically RIGHT */
    }

    while (1) {
        int32_t  ticks   = enc_total_abs();
        uint32_t elapsed = timer_get_ms() - t_start;

        if (ticks >= tgt || elapsed >= TURN_TIMEOUT_MS) {
            Motor_Stop();

            int32_t el, er;
            enc_read(&el, &er);

            if (dir == 0u) UART_SendString("LEFT  ticks=");
            else           UART_SendString("RIGHT ticks=");
            print_i32(ticks);
            UART_SendString(" time="); print_i32((int32_t)elapsed); UART_SendString("ms");
            UART_SendString("  L="); print_i32(el < 0 ? -el : el);
            UART_SendString(" R=");  print_i32(er < 0 ? -er : er);
            if (elapsed >= TURN_TIMEOUT_MS && ticks < tgt)
                UART_SendString(" [TIMEOUT — increase TURN_OUTER or check encoders]");
            UART_SendString("\r\n");
            break;
        }
    }

    /* pause so you can see where the robot stopped */
    timer_delay_ms(800u);
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void) {
    timer_init();
    UART_Init(9600UL);
    encoder_init();
    Motor_Init();
    sei();

    UART_SendString("test_motor_cal ready\r\n");
    UART_SendString("TICKS_L="); print_i32((int32_t)TURN_90_TICKS_L);
    UART_SendString(" TICKS_R="); print_i32((int32_t)TURN_90_TICKS_R);
    UART_SendString(" OUTER=");  print_i32((int32_t)TURN_OUTER);
    UART_SendString(" INNER=");  print_i32((int32_t)TURN_INNER);
    UART_SendString("\r\n");

    while (1) {
        /* ── Forward before LEFT turn ─────────────────────────────── */
        UART_SendString("FWD L\r\n");
        Motor_Forward(FORWARD_SPEED, FORWARD_SPEED);
        timer_delay_ms(STRAIGHT_MS);
        Motor_Stop();
        timer_delay_ms(300u);

        /* ── LEFT pivot ───────────────────────────────────────────── */
        do_pivot(0u);

        /* ── Forward before RIGHT turn ────────────────────────────── */
        UART_SendString("FWD R\r\n");
        Motor_Forward(FORWARD_SPEED, FORWARD_SPEED);
        timer_delay_ms(STRAIGHT_MS);
        Motor_Stop();
        timer_delay_ms(300u);

        /* ── RIGHT pivot ──────────────────────────────────────────── */
        do_pivot(1u);
    }

    return 0;
}
