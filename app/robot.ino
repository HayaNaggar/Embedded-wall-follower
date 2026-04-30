/**
 * @file    robot.ino
 * @brief   Wall-Following Autonomous Robot – Arduino IDE Sketch
 * 
 * This is an Arduino sketch version of the wall-follower robot.
 * It combines all the embedded C code into a format compatible with Arduino IDE.
 *
 * Hardware (ATmega328P / Arduino Nano):
 *  ┌─────────────────────────────────────────────────────┐
 *  │  Sensor            Pin           Role               │
 *  │  IR Left      →  PD3  (D3)   Front-left corner     │
 *  │  IR Right     →  PD4  (D4)   Front-right corner    │
 *  │  US Front     →  PC0/PC1     Forward obstacle       │
 *  │  US Left      →  PC2/PC3     Left wall distance     │
 *  │  US Right     →  PC4/PC5     Right wall distance    │
 *  │  Motor IN1-4  →  PB0-PB3                            │
 *  │  PWM L / R    →  PD6/PD5    (Timer0 OC0A/OC0B)     │
 *  │  UART TX/RX   →  PD1/PD0                            │
 *  └─────────────────────────────────────────────────────┘
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* Application / module headers */
#include "../drivers/sensors/IR/IR_sensor.h"
#include "../drivers/uart/uart.h"
#include "../drivers/timer/timer.h"
#include "../modules/motor/motor.h"

/* =========================================================================
 * Ultrasonic sensor pin configuration
 * ========================================================================= */
#define US_FRONT_TRIG_PIN   0   /* PC0 */
#define US_FRONT_ECHO_PIN   1   /* PC1 */

#define US_LEFT_TRIG_PIN    2   /* PC2 */
#define US_LEFT_ECHO_PIN    3   /* PC3 */

#define US_RIGHT_TRIG_PIN   4   /* PC4 */
#define US_RIGHT_ECHO_PIN   5   /* PC5 */

/* =========================================================================
 * Tunable parameters
 * ========================================================================= */
#define BASE_SPEED          160
#define TURN_SPEED_FAST     180
#define TURN_SPEED_SLOW      80
#define TURN_SPEED_SHARP    160

#define WALL_TARGET_CM       15
#define WALL_TOLERANCE_CM     4
#define FRONT_OBSTACLE_CM    20

#define TURN_DURATION_MS    550
#define WALL_LOST_CREEP_MS  300

#define MAX_TURNS           32

#define BAUD_RATE           9600UL

/* =========================================================================
 * FSM state definitions
 * ========================================================================= */
typedef enum {
    STATE_FORWARD    = 0,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_WALL_LOST,
    STATE_STOP
} robot_state_t;

/* =========================================================================
 * Turn log
 * ========================================================================= */
typedef enum { TURN_LEFT = 'L', TURN_RIGHT = 'R' } turn_dir_t;

static uint8_t    g_turn_count            = 0;
static turn_dir_t g_turn_log[MAX_TURNS];

/* =========================================================================
 * Ultrasonic helper functions
 * ========================================================================= */

static uint16_t ultrasonic_read_cm(uint8_t trig_pin, uint8_t echo_pin)
{
    uint32_t count = 0;

    /* Send 10 µs trigger pulse */
    PORTC |=  (1 << trig_pin);
    timer_delay_us(10);
    PORTC &= ~(1 << trig_pin);

    /* Wait for echo to go HIGH (with timeout) */
    count = 0;
    while (!(PINC & (1 << echo_pin))) {
        if (++count > 30000UL) return 0;
    }

    /* Measure echo HIGH duration */
    count = 0;
    while (PINC & (1 << echo_pin)) {
        count++;
        timer_delay_us(1);
        if (count > 30000UL) break;
    }

    return (uint16_t)(count / 58);
}

static void ultrasonic_init_all(void)
{
    /* TRIG pins as outputs */
    DDRC |=  (1 << US_FRONT_TRIG_PIN) |
             (1 << US_LEFT_TRIG_PIN)  |
             (1 << US_RIGHT_TRIG_PIN);

    /* ECHO pins as inputs */
    DDRC &= ~((1 << US_FRONT_ECHO_PIN) |
              (1 << US_LEFT_ECHO_PIN)  |
              (1 << US_RIGHT_ECHO_PIN));

    /* Ensure TRIG lines start LOW */
    PORTC &= ~((1 << US_FRONT_TRIG_PIN) |
               (1 << US_LEFT_TRIG_PIN)  |
               (1 << US_RIGHT_TRIG_PIN));
}

/* =========================================================================
 * UART reporting helpers
 * ========================================================================= */

static void uart_send_uint(uint16_t value)
{
    char buf[6];
    uint8_t i = 0;

    if (value == 0) {
        UART_SendChar('0');
        return;
    }

    while (value > 0) {
        buf[i++] = '0' + (value % 10);
        value   /= 10;
    }

    while (i > 0) {
        UART_SendChar(buf[--i]);
    }
}

static void report_send(void)
{
    UART_SendString("Turns: ");
    uart_send_uint(g_turn_count);
    UART_SendString("\r\nSequence: ");

    for (uint8_t i = 0; i < g_turn_count; i++) {
        UART_SendChar((char)g_turn_log[i]);
        if (i < (uint8_t)(g_turn_count - 1)) {
            UART_SendString(", ");
        }
    }
    UART_SendString("\r\n");
}

/* =========================================================================
 * Turn logging helper
 * ========================================================================= */
static void log_turn(turn_dir_t dir)
{
    if (g_turn_count < MAX_TURNS) {
        g_turn_log[g_turn_count] = dir;
    }
    g_turn_count++;
}

/* =========================================================================
 * FSM – state handlers
 * ========================================================================= */

static robot_state_t fsm_forward(void)
{
    /* --- Read all sensors --- */
    ir_snapshot_t ir = ir_read_all();

    uint16_t dist_front = ultrasonic_read_cm(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
    uint16_t dist_left  = ultrasonic_read_cm(US_LEFT_TRIG_PIN,  US_LEFT_ECHO_PIN);
    uint16_t dist_right = ultrasonic_read_cm(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

    /* Check for frontal obstacle first */
    if (dist_front > 0 && dist_front < FRONT_OBSTACLE_CM) {
        Motor_Stop();
        if (ir.right_wall) {
            return STATE_TURN_LEFT;
        } else {
            return STATE_TURN_RIGHT;
        }
    }

    /* IR: detect junction opening */
    if (!ir.left_wall && ir.right_wall) {
        Motor_Stop();
        return STATE_TURN_LEFT;
    }
    if (ir.left_wall && !ir.right_wall) {
        Motor_Stop();
        return STATE_TURN_RIGHT;
    }
    if (!ir.left_wall && !ir.right_wall) {
        Motor_Stop();
        return STATE_WALL_LOST;
    }

    /* Wall-hugging correction */
    uint8_t left_speed  = BASE_SPEED;
    uint8_t right_speed = BASE_SPEED;

    if (dist_right > 0) {
        int16_t error = (int16_t)dist_right - (int16_t)WALL_TARGET_CM;

        if (error > (int16_t)WALL_TOLERANCE_CM) {
            left_speed  = TURN_SPEED_FAST;
            right_speed = TURN_SPEED_SLOW;
        } else if (error < -(int16_t)WALL_TOLERANCE_CM) {
            left_speed  = TURN_SPEED_SLOW;
            right_speed = TURN_SPEED_FAST;
        }
    }

    if (dist_left > 0 && dist_left < (WALL_TARGET_CM - WALL_TOLERANCE_CM)) {
        left_speed  = TURN_SPEED_SLOW;
        right_speed = TURN_SPEED_FAST;
    }

    Motor_Forward(left_speed, right_speed);
    return STATE_FORWARD;
}

static robot_state_t fsm_turn_left(void)
{
    log_turn(TURN_LEFT);
    Motor_TurnLeft(TURN_SPEED_SLOW, TURN_SPEED_SHARP);
    timer_delay_ms(TURN_DURATION_MS);
    Motor_Stop();
    timer_delay_ms(50);

    return STATE_FORWARD;
}

static robot_state_t fsm_turn_right(void)
{
    log_turn(TURN_RIGHT);
    Motor_TurnRight(TURN_SPEED_SHARP, TURN_SPEED_SLOW);
    timer_delay_ms(TURN_DURATION_MS);
    Motor_Stop();
    timer_delay_ms(50);

    return STATE_FORWARD;
}

static robot_state_t fsm_wall_lost(void)
{
    uint32_t start = timer_get_ms();

    while ((timer_get_ms() - start) < WALL_LOST_CREEP_MS) {
        uint16_t dist_left  = ultrasonic_read_cm(US_LEFT_TRIG_PIN,  US_LEFT_ECHO_PIN);
        uint16_t dist_right = ultrasonic_read_cm(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

        if ((dist_left  > 0 && dist_left  < (WALL_TARGET_CM + 20)) ||
            (dist_right > 0 && dist_right < (WALL_TARGET_CM + 20))) {
            Motor_Stop();
            return STATE_FORWARD;
        }

        Motor_Forward(BASE_SPEED / 2, BASE_SPEED / 2);
    }

    Motor_Stop();
    return STATE_TURN_LEFT;
}

static robot_state_t fsm_stop(void)
{
    Motor_Stop();
    report_send();

    while (1) { /* idle */ }

    return STATE_STOP;
}

/* =========================================================================
 * System initialisation
 * ========================================================================= */
static void system_init(void)
{
    /* 1. Timer driver */
    timer_init();

    /* 2. UART @ 9600 baud */
    UART_Init(BAUD_RATE);

    /* 3. IR sensors */
    ir_sensor_init();

    /* 4. Ultrasonic sensors */
    ultrasonic_init_all();

    /* 5. Motors + PWM */
    Motor_Init();

    /* 6. Enable global interrupts */
    sei();

    /* Startup delay */
    timer_delay_ms(500);

    UART_SendString("Wall-follower ready.\r\n");
}

/* =========================================================================
 * Arduino setup() and loop() - entry point for Arduino IDE
 * ========================================================================= */

static robot_state_t g_current_state = STATE_FORWARD;

void setup(void)
{
    system_init();
}

void loop(void)
{
    switch (g_current_state) {
        case STATE_FORWARD:
            g_current_state = fsm_forward();
            break;

        case STATE_TURN_LEFT:
            g_current_state = fsm_turn_left();
            break;

        case STATE_TURN_RIGHT:
            g_current_state = fsm_turn_right();
            break;

        case STATE_WALL_LOST:
            g_current_state = fsm_wall_lost();
            break;

        case STATE_STOP:
            g_current_state = fsm_stop();
            break;

        default:
            Motor_Stop();
            g_current_state = STATE_STOP;
            break;
    }
}
