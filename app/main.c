/**
 * @file    main.c
 * @brief   Wall-Following Autonomous Robot – Application Entry Point
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
 *
 * FSM States:
 *   FORWARD       – drive straight, hugging the right wall
 *   TURN_LEFT     – executing a 90° left turn
 *   TURN_RIGHT    – executing a 90° right turn
 *   WALL_LOST     – neither IR sees a wall; creep and re-acquire
 *   STOP          – track complete or manual stop
 *
 * Turn detection logic (IR sensors – front-corner placement):
 *   left_wall=1, right_wall=1  → corridor, keep going straight
 *   left_wall=0, right_wall=1  → left opening → LEFT turn
 *   left_wall=1, right_wall=0  → right opening → RIGHT turn
 *   left_wall=0, right_wall=0  → WALL_LOST
 *
 * Ultrasonic sensors are used to:
 *   • Keep side-wall distances within target range (wall hugging)
 *   • Detect a frontal obstacle that forces a turn
 *
 * Communication:
 *   After the track is complete (STOP state) the robot sends via UART:
 *     "Turns: N\r\nSequence: X, X, ...\r\n"
 *
 * Coding rules:
 *   • Register-level drivers only – no Arduino APIs
 *   • Non-blocking design: all timing uses timer_get_ms()
 *   • Blocking delays only inside initialisation and turn manoeuvres
 */

/* =========================================================================
 * Includes
 * ========================================================================= */
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
 *
 * Three HC-SR04 sensors wired on PORTC:
 *   Front  : TRIG = PC0, ECHO = PC1
 *   Left   : TRIG = PC2, ECHO = PC3
 *   Right  : TRIG = PC4, ECHO = PC5
 * ========================================================================= */
#define US_FRONT_TRIG_PIN   0   /* PC0 */
#define US_FRONT_ECHO_PIN   1   /* PC1 */

#define US_LEFT_TRIG_PIN    2   /* PC2 */
#define US_LEFT_ECHO_PIN    3   /* PC3 */

#define US_RIGHT_TRIG_PIN   4   /* PC4 */
#define US_RIGHT_ECHO_PIN   5   /* PC5 */

/* =========================================================================
 * Tunable parameters  (adjust during testing)
 * ========================================================================= */
#define BASE_SPEED          160     /* Normal forward PWM duty (0-255)       */
#define TURN_SPEED_FAST     180     /* Outer wheel during correction          */
#define TURN_SPEED_SLOW      80     /* Inner wheel during correction          */
#define TURN_SPEED_SHARP    160     /* Both wheels during a 90° pivot         */

#define WALL_TARGET_CM       15     /* Desired side-wall distance (cm)        */
#define WALL_TOLERANCE_CM     4     /* ±4 cm dead-band before correcting      */
#define FRONT_OBSTACLE_CM    20     /* Stop/turn when front US < this (cm)    */

#define TURN_DURATION_MS    550     /* Time to execute one 90° turn (ms)      */
                                    /* Tune on actual hardware                */
#define WALL_LOST_CREEP_MS  300     /* Slow creep while searching for walls   */

#define MAX_TURNS           32      /* Maximum turns to log                   */

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
 * Ultrasonic helper functions  (inline sensor read, no extra module needed)
 * ========================================================================= */

/**
 * @brief  Read distance (cm) from one HC-SR04 on PORTC.
 *
 * @param  trig_pin  Bit number of TRIG pin in PORTC (0-5)
 * @param  echo_pin  Bit number of ECHO pin in PORTC (0-5)
 * @return Distance in cm, or 0 on timeout.
 */
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
        if (++count > 30000UL) return 0;   /* sensor not responding */
    }

    /* Measure echo HIGH duration */
    count = 0;
    while (PINC & (1 << echo_pin)) {
        count++;
        timer_delay_us(1);
        if (count > 30000UL) break;        /* out-of-range (~5 m) */
    }

    /* distance = count / 58  (using 343 m/s speed of sound) */
    return (uint16_t)(count / 58);
}

/**
 * @brief  Init PORTC pins for all three ultrasonic sensors.
 */
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

/**
 * @brief  Send a decimal integer over UART (no stdlib needed).
 */
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

    /* buf is in reverse order */
    while (i > 0) {
        UART_SendChar(buf[--i]);
    }
}

/**
 * @brief  Transmit the full turn report to the PC.
 *
 * Output format (exactly as required):
 *   Turns: 5\r\n
 *   Sequence: L, R, R, L, L\r\n
 */
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
    g_turn_count++;   /* Count even past MAX_TURNS; only log up to MAX */
}

/* =========================================================================
 * FSM – state handlers
 *
 * Each handler is called every loop iteration while in its state.
 * It returns the NEXT state (which may be itself for no transition).
 * ========================================================================= */

/* -----------------------------------------------------------------------
 * STATE_FORWARD
 * Drive straight.  Use side ultrasonic sensors to hug the right wall.
 * IR sensors detect the entrance of a turn junction.
 * ----------------------------------------------------------------------- */
static robot_state_t fsm_forward(void)
{
    /* --- Read all sensors --- */
    ir_snapshot_t ir = ir_read_all();

    uint16_t dist_front = ultrasonic_read_cm(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
    uint16_t dist_left  = ultrasonic_read_cm(US_LEFT_TRIG_PIN,  US_LEFT_ECHO_PIN);
    uint16_t dist_right = ultrasonic_read_cm(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

    /* --- Check for frontal obstacle first (highest priority) --- */
    if (dist_front > 0 && dist_front < FRONT_OBSTACLE_CM) {
        Motor_Stop();
        /*
         * Decide turn direction from IR state.
         * If both walls gone, default to left.
         */
        if (ir.right_wall) {
            return STATE_TURN_LEFT;
        } else {
            return STATE_TURN_RIGHT;
        }
    }

    /* --- IR: detect junction opening (turn entrance) --- */
    if (!ir.left_wall && ir.right_wall) {
        /* Left wall disappeared → Left junction ahead */
        Motor_Stop();
        return STATE_TURN_LEFT;
    }
    if (ir.left_wall && !ir.right_wall) {
        /* Right wall disappeared → Right junction ahead */
        Motor_Stop();
        return STATE_TURN_RIGHT;
    }
    if (!ir.left_wall && !ir.right_wall) {
        /* Both walls gone – open space or T-junction */
        Motor_Stop();
        return STATE_WALL_LOST;
    }

    /* --- Wall-hugging correction with side ultrasonics --- */
    /*
     * Strategy: hug the right wall.
     * If too close to right wall → steer left (slow right motor).
     * If too far from right wall → steer right (slow left motor).
     * Left wall distance used as secondary guard.
     */
    uint8_t left_speed  = BASE_SPEED;
    uint8_t right_speed = BASE_SPEED;

    /* Right wall correction */
    if (dist_right > 0) {
        int16_t error = (int16_t)dist_right - (int16_t)WALL_TARGET_CM;

        if (error > (int16_t)WALL_TOLERANCE_CM) {
            /* Too far from right wall → turn right slightly */
            left_speed  = TURN_SPEED_FAST;
            right_speed = TURN_SPEED_SLOW;
        } else if (error < -(int16_t)WALL_TOLERANCE_CM) {
            /* Too close to right wall → turn left slightly */
            left_speed  = TURN_SPEED_SLOW;
            right_speed = TURN_SPEED_FAST;
        }
    }

    /* Left wall guard: if unexpectedly close, steer away */
    if (dist_left > 0 && dist_left < (WALL_TARGET_CM - WALL_TOLERANCE_CM)) {
        left_speed  = TURN_SPEED_SLOW;
        right_speed = TURN_SPEED_FAST;
    }

    Motor_Forward(left_speed, right_speed);
    return STATE_FORWARD;
}

/* -----------------------------------------------------------------------
 * STATE_TURN_LEFT
 * Execute a 90° left pivot, log the turn, then return to FORWARD.
 * Uses a time-based approach (tune TURN_DURATION_MS on hardware).
 * ----------------------------------------------------------------------- */
static robot_state_t fsm_turn_left(void)
{
    log_turn(TURN_LEFT);

    /*
     * Pivot left: left motor reverse, right motor forward.
     * The L298N Motor_TurnLeft currently runs both forward with
     * differential speed; for a sharper 90° turn we drive directly
     * via the motor module.  Replace with Motor_PivotLeft() if you
     * add that function to motor.h.
     */
    Motor_TurnLeft(TURN_SPEED_SLOW, TURN_SPEED_SHARP);
    timer_delay_ms(TURN_DURATION_MS);
    Motor_Stop();
    timer_delay_ms(50);           /* brief settle */

    return STATE_FORWARD;
}

/* -----------------------------------------------------------------------
 * STATE_TURN_RIGHT
 * Execute a 90° right pivot, log the turn, then return to FORWARD.
 * ----------------------------------------------------------------------- */
static robot_state_t fsm_turn_right(void)
{
    log_turn(TURN_RIGHT);

    Motor_TurnRight(TURN_SPEED_SHARP, TURN_SPEED_SLOW);
    timer_delay_ms(TURN_DURATION_MS);
    Motor_Stop();
    timer_delay_ms(50);

    return STATE_FORWARD;
}

/* -----------------------------------------------------------------------
 * STATE_WALL_LOST
 * Creep slowly and poll ultrasonics until at least one wall is found.
 * If after WALL_LOST_CREEP_MS still nothing, try turning left (default).
 * ----------------------------------------------------------------------- */
static robot_state_t fsm_wall_lost(void)
{
    uint32_t start = timer_get_ms();

    while ((timer_get_ms() - start) < WALL_LOST_CREEP_MS) {
        uint16_t dist_left  = ultrasonic_read_cm(US_LEFT_TRIG_PIN,  US_LEFT_ECHO_PIN);
        uint16_t dist_right = ultrasonic_read_cm(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);

        /* Re-acquired at least one wall */
        if ((dist_left  > 0 && dist_left  < (WALL_TARGET_CM + 20)) ||
            (dist_right > 0 && dist_right < (WALL_TARGET_CM + 20))) {
            Motor_Stop();
            return STATE_FORWARD;
        }

        /* Slow creep forward while searching */
        Motor_Forward(BASE_SPEED / 2, BASE_SPEED / 2);
    }

    Motor_Stop();
    /* Default recovery: turn left and try to re-acquire */
    return STATE_TURN_LEFT;
}

/* -----------------------------------------------------------------------
 * STATE_STOP
 * Send the turn report once, then sit idle.
 * ----------------------------------------------------------------------- */
static robot_state_t fsm_stop(void)
{
    Motor_Stop();
    report_send();

    /* Stay in STOP forever */
    while (1) { /* idle */ }

    return STATE_STOP;  /* unreachable, silences compiler warning */
}

/* =========================================================================
 * System initialisation
 * ========================================================================= */
static void system_init(void)
{
    /* 1. Timer driver (must be first – other modules may use timer_get_ms) */
    timer_init();

    /* 2. UART @ 9600 baud */
    UART_Init(BAUD_RATE);

    /* 3. IR sensors (PD3, PD4 as inputs) */
    ir_sensor_init();

    /* 4. Ultrasonic sensors (PORTC) */
    ultrasonic_init_all();

    /* 5. Motors + PWM */
    Motor_Init();

    /* 6. Enable global interrupts (required for Timer0 CTC ISR) */
    sei();

    /* Brief startup delay to let sensors power up */
    timer_delay_ms(500);

    UART_SendString("Wall-follower ready.\r\n");
}

/* =========================================================================
 * Main
 * ========================================================================= */
int main(void)
{
    system_init();

    robot_state_t state = STATE_FORWARD;

    while (1) {
        switch (state) {
            case STATE_FORWARD:
                state = fsm_forward();
                break;

            case STATE_TURN_LEFT:
                state = fsm_turn_left();
                break;

            case STATE_TURN_RIGHT:
                state = fsm_turn_right();
                break;

            case STATE_WALL_LOST:
                state = fsm_wall_lost();
                break;

            case STATE_STOP:
                state = fsm_stop();    /* never returns */
                break;

            default:
                /* Should never reach here; safe fallback */
                Motor_Stop();
                state = STATE_STOP;
                break;
        }
    }

    return 0;   /* unreachable */
}