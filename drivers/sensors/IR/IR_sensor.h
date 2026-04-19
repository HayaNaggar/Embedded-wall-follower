/**
 * @file  IR_sensor.h
 * @brief IR sensor – 2 diagonal front-corner open-space detectors.
 *
 * Physical placement:
 *   LEFT  sensor: front-left  corner, angled outward to the left wall.
 *   RIGHT sensor: front-right corner, angled outward to the right wall.
 *
 * Role:
 *   detect when a wall DISAPPEARS beside the robot, signalling an upcoming turn junction.
 *
 *   Normal travel  → both sensors see walls (output 1, 1)
 *   Left turn ahead  → left  sensor loses wall first (output 0, 1) → turn LEFT
 *   Right turn ahead → right sensor loses wall first (output 1, 0) → turn RIGHT
 *
 *   The case (0, 0) — both open — does not occur on this track.
 *
 * Pin mapping:
 *   LEFT  sensor OUT → D3  (PORTD, bit 3)
 *   RIGHT sensor OUT → D4  (PORTD, bit 4)
 */

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "gpio.h"

/* -----------------------------------------------------------------------
 * Pin mapping – edit to match your wiring
 * ---------------------------------------------------------------------- */
#define IR_LEFT_PORT    GPIO_PORT_D
#define IR_LEFT_PIN     3

#define IR_RIGHT_PORT   GPIO_PORT_D
#define IR_RIGHT_PIN    4

/* -----------------------------------------------------------------------
 * Active level:
 *   GPIO_LOW  → OUT pulls LOW  when obstacle detected (most modules)
 * ---------------------------------------------------------------------- */
#define IR_ACTIVE_LEVEL  GPIO_LOW

/* -----------------------------------------------------------------------
 * Sensor identifiers
 * ---------------------------------------------------------------------- */
typedef enum {
    IR_SENSOR_LEFT  = 0,
    IR_SENSOR_RIGHT = 1
} ir_sensor_id_t;

#define IR_SENSOR_COUNT 2

/* -----------------------------------------------------------------------
 * Snapshot – always read both sensors together once per FSM tick.
 *
 * Fields use "wall_present" naming to make FSM conditions readable:
 *   snapshot.left_wall  = 1 → left wall still there, no turn yet
 *   snapshot.left_wall  = 0 → left wall gone → LEFT turn detected
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t left_wall;   /**< 1 = wall present on left,  0 = open space */
    uint8_t right_wall;  /**< 1 = wall present on right, 0 = open space */
} ir_snapshot_t;

/* -----------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise both IR sensor pins as inputs (no pull-up).
 *         Call once during system init.
 */
void ir_sensor_init(void);

/**
 * @brief  Read both sensors atomically into a snapshot.
 *
 *         Call ONCE at the start of each FSM tick and pass the
 *         snapshot to your state logic. Do not call twice per tick.
 *
 * @return ir_snapshot_t  .left_wall and .right_wall set.
 */
ir_snapshot_t ir_read_all(void);

/**
 * @brief  Returns 1 if the wall is still present on that side, 0 if open.
 * @param  sensor  IR_SENSOR_LEFT or IR_SENSOR_RIGHT.
 */
uint8_t ir_is_wall_present(ir_sensor_id_t sensor);

/**
 * @brief  Raw GPIO read before active-level inversion (debug only).
 */
uint8_t ir_read_raw(ir_sensor_id_t sensor);

#endif /* IR_SENSOR_H */