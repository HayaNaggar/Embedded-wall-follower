/**
 * @file  IR_sensor.c
 * @brief IR sensor driver – 2 diagonal front-corner open-space detectors.
 */

#include "IR_sensor.h"

typedef struct {
    gpio_port_t port;
    uint8_t     pin;

} ir_pin_map_t;

static const ir_pin_map_t ir_pins[IR_SENSOR_COUNT] = {
    [IR_SENSOR_LEFT]  = { IR_LEFT_PORT,  IR_LEFT_PIN  },
    [IR_SENSOR_RIGHT] = { IR_RIGHT_PORT, IR_RIGHT_PIN },
};

void ir_sensor_init(void)
{
    for (uint8_t i = 0; i < IR_SENSOR_COUNT; i++) {
        gpio_set_direction(ir_pins[i].port, ir_pins[i].pin, GPIO_INPUT);
        gpio_set_pullup(ir_pins[i].port, ir_pins[i].pin, GPIO_PULLUP_DISABLE);
    }
}

uint8_t ir_is_wall_present(ir_sensor_id_t sensor)
{
    if (sensor >= IR_SENSOR_COUNT) return 1; /* safe default: assume wall */
    uint8_t raw = gpio_read(ir_pins[sensor].port, ir_pins[sensor].pin);
    return (raw == IR_ACTIVE_LEVEL) ? 1 : 0;
}

ir_snapshot_t ir_read_all(void)
{
    ir_snapshot_t s;
    s.left_wall  = ir_is_wall_present(IR_SENSOR_LEFT);
    s.right_wall = ir_is_wall_present(IR_SENSOR_RIGHT);
    return s;
}

uint8_t ir_read_raw(ir_sensor_id_t sensor)
{
    if (sensor >= IR_SENSOR_COUNT) return GPIO_LOW;
    return gpio_read(ir_pins[sensor].port, ir_pins[sensor].pin);
}