#ifndef ROBOSZPON_AXIS_H
#define ROBOSZPON_AXIS_H

#include <stdint.h>

#define ROBOSZPON_AXIS_STATE_STOPPED 0x00
#define ROBOSZPON_AXIS_STATE_RUNNING 0x01
#define ROBOSZPON_AXIS_STATE_ERROR 0x02

typedef struct {
    float command_timeout;
    // axis parameters
} roboszpon_axis_params_t;

typedef struct {
    uint8_t state;
    roboszpon_axis_params_t params;
    // axis elements (motor, controller, message queue, etc)
} roboszpon_axis_t;

void roboszpon_axis_step(roboszpon_axis_t* axis);

#endif // ROBOSZPON_AXIS_H