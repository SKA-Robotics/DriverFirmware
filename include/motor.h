#ifndef MOTOR_H
#define MOTOR_H

#include "system.h"
#include <stdint.h>

#define FORWARD 1
#define REVERSE 0
#define PI 3.14159265359f

#define ENCODER_RESOLUTION 65535
#define ENCODER_REVOLUTION_THRESHOLD 32137

typedef struct motor motor_t;
typedef struct motor_state motor_state_t;

struct motor_state {
    uint8_t direction;
    uint16_t prevPositionMeasurement;
    int32_t positionRaw;
    float position;
    float velocity;
    float current;
    float prevVelocity;
};

struct motor {
    volatile uint32_t* pwmChannelForward;
    volatile uint32_t* pwmChannelReverse;
    uint32_t adcChannel;
    GPIO_TypeDef* encoderCsPort;
    uint16_t encoderCsPin;
    motor_state_t state;
};

void SetMotorDuty(motor_t* motor, float duty);
void UpdateMotorState(motor_t* motor);

#endif // MOTOR_H