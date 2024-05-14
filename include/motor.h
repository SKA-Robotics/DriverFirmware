#ifndef MOTOR_H
#define MOTOR_H

#include "system.h"
#include <stdint.h>

#define FORWARD 1
#define REVERSE 0

#define ENCODER_RESOLUTION 65535
#define ENCODER_REVOLUTION_THRESHOLD 32137

#define CURRENT_ADC_DEADZONE 3
#define CURRENT_MULT 0.00125f
#define CURRENT_OFFSET 0.0000f

typedef struct motor motor_t;
typedef struct motor_state motor_state_t;

// Represents a current state of a motor
struct motor_state {
    uint8_t direction; // Motor rotation direction. Should be FORWARD or REVERSE
    uint16_t prevPositionMeasurement;
    int32_t positionRaw; // Motor position (encoder value)
    float position;      // Motor position (radians)
    float velocity;      // Motor speed (radians per second)
    float current; // Motor current (probably milliampers. TODO: calibrate)
};

// Represents a motor
struct motor {
    // Address of TIM capture/compare register
    // corresponding to forward PWM signal
    volatile uint32_t* pwmChannelForward;
    // Address of TIM capture/compare register
    // corresponding to reverse PWM signal
    volatile uint32_t* pwmChannelReverse;
    // ADC channel corresponding to motor current measurement
    uint32_t adcChannel;
    GPIO_TypeDef* encoderCsPort; // Port of encoder chip select pin
    uint16_t encoderCsPin;       // Encoder chip select pin
    motor_state_t state;         // Current state of the motor
};

void SetMotorDuty(motor_t* motor, float duty);
void UpdateMotorState(motor_t* motor);

#endif // MOTOR_H