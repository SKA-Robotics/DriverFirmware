#ifndef MOTOR_H
#define MOTOR_H

#include "filter.h"
#include "ma730_driver.h"
#include "system.h"
#include <stdint.h>

#define FORWARD 1
#define REVERSE 0

#define ENCODER_RESOLUTION 65535
#define ENCODER_REVOLUTION_THRESHOLD 32137

#define CURRENT_ADC_DEADZONE 3
#define CURRENT_MULT 0.00145f
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
    float duty;    // Motor duty command (-1;1)
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
    ma730_device_t encoder; // Encoder connected to the motor axis
    uint8_t invertAxis;     // Inverts the motor rotation direction
    float positionOffset;   // Motor position offset
    iir_filter_t
        currentMeasurementFilter; // IIR filter applied to measured current
    iir_filter_t
        velocityMeasurementFilter; // IIR filter applied to measured velocity
    motor_state_t state;           // Current state of the motor
};

void Motor_SetDuty(motor_t* motor, float duty);
void Motor_UpdateState(motor_t* motor);

#endif // MOTOR_H