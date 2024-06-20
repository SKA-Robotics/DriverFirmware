#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "filter.h"
#include "motor.h"
#include "pid.h"

#define MOTOR_CONTROLLER_MODE_DUTY_STPT 0x00
#define MOTOR_CONTROLLER_MODE_VELOCITY_STPT 0x01
#define MOTOR_CONTROLLER_MODE_POSITION_STPT 0x02

typedef struct {
    float minDuty;
    float maxDuty;
    float minCurrent;
    float maxCurrent;
    float minVelocity;
    float maxVelocity;
    float minPosition;
    float maxPosition;
    float currentFeedforward;
    float dutyDeadzone;
} motor_controller_params_t;

typedef struct {
    motor_t* motor;
    uint8_t mode;
    float dutySetpoint;
    float velocitySetpoint;
    float positionSetpoint;
    pid_controller_t velocityPid;
    pid_controller_t positionPid;
    pid_controller_t currentPid;
    iir_filter_t currentPidOutputFilter;
    iir_filter_t velocityPidOutputFilter;
    iir_filter_t positionPidOutputFilter;
    motor_controller_params_t params;
} motor_controller_t;

void MotorController_Step(motor_controller_t* controller);
void MotorController_SetDutySetpoint(motor_controller_t* controller,
                                     float setpoint);
void MotorController_SetVelocitySetpoint(motor_controller_t* controller,
                                         float setpoint);
void MotorController_SetPositionSetpoint(motor_controller_t* controller,
                                         float setpoint);

#endif // MOTOR_CONTROLLER_H