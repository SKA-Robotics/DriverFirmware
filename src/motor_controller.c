#include "motor_controller.h"

float clampf(float value, float min, float max) {
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

void motorControllerStep(motor_controller_t* controller) {
    switch (controller->mode) {
    case MOTOR_CONTROLLER_MODE_DUTY_STPT:
        dutyControl(controller, controller->dutySetpoint);
        break;
    case MOTOR_CONTROLLER_MODE_VELOCITY_STPT:
        velocityControl(controller, controller->velocitySetpoint);
        break;
    case MOTOR_CONTROLLER_MODE_POSITION_STPT:
        positionControl(controller, controller->positionSetpoint);
        break;
    default:
        break;
    }
}

void motorControllerSetPositionSetpoint(motor_controller_t* controller,
                                        float setpoint) {
    controller->positionSetpoint = setpoint;
    controller->mode = MOTOR_CONTROLLER_MODE_POSITION_STPT;
}

void motorControllerSetVelocitySetpoint(motor_controller_t* controller,
                                        float setpoint) {
    controller->velocitySetpoint = setpoint;
    controller->mode = MOTOR_CONTROLLER_MODE_VELOCITY_STPT;
}

void motorControllerSetDutySetpoint(motor_controller_t* controller,
                                    float setpoint) {
    controller->dutySetpoint = setpoint;
    controller->mode = MOTOR_CONTROLLER_MODE_DUTY_STPT;
}

void dutyControl(motor_controller_t* controller, float setpoint) {
    SetMotorDuty(controller->motor, clampf(setpoint, controller->params.minDuty,
                                           controller->params.maxDuty));
}

void velocityControl(motor_controller_t* controller, float setpoint) {}

void positionControl(motor_controller_t* controller, float setpoint) {}