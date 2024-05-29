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
        UpdateMotorState(controller->motor);
        dutyControl(controller, controller->dutySetpoint);
        break;
    case MOTOR_CONTROLLER_MODE_VELOCITY_STPT:
        UpdateMotorState(controller->motor);
        velocityControl(controller, controller->velocitySetpoint);
        break;
    case MOTOR_CONTROLLER_MODE_POSITION_STPT:
        UpdateMotorState(controller->motor);
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

void velocityControl(motor_controller_t* controller, float setpoint) {
    float velocitySetpoint = clampf(setpoint, controller->params.minVelocity,
                                    controller->params.maxVelocity);
    float velocityMeasurement = controller->motor->state.velocity;
    float e = velocitySetpoint - velocityMeasurement;
    float currentSetpoint = StepPid(&controller->velocityPid, e);
    // feedforward from trajectory generator:
    // currentSetpoint += controller->trajectoryGenerator->acceleration *
    //                    controller->params.accelerationFeedForwardGain;
    // filter currentSetpoint here
    currentSetpoint = clampf(currentSetpoint, controller->params.minCurrent,
                             controller->params.maxCurrent);
    float currentMeasurement = controller->motor->state.current;
    e = currentSetpoint - currentMeasurement;
    float dutySetpoint = StepPid(&controller->currentPid, e);
    dutyControl(controller, dutySetpoint);
    dutyControl(controller, currentSetpoint);
}

void positionControl(motor_controller_t* controller, float setpoint) {
    float positionSetpoint = clampf(setpoint, controller->params.minPosition,
                                    controller->params.maxPosition);
    float positionMeasurement = controller->motor->state.position;
    float e = positionSetpoint - positionMeasurement;
    float velocitySetpoint = StepPid(&controller->positionPid, e);
    // feedforward from trajectory generator:
    // velocitySetpoint += controller->trajectoryGenerator->velocity *
    //                     controller->params.velocityFeedForwardGain;
    // filter velocitySetpoint here
    velocityControl(controller, velocitySetpoint);
}