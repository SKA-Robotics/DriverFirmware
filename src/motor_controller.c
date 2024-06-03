#include "motor_controller.h"

float clampf(float value, float min, float max) {
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

void DutyControl(motor_controller_t* controller, float setpoint);
void VelocityControl(motor_controller_t* controller, float setpoint);
void PositionControl(motor_controller_t* controller, float setpoint);

void MotorController_Step(motor_controller_t* controller) {
    switch (controller->mode) {
    case MOTOR_CONTROLLER_MODE_DUTY_STPT:
        Motor_UpdateState(controller->motor);
        DutyControl(controller, controller->dutySetpoint);
        break;
    case MOTOR_CONTROLLER_MODE_VELOCITY_STPT:
        Motor_UpdateState(controller->motor);
        VelocityControl(controller, controller->velocitySetpoint);
        break;
    case MOTOR_CONTROLLER_MODE_POSITION_STPT:
        Motor_UpdateState(controller->motor);
        PositionControl(controller, controller->positionSetpoint);
        break;
    default:
        break;
    }
}

void MotorController_SetPositionSetpoint(motor_controller_t* controller,
                                         float setpoint) {
    controller->positionSetpoint = setpoint;
    controller->mode = MOTOR_CONTROLLER_MODE_POSITION_STPT;
}

void MotorController_SetVelocitySetpoint(motor_controller_t* controller,
                                         float setpoint) {
    controller->velocitySetpoint = setpoint;
    controller->mode = MOTOR_CONTROLLER_MODE_VELOCITY_STPT;
}

void MotorController_SetDutySetpoint(motor_controller_t* controller,
                                     float setpoint) {
    controller->dutySetpoint = setpoint;
    controller->mode = MOTOR_CONTROLLER_MODE_DUTY_STPT;
}

void DutyControl(motor_controller_t* controller, float setpoint) {
    Motor_SetDuty(controller->motor,
                  clampf(setpoint, controller->params.minDuty,
                         controller->params.maxDuty));
}

void VelocityControl(motor_controller_t* controller, float setpoint) {
    float velocitySetpoint = clampf(setpoint, controller->params.minVelocity,
                                    controller->params.maxVelocity);
    float velocityMeasurement = controller->motor->state.velocity;
    float e = velocitySetpoint - velocityMeasurement;
    float currentSetpoint = StepPid(&controller->velocityPid, e);
    currentSetpoint =
        ApplyIirFilter(&controller->velocityPidOutputFilter, currentSetpoint);
    float currentMeasurement = controller->motor->state.current;
    e = currentSetpoint - currentMeasurement;
    float dutySetpoint = StepPid(&controller->currentPid, e);
    dutySetpoint =
        ApplyIirFilter(&controller->currentPidOutputFilter, dutySetpoint);
    DutyControl(controller, dutySetpoint);
}

void PositionControl(motor_controller_t* controller, float setpoint) {
    float positionSetpoint = clampf(setpoint, controller->params.minPosition,
                                    controller->params.maxPosition);
    float positionMeasurement = controller->motor->state.position;
    float e = positionSetpoint - positionMeasurement;
    float velocitySetpoint = StepPid(&controller->positionPid, e);
    velocitySetpoint =
        ApplyIirFilter(&controller->positionPidOutputFilter, velocitySetpoint);
    VelocityControl(controller, velocitySetpoint);
}
