#include "roboszpon_axis.h"
#include "mag_alpha_driver.h"
#include "message_serialization.h"
#include "roboszpon_message.h"

void roboszponAxisStoppedStep(roboszpon_axis_t* axis);
void roboszponAxisRunningStep(roboszpon_axis_t* axis);
void roboszponAxisErrorStep(roboszpon_axis_t* axis);
uint64_t roboszponAxisCheckError(roboszpon_axis_t* axis);
void roboszponAxisApplyMotorCommand(roboszpon_axis_t* axis,
                                    roboszpon_message_t* command);
void roboszponAxisWriteParam(roboszpon_axis_t* axis, uint8_t paramId,
                             float value);
float roboszponAxisReadParam(roboszpon_axis_t* axis, uint8_t paramId);

void roboszponAxisStep(roboszpon_axis_t* axis) {
    axis->flags = roboszponAxisCheckError(axis);
    switch (axis->state) {
    case ROBOSZPON_AXIS_STATE_STOPPED:
        roboszponAxisStoppedStep(axis);
        return;
    case ROBOSZPON_AXIS_STATE_RUNNING:
        roboszponAxisRunningStep(axis);
        return;
    case ROBOSZPON_AXIS_STATE_ERROR:
        roboszponAxisErrorStep(axis);
        return;
    default:
        // What are you even doing here?
        return;
    }
}

void roboszponAxisStoppedStep(roboszpon_axis_t* axis) {
    // Check message queue, apply configuration messages, delete run
    // mode messages If ARM command is received, set mode to
    // ROBOSZPON_AXIS_STATE_RUNNING
    while (!MessageQueue_IsEmpty(axis->messageQueue)) {
        roboszpon_message_t message;
        MessageQueue_Dequeue(axis->messageQueue, &message);
        switch (message.id) {
        case MSG_PARAMETER_WRITE:
            printf("Parameter write command received. Payload: %lld\n",
                   message.data);
            message_parameter_write_t param =
                interpretParameterWriteMessage(&message);
            roboszponAxisWriteParam(axis, param.paramId, param.value);
            break;
        case MSG_PARAMETER_READ:
            printf("Parameter read command received. Payload: %lld\n",
                   message.data);
            message_parameter_read_t request =
                interpretParameterReadMessage(&message);
            float value = roboszponAxisReadParam(axis, request.paramId);
            sendParameterResponseMessage(axis->nodeId, request.paramId, value);
            break;
        case MSG_ACTION_REQUEST:
            if (message.data == ACTION_ARM) {
                printf("Armed\n");
                axis->state = ROBOSZPON_AXIS_STATE_RUNNING;
            }
            // TODO: implement all the other actions
            break;
        default:
            break;
        }
    }
}

void roboszponAxisRunningStep(roboszpon_axis_t* axis) {
    // Check for errors. If there is an error, set motor effort to 0, set
    // error LED on, break and set mode to ROBOSZPON_AXIS_STATE_ERROR
    // Get latest axis command from message queue, delete other messages
    // If there was a disarm command, set mode to
    // ROBOSZPON_AXIS_STATE_STOPPED, set motor effort to 0 and break. Else,
    // pass the command to trajectory generator -> motor controller -> motor
    if (axis->flags != 0) {
        SetMotorDuty(axis->motor, 0.0f);
        HAL_GPIO_WritePin(axis->errorLedPort, axis->errorLedPin, GPIO_PIN_SET);
        printf("Error detected\n");
        axis->state = ROBOSZPON_AXIS_STATE_ERROR;
    }
    int isThereAMotorCommand = 0;
    roboszpon_message_t latestMotorCommand;
    while (!MessageQueue_IsEmpty(axis->messageQueue)) {
        roboszpon_message_t message;
        MessageQueue_Dequeue(axis->messageQueue, &message);
        switch (message.id) {
        case MSG_EMERGENCY_STOP:
            printf("Emergency stop\n");
            printf("Disarmed\n");
            SetMotorDuty(axis->motor, 0.0f);
            axis->state = ROBOSZPON_AXIS_STATE_STOPPED;
            return;
        case MSG_MOTOR_COMMAND:
            isThereAMotorCommand = 1;
            latestMotorCommand.id = message.id;
            latestMotorCommand.data = message.data;
            break;
        case MSG_ACTION_REQUEST:
            if (interpretActionRequestMessage(&message).actionId ==
                ACTION_DISARM) {
                SetMotorDuty(axis->motor, 0.0f);
                printf("Disarmed\n");
                axis->state = ROBOSZPON_AXIS_STATE_STOPPED;
                return;
            }
            break;
        default:
            break;
        }
    }
    if (isThereAMotorCommand) {
        printf("Received motor command\n");
        roboszponAxisApplyMotorCommand(axis, &latestMotorCommand);
    }
}

void roboszponAxisErrorStep(roboszpon_axis_t* axis) {
    // Delete all messages from message queue. If there is a disarm command,
    // set mode to DISARM, reset error LED and break. else, check for
    // errors.
    while (!MessageQueue_IsEmpty(axis->messageQueue)) {
        roboszpon_message_t message;
        MessageQueue_Dequeue(axis->messageQueue, &message);
        if (message.id == MSG_ACTION_REQUEST) {
            if (interpretActionRequestMessage(&message).actionId ==
                ACTION_DISARM) {
                HAL_GPIO_WritePin(axis->errorLedPort, axis->errorLedPin,
                                  GPIO_PIN_RESET);
                printf("Disarmed\n");
                axis->state = ROBOSZPON_AXIS_STATE_STOPPED;
            }
        }
    }
    // If there are no errors, reset error LED, set mode to
    // ROBOSZPON_AXIS_STATE_RUNNING and break.
    if (axis->flags == 0) {
        HAL_GPIO_WritePin(axis->errorLedPort, axis->errorLedPin,
                          GPIO_PIN_RESET);
        printf("Error is gone\n");
        axis->state = ROBOSZPON_AXIS_STATE_RUNNING;
    }
}

uint64_t roboszponAxisCheckError(roboszpon_axis_t* axis) {
    uint64_t error = 0;
    uint8_t encoderError =
        MA730_GetError(axis->motor->encoderCsPort, axis->motor->encoderCsPin);
    error |= encoderError;
    // TODO: check for other errors. (Command timeout included)
    return error;
}

void roboszponAxisApplyMotorCommand(roboszpon_axis_t* axis,
                                    roboszpon_message_t* command) {
    message_motor_command_t message = interpretMotorCommandMessage(command);
    switch (message.controlSignalId) {
    case CTRLSIGNAL_DUTY:
        printf("Duty set to %f\n", message.value);
        motorControllerSetDutySetpoint(axis->motorController, message.value);
        break;
    case CTRLSIGNAL_VELOCITY:
        printf("Velocity set to %f\n", message.value);
        motorControllerSetDutySetpoint(axis->motorController, message.value);
        motorControllerSetVelocitySetpoint(axis->motorController,
                                           message.value);
        break;
    case CTRLSIGNAL_POSITION:
        printf("Position set to %f\n", message.value);
        motorControllerSetDutySetpoint(axis->motorController, message.value);
        motorControllerSetPositionSetpoint(axis->motorController,
                                           message.value);
        break;
    }
}

void roboszponAxisWriteParam(roboszpon_axis_t* axis, uint8_t paramId,
                             float value) {
    switch (paramId) {
    case PARAM_COMMAND_TIMEOUT:
        axis->commandTimeout = (uint32_t)(value * 1000.0f);
        printf("Set timeout to %ld milliseconds.", axis->commandTimeout);
        break;
    case PARAM_ENCODER_OFFSET:
        // TODO: implement encoder offset
        break;
    case PARAM_PPID_Kp:
        axis->motorController->positionPid.Kp = value;
        break;
    case PARAM_PPID_Ki:
        axis->motorController->positionPid.Ki = value;
        break;
    case PARAM_PPID_Kd:
        axis->motorController->positionPid.Kd = value;
        break;
    case PARAM_PPID_Kaw:
        axis->motorController->positionPid.Kaw = value;
        break;
    case PARAM_PPID_deadzone:
        axis->motorController->positionPid.deadzone = value;
        break;
    case PARAM_PPID_Umax:
        axis->motorController->positionPid.u_max = value;
        break;
    case PARAM_PPID_dUmax:
        axis->motorController->positionPid.du_max = value;
        break;
    case PARAM_VPID_Kp:
        axis->motorController->velocityPid.Kp = value;
        break;
    case PARAM_VPID_Ki:
        axis->motorController->velocityPid.Ki = value;
        break;
    case PARAM_VPID_Kd:
        axis->motorController->velocityPid.Kd = value;
        break;
    case PARAM_VPID_Kaw:
        axis->motorController->velocityPid.Kaw = value;
        break;
    case PARAM_VPID_deadzone:
        axis->motorController->velocityPid.deadzone = value;
        break;
    case PARAM_VPID_Umax:
        axis->motorController->velocityPid.u_max = value;
        break;
    case PARAM_VPID_dUmax:
        axis->motorController->velocityPid.du_max = value;
        break;
    case PARAM_CPID_Kp:
        axis->motorController->currentPid.Kp = value;
        break;
    case PARAM_CPID_Ki:
        axis->motorController->currentPid.Ki = value;
        break;
    case PARAM_CPID_Kd:
        axis->motorController->currentPid.Kd = value;
        break;
    case PARAM_CPID_Kaw:
        axis->motorController->currentPid.Kaw = value;
        break;
    case PARAM_CPID_deadzone:
        axis->motorController->currentPid.deadzone = value;
        break;
    case PARAM_CPID_Umax:
        axis->motorController->currentPid.u_max = value;
        break;
    case PARAM_CPID_dUmax:
        axis->motorController->currentPid.du_max = value;
        break;
    case PARAM_MIN_POSITION:
        axis->motorController->params.minPosition = value;
        break;
    case PARAM_MAX_POSITION:
        axis->motorController->params.maxPosition = value;
        break;
    case PARAM_MIN_VELOCITY:
        axis->motorController->params.minVelocity = value;
        break;
    case PARAM_MAX_VELOCITY:
        axis->motorController->params.maxVelocity = value;
        break;
    case PARAM_MIN_CURRENT:
        axis->motorController->params.minCurrent = value;
        break;
    case PARAM_MAX_CURRENT:
        axis->motorController->params.maxCurrent = value;
        break;
    case PARAM_MIN_DUTY:
        axis->motorController->params.minDuty = value;
        break;
    case PARAM_MAX_DUTY:
        axis->motorController->params.maxDuty = value;
        break;
    default:
        break;
    }
}

float roboszponAxisReadParam(roboszpon_axis_t* axis, uint8_t paramId) {
    switch (paramId) {
    case PARAM_COMMAND_TIMEOUT:
        return ((float)axis->commandTimeout) / 1000.0f;
    case PARAM_ENCODER_OFFSET:
        // TODO: implement encoder offset
        return 0.0;
    case PARAM_PPID_Kp:
        return axis->motorController->positionPid.Kp;
    case PARAM_PPID_Ki:
        return axis->motorController->positionPid.Ki;
    case PARAM_PPID_Kd:
        return axis->motorController->positionPid.Kd;
    case PARAM_PPID_Kaw:
        return axis->motorController->positionPid.Kaw;
    case PARAM_PPID_deadzone:
        return axis->motorController->positionPid.deadzone;
    case PARAM_PPID_Umax:
        return axis->motorController->positionPid.u_max;
    case PARAM_PPID_dUmax:
        return axis->motorController->positionPid.du_max;
    case PARAM_VPID_Kp:
        return axis->motorController->velocityPid.Kp;
    case PARAM_VPID_Ki:
        return axis->motorController->velocityPid.Ki;
    case PARAM_VPID_Kd:
        return axis->motorController->velocityPid.Kd;
    case PARAM_VPID_Kaw:
        return axis->motorController->velocityPid.Kaw;
    case PARAM_VPID_deadzone:
        return axis->motorController->velocityPid.deadzone;
    case PARAM_VPID_Umax:
        return axis->motorController->velocityPid.u_max;
    case PARAM_VPID_dUmax:
        return axis->motorController->velocityPid.du_max;
    case PARAM_CPID_Kp:
        return axis->motorController->currentPid.Kp;
    case PARAM_CPID_Ki:
        return axis->motorController->currentPid.Ki;
    case PARAM_CPID_Kd:
        return axis->motorController->currentPid.Kd;
    case PARAM_CPID_Kaw:
        return axis->motorController->currentPid.Kaw;
    case PARAM_CPID_deadzone:
        return axis->motorController->currentPid.deadzone;
    case PARAM_CPID_Umax:
        return axis->motorController->currentPid.u_max;
    case PARAM_CPID_dUmax:
        return axis->motorController->currentPid.du_max;
    case PARAM_MIN_POSITION:
        return axis->motorController->params.minPosition;
    case PARAM_MAX_POSITION:
        return axis->motorController->params.maxPosition;
    case PARAM_MIN_VELOCITY:
        return axis->motorController->params.minVelocity;
    case PARAM_MAX_VELOCITY:
        return axis->motorController->params.maxVelocity;
    case PARAM_MIN_CURRENT:
        return axis->motorController->params.minCurrent;
    case PARAM_MAX_CURRENT:
        return axis->motorController->params.maxCurrent;
    case PARAM_MIN_DUTY:
        return axis->motorController->params.minDuty;
    case PARAM_MAX_DUTY:
        return axis->motorController->params.maxDuty;
    default:
        return 0.0f;
    }
}