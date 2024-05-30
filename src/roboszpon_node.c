#include "roboszpon_node.h"
#include "ma730_driver.h"
#include "message_serialization.h"
#include "roboszpon_message.h"

uint64_t RoboszponNode_CheckError(roboszpon_node_t* node);
void RoboszponNode_StoppedStep(roboszpon_node_t* node);
void RoboszponNode_RunningStep(roboszpon_node_t* node);
void RoboszponNode_ErrorStep(roboszpon_node_t* node);
void RoboszponNode_Disarm(roboszpon_node_t* node);
void RoboszponNode_ApplyMotorCommand(roboszpon_node_t* node,
                                     roboszpon_message_t* command);
void RoboszponNode_WriteParam(roboszpon_node_t* node, uint8_t paramId,
                              float value);
float RoboszponNode_ReadParam(roboszpon_node_t* node, uint8_t paramId);

void RoboszponNode_Step(roboszpon_node_t* node) {
    node->flags = RoboszponNode_CheckError(node);
    SendMessage_StatusReport(node);
    switch (node->state) {
    case ROBOSZPON_NODE_STATE_STOPPED:
        RoboszponNode_StoppedStep(node);
        return;
    case ROBOSZPON_NODE_STATE_RUNNING:
        SendMessage_AxisReport(node);
        SendMessage_MotorReport(node);
        RoboszponNode_RunningStep(node);
        return;
    case ROBOSZPON_NODE_STATE_ERROR:
        RoboszponNode_ErrorStep(node);
        return;
    default:
        return;
    }
}

uint64_t RoboszponNode_CheckError(roboszpon_node_t* node) {
    uint64_t error = 0;
    uint8_t encoderError =
        MA730_GetError(node->motor->encoderCsPort, node->motor->encoderCsPin);
    error |= encoderError;
    // TODO: check for other errors. (Command timeout included)
    return error;
}

void RoboszponNode_StoppedStep(roboszpon_node_t* node) {
    roboszpon_message_t message;
    while (!MessageQueue_IsEmpty(node->messageQueue)) {
        MessageQueue_Dequeue(node->messageQueue, &message);
        switch (message.id) {
        case MSG_PARAMETER_WRITE: {
            message_parameter_write_t param =
                ParseMessage_ParameterWrite(&message);
            RoboszponNode_WriteParam(node, param.paramId, param.value);
        } break;
        case MSG_PARAMETER_READ: {
            message_parameter_read_t request =
                ParseMessage_ParameterRead(&message);
            float value = RoboszponNode_ReadParam(node, request.paramId);
            SendMessage_ParameterResponse(node->nodeId, request.paramId, value);
        } break;
        case MSG_ACTION_REQUEST:
            if (message.data == ACTION_ARM) {
                node->state = ROBOSZPON_NODE_STATE_RUNNING;
            }
            // TODO: implement all the other actions
            break;
        default:
            // Ignore all the other messages
            break;
        }
    }
}

void RoboszponNode_RunningStep(roboszpon_node_t* node) {
    if (node->flags != ROBOSZPON_NO_ERROR) { // Error detected
        Motor_SetDuty(node->motor, 0.0f);
        HAL_GPIO_WritePin(node->errorLedPort, node->errorLedPin, GPIO_PIN_SET);
        node->state = ROBOSZPON_NODE_STATE_ERROR;
    }
    int isThereAMotorCommand = 0;
    roboszpon_message_t message;
    roboszpon_message_t latestMotorCommand;
    while (!MessageQueue_IsEmpty(node->messageQueue)) {
        MessageQueue_Dequeue(node->messageQueue, &message);
        switch (message.id) {
        case MSG_EMERGENCY_STOP:
            RoboszponNode_Disarm(node);
            node->state = ROBOSZPON_NODE_STATE_STOPPED;
            return;
        case MSG_MOTOR_COMMAND:
            isThereAMotorCommand = 1;
            latestMotorCommand.id = message.id;
            latestMotorCommand.data = message.data;
            break;
        case MSG_ACTION_REQUEST:
            if (ParseMessage_ActionRequest(&message).actionId ==
                ACTION_DISARM) {
                RoboszponNode_Disarm(node);
                node->state = ROBOSZPON_NODE_STATE_STOPPED;
                return;
            }
            break;
        default:
            break;
        }
    }
    if (isThereAMotorCommand) {
        RoboszponNode_ApplyMotorCommand(node, &latestMotorCommand);
    }
}

void RoboszponNode_ErrorStep(roboszpon_node_t* node) {
    roboszpon_message_t message;
    while (!MessageQueue_IsEmpty(node->messageQueue)) {
        MessageQueue_Dequeue(node->messageQueue, &message);
        if (message.id == MSG_ACTION_REQUEST) {
            if (ParseMessage_ActionRequest(&message).actionId ==
                ACTION_DISARM) {
                HAL_GPIO_WritePin(node->errorLedPort, node->errorLedPin,
                                  GPIO_PIN_RESET);
                node->state = ROBOSZPON_NODE_STATE_STOPPED;
            }
        }
    }
    if (node->flags == ROBOSZPON_NO_ERROR) { // All errors resolved
        HAL_GPIO_WritePin(node->errorLedPort, node->errorLedPin,
                          GPIO_PIN_RESET);
        node->state = ROBOSZPON_NODE_STATE_RUNNING;
    }
}

void RoboszponNode_Disarm(roboszpon_node_t* node) {
    MotorController_SetPositionSetpoint(node->motorController, 0.0f);
    MotorController_SetVelocitySetpoint(node->motorController, 0.0f);
    MotorController_SetDutySetpoint(node->motorController, 0.0f);
    Motor_SetDuty(node->motor, 0.0f);
}

void RoboszponNode_ApplyMotorCommand(roboszpon_node_t* node,
                                     roboszpon_message_t* command) {
    message_motor_command_t message = ParseMessage_MotorCommand(command);
    switch (message.controlSignalId) {
    case CTRLSIGNAL_DUTY:
        MotorController_SetDutySetpoint(node->motorController, message.value);
        break;
    case CTRLSIGNAL_VELOCITY:
        MotorController_SetDutySetpoint(node->motorController, message.value);
        MotorController_SetVelocitySetpoint(node->motorController,
                                            message.value);
        break;
    case CTRLSIGNAL_POSITION:
        MotorController_SetDutySetpoint(node->motorController, message.value);
        MotorController_SetPositionSetpoint(node->motorController,
                                            message.value);
        break;
    }
}

void RoboszponNode_WriteParam(roboszpon_node_t* node, uint8_t paramId,
                              float value) {
    switch (paramId) {
    case PARAM_COMMAND_TIMEOUT:
        node->commandTimeout = (uint32_t)(value * 1000.0f);
        break;
    case PARAM_ENCODER_OFFSET:
        // TODO: implement encoder offset
        break;
    case PARAM_PPID_Kp:
        node->motorController->positionPid.Kp = value;
        break;
    case PARAM_PPID_Ki:
        node->motorController->positionPid.Ki = value;
        break;
    case PARAM_PPID_Kd:
        node->motorController->positionPid.Kd = value;
        break;
    case PARAM_PPID_Kaw:
        node->motorController->positionPid.Kaw = value;
        break;
    case PARAM_PPID_deadzone:
        node->motorController->positionPid.deadzone = value;
        break;
    case PARAM_PPID_Umax:
        node->motorController->positionPid.u_max = value;
        break;
    case PARAM_PPID_dUmax:
        node->motorController->positionPid.du_max = value;
        break;
    case PARAM_VPID_Kp:
        node->motorController->velocityPid.Kp = value;
        break;
    case PARAM_VPID_Ki:
        node->motorController->velocityPid.Ki = value;
        break;
    case PARAM_VPID_Kd:
        node->motorController->velocityPid.Kd = value;
        break;
    case PARAM_VPID_Kaw:
        node->motorController->velocityPid.Kaw = value;
        break;
    case PARAM_VPID_deadzone:
        node->motorController->velocityPid.deadzone = value;
        break;
    case PARAM_VPID_Umax:
        node->motorController->velocityPid.u_max = value;
        break;
    case PARAM_VPID_dUmax:
        node->motorController->velocityPid.du_max = value;
        break;
    case PARAM_CPID_Kp:
        node->motorController->currentPid.Kp = value;
        break;
    case PARAM_CPID_Ki:
        node->motorController->currentPid.Ki = value;
        break;
    case PARAM_CPID_Kd:
        node->motorController->currentPid.Kd = value;
        break;
    case PARAM_CPID_Kaw:
        node->motorController->currentPid.Kaw = value;
        break;
    case PARAM_CPID_deadzone:
        node->motorController->currentPid.deadzone = value;
        break;
    case PARAM_CPID_Umax:
        node->motorController->currentPid.u_max = value;
        break;
    case PARAM_CPID_dUmax:
        node->motorController->currentPid.du_max = value;
        break;
    case PARAM_MIN_POSITION:
        node->motorController->params.minPosition = value;
        break;
    case PARAM_MAX_POSITION:
        node->motorController->params.maxPosition = value;
        break;
    case PARAM_MIN_VELOCITY:
        node->motorController->params.minVelocity = value;
        break;
    case PARAM_MAX_VELOCITY:
        node->motorController->params.maxVelocity = value;
        break;
    case PARAM_MIN_CURRENT:
        node->motorController->params.minCurrent = value;
        break;
    case PARAM_MAX_CURRENT:
        node->motorController->params.maxCurrent = value;
        break;
    case PARAM_MIN_DUTY:
        node->motorController->params.minDuty = value;
        break;
    case PARAM_MAX_DUTY:
        node->motorController->params.maxDuty = value;
        break;
    default:
        break;
    }
}

float RoboszponNode_ReadParam(roboszpon_node_t* node, uint8_t paramId) {
    switch (paramId) {
    case PARAM_COMMAND_TIMEOUT:
        return ((float)node->commandTimeout) / 1000.0f;
    case PARAM_ENCODER_OFFSET:
        // TODO: implement encoder offset
        return 0.0;
    case PARAM_PPID_Kp:
        return node->motorController->positionPid.Kp;
    case PARAM_PPID_Ki:
        return node->motorController->positionPid.Ki;
    case PARAM_PPID_Kd:
        return node->motorController->positionPid.Kd;
    case PARAM_PPID_Kaw:
        return node->motorController->positionPid.Kaw;
    case PARAM_PPID_deadzone:
        return node->motorController->positionPid.deadzone;
    case PARAM_PPID_Umax:
        return node->motorController->positionPid.u_max;
    case PARAM_PPID_dUmax:
        return node->motorController->positionPid.du_max;
    case PARAM_VPID_Kp:
        return node->motorController->velocityPid.Kp;
    case PARAM_VPID_Ki:
        return node->motorController->velocityPid.Ki;
    case PARAM_VPID_Kd:
        return node->motorController->velocityPid.Kd;
    case PARAM_VPID_Kaw:
        return node->motorController->velocityPid.Kaw;
    case PARAM_VPID_deadzone:
        return node->motorController->velocityPid.deadzone;
    case PARAM_VPID_Umax:
        return node->motorController->velocityPid.u_max;
    case PARAM_VPID_dUmax:
        return node->motorController->velocityPid.du_max;
    case PARAM_CPID_Kp:
        return node->motorController->currentPid.Kp;
    case PARAM_CPID_Ki:
        return node->motorController->currentPid.Ki;
    case PARAM_CPID_Kd:
        return node->motorController->currentPid.Kd;
    case PARAM_CPID_Kaw:
        return node->motorController->currentPid.Kaw;
    case PARAM_CPID_deadzone:
        return node->motorController->currentPid.deadzone;
    case PARAM_CPID_Umax:
        return node->motorController->currentPid.u_max;
    case PARAM_CPID_dUmax:
        return node->motorController->currentPid.du_max;
    case PARAM_MIN_POSITION:
        return node->motorController->params.minPosition;
    case PARAM_MAX_POSITION:
        return node->motorController->params.maxPosition;
    case PARAM_MIN_VELOCITY:
        return node->motorController->params.minVelocity;
    case PARAM_MAX_VELOCITY:
        return node->motorController->params.maxVelocity;
    case PARAM_MIN_CURRENT:
        return node->motorController->params.minCurrent;
    case PARAM_MAX_CURRENT:
        return node->motorController->params.maxCurrent;
    case PARAM_MIN_DUTY:
        return node->motorController->params.minDuty;
    case PARAM_MAX_DUTY:
        return node->motorController->params.maxDuty;
    default:
        return 0.0f;
    }
}