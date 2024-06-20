#include "flash_memory.h"
#include "ma730_driver.h"
#include "message_serialization.h"
#include "ntc_driver.h"
#include "roboszpon_config.h"
#include "roboszpon_message.h"
#include <math.h>

void RoboszponNode_UpdateFlags(roboszpon_node_t* node);
void RoboszponNode_StoppedStep(roboszpon_node_t* node);
void RoboszponNode_RunningStep(roboszpon_node_t* node);
void RoboszponNode_ErrorStep(roboszpon_node_t* node);
void RoboszponNode_StopMotor(roboszpon_node_t* node);
void RoboszponNode_PerformActionStopped(roboszpon_node_t* node,
                                        uint8_t actionId);
void RoboszponNode_ApplyMotorCommand(roboszpon_node_t* node,
                                     roboszpon_message_t* command);
void RoboszponConfig_WriteParam(roboszpon_node_t* node, uint8_t paramId,
                                float value);
float RoboszponConfig_ReadParam(roboszpon_node_t* node, uint8_t paramId);

void RoboszponNode_Report(roboszpon_node_t* node) {
    SendMessage_StatusReport(node);
    if (node->state == ROBOSZPON_NODE_STATE_RUNNING) {
        SendMessage_AxisReport(node);
        SendMessage_MotorReport(node);
    }
}

void RoboszponNode_Step(roboszpon_node_t* node) {
    RoboszponNode_UpdateFlags(node);
    switch (node->state) {
    case ROBOSZPON_NODE_STATE_STOPPED:
        RoboszponNode_StoppedStep(node);
        return;
    case ROBOSZPON_NODE_STATE_RUNNING:
        RoboszponNode_RunningStep(node);
        return;
    case ROBOSZPON_NODE_STATE_ERROR:
        RoboszponNode_ErrorStep(node);
        return;
    default:
        return;
    }
}

void RoboszponNode_UpdateFlags(roboszpon_node_t* node) {
    // 1. Check for encoder errors
    uint8_t error =
        (node->disableEncoderErrors) ? 0 : MA730_GetError(node->motor->encoder);
    node->flags &= ~ROBOSZPON_ENC_ERROR_MASK;
    node->flags |= (error & ROBOSZPON_ENC_ERROR_MASK);
    // 2. Check for drv errors
    error = DRV8873_GetError(node->drv8873);
    node->flags &= ~ROBOSZPON_DRV_ERROR_MASK;
    node->flags |= (((uint16_t)error << 8) & ROBOSZPON_DRV_ERROR_MASK);
    // 3. Check for overheating / reset overheating error
    node->temperature = NTC_ADC2Temperature(ReadAdc(ADC_THERMISTOR));
    if (node->flags & ROBOSZPON_ERROR_OVERHEAT) {
        if (node->temperature < node->overheatResetThreshold) {
            node->flags &= ~ROBOSZPON_ERROR_OVERHEAT;
        }
    } else {
        if (node->temperature > node->overheatThreshold) {
            node->flags |= ROBOSZPON_ERROR_OVERHEAT;
        }
    }
    // 4. Check for command timeout
    uint8_t timedOut =
        (node->latestCommandTime + node->commandTimeout < HAL_GetTick());
    if (timedOut && node->state == ROBOSZPON_NODE_STATE_RUNNING) {
        node->flags |= ROBOSZPON_ERROR_CMDTIMEOUT;
    } else {
        node->flags &= ~ROBOSZPON_ERROR_CMDTIMEOUT;
    }
}

void RoboszponNode_StoppedStep(roboszpon_node_t* node) {
    roboszpon_message_t message;
    while (!MessageQueue_IsEmpty(node->messageQueue)) {
        MessageQueue_Dequeue(node->messageQueue, &message);
        switch (message.id) {
        case MSG_PARAMETER_WRITE: {
            message_parameter_write_t param =
                ParseMessage_ParameterWrite(&message);
            RoboszponConfig_WriteParam(node, param.paramId, param.value);
        } break;
        case MSG_PARAMETER_READ: {
            message_parameter_read_t request =
                ParseMessage_ParameterRead(&message);
            float value = RoboszponConfig_ReadParam(node, request.paramId);
            SendMessage_ParameterResponse(node->nodeId, request.paramId, value);
        } break;
        case MSG_ACTION_REQUEST: {
            uint8_t actionId = ParseMessage_ActionRequest(&message).actionId;
            RoboszponNode_PerformActionStopped(node, actionId);
        } break;
        default:
            // Ignore all the other messages
            break;
        }
    }
}

void RoboszponNode_RunningStep(roboszpon_node_t* node) {
    if (node->flags != ROBOSZPON_NO_ERROR) { // Error detected
        RoboszponNode_StopMotor(node);
        if ((node->flags & ERROR_MASK) != ROBOSZPON_NO_ERROR) { // Crit. error
            HAL_GPIO_WritePin(node->errorLedPort, node->errorLedPin,
                              GPIO_PIN_SET);
            node->state = ROBOSZPON_NODE_STATE_ERROR;
        }
    }
    int isThereAMotorCommand = 0;
    roboszpon_message_t message;
    roboszpon_message_t latestMotorCommand;
    while (!MessageQueue_IsEmpty(node->messageQueue)) {
        MessageQueue_Dequeue(node->messageQueue, &message);
        switch (message.id) {
        case MSG_EMERGENCY_STOP:
            RoboszponNode_StopMotor(node);
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
                RoboszponNode_StopMotor(node);
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
    if ((node->flags & ERROR_MASK) == ROBOSZPON_NO_ERROR) {
        // All errors resolved
        HAL_GPIO_WritePin(node->errorLedPort, node->errorLedPin,
                          GPIO_PIN_RESET);
        node->state = ROBOSZPON_NODE_STATE_RUNNING;
        node->latestCommandTime = HAL_GetTick();
    }
}

void RoboszponNode_StopMotor(roboszpon_node_t* node) {
    MotorController_SetPositionSetpoint(node->motorController, 0.0f);
    MotorController_SetVelocitySetpoint(node->motorController, 0.0f);
    MotorController_SetDutySetpoint(node->motorController, 0.0f);
    Motor_SetDuty(node->motor, 0.0f);
}

void RoboszponNode_PerformActionStopped(roboszpon_node_t* node,
                                        uint8_t actionId) {
    switch (actionId) {
    case ACTION_ARM:
        node->state = ROBOSZPON_NODE_STATE_RUNNING;
        node->latestCommandTime = HAL_GetTick();
        break;
    case ACTION_COMMIT_CONFIG:
        Flash_SaveNodeConfig(node->configAddress, node);
        break;
    case ACTION_RESTORE_CONFIG:
        Flash_LoadNodeConfig(node->configAddress, node);
        break;
    case ACTION_SET_FACTORY_CONFIG:
        RoboszponConfig_LoadDefault(node);
        Flash_SaveNodeConfig(node->configAddress, node);
        break;
    case ACTION_SOFTWARE_RESET:
        NVIC_SystemReset();
        break;
    default:
        break;
    }
}

void RoboszponNode_ApplyMotorCommand(roboszpon_node_t* node,
                                     roboszpon_message_t* command) {
    node->latestCommandTime = HAL_GetTick();
    node->flags &= !ROBOSZPON_ERROR_CMDTIMEOUT;
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
    default:
        break;
    }
}
