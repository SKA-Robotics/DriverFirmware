#include "roboszpon_config.h"
#include "firmware_settings.h"
#include <math.h>

void RoboszponConfig_LoadDefault(roboszpon_node_t* node) {
    RoboszponConfig_WriteParam(node, PARAM_COMMAND_TIMEOUT,
                               DEFAULT_COMMAND_TIMEOUT);
    RoboszponConfig_WriteParam(node, PARAM_ENCODER_ZERO, DEFAULT_ENCODER_ZERO);
    RoboszponConfig_WriteParam(node, PARAM_AXIS_OFFSET, DEFAULT_AXIS_OFFSET);
    RoboszponConfig_WriteParam(node, PARAM_REPORT_RATE, DEFAULT_REPORT_RATE);
    RoboszponConfig_WriteParam(node, PARAM_PPID_Kp, DEFAULT_PPID_Kp);
    RoboszponConfig_WriteParam(node, PARAM_PPID_Ki, DEFAULT_PPID_Ki);
    RoboszponConfig_WriteParam(node, PARAM_PPID_Kd, DEFAULT_PPID_Kd);
    RoboszponConfig_WriteParam(node, PARAM_PPID_deadzone,
                               DEFAULT_PPID_deadzone);
    RoboszponConfig_WriteParam(node, PARAM_PPID_dUmax, DEFAULT_PPID_dUmax);
    RoboszponConfig_WriteParam(node, PARAM_VPID_Kp, DEFAULT_VPID_Kp);
    RoboszponConfig_WriteParam(node, PARAM_VPID_Ki, DEFAULT_VPID_Ki);
    RoboszponConfig_WriteParam(node, PARAM_VPID_Kd, DEFAULT_VPID_Kd);
    RoboszponConfig_WriteParam(node, PARAM_VPID_deadzone,
                               DEFAULT_VPID_deadzone);
    RoboszponConfig_WriteParam(node, PARAM_VPID_dUmax, DEFAULT_VPID_dUmax);
    RoboszponConfig_WriteParam(node, PARAM_CPID_Kp, DEFAULT_CPID_Kp);
    RoboszponConfig_WriteParam(node, PARAM_CPID_Ki, DEFAULT_CPID_Ki);
    RoboszponConfig_WriteParam(node, PARAM_CPID_Kd, DEFAULT_CPID_Kd);
    RoboszponConfig_WriteParam(node, PARAM_CPID_deadzone,
                               DEFAULT_CPID_deadzone);
    RoboszponConfig_WriteParam(node, PARAM_CPID_dUmax, DEFAULT_CPID_dUmax);
    RoboszponConfig_WriteParam(node, PARAM_CURRENT_FeedForward,
                               DEFAULT_CURRENT_FeedForward);
    RoboszponConfig_WriteParam(node, PARAM_IIR_VALUE_CURMEAS,
                               DEFAULT_IIR_VALUE_CURMEAS);
    RoboszponConfig_WriteParam(node, PARAM_IIR_VALUE_VELMEAS,
                               DEFAULT_IIR_VALUE_VELMEAS);
    RoboszponConfig_WriteParam(node, PARAM_IIR_VALUE_PPIDU,
                               DEFAULT_IIR_VALUE_PPIDU);
    RoboszponConfig_WriteParam(node, PARAM_IIR_VALUE_VPIDU,
                               DEFAULT_IIR_VALUE_VPIDU);
    RoboszponConfig_WriteParam(node, PARAM_IIR_VALUE_CPIDU,
                               DEFAULT_IIR_VALUE_CPIDU);
    RoboszponConfig_WriteParam(node, PARAM_MIN_POSITION, DEFAULT_MIN_POSITION);
    RoboszponConfig_WriteParam(node, PARAM_MAX_POSITION, DEFAULT_MAX_POSITION);
    RoboszponConfig_WriteParam(node, PARAM_MIN_VELOCITY, DEFAULT_MIN_VELOCITY);
    RoboszponConfig_WriteParam(node, PARAM_MAX_VELOCITY, DEFAULT_MAX_VELOCITY);
    RoboszponConfig_WriteParam(node, PARAM_MIN_CURRENT, DEFAULT_MIN_CURRENT);
    RoboszponConfig_WriteParam(node, PARAM_MAX_CURRENT, DEFAULT_MAX_CURRENT);
    RoboszponConfig_WriteParam(node, PARAM_MIN_DUTY, DEFAULT_MIN_DUTY);
    RoboszponConfig_WriteParam(node, PARAM_MAX_DUTY, DEFAULT_MAX_DUTY);
    RoboszponConfig_WriteParam(node, PARAM_OVERHEAT_TEMPERATURE,
                               DEFAULT_OVERHEAT_TEMPERATURE);
    RoboszponConfig_WriteParam(node, PARAM_NO_OVERHEAT_TEMPERATURE,
                               DEFAULT_NO_OVERHEAT_TEMPERATURE);
    RoboszponConfig_WriteParam(node, PARAM_INVERT_AXIS, DEFAULT_INVERT_AXIS);
    RoboszponConfig_WriteParam(node, PARAM_INVERT_ENCODER,
                               DEFAULT_INVERT_ENCODER);
    RoboszponConfig_WriteParam(node, PARAM_ENCODER_FILTER_WINDOW,
                               DEFAULT_ENCODER_FILTER_WINDOW);
}

void RoboszponConfig_WriteParam(roboszpon_node_t* node, uint8_t paramId,
                                float value) {
    switch (paramId) {
    case PARAM_COMMAND_TIMEOUT:
        node->commandTimeout = (uint32_t)(value * 1000.0f);
        break;
    case PARAM_ENCODER_ZERO:
        value = fmodf(value, 1.0f);
        MA730_SetZero(node->motor->encoder, value);
        break;
    case PARAM_AXIS_OFFSET:
        node->motor->positionOffset = value;
        break;
    case PARAM_REPORT_RATE: {
        uint32_t period = roundf(500.0f / value);
        if (period < 1) {
            period = 1;
        }
        node->reportPeriod = period;
    } break;
    case PARAM_PPID_Kp:
        node->motorController->positionPid.Kp = value;
        break;
    case PARAM_PPID_Ki:
        node->motorController->positionPid.Ki = value;
        break;
    case PARAM_PPID_Kd:
        node->motorController->positionPid.Kd = value;
        break;
    case PARAM_PPID_deadzone:
        node->motorController->positionPid.deadzone = value;
        break;
    case PARAM_PPID_dUmax:
        node->motorController->positionPid.duMax = value;
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
    case PARAM_VPID_deadzone:
        node->motorController->velocityPid.deadzone = value;
        break;
    case PARAM_VPID_dUmax:
        node->motorController->velocityPid.duMax = value;
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
    case PARAM_CPID_deadzone:
        node->motorController->currentPid.deadzone = value;
        break;
    case PARAM_CPID_dUmax:
        node->motorController->currentPid.duMax = value;
        break;
    case PARAM_CURRENT_FeedForward:
        node->motorController->params.currentFeedforward = value;
        break;
    case PARAM_IIR_VALUE_CURMEAS:
        node->motor->currentMeasurementFilter.coefficient = value;
        break;
    case PARAM_IIR_VALUE_VELMEAS:
        node->motor->velocityMeasurementFilter.coefficient = value;
        break;
    case PARAM_IIR_VALUE_PPIDU:
        node->motorController->positionPidOutputFilter.coefficient = value;
        break;
    case PARAM_IIR_VALUE_VPIDU:
        node->motorController->velocityPidOutputFilter.coefficient = value;
        break;
    case PARAM_IIR_VALUE_CPIDU:
        node->motorController->currentPidOutputFilter.coefficient = value;
        break;
    case PARAM_MIN_POSITION:
        node->motorController->params.minPosition = value;
        break;
    case PARAM_MAX_POSITION:
        node->motorController->params.maxPosition = value;
        break;
    case PARAM_MIN_VELOCITY:
        node->motorController->params.minVelocity = value;
        node->motorController->positionPid.uMin = value;
        break;
    case PARAM_MAX_VELOCITY:
        node->motorController->params.maxVelocity = value;
        node->motorController->positionPid.uMax = value;
        break;
    case PARAM_MIN_CURRENT:
        node->motorController->params.minCurrent = value;
        node->motorController->velocityPid.uMin = value;
        break;
    case PARAM_MAX_CURRENT:
        node->motorController->params.maxCurrent = value;
        node->motorController->velocityPid.uMax = value;
        break;
    case PARAM_MIN_DUTY:
        node->motorController->params.minDuty = value;
        node->motorController->currentPid.uMin = value;
        break;
    case PARAM_MAX_DUTY:
        node->motorController->params.maxDuty = value;
        node->motorController->currentPid.uMax = value;
        break;
    case PARAM_OVERHEAT_TEMPERATURE:
        node->overheatThreshold = (uint32_t)(value * 10.0f);
        break;
    case PARAM_NO_OVERHEAT_TEMPERATURE:
        node->overheatResetThreshold = (uint32_t)(value * 10.0f);
        break;
    case PARAM_INVERT_AXIS:
        node->motor->invertAxis = (value > 0.5f);
        break;
    case PARAM_INVERT_ENCODER:
        MA730_SetRotationDirection(node->motor->encoder,
                                   (value > 0.5f) ? MA730_DIRECTION_REVERSE
                                                  : MA730_DIRECTION_FORWARD);
        break;
    case PARAM_ENCODER_FILTER_WINDOW:
        MA730_SetFilterWindow(node->motor->encoder, value);
        break;
    default:
        break;
    }
}

float RoboszponConfig_ReadParam(roboszpon_node_t* node, uint8_t paramId) {
    switch (paramId) {
    case PARAM_COMMAND_TIMEOUT:
        return ((float)node->commandTimeout) / 1000.0f;
    case PARAM_ENCODER_ZERO:
        return MA730_GetZero(node->motor->encoder);
    case PARAM_AXIS_OFFSET:
        return node->motor->positionOffset;
    case PARAM_REPORT_RATE:
        return 500.0f / (float)node->reportPeriod;
    case PARAM_PPID_Kp:
        return node->motorController->positionPid.Kp;
    case PARAM_PPID_Ki:
        return node->motorController->positionPid.Ki;
    case PARAM_PPID_Kd:
        return node->motorController->positionPid.Kd;
    case PARAM_PPID_deadzone:
        return node->motorController->positionPid.deadzone;
    case PARAM_PPID_dUmax:
        return node->motorController->positionPid.duMax;
    case PARAM_VPID_Kp:
        return node->motorController->velocityPid.Kp;
    case PARAM_VPID_Ki:
        return node->motorController->velocityPid.Ki;
    case PARAM_VPID_Kd:
        return node->motorController->velocityPid.Kd;
    case PARAM_VPID_deadzone:
        return node->motorController->velocityPid.deadzone;
    case PARAM_VPID_dUmax:
        return node->motorController->velocityPid.duMax;
    case PARAM_CURRENT_FeedForward:
        return node->motorController->params.currentFeedforward;
    case PARAM_IIR_VALUE_CURMEAS:
        return node->motor->currentMeasurementFilter.coefficient;
    case PARAM_IIR_VALUE_VELMEAS:
        return node->motor->velocityMeasurementFilter.coefficient;
    case PARAM_IIR_VALUE_PPIDU:
        return node->motorController->positionPidOutputFilter.coefficient;
    case PARAM_IIR_VALUE_VPIDU:
        return node->motorController->velocityPidOutputFilter.coefficient;
    case PARAM_IIR_VALUE_CPIDU:
        return node->motorController->currentPidOutputFilter.coefficient;
    case PARAM_CPID_Kp:
        return node->motorController->currentPid.Kp;
    case PARAM_CPID_Ki:
        return node->motorController->currentPid.Ki;
    case PARAM_CPID_Kd:
        return node->motorController->currentPid.Kd;
    case PARAM_CPID_deadzone:
        return node->motorController->currentPid.deadzone;
    case PARAM_CPID_dUmax:
        return node->motorController->currentPid.duMax;
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
    case PARAM_OVERHEAT_TEMPERATURE:
        return (float)node->overheatThreshold * 0.1f;
    case PARAM_NO_OVERHEAT_TEMPERATURE:
        return (float)node->overheatResetThreshold * 0.1f;
    case PARAM_INVERT_AXIS:
        return (node->motor->invertAxis) ? 1.0f : 0.0f;
    case PARAM_INVERT_ENCODER:
        return (MA730_GetRotationDirection(node->motor->encoder) ==
                MA730_DIRECTION_REVERSE)
                   ? 1.0f
                   : 0.0f;
    case PARAM_ENCODER_FILTER_WINDOW:
        return MA730_GetFilterWindow(node->motor->encoder);
    default:
        return 0.0f;
    }
}