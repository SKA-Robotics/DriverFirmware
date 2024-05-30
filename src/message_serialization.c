#include "message_serialization.h"
#include "system.h"

uint16_t buildFrameId(uint8_t nodeId, uint8_t messageId);

void sendStatusReportMessage(roboszpon_axis_t* axis) {
    uint16_t frameId = buildFrameId(axis->nodeId, MSG_STATUS_REPORT);
    uint64_t flags =
        axis->flags &
        0x3FFFFFFFFFFFFFFF; // Make sure two most significant bits are free
    uint8_t mode = axis->state;
    uint64_t data = ((uint64_t)mode) << 62 | flags;
    if (transmitCanFrame(frameId, data, CAN_TX_TIMEOUT) != HAL_OK) {
        printf("CAN error\n"); // TODO: Create can error handler.
    }
}

void sendAxisReportMessage(roboszpon_axis_t* axis) {
    uint16_t frameId = buildFrameId(axis->nodeId, MSG_AXIS_REPORT);
    uint32_t position = *(uint32_t*)&axis->motor->state.position;
    uint32_t velocity = *(uint32_t*)&axis->motor->state.velocity;
    uint64_t data = (uint64_t)position << 32 | velocity;
    if (transmitCanFrame(frameId, data, CAN_TX_TIMEOUT) != HAL_OK) {
        printf("CAN error\n");
    }
}

void sendMotorReportMessage(roboszpon_axis_t* axis) {
    uint16_t frameId = buildFrameId(axis->nodeId, MSG_MOTOR_REPORT);
    uint32_t current = *(uint32_t*)&axis->motor->state.current;
    uint32_t duty = *(uint32_t*)&axis->motor->state.duty;
    uint64_t data = (uint64_t)current << 32 | duty;
    if (transmitCanFrame(frameId, data, CAN_TX_TIMEOUT) != HAL_OK) {
        printf("CAN error\n");
    }
}

void sendParameterResponseMessage(uint8_t nodeId, uint8_t paramId,
                                  float value) {
    uint16_t frameId = buildFrameId(nodeId, MSG_PARAMETER_RESPONSE);
    uint32_t valueBits = *(uint32_t*)&value;
    uint64_t data = ((uint64_t)paramId << 56) | ((uint64_t)valueBits << 24);
    if (transmitCanFrame(frameId, data, CAN_TX_TIMEOUT) != HAL_OK) {
        printf("CAN error\n");
    }
}

uint16_t buildFrameId(uint8_t nodeId, uint8_t messageId) {
    uint16_t result = (uint16_t)(nodeId & 0b11111) << 6;
    result |= messageId & 0b111111;
    return result;
}