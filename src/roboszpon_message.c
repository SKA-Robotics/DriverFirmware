#include "roboszpon_message.h"

roboszpon_message_t DecodeCanMessage(uint32_t arbitrationId, uint8_t dataLength,
                                     uint8_t* data) {
    roboszpon_message_t message = {0};
    if (arbitrationId ==
        EMERGENCY_STOP_ARBITRATION_ID) { // Special code for emergency stop
        message.nodeId = 0;
        message.id = MSG_EMERGENCY_STOP;
        message.data = 0;
        return message;
    }
    // 5 MSB are the node ID. 6 LSB are the message ID.
    message.nodeId = (arbitrationId & 0b11111000000) >> 6;
    message.id = arbitrationId & 0b00000111111;
    for (int i = 0; i < dataLength; i++) {
        message.data = message.data << 8;
        message.data |= data[i];
    }
    return message;
}

message_motor_command_t
ParseMessage_MotorCommand(roboszpon_message_t* message) {
    message_motor_command_t result;
    result.controlSignalId = (message->data >> 56) & 0xff;
    uint32_t valueBits = (message->data >> 24) & 0xffffffff;
    result.value = *(float*)&valueBits;
    return result;
}

message_action_request_t
ParseMessage_ActionRequest(roboszpon_message_t* message) {
    message_action_request_t result;
    result.actionId = message->data;
    return result;
}

message_parameter_write_t
ParseMessage_ParameterWrite(roboszpon_message_t* message) {
    message_parameter_write_t result;
    result.paramId = (message->data >> 56) & 0xff;
    uint32_t valueBits = (message->data >> 24) & 0xffffffff;
    result.value = *(float*)&valueBits;
    return result;
}

message_parameter_read_t
ParseMessage_ParameterRead(roboszpon_message_t* message) {
    message_parameter_read_t result;
    result.paramId = message->data;
    return result;
}