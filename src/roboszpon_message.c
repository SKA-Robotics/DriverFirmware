#include "roboszpon_message.h"

roboszpon_message_t interpretCanMessage(uint32_t frameId, uint8_t dataLength,
                                        uint8_t* data) {
    roboszpon_message_t message = {0};
    if (frameId == 0x01) { // Special code for emergency stop
        message.nodeId = 0;
        message.id = MSG_EMERGENCY_STOP;
        message.data = 0;
        return message;
    }
    message.nodeId = (frameId & 0b11111000000) >> 6;
    message.id = frameId & 0b00000111111;
    for (int i = 0; i < dataLength; i++) {
        message.data = message.data << 8;
        message.data |= data[i];
    }
    printf("Message Received: ");
    printf("NodeID %#02x, MessageID %#02x, ", message.nodeId, message.id);
    printf("Data: ", frameId);
    printf("%#02x ", data[0]);
    printf("%#02x ", data[1]);
    printf("%#02x ", data[2]);
    printf("%#02x ", data[3]);
    printf("%#02x ", data[4]);
    printf("%#02x ", data[5]);
    printf("%#02x ", data[6]);
    printf("%#02x\n", data[7]);
    return message;
}

message_motor_command_t
interpretMotorCommandMessage(roboszpon_message_t* message) {
    message_motor_command_t result;
    result.controlSignalId = (message->data >> 56) & 0xff;
    uint32_t valueBits = (message->data >> 24) & 0xffffffff;
    result.value = *(float*)&valueBits;
    return result;
}

message_action_request_t
interpretActionRequestMessage(roboszpon_message_t* message) {
    message_action_request_t result;
    result.actionId = message->data;
    return result;
}