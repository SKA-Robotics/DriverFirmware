#ifndef ROBOSZPON_MESSAGE_H
#define ROBOSZPON_MESSAGE_H

#include <stdint.h>

#define NODEID_BROADCAST 0x00

#define MSG_EMERGENCY_STOP 0x00
#define MSG_MOTOR_COMMAND 0x01
#define MSG_ACTION_REQUEST 0x02
#define MSG_STATUS_REPORT 0x03
#define MSG_AXIS_REPORT 0x04
#define MSG_MOTOR_REPORT 0x05
#define MSG_PARAMETER_WRITE 0x06
#define MSG_PARAMETER_READ 0x07
#define MSG_PARAMETER_RESPONSE 0x08

#define ACTION_ARM 0x00
#define ACTION_DISARM 0x01
#define ACTION_SET_AXIS_ZERO_HERE 0x02
#define ACTION_COMMIT_CONFIG 0x03

#define PARAM_COMMAND_TIMEOUT 0x00
#define PARAM_ENCODER_OFFSET 0x01
#define PARAM_AXIS_OFFSET 0x02
// and many more ...

#define CTRLSIGNAL_DUTY 0x00
#define CTRLSIGNAL_VELOCITY 0x01
#define CTRLSIGNAL_POSITION 0x02

typedef struct {
    uint8_t nodeId;
    uint8_t id;
    uint64_t data;
} roboszpon_message_t;

roboszpon_message_t interpretCanMessage(uint32_t frameId, uint8_t dataLength,
                                        uint8_t* data);

typedef struct {
    uint8_t controlSignalId;
    float value;
} message_motor_command_t;

message_motor_command_t
interpretMotorCommandMessage(roboszpon_message_t* message);

typedef struct {
    uint8_t actionId;
} message_action_request_t;

message_action_request_t
interpretActionRequestMessage(roboszpon_message_t* message);

#endif // ROBOSZPON_MESSAGE_H