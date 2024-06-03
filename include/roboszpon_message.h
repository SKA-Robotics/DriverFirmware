#ifndef ROBOSZPON_MESSAGE_H
#define ROBOSZPON_MESSAGE_H

#include <stdint.h>

#define NODEID_BROADCAST 0x00
#define EMERGENCY_STOP_ARBITRATION_ID 0x001

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
#define ACTION_COMMIT_CONFIG 0x02
#define ACTION_RESTORE_CONFIG 0x03
#define ACTION_SET_FACTORY_CONFIG 0x04

#define PARAM_COMMAND_TIMEOUT 0x00
#define PARAM_ENCODER_ZERO 0x01
#define PARAM_AXIS_OFFSET 0x02
#define PARAM_PPID_Kp 0x04
#define PARAM_PPID_Ki 0x05
#define PARAM_PPID_Kd 0x06
#define PARAM_PPID_deadzone 0x08
#define PARAM_PPID_dUmax 0x0a
#define PARAM_VPID_Kp 0x0c
#define PARAM_VPID_Ki 0x0d
#define PARAM_VPID_Kd 0x0e
#define PARAM_VPID_deadzone 0x10
#define PARAM_VPID_dUmax 0x12
#define PARAM_CPID_Kp 0x14
#define PARAM_CPID_Ki 0x15
#define PARAM_CPID_Kd 0x16
#define PARAM_CPID_deadzone 0x18
#define PARAM_CPID_dUmax 0x1a
#define PARAM_IIR_VALUE_CURMEAS 0x1c
#define PARAM_IIR_VALUE_VELMEAS 0x1d
#define PARAM_IIR_VALUE_PPIDU 0x1e
#define PARAM_IIR_VALUE_VPIDU 0x1f
#define PARAM_IIR_VALUE_CPIDU 0x20
#define PARAM_MIN_POSITION 0x23
#define PARAM_MAX_POSITION 0x24
#define PARAM_MIN_VELOCITY 0x25
#define PARAM_MAX_VELOCITY 0x26
#define PARAM_MIN_CURRENT 0x27
#define PARAM_MAX_CURRENT 0x28
#define PARAM_MIN_DUTY 0x29
#define PARAM_MAX_DUTY 0x2f
#define PARAM_OVERHEAT_TEMPERATURE 0x50
#define PARAM_NO_OVERHEAT_TEMPERATURE 0x51
#define PARAM_INVERT_AXIS 0x52
#define PARAM_INVERT_ENCODER 0x53
#define PARAM_ENCODER_FILTER_WINDOW 0x54
// and many more ... Don't exceed 0x7f, as it wouldn't fit in the flash
#define MIN_PARAM_ADDRESS 0x00
#define MAX_PARAM_ADDRESS 0x7f

#define CTRLSIGNAL_DUTY 0x00
#define CTRLSIGNAL_VELOCITY 0x01
#define CTRLSIGNAL_POSITION 0x02

typedef struct {
    uint8_t nodeId;
    uint8_t id;
    uint64_t data;
} roboszpon_message_t;

roboszpon_message_t DecodeCanMessage(uint32_t arbitrationId, uint8_t dataLength,
                                     uint8_t* data);

typedef struct {
    uint8_t controlSignalId;
    float value;
} message_motor_command_t;

message_motor_command_t ParseMessage_MotorCommand(roboszpon_message_t* message);

typedef struct {
    uint8_t actionId;
} message_action_request_t;

message_action_request_t
ParseMessage_ActionRequest(roboszpon_message_t* message);

typedef struct {
    uint8_t paramId;
    float value;
} message_parameter_write_t;

message_parameter_write_t
ParseMessage_ParameterWrite(roboszpon_message_t* message);

typedef struct {
    uint8_t paramId;
} message_parameter_read_t;

message_parameter_read_t
ParseMessage_ParameterRead(roboszpon_message_t* message);

#endif // ROBOSZPON_MESSAGE_H