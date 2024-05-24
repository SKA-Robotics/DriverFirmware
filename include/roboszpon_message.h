#ifndef ROBOSZPON_MESSAGE_H
#define ROBOSZPON_MESSAGE_H

#include <stdint.h>

typedef struct {
    uint8_t id;
    uint64_t data;
} roboszpon_message_t;

#define MSG_STATUS_REPORT 0x01
#define MSG_AXIS_REPORT 0x02
#define MSG_MOTOR_REPORT 0x03
#define MSG_ACTION_REQUEST 0x04
#define MSG_PARAMETER_WRITE 0x05
#define MSG_PARAMETER_READ 0x06
#define MSG_PARAMETER_RESPONSE 0x07

#define ACTION_ARM 0x00
#define ACTION_DISARM 0x01
#define ACTION_SET_AXIS_ZERO_HERE 0x02

#define PARAM_COMMAND_TIMEOUT 0x00
#define PARAM_ENCODER_OFFSET 0x01
#define PARAM_AXIS_OFFSET 0x02
// and many more ...

#endif // ROBOSZPON_MESSAGE_H