#ifndef MESSAGE_SERIALIZATION_H
#define MESSAGE_SERIALIZATION_H

#include "roboszpon_message.h"
#include "roboszpon_node.h"

void sendStatusReportMessage(roboszpon_axis_t* axis);

void sendAxisReportMessage(roboszpon_axis_t* axis);

void sendMotorReportMessage(roboszpon_axis_t* axis);

void sendParameterResponseMessage(uint8_t nodeId, uint8_t paramId, float value);

#endif // MESSAGE_SERIALIZATION_H