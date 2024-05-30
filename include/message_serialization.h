#ifndef MESSAGE_SERIALIZATION_H
#define MESSAGE_SERIALIZATION_H

#include "roboszpon_message.h"
#include "roboszpon_node.h"

void SendMessage_StatusReport(roboszpon_node_t* node);

void SendMessage_AxisReport(roboszpon_node_t* node);

void SendMessage_MotorReport(roboszpon_node_t* node);

void SendMessage_ParameterResponse(uint8_t nodeId, uint8_t paramId,
                                   float value);

#endif // MESSAGE_SERIALIZATION_H