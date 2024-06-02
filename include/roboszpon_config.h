#ifndef ROBOSZPON_CONFIG_H
#define ROBOSZPON_CONFIG_H

#include "roboszpon_node.h"

void RoboszponConfig_LoadDefault(roboszpon_node_t* node);

void RoboszponConfig_WriteParam(roboszpon_node_t* node, uint8_t paramId,
                                float value);

float RoboszponConfig_ReadParam(roboszpon_node_t* node, uint8_t paramId);

#endif // ROBOSZPON_CONFIG_H