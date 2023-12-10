#ifndef MAG_ALPHA_DRIVER
#define MAG_ALPHA_DRIVER

#include "system.h"

uint16_t readMagAlphaAngle(GPIO_TypeDef* csPort, uint16_t csPin);

#endif // MAG_ALPHA_DRIVER