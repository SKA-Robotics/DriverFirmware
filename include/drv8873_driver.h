#ifndef DRV8873_DRIVER
#define DRV8873_DRIVER

#include "system.h"

uint16_t DRV8873_ReadRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                              uint8_t registerAddress);

#endif // DRV8873_DRIVER