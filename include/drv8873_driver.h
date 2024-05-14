#ifndef DRV8873_DRIVER
#define DRV8873_DRIVER

#include "system.h"

uint8_t DRV8873_ReadRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                             uint8_t registerAddress);

// void DRV8873_WriteRegister(GPIO_TypeDef* csPort, uint16_t csPin,
//                            uint8_t registerAddress, uint8_t value);

#endif // DRV8873_DRIVER