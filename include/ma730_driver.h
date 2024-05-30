#ifndef MA730_DRIVER
#define MA730_DRIVER

#include "system.h"

#define MA730_NoError 0
#define MA730_Error_MGH 0b10000000
#define MA730_Error_MGL 0b01000000
#define MA730_Error_Disconnected 0b00100000

uint16_t MA730_ReadAngle(GPIO_TypeDef* csPort, uint16_t csPin);

uint16_t MA730_ReadRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                            uint8_t registerAddress);

void MA730_WriteRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                         uint8_t registerAddress, uint8_t value);

uint8_t MA730_GetError(GPIO_TypeDef* csPort, uint16_t csPin);

#endif // MA730_DRIVER