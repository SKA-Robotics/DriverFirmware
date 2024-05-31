#ifndef MA730_DRIVER
#define MA730_DRIVER

#include "system.h"

#define MA730_NoError 0
#define MA730_Error_MGH 0b10000000
#define MA730_Error_MGL 0b01000000
#define MA730_Error_Disconnected 0b00100000

#define MA730_DIRECTION_FORWARD 0x00
#define MA730_DIRECTION_REVERSE 0x80

typedef struct {
    GPIO_TypeDef* csPort;
    uint16_t csPin;
} ma730_device_t;

uint16_t MA730_ReadAngle(ma730_device_t encoder);

uint16_t MA730_ReadRegister(ma730_device_t encoder, uint8_t registerAddress);

void MA730_WriteRegister(ma730_device_t encoder, uint8_t registerAddress,
                         uint8_t value);

uint8_t MA730_GetError(ma730_device_t encoder);

float MA730_GetZero(ma730_device_t encoder);

void MA730_SetZero(ma730_device_t encoder, float zeroPosition);

uint8_t MA730_GetRotationDirection(ma730_device_t encoder);

void MA730_SetRotationDirection(ma730_device_t encoder, uint8_t direction);

#endif // MA730_DRIVER