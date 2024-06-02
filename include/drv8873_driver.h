#ifndef DRV8873_DRIVER
#define DRV8873_DRIVER

#include "system.h"

#define DRV8873_NO_ERROR 0
#define DRV8873_ERROR_Fault (1 << 6)
#define DRV8873_ERROR_OTW (1 << 5)
#define DRV8873_ERROR_UVLO (1 << 4)
#define DRV8873_ERROR_CPUV (1 << 3)
#define DRV8873_ERROR_OCP (1 << 2)
#define DRV8873_ERROR_TSD (1 << 1)
#define DRV8873_ERROR_OLD (1 << 0)

#define DRV8873_CLR_FLT_BIT (1 << 7)

typedef struct {
    GPIO_TypeDef* csPort;
    uint16_t csPin;
} drv8873_device_t;

uint8_t DRV8873_ReadRegister(drv8873_device_t device, uint8_t registerAddress);

void DRV8873_WriteRegister(drv8873_device_t device, uint8_t registerAddress,
                           uint8_t value);

uint8_t DRV8873_GetError(drv8873_device_t device);

#endif // DRV8873_DRIVER