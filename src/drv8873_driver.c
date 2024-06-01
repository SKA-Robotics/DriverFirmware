#include "drv8873_driver.h"

void SetSpiMode0() {
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    HAL_SPI_Init(&hspi1);
}

void SetSpiMode1() {
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    HAL_SPI_Init(&hspi1);
}

uint8_t DRV8873_ReadRegister(drv8873_device_t device, uint8_t registerAddress) {

    uint32_t timeout = 10;
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b01000000) | ((registerAddress & 0x1f) << 1);

    __disable_irq(); // Critical section: SPI mode is switched for a while

    SetSpiMode1();
    HAL_GPIO_WritePin(device.csPort, device.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, timeout);
    HAL_GPIO_WritePin(device.csPort, device.csPin, GPIO_PIN_SET);
    SetSpiMode0();

    __enable_irq();

    return rxData[1];
}

void DRV8873_WriteRegister(drv8873_device_t device, uint8_t registerAddress,
                           uint8_t value) {

    uint32_t timeout = 10;
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b00000000) | ((registerAddress & 0x1f) << 1);
    txData[1] = value;

    __disable_irq(); // Critical section: SPI mode is switched for a while

    SetSpiMode1();
    HAL_GPIO_WritePin(device.csPort, device.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, timeout);
    HAL_GPIO_WritePin(device.csPort, device.csPin, GPIO_PIN_SET);
    SetSpiMode0();

    __enable_irq();
}

uint8_t DRV8873_GetError(drv8873_device_t device) {
    uint8_t flags = DRV8873_ReadRegister(device, 0x0);
    if (flags != DRV8873_NO_ERROR) {
        uint8_t prevValue = DRV8873_ReadRegister(device, 0x4);
        DRV8873_WriteRegister(device, 0x4, prevValue | DRV8873_CLR_FLT_BIT);
    }
    return flags;
}