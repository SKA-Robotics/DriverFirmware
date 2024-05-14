#include "drv8873_driver.h"

uint16_t DRV8873_ReadRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                              uint8_t registerAddress) {
    uint32_t timeout = 10;
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b01000000) | ((0b00011111 & registerAddress) << 1);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, timeout);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    HAL_Delay(100);
    txData[0] = 0;
    txData[1] = 0;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, timeout);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    return rxData[0] << 8 | rxData[1];
}