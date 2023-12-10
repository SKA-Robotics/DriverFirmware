#ifndef MAG_ALPHA_DRIVER
#define MAG_ALPHA_DRIVER

#include "system.h"

/*
Functions for communication with MagAlpha encoders
*/

uint16_t readMagAlphaAngle(GPIO_TypeDef* csPort, uint16_t csPin) {
    uint32_t timeout = 10;
    uint8_t txData[2];
    uint8_t rxData[2];
    txData[1] = 0;
    txData[0] = 0;
    uint16_t angleSensor;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, timeout);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    angleSensor = rxData[0] << 8 | rxData[1];
    return angleSensor;
}

#endif // MAG_ALPHA_DRIVER