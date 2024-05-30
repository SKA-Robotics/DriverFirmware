#include "ma730_driver.h"
#include "system.h"

uint16_t MA730_ReadAngle(GPIO_TypeDef* csPort, uint16_t csPin) {
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    return rxData[0] << 8 | rxData[1];
}

uint16_t MA730_ReadRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                            uint8_t registerAddress) {
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b010 << 5) | (registerAddress & 0x1f);
    txData[1] = 0x00;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    txData[0] = 0x00;
    txData[1] = 0x00;
    DelayMicroseconds(1);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    DelayMicroseconds(1);
    return rxData[1] << 8 | rxData[0];
}

void MA730_WriteRegister(GPIO_TypeDef* csPort, uint16_t csPin,
                         uint8_t registerAddress, uint8_t value) {
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b100 << 5) | (registerAddress & 0x1f);
    txData[1] = value;
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    txData[0] = 0x00;
    txData[1] = 0x00;
    HAL_Delay(
        20); // 20 ms delay necessary for encoder non-volatile memory updates
             // (https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MA732GQ-Z/
             // - page 11)
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    DelayMicroseconds(1);
}

uint8_t MA730_GetError(GPIO_TypeDef* csPort, uint16_t csPin) {
    uint16_t flags = MA730_ReadRegister(csPort, csPin, 0x1b);
    // Check for disconnected device
    // This method is not 100% correct. TODO: find a better one
    if (flags == 0xffff) {
        return MA730_Error_Disconnected;
    }
    // Correct MGL signal
    // (https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MA732GQ-Z/
    // - page 22)
    flags &= 0b10111111;
    uint8_t mg1l = (flags >> 3) & 0x01;
    uint8_t mg2l = (flags >> 2) & 0x01;
    uint8_t mgl = !(mg1l | mg2l) & 0x01;
    flags |= mgl << 6;
    flags &= 0b11000000;
    return flags;
}

float MA730_GetZero(GPIO_TypeDef* csPort, uint16_t csPin) {
    uint8_t registerValueLower = MA730_ReadRegister(csPort, csPin, 0x0);
    uint8_t registerValueUpper = MA730_ReadRegister(csPort, csPin, 0x1);

    uint16_t registerValue =
        (((uint16_t)registerValueUpper) << 8) | registerValueLower;
    return 1.0f - (float)registerValue / (float)0xffff;
}

void MA730_SetZero(GPIO_TypeDef* csPort, uint16_t csPin, float zeroPosition) {
    uint16_t registerValue = ((float)0xffff) * (1 - zeroPosition);
    uint8_t registerValueLower = registerValue & 0xff;
    uint8_t registerValueUpper = (registerValue >> 8) & 0xff;
    MA730_WriteRegister(csPort, csPin, 0x0, registerValueLower);
    MA730_WriteRegister(csPort, csPin, 0x1, registerValueUpper);
}