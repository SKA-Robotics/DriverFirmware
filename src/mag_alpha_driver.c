#include "mag_alpha_driver.h"

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
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    txData[0] = 0;
    txData[1] = 0;
    HAL_Delay(1); // TODO: This should be at least 0.75 microsecond. Utilize
                  // additional timer to make it so.
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
    return rxData[1] << 8 | rxData[0];
}

uint8_t MA730_CheckError(GPIO_TypeDef* csPort, uint16_t csPin) {
    uint16_t flags = MA730_ReadRegister(csPort, csPin, 0x1b);
    // Check for disconnected device
    // The device should return 0x00 as a second byte when responding to a read
    // command. If the response is 0xffff, there must be no device on the other
    // end of the wire: report an error.
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