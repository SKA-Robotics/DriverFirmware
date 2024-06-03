#include "ma730_driver.h"
#include "system.h"

uint16_t MA730_ReadAngle(ma730_device_t encoder) {
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_SET);
    return rxData[0] << 8 | rxData[1];
}

uint16_t MA730_ReadRegister(ma730_device_t encoder, uint8_t registerAddress) {
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b010 << 5) | (registerAddress & 0x1f);
    txData[1] = 0x00;
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_SET);
    txData[0] = 0x00;
    txData[1] = 0x00;
    DelayMicroseconds(1);
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_SET);
    DelayMicroseconds(1);
    return rxData[1] << 8 | rxData[0];
}

void MA730_WriteRegister(ma730_device_t encoder, uint8_t registerAddress,
                         uint8_t value) {
    uint8_t txData[2] = {0};
    uint8_t rxData[2] = {0};
    txData[0] = (0b100 << 5) | (registerAddress & 0x1f);
    txData[1] = value;
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_SET);
    txData[0] = 0x00;
    txData[1] = 0x00;
    HAL_Delay(
        20); // 20 ms delay necessary for encoder non-volatile memory updates
             // (https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MA732GQ-Z/
             // - page 11)
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 10);
    HAL_GPIO_WritePin(encoder.csPort, encoder.csPin, GPIO_PIN_SET);
    DelayMicroseconds(1);
}

uint8_t MA730_GetError(ma730_device_t encoder) {
    uint16_t flags = MA730_ReadRegister(encoder, 0x1b);
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
    uint8_t mgl = ~(mg1l | mg2l) & 0x01;
    flags |= mgl << 6;
    flags &= 0b11000000;
    return flags;
}

float MA730_GetZero(ma730_device_t encoder) {
    uint8_t registerValueLower = MA730_ReadRegister(encoder, 0x0);
    uint8_t registerValueUpper = MA730_ReadRegister(encoder, 0x1);

    uint16_t registerValue =
        (((uint16_t)registerValueUpper) << 8) | registerValueLower;
    return 1.0f - (float)registerValue / (float)0xffff;
}

void MA730_SetZero(ma730_device_t encoder, float zeroPosition) {
    uint16_t registerValue = ((float)0xffff) * (1 - zeroPosition);
    uint8_t registerValueLower = registerValue & 0xff;
    uint8_t registerValueUpper = (registerValue >> 8) & 0xff;
    MA730_WriteRegister(encoder, 0x0, registerValueLower);
    MA730_WriteRegister(encoder, 0x1, registerValueUpper);
}

uint8_t MA730_GetRotationDirection(ma730_device_t encoder) {
    return MA730_ReadRegister(encoder, 0x9);
}

void MA730_SetRotationDirection(ma730_device_t encoder, uint8_t direction) {
    MA730_WriteRegister(encoder, 0x9, direction);
}

float MA730_GetFilterWindow(ma730_device_t encoder) {
    uint8_t registerValue = MA730_ReadRegister(encoder, 0xe);
    switch (registerValue) {
    case 51:
        return 0.64f;
    case 68:
        return 0.128f;
    case 85:
        return 0.256f;
    case 102:
        return 0.512f;
    case 119:
        return 1.024f;
    case 136:
        return 2.048f;
    case 153:
        return 4.096f;
    case 170:
        return 8.192f;
    case 187:
        return 16.384f;
    default:
        return -1.0f; // Return a negative value if the register value is
                      // unrecognized
    }
}

void MA730_SetFilterWindow(ma730_device_t encoder, float windowLenght) {
    uint8_t registerValue;
    if (windowLenght < 0.128f) {
        registerValue = 51;
    } else if (windowLenght < 0.256f) {
        registerValue = 68;
    } else if (windowLenght < 0.512f) {
        registerValue = 85;
    } else if (windowLenght < 1.024f) {
        registerValue = 102;
    } else if (windowLenght < 2.048f) {
        registerValue = 119;
    } else if (windowLenght < 4.096f) {
        registerValue = 136;
    } else if (windowLenght < 8.192f) {
        registerValue = 153;
    } else if (windowLenght < 16.384f) {
        registerValue = 170;
    } else {
        registerValue = 187;
    }
    MA730_WriteRegister(encoder, 0xe, registerValue);
}