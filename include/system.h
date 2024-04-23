#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32f1xx_hal.h"

#define MAX_PWM 1023
#define DELTA_TIME 0.005f

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart;
CAN_HandleTypeDef hcan;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_PWM_Init(void);
void MX_TIM2_Init(void);
void MX_ADC_Init(void);
void MX_SPI_Init(void);
void MX_USART3_UART_Init(void);
void MX_CAN_Init(void);

#define ADC_CHANNELS 3

volatile static uint16_t adcBuffer[ADC_CHANNELS];

uint16_t readAdc(uint32_t channel);

#endif // SYSTEM_H