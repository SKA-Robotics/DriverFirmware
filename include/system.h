#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32f1xx_hal.h"

#define ADC_CHANNELS 3
#define MAX_PWM 1023

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma1;
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_PWM_Init(void);
void MX_TIM2_Init(void);
void MX_ADC_Init(void);
void MX_SPI_Init(void);
void MX_USART2_UART_Init(void);

volatile static uint16_t adcBuffer[ADC_CHANNELS];

uint16_t readAdc(uint32_t channel);

#endif // SYSTEM_H