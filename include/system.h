#ifndef SYSTEM_H
#define SYSTEM_H

#include "firmware_settings.h"
#include "stm32f1xx_hal.h"

#define MAX_PWM 1023
#define DELTA_TIME 0.01f
#define CAN_TX_TIMEOUT 100

#define UART_TX_PIN GPIO_PIN_10
#define UART_RX_PIN GPIO_PIN_11
#define UART_PORT GPIOB
#define LED_POWER_PIN GPIO_PIN_7
#define LED_CAN_PIN GPIO_PIN_6
#define LED_ENC0_PIN GPIO_PIN_5
#define LED_ENC1_PIN GPIO_PIN_4
#define LED_PORT GPIOB
#define PWM0_CH1_PIN GPIO_PIN_6
#define PWM0_CH2_PIN GPIO_PIN_7
#define PWM0_PORT GPIOC
#define PWM1_CH1_PIN GPIO_PIN_8
#define PWM1_CH2_PIN GPIO_PIN_9
#define PWM1_PORT GPIOA
#define THERMISTOR_PIN GPIO_PIN_0
#define THERMISTOR_PORT GPIOB
#define CURRENT0_PIN GPIO_PIN_1
#define CURRENT0_PORT GPIOB
#define CURRENT1_PIN GPIO_PIN_5
#define CURRENT1_PORT GPIOC
#define NFAULT0_PIN GPIO_PIN_15
#define NFAULT0_PORT GPIOB
#define NFAULT1_PIN GPIO_PIN_10
#define NFAULT1_PORT GPIOC
#define SPI_MOSI_PIN GPIO_PIN_7
#define SPI_MISO_PIN GPIO_PIN_6
#define SPI_SCK_PIN GPIO_PIN_5
#define SPI_PORT GPIOA
#define CS_ENC0_PIN GPIO_PIN_3
#define CS_DRV0_PIN GPIO_PIN_15
#define CS_ENC0DRV0_PORT GPIOA
#define CS_ENC1_PIN GPIO_PIN_12
#define CS_DRV1_PIN GPIO_PIN_13
#define CS_ENC1DRV1_PORT GPIOB
#define CAN_TX_PIN GPIO_PIN_12
#define CAN_RX_PIN GPIO_PIN_11
#define CAN_PORT GPIOA

#define ADC_THERMISTOR 0
#define ADC_MOTOR0_CURRENT 2
#define ADC_MOTOR1_CURRENT 1

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart;
CAN_HandleTypeDef hcan;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_PWM_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_ADC_Init(void);
void MX_SPI_Init(void);
void MX_USART3_UART_Init(void);
void MX_CAN_Init(void);

#define ADC_CHANNELS 3

volatile static uint16_t adcBuffer[ADC_CHANNELS];

void DelayMicroseconds(uint16_t us);

uint16_t ReadAdc(uint32_t channel);

// returns HAL_OK on success
int TransmitCanFrame(uint16_t arbitrationId, uint64_t data, uint32_t timeout);

void ErrorHandler(void);

#endif // SYSTEM_H