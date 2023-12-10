#include "output.h"
#include "system.h"

void ControlLoop() {
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 1023;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 1023;
    // current = 0.9f * current + 0.1f * ((float)adc_value * 1.419f);

    printf("I1_raw=%d\t", readAdc(ADC_CHANNEL_0));
    printf("I2_raw=%d\t", readAdc(ADC_CHANNEL_1));
    printf("\n");
}

void SysTick_Handler(void) { HAL_IncTick(); }

void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim2) {
        ControlLoop();
    }
}

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_PWM_Init();
    MX_TIM2_Init();
    MX_ADC_Init();
    MX_SPI_Init();
    MX_USART2_UART_Init();

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    while (1) {
        HAL_Delay(1000);
    }
}
