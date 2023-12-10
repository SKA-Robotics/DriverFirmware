// #include "mag_alpha_driver.h"
#include "motor.h"
#include "output.h"
#include "system.h"

void ControlLoop() {
    UpdateMotorState(&motor1);

    int potentiometer = readAdc(ADC_CHANNEL_2);
    float duty = (potentiometer - 2048) / 2048.0f;
    printf("Duty:%.2f,\t", duty);
    printf("Current:%.3f,\t", motor1.state.current);
    printf("Position:%.3f,\t", motor1.state.position);
    printf("Velocity:%.3f,\t", motor1.state.velocity);
    printf("\n");
    SetMotorDuty(&motor1, duty);
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
