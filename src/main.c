#include "motor.h"
#include "output.h"
#include "roboszpon_axis.h"
#include "system.h"

motor_t motor0 = {
    .pwmChannelForward = &TIM3->CCR1,
    .pwmChannelReverse = &TIM3->CCR2,
    .adcChannel = ADC_MOTOR0_CURRENT,
    .encoderCsPort = CS_ENC0DRV0_PORT,
    .encoderCsPin = CS_ENC0_PIN,
    .state = {0}, // Default initial state
};

motor_t motor1 = {
    .pwmChannelForward = &TIM1->CCR1,
    .pwmChannelReverse = &TIM1->CCR2,
    .adcChannel = ADC_MOTOR1_CURRENT,
    .encoderCsPort = CS_ENC1DRV1_PORT,
    .encoderCsPin = CS_ENC1_PIN,
    .state = {0}, // Default initial state
};

roboszpon_axis_t axis0 = {.state = ROBOSZPON_AXIS_STATE_STOPPED,
                          .motor = &motor0,
                          .errorLedPort = LED_PORT,
                          .errorLedPin = LED_ENC0_PIN};
roboszpon_axis_t axis1 = {.state = ROBOSZPON_AXIS_STATE_STOPPED,
                          .motor = &motor1,
                          .errorLedPort = LED_PORT,
                          .errorLedPin = LED_ENC1_PIN};

void MainLoop() {
    roboszponAxisStep(&axis0);
    roboszponAxisStep(&axis1);
}

void MotorControlLoop() {
    // axis0.motor_controller.step();
    // axis1.motor_controller.step();
}

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_PWM_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_ADC_Init();
    MX_SPI_Init();
    MX_CAN_Init();
    // Set interrupt priorities
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 1);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    HAL_GPIO_WritePin(LED_PORT, LED_CAN_PIN | LED_ENC0_PIN | LED_ENC1_PIN,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_POWER_PIN, GPIO_PIN_SET);

    axis0.state = ROBOSZPON_AXIS_STATE_RUNNING;
    while (1) {
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim2) {
        MotorControlLoop();
    } else if (htim == &htim4) {
        MainLoop();
    }
}
