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

motor_controller_t motorController0 = {
    .motor = &motor0, .params = {.minDuty = -1.0f, .maxDuty = 1.0f}};

motor_controller_t motorController1 = {
    .motor = &motor1, .params = {.minDuty = -1.0f, .maxDuty = 1.0f}};

roboszpon_axis_t axis0 = {.state = ROBOSZPON_AXIS_STATE_RUNNING,
                          .motor = &motor0,
                          .motorController = &motorController0,
                          .errorLedPort = LED_PORT,
                          .errorLedPin = LED_ENC0_PIN};
roboszpon_axis_t axis1 = {.state = ROBOSZPON_AXIS_STATE_RUNNING,
                          .motor = &motor1,
                          .motorController = &motorController1,
                          .errorLedPort = LED_PORT,
                          .errorLedPin = LED_ENC1_PIN};

void MainLoop() {
    roboszponAxisStep(&axis0);
    roboszponAxisStep(&axis1);
}

void MotorControlLoop() {
    if (axis0.state == ROBOSZPON_AXIS_STATE_RUNNING) {
        motorControllerStep(axis0.motorController);
    }
    if (axis1.state == ROBOSZPON_AXIS_STATE_RUNNING) {
        motorControllerStep(axis1.motorController);
    }
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

    // Start the timers, begin working
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim4);
    while (1) {
        HAL_Delay(3000);
        float random = (float)(HAL_GetTick() * 145 % 2000) / 1000.0f - 1.0f;
        printf("Setting duty to %.02f\n", random);
        motorControllerSetDutySetpoint(&motorController0, random);
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
