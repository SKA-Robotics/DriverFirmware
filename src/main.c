#include "math.h"
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
    .positionOffset = 0.0f,
    .state = {0}, // Default initial state
};
motor_t motor1 = {
    .pwmChannelForward = &TIM1->CCR1,
    .pwmChannelReverse = &TIM1->CCR2,
    .adcChannel = ADC_MOTOR1_CURRENT,
    .encoderCsPort = CS_ENC1DRV1_PORT,
    .encoderCsPin = CS_ENC1_PIN,
    .positionOffset = 0.0f,
    .state = {0}, // Default initial state
};

motor_controller_t motorController0 = {
    .motor = &motor0,
    .params = {.minDuty = -1.0f,
               .maxDuty = 1.0f,
               .minCurrent = -1.0f,
               .maxCurrent = 1.0f,
               .minVelocity = -1.0f,
               .maxVelocity = 1.0f,
               .minPosition = -INFINITY,
               .maxPosition = +INFINITY},
    .currentPid = {.Kp = 1.0f,
                   .Ki = 0.1f,
                   .Kd = 0.0f,
                   .Kaw = 0.8f,
                   .deadzone = 0.01f,
                   .du_max = +INFINITY,
                   .u_max = +INFINITY},
    .velocityPid = {.Kp = 1.0f,
                    .Ki = 0.1f,
                    .Kd = 0.0f,
                    .Kaw = 0.8f,
                    .deadzone = 0.01f,
                    .du_max = +INFINITY,
                    .u_max = +INFINITY},
    .positionPid = {.Kp = 3.0f,
                    .Ki = 0.2f,
                    .Kd = 0.0f,
                    .Kaw = 0.8f,
                    .deadzone = 0.01f,
                    .du_max = +INFINITY,
                    .u_max = +INFINITY},
};

motor_controller_t motorController1 = {
    .motor = &motor1,
    .params = {.minDuty = -1.0f,
               .maxDuty = 1.0f,
               .minCurrent = -1.0f,
               .maxCurrent = 1.0f,
               .minVelocity = -1.0f,
               .maxVelocity = 1.0f,
               .minPosition = -INFINITY,
               .maxPosition = +INFINITY},
    .currentPid = {.Kp = 1.0f,
                   .Ki = 0.0f,
                   .Kd = 0.0f,
                   .Kaw = 0.8f,
                   .deadzone = 0.01f,
                   .du_max = +INFINITY,
                   .u_max = +INFINITY},
    .velocityPid = {.Kp = 1.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .Kaw = 0.8f,
                    .deadzone = 0.01f,
                    .du_max = +INFINITY,
                    .u_max = +INFINITY},
    .positionPid = {.Kp = 1.0f,
                    .Ki = 0.0f,
                    .Kd = 0.0f,
                    .Kaw = 0.8f,
                    .deadzone = 0.01f,
                    .du_max = +INFINITY,
                    .u_max = +INFINITY},
};

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
        float random = (float)(HAL_GetTick() * 145 % 6000) / 1000.0f - 3.0f;
        printf("Setting position to %.02f\n", random);
        motorControllerSetPositionSetpoint(&motorController0, random);
        HAL_Delay(5000);
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
