#include "motor.h"
#include "output.h"
// #include "pid.h"
#include "drv8873_driver.h"
#include "mag_alpha_driver.h"
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

// pid_controller_t positionPid = {0};
// pid_controller_t velocityPid = {0};
// pid_controller_t currentPid = {0};

int count = 12;
int counter = 0;

void ControlLoop() {
    // int potentiometer = readAdc(ADC_CHANNEL_2);
    // float setpoint = 0.9f * setpoint + 0.1f * (potentiometer - 2048) /
    // 2048.0f; float position_setpoint = 10.0f * setpoint;

    // float velocity_setpoint =
    //     StepPid(&positionPid, position_setpoint - motor0.state.position);

    // float current_setpoint =
    //     StepPid(&velocityPid, velocity_setpoint - motor0.state.velocity);

    // float duty = StepPid(&currentPid, current_setpoint -
    // motor0.state.current);
    float duty = -0.6f;

    if (++counter == count) {
        counter = 0;
        // for (uint i = 0; i < 6; ++i) {
        //     uint16_t r = DRV8873_ReadRegister(CS_ENC0DRV0_PORT, CS_DRV0_PIN,
        //     i); printf("Register %d: ", i); for (int i = 15; i >= 0; i--) {
        //         printf("%d", (r >> i) & 1);
        //     }
        //     printf(",\t");
        // }
        // printf("\n");
        // UpdateMotorState(&motor0);
        // SetMotorDuty(&motor0, duty);
        uint8_t error = MA730_CheckError(CS_ENC0DRV0_PORT, CS_ENC0_PIN);
        for (int i = 7; i >= 0; i--) {
            printf("%d", (error >> i) & 1);
        }
        printf(",\n");

        // if (error != MA730_NoError) {
        //     printf("Encoder error: %d", error);
        // }
        // // printf("PositionSetpoint:%0.2f,\t", position_setpoint);
        // printf("Position:%.3f,\t", motor0.state.position);
        // printf("Velocity:%.3f,\t", motor0.state.velocity);
        // printf("Current:%.3f,\t", motor0.state.current);
        // printf("Duty:%.5f,\t", duty);
        // printf("\n");
    }
}

int main() {

    // positionPid.Kp = 3.0f;
    // positionPid.Ki = 1.0f;
    // positionPid.Kd = 0.1f;
    // positionPid.Kaw = 0.7f;
    // positionPid.d_d = 0.6f;
    // positionPid.deadzone = 0.0f;
    // positionPid.u_max = 21.37f;
    // positionPid.du_max = 1.0f;

    // velocityPid.Kp = 1.0f;
    // velocityPid.Ki = 0.0f;
    // velocityPid.Kd = 0.0f;
    // velocityPid.Kaw = 0.7f;
    // velocityPid.d_d = 0.6f;
    // velocityPid.deadzone = 0.0f;
    // velocityPid.u_max = 1.0f;
    // velocityPid.du_max = 0.1f;

    // currentPid.Kp = 1.0f;
    // currentPid.Ki = 0.0f;
    // currentPid.Kd = 0.0f;
    // currentPid.Kaw = 0.7f;
    // currentPid.d_d = 0.7f;
    // currentPid.deadzone = 0.0f;
    // currentPid.u_max = 1.0f;
    // currentPid.du_max = 0.2f;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_PWM_Init();
    MX_TIM2_Init();
    MX_ADC_Init();
    MX_SPI_Init();
    // MX_CAN_Init();

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                      GPIO_PIN_RESET);

    while (1) {
        // // DRV SPI test:
        // for (uint8_t registerAddress = 0; registerAddress <= 5;
        //      ++registerAddress) {
        //     uint16_t response = DRV8873_ReadRegister(
        //         CS_ENC0DRV0_PORT, CS_DRV0_PIN, registerAddress);
        //     printf("R%d:", registerAddress);
        //     for (int i = 15; i >= 0; i--) {
        //         printf("%d", (response >> i) & 1);
        //     }
        //     printf(",\t");
        // }
        // printf("\n");

        HAL_Delay(1000);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }

void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim2) {
        ControlLoop();
    }
}
