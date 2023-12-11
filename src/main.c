#include "motor.h"
#include "output.h"
#include "pid.h"
#include "system.h"

motor_t motor1 = {
    .pwmChannelForward = &TIM1->CCR1,
    .pwmChannelReverse = &TIM1->CCR2,
    .adcChannel = ADC_CHANNEL_0,
    .encoderCsPort = GPIOA,
    .encoderCsPin = GPIO_PIN_15,
    .state = {0}, // Default initial state
};

motor_t motor2 = {
    .pwmChannelForward = &TIM1->CCR3,
    .pwmChannelReverse = &TIM1->CCR4,
    .adcChannel = ADC_CHANNEL_1,
    .encoderCsPort = GPIOA,      // TODO: Configure encoder pin
    .encoderCsPin = GPIO_PIN_15, // TOFO: Configure encoder pin
    .state = {0},                // Default initial state
};

pid_controller_t positionPid = {0};
pid_controller_t velocityPid = {0};
pid_controller_t currentPid = {0};

int count = 12;
int counter = 0;

float setpoint = 0.0f;
float velocity_error = 0.0f;
// float filter_constant = 0.8f;

void ControlLoop() {
    UpdateMotorState(&motor1);
    int potentiometer = readAdc(ADC_CHANNEL_2);
    setpoint = 0.9f * setpoint + 0.1f * (potentiometer - 2048) / 54.0f;
    float position_setpoint = setpoint;

    float velocity_setpoint =
        StepPid(&positionPid, position_setpoint - motor1.state.position);

    float duty =
        StepPid(&velocityPid, velocity_setpoint - motor1.state.velocity);

    // Pętla prądowa na ten moment zupełnie nie działa
    // float duty = StepPid(&currentPid, current_setpoint -
    // motor1.state.current);

    if (++counter == count) {

        counter = 0;

        printf("PositionSetpoint:%0.2f,\t", position_setpoint);
        printf("VelocitySetpoint:%0.2f,\t", velocity_setpoint);
        printf("Position:%.3f,\t", motor1.state.position);
        printf("Velocity:%.3f,\t", motor1.state.velocity);
        printf("Duty:%.2f,\t", duty);
        printf("\n");
    }

    SetMotorDuty(&motor1, duty);
}

int main() {

    positionPid.Kp = 2.0f;
    positionPid.Ki = 1.0f;
    positionPid.Kd = 0.0f;
    positionPid.Kaw = 0.7f;
    positionPid.d_d = 0.6f;
    positionPid.deadzone = 0.0f;
    positionPid.u_max = 21.37f;
    positionPid.du_max = 1.0f;

    velocityPid.Kp = 0.2f;
    velocityPid.Ki = 0.6f;
    velocityPid.Kd = 0.0f;
    velocityPid.Kaw = 0.7f;
    velocityPid.d_d = 0.6f;
    velocityPid.deadzone = 0.0f;
    velocityPid.u_max = 1.0f;
    velocityPid.du_max = 0.1f;

    // currentPid.Kp = 2.0f;
    // currentPid.Ki = 0.0f;
    // currentPid.Kd = 0.0f;
    // currentPid.d_d = 0.0f;
    // positionPid.u_max = 1.0f;
    // positionPid.du_max = 0.2f;

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

void SysTick_Handler(void) { HAL_IncTick(); }

void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim2) {
        ControlLoop();
    }
}
