#include "motor.h"
#include "output.h"
#include "system.h"

struct pid_controller {
    float Kp;
    float Ki;
    float Kd;
    float d_d;
    float prevE;
    float prevUd;
    float prevUi;
};

typedef struct pid_controller pid_controller_t;

float StepPid(pid_controller_t* pid, float e) {
    float u_d = (1 - pid->d_d) * (e - pid->prevE) + pid->d_d * pid->prevUd;
    float u_i = e + pid->prevUi;
    float u = pid->Kp * e + pid->Ki * u_i + pid->Kd * u_d;
    pid->prevE = e;
    pid->prevUd = u_d;
    pid->prevUi = u_i;
    return u;
}

pid_controller_t positionPid = {0};
pid_controller_t velocityPid = {0};
pid_controller_t currentPid = {0};

void ControlLoop() {
    UpdateMotorState(&motor1);
    int potentiometer = readAdc(ADC_CHANNEL_2);
    float position_setpoint = (potentiometer - 2048) / 204.0f;

    float velocity_setpoint =
        StepPid(&positionPid, position_setpoint - motor1.state.position);

    float current_setpoint =
        StepPid(&velocityPid, velocity_setpoint - motor1.state.velocity);

    float duty = StepPid(&currentPid, current_setpoint - motor1.state.current);

    printf("Setpoint:%0.2f,\t", position_setpoint);
    printf("Duty:%.2f,\t", duty);
    printf("Current:%.3f,\t", motor1.state.current);
    printf("Position:%.3f,\t", motor1.state.position);
    printf("Velocity:%.3f,\t", motor1.state.velocity);
    printf("\n");

    SetMotorDuty(&motor1, duty);
}

int main() {

    positionPid.Kp = 1.0f;
    positionPid.Ki = 0.0f;
    positionPid.Kd = 0.0f;
    positionPid.d_d = 0.0f;

    velocityPid.Kp = 0.1f;
    velocityPid.Ki = 0.0f;
    velocityPid.Kd = 0.0f;
    velocityPid.d_d = 0.0f;

    currentPid.Kp = 1.0f;
    currentPid.Ki = 0.0f;
    currentPid.Kd = 0.0f;
    currentPid.d_d = 0.0f;

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
