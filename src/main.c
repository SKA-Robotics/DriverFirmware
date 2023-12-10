#include "mag_alpha_driver.h"
#include "output.h"
#include "system.h"

#include <math.h>

struct motor_state {
    uint8_t direction;
    uint16_t prevPositionMeasurement;
    int32_t positionRaw;
    float position;
    float velocity;
    float current;
};

#define FORWARD 1
#define REVERSE 0

#define PI 3.14159265359f
#define ENCODER_RESOLUTION 65535
#define ENCODER_REVOLUTION_THRESHOLD 32137
#define DELTA_TIME 0.01f

typedef struct motor_state motor_state_t;

struct motor {
    volatile uint32_t* pwmChannelForward;
    volatile uint32_t* pwmChannelReverse;
    uint32_t adcChannel;
    GPIO_TypeDef* encoderCsPort;
    uint16_t encoderCsPin;
    motor_state_t state;
};

typedef struct motor motor_t;

motor_t Motor1 = {
    .pwmChannelForward = &TIM1->CCR1,
    .pwmChannelReverse = &TIM1->CCR2,
    .adcChannel = ADC_CHANNEL_0,
    .encoderCsPort = GPIOA,
    .encoderCsPin = GPIO_PIN_15,
    .state = {0}, // Default initial state
};

void SetMotorDuty(motor_t* motor, float duty) {
    uint32_t pwm = fabs(duty) * 1023;
    if (pwm > MAX_PWM) {
        pwm = MAX_PWM;
    }
    if (duty >= 0) {
        *(motor->pwmChannelForward) = pwm;
        *(motor->pwmChannelReverse) = 0;
        motor->state.direction = FORWARD;
    } else {
        *(motor->pwmChannelForward) = 0;
        *(motor->pwmChannelReverse) = pwm;
        motor->state.direction = REVERSE;
    }
}

void UpdateMotorState(motor_t* motor) {
    uint16_t positionMeasurement =
        readMagAlphaAngle(motor->encoderCsPort, motor->encoderCsPin);
    long deltaPositionRaw =
        positionMeasurement - motor->state.prevPositionMeasurement;
    motor->state.prevPositionMeasurement = positionMeasurement;

    if (deltaPositionRaw > ENCODER_REVOLUTION_THRESHOLD) {
        // gone below 0
        deltaPositionRaw -= ENCODER_RESOLUTION;
    } else if (deltaPositionRaw < -ENCODER_REVOLUTION_THRESHOLD) {
        // gone above 65536
        deltaPositionRaw += ENCODER_RESOLUTION;
    }

    motor->state.positionRaw += deltaPositionRaw;

    motor->state.position =
        motor->state.positionRaw * 2 * PI / ENCODER_RESOLUTION;

    motor->state.velocity =
        deltaPositionRaw * 2 * PI / ENCODER_RESOLUTION / DELTA_TIME;

    uint16_t adc_value = readAdc(motor->adcChannel);
    float scaled_adc_value = (float)adc_value * 0.0015f;
    motor->state.current = (motor->state.direction == FORWARD)
                               ? scaled_adc_value
                               : -scaled_adc_value;
}

void ControlLoop() {
    UpdateMotorState(&Motor1);
    float current = Motor1.state.current;
    int potentiometer = readAdc(ADC_CHANNEL_2);
    float duty = (potentiometer - 2048) / 2048.0f;
    uint16_t pwm = fabs(duty) * 1023;
    printf("Duty:%.2f,\t", duty);
    // printf("PWM CCR:%d,\t", pwm);
    printf("Current:%.3f,\t", current);
    // printf("Current_raw:%d,\t", readAdc(ADC_CHANNEL_0));
    printf("Position:%.3f,\t", Motor1.state.position);
    // printf("Position_raw:%ld,\t", Motor1.state.positionRaw);
    printf("Velocity:%.3f,\t", Motor1.state.velocity);
    printf("\n");
    SetMotorDuty(&Motor1, duty);
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
