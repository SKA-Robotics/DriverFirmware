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

message_queue_t messageQueue0;
message_queue_t messageQueue1;

roboszpon_axis_t axis0 = {.nodeId = 0x1,
                          .state = ROBOSZPON_AXIS_STATE_STOPPED,
                          .motor = &motor0,
                          .motorController = &motorController0,
                          .messageQueue = &messageQueue0,
                          .errorLedPort = LED_PORT,
                          .errorLedPin = LED_ENC0_PIN};
roboszpon_axis_t axis1 = {.nodeId = 0x2,
                          .state = ROBOSZPON_AXIS_STATE_STOPPED,
                          .motor = &motor1,
                          .motorController = &motorController1,
                          .messageQueue = &messageQueue1,
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
    MessageQueue_Init(&messageQueue0);
    MessageQueue_Init(&messageQueue1);
    // Set interrupt priorities
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    HAL_GPIO_WritePin(LED_PORT, LED_CAN_PIN | LED_ENC0_PIN | LED_ENC1_PIN,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_POWER_PIN, GPIO_PIN_SET);

    // Start the timers, begin working
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim4);

    while (1) {
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4); }
void USB_LP_CAN1_RX0_IRQHandler(void) {
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim2) {
        MotorControlLoop();
    } else if (htim == &htim4) {
        MainLoop();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    roboszpon_message_t message =
        interpretCanMessage(RxHeader.StdId, RxHeader.DLC, RxData);
    if (message.nodeId == axis0.nodeId) {
        MessageQueue_Enqueue(axis0.messageQueue, message);
    } else if (message.nodeId == axis1.nodeId) {
        MessageQueue_Enqueue(axis1.messageQueue, message);
    } else if (message.nodeId == NODEID_BROADCAST) {
        MessageQueue_Enqueue(axis0.messageQueue, message);
        MessageQueue_Enqueue(axis1.messageQueue, message);
    }
}