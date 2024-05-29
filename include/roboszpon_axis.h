#ifndef ROBOSZPON_AXIS_H
#define ROBOSZPON_AXIS_H

#include <message_queue.h>
#include <motor.h>
#include <motor_controller.h>
#include <stdint.h>

#define ROBOSZPON_AXIS_STATE_STOPPED 0x00
#define ROBOSZPON_AXIS_STATE_RUNNING 0x01
#define ROBOSZPON_AXIS_STATE_ERROR 0x02

typedef struct {
    uint8_t nodeId; // CAN node ID of the motor axis
    uint8_t state;  // Current state of the axis state machine
    uint64_t flags; // Axis status flags, sent over CAN. 2 MSB reserved!
    motor_t* motor; // Pointer to the motor corresponding to the axis
    motor_controller_t* motorController; // Pointer to motor controller
    message_queue_t* messageQueue;       // Message queue pointer
    GPIO_TypeDef* errorLedPort;          // GPIO port of the error LED
    uint16_t errorLedPin;                // GPIO pin of the error LED
    uint32_t commandTimeout;             // Command timeout (milliseconds)
} roboszpon_axis_t;

void roboszponAxisStep(roboszpon_axis_t* axis);

#endif // ROBOSZPON_AXIS_H