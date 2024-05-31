#ifndef ROBOSZPON_NODE_H
#define ROBOSZPON_NODE_H

#include <message_queue.h>
#include <motor.h>
#include <motor_controller.h>
#include <stdint.h>

#define ROBOSZPON_NO_ERROR 0
#define ROBOSZPON_ERROR_ENCDISCONNECT 1 << 5
#define ROBOSZPON_ERROR_ENCMGL 1 << 6
#define ROBOSZPON_ERROR_ENCMGH 1 << 7
#define ROBOSZPON_ERROR_CMDTIMEOUT 1 << 8
#define ROBOSZPON_ERROR_OVERHEAT 1 << 9

#define ROBOSZPON_NODE_STATE_STOPPED 0x00
#define ROBOSZPON_NODE_STATE_RUNNING 0x01
#define ROBOSZPON_NODE_STATE_ERROR 0x02

typedef struct {
    uint8_t nodeId; // CAN node ID of the motor node
    uint8_t state;  // Current state of the node state machine
    uint16_t flags; // node status flags, sent over CAN.
    motor_t* motor; // Pointer to the motor corresponding to the node
    motor_controller_t* motorController; // Pointer to motor controller
    message_queue_t* messageQueue;       // Message queue pointer
    GPIO_TypeDef* errorLedPort;          // GPIO port of the error LED
    uint16_t errorLedPin;                // GPIO pin of the error LED
    uint32_t commandTimeout;             // Command timeout (milliseconds)
    uint32_t temperature;                // Board temperature in 0.1°C
    uint32_t overheatThreshold;          // Overheat error threshold (0.1°C)
    uint32_t overheatResetThreshold; // Temperature below which overheat error
                                     // is cleared
} roboszpon_node_t;

void RoboszponNode_Step(roboszpon_node_t* node);

#endif // ROBOSZPON_NODE_H