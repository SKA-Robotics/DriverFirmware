#ifndef ROBOSZPON_NODE_H
#define ROBOSZPON_NODE_H

#include "drv8873_driver.h"
#include <message_queue.h>
#include <motor.h>
#include <motor_controller.h>
#include <stdint.h>

#define ROBOSZPON_NO_ERROR 0
#define ROBOSZPON_ERROR_OVERHEAT (1 << 4)
#define ROBOSZPON_ERROR_EncDISCONNECT (1 << 5)
#define ROBOSZPON_ERROR_EncMGL (1 << 6)
#define ROBOSZPON_ERROR_EncMGH (1 << 7)
#define ROBOSZPON_ERROR_DrvOLD (1 << 8)
#define ROBOSZPON_ERROR_DrvTSD (1 << 9)
#define ROBOSZPON_ERROR_DrvOCP (1 << 10)
#define ROBOSZPON_ERROR_DrvCPUV (1 << 11)
#define ROBOSZPON_ERROR_DrvUVLO (1 << 12)
#define ROBOSZPON_ERROR_DrvOTW (1 << 13)
#define ROBOSZPON_ERROR_DrvFault (1 << 14)
#define ROBOSZPON_ERROR_CMDTIMEOUT (1 << 15)

#define ERROR_MASK 0x7fff // CMDTIMEOUT doesn't cause an error state

#define ROBOSZPON_ENC_ERROR_MASK 0b11100000
#define ROBOSZPON_DRV_ERROR_MASK (0b01111111 << 8)

#define ROBOSZPON_NODE_STATE_STOPPED 0x00
#define ROBOSZPON_NODE_STATE_RUNNING 0x01
#define ROBOSZPON_NODE_STATE_ERROR 0x02

typedef struct {
    uint8_t nodeId;         // CAN node ID of the motor node
    uint32_t configAddress; // Address of node configuration in FLASH memory
    uint8_t state;          // Current state of the node state machine
    uint16_t flags;         // node status flags, sent over CAN.
    motor_t* motor;         // Pointer to the motor corresponding to the node
    motor_controller_t* motorController; // Pointer to motor controller
    message_queue_t* messageQueue;       // Message queue pointer
    drv8873_device_t drv8873;            // Pointer to the DRV8873 device
    GPIO_TypeDef* errorLedPort;          // GPIO port of the error LED
    uint16_t errorLedPin;                // GPIO pin of the error LED
    uint32_t commandTimeout;             // Command timeout (milliseconds)
    uint32_t temperature;                // Board temperature in 0.1°C
    uint32_t overheatThreshold;          // Overheat error threshold (0.1°C)
    uint32_t overheatResetThreshold; // Temperature below which overheat error
                                     // is cleared
    uint32_t latestCommandTime;      // Time (ticks) of the latest motor command
    uint32_t reportPeriod;           // Period of consecutive sent node reports
} roboszpon_node_t;

void RoboszponNode_Report(roboszpon_node_t* node);
void RoboszponNode_Step(roboszpon_node_t* node);

#endif // ROBOSZPON_NODE_H