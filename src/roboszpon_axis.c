#include "roboszpon_axis.h"
#include "mag_alpha_driver.h"
#include "roboszpon_message.h"

void roboszponAxisStoppedStep(roboszpon_axis_t* axis);
void roboszponAxisRunningStep(roboszpon_axis_t* axis);
void roboszponAxisErrorStep(roboszpon_axis_t* axis);
uint8_t roboszponAxisCheckError(roboszpon_axis_t* axis);

void roboszponAxisStep(roboszpon_axis_t* axis) {
    switch (axis->state) {
    case ROBOSZPON_AXIS_STATE_STOPPED:
        roboszponAxisStoppedStep(axis);
        return;
    case ROBOSZPON_AXIS_STATE_RUNNING:
        roboszponAxisRunningStep(axis);
        return;
    case ROBOSZPON_AXIS_STATE_ERROR:
        roboszponAxisErrorStep(axis);
        return;
    default:
        // What are you even doing here?
        return;
    }
}

void roboszponAxisStoppedStep(roboszpon_axis_t* axis) {
    // Check message queue, apply configuration messages, delete run
    // mode messages If ARM command is received, set mode to
    // ROBOSZPON_AXIS_STATE_RUNNING
    while (!MessageQueue_IsEmpty(axis->messageQueue)) {
        printf("Checking a message...\n");
        roboszpon_message_t message;
        MessageQueue_Dequeue(axis->messageQueue, &message);
        switch (message.id) {
        case MSG_PARAMETER_WRITE:
            // TODO: Set the parameter here.
            printf("Parameter write command received. Payload: %lld\n",
                   message.data);
            break;
        case MSG_PARAMETER_READ:
            // TODO: respond with the parameter value.
            printf("Parameter read command received. Payload: %lld\n",
                   message.data);
            break;
        case MSG_ACTION_REQUEST:
            if (message.data == ACTION_ARM) {
                axis->state = ROBOSZPON_AXIS_STATE_RUNNING;
            }
            // TODO: implement all the other actions
            break;
        default:
            break;
        }
    }
}

void roboszponAxisRunningStep(roboszpon_axis_t* axis) {
    // Check for errors. If there is an error, set motor effort to 0, set
    // error LED on, break and set mode to ROBOSZPON_AXIS_STATE_ERROR
    // Get latest axis command from message queue, delete other messages
    // If there was a disarm command, set mode to
    // ROBOSZPON_AXIS_STATE_STOPPED, set motor effort to 0 and break. Else,
    // pass the command to trajectory generator -> motor controller -> motor
    if (roboszponAxisCheckError(axis) != 0) {
        SetMotorDuty(axis->motor, 0.0f);
        HAL_GPIO_WritePin(axis->errorLedPort, axis->errorLedPin, GPIO_PIN_SET);
        axis->state = ROBOSZPON_AXIS_STATE_ERROR;
    }
    int isThereAMotorCommand = 0;
    roboszpon_message_t latestMotorCommand;
    while (!MessageQueue_IsEmpty(axis->messageQueue)) {
        roboszpon_message_t message;
        MessageQueue_Dequeue(axis->messageQueue, &message);
        switch (message.id) {
        case MSG_MOTOR_COMMAND:
            isThereAMotorCommand = 1;
            latestMotorCommand.id = message.id;
            latestMotorCommand.data = message.data;
            break;
        case MSG_ACTION_REQUEST:
            if (message.data == ACTION_DISARM) {
                SetMotorDuty(axis->motor, 0.0f);
                axis->state = ROBOSZPON_AXIS_STATE_STOPPED;
                return;
            }
            break;
        default:
            break;
        }
    }
    // TODO: Apply the motor command here
    if (isThereAMotorCommand) {
        printf("Received motor command. Payload: %lld\n",
               latestMotorCommand.data);
    }
}

void roboszponAxisErrorStep(roboszpon_axis_t* axis) {
    // Delete all messages from message queue. If there is a disarm command,
    // set mode to DISARM, reset error LED and break. else, check for
    // errors.
    while (!MessageQueue_IsEmpty(axis->messageQueue)) {
        roboszpon_message_t message;
        MessageQueue_Dequeue(axis->messageQueue, &message);
        if (message.id == MSG_ACTION_REQUEST) {
            if (message.data == ACTION_DISARM) {
                HAL_GPIO_WritePin(axis->errorLedPort, axis->errorLedPin,
                                  GPIO_PIN_RESET);
                axis->state = ROBOSZPON_AXIS_STATE_STOPPED;
            }
        }
    }
    // If there are no errors, reset error LED, set mode to
    // ROBOSZPON_AXIS_STATE_RUNNING and break.
    if (roboszponAxisCheckError(axis) == 0) {
        HAL_GPIO_WritePin(axis->errorLedPort, axis->errorLedPin,
                          GPIO_PIN_RESET);
        axis->state = ROBOSZPON_AXIS_STATE_RUNNING;
    }
}

uint8_t roboszponAxisCheckError(roboszpon_axis_t* axis) {
    uint8_t encoderError =
        MA730_GetError(axis->motor->encoderCsPort, axis->motor->encoderCsPin);
    // TODO: check for other errors. (Command timeout included)
    return encoderError;
}