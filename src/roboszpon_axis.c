#include "roboszpon_axis.h"

void roboszpon_axis_step(roboszpon_axis_t* axis) {
    switch (axis->state) {
    case ROBOSZPON_AXIS_STATE_STOPPED:
        // Check message queue, apply configuration messages, delete run mode
        // messages If ARM command is received, set mode to
        // ROBOSZPON_AXIS_STATE_RUNNING
        break;
    case ROBOSZPON_AXIS_STATE_RUNNING:
        // Get latest axis command from message queue, delete other messages
        // If there was a disarm command, set mode to
        // ROBOSZPON_AXIS_STATE_STOPPED, set motor effort to 0 and break. Else,
        // pass the command to trajectory generator -> motor controller -> motor
        // Check for errors. If there is an error, set motor effort to 0, set
        // error LED on, break and set mode to ROBOSZPON_AXIS_STATE_ERROR
        break;
    case ROBOSZPON_AXIS_STATE_ERROR:
        // Delete all messages from message queue. If there is a disarm command,
        // set mode to DISARM, reset error LED and break. else, check for
        // errors. If there are no errors, reset error LED, set mode to
        // ROBOSZPON_AXIS_STATE_RUNNING and break.
        break;
    default:
        // What are you even doing here?
        break;
    }
}
