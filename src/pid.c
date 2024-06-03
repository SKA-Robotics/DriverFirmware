#include "pid.h"
#include "output.h"
#include "system.h"
#include <math.h>

float StepPid(pid_controller_t* pid, float e) {
    if (fabs(e) < pid->deadzone) {
        e = 0.0f;
    }

    float u_p = pid->Kp * e;
    float u_i = pid->Ki * (e * DELTA_TIME) + pid->prevUi;
    float u_d = pid->Kd * (e - pid->prevE) / DELTA_TIME;
    float u_raw = u_p + u_i + u_d;

    // Clamp U signal increment
    float du = u_raw - pid->prevU;
    if (du > pid->duMax) {
        du = pid->duMax;
    } else if (du < -pid->duMax) {
        du = -pid->duMax;
    }

    float u = pid->prevU + du;

    // Clamp U signal
    if (u > pid->uMax) {
        u = pid->uMax;
        // Anti-windup protection
        u_i = pid->uMax - u_p - u_d;
        if (u_i < 0.0f) {
            u_i = 0.0f;
        }
    } else if (u < pid->uMin) {
        u = pid->uMin;
        u_i = pid->uMin - u_p - u_d;
        if (u_i > 0.0f) {
            u_i = 0.0f;
        }
    }

    pid->prevE = e;
    pid->prevU = u;
    pid->prevUi = u_i;
    return u;
}