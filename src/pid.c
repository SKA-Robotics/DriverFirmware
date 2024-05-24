#include "pid.h"
#include "output.h"
#include "system.h"
#include <math.h>

float StepPid(pid_controller_t* pid, float e) {
    if (fabs(e) < pid->deadzone) {
        e = 0.0f;
    }

    float u_d = (e - pid->prevE) / DELTA_TIME;
    float u_i =
        e * DELTA_TIME + pid->prevUi + pid->Kaw * pid->antiwindup_correction;
    float u_raw = pid->Kp * e + pid->Ki * u_i + pid->Kd * u_d;

    // Clamp U signal increment
    float du = u_raw - pid->prevU;
    if (du > pid->du_max) {
        du = pid->du_max;
    } else if (du < -pid->du_max) {
        du = -pid->du_max;
    }

    float u = pid->prevU + du;

    // Clamp U signal
    if (u > pid->u_max) {
        u = pid->u_max;
    } else if (u < -pid->u_max) {
        u = -pid->u_max;
    }

    pid->antiwindup_correction = u - u_raw;

    pid->prevE = e;
    pid->prevU = u;
    pid->prevUi = u_i;
    return u;
}