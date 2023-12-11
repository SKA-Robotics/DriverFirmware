#include "pid.h"
#include "output.h"
#include <math.h>

float StepPid(pid_controller_t* pid, float e) {
    if (fabs(e) < pid->deadzone) {
        e = 0.0f;
    }

    float u_d = (1 - pid->d_d) * (e - pid->prevE) + pid->d_d * pid->prevUd;
    float u_i = e + pid->prevUi;
    float u = pid->Kp * e + pid->Ki * u_i + pid->Kd * u_d;

    // Clamp U signal increment
    float du = u - pid->prevU;
    if (du > pid->du_max) {
        du = pid->du_max;
    } else if (du < -pid->du_max) {
        du = -pid->du_max;
    }

    u = pid->prevU + du;

    // Clamp U signal
    if (u > pid->u_max) {
        // printf("\tClamping top\t");

        u = pid->u_max;
    } else if (u < -pid->u_max) {
        // printf("\tClamping bottom\t");
        u = -pid->u_max;
    }

    pid->prevE = e;
    pid->prevU = u;
    pid->prevUd = u_d;
    pid->prevUi = u_i;
    return u;
}