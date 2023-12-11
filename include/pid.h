#ifndef PID_H
#define PID_H

typedef struct pid_controller pid_controller_t;

struct pid_controller {
    float Kp;
    float Ki;
    float Kd;
    float d_d;
    float deadzone;
    float u_max;
    float du_max;
    float prevE;
    float prevUd;
    float prevUi;
    float prevU;
};

float StepPid(pid_controller_t* pid, float e);

#endif // PID_H