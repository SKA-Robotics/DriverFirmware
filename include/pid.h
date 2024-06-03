#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float deadzone;
    float uMin;
    float uMax;
    float duMax;
    float prevE;
    float prevUi;
    float prevU;
    float antiwindup_correction;
} pid_controller_t;

float StepPid(pid_controller_t* pid, float e);

#endif // PID_H