#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float previous_error;

    float output_min;
    float output_max;

    float integral_min;
    float integral_max;

    uint8_t first_update;

} PIDController;


void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);
void PID_SetSetpoint(PIDController *pid,
                     float setpoint);


float PID_Update(PIDController *pid,
                 float measurement,
                 float dt_seconds);


void PID_Reset(PIDController *pid);

#endif
