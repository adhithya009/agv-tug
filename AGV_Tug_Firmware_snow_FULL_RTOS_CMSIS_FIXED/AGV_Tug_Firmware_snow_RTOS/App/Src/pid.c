#include "pid.h"

/*
 * Clamp one floating-point value between
 * a minimum and maximum limit.
 */
static float clampf(float value,
                    float minimum,
                    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

void PID_Init(PIDController *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    /*
     * Desired value defaults to zero.
     * For line following, zero means centred.
     */
    pid->setpoint = 0.0f;

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;

    /*
     * Basic integral limiting.
     */
    pid->integral_min = output_min;

    pid->integral_max = output_max;

    /*
     * Prevent derivative spike during
     * the first PID calculation.
     */
    pid->first_update = 1U;
}


void PID_SetSetpoint(PIDController *pid,
                     float setpoint)
{
    if (pid != 0)
    {
        pid->setpoint = setpoint;
    }
}

void PID_Reset(PIDController *pid)
{
    if (pid == 0)
    {
        return;
    }

    /*
     * Forget previous controller history.
     */
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->first_update = 1U;
}

float PID_Update(PIDController *pid,
                 float measurement,
                 float dt_seconds)
{
    float error;
    float derivative = 0.0f;
    float output;

    /*
     * Cannot calculate PID without
     * a controller or valid time step.
     */
    if ((pid == 0) ||
        (dt_seconds <= 0.0f))
    {
        return 0.0f;
    }

    /*
     * Standard closed-loop error:
     *
     * error = desired - measured
     */
    error =
        pid->setpoint - measurement;

    /*
     * Accumulate error over time.
     */
    pid->integral =
        clampf(pid->integral +
               (error * dt_seconds),
               pid->integral_min,
               pid->integral_max);
    /*
     * Do not calculate derivative on
     * the first PID sample.
     */
    if (!pid->first_update)
    {
        derivative =
            (error - pid->previous_error) /
            dt_seconds;
    }

    /*
     * PID = P + I + D.
     */
    output =
        (pid->kp * error) +
        (pid->ki * pid->integral) +
        (pid->kd * derivative);

    /*
     * Save error for next derivative calculation.
     */
    pid->previous_error = error;
    pid->first_update = 0U;

    /*
     * Prevent excessive motor correction.
     */
    return clampf(output,
                  pid->output_min,
                  pid->output_max);
}
