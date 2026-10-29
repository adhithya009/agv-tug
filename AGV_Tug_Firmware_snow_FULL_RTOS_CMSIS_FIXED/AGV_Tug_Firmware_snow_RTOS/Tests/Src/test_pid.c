#include "test_pid.h"

#include <stdint.h>

#include "pid.h"
#include "motor.h"


/*
 * ==============================================================
 * SOFTWARE PID TEST INPUTS
 * ==============================================================
 *
 * These simulate line positions:
 *
 * +2 -> line strongly right
 * +1 -> line slightly right
 *  0 -> centred
 * -1 -> line slightly left
 * -2 -> line strongly left
 */
static const float test_measurements[5] =
{
     2.0f,
     1.0f,
     0.0f,
    -1.0f,
    -2.0f
};


/*
 * PID outputs can be inspected in CubeIDE.
 */
volatile float debug_pid_output[5];


/*
 * ==============================================================
 * PID TEST
 * ==============================================================
 */
void Test_PID(void)
{
    PIDController pid;


    /*
     * Software-only test.
     */
    Motor_Stop();


    /*
     * P-only controller:
     *
     * Kp = 10
     * Ki = 0
     * Kd = 0
     *
     * output limited to +/-40.
     */
    PID_Init(
        &pid,
        10.0f,
        0.0f,
        0.0f,
        -40.0f,
        40.0f);


    /*
     * Desired line position is centre.
     */
    PID_SetSetpoint(
        &pid,
        0.0f);


    /*
     * Feed all simulated line measurements.
     */
    for (uint8_t i = 0U;
         i < 5U;
         i++)
    {
        debug_pid_output[i] =
            PID_Update(
                &pid,
                test_measurements[i],
                0.010f);
    }


    /*
     * Keep values available for debugger.
     */
    while (1)
    {
        Motor_Stop();
    }
}
