#include "test_motor_encoder.h"

#include <stdint.h>

#include "stm32f4xx.h"
#include "gpio_cfg.h"
#include "timer_cfg.h"
#include "encoder.h"
#include "motor.h"


/*
 * ==============================================================
 * TEST SETTINGS
 * ==============================================================
 *
 * Both motors receive exactly the same PWM.
 *
 * Control period = 10 ms
 *
 * 500 samples × 10 ms = 5000 ms = 5 seconds
 */

#define TEST_MOTOR_SPEED_PERCENT     75.0f
#define TEST_DURATION_TICKS          500U

#define LEFT_ENCODER_CPR             10.0f
#define RIGHT_ENCODER_CPR            11.0f


/*
 * ==============================================================
 * DEBUG VARIABLES
 * ==============================================================
 */

/*
 * 0 = test still running
 * 1 = test completed
 */
volatile uint8_t debug_motor_encoder_done = 0U;


/*
 * Number of 10 ms periods completed.
 */
volatile uint32_t debug_motor_encoder_ticks = 0U;


/*
 * Approximate elapsed test time.
 */
volatile uint32_t debug_motor_encoder_ms = 0U;


/*
 * Final/continuous encoder counts.
 */
volatile uint16_t debug_motor_left_count = 0U;
volatile uint16_t debug_motor_right_count = 0U;


/*
 * Latest 10 ms encoder increments.
 */
volatile int16_t debug_motor_left_delta = 0;
volatile int16_t debug_motor_right_delta = 0;


/*
 * Calculated number of wheel revolutions.
 *
 * left:
 * counts / 10 CPR
 *
 * right:
 * counts / 11 CPR
 */
volatile float debug_motor_left_revolutions = 0.0f;
volatile float debug_motor_right_revolutions = 0.0f;


/*
 * Difference between actual wheel revolutions.
 *
 * Ideally close to zero if both wheels rotate
 * at approximately the same speed.
 */
volatile float debug_motor_revolution_difference = 0.0f;


/*
 * ==============================================================
 * CONVERT PWM PERCENTAGE TO TIM1 CCR
 * ==============================================================
 */

static uint32_t Test_SpeedToCCR(float speed_percent)
{
    if (speed_percent > 100.0f)
    {
        speed_percent = 100.0f;
    }

    if (speed_percent < 0.0f)
    {
        speed_percent = 0.0f;
    }

    return (uint32_t)(
        ((float)(TIM1->ARR + 1U) *
         speed_percent) /
        100.0f);
}


/*
 * ==============================================================
 * START BOTH MOTORS FORWARD AT 75%
 * ==============================================================
 */

static void Test_StartBothMotors(void)
{
    /*
     * Left motor:
     *
     * PB1  IN1 = HIGH
     * PB15 IN2 = LOW
     *
     * Right motor:
     *
     * PB14 IN3 = HIGH
     * PB13 IN4 = LOW
     */
    GPIO_MotorDirection(
        1U,
        0U,
        1U,
        0U);


    /*
     * Left PWM:
     *
     * PA8 -> TIM1_CH1.
     */
    TIM1->CCR1 =
        Test_SpeedToCCR(
            TEST_MOTOR_SPEED_PERCENT);


    /*
     * Right PWM:
     *
     * PA11 -> TIM1_CH4.
     */
    TIM1->CCR4 =
        Test_SpeedToCCR(
            TEST_MOTOR_SPEED_PERCENT);
}


/*
 * ==============================================================
 * UPDATE DEBUG VALUES
 * ==============================================================
 */

static void Test_UpdateEncoderDebug(void)
{
    /*
     * Read latest hardware encoder counts.
     */
    Encoder_Update();


    debug_motor_left_count =
        Encoder_GetLeftRaw();

    debug_motor_right_count =
        Encoder_GetRightRaw();


    debug_motor_left_delta =
        Encoder_GetLeftDelta();

    debug_motor_right_delta =
        Encoder_GetRightDelta();


    /*
     * Convert counts into actual wheel revolutions.
     *
     * CPR differs slightly between the two encoders
     * based on our measured calibration.
     */
    debug_motor_left_revolutions =
        (float)debug_motor_left_count /
        LEFT_ENCODER_CPR;


    debug_motor_right_revolutions =
        (float)debug_motor_right_count /
        RIGHT_ENCODER_CPR;


    /*
     * Positive:
     * left wheel rotated more.
     *
     * Negative:
     * right wheel rotated more.
     */
    debug_motor_revolution_difference =
        debug_motor_left_revolutions -
        debug_motor_right_revolutions;
}


/*
 * ==============================================================
 * MOTOR + ENCODER TEST
 * ==============================================================
 */

void Test_MotorEncoder(void)
{
    /*
     * Always begin safely.
     */
    Motor_Stop();


    /*
     * Reset hardware + software encoder counters.
     */
    Encoder_Reset();


    /*
     * Reset debugger results.
     */
    debug_motor_encoder_done = 0U;

    debug_motor_encoder_ticks = 0U;

    debug_motor_encoder_ms = 0U;

    debug_motor_left_count = 0U;

    debug_motor_right_count = 0U;

    debug_motor_left_delta = 0;

    debug_motor_right_delta = 0;

    debug_motor_left_revolutions = 0.0f;

    debug_motor_right_revolutions = 0.0f;

    debug_motor_revolution_difference = 0.0f;


    /*
     * Start BOTH motors at exactly 75%.
     */
    Test_StartBothMotors();


    /*
     * ==========================================================
     * RUN FOR EXACTLY 500 CONTROL PERIODS
     * ==========================================================
     *
     * 500 × 10 ms = 5 seconds.
     */

    while (debug_motor_encoder_ticks <
           TEST_DURATION_TICKS)
    {
        if (TIMER_ControlPeriodElapsed())
        {
            /*
             * Read encoder counts during motor operation.
             */
            Test_UpdateEncoderDebug();


            /*
             * One additional 10 ms period completed.
             */
            debug_motor_encoder_ticks++;


            /*
             * Convert tick count into milliseconds.
             */
            debug_motor_encoder_ms =
                debug_motor_encoder_ticks *
                10U;
        }
    }


    /*
     * ==========================================================
     * STOP BOTH MOTORS
     * ==========================================================
     */

    Motor_Stop();


    /*
     * Read encoder counts once more after stopping.
     */
    Test_UpdateEncoderDebug();


    /*
     * Mark test completed.
     */
    debug_motor_encoder_done = 1U;


    /*
     * ==========================================================
     * HOLD RESULTS
     * ==========================================================
     */

    while (1)
    {
        Motor_Stop();
    }
}
