#include "test_line_follow.h"

#include <stdint.h>

#include "stm32f4xx.h"
#include "gpio_cfg.h"
#include "ir_array.h"
#include "timer_cfg.h"

/*
 * ==============================================================
 * LINE FOLLOW SETTINGS
 * ==============================================================
 */

#define LINE_BASE_SPEED   75.0f
#define LINE_KP            2.5f
#define LINE_MIN_SPEED    70.0f
#define LINE_MAX_SPEED    80.0f

/*
 * ==============================================================
 * DEBUG VARIABLES
 * ==============================================================
 */

/*
 * Raw electrical levels directly from STM32 pins.
 *
 * Left   IR -> PA0
 * Centre IR -> PA1
 * Right  IR -> PA4
 */
volatile uint8_t debug_raw_left = 0U;
volatile uint8_t debug_raw_center = 0U;
volatile uint8_t debug_raw_right = 0U;


/*
 * Processed IR pattern returned by IRArray_ReadPattern().
 */
volatile uint8_t debug_line_pattern = 0U;


/*
 * Processed line position error.
 */
volatile int8_t debug_follow_error = 0;


/*
 * Steering calculation.
 */
volatile float debug_line_correction = 0.0f;


/*
 * Final requested motor speeds.
 */
volatile float debug_line_left_speed = 0.0f;
volatile float debug_line_right_speed = 0.0f;


/*
 * Special line states.
 */
volatile uint8_t debug_line_lost = 0U;
volatile uint8_t debug_line_junction = 0U;


/*
 * ==============================================================
 * LIMIT MOTOR SPEED
 * ==============================================================
 *
 * When a motor is moving, keep it between 70% and 80%.
 */

static float Line_ClampSpeed(float speed)
{
    if (speed > LINE_MAX_SPEED)
    {
        speed = LINE_MAX_SPEED;
    }

    if (speed < LINE_MIN_SPEED)
    {
        speed = LINE_MIN_SPEED;
    }

    return speed;
}


/*
 * ==============================================================
 * CONVERT SPEED % TO TIM1 CCR
 * ==============================================================
 */

static uint32_t Line_SpeedToCCR(float speed)
{
    float ccr;

    speed = Line_ClampSpeed(speed);

    ccr =
        ((float)(TIM1->ARR + 1U) *
         speed) /
        100.0f;

    return (uint32_t)ccr;
}


/*
 * ==============================================================
 * DRIVE BOTH MOTORS FORWARD
 * ==============================================================
 */

static void Line_SetMotorSpeeds(float left,
                                float right)
{
    left = Line_ClampSpeed(left);

    right = Line_ClampSpeed(right);


    /*
     * LEFT MOTOR:
     *
     * PB1  = IN1 HIGH
     * PB15 = IN2 LOW
     *
     * RIGHT MOTOR:
     *
     * PB14 = IN3 HIGH
     * PB13 = IN4 LOW
     */
    GPIO_MotorDirection(
        1U,
        0U,
        1U,
        0U);


    /*
     * Left motor PWM:
     *
     * PA8 -> TIM1_CH1.
     */
    TIM1->CCR1 =
        Line_SpeedToCCR(left);


    /*
     * Right motor PWM:
     *
     * PA11 -> TIM1_CH4.
     */
    TIM1->CCR4 =
        Line_SpeedToCCR(right);
}


/*
 * ==============================================================
 * STOP BOTH MOTORS
 * ==============================================================
 */

static void Line_Stop(void)
{
    /*
     * Stop means real 0% duty cycle.
     *
     * Do NOT use Line_ClampSpeed() here.
     */
    TIM1->CCR1 = 0U;

    TIM1->CCR4 = 0U;


    /*
     * Disable all L298N direction inputs.
     */
    GPIO_MotorDirection(
        0U,
        0U,
        0U,
        0U);
}


/*
 * ==============================================================
 * LINE FOLLOW TEST
 * ==============================================================
 */

void Test_LineFollow(void)
{
    /*
     * Always begin with motors stopped.
     */
    Line_Stop();


    while (1)
    {
        /*
         * Execute approximately every control period.
         */
        if (TIMER_ControlPeriodElapsed())
        {
            /*
             * ==================================================
             * READ RAW IR GPIO LEVELS
             * ==================================================
             *
             * These values are taken BEFORE any inversion or
             * processing inside IRArray_ReadPattern().
             *
             * PA0 = left sensor
             * PA1 = centre sensor
             * PA4 = right sensor
             */


            debug_raw_left =
                (uint8_t)
                ((GPIOA->IDR >> 0U) & 0x01U);


            debug_raw_center =
                (uint8_t)
                ((GPIOA->IDR >> 1U) & 0x01U);


            debug_raw_right =
                (uint8_t)
                ((GPIOA->IDR >> 4U) & 0x01U);


            /*
             * ==================================================
             * READ PROCESSED IR PATTERN
             * ==================================================
             */

            debug_line_pattern =
                IRArray_ReadPattern();


            /*
             * 000 means the software believes
             * no sensor is detecting the line.
             */
            debug_line_lost =
                (debug_line_pattern == 0x00U);


            /*
             * 111 means all three processed sensors
             * detect black.
             */
            debug_line_junction =
                IRArray_IsJunctionCandidate(
                    debug_line_pattern);


            /*
             * ==================================================
             * LINE LOST
             * ==================================================
             */

            if (debug_line_lost)
            {
                debug_follow_error = 0;

                debug_line_correction = 0.0f;

                debug_line_left_speed = 0.0f;

                debug_line_right_speed = 0.0f;


                Line_Stop();


                continue;
            }


            /*
             * ==================================================
             * JUNCTION
             * ==================================================
             */

            if (debug_line_junction)
            {
                debug_follow_error = 0;

                debug_line_correction = 0.0f;

                debug_line_left_speed = 0.0f;

                debug_line_right_speed = 0.0f;


                Line_Stop();


                continue;
            }


            /*
             * ==================================================
             * CALCULATE LINE ERROR
             * ==================================================
             *
             * Expected mapping:
             *
             * 001 -> -2
             * 011 -> -1
             * 010 ->  0
             * 110 -> +1
             * 100 -> +2
             */

            debug_follow_error =
                IRArray_GetLineError(
                    debug_line_pattern);


            /*
             * ==================================================
             * P CONTROLLER
             * ==================================================
             *
             * Desired line error = 0.
             */

            debug_line_correction =
                -LINE_KP *
                (float)debug_follow_error;


            /*
             * ==================================================
             * MOTOR MIXING
             * ==================================================
             */

            debug_line_left_speed =
                LINE_BASE_SPEED -
                debug_line_correction;


            debug_line_right_speed =
                LINE_BASE_SPEED +
                debug_line_correction;


            /*
             * Keep each moving wheel between
             * 70% and 80%.
             */

            debug_line_left_speed =
                Line_ClampSpeed(
                    debug_line_left_speed);


            debug_line_right_speed =
                Line_ClampSpeed(
                    debug_line_right_speed);


            /*
             * ==================================================
             * APPLY MOTOR COMMAND
             * ==================================================
             */

            Line_SetMotorSpeeds(
                debug_line_left_speed,
                debug_line_right_speed);
        }
    }
}
