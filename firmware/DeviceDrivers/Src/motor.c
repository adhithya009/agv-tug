#include "stm32f4xx.h"

#include "motor.h"
#include "gpio_cfg.h"
#include "timer_cfg.h"


#define MOTOR_MAX_PWM_PERCENT 100

#define LEFT_MOTOR_INVERT   1
#define RIGHT_MOTOR_INVERT  0


static int8_t motor_left_command = 0;
static int8_t motor_right_command = 0;

static int8_t motor_left_hw_previous = 0;
static int8_t motor_right_hw_previous = 0;


/* ==============================================================
 * CLAMP SPEED
 * ============================================================== */

static int8_t Motor_ClampSpeed(
    int8_t speed)
{
    if (speed > MOTOR_MAX_PWM_PERCENT)
    {
        speed = MOTOR_MAX_PWM_PERCENT;
    }

    if (speed < -MOTOR_MAX_PWM_PERCENT)
    {
        speed = -MOTOR_MAX_PWM_PERCENT;
    }

    return speed;
}


/* ==============================================================
 * GET MAGNITUDE
 * ============================================================== */

static uint8_t Motor_GetMagnitude(
    int8_t speed)
{
    if (speed < 0)
    {
        return (uint8_t)(-speed);
    }

    return (uint8_t)speed;
}


/* ==============================================================
 * GET SIGN
 * ============================================================== */

static int8_t Motor_GetSign(
    int8_t value)
{
    if (value > 0)
    {
        return 1;
    }

    if (value < 0)
    {
        return -1;
    }

    return 0;
}


/* ==============================================================
 * REVERSAL DEAD TIME
 *
 * Prevents changing H-bridge direction while PWM is active.
 * ============================================================== */

static void Motor_ReversalDeadTime(void)
{
    for (volatile uint32_t i = 0U;
         i < 2500U;
         i++)
    {
        __NOP();
    }
}


/* ==============================================================
 * APPLY MOTOR COMMAND
 * ============================================================== */

static void Motor_Apply(
    int8_t left_command,
    int8_t right_command)
{
    left_command =
        Motor_ClampSpeed(
            left_command);

    right_command =
        Motor_ClampSpeed(
            right_command);


    motor_left_command =
        left_command;

    motor_right_command =
        right_command;


    int8_t left_hw =
        left_command;

    int8_t right_hw =
        right_command;


#if LEFT_MOTOR_INVERT
    left_hw = -left_hw;
#endif


#if RIGHT_MOTOR_INVERT
    right_hw = -right_hw;
#endif


    uint8_t left_reversing =
        0U;

    uint8_t right_reversing =
        0U;


    if ((Motor_GetSign(
             motor_left_hw_previous) != 0) &&
        (Motor_GetSign(
             left_hw) != 0) &&
        (Motor_GetSign(
             motor_left_hw_previous) !=
         Motor_GetSign(
             left_hw)))
    {
        left_reversing =
            1U;
    }


    if ((Motor_GetSign(
             motor_right_hw_previous) != 0) &&
        (Motor_GetSign(
             right_hw) != 0) &&
        (Motor_GetSign(
             motor_right_hw_previous) !=
         Motor_GetSign(
             right_hw)))
    {
        right_reversing =
            1U;
    }


    /*
     * When changing polarity:
     *
     * PWM OFF
     * direction changes
     * PWM ON
     */
    if (left_reversing ||
        right_reversing)
    {
        TIMER_SetLeftPWM(
            0U);

        TIMER_SetRightPWM(
            0U);


        Motor_ReversalDeadTime();
    }


    uint8_t in1 = 0U;
    uint8_t in2 = 0U;
    uint8_t in3 = 0U;
    uint8_t in4 = 0U;


    /*
     * LEFT MOTOR.
     */
    if (left_hw > 0)
    {
        in1 = 1U;
        in2 = 0U;
    }
    else if (left_hw < 0)
    {
        in1 = 0U;
        in2 = 1U;
    }


    /*
     * RIGHT MOTOR.
     */
    if (right_hw > 0)
    {
        in3 = 1U;
        in4 = 0U;
    }
    else if (right_hw < 0)
    {
        in3 = 0U;
        in4 = 1U;
    }


    GPIO_MotorDirection(
        in1,
        in2,
        in3,
        in4);


    TIMER_SetLeftPWM(
        Motor_GetMagnitude(
            left_hw));


    TIMER_SetRightPWM(
        Motor_GetMagnitude(
            right_hw));


    motor_left_hw_previous =
        left_hw;

    motor_right_hw_previous =
        right_hw;
}


/* ==============================================================
 * INITIALIZATION
 * ============================================================== */

void Motor_Init(void)
{
    motor_left_command =
        0;

    motor_right_command =
        0;


    motor_left_hw_previous =
        0;

    motor_right_hw_previous =
        0;


    TIMER_SetLeftPWM(
        0U);

    TIMER_SetRightPWM(
        0U);


    GPIO_MotorDirection(
        0U,
        0U,
        0U,
        0U);
}


/* ==============================================================
 * SET BOTH MOTORS
 * ============================================================== */

void Motor_Set(
    int8_t left_percent,
    int8_t right_percent)
{
    Motor_Apply(
        left_percent,
        right_percent);
}


/* ==============================================================
 * SET LEFT
 * ============================================================== */

void Motor_SetLeft(
    int8_t speed_percent)
{
    Motor_Apply(
        speed_percent,
        motor_right_command);
}


/* ==============================================================
 * SET RIGHT
 * ============================================================== */

void Motor_SetRight(
    int8_t speed_percent)
{
    Motor_Apply(
        motor_left_command,
        speed_percent);
}


/* ==============================================================
 * STOP
 * ============================================================== */

void Motor_Stop(void)
{
    motor_left_command =
        0;

    motor_right_command =
        0;


    motor_left_hw_previous =
        0;

    motor_right_hw_previous =
        0;


    TIMER_SetLeftPWM(
        0U);

    TIMER_SetRightPWM(
        0U);


    GPIO_MotorDirection(
        0U,
        0U,
        0U,
        0U);
}


/* ==============================================================
 * FORWARD
 * ============================================================== */

void Motor_Forward(
    uint8_t speed_percent)
{
    if (speed_percent >
        MOTOR_MAX_PWM_PERCENT)
    {
        speed_percent =
            MOTOR_MAX_PWM_PERCENT;
    }


    Motor_Set(
        (int8_t)speed_percent,
        (int8_t)speed_percent);
}


/* ==============================================================
 * BACKWARD
 * ============================================================== */

void Motor_Backward(
    uint8_t speed_percent)
{
    if (speed_percent >
        MOTOR_MAX_PWM_PERCENT)
    {
        speed_percent =
            MOTOR_MAX_PWM_PERCENT;
    }


    Motor_Set(
        -(int8_t)speed_percent,
        -(int8_t)speed_percent);
}


/* ==============================================================
 * DIFFERENTIAL LEFT
 * ============================================================== */

void Motor_TurnLeft(
    uint8_t speed_percent)
{
    if (speed_percent >
        MOTOR_MAX_PWM_PERCENT)
    {
        speed_percent =
            MOTOR_MAX_PWM_PERCENT;
    }


    Motor_Set(
        -(int8_t)speed_percent,
         (int8_t)speed_percent);
}


/* ==============================================================
 * DIFFERENTIAL RIGHT
 * ============================================================== */

void Motor_TurnRight(
    uint8_t speed_percent)
{
    if (speed_percent >
        MOTOR_MAX_PWM_PERCENT)
    {
        speed_percent =
            MOTOR_MAX_PWM_PERCENT;
    }


    Motor_Set(
         (int8_t)speed_percent,
        -(int8_t)speed_percent);
}
