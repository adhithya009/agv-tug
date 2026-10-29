#include <stdint.h>

#include "stm32f4xx.h"

#include "system_clock.h"
#include "gpio_cfg.h"
#include "timer_cfg.h"
#include "i2c_cfg.h"
#include "spi_cfg.h"
#include "uart_cfg.h"

#include "motor.h"
#include "encoder.h"

#include "test_ir.h"
#include "test_encoder.h"
#include "test_motor.h"
#include "test_astar.h"
#include "test_pid.h"
#include "test_mpu6050.h"
#include "test_hc05.h"
#include "test_rfid.h"
#include "test_obstacle.h"
#include "test_oled.h"
#include "test_odometry.h"
#include "test_kalman.h"
#include "test_line_follow.h"
#include "test_i2c_scanner.h"
#include "test_motor_encoder.h"
#include "test_full_navigation.h"
#include "test_motor_forward.h"
#include "test_route_navigation.h"
#include "test_line_map_no_mpu.h"
#include "test_rtos_line_map.h"

/*
 * NEW:
 * Manual one-wheel-revolution encoder calibration test.
 */
#include "test_encoder_one_revolution.h"


/* ==============================================================
 * TEST IDS
 * ============================================================== */

#define TEST_IR                       1
#define TEST_ENCODER                  2
#define TEST_MOTOR                    3
#define TEST_ASTAR                    4
#define TEST_PID                      5
#define TEST_MPU6050                  6
#define TEST_HC05                     7
#define TEST_RFID                     8
#define TEST_OBSTACLE                 9
#define TEST_OLED                    10
#define TEST_ODOMETRY                11
#define TEST_KALMAN                  12
#define TEST_LINE_FOLLOW             13
#define TEST_I2C_SCANNER             14
#define TEST_MOTOR_ENCODER           15
#define TEST_FULL_NAVIGATION         16
#define TEST_MOTOR_FORWARD           17
#define TEST_ROUTE_NAVIGATION        18
#define TEST_LINE_MAP_NO_MPU         19

/*
 * NEW encoder CPR calibration test.
 */
#define TEST_ENCODER_ONE_REVOLUTION  20
#define TEST_RTOS_LINE_MAP           21


/* ==============================================================
 * ACTIVE TEST
 * ============================================================== */

/*
 * Rotate each wheel manually exactly one revolution.
 *
 * Motors remain stopped.
 */
#define ACTIVE_TEST TEST_RTOS_LINE_MAP


/* ==============================================================
 * MAIN DEBUG
 * ============================================================== */

volatile uint8_t
debug_main_stage =
    0U;


volatile uint8_t
debug_main_active_test =
    0U;


/* ==============================================================
 * MAIN
 * ============================================================== */

int main(void)
{
    debug_main_stage =
        1U;


    debug_main_active_test =
        ACTIVE_TEST;


    /*
     * Enable Cortex-M4 FPU.
     */
    SCB->CPACR |=
        (0xFU << 20U);


    __DSB();


    __ISB();


    /* ==========================================================
     * SYSTEM CLOCK
     * ========================================================== */

    SystemClock_Init();


    debug_main_stage =
        2U;


    /* ==========================================================
     * GPIO
     * ========================================================== */

    GPIO_Init();


    debug_main_stage =
        3U;


    /* ==========================================================
     * TIMERS
     *
     * TIM1 -> motor PWM
     * TIM3 -> left encoder
     * TIM4 -> right encoder
     * TIM5 -> 10 ms control timing
     * ========================================================== */

#if ACTIVE_TEST == TEST_RTOS_LINE_MAP

    /* FreeRTOS SysTick replaces TIM5 as the 10 ms scheduler. */
    TIMER_InitForRTOS();

#else

    /* Existing bare-metal tests continue to use TIM5. */
    TIMER_Init();

#endif


    debug_main_stage =
        4U;


    /* ==========================================================
     * I2C
     * ========================================================== */

    I2C1_Init();


    debug_main_stage =
        5U;


    /* ==========================================================
     * SPI
     * ========================================================== */

    SPI1_Init();


    debug_main_stage =
        6U;


    /* ==========================================================
     * UART
     * ========================================================== */

    UART1_Init(
        9600U);


    debug_main_stage =
        7U;


    /* ==========================================================
     * MOTOR
     * ========================================================== */

    Motor_Init();


    debug_main_stage =
        8U;


    /* ==========================================================
     * ENCODER
     *
     * Resets TIM3/TIM4 software state.
     * ========================================================== */

    /* Encoder_Init() bypassed: encoders removed */


    debug_main_stage =
        9U;


    /*
     * All peripheral initialization complete.
     */
    debug_main_stage =
        10U;


    /* ==========================================================
     * TEST SELECTION
     * ========================================================== */

#if ACTIVE_TEST == TEST_IR

    Test_IR();


#elif ACTIVE_TEST == TEST_ENCODER

    Test_Encoder();


#elif ACTIVE_TEST == TEST_MOTOR

    Test_Motor();


#elif ACTIVE_TEST == TEST_ASTAR

    Test_AStar();


#elif ACTIVE_TEST == TEST_PID

    Test_PID();


#elif ACTIVE_TEST == TEST_MPU6050

    Test_MPU6050();


#elif ACTIVE_TEST == TEST_HC05

    Test_HC05();


#elif ACTIVE_TEST == TEST_RFID

    Test_RFID();


#elif ACTIVE_TEST == TEST_OBSTACLE

    Test_Obstacle();


#elif ACTIVE_TEST == TEST_OLED

    Test_OLED();


#elif ACTIVE_TEST == TEST_ODOMETRY

    Test_Odometry();


#elif ACTIVE_TEST == TEST_KALMAN

    Test_Kalman();


#elif ACTIVE_TEST == TEST_LINE_FOLLOW

    Test_LineFollow();


#elif ACTIVE_TEST == TEST_I2C_SCANNER

    Test_I2CScanner();


#elif ACTIVE_TEST == TEST_MOTOR_ENCODER

    Test_MotorEncoder();


#elif ACTIVE_TEST == TEST_FULL_NAVIGATION

    Test_FullNavigation();


#elif ACTIVE_TEST == TEST_MOTOR_FORWARD

    Test_MotorForward();


#elif ACTIVE_TEST == TEST_ROUTE_NAVIGATION

    Test_RouteNavigation();


#elif ACTIVE_TEST == TEST_LINE_MAP_NO_MPU

    Test_LineMapNoMPU();


#elif ACTIVE_TEST == TEST_ENCODER_ONE_REVOLUTION

    /*
     * TEST 20.
     *
     * Motors remain stopped.
     * Rotate wheels manually.
     */
    Test_EncoderOneRevolution();


#elif ACTIVE_TEST == TEST_RTOS_LINE_MAP

    /* Fully integrated FreeRTOS navigation test. */
    Test_RTOSLineMap();


#else

#error "Invalid ACTIVE_TEST selected"

#endif


    /* ==========================================================
     * SAFETY FALLBACK
     * ========================================================== */

    while (1)
    {
        Motor_Stop();
    }
}
