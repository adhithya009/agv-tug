/**
 ******************************************************************************
 * @file    : main.c
 * @brief   : Smart Autonomous AGV Tug - Main Firmware
 ******************************************************************************
 */

#include <stdint.h>

#include "stm32f4xx.h"

#include "system_clock.h"
#include "gpio_cfg.h"
#include "timer_cfg.h"

#include "motor.h"
#include "encoder.h"

#include "map.h"
#include "astar.h"
#include "navigation.h"


/*
 * ==============================================================
 * GLOBAL ROUTE STATE
 * ==============================================================
 *
 * Temporary global variables allow us to inspect
 * the route in CubeIDE.
 *
 * Later these will become part of the complete
 * robot-state/application layer.
 */
AStarPath g_route;

volatile uint8_t g_route_found = 0U;


/*
 * ==============================================================
 * MAIN
 * ==============================================================
 */
int main(void)
{
    /*
     * ----------------------------------------------------------
     * ENABLE FPU
     * ----------------------------------------------------------
     *
     * Enable Cortex-M4 CP10 and CP11 floating-point units.
     *
     * Required for:
     *
     * PID
     * Kalman
     * velocity
     * heading
     * pose calculations
     */
    SCB->CPACR |= (0xFU << 20);

    __DSB();
    __ISB();


    /*
     * ----------------------------------------------------------
     * SYSTEM CLOCK
     * ----------------------------------------------------------
     *
     * SYSCLK = 84 MHz
     * AHB    = 84 MHz
     * APB1   = 42 MHz
     * APB2   = 84 MHz
     */
    SystemClock_Init();


    /*
     * ----------------------------------------------------------
     * GPIO
     * ----------------------------------------------------------
     *
     * Configure:
     *
     * motor direction pins
     * motor PWM alternate functions
     * encoder alternate functions
     * IR inputs
     * SPI
     * I2C
     * UART
     * RFID control
     * onboard LED
     */
    GPIO_Init();


    /*
     * ----------------------------------------------------------
     * TIMERS
     * ----------------------------------------------------------
     *
     * TIM1 -> motor PWM
     *
     * TIM3 -> left quadrature encoder
     *
     * TIM4 -> right quadrature encoder
     *
     * TIM5 -> 10 ms control-loop timer
     */
    TIMER_Init();


    /*
     * ----------------------------------------------------------
     * MOTOR DRIVER
     * ----------------------------------------------------------
     *
     * Starts both motors safely stopped.
     */
    Motor_Init();


    /*
     * ----------------------------------------------------------
     * ENCODERS
     * ----------------------------------------------------------
     */
    Encoder_Init();


    /*
     * ----------------------------------------------------------
     * PHYSICAL MAP
     * ----------------------------------------------------------
     *
     * Initialize graph containing:
     *
     * S1-S6
     * J1-J10
     *
     * and clear temporary blocked-node state.
     */
    Map_Init();


    /*
     * ----------------------------------------------------------
     * NAVIGATION / PID
     * ----------------------------------------------------------
     *
     * Initial controller:
     *
     * Kp = 10
     * Ki = 0
     * Kd = 0
     *
     * dt = 0.010 seconds
     *
     * base speed = 30%
     *
     * Start with P-only control.
     * Add D only after physical tuning.
     */
    Navigation_Init(
        10.0f,
        0.0f,
        0.0f,
        0.010f,
        30U);


    /*
     * ----------------------------------------------------------
     * TEMPORARY START / DESTINATION
     * ----------------------------------------------------------
     *
     * For now:
     *
     * start = S1
     * goal  = S4
     *
     * Later:
     *
     * current position comes from localization/RFID.
     *
     * destination comes from HC-05.
     */
    MapNodeId start =
        MAP_S1;

    MapNodeId goal =
        MAP_S4;


    /*
     * ----------------------------------------------------------
     * A* ROUTE PLANNING
     * ----------------------------------------------------------
     */
    g_route_found =
        AStar_FindPath(
            start,
            goal,
            &g_route);


    /*
     * Give route to physical navigation if successful.
     */
    if (g_route_found)
    {
        Navigation_SetPath(
            &g_route);
    }
    else
    {
        /*
         * No valid route means robot must not move.
         */
        Navigation_Stop();
    }


    /*
     * ----------------------------------------------------------
     * MAIN CONTROL LOOP
     * ----------------------------------------------------------
     *
     * TIM5 generates one event every 10 ms.
     *
     * Therefore:
     *
     * control frequency = 100 Hz
     */
    while (1)
    {
        if (TIMER_ControlPeriodElapsed())
        {
            /*
             * Calculate wheel movement during the
             * latest control interval.
             */
            Encoder_Update();


            /*
             * Run:
             *
             * IR reading
             * line error
             * PID
             * junction handling
             * A* route execution
             * motor commands
             */
            Navigation_Update();
        }
    }
}
