#include "test_obstacle.h"

#include <stdint.h>

#include "obstacle.h"
#include "motor.h"
#include "timer_cfg.h"


/* ==============================================================
 * OBSTACLE TEST DEBUG VARIABLES
 *
 * Names are intentionally unique so they do not conflict
 * with debug variables inside test_ir.c.
 * ============================================================== */

/*
 * Raw electrical value from obstacle IR.
 *
 * A5 / PC0
 *
 * Current expected polarity:
 *
 * 1 = no obstacle
 * 0 = obstacle
 */
volatile uint8_t debug_obstacle_test_raw =
    0U;


/*
 * Debounced obstacle result.
 *
 * 0 = no obstacle
 * 1 = confirmed obstacle
 */
volatile uint8_t debug_obstacle_test_detected =
    0U;


/*
 * Number of 10 ms test cycles.
 */
volatile uint32_t debug_obstacle_test_ticks =
    0U;


/* ==============================================================
 * OBSTACLE SENSOR TEST
 * ============================================================== */

void Test_Obstacle(void)
{
    /*
     * Sensor-only test.
     *
     * Motors must never move.
     */
    Motor_Stop();


    /*
     * Reset obstacle debounce logic.
     */
    Obstacle_Init();


    debug_obstacle_test_raw =
        0U;


    debug_obstacle_test_detected =
        0U;


    debug_obstacle_test_ticks =
        0U;


    while (1)
    {
        /*
         * Safety:
         *
         * Keep motors stopped continuously.
         */
        Motor_Stop();


        /*
         * Execute test every 10 ms.
         */
        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }


        debug_obstacle_test_ticks++;


        /*
         * Read raw A5 / PC0 obstacle sensor.
         */
        debug_obstacle_test_raw =
            Obstacle_ReadRaw();


        /*
         * Update 3-sample / 30 ms debounce.
         */
        Obstacle_Update();


        /*
         * Read filtered obstacle result.
         */
        debug_obstacle_test_detected =
            Obstacle_IsDetected();
    }
}
