#include "obstacle.h"

#include "ir_array.h"

#include <stdint.h>


/* ==============================================================
 * OBSTACLE SENSOR SETTINGS
 * ============================================================== */

/*
 * Hardware:
 *
 * A5 / PC0 -> OBSTACLE IR
 *
 * Current polarity assumption:
 *
 * LOW  = obstacle detected
 * HIGH = no obstacle
 *
 * If physical testing shows the opposite,
 * change this from 0U to 1U.
 */
#define OBSTACLE_ACTIVE_LEVEL      0U


/*
 * Require three consecutive active samples
 * before declaring an obstacle.
 *
 * Navigation/control loop = 100 Hz
 *
 * 3 samples x 10 ms = 30 ms
 */
#define OBSTACLE_DEBOUNCE_COUNT    10U


/* ==============================================================
 * PRIVATE STATE
 * ============================================================== */

static uint8_t detection_count =
    0U;

static uint8_t detected =
    0U;


/* ==============================================================
 * INITIALIZATION
 * ============================================================== */

void Obstacle_Init(void)
{
    detection_count =
        0U;

    detected =
        0U;
}


/* ==============================================================
 * RAW SENSOR READ
 *
 * A5 / PC0
 *
 * Returns the electrical GPIO state:
 *
 * 0 = LOW
 * 1 = HIGH
 * ============================================================== */

uint8_t Obstacle_ReadRaw(void)
{
    /*
     * Pin reading is handled by ir_array.c.
     *
     * This avoids defining PC0 in multiple drivers.
     */
    return
        IRArray_ReadObstacleRaw();
}


/* ==============================================================
 * UPDATE OBSTACLE STATE
 *
 * Call this once every 10 ms.
 * ============================================================== */

void Obstacle_Update(void)
{
    uint8_t raw =
        Obstacle_ReadRaw();


    /*
     * Sensor is currently reporting
     * the obstacle-active level.
     */
    if (raw ==
        OBSTACLE_ACTIVE_LEVEL)
    {
        /*
         * Increase debounce counter without
         * allowing uint8_t overflow.
         */
        if (detection_count <
            OBSTACLE_DEBOUNCE_COUNT)
        {
            detection_count++;
        }


        /*
         * Obstacle must remain detected for
         * three consecutive samples.
         */
        if (detection_count >=
            OBSTACLE_DEBOUNCE_COUNT)
        {
            detected =
                1U;
        }
    }
    else
    {
        /*
         * Sensor is clear.
         *
         * Reset debounce immediately.
         */
        detection_count =
            0U;

        detected =
            0U;
    }
}


/* ==============================================================
 * GET FILTERED OBSTACLE STATE
 *
 * Returns:
 *
 * 1 = confirmed obstacle
 * 0 = no confirmed obstacle
 * ============================================================== */

uint8_t Obstacle_IsDetected(void)
{
    return
        detected;
}
