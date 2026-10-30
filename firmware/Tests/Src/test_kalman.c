#include "test_kalman.h"

#include <stdint.h>

#include "kalman.h"
#include "stm32f4xx.h"


/*
 * ==============================================================
 * DEBUG VARIABLES
 * ==============================================================
 *
 * Watch these in CubeIDE Expressions:
 *
 * debug_kalman_index
 * debug_kalman_input
 * debug_kalman_output
 */

volatile uint8_t debug_kalman_index = 0U;

volatile float debug_kalman_input = 0.0f;

volatile float debug_kalman_output = 0.0f;


/*
 * ==============================================================
 * TEST FILTER INSTANCE
 * ==============================================================
 */

static Kalman1D test_filter;


/*
 * ==============================================================
 * ARTIFICIAL NOISY SENSOR DATA
 * ==============================================================
 *
 * These values imitate a noisy gyro signal.
 *
 * The raw input jumps sharply.
 * The Kalman output should change more smoothly.
 */

static const float test_data[] =
{
    0.0f,
    8.0f,
    -5.0f,
    6.0f,
    -3.0f,
    4.0f,
    -2.0f,
    1.0f,
    0.0f
};


/*
 * Number of samples in test_data[].
 */
#define KALMAN_TEST_SAMPLE_COUNT \
    ((uint8_t)(sizeof(test_data) / sizeof(test_data[0])))


/*
 * ==============================================================
 * SIMPLE DEBUG DELAY
 * ==============================================================
 *
 * This is intentionally slow so the changing values
 * are easier to observe using the debugger.
 */

static void Test_KalmanDelay(void)
{
    for (volatile uint32_t i = 0U;
         i < 4000000U;
         i++)
    {
        __NOP();
    }
}


/*
 * ==============================================================
 * KALMAN FILTER TEST
 * ==============================================================
 */

void Test_Kalman(void)
{
    /*
     * Initialize the filter.
     *
     * Q = process noise.
     * R = measurement noise.
     *
     * These are test values, not final tuned values.
     */
    Kalman1D_Init(
        &test_filter,
        0.01f,
        0.10f,
        0.0f,
        1.0f);


    while (1)
    {
        /*
         * Feed every artificial sensor sample
         * into the Kalman filter.
         */
        for (uint8_t i = 0U;
             i < KALMAN_TEST_SAMPLE_COUNT;
             i++)
        {
            /*
             * Show which sample is currently active.
             */
            debug_kalman_index = i;


            /*
             * Copy the noisy measurement
             * so CubeIDE can display it.
             */
            debug_kalman_input =
                test_data[i];


            /*
             * Pass the noisy measurement
             * through the Kalman filter.
             */
            debug_kalman_output =
                Kalman1D_Update(
                    &test_filter,
                    debug_kalman_input);


            /*
             * Slow down the test so the values
             * are easy to inspect.
             */
            Test_KalmanDelay();
        }
    }
}
