#include "test_encoder.h"

#include <stdint.h>

/*
 * Required because this test directly accesses
 * GPIOB->IDR and GPIOC->IDR.
 */
#include "stm32f4xx.h"

#include "encoder.h"
#include "motor.h"
#include "timer_cfg.h"


/*
 * ==============================================================
 * DEBUG VARIABLES
 * ==============================================================
 */

/*
 * Total pulses counted since reset.
 */
volatile uint16_t debug_left_raw = 0U;

volatile uint16_t debug_right_raw = 0U;


/*
 * Number of pulses detected during the
 * latest 10 ms sampling interval.
 */
volatile int16_t debug_left_delta = 0;

volatile int16_t debug_right_delta = 0;


/*
 * Actual instantaneous digital outputs
 * from the two LM393 modules.
 *
 * Left DO  -> PC6
 * Right DO -> PB6
 */
volatile uint8_t debug_left_do = 0U;

volatile uint8_t debug_right_do = 0U;


/*
 * ==============================================================
 * ENCODER TEST
 * ==============================================================
 */
void Test_Encoder(void)
{
    /*
     * Motors must not move during this test.
     *
     * Wheels are rotated manually.
     */
    Motor_Stop();


    /*
     * Begin both encoder counters at zero.
     */
    Encoder_Reset();


    while (1)
    {
        /*
         * ------------------------------------------------------
         * LEFT LM393 DIGITAL OUTPUT
         * ------------------------------------------------------
         *
         * DO -> PC6.
         *
         * This lets us verify the sensor signal itself
         * independently of TIM3.
         */
        if ((GPIOC->IDR & (1UL << 6U)) != 0U)
        {
            debug_left_do = 1U;
        }
        else
        {
            debug_left_do = 0U;
        }


        /*
         * ------------------------------------------------------
         * RIGHT LM393 DIGITAL OUTPUT
         * ------------------------------------------------------
         *
         * DO -> PB6.
         */
        if ((GPIOB->IDR & (1UL << 6U)) != 0U)
        {
            debug_right_do = 1U;
        }
        else
        {
            debug_right_do = 0U;
        }


        /*
         * ------------------------------------------------------
         * ENCODER SAMPLING
         * ------------------------------------------------------
         *
         * TIM5 gives us one event every 10 ms.
         */
        if (TIMER_ControlPeriodElapsed())
        {
            /*
             * Read TIM3/TIM4 and calculate
             * pulses since previous sample.
             */
            Encoder_Update();


            /*
             * Copy results into volatile variables
             * so CubeIDE can display them.
             */
            debug_left_raw =
                Encoder_GetLeftRaw();

            debug_right_raw =
                Encoder_GetRightRaw();


            debug_left_delta =
                Encoder_GetLeftDelta();

            debug_right_delta =
                Encoder_GetRightDelta();
        }
    }
}
