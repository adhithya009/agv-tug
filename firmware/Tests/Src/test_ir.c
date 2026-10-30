#include "test_ir.h"

#include <stdint.h>

#include "stm32f4xx.h"

#include "ir_array.h"
#include "timer_cfg.h"
#include "motor.h"


/* ==============================================================
 * RAW SENSOR DEBUG VARIABLES
 * ============================================================== */

/*
 * Physical order:
 *
 * A0 / PA0 -> LEFT
 *
 * A1 / PA1 -> LEFT-CENTER
 *
 * A2 / PA4 -> CENTER
 *
 * A3 / PB0 -> RIGHT-CENTER
 *
 * A4 / PC1 -> RIGHT
 *
 * A5 / PC0 -> OBSTACLE
 */

volatile uint8_t
debug_pa0_left_raw =
    0U;


volatile uint8_t
debug_pa1_left_center_raw =
    0U;


volatile uint8_t
debug_pa4_center_raw =
    0U;


volatile uint8_t
debug_pb0_right_center_raw =
    0U;


volatile uint8_t
debug_pc1_right_raw =
    0U;


volatile uint8_t
debug_obstacle_raw =
    0U;


/*
 * Five-bit pattern built directly
 * from GPIO values.
 */
volatile uint8_t
debug_gpio_pattern_raw =
    0U;


/* ==============================================================
 * PROCESSED DRIVER DEBUG VARIABLES
 * ============================================================== */

volatile uint8_t
debug_ir_pattern =
    0U;


volatile int8_t
debug_line_error =
    0;


volatile uint8_t
debug_line_detected =
    0U;


volatile uint8_t
debug_junction =
    0U;


/* ==============================================================
 * IR SENSOR TEST
 * ============================================================== */

void Test_IR(void)
{
    /*
     * This is a sensor-only test.
     *
     * Motors must never move.
     */
    Motor_Stop();


    while (1)
    {
        /*
         * A0 / PA0
         *
         * LEFTMOST sensor.
         */
        debug_pa0_left_raw =
            (uint8_t)(
                (GPIOA->IDR >> 0U) &
                1U
            );


        /*
         * A1 / PA1
         *
         * LEFT-CENTER sensor.
         */
        debug_pa1_left_center_raw =
            (uint8_t)(
                (GPIOA->IDR >> 1U) &
                1U
            );


        /*
         * A2 / PA4
         *
         * CENTER sensor.
         */
        debug_pa4_center_raw =
            (uint8_t)(
                (GPIOA->IDR >> 4U) &
                1U
            );


        /*
         * A3 / PB0
         *
         * RIGHT-CENTER sensor.
         */
        debug_pb0_right_center_raw =
            (uint8_t)(
                (GPIOB->IDR >> 0U) &
                1U
            );


        /*
         * A4 / PC1
         *
         * RIGHTMOST sensor.
         */
        debug_pc1_right_raw =
            (uint8_t)(
                (GPIOC->IDR >> 1U) &
                1U
            );


        /*
         * A5 / PC0
         *
         * Obstacle sensor.
         */
        debug_obstacle_raw =
            (uint8_t)(
                (GPIOC->IDR >> 0U) &
                1U
            );


        /*
         * Build raw five-bit pattern.
         *
         * bit4 bit3 bit2 bit1 bit0
         *
         *  R    RC    C    LC    L
         */
        debug_gpio_pattern_raw =
            (uint8_t)(
                (debug_pa0_left_raw
                    << 0U) |

                (debug_pa1_left_center_raw
                    << 1U) |

                (debug_pa4_center_raw
                    << 2U) |

                (debug_pb0_right_center_raw
                    << 3U) |

                (debug_pc1_right_raw
                    << 4U)
            );


        /*
         * Process through the real driver
         * every 10 ms control period.
         */
        if (TIMER_ControlPeriodElapsed())
        {
            debug_ir_pattern =
                IRArray_ReadPattern();


            debug_line_error =
                IRArray_GetLineError(
                    debug_ir_pattern
                );


            debug_line_detected =
                IRArray_LineDetected(
                    debug_ir_pattern
                );


            debug_junction =
                IRArray_IsJunctionCandidate(
                    debug_ir_pattern
                );


            debug_obstacle_raw =
                IRArray_ReadObstacleRaw();
        }
    }
}
