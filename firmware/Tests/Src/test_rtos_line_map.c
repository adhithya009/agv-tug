#include "test_rtos_line_map.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "rtos_tasks.h"

/*
 * HardwareTests entry point for the fully integrated RTOS build.
 * Board-level register initialization stays in test_main.c.
 */
volatile uint8_t debug_rtos_test_stage = 0U;

void Test_RTOSLineMap(void)
{
    Motor_Stop();
    debug_rtos_test_stage = 1U;

    RTOS_Tasks_Init();
    debug_rtos_test_stage = 2U;

    RTOS_Tasks_Create();
    debug_rtos_test_stage = 3U;

    vTaskStartScheduler();

    /* A correctly started scheduler never returns here. */
    Motor_Stop();
    debug_rtos_test_stage = 0xFFU;

    for (;;)
    {
    }
}
