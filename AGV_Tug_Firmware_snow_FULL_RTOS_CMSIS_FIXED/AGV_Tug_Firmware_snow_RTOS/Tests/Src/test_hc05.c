#include "test_hc05.h"

#include <stdint.h>

#include "hc05.h"
#include "motor.h"


/*
 * Watch these in CubeIDE.
 */
volatile uint8_t debug_hc05_destination = 0U;

volatile uint32_t debug_hc05_command_count = 0U;


void Test_HC05(void)
{
    Motor_Stop();

    HC05_Init();

    HC05_SendString(
        "AGV HC05 READY\r\n");

    while (1)
    {
        uint8_t station;

        if (HC05_PollDestination(
                &station))
        {
            debug_hc05_destination =
                station;

            debug_hc05_command_count++;
        }
    }
}
