#include "test_oled.h"

#include <stdint.h>

#include "oled.h"
#include "motor.h"


volatile uint8_t debug_oled_init_ok = 0U;


void Test_OLED(void)
{
    Motor_Stop();

    debug_oled_init_ok =
        OLED_Init();

    if (!debug_oled_init_ok)
    {
        while (1)
        {
            Motor_Stop();
        }
    }

    OLED_Clear();

    OLED_SetCursor(
        0U,
        0U);

    OLED_WriteString(
        "AGV READY");

    OLED_SetCursor(
        2U,
        0U);

    OLED_WriteString(
        "CURRENT:S1");

    OLED_SetCursor(
        4U,
        0U);

    OLED_WriteString(
        "DEST:S4");

    OLED_SetCursor(
        6U,
        0U);

    OLED_WriteString(
        "STATUS:OK");

    while (1)
    {
        Motor_Stop();
    }
}
