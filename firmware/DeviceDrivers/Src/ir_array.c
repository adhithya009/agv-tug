#include "ir_array.h"

#include "stm32f4xx.h"


/* ==============================================================
 * READ FIVE LINE SENSORS
 * ============================================================== */

uint8_t IRArray_ReadPattern(void)
{
    uint8_t left;
    uint8_t left_center;
    uint8_t center;
    uint8_t right_center;
    uint8_t right;


    /*
     * A0 / PA0
     *
     * LEFTMOST.
     */
    left =
        (uint8_t)(
            (GPIOA->IDR >> 0U) &
            1U
        );


    /*
     * A1 / PA1
     *
     * LEFT-CENTER.
     */
    left_center =
        (uint8_t)(
            (GPIOA->IDR >> 1U) &
            1U
        );


    /*
     * A2 / PA4
     *
     * CENTER.
     */
    center =
        (uint8_t)(
            (GPIOA->IDR >> 4U) &
            1U
        );


    /*
     * A3 / PB0
     *
     * RIGHT-CENTER.
     */
    right_center =
        (uint8_t)(
            (GPIOB->IDR >> 0U) &
            1U
        );


    /*
     * A4 / PC1
     *
     * RIGHTMOST.
     */
    right =
        (uint8_t)(
            (GPIOC->IDR >> 1U) &
            1U
        );


    /*
     * Verified line sensor convention:
     *
     * WHITE -> 0
     * BLACK -> 1
     *
     * Therefore no inversion is required.
     *
     *
     * Returned pattern:
     *
     * bit4 bit3 bit2 bit1 bit0
     *
     *  R    RC    C    LC    L
     */
    return (uint8_t)(
        (left         << 0U) |
        (left_center  << 1U) |
        (center       << 2U) |
        (right_center << 3U) |
        (right        << 4U)
    );
}


/* ==============================================================
 * WEIGHTED LINE ERROR
 * ============================================================== */

int8_t IRArray_GetLineError(uint8_t pattern)
{
    int16_t weighted_sum =
        0;

    uint8_t active_count =
        0U;


    /*
     * Ignore anything outside the five
     * line-sensor bits.
     */
    pattern &=
        IR_PATTERN_MASK;


    /*
     * LEFTMOST.
     *
     * Weight = -4.
     */
    if ((pattern &
         IR_LEFT_BIT) != 0U)
    {
        weighted_sum +=
            -4;

        active_count++;
    }


    /*
     * LEFT-CENTER.
     *
     * Weight = -2.
     */
    if ((pattern &
         IR_LEFT_CENTER_BIT) != 0U)
    {
        weighted_sum +=
            -2;

        active_count++;
    }


    /*
     * CENTER.
     *
     * Weight = 0.
     */
    if ((pattern &
         IR_CENTER_BIT) != 0U)
    {
        active_count++;
    }


    /*
     * RIGHT-CENTER.
     *
     * Weight = +2.
     */
    if ((pattern &
         IR_RIGHT_CENTER_BIT) != 0U)
    {
        weighted_sum +=
            2;

        active_count++;
    }


    /*
     * RIGHTMOST.
     *
     * Weight = +4.
     */
    if ((pattern &
         IR_RIGHT_BIT) != 0U)
    {
        weighted_sum +=
            4;

        active_count++;
    }


    /*
     * 00000 = line completely lost.
     *
     * Navigation handles this separately.
     */
    if (active_count == 0U)
    {
        return 0;
    }


    /*
     * Average position of all active sensors.
     *
     * Examples:
     *
     * 00001 -> -4
     *
     * 00011:
     *
     * (-4 + -2) / 2 = -3
     *
     * 00110:
     *
     * (-2 + 0) / 2 = -1
     *
     * 01100:
     *
     * (0 + 2) / 2 = +1
     *
     * 11000:
     *
     * (2 + 4) / 2 = +3
     */
    return (int8_t)(
        weighted_sum /
        (int16_t)active_count
    );
}


/* ==============================================================
 * LINE DETECTED
 * ============================================================== */

uint8_t IRArray_LineDetected(uint8_t pattern)
{
    /*
     * 00000 is the only completely
     * lost-line state.
     */
    if ((pattern &
         IR_PATTERN_MASK) == 0U)
    {
        return 0U;
    }


    return 1U;
}


/* ==============================================================
 * JUNCTION CANDIDATE
 * ============================================================== */

uint8_t IRArray_IsJunctionCandidate(uint8_t pattern)
{
    uint8_t active_count =
        0U;


    pattern &=
        IR_PATTERN_MASK;


    /*
     * Count number of active sensors.
     */
    while (pattern != 0U)
    {
        active_count =
            (uint8_t)(
                active_count +
                (pattern & 1U)
            );


        pattern =
            (uint8_t)(
                pattern >> 1U
            );
    }


    /*
     * 4 or 5 active sensors means
     * a wide-black region.
     *
     * Examples:
     *
     * 11111
     * 11110
     * 01111
     */
    if (active_count >= 4U)
    {
        return 1U;
    }


    return 0U;
}


/* ==============================================================
 * OBSTACLE SENSOR
 *
 * A5 -> PC0
 * ============================================================== */

uint8_t IRArray_ReadObstacleRaw(void)
{
    return (uint8_t)(
        (GPIOC->IDR >> 0U) &
        1U
    );
}
