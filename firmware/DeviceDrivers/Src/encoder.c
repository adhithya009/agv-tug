#include "encoder.h"
#include "timer_cfg.h"


/* ==============================================================
 * CURRENT HARDWARE COUNTERS
 * ============================================================== */

static uint16_t left_raw = 0U;

static uint16_t right_raw = 0U;


/* ==============================================================
 * PREVIOUS HARDWARE COUNTERS
 * ============================================================== */

static uint16_t previous_left = 0U;

static uint16_t previous_right = 0U;


/* ==============================================================
 * LATEST PULSE DIFFERENCE
 * ============================================================== */

static int16_t left_delta = 0;

static int16_t right_delta = 0;


/* ==============================================================
 * INITIALIZATION
 * ============================================================== */

void Encoder_Init(void)
{
    Encoder_Reset();
}


/* ==============================================================
 * RESET
 * ============================================================== */

void Encoder_Reset(void)
{
    /*
     * Reset TIM3 and TIM4.
     */
    TIMER_ResetEncoderCounts();


    left_raw = 0U;
    right_raw = 0U;


    previous_left = 0U;
    previous_right = 0U;


    left_delta = 0;
    right_delta = 0;
}


/* ==============================================================
 * UPDATE
 * ============================================================== */

void Encoder_Update(void)
{
    /*
     * Read total hardware counts.
     */
    left_raw =
        TIMER_GetLeftEncoderCount();

    right_raw =
        TIMER_GetRightEncoderCount();


    /*
     * Unsigned subtraction handles
     * 16-bit timer wraparound.
     */
    uint16_t left_difference =
        (uint16_t)
        (left_raw - previous_left);

    uint16_t right_difference =
        (uint16_t)
        (right_raw - previous_right);


    /*
     * Convert latest pulse differences.
     */
    left_delta =
        (int16_t)left_difference;

    right_delta =
        (int16_t)right_difference;


    /*
     * Save current counts for next update.
     */
    previous_left =
        left_raw;

    previous_right =
        right_raw;
}


/* ==============================================================
 * GETTERS
 * ============================================================== */

uint16_t Encoder_GetLeftRaw(void)
{
    return left_raw;
}


uint16_t Encoder_GetRightRaw(void)
{
    return right_raw;
}


int16_t Encoder_GetLeftDelta(void)
{
    return left_delta;
}


int16_t Encoder_GetRightDelta(void)
{
    return right_delta;
}
