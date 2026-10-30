#include "test_encoder_one_revolution.h"

#include <stdint.h>
#include "stm32f4xx.h"
#include "motor.h"

volatile uint8_t debug_left_encoder_pin = 0U;
volatile uint8_t debug_right_encoder_pin = 0U;

volatile uint32_t debug_left_changes = 0U;
volatile uint32_t debug_right_changes = 0U;

volatile uint8_t debug_encoder_raw_test = 0U;

void Test_EncoderOneRevolution(void)
{
    Motor_Stop();

    /* PC6 and PB6 -> normal GPIO input */
    GPIOC->MODER &= ~(3U << (6U * 2U));
    GPIOB->MODER &= ~(3U << (6U * 2U));

    /* Enable pull-ups for LM393/open-collector style output. */
    GPIOC->PUPDR &= ~(3U << (6U * 2U));
    GPIOC->PUPDR |=  (1U << (6U * 2U));

    GPIOB->PUPDR &= ~(3U << (6U * 2U));
    GPIOB->PUPDR |=  (1U << (6U * 2U));

    uint8_t previous_left =
        (uint8_t)((GPIOC->IDR >> 6U) & 1U);

    uint8_t previous_right =
        (uint8_t)((GPIOB->IDR >> 6U) & 1U);

    debug_encoder_raw_test = 1U;

    while (1)
    {
        Motor_Stop();

        debug_left_encoder_pin =
            (uint8_t)((GPIOC->IDR >> 6U) & 1U);

        debug_right_encoder_pin =
            (uint8_t)((GPIOB->IDR >> 6U) & 1U);

        if (debug_left_encoder_pin != previous_left)
        {
            debug_left_changes++;
            previous_left = debug_left_encoder_pin;
        }

        if (debug_right_encoder_pin != previous_right)
        {
            debug_right_changes++;
            previous_right = debug_right_encoder_pin;
        }
    }
}
