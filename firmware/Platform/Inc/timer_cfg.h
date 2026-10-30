#ifndef TIMER_CFG_H
#define TIMER_CFG_H

#include <stdint.h>

/*
 * Initialize all timers:
 *
 * TIM1 -> Motor PWM
 * TIM3 -> Left LM393 pulse counter
 * TIM4 -> Right LM393 pulse counter
 * TIM5 -> 100 Hz control timer
 */
void TIMER_Init(void);

/* RTOS build: PWM/encoder timers only; FreeRTOS owns scheduling. */
void TIMER_InitForRTOS(void);


/*
 * Set motor PWM duty cycle.
 *
 * Range:
 * 0 to 100 percent.
 */
void TIMER_SetLeftPWM(uint8_t duty_percent);

void TIMER_SetRightPWM(uint8_t duty_percent);


/*
 * Returns 1 every 10 ms.
 *
 * Used as the control-loop timing source.
 */
uint8_t TIMER_ControlPeriodElapsed(void);


/*
 * Read raw hardware pulse counters.
 *
 * TIM3 -> Left encoder
 * TIM4 -> Right encoder
 */
uint16_t TIMER_GetLeftEncoderCount(void);

uint16_t TIMER_GetRightEncoderCount(void);


/*
 * Reset both LM393 hardware counters.
 */
void TIMER_ResetEncoderCounts(void);

#endif
