#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * Initialize the motor driver.
 *
 * TIMER_Init() and GPIO_Init() must already have been called.
 */
void Motor_Init(void);

/*
 * Set individual wheel speeds.
 *
 * Range:
 * -80 to +80
 *
 * Positive = forward
 * Negative = reverse
 * Zero     = stop
 */
void Motor_SetLeft(int8_t speed_percent);
void Motor_SetRight(int8_t speed_percent);

/*
 * Set both wheel speeds together.
 *s
 * This will become the main interface used by PID/navigation.
 */
void Motor_Set(int8_t left_percent,
               int8_t right_percent);

/* Immediately stop both motors. */
void Motor_Stop(void);

/* Convenience movement functions for hardware testing. */
void Motor_Forward(uint8_t speed_percent);
void Motor_Backward(uint8_t speed_percent);
void Motor_TurnLeft(uint8_t speed_percent);
void Motor_TurnRight(uint8_t speed_percent);

#endif
