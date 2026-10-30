#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>


/*
 * Reset obstacle detection state.
 */
void Obstacle_Init(void);


/*
 * Read PC0 (A5) directly.
 *
 * Returns:
 * 0 or 1.
 */
uint8_t Obstacle_ReadRaw(void);


/*
 * Update debounce logic.
 *
 * Call periodically.
 */
void Obstacle_Update(void);


/*
 * Returns:
 *
 * 1 -> obstacle detected
 * 0 -> clear
 */
uint8_t Obstacle_IsDetected(void);

#endif
