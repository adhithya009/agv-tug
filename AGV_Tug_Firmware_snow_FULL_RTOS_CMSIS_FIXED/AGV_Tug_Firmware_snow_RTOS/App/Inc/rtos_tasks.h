#ifndef RTOS_TASKS_H
#define RTOS_TASKS_H

#include <stdint.h>

/*
 * RTOS state machine for the integrated AGV HardwareTests build.
 * Values are intentionally stable so they are easy to watch in
 * STM32CubeIDE Live Expressions.
 */
typedef enum
{
    RTOS_AGV_CALIBRATING = 0,
    RTOS_AGV_WAIT_START_RFID,
    RTOS_AGV_RUNNING,
    RTOS_AGV_WAIT_DEST_RFID,
    RTOS_AGV_DESTINATION,
    RTOS_AGV_ASTAR_FAIL,
    RTOS_AGV_INDEX_ERROR,
    RTOS_AGV_TIMEOUT,
    RTOS_AGV_BLOCKED_NO_PATH
} RTOSAgvState;

/* Initialize the working AGV modules and RTOS synchronization objects. */
void RTOS_Tasks_Init(void);

/* Create the control, obstacle, RFID and display tasks. */
void RTOS_Tasks_Create(void);

/* Current integrated AGV state. */
RTOSAgvState RTOS_GetAgvState(void);

#endif
