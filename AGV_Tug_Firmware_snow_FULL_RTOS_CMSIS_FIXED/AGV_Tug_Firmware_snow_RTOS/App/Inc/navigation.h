#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>

#include "astar.h"


/* ==============================================================
 * NAVIGATION MODES
 * ============================================================== */

typedef enum
{
    NAV_MODE_FOLLOW_LINE = 0,
    NAV_MODE_TURN_LEFT,
    NAV_MODE_TURN_RIGHT,
    NAV_MODE_GO_STRAIGHT,
    NAV_MODE_UTURN,
    NAV_MODE_STOPPED

} NavigationMode;


/* ==============================================================
 * JUNCTION ACTION
 * ============================================================== */

typedef enum
{
    NAV_ACTION_STRAIGHT = 0,
    NAV_ACTION_LEFT,
    NAV_ACTION_RIGHT,
    NAV_ACTION_UTURN,
    NAV_ACTION_STOP

} NavigationAction;


/* ==============================================================
 * INITIALIZATION
 * ============================================================== */

void Navigation_Init(
    float kp,
    float ki,
    float kd,
    float dt,
    uint8_t base_speed);


/* ==============================================================
 * PATH
 * ============================================================== */

void Navigation_SetPath(
    const AStarPath *path);

/*
 * Start mid-edge U-turn to return along edge towards from_node
 * using detour_path (nodes[0] = target_node, nodes[1] = from_node, ...).
 * bend_pending_return: 1 if return track has internal bend pending, 0 if straight.
 */
void Navigation_StartMidEdgeUTurn(
    const AStarPath *detour_path,
    uint8_t bend_pending_return);


/* ==============================================================
 * UPDATE
 * ============================================================== */

void Navigation_Update(void);


void Navigation_UpdateWithSensors(
    uint8_t ir_pattern,
    float gyro_z_dps,
    uint8_t gyro_valid);


/* ==============================================================
 * CONTROL
 * ============================================================== */

void Navigation_Stop(void);


/* ==============================================================
 * STATUS
 * ============================================================== */

uint8_t Navigation_GetPathIndex(void);


float Navigation_GetTurnYawDeg(void);

/*
 * Returns 1 when the final path node has been reached
 * and navigation has stopped there.
 */
uint8_t Navigation_IsRouteComplete(void);

#endif
