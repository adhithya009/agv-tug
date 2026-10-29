#include "test_route_navigation.h"

#include <stdint.h>

#include "map.h"
#include "astar.h"
#include "navigation.h"
#include "ir_array.h"
#include "motor.h"
#include "timer_cfg.h"


/*
 * ==============================================================
 * ROUTE TEST SETTINGS
 * ==============================================================
 *
 * Keep the exact line follower tuning that is currently working.
 */

#define ROUTE_KP                    3.0f
#define ROUTE_KI                    0.0f
#define ROUTE_KD                    0.0f

#define ROUTE_DT_S                  0.010f

/*
 * Current working cruise PWM.
 */
#define ROUTE_BASE_SPEED            68U

/*
 * Keep robot stopped for 2 seconds before starting.
 */
#define ROUTE_START_DELAY_TICKS     200U

/*
 * 60 second safety timeout.
 */
#define ROUTE_TIMEOUT_TICKS         6000U

/*
 * Require 3 consecutive 111 samples at final station.
 */
#define ROUTE_DEST_CONFIRM_TICKS    3U


/*
 * ==============================================================
 * ROUTE TEST STATES
 * ==============================================================
 */

#define ROUTE_STATE_STARTING        0U
#define ROUTE_STATE_RUNNING         1U
#define ROUTE_STATE_DESTINATION     2U
#define ROUTE_STATE_ASTAR_FAIL      3U
#define ROUTE_STATE_TIMEOUT         4U
#define ROUTE_STATE_INDEX_ERROR     5U


/*
 * ==============================================================
 * GENERAL DEBUG
 * ==============================================================
 */

volatile uint8_t debug_route_state = 0U;

volatile uint8_t debug_route_done = 0U;

volatile uint8_t debug_route_astar_ok = 0U;

volatile uint8_t debug_route_expected_path_ok = 0U;


/*
 * ==============================================================
 * COMPLETE A* PATH
 * ==============================================================
 */

volatile uint8_t debug_route_path_length = 0U;

volatile uint8_t debug_route_path[MAP_NODE_COUNT];


/*
 * ==============================================================
 * LIVE ROUTE POSITION
 * ==============================================================
 */

volatile uint8_t debug_route_path_index = 0U;

volatile uint8_t debug_route_previous_node =
    (uint8_t)MAP_NODE_INVALID;

volatile uint8_t debug_route_current_node =
    (uint8_t)MAP_NODE_INVALID;

volatile uint8_t debug_route_next_node =
    (uint8_t)MAP_NODE_INVALID;


/*
 * ==============================================================
 * EXPECTED JUNCTION ACTION
 * ==============================================================
 *
 * NavigationAction:
 *
 * 0 = STRAIGHT
 * 1 = LEFT
 * 2 = RIGHT
 * 3 = U-TURN
 * 4 = STOP
 */

volatile uint8_t debug_route_expected_action =
    (uint8_t)NAV_ACTION_STOP;


/*
 * ==============================================================
 * IR DEBUG
 * ==============================================================
 */

volatile uint8_t debug_route_ir_pattern = 0U;


/*
 * ==============================================================
 * TIME DEBUG
 * ==============================================================
 */

volatile uint32_t debug_route_ticks = 0U;

volatile uint32_t debug_route_elapsed_ms = 0U;

volatile uint32_t debug_route_start_delay_ms = 0U;

volatile uint8_t debug_route_destination_confirm = 0U;


/*
 * ==============================================================
 * PATH-INDEX ERROR DEBUG
 * ==============================================================
 */

volatile uint8_t debug_route_index_jump = 0U;

volatile uint8_t debug_route_index_change_count = 0U;


/*
 * ==============================================================
 * ROUTE TRANSITION LOG
 * ==============================================================
 *
 * Every time Navigation_GetPathIndex() changes,
 * record exactly what happened.
 *
 * Example:
 *
 * old = 1
 * new = 2
 *
 * means:
 *
 * J2 was completed and robot is now approaching J1.
 */

volatile uint8_t
debug_route_transition_old_index[MAP_NODE_COUNT];

volatile uint8_t
debug_route_transition_new_index[MAP_NODE_COUNT];

volatile uint8_t
debug_route_transition_ir[MAP_NODE_COUNT];

volatile uint8_t
debug_route_transition_target[MAP_NODE_COUNT];

volatile uint32_t
debug_route_transition_tick[MAP_NODE_COUNT];


/*
 * ==============================================================
 * PRIVATE A* PATH
 * ==============================================================
 */

static AStarPath route_path;


/*
 * ==============================================================
 * OPPOSITE MAP DIRECTION
 * ==============================================================
 */

static MapDirection RouteDebug_Opposite(
    MapDirection direction)
{
    if (direction == MAP_DIR_INVALID)
    {
        return MAP_DIR_INVALID;
    }

    return (MapDirection)
        (((uint8_t)direction + 2U) % 4U);
}


/*
 * ==============================================================
 * CALCULATE EXPECTED ACTION
 * ==============================================================
 *
 * This is diagnostic only.
 *
 * It does NOT control the motors.
 */

static NavigationAction RouteDebug_GetAction(
    MapNodeId previous,
    MapNodeId current,
    MapNodeId next)
{
    MapDirection back =
        Map_GetExitDirection(
            current,
            previous);

    MapDirection incoming =
        RouteDebug_Opposite(
            back);

    MapDirection outgoing =
        Map_GetExitDirection(
            current,
            next);

    if ((incoming == MAP_DIR_INVALID) ||
        (outgoing == MAP_DIR_INVALID))
    {
        return NAV_ACTION_STOP;
    }

    if (incoming == outgoing)
    {
        return NAV_ACTION_STRAIGHT;
    }

    if ((((uint8_t)incoming + 2U) % 4U) ==
        (uint8_t)outgoing)
    {
        return NAV_ACTION_UTURN;
    }

    if ((((uint8_t)incoming + 1U) % 4U) ==
        (uint8_t)outgoing)
    {
        return NAV_ACTION_RIGHT;
    }

    return NAV_ACTION_LEFT;
}


/*
 * ==============================================================
 * CLEAR DEBUG ARRAYS
 * ==============================================================
 */

static void RouteDebug_ClearArrays(void)
{
    for (uint8_t i = 0U;
         i < (uint8_t)MAP_NODE_COUNT;
         i++)
    {
        debug_route_path[i] =
            (uint8_t)MAP_NODE_INVALID;

        debug_route_transition_old_index[i] =
            0xFFU;

        debug_route_transition_new_index[i] =
            0xFFU;

        debug_route_transition_ir[i] =
            0xFFU;

        debug_route_transition_target[i] =
            (uint8_t)MAP_NODE_INVALID;

        debug_route_transition_tick[i] =
            0U;
    }
}


/*
 * ==============================================================
 * COPY A* PATH
 * ==============================================================
 */

static void RouteDebug_CopyPath(void)
{
    debug_route_path_length =
        route_path.length;

    for (uint8_t i = 0U;
         i < route_path.length;
         i++)
    {
        debug_route_path[i] =
            (uint8_t)route_path.nodes[i];
    }
}


/*
 * ==============================================================
 * CHECK EXPECTED S2 -> S3 PATH
 * ==============================================================
 *
 * Expected:
 *
 * S2 -> J2 -> J1 -> S5 -> J8 -> S3
 *
 * Numeric:
 *
 * 1 -> 7 -> 6 -> 4 -> 13 -> 2
 */

static uint8_t RouteDebug_CheckExpectedPath(void)
{
    if (route_path.length != 6U)
    {
        return 0U;
    }

    if (route_path.nodes[0] != MAP_S2)
    {
        return 0U;
    }

    if (route_path.nodes[1] != MAP_J2)
    {
        return 0U;
    }

    if (route_path.nodes[2] != MAP_J1)
    {
        return 0U;
    }

    if (route_path.nodes[3] != MAP_S5)
    {
        return 0U;
    }

    if (route_path.nodes[4] != MAP_J8)
    {
        return 0U;
    }

    if (route_path.nodes[5] != MAP_S3)
    {
        return 0U;
    }

    return 1U;
}


/*
 * ==============================================================
 * UPDATE LIVE ROUTE CONTEXT
 * ==============================================================
 */

static void RouteDebug_UpdateContext(void)
{
    uint8_t index =
        Navigation_GetPathIndex();

    debug_route_path_index =
        index;

    debug_route_previous_node =
        (uint8_t)MAP_NODE_INVALID;

    debug_route_current_node =
        (uint8_t)MAP_NODE_INVALID;

    debug_route_next_node =
        (uint8_t)MAP_NODE_INVALID;

    debug_route_expected_action =
        (uint8_t)NAV_ACTION_STOP;


    /*
     * Invalid route index.
     */
    if (index >= route_path.length)
    {
        return;
    }


    /*
     * Node robot is currently approaching.
     */
    debug_route_current_node =
        (uint8_t)route_path.nodes[index];


    /*
     * Previous graph node.
     */
    if (index > 0U)
    {
        debug_route_previous_node =
            (uint8_t)route_path.nodes[index - 1U];
    }


    /*
     * Next graph node.
     */
    if ((index + 1U) < route_path.length)
    {
        debug_route_next_node =
            (uint8_t)route_path.nodes[index + 1U];


        /*
         * Calculate what action SHOULD happen
         * when current node is reached.
         */
        debug_route_expected_action =
            (uint8_t)RouteDebug_GetAction(
                route_path.nodes[index - 1U],
                route_path.nodes[index],
                route_path.nodes[index + 1U]);
    }
}


/*
 * ==============================================================
 * RECORD PATH INDEX CHANGE
 * ==============================================================
 */

static void RouteDebug_LogTransition(
    uint8_t old_index,
    uint8_t new_index,
    uint8_t ir_pattern)
{
    uint8_t log_index =
        debug_route_index_change_count;


    /*
     * Detect impossible jump:
     *
     * 1 -> 3
     *
     * instead of:
     *
     * 1 -> 2.
     */
    if (new_index > (uint8_t)(old_index + 1U))
    {
        debug_route_index_jump =
            1U;
    }


    /*
     * Keep log inside available array.
     */
    if (log_index >=
        (uint8_t)MAP_NODE_COUNT)
    {
        return;
    }


    debug_route_transition_old_index[log_index] =
        old_index;

    debug_route_transition_new_index[log_index] =
        new_index;

    debug_route_transition_ir[log_index] =
        ir_pattern;


    /*
     * Record the node software believed
     * it had just completed.
     */
    if (old_index < route_path.length)
    {
        debug_route_transition_target[log_index] =
            (uint8_t)route_path.nodes[old_index];
    }


    debug_route_transition_tick[log_index] =
        debug_route_ticks;


    debug_route_index_change_count++;
}


/*
 * ==============================================================
 * STOP TEST
 * ==============================================================
 */

static void RouteDebug_Stop(
    uint8_t state)
{
    Navigation_Stop();

    Motor_Stop();

    debug_route_state =
        state;

    debug_route_done =
        1U;
}


/*
 * ==============================================================
 * ROUTE NAVIGATION TEST
 * ==============================================================
 */

void Test_RouteNavigation(void)
{
    /*
     * ----------------------------------------------------------
     * SAFE START
     * ----------------------------------------------------------
     */

    Motor_Stop();


    /*
     * Reset dynamic map state.
     */
    Map_Init();


    /*
     * Use the same working line-follow configuration.
     */
    Navigation_Init(
        ROUTE_KP,
        ROUTE_KI,
        ROUTE_KD,
        ROUTE_DT_S,
        ROUTE_BASE_SPEED);


    /*
     * ----------------------------------------------------------
     * RESET DEBUG DATA
     * ----------------------------------------------------------
     */

    debug_route_state =
        ROUTE_STATE_STARTING;

    debug_route_done =
        0U;

    debug_route_astar_ok =
        0U;

    debug_route_expected_path_ok =
        0U;

    debug_route_path_length =
        0U;

    debug_route_path_index =
        0U;

    debug_route_ir_pattern =
        0U;

    debug_route_ticks =
        0U;

    debug_route_elapsed_ms =
        0U;

    debug_route_start_delay_ms =
        0U;

    debug_route_destination_confirm =
        0U;

    debug_route_index_jump =
        0U;

    debug_route_index_change_count =
        0U;

    RouteDebug_ClearArrays();


    /*
     * ----------------------------------------------------------
     * RUN A*
     *
     * START = S2
     * GOAL  = S3
     * ----------------------------------------------------------
     */

    debug_route_astar_ok =
        AStar_FindPath(
            MAP_S2,
            MAP_S3,
            &route_path);


    /*
     * A* failure means physical navigation
     * cannot begin.
     */
    if (!debug_route_astar_ok)
    {
        RouteDebug_Stop(
            ROUTE_STATE_ASTAR_FAIL);

        while (1)
        {
            Motor_Stop();
        }
    }


    /*
     * Copy route so CubeIDE can display it.
     */
    RouteDebug_CopyPath();


    /*
     * Verify which equal-cost route A* selected.
     */
    debug_route_expected_path_ok =
        RouteDebug_CheckExpectedPath();


    /*
     * ----------------------------------------------------------
     * TWO SECOND START DELAY
     * ----------------------------------------------------------
     */

    uint32_t start_ticks =
        0U;


    while (start_ticks <
           ROUTE_START_DELAY_TICKS)
    {
        Motor_Stop();


        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }


        start_ticks++;


        debug_route_start_delay_ms =
            start_ticks * 10U;
    }


    /*
     * ----------------------------------------------------------
     * START PHYSICAL ROUTE
     * ----------------------------------------------------------
     */

    Navigation_SetPath(
        &route_path);


    debug_route_state =
        ROUTE_STATE_RUNNING;


    /*
     * Initial expected target should be J2.
     */
    RouteDebug_UpdateContext();


    /*
     * ----------------------------------------------------------
     * MAIN 10 ms LOOP
     * ----------------------------------------------------------
     */

    while (debug_route_state ==
           ROUTE_STATE_RUNNING)
    {
        if (!TIMER_ControlPeriodElapsed())
        {
            continue;
        }


        /*
         * Time bookkeeping.
         */
        debug_route_ticks++;

        debug_route_elapsed_ms =
            debug_route_ticks * 10U;


        /*
         * Read current IR pattern before navigation update.
         *
         * Navigation_Update() will independently read the
         * same physical sensors immediately afterwards.
         */
        debug_route_ir_pattern =
            IRArray_ReadPattern();


        /*
         * Record route position BEFORE navigation executes.
         */
        uint8_t old_index =
            Navigation_GetPathIndex();


        /*
         * Real production line follower + route controller.
         */
        Navigation_Update();


        /*
         * Record route position AFTER navigation executes.
         */
        uint8_t new_index =
            Navigation_GetPathIndex();


        /*
         * If route position changed, record the event.
         */
        if (new_index != old_index)
        {
            RouteDebug_LogTransition(
                old_index,
                new_index,
                debug_route_ir_pattern);
        }


        /*
         * Protect against corrupted path progress.
         */
        if (new_index >= route_path.length)
        {
            RouteDebug_Stop(
                ROUTE_STATE_INDEX_ERROR);

            break;
        }


        /*
         * Update previous/current/next node display.
         */
        RouteDebug_UpdateContext();


        /*
         * ------------------------------------------------------
         * TEMPORARY S3 DESTINATION CONFIRMATION
         * ------------------------------------------------------
         *
         * When software is approaching the final path node,
         * 111 should represent the S3 station marker.
         */

        if (new_index ==
            (uint8_t)(route_path.length - 1U))
        {
            if (debug_route_ir_pattern ==
                0x07U)
            {
                if (debug_route_destination_confirm <
                    ROUTE_DEST_CONFIRM_TICKS)
                {
                    debug_route_destination_confirm++;
                }
            }
            else
            {
                debug_route_destination_confirm =
                    0U;
            }


            if (debug_route_destination_confirm >=
                ROUTE_DEST_CONFIRM_TICKS)
            {
                RouteDebug_Stop(
                    ROUTE_STATE_DESTINATION);

                break;
            }
        }
        else
        {
            debug_route_destination_confirm =
                0U;
        }


        /*
         * ------------------------------------------------------
         * SAFETY TIMEOUT
         * ------------------------------------------------------
         */

        if (debug_route_ticks >=
            ROUTE_TIMEOUT_TICKS)
        {
            RouteDebug_Stop(
                ROUTE_STATE_TIMEOUT);

            break;
        }
    }


    /*
     * Final safe state.
     */
    while (1)
    {
        Motor_Stop();
    }
}
