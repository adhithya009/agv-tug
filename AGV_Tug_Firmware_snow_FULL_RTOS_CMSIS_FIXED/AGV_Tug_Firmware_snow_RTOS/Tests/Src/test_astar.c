#include "test_astar.h"

#include <stdint.h>

#include "astar.h"
#include "map.h"
#include "motor.h"


/*
 * ==============================================================
 * DEBUG VARIABLES
 * ==============================================================
 *
 * Normal route:
 *
 * debug_normal_result
 * debug_normal_path
 *
 * Blocked-node route:
 *
 * debug_blocked_result
 * debug_blocked_path
 */

/*
 * Normal S1 -> S4 path.
 */
AStarPath debug_normal_path;

volatile uint8_t debug_normal_result = 0U;


/*
 * S1 -> S4 path while J5 is blocked.
 */
AStarPath debug_blocked_path;

volatile uint8_t debug_blocked_result = 0U;


/*
 * ==============================================================
 * A* TEST
 * ==============================================================
 */
void Test_AStar(void)
{
    /*
     * A* test must never move motors.
     */
    Motor_Stop();


    /*
     * Clear all temporary blocked-node states.
     */
    Map_Init();


    /*
     * ----------------------------------------------------------
     * TEST 1:
     * NORMAL ROUTE
     *
     * S1 -> S4
     * ----------------------------------------------------------
     */
    debug_normal_result =
        AStar_FindPath(
            MAP_S1,
            MAP_S4,
            &debug_normal_path);


    /*
     * ----------------------------------------------------------
     * TEST 2:
     * BLOCK J5
     *
     * A* should find another route that does
     * not contain MAP_J5.
     * ----------------------------------------------------------
     */
    Map_BlockNode(MAP_J5);


    debug_blocked_result =
        AStar_FindPath(
            MAP_S1,
            MAP_S4,
            &debug_blocked_path);


    /*
     * Restore map after test.
     */
    Map_UnblockNode(MAP_J5);


    /*
     * Stay here so debugger can inspect results.
     */
    while (1)
    {
        Motor_Stop();
    }
}
