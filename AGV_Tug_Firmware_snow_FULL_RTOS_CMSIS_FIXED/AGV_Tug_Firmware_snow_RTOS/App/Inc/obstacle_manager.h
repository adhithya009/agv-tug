#ifndef OBSTACLE_MANAGER_H
#define OBSTACLE_MANAGER_H

#include <stdint.h>

#include "astar.h"
#include "map.h"


/*
 * Stop AGV, mark blocked target_node and calculate alternate route
 * heading back towards from_node to reach goal_node.
 *
 * detour_path is populated with:
 *   nodes[0] = target_node (behind AGV on edge)
 *   nodes[1] = from_node (first destination to reach after U-turn)
 *   nodes[2..length-1] = subsequent nodes to goal_node
 *
 * Returns:
 * 1 -> alternate path found
 * 0 -> no valid alternate route
 */
uint8_t ObstacleManager_ReplanWithHeading(
    MapNodeId from_node,
    MapNodeId target_node,
    MapNodeId goal_node,
    AStarPath *detour_path);

/*
 * Stop AGV, mark blocked node and calculate
 * another route (unconstrained start heading).
 *
 * Returns:
 * 1 -> alternate path found
 * 0 -> no valid alternate route
 */
uint8_t ObstacleManager_Replan(
    MapNodeId current_node,
    MapNodeId blocked_node,
    MapNodeId goal_node,
    AStarPath *new_path);

/*
 * Remove temporary obstacle from map.
 */
void ObstacleManager_ClearNode(
    MapNodeId node);

#endif
