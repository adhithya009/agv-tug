#include "obstacle_manager.h"

#include "motor.h"


/* ==============================================================
 * DYNAMIC HEADING-AWARE OBSTACLE REPLAN
 * ============================================================== */

uint8_t ObstacleManager_ReplanWithHeading(
    MapNodeId from_node,
    MapNodeId target_node,
    MapNodeId goal_node,
    AStarPath *detour_path)
{
    /* Safety first: stop motors immediately */
    Motor_Stop();

    if (detour_path == 0)
    {
        return 0U;
    }

    if (!Map_IsValidNode(from_node) ||
        !Map_IsValidNode(target_node) ||
        !Map_IsValidNode(goal_node))
    {
        return 0U;
    }

    /* Never block the node we are returning to */
    if (target_node == from_node)
    {
        return 0U;
    }

    /* Block the target node in the map */
    Map_BlockNode(target_node);

    /*
     * When returning to from_node, find the optimal alternate exit
     * from from_node to goal_node that does not go back into target_node.
     */
    AStarPath new_path;
    uint8_t found = 0U;

    MapNodeId neighbors[MAP_NODE_COUNT];
    uint8_t n_count = Map_GetNeighbors(from_node, neighbors, (uint8_t)MAP_NODE_COUNT);

    for (uint8_t i = 0U; i < n_count; i++)
    {
        MapNodeId nbr = neighbors[i];
        if ((nbr == target_node) || Map_IsBlocked(nbr))
        {
            continue;
        }

        MapDirection out_dir = Map_GetExitDirection(from_node, nbr);
        if (out_dir == MAP_DIR_INVALID)
        {
            continue;
        }

        AStarPath candidate;
        if (AStar_FindPathWithHeading(from_node, goal_node, out_dir, &candidate))
        {
            if (!found || (candidate.length < new_path.length))
            {
                new_path = candidate;
                found = 1U;
            }
        }
    }

    /* Fallback to unconstrained shortest-path A* */
    if (!found)
    {
        found = AStar_FindPath(from_node, goal_node, &new_path);
    }

    if (!found || (new_path.length < 2U))
    {
        return 0U;
    }

    /*
     * Build the combined detour path:
     *   nodes[0] = target_node (behind AGV on current edge)
     *   nodes[1] = from_node (first destination to reach after U-turn)
     *   nodes[2..length-1] = subsequent detour nodes from new_path
     */
    detour_path->nodes[0] = target_node;
    detour_path->nodes[1] = from_node;

    for (uint8_t i = 1U; i < new_path.length; i++)
    {
        if ((i + 1U) >= (uint8_t)MAP_NODE_COUNT)
        {
            return 0U;
        }
        detour_path->nodes[i + 1U] = new_path.nodes[i];
    }
    detour_path->length = (uint8_t)(new_path.length + 1U);

    return 1U;
}


/* ==============================================================
 * BASIC UNCONSTRAINED REPLAN
 * ============================================================== */

uint8_t ObstacleManager_Replan(
    MapNodeId current_node,
    MapNodeId blocked_node,
    MapNodeId goal_node,
    AStarPath *new_path)
{
    Motor_Stop();

    if (new_path == 0)
    {
        return 0U;
    }

    if (!Map_IsValidNode(current_node) ||
        !Map_IsValidNode(blocked_node) ||
        !Map_IsValidNode(goal_node))
    {
        return 0U;
    }

    if (blocked_node == current_node)
    {
        return 0U;
    }

    Map_BlockNode(blocked_node);

    return AStar_FindPath(
        current_node,
        goal_node,
        new_path);
}


/* ==============================================================
 * CLEAR BLOCKED NODE
 * ============================================================== */

void ObstacleManager_ClearNode(
    MapNodeId node)
{
    if (Map_IsValidNode(node))
    {
        Map_UnblockNode(node);
    }
}
