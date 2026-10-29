#include "astar.h"
#include "map.h"


#define ASTAR_INF              0xFFFFU
#define ASTAR_HEADING_COUNT    4U

#define ASTAR_STATE_COUNT \
    ((uint8_t)MAP_NODE_COUNT * ASTAR_HEADING_COUNT)

#define ASTAR_STATE_INVALID    0xFFU


/*
 * ==============================================================
 * ORIGINAL NODE-ONLY A*
 * ==============================================================
 */

static uint16_t node_g_cost[MAP_NODE_COUNT];

static uint16_t node_f_cost[MAP_NODE_COUNT];

static uint8_t node_open_set[MAP_NODE_COUNT];

static uint8_t node_closed_set[MAP_NODE_COUNT];

static MapNodeId node_parent[MAP_NODE_COUNT];


/*
 * ==============================================================
 * HEADING-AWARE A*
 * ==============================================================
 *
 * State is:
 *
 * (node, heading)
 *
 * Example:
 *
 * (J5, WEST)
 *
 * is different from:
 *
 * (J5, SOUTH)
 */

static uint16_t
heading_g_distance[ASTAR_STATE_COUNT];

static uint16_t
heading_f_distance[ASTAR_STATE_COUNT];

static uint16_t
heading_g_turns[ASTAR_STATE_COUNT];

static uint8_t
heading_open_set[ASTAR_STATE_COUNT];

static uint8_t
heading_closed_set[ASTAR_STATE_COUNT];

static uint8_t
heading_parent[ASTAR_STATE_COUNT];


/*
 * ==============================================================
 * HEURISTIC
 * ==============================================================
 */

static uint16_t Heuristic(
    MapNodeId a,
    MapNodeId b)
{
    MapPosition pa =
        Map_GetPosition(a);

    MapPosition pb =
        Map_GetPosition(b);


    int dr =
        (int)pb.row -
        (int)pa.row;

    int dc =
        (int)pb.col -
        (int)pa.col;


    if (dr < 0)
    {
        dr = -dr;
    }


    if (dc < 0)
    {
        dc = -dc;
    }


    return
        (uint16_t)(dr + dc);
}


/*
 * ==============================================================
 * OPPOSITE DIRECTION
 * ==============================================================
 */

static MapDirection OppositeDirection(
    MapDirection direction)
{
    if ((uint8_t)direction >=
        ASTAR_HEADING_COUNT)
    {
        return
            MAP_DIR_INVALID;
    }


    return
        (MapDirection)
        (((uint8_t)direction + 2U) %
         4U);
}


/*
 * ==============================================================
 * TURN COST
 * ==============================================================
 *
 * Used only when two routes have the
 * same physical distance.
 *
 * STRAIGHT = 0
 * LEFT/RIGHT = 1
 * U-TURN = 2
 */

static uint8_t TurnCost(
    MapDirection incoming,
    MapDirection outgoing)
{
    if (((uint8_t)incoming >=
         ASTAR_HEADING_COUNT) ||
        ((uint8_t)outgoing >=
         ASTAR_HEADING_COUNT))
    {
        return 255U;
    }


    if (incoming ==
        outgoing)
    {
        return 0U;
    }


    if ((((uint8_t)incoming + 2U) %
         4U) ==
        (uint8_t)outgoing)
    {
        return 2U;
    }


    return 1U;
}


/*
 * ==============================================================
 * RESET ORIGINAL A*
 * ==============================================================
 */

static void ResetNodeSearch(void)
{
    for (uint8_t i = 0U;
         i < (uint8_t)MAP_NODE_COUNT;
         i++)
    {
        node_g_cost[i] =
            ASTAR_INF;

        node_f_cost[i] =
            ASTAR_INF;

        node_open_set[i] =
            0U;

        node_closed_set[i] =
            0U;

        node_parent[i] =
            MAP_NODE_INVALID;
    }
}


/*
 * ==============================================================
 * BUILD ORIGINAL PATH
 * ==============================================================
 */

static uint8_t BuildNodePath(
    MapNodeId start,
    MapNodeId goal,
    AStarPath *path)
{
    MapNodeId reverse[
        MAP_NODE_COUNT];


    MapNodeId current =
        goal;


    uint8_t count =
        0U;


    while (current !=
           start)
    {
        if (count >=
            ((uint8_t)MAP_NODE_COUNT -
             1U))
        {
            return 0U;
        }


        reverse[count++] =
            current;


        MapNodeId previous =
            node_parent[
                (uint8_t)current];


        if (!Map_IsValidNode(
                previous))
        {
            return 0U;
        }


        current =
            previous;
    }


    reverse[count++] =
        start;


    path->length =
        count;


    for (uint8_t i = 0U;
         i < count;
         i++)
    {
        path->nodes[i] =
            reverse[
                count - 1U - i];
    }


    return 1U;
}


/*
 * ==============================================================
 * ORIGINAL DISTANCE-ONLY A*
 * ==============================================================
 */

uint8_t AStar_FindPath(
    MapNodeId start,
    MapNodeId goal,
    AStarPath *path)
{
    if (path == 0)
    {
        return 0U;
    }


    path->length =
        0U;


    if (!Map_IsValidNode(start) ||
        !Map_IsValidNode(goal))
    {
        return 0U;
    }


    if (Map_IsBlocked(start) ||
        Map_IsBlocked(goal))
    {
        return 0U;
    }


    if (start ==
        goal)
    {
        path->nodes[0] =
            start;

        path->length =
            1U;

        return 1U;
    }


    ResetNodeSearch();


    node_g_cost[
        (uint8_t)start] =
        0U;


    node_f_cost[
        (uint8_t)start] =
        Heuristic(
            start,
            goal);


    node_open_set[
        (uint8_t)start] =
        1U;


    while (1)
    {
        MapNodeId current =
            MAP_NODE_INVALID;


        uint16_t best_f =
            ASTAR_INF;


        for (uint8_t i = 0U;
             i < (uint8_t)MAP_NODE_COUNT;
             i++)
        {
            if (!node_open_set[i])
            {
                continue;
            }


            if (node_f_cost[i] <
                best_f)
            {
                best_f =
                    node_f_cost[i];

                current =
                    (MapNodeId)i;
            }
        }


        if (current ==
            MAP_NODE_INVALID)
        {
            return 0U;
        }


        if (current ==
            goal)
        {
            return
                BuildNodePath(
                    start,
                    goal,
                    path);
        }


        node_open_set[
            (uint8_t)current] =
            0U;


        node_closed_set[
            (uint8_t)current] =
            1U;


        MapNodeId neighbors[
            MAP_NODE_COUNT];


        uint8_t neighbor_count =
            Map_GetNeighbors(
                current,
                neighbors,
                MAP_NODE_COUNT);


        for (uint8_t i = 0U;
             i < neighbor_count;
             i++)
        {
            MapNodeId neighbor =
                neighbors[i];


            if (node_closed_set[
                    (uint8_t)neighbor])
            {
                continue;
            }


            uint8_t edge_cost =
                Map_GetEdgeCost(
                    current,
                    neighbor);


            if (edge_cost ==
                255U)
            {
                continue;
            }


            uint16_t new_g =
                node_g_cost[
                    (uint8_t)current] +
                (uint16_t)edge_cost;


            if (new_g <
                node_g_cost[
                    (uint8_t)neighbor])
            {
                node_g_cost[
                    (uint8_t)neighbor] =
                    new_g;


                node_f_cost[
                    (uint8_t)neighbor] =
                    new_g +
                    Heuristic(
                        neighbor,
                        goal);


                node_parent[
                    (uint8_t)neighbor] =
                    current;


                node_open_set[
                    (uint8_t)neighbor] =
                    1U;
            }
        }
    }
}


/*
 * ==============================================================
 * HEADING STATE HELPERS
 * ==============================================================
 */

static uint8_t StateIndex(
    MapNodeId node,
    MapDirection heading)
{
    return
        (uint8_t)
        (((uint8_t)node *
          ASTAR_HEADING_COUNT) +
         (uint8_t)heading);
}


static MapNodeId StateNode(
    uint8_t state)
{
    return
        (MapNodeId)
        (state /
         ASTAR_HEADING_COUNT);
}


static MapDirection StateHeading(
    uint8_t state)
{
    return
        (MapDirection)
        (state %
         ASTAR_HEADING_COUNT);
}


/*
 * ==============================================================
 * RESET HEADING-AWARE SEARCH
 * ==============================================================
 */

static void ResetHeadingSearch(void)
{
    for (uint8_t i = 0U;
         i < ASTAR_STATE_COUNT;
         i++)
    {
        heading_g_distance[i] =
            ASTAR_INF;

        heading_f_distance[i] =
            ASTAR_INF;

        heading_g_turns[i] =
            ASTAR_INF;

        heading_open_set[i] =
            0U;

        heading_closed_set[i] =
            0U;

        heading_parent[i] =
            ASTAR_STATE_INVALID;
    }
}


/*
 * ==============================================================
 * BUILD HEADING-AWARE PATH
 * ==============================================================
 */

static uint8_t BuildHeadingPath(
    uint8_t start_state,
    uint8_t goal_state,
    AStarPath *path)
{
    MapNodeId reverse[
        MAP_NODE_COUNT];


    uint8_t current_state =
        goal_state;


    uint8_t count =
        0U;


    while (current_state !=
           start_state)
    {
        if ((current_state ==
             ASTAR_STATE_INVALID) ||
            (count >=
             ((uint8_t)MAP_NODE_COUNT -
              1U)))
        {
            return 0U;
        }


        reverse[count++] =
            StateNode(
                current_state);


        current_state =
            heading_parent[
                current_state];
    }


    reverse[count++] =
        StateNode(
            start_state);


    path->length =
        count;


    for (uint8_t i = 0U;
         i < count;
         i++)
    {
        path->nodes[i] =
            reverse[
                count - 1U - i];
    }


    return 1U;
}


/*
 * ==============================================================
 * HEADING-AWARE A*
 * ==============================================================
 */

uint8_t AStar_FindPathWithHeading(
    MapNodeId start,
    MapNodeId goal,
    MapDirection start_heading,
    AStarPath *path)
{
    if (path == 0)
    {
        return 0U;
    }


    path->length =
        0U;


    if (!Map_IsValidNode(start) ||
        !Map_IsValidNode(goal))
    {
        return 0U;
    }


    if ((uint8_t)start_heading >=
        ASTAR_HEADING_COUNT)
    {
        return 0U;
    }


    if (Map_IsBlocked(start) ||
        Map_IsBlocked(goal))
    {
        return 0U;
    }


    if (start ==
        goal)
    {
        path->nodes[0] =
            start;

        path->length =
            1U;

        return 1U;
    }


    ResetHeadingSearch();


    uint8_t start_state =
        StateIndex(
            start,
            start_heading);


    heading_g_distance[
        start_state] =
        0U;


    heading_g_turns[
        start_state] =
        0U;


    heading_f_distance[
        start_state] =
        Heuristic(
            start,
            goal);


    heading_open_set[
        start_state] =
        1U;


    while (1)
    {
        uint8_t current_state =
            ASTAR_STATE_INVALID;


        uint16_t best_f_distance =
            ASTAR_INF;


        uint16_t best_turns =
            ASTAR_INF;


        /*
         * First choose minimum physical
         * distance.
         *
         * If distance ties, choose the
         * state with fewer turns.
         */
        for (uint8_t i = 0U;
             i < ASTAR_STATE_COUNT;
             i++)
        {
            if (!heading_open_set[i])
            {
                continue;
            }


            if ((heading_f_distance[i] <
                 best_f_distance) ||

                ((heading_f_distance[i] ==
                  best_f_distance) &&

                 (heading_g_turns[i] <
                  best_turns)))
            {
                current_state =
                    i;


                best_f_distance =
                    heading_f_distance[i];


                best_turns =
                    heading_g_turns[i];
            }
        }


        if (current_state ==
            ASTAR_STATE_INVALID)
        {
            return 0U;
        }


        MapNodeId current_node =
            StateNode(
                current_state);


        MapDirection current_heading =
            StateHeading(
                current_state);


        /*
         * Since goal heuristic is zero,
         * the first goal state removed
         * from the open set is our best
         * distance/turn combination.
         */
        if (current_node ==
            goal)
        {
            return
                BuildHeadingPath(
                    start_state,
                    current_state,
                    path);
        }


        heading_open_set[
            current_state] =
            0U;


        heading_closed_set[
            current_state] =
            1U;


        MapNodeId neighbors[
            MAP_NODE_COUNT];


        uint8_t neighbor_count =
            Map_GetNeighbors(
                current_node,
                neighbors,
                MAP_NODE_COUNT);


        for (uint8_t i = 0U;
             i < neighbor_count;
             i++)
        {
            MapNodeId neighbor =
                neighbors[i];


            uint8_t edge_cost =
                Map_GetEdgeCost(
                    current_node,
                    neighbor);


            if (edge_cost ==
                255U)
            {
                continue;
            }


            /*
             * Direction in which the path
             * leaves the current node.
             */
            MapDirection outgoing =
                Map_GetExitDirection(
                    current_node,
                    neighbor);


            /*
             * Direction from neighbor back
             * toward current.
             */
            MapDirection neighbor_back =
                Map_GetExitDirection(
                    neighbor,
                    current_node);


            if (((uint8_t)outgoing >=
                 ASTAR_HEADING_COUNT) ||

                ((uint8_t)neighbor_back >=
                 ASTAR_HEADING_COUNT))
            {
                continue;
            }


            /*
             * CRITICAL:
             *
             * Navigation does not turn at
             * the starting node.
             *
             * Therefore the first graph edge
             * MUST already match how the
             * robot is physically facing.
             */
            if ((current_state ==
                 start_state) &&

                (outgoing !=
                 start_heading))
            {
                continue;
            }


            /*
             * Heading when the robot arrives
             * at the neighbour.
             *
             * Example:
             *
             * neighbour -> current = EAST
             *
             * means:
             *
             * current -> neighbour arrives WEST.
             */
            MapDirection arrival_heading =
                OppositeDirection(
                    neighbor_back);


            if (arrival_heading ==
                MAP_DIR_INVALID)
            {
                continue;
            }


            uint8_t turn_cost =
                TurnCost(
                    current_heading,
                    outgoing);


            if (turn_cost ==
                255U)
            {
                continue;
            }


            uint8_t next_state =
                StateIndex(
                    neighbor,
                    arrival_heading);


            if (heading_closed_set[
                    next_state])
            {
                continue;
            }


            uint16_t new_distance =
                heading_g_distance[
                    current_state] +
                (uint16_t)edge_cost;


            uint16_t new_turns =
                heading_g_turns[
                    current_state] +
                (uint16_t)turn_cost;


            uint8_t better =
                0U;


            if (new_distance <
                heading_g_distance[
                    next_state])
            {
                better =
                    1U;
            }
            else if (
                (new_distance ==
                 heading_g_distance[
                     next_state]) &&

                (new_turns <
                 heading_g_turns[
                     next_state]))
            {
                better =
                    1U;
            }


            if (!better)
            {
                continue;
            }


            heading_g_distance[
                next_state] =
                new_distance;


            heading_g_turns[
                next_state] =
                new_turns;


            heading_f_distance[
                next_state] =
                new_distance +
                Heuristic(
                    neighbor,
                    goal);


            heading_parent[
                next_state] =
                current_state;


            heading_open_set[
                next_state] =
                1U;
        }
    }
}
