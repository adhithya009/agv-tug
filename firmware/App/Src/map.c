#include "map.h"


/*
 * ==============================================================
 * NODE COORDINATES
 * ==============================================================
 *
 * These are logical positions on the 6x6 reference grid.
 *
 * They do NOT mean the robot detects every grid cell.
 * Only these 16 positions are real navigation nodes.
 */
static const MapPosition node_position[MAP_NODE_COUNT] =
{
    /* S1  */ {0U, 0U},
    /* S2  */ {0U, 5U},
    /* S3  */ {5U, 0U},
    /* S4  */ {5U, 5U},

    /* S5  */ {2U, 1U},
    /* S6  */ {3U, 4U},

    /* J1  */ {1U, 2U},
    /* J2  */ {0U, 4U},

    /* J3  */ {2U, 2U},
    /* J4  */ {2U, 3U},
    /* J5  */ {2U, 4U},

    /* J6  */ {3U, 2U},
    /* J7  */ {3U, 3U},

    /* J8  */ {4U, 1U},
    /* J9  */ {4U, 2U},
    /* J10 */ {4U, 3U}
};


/*
 * One physical black-track connection.
 *
 * a / b:
 *      nodes connected by this edge.
 *
 * cost:
 *      physical track length expressed in logical grid units.
 *
 * dir_a:
 *      direction track initially leaves node a.
 *
 * dir_b:
 *      direction track initially leaves node b.
 *
 * We store BOTH directions because some edges contain bends.
 */
typedef struct
{
    MapNodeId a;
    MapNodeId b;

    uint8_t cost;

    MapDirection dir_a;
    MapDirection dir_b;

} MapEdge;


/*
 * ==============================================================
 * EXACT GRAPH OF THE PRINTED FLEX
 * ==============================================================
 */
static const MapEdge edges[] =
{
    /*
     * S1 connections.
     */
    {MAP_S1, MAP_S5, 3U,
     MAP_DIR_SOUTH, MAP_DIR_WEST},

    {MAP_S1, MAP_J1, 3U,
     MAP_DIR_EAST, MAP_DIR_NORTH},


    /*
     * S2 connections.
     */
    {MAP_S2, MAP_J2, 1U,
     MAP_DIR_WEST, MAP_DIR_EAST},

    {MAP_S2, MAP_J5, 3U,
     MAP_DIR_SOUTH, MAP_DIR_EAST},


    /*
     * S3 connections.
     */
    {MAP_S3, MAP_J8, 2U,
     MAP_DIR_NORTH, MAP_DIR_WEST},

    {MAP_S3, MAP_J9, 3U,
     MAP_DIR_EAST, MAP_DIR_SOUTH},


    /*
     * S4 connections.
     */
    {MAP_S4, MAP_S6, 3U,
     MAP_DIR_NORTH, MAP_DIR_EAST},

    {MAP_S4, MAP_J10, 3U,
     MAP_DIR_WEST, MAP_DIR_SOUTH},


    /*
     * S5 connections.
     */
    {MAP_S5, MAP_J1, 2U,
     MAP_DIR_NORTH, MAP_DIR_WEST},

    {MAP_S5, MAP_J3, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_S5, MAP_J8, 2U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},


    /*
     * Upper network.
     */
    {MAP_J1, MAP_J2, 3U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_J2, MAP_J5, 2U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},


    /*
     * Middle-upper network.
     */
    {MAP_J3, MAP_J4, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_J3, MAP_J6, 1U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},

    {MAP_J4, MAP_J5, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_J4, MAP_J7, 1U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},

    {MAP_J5, MAP_S6, 1U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},


    /*
     * Middle-lower network.
     */
    {MAP_J6, MAP_J7, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_J6, MAP_J9, 1U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},

    {MAP_J7, MAP_S6, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_J7, MAP_J10, 1U,
     MAP_DIR_SOUTH, MAP_DIR_NORTH},


    /*
     * S6 lower connection.
     *
     * Physical edge bends:
     *
     * S6 -> SOUTH -> WEST -> J10
     */
    {MAP_S6, MAP_J10, 2U,
     MAP_DIR_SOUTH, MAP_DIR_EAST},


    /*
     * Lower network.
     */
    {MAP_J8, MAP_J9, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST},

    {MAP_J9, MAP_J10, 1U,
     MAP_DIR_EAST, MAP_DIR_WEST}
};


/*
 * Number of real track edges.
 */
#define MAP_EDGE_COUNT \
    ((uint8_t)(sizeof(edges) / sizeof(edges[0])))


/*
 * Dynamic obstacle state.
 *
 * 0 = available
 * 1 = temporarily blocked
 */
static uint8_t blocked[MAP_NODE_COUNT];


/*
 * ==============================================================
 * INITIALIZATION
 * ==============================================================
 */
void Map_Init(void)
{
    /*
     * Printed track itself is permanent.
     *
     * Only temporary obstacle state needs resetting.
     */
    for (uint8_t i = 0U;
         i < (uint8_t)MAP_NODE_COUNT;
         i++)
    {
        blocked[i] = 0U;
    }
}


/*
 * ==============================================================
 * VALIDATION
 * ==============================================================
 */
uint8_t Map_IsValidNode(MapNodeId node)
{
    /*
     * Every real node has an enum value below MAP_NODE_COUNT.
     */
    if ((uint8_t)node < (uint8_t)MAP_NODE_COUNT)
    {
        return 1U;
    }

    return 0U;
}


/*
 * ==============================================================
 * DYNAMIC BLOCKING
 * ==============================================================
 */
void Map_BlockNode(MapNodeId node)
{
    if (Map_IsValidNode(node))
    {
        blocked[(uint8_t)node] = 1U;
    }
}


void Map_UnblockNode(MapNodeId node)
{
    if (Map_IsValidNode(node))
    {
        blocked[(uint8_t)node] = 0U;
    }
}


uint8_t Map_IsBlocked(MapNodeId node)
{
    /*
     * Invalid nodes are treated as unusable.
     */
    if (!Map_IsValidNode(node))
    {
        return 1U;
    }

    return blocked[(uint8_t)node];
}


/*
 * ==============================================================
 * CONNECTION CHECK
 * ==============================================================
 */
uint8_t Map_AreConnected(MapNodeId a,
                         MapNodeId b)
{
    /*
     * Invalid or blocked nodes cannot be traversed.
     */
    if (!Map_IsValidNode(a) ||
        !Map_IsValidNode(b))
    {
        return 0U;
    }

    if (Map_IsBlocked(a) ||
        Map_IsBlocked(b))
    {
        return 0U;
    }

    /*
     * Search the physical edge table.
     */
    for (uint8_t i = 0U;
         i < MAP_EDGE_COUNT;
         i++)
    {
        if ((edges[i].a == a &&
             edges[i].b == b) ||
            (edges[i].a == b &&
             edges[i].b == a))
        {
            return 1U;
        }
    }

    return 0U;
}


/*
 * ==============================================================
 * EDGE COST
 * ==============================================================
 */
uint8_t Map_GetEdgeCost(MapNodeId a,
                        MapNodeId b)
{
    if (!Map_AreConnected(a, b))
    {
        /*
         * 255 means there is no usable direct connection.
         */
        return 255U;
    }

    for (uint8_t i = 0U;
         i < MAP_EDGE_COUNT;
         i++)
    {
        if ((edges[i].a == a &&
             edges[i].b == b) ||
            (edges[i].a == b &&
             edges[i].b == a))
        {
            return edges[i].cost;
        }
    }

    return 255U;
}


/*
 * ==============================================================
 * EXIT DIRECTION
 * ==============================================================
 */
MapDirection Map_GetExitDirection(MapNodeId from,
                                  MapNodeId to)
{
    if (!Map_AreConnected(from, to))
    {
        return MAP_DIR_INVALID;
    }

    for (uint8_t i = 0U;
         i < MAP_EDGE_COUNT;
         i++)
    {
        /*
         * Travelling a -> b.
         */
        if (edges[i].a == from &&
            edges[i].b == to)
        {
            return edges[i].dir_a;
        }

        /*
         * Travelling b -> a.
         */
        if (edges[i].b == from &&
            edges[i].a == to)
        {
            return edges[i].dir_b;
        }
    }

    return MAP_DIR_INVALID;
}


/*
 * ==============================================================
 * LOGICAL POSITION
 * ==============================================================
 */
MapPosition Map_GetPosition(MapNodeId node)
{
    /*
     * Safe invalid result.
     */
    MapPosition invalid =
    {
        255U,
        255U
    };

    if (!Map_IsValidNode(node))
    {
        return invalid;
    }

    return node_position[(uint8_t)node];
}


/*
 * ==============================================================
 * NEIGHBOUR ENUMERATION
 * ==============================================================
 */
uint8_t Map_GetNeighbors(MapNodeId node,
                         MapNodeId *neighbors,
                         uint8_t max_neighbors)
{
    uint8_t count = 0U;

    if (!Map_IsValidNode(node) ||
        neighbors == 0 ||
        max_neighbors == 0U)
    {
        return 0U;
    }

    /*
     * Inspect every physical edge and collect nodes connected
     * to the requested node.
     */
    for (uint8_t i = 0U;
         i < MAP_EDGE_COUNT;
         i++)
    {
        MapNodeId candidate =
            MAP_NODE_INVALID;

        if (edges[i].a == node)
        {
            candidate = edges[i].b;
        }
        else if (edges[i].b == node)
        {
            candidate = edges[i].a;
        }

        /*
         * Skip edges that do not touch this node.
         */
        if (candidate == MAP_NODE_INVALID)
        {
            continue;
        }

        /*
         * Skip currently blocked destinations.
         */
        if (Map_IsBlocked(candidate))
        {
            continue;
        }

        if (count >= max_neighbors)
        {
            break;
        }

        neighbors[count++] = candidate;
    }

    return count;
}
