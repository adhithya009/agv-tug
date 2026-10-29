#ifndef MAP_H
#define MAP_H

#include <stdint.h>

/*
 * ==============================================================
 * REAL NAVIGATION NODES ON THE PRINTED FLEX
 * ==============================================================
 *
 * Stations / labelled points:
 * S1, S2, S3, S4, S5, S6
 *
 * Internal route-choice junctions:
 * J1 ... J10
 */
typedef enum
{
    MAP_S1 = 0,
    MAP_S2,
    MAP_S3,
    MAP_S4,
    MAP_S5,
    MAP_S6,

    MAP_J1,
    MAP_J2,
    MAP_J3,
    MAP_J4,
    MAP_J5,
    MAP_J6,
    MAP_J7,
    MAP_J8,
    MAP_J9,
    MAP_J10,

    MAP_NODE_COUNT,
    MAP_NODE_INVALID = 255
} MapNodeId;


/*
 * Physical direction in which a track leaves a node.
 *
 * Values are deliberately clockwise:
 *
 * NORTH = 0
 * EAST  = 1
 * SOUTH = 2
 * WEST  = 3
 *
 * This makes LEFT/RIGHT calculations easy later.
 */
typedef enum
{
    MAP_DIR_NORTH = 0,
    MAP_DIR_EAST,
    MAP_DIR_SOUTH,
    MAP_DIR_WEST,

    MAP_DIR_INVALID = 255
} MapDirection;


/*
 * Logical 6x6 position.
 *
 * Used by A* only for the heuristic.
 */
typedef struct
{
    uint8_t row;
    uint8_t col;
} MapPosition;


/* Initialize dynamic map state. */
void Map_Init(void);


/* Check whether a node ID actually exists. */
uint8_t Map_IsValidNode(MapNodeId node);


/* Temporarily block/unblock a navigation node. */
void Map_BlockNode(MapNodeId node);
void Map_UnblockNode(MapNodeId node);
uint8_t Map_IsBlocked(MapNodeId node);


/* Check whether a physical black-track edge connects two nodes. */
uint8_t Map_AreConnected(MapNodeId a,
                         MapNodeId b);


/* Return physical track cost between directly connected nodes. */
uint8_t Map_GetEdgeCost(MapNodeId a,
                        MapNodeId b);


/*
 * Return the direction in which the track leaves
 * 'from' when travelling toward 'to'.
 */
MapDirection Map_GetExitDirection(MapNodeId from,
                                  MapNodeId to);


/* Return logical coordinate used by A* heuristic. */
MapPosition Map_GetPosition(MapNodeId node);


/*
 * Copy connected usable neighbours into the supplied array.
 *
 * Returns number of neighbours written.
 */
uint8_t Map_GetNeighbors(MapNodeId node,
                         MapNodeId *neighbors,
                         uint8_t max_neighbors);

#endif
