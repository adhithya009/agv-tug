#ifndef ASTAR_H
#define ASTAR_H

#include <stdint.h>
#include "map.h"

/*
 * Path returned by A*.
 *
 * nodes[0]          = starting node
 * nodes[length - 1] = destination node
 */
typedef struct
{
    MapNodeId nodes[MAP_NODE_COUNT];
    uint8_t length;

} AStarPath;


/*
 * Pure shortest-path search.
 *
 * Keep this API so existing code continues
 * to compile unchanged.
 */
uint8_t AStar_FindPath(MapNodeId start,
                       MapNodeId goal,
                       AStarPath *path);


/*
 * Heading-aware shortest-path search.
 *
 * The first edge MUST leave the start node
 * in start_heading because Navigation_SetPath()
 * assumes the robot is already physically
 * facing along the first route edge.
 *
 * For equal-distance routes, fewer turns
 * are preferred.
 */
uint8_t AStar_FindPathWithHeading(
    MapNodeId start,
    MapNodeId goal,
    MapDirection start_heading,
    AStarPath *path);

#endif
