#ifndef RFID_STATION_H
#define RFID_STATION_H

#include <stdint.h>

#include "rfid.h"
#include "map.h"


/*
 * Returns:
 *
 * 1 = S1
 * 2 = S2
 * 3 = S3
 * 4 = S4
 * 5 = S5
 * 6 = S6
 *
 * 0 = unknown RFID
 */
uint8_t RFIDStation_GetStation(
    const RFID_UID *uid);

/*
 * Convert RFID UID to MapNodeId (MAP_S1 .. MAP_S6).
 *
 * Returns:
 * 1 if valid station tag matched and *node populated.
 * 0 if unknown tag or null pointer.
 */
uint8_t RFIDStation_GetNode(
    const RFID_UID *uid,
    MapNodeId *node);

/*
 * Check whether the UID matches the expected station node.
 *
 * Returns 1 if matched, 0 otherwise.
 */
uint8_t RFIDStation_MatchesNode(
    const RFID_UID *uid,
    MapNodeId expected_node);

#endif
