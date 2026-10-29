#include "rfid_station.h"


/* ==============================================================
 * KNOWN STATION TAGS
 * ============================================================== */

static const uint8_t station_uid[6][4] =
{
    /* S1 */
    {0x96U, 0x43U, 0x69U, 0xE7U},

    /* S2 */
    {0x22U, 0x4CU, 0xDDU, 0x06U},

    /* S3 */
    {0xF3U, 0x76U, 0x69U, 0xE7U},

    /* S4 */
    {0x42U, 0x12U, 0xDDU, 0x06U},

    /* S5 */
    {0x7DU, 0xDBU, 0xE0U, 0x06U},

    /* S6 */
    {0x74U, 0x66U, 0xD3U, 0x06U}
};


/* ==============================================================
 * COMPARE
 * ============================================================== */

static uint8_t RFIDStation_Matches(
    const RFID_UID *uid,
    const uint8_t expected[4])
{
    if (uid == 0)
    {
        return 0U;
    }

    for (uint8_t i = 0U;
         i < 4U;
         i++)
    {
        if (uid->uid[i] !=
            expected[i])
        {
            return 0U;
        }
    }

    return 1U;
}


/* ==============================================================
 * GET STATION NUMBER (1..6)
 * ============================================================== */

uint8_t RFIDStation_GetStation(
    const RFID_UID *uid)
{
    for (uint8_t i = 0U;
         i < 6U;
         i++)
    {
        if (RFIDStation_Matches(
                uid,
                station_uid[i]))
        {
            return (uint8_t)(i + 1U);
        }
    }

    return 0U;
}


/* ==============================================================
 * GET MAP NODE ID (MAP_S1..MAP_S6)
 * ============================================================== */

uint8_t RFIDStation_GetNode(
    const RFID_UID *uid,
    MapNodeId *node)
{
    uint8_t st = RFIDStation_GetStation(uid);

    if ((st >= 1U) && (st <= 6U) && (node != 0))
    {
        *node = (MapNodeId)(st - 1U);
        return 1U;
    }

    return 0U;
}


/* ==============================================================
 * MATCH SPECIFIC NODE
 * ============================================================== */

uint8_t RFIDStation_MatchesNode(
    const RFID_UID *uid,
    MapNodeId expected_node)
{
    if ((uint8_t)expected_node < 6U)
    {
        return RFIDStation_Matches(uid, station_uid[(uint8_t)expected_node]);
    }

    return 0U;
}
