#ifndef RFID_H
#define RFID_H

#include <stdint.h>


typedef struct
{
    /*
     * Standard 4-byte UID.
     */
    uint8_t uid[4];

} RFID_UID;


/*
 * Initialize MFRC522.
 */
void RFID_Init(void);

/*
 * Look for a card and read its UID.
 *
 * Returns:
 * 1 -> card detected and UID valid
 * 0 -> no valid card
 */
uint8_t RFID_ReadUID(RFID_UID *uid);

/*
 * Compare two UIDs.
 */
uint8_t RFID_UIDEquals(const RFID_UID *a,
                       const RFID_UID *b);

#endif
