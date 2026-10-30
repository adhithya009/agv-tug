#include "test_rfid.h"

#include <stdint.h>

#include "rfid.h"
#include "rfid_station.h"
#include "motor.h"


volatile uint8_t debug_rfid_detected = 0U;

volatile uint8_t debug_rfid_uid0 = 0U;
volatile uint8_t debug_rfid_uid1 = 0U;
volatile uint8_t debug_rfid_uid2 = 0U;
volatile uint8_t debug_rfid_uid3 = 0U;

volatile uint8_t debug_rfid_station = 0U;


void Test_RFID(void)
{
    Motor_Stop();

    RFID_Init();

    while (1)
    {
        RFID_UID uid;

        if (RFID_ReadUID(&uid))
        {
            debug_rfid_detected = 1U;

            debug_rfid_uid0 =
                uid.uid[0];

            debug_rfid_uid1 =
                uid.uid[1];

            debug_rfid_uid2 =
                uid.uid[2];

            debug_rfid_uid3 =
                uid.uid[3];

            /*
             * Will remain zero until we insert
             * real station UID values.
             */
            debug_rfid_station =
                RFIDStation_GetStation(
                    &uid);
        }
        else
        {
            debug_rfid_detected = 0U;
        }
    }
}
