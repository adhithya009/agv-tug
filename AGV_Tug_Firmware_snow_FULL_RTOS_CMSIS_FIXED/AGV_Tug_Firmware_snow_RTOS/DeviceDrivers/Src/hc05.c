#include "hc05.h"

#include "uart_cfg.h"


#define HC05_BUFFER_SIZE 16U


static char rx_buffer[HC05_BUFFER_SIZE];

static uint8_t rx_index = 0U;


void HC05_Init(void)
{
    /*
     * HC-05 default data-mode baud rate is
     * commonly 9600 baud.
     */
    UART1_Init(9600U);

    rx_index = 0U;
}


void HC05_SendString(const char *text)
{
    UART1_WriteString(text);
}


static uint8_t HC05_ParseDestination(uint8_t *station)
{
    if (station == 0)
    {
        return 0U;
    }

    /*
     * Accepted commands:
     *
     * S1
     * S2
     * ...
     * S6
     */
    if ((rx_index == 2U) &&
        ((rx_buffer[0] == 'S') ||
         (rx_buffer[0] == 's')) &&
        (rx_buffer[1] >= '1') &&
        (rx_buffer[1] <= '6'))
    {
        *station =
            (uint8_t)(rx_buffer[1] - '0');

        return 1U;
    }

    return 0U;
}


uint8_t HC05_PollDestination(uint8_t *station)
{
    uint8_t data;

    /*
     * No new byte available.
     */
    if (!UART1_ReadByteNonBlocking(&data))
    {
        return 0U;
    }

    /*
     * Ignore carriage return.
     */
    if (data == '\r')
    {
        return 0U;
    }

    /*
     * Newline means command is complete.
     */
    if (data == '\n')
    {
        uint8_t valid =
            HC05_ParseDestination(station);

        rx_index = 0U;

        if (valid)
        {
            HC05_SendString("DEST OK\r\n");
        }
        else
        {
            HC05_SendString("INVALID\r\n");
        }

        return valid;
    }

    /*
     * Store normal character.
     */
    if (rx_index < (HC05_BUFFER_SIZE - 1U))
    {
        rx_buffer[rx_index] = (char)data;

        rx_index++;
    }
    else
    {
        /*
         * Overflow protection:
         * discard malformed command.
         */
        rx_index = 0U;
    }

    return 0U;
}
