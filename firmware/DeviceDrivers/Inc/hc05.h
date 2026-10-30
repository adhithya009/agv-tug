#ifndef HC05_H
#define HC05_H

#include <stdint.h>

/*
 * Initialize HC-05 UART interface.
 *
 * HC-05 commonly uses 9600 baud in data mode.
 */
void HC05_Init(void);

/*
 * Poll Bluetooth input.
 *
 * Phone should send:
 *
 * S1
 * S2
 * S3
 * ...
 * S6
 *
 * followed by newline.
 *
 * Returns:
 * 1 -> valid destination received
 * 0 -> no complete valid command
 *
 * station receives value 1..6.
 */
uint8_t HC05_PollDestination(uint8_t *station);

/*
 * Send text to phone.
 */
void HC05_SendString(const char *text);

#endif
