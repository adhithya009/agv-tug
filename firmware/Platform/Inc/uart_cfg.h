#ifndef UART_CFG_H
#define UART_CFG_H

#include <stdint.h>

/*
 * USART1:
 *
 * PA9  -> TX
 * PA10 -> RX
 *
 * Used by HC-05 Bluetooth module.
 */
void UART1_Init(uint32_t baudrate);

/*
 * Send one byte.
 */
void UART1_WriteByte(uint8_t data);

/*
 * Send a null-terminated string.
 */
void UART1_WriteString(const char *text);

/*
 * Non-blocking receive.
 *
 * Returns:
 * 1 -> byte received
 * 0 -> no byte currently available
 */
uint8_t UART1_ReadByteNonBlocking(uint8_t *data);

#endif
