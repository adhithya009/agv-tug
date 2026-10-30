#ifndef SPI_CFG_H
#define SPI_CFG_H

#include <stdint.h>

/*
 * SPI1:
 *
 * PB3 -> SCK
 * PB4 -> MISO
 * PB5 -> MOSI
 *
 * Used by MFRC522 RFID reader.
 */
void SPI1_Init(void);

/*
 * Full-duplex transfer of one byte.
 *
 * Returns byte received from slave.
 */
uint8_t SPI1_TransferByte(uint8_t data);

#endif
