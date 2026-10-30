#ifndef OLED_H
#define OLED_H

#include <stdint.h>


/*
 * SSD1306:
 *
 * I2C address = 0x3C
 * display     = 128 x 64
 */
uint8_t OLED_Init(void);


/*
 * Clear complete display.
 */
void OLED_Clear(void);


/*
 * page:
 * 0..7
 *
 * column:
 * 0..127
 */
void OLED_SetCursor(uint8_t page,
                    uint8_t column);


/*
 * Write one 5x7 character.
 */
void OLED_WriteChar(char c);


/*
 * Write null-terminated string.
 */
void OLED_WriteString(const char *text);

#endif
