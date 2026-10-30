#include "oled.h"

#include "i2c_cfg.h"


#define OLED_ADDRESS 0x3CU


/*
 * ==============================================================
 * FONT TABLES
 * ==============================================================
 *
 * 5 columns per character.
 */

static const uint8_t digits[10][5] =
{
    {0x3EU,0x51U,0x49U,0x45U,0x3EU},
    {0x00U,0x42U,0x7FU,0x40U,0x00U},
    {0x42U,0x61U,0x51U,0x49U,0x46U},
    {0x21U,0x41U,0x45U,0x4BU,0x31U},
    {0x18U,0x14U,0x12U,0x7FU,0x10U},
    {0x27U,0x45U,0x45U,0x45U,0x39U},
    {0x3CU,0x4AU,0x49U,0x49U,0x30U},
    {0x01U,0x71U,0x09U,0x05U,0x03U},
    {0x36U,0x49U,0x49U,0x49U,0x36U},
    {0x06U,0x49U,0x49U,0x29U,0x1EU}
};


static const uint8_t letters[26][5] =
{
    {0x7EU,0x11U,0x11U,0x11U,0x7EU}, /* A */
    {0x7FU,0x49U,0x49U,0x49U,0x36U}, /* B */
    {0x3EU,0x41U,0x41U,0x41U,0x22U}, /* C */
    {0x7FU,0x41U,0x41U,0x22U,0x1CU}, /* D */
    {0x7FU,0x49U,0x49U,0x49U,0x41U}, /* E */
    {0x7FU,0x09U,0x09U,0x09U,0x01U}, /* F */
    {0x3EU,0x41U,0x49U,0x49U,0x7AU}, /* G */
    {0x7FU,0x08U,0x08U,0x08U,0x7FU}, /* H */
    {0x00U,0x41U,0x7FU,0x41U,0x00U}, /* I */
    {0x20U,0x40U,0x41U,0x3FU,0x01U}, /* J */
    {0x7FU,0x08U,0x14U,0x22U,0x41U}, /* K */
    {0x7FU,0x40U,0x40U,0x40U,0x40U}, /* L */
    {0x7FU,0x02U,0x0CU,0x02U,0x7FU}, /* M */
    {0x7FU,0x04U,0x08U,0x10U,0x7FU}, /* N */
    {0x3EU,0x41U,0x41U,0x41U,0x3EU}, /* O */
    {0x7FU,0x09U,0x09U,0x09U,0x06U}, /* P */
    {0x3EU,0x41U,0x51U,0x21U,0x5EU}, /* Q */
    {0x7FU,0x09U,0x19U,0x29U,0x46U}, /* R */
    {0x46U,0x49U,0x49U,0x49U,0x31U}, /* S */
    {0x01U,0x01U,0x7FU,0x01U,0x01U}, /* T */
    {0x3FU,0x40U,0x40U,0x40U,0x3FU}, /* U */
    {0x1FU,0x20U,0x40U,0x20U,0x1FU}, /* V */
    {0x3FU,0x40U,0x38U,0x40U,0x3FU}, /* W */
    {0x63U,0x14U,0x08U,0x14U,0x63U}, /* X */
    {0x07U,0x08U,0x70U,0x08U,0x07U}, /* Y */
    {0x61U,0x51U,0x49U,0x45U,0x43U}  /* Z */
};


static uint8_t OLED_Command(uint8_t command)
{
    /*
     * SSD1306 control byte 0x00 means command.
     */
    return I2C1_WriteReg(
        OLED_ADDRESS,
        0x00U,
        command);
}


static uint8_t OLED_Data(uint8_t data)
{
    /*
     * SSD1306 control byte 0x40 means display data.
     */
    return I2C1_WriteReg(
        OLED_ADDRESS,
        0x40U,
        data);
}


static void OLED_GetGlyph(char c,
                          uint8_t glyph[5])
{
    /*
     * Convert lowercase to uppercase.
     */
    if ((c >= 'a') &&
        (c <= 'z'))
    {
        c =
            (char)(c - 'a' + 'A');
    }

    if ((c >= 'A') &&
        (c <= 'Z'))
    {
        uint8_t index =
            (uint8_t)(c - 'A');

        for (uint8_t i = 0U;
             i < 5U;
             i++)
        {
            glyph[i] =
                letters[index][i];
        }

        return;
    }

    if ((c >= '0') &&
        (c <= '9'))
    {
        uint8_t index =
            (uint8_t)(c - '0');

        for (uint8_t i = 0U;
             i < 5U;
             i++)
        {
            glyph[i] =
                digits[index][i];
        }

        return;
    }

    /*
     * Space.
     */
    if (c == ' ')
    {
        for (uint8_t i = 0U;
             i < 5U;
             i++)
        {
            glyph[i] = 0U;
        }

        return;
    }

    /*
     * Colon.
     */
    if (c == ':')
    {
        glyph[0] = 0x00U;
        glyph[1] = 0x36U;
        glyph[2] = 0x36U;
        glyph[3] = 0x00U;
        glyph[4] = 0x00U;

        return;
    }

    /*
     * Dash.
     */
    if (c == '-')
    {
        glyph[0] = 0x08U;
        glyph[1] = 0x08U;
        glyph[2] = 0x08U;
        glyph[3] = 0x08U;
        glyph[4] = 0x08U;

        return;
    }

    /*
     * Dot.
     */
    if (c == '.')
    {
        glyph[0] = 0x00U;
        glyph[1] = 0x60U;
        glyph[2] = 0x60U;
        glyph[3] = 0x00U;
        glyph[4] = 0x00U;

        return;
    }

    /*
     * Slash.
     */
    if (c == '/')
    {
        glyph[0] = 0x20U;
        glyph[1] = 0x10U;
        glyph[2] = 0x08U;
        glyph[3] = 0x04U;
        glyph[4] = 0x02U;

        return;
    }

    /*
     * Unknown character -> ?.
     */
    glyph[0] = 0x02U;
    glyph[1] = 0x01U;
    glyph[2] = 0x51U;
    glyph[3] = 0x09U;
    glyph[4] = 0x06U;
}


uint8_t OLED_Init(void)
{
    /*
     * Display OFF.
     */
    if (!OLED_Command(0xAEU))
    {
        return 0U;
    }

    /*
     * Clock divide.
     */
    OLED_Command(0xD5U);
    OLED_Command(0x80U);

    /*
     * Multiplex ratio for 64 rows.
     */
    OLED_Command(0xA8U);
    OLED_Command(0x3FU);

    /*
     * Display offset.
     */
    OLED_Command(0xD3U);
    OLED_Command(0x00U);

    /*
     * Start line = 0.
     */
    OLED_Command(0x40U);

    /*
     * Charge pump ON.
     */
    OLED_Command(0x8DU);
    OLED_Command(0x14U);

    /*
     * Page addressing mode.
     */
    OLED_Command(0x20U);
    OLED_Command(0x02U);

    /*
     * Segment remap.
     */
    OLED_Command(0xA1U);

    /*
     * COM scan direction.
     */
    OLED_Command(0xC8U);

    /*
     * COM pins.
     */
    OLED_Command(0xDAU);
    OLED_Command(0x12U);

    /*
     * Contrast.
     */
    OLED_Command(0x81U);
    OLED_Command(0x7FU);

    /*
     * Precharge period.
     */
    OLED_Command(0xD9U);
    OLED_Command(0xF1U);

    /*
     * VCOM detect.
     */
    OLED_Command(0xDBU);
    OLED_Command(0x40U);

    /*
     * Entire display follows RAM.
     */
    OLED_Command(0xA4U);

    /*
     * Normal, not inverted.
     */
    OLED_Command(0xA6U);

    /*
     * Display ON.
     */
    OLED_Command(0xAFU);

    OLED_Clear();

    return 1U;
}


void OLED_SetCursor(uint8_t page,
                    uint8_t column)
{
    if (page > 7U)
    {
        page = 7U;
    }

    if (column > 127U)
    {
        column = 127U;
    }

    /*
     * Page address.
     */
    OLED_Command(
        (uint8_t)(0xB0U | page));

    /*
     * Lower column nibble.
     */
    OLED_Command(
        (uint8_t)(column & 0x0FU));

    /*
     * Upper column nibble.
     */
    OLED_Command(
        (uint8_t)(0x10U |
        ((column >> 4U) & 0x0FU)));
}


void OLED_Clear(void)
{
    for (uint8_t page = 0U;
         page < 8U;
         page++)
    {
        OLED_SetCursor(page,
                       0U);

        for (uint8_t column = 0U;
             column < 128U;
             column++)
        {
            OLED_Data(0x00U);
        }
    }

    OLED_SetCursor(0U,
                   0U);
}


void OLED_WriteChar(char c)
{
    uint8_t glyph[5];

    OLED_GetGlyph(c,
                  glyph);

    for (uint8_t i = 0U;
         i < 5U;
         i++)
    {
        OLED_Data(glyph[i]);
    }

    /*
     * One blank column between characters.
     */
    OLED_Data(0x00U);
}


void OLED_WriteString(const char *text)
{
    if (text == 0)
    {
        return;
    }

    while (*text != '\0')
    {
        OLED_WriteChar(*text);

        text++;
    }
}
