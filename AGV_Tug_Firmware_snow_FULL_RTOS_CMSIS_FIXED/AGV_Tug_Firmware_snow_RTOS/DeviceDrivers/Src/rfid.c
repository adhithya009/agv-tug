#include "rfid.h"
#include "spi_cfg.h"
#include "gpio_cfg.h"

#include "stm32f4xx.h"


/* ==============================================================
 * MFRC522 COMMANDS
 * ============================================================== */

#define PCD_IDLE        0x00U
#define PCD_TRANSCEIVE  0x0CU
#define PCD_RESETPHASE  0x0FU

#define PICC_REQIDL     0x26U
#define PICC_ANTICOLL   0x93U


/* ==============================================================
 * MFRC522 REGISTERS
 * ============================================================== */

#define CommandReg      0x01U
#define ComIEnReg       0x02U
#define ComIrqReg       0x04U
#define ErrorReg        0x06U
#define FIFODataReg     0x09U
#define FIFOLevelReg    0x0AU
#define ControlReg      0x0CU
#define BitFramingReg   0x0DU
#define CollReg         0x0EU

#define ModeReg         0x11U
#define TxControlReg    0x14U
#define TxASKReg        0x15U

#define TModeReg        0x2AU
#define TPrescalerReg   0x2BU
#define TReloadRegH     0x2CU
#define TReloadRegL     0x2DU


/* ==============================================================
 * RFID CHIP SELECT
 *
 * PB10 -> MFRC522 SDA / CS
 *
 * Active LOW.
 * ============================================================== */

static void RFID_CS_Low(void)
{
    /*
     * GPIO abstraction keeps the physical
     * CS pin in gpio_cfg.c.
     */
    GPIO_RFID_CS(0U);
}


static void RFID_CS_High(void)
{
    GPIO_RFID_CS(1U);
}


/* ==============================================================
 * RFID RESET
 *
 * PA6 -> MFRC522 RST
 *
 * PB0 / Arduino A3 is now used by the
 * RIGHT-CENTER line IR sensor.
 * ============================================================== */

static void RFID_ResetLow(void)
{
    GPIO_RFID_RST(0U);
}


static void RFID_ResetHigh(void)
{
    GPIO_RFID_RST(1U);
}


/* ==============================================================
 * SIMPLE STARTUP DELAY
 * ============================================================== */

static void RFID_Delay(void)
{
    for (volatile uint32_t i = 0U;
         i < 100000U;
         i++)
    {
        __NOP();
    }
}


/* ==============================================================
 * WRITE MFRC522 REGISTER
 * ============================================================== */

static void RFID_WriteRegister(
    uint8_t reg,
    uint8_t value)
{
    /*
     * MFRC522 SPI write format:
     *
     * 0XXXXXX0
     */
    uint8_t address =
        (uint8_t)
        ((reg << 1U) & 0x7EU);


    RFID_CS_Low();


    SPI1_TransferByte(
        address);


    SPI1_TransferByte(
        value);


    RFID_CS_High();
}


/* ==============================================================
 * READ MFRC522 REGISTER
 * ============================================================== */

static uint8_t RFID_ReadRegister(
    uint8_t reg)
{
    /*
     * MFRC522 SPI read format:
     *
     * 1XXXXXX0
     */
    uint8_t address =
        (uint8_t)
        (((reg << 1U) & 0x7EU) |
         0x80U);


    RFID_CS_Low();


    SPI1_TransferByte(
        address);


    uint8_t value =
        SPI1_TransferByte(
            0x00U);


    RFID_CS_High();


    return value;
}


/* ==============================================================
 * SET REGISTER BIT MASK
 * ============================================================== */

static void RFID_SetBitMask(
    uint8_t reg,
    uint8_t mask)
{
    uint8_t value =
        RFID_ReadRegister(reg);


    RFID_WriteRegister(
        reg,
        (uint8_t)(value | mask));
}


/* ==============================================================
 * CLEAR REGISTER BIT MASK
 * ============================================================== */

static void RFID_ClearBitMask(
    uint8_t reg,
    uint8_t mask)
{
    uint8_t value =
        RFID_ReadRegister(reg);


    RFID_WriteRegister(
        reg,
        (uint8_t)(value & (~mask)));
}


/* ==============================================================
 * ENABLE RFID ANTENNA
 * ============================================================== */

static void RFID_AntennaOn(void)
{
    uint8_t value =
        RFID_ReadRegister(
            TxControlReg);


    if ((value & 0x03U) !=
        0x03U)
    {
        RFID_SetBitMask(
            TxControlReg,
            0x03U);
    }
}


/* ==============================================================
 * COMMUNICATE WITH RFID CARD
 * ============================================================== */

static uint8_t RFID_ToCard(
    uint8_t command,
    uint8_t *send_data,
    uint8_t send_length,
    uint8_t *back_data,
    uint16_t *back_length_bits)
{
    uint8_t irq_enable =
        0U;

    uint8_t wait_irq =
        0U;


    /*
     * Request and anticollision both
     * use TRANSCEIVE.
     */
    if (command ==
        PCD_TRANSCEIVE)
    {
        irq_enable =
            0x77U;

        wait_irq =
            0x30U;
    }


    RFID_WriteRegister(
        ComIEnReg,
        (uint8_t)(irq_enable | 0x80U));


    RFID_ClearBitMask(
        ComIrqReg,
        0x80U);


    RFID_SetBitMask(
        FIFOLevelReg,
        0x80U);


    RFID_WriteRegister(
        CommandReg,
        PCD_IDLE);


    /*
     * Load outgoing data into MFRC522 FIFO.
     */
    for (uint8_t i = 0U;
         i < send_length;
         i++)
    {
        RFID_WriteRegister(
            FIFODataReg,
            send_data[i]);
    }


    /*
     * Start requested MFRC522 command.
     */
    RFID_WriteRegister(
        CommandReg,
        command);


    if (command ==
        PCD_TRANSCEIVE)
    {
        RFID_SetBitMask(
            BitFramingReg,
            0x80U);
    }


    /*
     * Wait for transmit / receive completion.
     */
    uint16_t timeout =
        2000U;

    uint8_t irq_value;


    do
    {
        irq_value =
            RFID_ReadRegister(
                ComIrqReg);


        if (timeout ==
            0U)
        {
            break;
        }


        timeout--;


    } while (
        ((irq_value & 0x01U) == 0U) &&
        ((irq_value & wait_irq) == 0U));


    /*
     * Stop transmission.
     */
    RFID_ClearBitMask(
        BitFramingReg,
        0x80U);


    /*
     * Communication timeout.
     */
    if (timeout ==
        0U)
    {
        return
            0U;
    }


    /*
     * Reject MFRC522 communication errors.
     */
    if ((RFID_ReadRegister(
             ErrorReg) &
         0x1BU) !=
        0U)
    {
        return
            0U;
    }


    /*
     * Read received bytes.
     */
    if (command ==
        PCD_TRANSCEIVE)
    {
        uint8_t fifo_count =
            RFID_ReadRegister(
                FIFOLevelReg);


        uint8_t last_bits =
            (uint8_t)
            (RFID_ReadRegister(
                 ControlReg) &
             0x07U);


        if (last_bits !=
            0U)
        {
            *back_length_bits =
                (uint16_t)
                ((fifo_count - 1U) *
                 8U +
                 last_bits);
        }
        else
        {
            *back_length_bits =
                (uint16_t)
                (fifo_count *
                 8U);
        }


        if (fifo_count ==
            0U)
        {
            fifo_count =
                1U;
        }


        if (fifo_count >
            16U)
        {
            fifo_count =
                16U;
        }


        for (uint8_t i = 0U;
             i < fifo_count;
             i++)
        {
            back_data[i] =
                RFID_ReadRegister(
                    FIFODataReg);
        }
    }


    return
        1U;
}


/* ==============================================================
 * SEARCH FOR RFID CARD
 * ============================================================== */

static uint8_t RFID_Request(void)
{
    uint8_t buffer[2];

    uint16_t bits =
        0U;


    /*
     * REQA uses seven data bits.
     */
    RFID_WriteRegister(
        BitFramingReg,
        0x07U);


    buffer[0] =
        PICC_REQIDL;


    if (!RFID_ToCard(
            PCD_TRANSCEIVE,
            buffer,
            1U,
            buffer,
            &bits))
    {
        return
            0U;
    }


    /*
     * REQA response must be 16 bits.
     */
    if (bits !=
        0x10U)
    {
        return
            0U;
    }


    return
        1U;
}


/* ==============================================================
 * RFID ANTICOLLISION
 * ============================================================== */

static uint8_t RFID_Anticollision(
    uint8_t serial[5])
{
    uint8_t buffer[5];

    uint16_t bits =
        0U;


    RFID_WriteRegister(
        BitFramingReg,
        0x00U);


    RFID_ClearBitMask(
        CollReg,
        0x80U);


    /*
     * CL1 anticollision command.
     */
    buffer[0] =
        PICC_ANTICOLL;


    buffer[1] =
        0x20U;


    if (!RFID_ToCard(
            PCD_TRANSCEIVE,
            buffer,
            2U,
            serial,
            &bits))
    {
        return
            0U;
    }


    /*
     * Four UID bytes + one BCC byte.
     */
    if (bits !=
        40U)
    {
        return
            0U;
    }


    /*
     * Validate BCC:
     *
     * UID0 XOR UID1 XOR UID2 XOR UID3.
     */
    uint8_t check =
        serial[0] ^
        serial[1] ^
        serial[2] ^
        serial[3];


    if (check !=
        serial[4])
    {
        return
            0U;
    }


    /*
     * 0x88 indicates UID cascade.
     *
     * Current project uses normal
     * four-byte UIDs.
     */
    if (serial[0] ==
        0x88U)
    {
        return
            0U;
    }


    return
        1U;
}


/* ==============================================================
 * INITIALIZE MFRC522
 * ============================================================== */

void RFID_Init(void)
{
    /*
     * Ensure SPI1 is configured.
     */
    SPI1_Init();


    /*
     * RFID is deselected initially.
     */
    RFID_CS_High();


    /*
     * Hardware reset using PA6.
     */
    RFID_ResetLow();


    RFID_Delay();


    RFID_ResetHigh();


    RFID_Delay();


    /*
     * MFRC522 software reset.
     */
    RFID_WriteRegister(
        CommandReg,
        PCD_RESETPHASE);


    RFID_Delay();


    /*
     * Standard MFRC522 timer setup.
     */
    RFID_WriteRegister(
        TModeReg,
        0x8DU);


    RFID_WriteRegister(
        TPrescalerReg,
        0x3EU);


    RFID_WriteRegister(
        TReloadRegL,
        30U);


    RFID_WriteRegister(
        TReloadRegH,
        0U);


    RFID_WriteRegister(
        TxASKReg,
        0x40U);


    RFID_WriteRegister(
        ModeReg,
        0x3DU);


    /*
     * Enable RF field.
     */
    RFID_AntennaOn();
}


/* ==============================================================
 * READ RFID UID
 * ============================================================== */

uint8_t RFID_ReadUID(
    RFID_UID *uid)
{
    if (uid ==
        0)
    {
        return
            0U;
    }


    /*
     * First check whether a card exists.
     */
    if (!RFID_Request())
    {
        return
            0U;
    }


    uint8_t serial[5];


    /*
     * Obtain card UID.
     */
    if (!RFID_Anticollision(
            serial))
    {
        return
            0U;
    }


    uid->uid[0] =
        serial[0];


    uid->uid[1] =
        serial[1];


    uid->uid[2] =
        serial[2];


    uid->uid[3] =
        serial[3];


    return
        1U;
}


/* ==============================================================
 * COMPARE TWO RFID UIDS
 * ============================================================== */

uint8_t RFID_UIDEquals(
    const RFID_UID *a,
    const RFID_UID *b)
{
    if ((a ==
         0) ||

        (b ==
         0))
    {
        return
            0U;
    }


    for (uint8_t i = 0U;
         i < 4U;
         i++)
    {
        if (a->uid[i] !=
            b->uid[i])
        {
            return
                0U;
        }
    }


    return
        1U;
}
