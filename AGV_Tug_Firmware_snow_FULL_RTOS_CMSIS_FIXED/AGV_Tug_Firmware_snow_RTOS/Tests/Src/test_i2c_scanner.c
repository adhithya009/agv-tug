#include "test_i2c_scanner.h"

#include <stdint.h>

#include "stm32f4xx.h"
#include "motor.h"


/*
 * ==============================================================
 * DEBUG RESULTS
 * ==============================================================
 *
 * Watch:
 *
 * debug_i2c_count
 * debug_i2c_found
 *
 * Also watch the specific addresses:
 *
 * debug_i2c_3c
 * debug_i2c_68
 * debug_i2c_69
 * debug_i2c_70
 */

volatile uint8_t debug_i2c_count = 0U;

volatile uint8_t debug_i2c_found[16] =
{
    0U
};

volatile uint8_t debug_i2c_3c = 0U;
volatile uint8_t debug_i2c_68 = 0U;
volatile uint8_t debug_i2c_69 = 0U;
volatile uint8_t debug_i2c_70 = 0U;


/*
 * Maximum time we wait for an I2C hardware flag.
 */
#define I2C_SCAN_TIMEOUT 100000U


/*
 * ==============================================================
 * PROBE ONE 7-BIT I2C ADDRESS
 * ==============================================================
 *
 * Returns:
 *
 * 1 -> device acknowledged
 * 0 -> no acknowledgement
 */

static uint8_t I2C1_ProbeAddress(uint8_t address)
{
    uint32_t timeout;


    /*
     * ----------------------------------------------------------
     * WAIT UNTIL BUS IS FREE
     * ----------------------------------------------------------
     */

    timeout = I2C_SCAN_TIMEOUT;

    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U)
    {
        if (timeout-- == 0U)
        {
            return 0U;
        }
    }


    /*
     * ----------------------------------------------------------
     * GENERATE START CONDITION
     * ----------------------------------------------------------
     */

    I2C1->CR1 |= I2C_CR1_START;


    /*
     * Wait for SB:
     *
     * Start condition generated.
     */
    timeout = I2C_SCAN_TIMEOUT;

    while ((I2C1->SR1 & I2C_SR1_SB) == 0U)
    {
        if (timeout-- == 0U)
        {
            I2C1->CR1 |= I2C_CR1_STOP;

            return 0U;
        }
    }


    /*
     * ----------------------------------------------------------
     * SEND 7-BIT ADDRESS
     * ----------------------------------------------------------
     *
     * Our scanner uses normal 7-bit addresses.
     *
     * Example:
     *
     * address = 0x70
     *
     * transmitted address byte =
     *
     * 0x70 << 1 = 0xE0
     */

    I2C1->DR =
        ((uint32_t)address << 1U);


    /*
     * ----------------------------------------------------------
     * WAIT FOR ACK OR NACK
     * ----------------------------------------------------------
     */

    timeout = I2C_SCAN_TIMEOUT;

    while (timeout-- != 0U)
    {
        /*
         * ADDR = slave acknowledged address.
         */
        if ((I2C1->SR1 & I2C_SR1_ADDR) != 0U)
        {
            volatile uint32_t dummy;


            /*
             * Clear ADDR flag by reading
             * SR1 followed by SR2.
             */
            dummy = I2C1->SR1;

            dummy = I2C1->SR2;

            (void)dummy;


            /*
             * End the transaction.
             */
            I2C1->CR1 |= I2C_CR1_STOP;


            return 1U;
        }


        /*
         * AF = acknowledge failure.
         *
         * The slave did not respond.
         */
        if ((I2C1->SR1 & I2C_SR1_AF) != 0U)
        {
            /*
             * Clear acknowledge-failure flag.
             */
            I2C1->SR1 &= ~I2C_SR1_AF;


            /*
             * Release the bus.
             */
            I2C1->CR1 |= I2C_CR1_STOP;


            return 0U;
        }
    }


    /*
     * Timeout.
     */
    I2C1->CR1 |= I2C_CR1_STOP;


    return 0U;
}


/*
 * ==============================================================
 * COMPLETE I2C BUS SCAN
 * ==============================================================
 */

void Test_I2CScanner(void)
{
    uint8_t address;


    /*
     * No motor movement required.
     */
    Motor_Stop();


    /*
     * Reset all debugger results.
     */
    debug_i2c_count = 0U;

    debug_i2c_3c = 0U;
    debug_i2c_68 = 0U;
    debug_i2c_69 = 0U;
    debug_i2c_70 = 0U;


    /*
     * Clear address-result array.
     */
    for (uint8_t i = 0U; i < 16U; i++)
    {
        debug_i2c_found[i] = 0U;
    }


    /*
     * ==========================================================
     * SCAN VALID 7-BIT I2C ADDRESSES
     * ==========================================================
     *
     * 0x00-0x07 are reserved.
     *
     * 0x78-0x7F are also reserved.
     */

    for (address = 0x08U;
         address <= 0x77U;
         address++)
    {
        if (I2C1_ProbeAddress(address) != 0U)
        {
            /*
             * Save found address.
             */
            if (debug_i2c_count < 16U)
            {
                debug_i2c_found[debug_i2c_count] =
                    address;

                debug_i2c_count++;
            }


            /*
             * Record addresses we specifically
             * care about.
             */

            if (address == 0x3CU)
            {
                debug_i2c_3c = 1U;
            }


            if (address == 0x68U)
            {
                debug_i2c_68 = 1U;
            }


            if (address == 0x69U)
            {
                debug_i2c_69 = 1U;
            }


            if (address == 0x70U)
            {
                debug_i2c_70 = 1U;
            }
        }
    }


    /*
     * ==========================================================
     * HOLD RESULTS
     * ==========================================================
     *
     * Stay here so CubeIDE can inspect the variables.
     */

    while (1)
    {
        Motor_Stop();

        __NOP();
    }
}
