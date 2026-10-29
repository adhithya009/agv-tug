#include "i2c_cfg.h"

#include "stm32f4xx.h"


/*
 * Timeout prevents the CPU from hanging forever
 * if an I2C device is disconnected or the bus locks.
 */
#define I2C_TIMEOUT 100000U


/*
 * --------------------------------------------------------------
 * CLEAR COMMON I2C ERRORS
 * --------------------------------------------------------------
 */
static void I2C1_ClearErrors(void)
{
    /*
     * Clear:
     *
     * BERR = bus error
     * ARLO = arbitration lost
     * AF   = acknowledge failure
     * OVR  = overrun
     */
    I2C1->SR1 &= ~(I2C_SR1_BERR |
                   I2C_SR1_ARLO |
                   I2C_SR1_AF   |
                   I2C_SR1_OVR);
}


/*
 * --------------------------------------------------------------
 * ABORT CURRENT TRANSACTION
 * --------------------------------------------------------------
 */
static void I2C1_Abort(void)
{
    /*
     * Generate STOP so the bus is released.
     */
    I2C1->CR1 |= I2C_CR1_STOP;

    /*
     * Restore normal receive configuration.
     */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 &= ~I2C_CR1_POS;

    I2C1_ClearErrors();
}


/*
 * --------------------------------------------------------------
 * WAIT FOR SR1 FLAG
 * --------------------------------------------------------------
 */
static uint8_t I2C1_WaitSR1(uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((I2C1->SR1 & flag) == 0U)
    {
        /*
         * Detect communication errors while waiting.
         */
        if ((I2C1->SR1 &
             (I2C_SR1_BERR |
              I2C_SR1_ARLO |
              I2C_SR1_AF   |
              I2C_SR1_OVR)) != 0U)
        {
            I2C1_Abort();
            return 0U;
        }

        if (timeout == 0U)
        {
            I2C1_Abort();
            return 0U;
        }

        timeout--;
    }

    return 1U;
}


/*
 * --------------------------------------------------------------
 * WAIT UNTIL BUS BECOMES FREE
 * --------------------------------------------------------------
 */
static uint8_t I2C1_WaitBusFree(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U)
    {
        if (timeout == 0U)
        {
            return 0U;
        }

        timeout--;
    }

    return 1U;
}


/*
 * --------------------------------------------------------------
 * CLEAR ADDR FLAG
 * --------------------------------------------------------------
 *
 * STM32 requires SR1 followed by SR2 read.
 */
static void I2C1_ClearADDR(void)
{
    volatile uint32_t dummy;

    dummy = I2C1->SR1;
    dummy = I2C1->SR2;

    (void)dummy;
}


/*
 * --------------------------------------------------------------
 * I2C1 INITIALIZATION
 * --------------------------------------------------------------
 */
void I2C1_Init(void)
{
    /*
     * Enable I2C1 peripheral clock.
     */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;


    /*
     * Reset I2C1 into a known state.
     */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;


    /*
     * Disable peripheral while configuring.
     */
    I2C1->CR1 &= ~I2C_CR1_PE;


    /*
     * APB1 peripheral clock is 42 MHz.
     *
     * CR2 FREQ must contain APB1 frequency
     * expressed in MHz.
     */
    I2C1->CR2 &= ~I2C_CR2_FREQ;
    I2C1->CR2 |= 42U;


    /*
     * Standard-mode I2C:
     *
     * Fscl = PCLK1 / (2 * CCR)
     *
     * CCR = 42 MHz / (2 * 100 kHz)
     *     = 210
     */
    I2C1->CCR = 210U;


    /*
     * Standard-mode maximum rise time:
     *
     * TRISE = FREQ + 1
     *       = 42 + 1
     *       = 43
     */
    I2C1->TRISE = 43U;


    /*
     * Bit 14 of OAR1 must remain set.
     *
     * We operate only as I2C master, so our own
     * slave address is irrelevant.
     */
    I2C1->OAR1 = (1U << 14);


    /*
     * Enable ACK for normal multi-byte reception.
     */
    I2C1->CR1 |= I2C_CR1_ACK;


    /*
     * Enable I2C1.
     */
    I2C1->CR1 |= I2C_CR1_PE;
}


/*
 * --------------------------------------------------------------
 * WRITE ONE REGISTER
 * --------------------------------------------------------------
 */
uint8_t I2C1_WriteReg(uint8_t dev_addr,
                      uint8_t reg_addr,
                      uint8_t data)
{
    /*
     * Wait until previous bus transaction is finished.
     */
    if (!I2C1_WaitBusFree())
    {
        return 0U;
    }


    /*
     * Generate START condition.
     */
    I2C1->CR1 |= I2C_CR1_START;

    if (!I2C1_WaitSR1(I2C_SR1_SB))
    {
        return 0U;
    }


    /*
     * Send 7-bit address followed by WRITE bit = 0.
     */
    I2C1->DR = ((uint32_t)dev_addr << 1U);

    if (!I2C1_WaitSR1(I2C_SR1_ADDR))
    {
        return 0U;
    }

    I2C1_ClearADDR();


    /*
     * Wait until data register can accept register address.
     */
    if (!I2C1_WaitSR1(I2C_SR1_TXE))
    {
        return 0U;
    }

    I2C1->DR = reg_addr;


    /*
     * Send register data.
     */
    if (!I2C1_WaitSR1(I2C_SR1_TXE))
    {
        return 0U;
    }

    I2C1->DR = data;


    /*
     * BTF confirms both bytes have completely left
     * the peripheral.
     */
    if (!I2C1_WaitSR1(I2C_SR1_BTF))
    {
        return 0U;
    }


    /*
     * End transaction.
     */
    I2C1->CR1 |= I2C_CR1_STOP;

    return 1U;
}


/*
 * --------------------------------------------------------------
 * READ MULTIPLE REGISTERS
 * --------------------------------------------------------------
 */
uint8_t I2C1_ReadRegs(uint8_t dev_addr,
                      uint8_t start_reg,
                      uint8_t *buffer,
                      uint16_t length)
{
    if ((buffer == 0) ||
        (length == 0U))
    {
        return 0U;
    }


    /*
     * Wait until bus becomes free.
     */
    if (!I2C1_WaitBusFree())
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * PHASE 1:
     * Tell slave which register we want.
     * ----------------------------------------------------------
     */
    I2C1->CR1 |= I2C_CR1_START;

    if (!I2C1_WaitSR1(I2C_SR1_SB))
    {
        return 0U;
    }


    /*
     * Address + WRITE.
     */
    I2C1->DR = ((uint32_t)dev_addr << 1U);

    if (!I2C1_WaitSR1(I2C_SR1_ADDR))
    {
        return 0U;
    }

    I2C1_ClearADDR();


    /*
     * Send starting register address.
     */
    if (!I2C1_WaitSR1(I2C_SR1_TXE))
    {
        return 0U;
    }

    I2C1->DR = start_reg;


    /*
     * Wait until register address is transmitted.
     */
    if (!I2C1_WaitSR1(I2C_SR1_BTF))
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * PHASE 2:
     * Repeated START and switch to READ.
     * ----------------------------------------------------------
     */
    I2C1->CR1 |= I2C_CR1_START;

    if (!I2C1_WaitSR1(I2C_SR1_SB))
    {
        return 0U;
    }


    /*
     * Address + READ bit.
     */
    I2C1->DR =
        (((uint32_t)dev_addr << 1U) | 1U);

    if (!I2C1_WaitSR1(I2C_SR1_ADDR))
    {
        return 0U;
    }


    /*
     * ----------------------------------------------------------
     * ONE BYTE RECEIVE
     * ----------------------------------------------------------
     */
    if (length == 1U)
    {
        /*
         * NACK the single received byte.
         */
        I2C1->CR1 &= ~I2C_CR1_ACK;

        I2C1_ClearADDR();

        /*
         * Request STOP before reading final byte.
         */
        I2C1->CR1 |= I2C_CR1_STOP;

        if (!I2C1_WaitSR1(I2C_SR1_RXNE))
        {
            return 0U;
        }

        buffer[0] =
            (uint8_t)I2C1->DR;

        /*
         * Restore normal ACK behaviour.
         */
        I2C1->CR1 |= I2C_CR1_ACK;

        return 1U;
    }


    /*
     * ----------------------------------------------------------
     * TWO BYTE RECEIVE
     * ----------------------------------------------------------
     */
    if (length == 2U)
    {
        /*
         * POS changes ACK/NACK handling for two-byte receive.
         */
        I2C1->CR1 |= I2C_CR1_POS;

        /*
         * NACK final byte.
         */
        I2C1->CR1 &= ~I2C_CR1_ACK;

        I2C1_ClearADDR();


        /*
         * Wait until both bytes are available.
         */
        if (!I2C1_WaitSR1(I2C_SR1_BTF))
        {
            return 0U;
        }


        /*
         * Generate STOP, then read both bytes.
         */
        I2C1->CR1 |= I2C_CR1_STOP;

        buffer[0] =
            (uint8_t)I2C1->DR;

        buffer[1] =
            (uint8_t)I2C1->DR;


        /*
         * Restore normal configuration.
         */
        I2C1->CR1 &= ~I2C_CR1_POS;
        I2C1->CR1 |= I2C_CR1_ACK;

        return 1U;
    }


    /*
     * ----------------------------------------------------------
     * THREE OR MORE BYTES
     * ----------------------------------------------------------
     */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 &= ~I2C_CR1_POS;

    I2C1_ClearADDR();


    uint16_t index = 0U;
    uint16_t remaining = length;


    /*
     * Read normally until exactly three bytes remain.
     */
    while (remaining > 3U)
    {
        if (!I2C1_WaitSR1(I2C_SR1_RXNE))
        {
            return 0U;
        }

        buffer[index++] =
            (uint8_t)I2C1->DR;

        remaining--;
    }


    /*
     * STM32-specific final-three-byte sequence.
     */
    if (!I2C1_WaitSR1(I2C_SR1_BTF))
    {
        return 0U;
    }


    /*
     * NACK the final byte.
     */
    I2C1->CR1 &= ~I2C_CR1_ACK;


    /*
     * Read first of the final three bytes.
     */
    buffer[index++] =
        (uint8_t)I2C1->DR;

    remaining--;


    /*
     * Wait until last two bytes have arrived.
     */
    if (!I2C1_WaitSR1(I2C_SR1_BTF))
    {
        return 0U;
    }


    /*
     * Generate STOP before reading final two bytes.
     */
    I2C1->CR1 |= I2C_CR1_STOP;


    buffer[index++] =
        (uint8_t)I2C1->DR;

    buffer[index++] =
        (uint8_t)I2C1->DR;


    /*
     * Restore ACK for future transfers.
     */
    I2C1->CR1 |= I2C_CR1_ACK;

    return 1U;
}


/*
 * --------------------------------------------------------------
 * READ ONE REGISTER
 * --------------------------------------------------------------
 */
uint8_t I2C1_ReadReg(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t *data)
{
    return I2C1_ReadRegs(dev_addr,
                         reg_addr,
                         data,
                         1U);
}
