#include "spi_cfg.h"

#include "stm32f4xx.h"


void SPI1_Init(void)
{
    /*
     * Enable SPI1 peripheral clock.
     */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /*
     * Disable SPI before configuration.
     */
    SPI1->CR1 &= ~SPI_CR1_SPE;

    /*
     * Master mode.
     */
    SPI1->CR1 |= SPI_CR1_MSTR;

    /*
     * SPI mode 0:
     *
     * CPOL = 0
     * CPHA = 0
     */
    SPI1->CR1 &= ~SPI_CR1_CPOL;
    SPI1->CR1 &= ~SPI_CR1_CPHA;

    /*
     * APB2 = 84 MHz.
     *
     * Prescaler /64:
     *
     * 84 MHz / 64
     * = 1.3125 MHz.
     *
     * Suitable for MFRC522.
     */
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_BR_2 |
                 SPI_CR1_BR_0;

    /*
     * 8-bit frame.
     */
    SPI1->CR1 &= ~SPI_CR1_DFF;

    /*
     * MSB first.
     */
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

    /*
     * Software slave management.
     *
     * Actual RFID CS is PB10 controlled manually.
     */
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;

    /*
     * Enable SPI1.
     */
    SPI1->CR1 |= SPI_CR1_SPE;
}


uint8_t SPI1_TransferByte(uint8_t data)
{
    /*
     * Wait until transmit buffer is empty.
     */
    while ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
    }

    /*
     * Write one byte.
     */
    *((volatile uint8_t *)&SPI1->DR) = data;

    /*
     * Wait until response arrives.
     */
    while ((SPI1->SR & SPI_SR_RXNE) == 0U)
    {
    }

    return *((volatile uint8_t *)&SPI1->DR);
}
