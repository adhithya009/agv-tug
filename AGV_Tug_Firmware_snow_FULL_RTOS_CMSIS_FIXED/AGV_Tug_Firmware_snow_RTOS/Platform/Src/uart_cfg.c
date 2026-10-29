#include "uart_cfg.h"

#include "stm32f4xx.h"


/*
 * USART1 is connected to APB2.
 *
 * Our APB2 clock = 84 MHz.
 */
#define UART1_CLOCK_HZ 84000000UL


void UART1_Init(uint32_t baudrate)
{
    if (baudrate == 0U)
    {
        baudrate = 9600U;
    }

    /*
     * Enable USART1 peripheral clock.
     */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /*
     * Disable USART while configuring.
     */
    USART1->CR1 &= ~USART_CR1_UE;

    /*
     * 8 data bits.
     */
    USART1->CR1 &= ~USART_CR1_M;

    /*
     * No parity.
     */
    USART1->CR1 &= ~USART_CR1_PCE;

    /*
     * One stop bit.
     */
    USART1->CR2 &= ~USART_CR2_STOP;

    /*
     * Hardware flow control disabled.
     */
    USART1->CR3 &= ~(USART_CR3_CTSE |
                     USART_CR3_RTSE);

    /*
     * For oversampling by 16:
     *
     * BRR is approximately peripheral_clock / baud.
     */
    USART1->BRR =
        (UART1_CLOCK_HZ + (baudrate / 2U)) /
        baudrate;

    /*
     * Enable transmitter and receiver.
     */
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;

    /*
     * Enable USART.
     */
    USART1->CR1 |= USART_CR1_UE;
}


void UART1_WriteByte(uint8_t data)
{
    /*
     * Wait until transmit register is empty.
     */
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
    }

    USART1->DR = data;
}


void UART1_WriteString(const char *text)
{
    if (text == 0)
    {
        return;
    }

    while (*text != '\0')
    {
        UART1_WriteByte((uint8_t)*text);
        text++;
    }
}


uint8_t UART1_ReadByteNonBlocking(uint8_t *data)
{
    if (data == 0)
    {
        return 0U;
    }

    /*
     * RXNE becomes 1 when a received byte is ready.
     */
    if ((USART1->SR & USART_SR_RXNE) == 0U)
    {
        return 0U;
    }

    *data = (uint8_t)(USART1->DR & 0xFFU);

    return 1U;
}
