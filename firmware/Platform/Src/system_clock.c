#include "system_clock.h"
#include "stm32f4xx.h"

// CPU frequency after clock configuration.
#define SYSTEM_CLOCK_HZ 84000000UL

void SystemClock_Init(void)
{
    // Enable the internal 16 MHz HSI oscillator.
    // WHY: HSI is our PLL input clock and does not require an external crystal.
    RCC->CR |= RCC_CR_HSION;

    // Wait until the HSI oscillator becomes stable.
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
    {
        // Wait here until hardware reports HSI ready.
    }

    // Enable the power-control peripheral clock.
    // WHY: voltage regulator settings belong to the PWR peripheral.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Configure voltage scaling for high-speed operation.
    // WHY: the regulator must support operation at 84 MHz.
    PWR->CR |= PWR_CR_VOS;

    // Configure Flash before increasing CPU frequency.
    // 2 wait states are required for 84 MHz operation.
    FLASH->ACR =
        FLASH_ACR_ICEN     |   // Enable instruction cache.
        FLASH_ACR_DCEN     |   // Enable data cache.
        FLASH_ACR_PRFTEN   |   // Enable prefetch.
        FLASH_ACR_LATENCY_2WS; // Flash latency = 2 wait states.

    // Disable PLL before changing its configuration.
    RCC->CR &= ~RCC_CR_PLLON;

    // Wait until PLL is completely stopped.
    while ((RCC->CR & RCC_CR_PLLRDY) != 0U)
    {
        // Wait until PLL ready flag clears.
    }

    // Configure PLL:
    // PLL source = HSI = 16 MHz
    // PLLM = 16  -> 16 MHz / 16 = 1 MHz
    // PLLN = 336 -> 1 MHz * 336 = 336 MHz
    // PLLP = 4   -> 336 MHz / 4 = 84 MHz SYSCLK
    // PLLQ = 7   -> 336 MHz / 7 = 48 MHz
    RCC->PLLCFGR =
        (16U  << RCC_PLLCFGR_PLLM_Pos) |
        (336U << RCC_PLLCFGR_PLLN_Pos) |
        (1U   << RCC_PLLCFGR_PLLP_Pos) |
        (7U   << RCC_PLLCFGR_PLLQ_Pos);

    // Configure bus prescalers.
    // AHB  = SYSCLK / 1 = 84 MHz.
    // APB1 = HCLK / 2   = 42 MHz.
    // APB2 = HCLK / 1   = 84 MHz.
    RCC->CFGR &= ~(RCC_CFGR_HPRE |
                   RCC_CFGR_PPRE1 |
                   RCC_CFGR_PPRE2);

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Start the PLL.
    RCC->CR |= RCC_CR_PLLON;

    // Wait until PLL locks to the requested frequency.
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
    {
        // Hardware is stabilizing PLL.
    }

    // Select PLL output as the processor system clock.
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Confirm that PLL really became the system clock.
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        // Wait for clock switch to complete.
    }
}


uint32_t SystemClock_GetFrequency(void)
{
    // Return the frequency configured above.
    return SYSTEM_CLOCK_HZ;
}
