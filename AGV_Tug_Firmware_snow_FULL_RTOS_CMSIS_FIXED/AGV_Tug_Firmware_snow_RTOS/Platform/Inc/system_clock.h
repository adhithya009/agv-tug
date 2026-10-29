#ifndef SYSTEM_CLOCK_H
#define SYSTEM_CLOCK_H

#include <stdint.h>

// Configures the STM32F401RE system clock to 84 MHz.
void SystemClock_Init(void);

// Returns the configured CPU clock frequency.
uint32_t SystemClock_GetFrequency(void);

#endif
