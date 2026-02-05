/**
  ******************************************************************************
  * @file    debug.cpp
  * @brief   Debug utilities implementation
  ******************************************************************************
  */

#include "debug.hpp"
#include <cstdio>
#include <cstdarg>

// Use the onboard user LEDs on Discovery board
// LD3 (Orange) = PD13, LD4 (Green) = PD12, LD5 (Red) = PD14, LD6 (Blue) = PD15
// These are the same pins as our PWM LEDs, so we'll use them for debug blinking

void Debug::init()
{
    // LEDs already initialized in MX_GPIO_Init
    // If using semihosting, initialize it
    #ifdef DEBUG_SEMIHOSTING
    initialise_monitor_handles();
    #endif
}

void Debug::printf(const char* format, ...)
{
    #ifdef DEBUG_SEMIHOSTING
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Output via semihosting
    puts(buffer);
    #else
    (void)format;  // Suppress unused warning
    #endif
}

void Debug::blink(uint32_t times, uint32_t delay_ms)
{
    // Use PD13 (North LED / LD3 Orange on Discovery)
    for (uint32_t i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(delay_ms);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(delay_ms);
    }
}

void Debug::error_accelerometer_init()
{
    // 2 long blinks = accelerometer init failed
    while (1) {
        blink(2, 500);
        HAL_Delay(1000);
    }
}

void Debug::error_accelerometer_read()
{
    // 3 short blinks = accelerometer read failed
    while (1) {
        blink(3, 200);
        HAL_Delay(1000);
    }
}

void Debug::checkpoint(uint8_t num)
{
    // Fast blinks to indicate checkpoint number
    blink(num, 100);
    HAL_Delay(500);
}

// Semihosting support
#ifdef DEBUG_SEMIHOSTING
extern "C" {
    void initialise_monitor_handles(void) {
        // Empty stub - linker will provide real implementation if using --specs=rdimon.specs
    }
}
#endif
