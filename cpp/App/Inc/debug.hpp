/**
  ******************************************************************************
  * @file    debug.hpp
  * @brief   Debug utilities
  ******************************************************************************
  */

#ifndef DEBUG_HPP
#define DEBUG_HPP

#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstdarg>
#include "rtt_log.h"

/**
 * @brief Simple debug output via semihosting
 */
class Debug {
public:
    static void init();
    static void printf(const char* format, ...);
    static void blink(uint32_t times, uint32_t delay_ms = 200);

    // Error code indicators using LED patterns
    static void error_accelerometer_init();    // 2 blinks
    static void error_accelerometer_read();    // 3 blinks
    static void checkpoint(uint8_t num);       // Fast blinks
};

// Checkpoint macros for quick debugging
#define DEBUG_CHECKPOINT(x) Debug::checkpoint(x)
#define DEBUG_PRINTF(...) rtt_println(__VA_ARGS__)

// Semihosting syscalls for printf
extern "C" {
    void initialise_monitor_handles(void);
}

#endif /* DEBUG_HPP */
