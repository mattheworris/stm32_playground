/**
  ******************************************************************************
  * @file    rtt_log.h
  * @brief   Simple RTT logging wrapper (like defmt for Rust)
  ******************************************************************************
  */

#ifndef RTT_LOG_H
#define RTT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SEGGER_RTT.h"

/**
 * @brief Initialize RTT (call once at startup)
 */
static inline void rtt_init(void) {
    SEGGER_RTT_Init();
}

/**
 * @brief Print formatted string to RTT (like defmt::println!)
 * Usage: rtt_println("Value: %d", x);
 */
#define rtt_println(fmt, ...) SEGGER_RTT_printf(0, fmt "\n", ##__VA_ARGS__)

/**
 * @brief Print without newline
 */
#define rtt_print(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* RTT_LOG_H */
