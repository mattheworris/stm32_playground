/**
  ******************************************************************************
  * @file    SEGGER_RTT_Conf.h
  * @brief   RTT configuration for STM32F407VG
  ******************************************************************************
  */

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

// Number of up buffers (target to host)
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS     (2)

// Number of down buffers (host to target)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS   (2)

// Size of the buffer for terminal output (up buffer 0)
#define BUFFER_SIZE_UP                    (1024)

// Size of the buffer for terminal input (down buffer 0)
#define BUFFER_SIZE_DOWN                  (16)

// Mode for terminal output
#define SEGGER_RTT_MODE_DEFAULT           SEGGER_RTT_MODE_NO_BLOCK_SKIP

#endif /* SEGGER_RTT_CONF_H */
