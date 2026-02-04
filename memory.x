MEMORY
{
  /* STM32F407VG: 1MB Flash, 192KB total SRAM */
  /* - 128KB SRAM at 0x2000_0000 (DMA accessible) */
  /* -  64KB CCM  at 0x1000_0000 (CPU only, no DMA) */
  FLASH  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM    : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);