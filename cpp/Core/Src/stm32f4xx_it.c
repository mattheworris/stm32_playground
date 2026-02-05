/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_it.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  // Save fault information before looping
  volatile uint32_t* cfsr = (volatile uint32_t*)0xE000ED28;   // Configurable Fault Status Register
  volatile uint32_t* hfsr = (volatile uint32_t*)0xE000ED2C;   // HardFault Status Register
  volatile uint32_t* mmfar = (volatile uint32_t*)0xE000ED34;  // MemManage Fault Address Register
  volatile uint32_t* bfar = (volatile uint32_t*)0xE000ED38;   // BusFault Address Register

  volatile uint32_t cfsr_val = *cfsr;
  volatile uint32_t hfsr_val = *hfsr;
  volatile uint32_t mmfar_val = *mmfar;
  volatile uint32_t bfar_val = *bfar;

  // These will show up in debugger
  (void)cfsr_val;
  (void)hfsr_val;
  (void)mmfar_val;
  (void)bfar_val;

  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
