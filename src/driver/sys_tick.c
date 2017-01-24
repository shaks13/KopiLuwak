/*******************************************************************************
 * @file sys_tick.c
 * @brief the file contains the systick functions
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "sys_tick.h"


volatile uint32_t msTicks = 0; /* counts 1ms timeTicks */
extern void  xPortSysTickHandler (void);

void init_systick (void)
{
	if (SysTick_Config(SystemCoreClockGet() / 1000)) while (1) ;
}

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/

void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
  xPortSysTickHandler ();
}

void disable_sys_tick()
{
	SysTick->CTRL &=0xFFFE;
}

void enable_sys_tick()
{
	SysTick->CTRL |= 1;
}
