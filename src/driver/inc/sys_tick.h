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
#include "em_system.h"

#ifndef SYS_TICK_H
#define SYS_TICK_H

extern volatile uint32_t msTicks; /* counts 1ms timeTicks */

/***************************************************************************//**
 * @brief
 *   Initializes systick at 1 ms.
 ******************************************************************************/
extern void init_systick (void);

extern void enable_sys_tick();
extern void disable_sys_tick();
#endif

