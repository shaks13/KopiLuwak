/*******************************************************************************
 * @file watchdog_process.h
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef WATCHDOG_PROCESS_H_
#define WATCHDOG_PROCESS_H_
#include "em_wdog.h"
#include "bsp.h"

void watchdog_InitWatchdog (void);
void watchdog_TickleWatchdog (void);

#endif /* WATCHDOG_PROCESS_H_ */
