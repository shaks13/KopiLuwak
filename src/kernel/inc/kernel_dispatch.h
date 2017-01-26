/*******************************************************************************
 * @file kernel_dispatch.h
 * @brief creation the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef KERNEL_DISPATCH_H
#define KERNEL_DISPATCH_H

#include "kernel_common.h"
#include "kernel_process.h"
#include "watchdog_process.h"



/*===========================================================================================================
						Enum definition
===========================================================================================================*/



/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/
void kernel_Dispatch ( void *pvParameters );



#endif

