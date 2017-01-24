/*******************************************************************************
 * @file sensor_dispatch.h
 * @brief this function set manage the incoming queue object of the task
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SERIAL_DISPATCH_H
#define SERIAL_DISPATCH_H

#include "kernel_common.h"
#include "srvM2M.h"
#include "serial_process.h"


void serial_Dispatch ( void *pvParameters );

#endif

