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

#ifndef SENSOR_DISPATCH_H
#define SENSOR_DISPATCH_H

#include "kernel_common.h"
#include "timerdrv.h"
#include "srvLIS2DH12.h"
#include "srvM2M.h"

#include "sensor_process.h"


void sensor_Dispatch ( void *pvParameters );

#endif

