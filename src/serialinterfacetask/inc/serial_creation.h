/*******************************************************************************
 * @file serial_creation.h
 * @brief creation the task
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SERIAL_CREATION_H
#define SERIAL_CREATION_H

#include "kernel_common.h"
#include "serial_dispatch.h"

#define SERIAL_STACK_SIZE_FOR_TASK    	(configMINIMAL_STACK_SIZE + 10)
#define SERIAL_TASK_PRIORITY          	(configMAX_PRIORITIES -1)
#define SERIAL_NBQUEUEITEM				4

uint8_t serial_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue );

#endif

