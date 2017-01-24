/*******************************************************************************
 * @file sensor_creation.h
 * @brief creation the task
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SENSOR_CREATION_H
#define SENSOR_CREATION_H

#include "kernel_common.h"

#include "sensor_dispatch.h"

#define SENSOR_STACK_SIZE_FOR_TASK    	(configMINIMAL_STACK_SIZE + 10)
#define SENSOR_TASK_PRIORITY          	(configMAX_PRIORITIES -1)
#define SENSOR_NBQUEUEITEM				4

uint8_t sensor_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue );

#endif

