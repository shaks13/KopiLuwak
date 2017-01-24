/*******************************************************************************
 * @file kernel_creation.h
 * @brief creation the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef KERNEL_CREATION_H
#define KERNEL_CREATION_H

#include "kernel_common.h"
#include "kernel_dispatch.h"


/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define KERNEL_STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define KERNEL_TASK_PRIORITY          (configMAX_PRIORITIES -2)

/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/

uint8_t kernel_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue );

#endif

