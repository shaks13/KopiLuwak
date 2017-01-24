/*******************************************************************************
 * @file rftask_creation.h
 * @brief creates the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef RFTASK_CREATION_H
#define RFTASK_CREATION_H

#include "kernel_common.h"
#include "rftask_dispatch.h"
#include "interface_spi.h"

#define RFTASK_STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define RFTASK_TASK_PRIORITY          (configMAX_PRIORITIES -1)
#define RFTASK_NBQUEUEITEM				4

uint8_t rftask_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue );

#endif

