/*******************************************************************************
 * @file rftask_creation.c
 * @brief creates the task
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "rftask_creation.h"


/***************************************************************************//**
 * @addtogroup RFtask
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Create the RF task
 *
 * @details
 *
 * @param[out] pCreatedTask   pointer to the created queue
 * @param[out] pCreatedQueue
 *   pointer to the created task
 * @return
 * CROSSRFID_UNSPECIFIED_ERROR : the task is not created
 * CROSSRFID_SUCCESSCODE: the task is created
 ******************************************************************************/
uint8_t rftask_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue )
{
uint8_t uReturnvalue =  CROSSRFID_ERROR_UNSPECIFIED;

	if ( pdTRUE == xTaskCreate( rftask_Dispatch ,
								(const char *) "RFTask",
								RFTASK_STACK_SIZE_FOR_TASK,
								NULL,
								RFTASK_TASK_PRIORITY,
								pCreatedTask))
	{
		uReturnvalue = CROSSRFID_SUCCESSCODE;
		(*pCreatedQueue) = xQueueCreate (RFTASK_NBQUEUEITEM, sizeof (Kernel_QueueItem_struct) );
	}

	return uReturnvalue;

}

/** @} (end addtogroup RFtask) */
