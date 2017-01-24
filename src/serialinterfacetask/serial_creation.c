/*******************************************************************************
 * @file serial_creation.c
 * @brief creation the task
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "serial_creation.h"


/***************************************************************************//**
 * @brief
 *   Create the sensor interface task
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
uint8_t serial_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue )
{
uint8_t uReturnvalue =  CROSSRFID_ERROR_UNSPECIFIED;

#if USEM2MINTERFACE==1
	if ( pdTRUE == xTaskCreate( serial_Dispatch ,
								(const char *) "SerialTask",
								SERIAL_STACK_SIZE_FOR_TASK,
								NULL,
								SERIAL_TASK_PRIORITY,
								pCreatedTask))
	{
		uReturnvalue = CROSSRFID_SUCCESSCODE;
		(*pCreatedQueue) = xQueueCreate (SERIAL_NBQUEUEITEM, sizeof (Kernel_QueueItem_struct) );
	}
#else
	*pCreatedTask = NULL;
	*pCreatedQueue = NULL;
#endif

	return uReturnvalue;

}



/** @} (end addtogroup SerialInterfaceTask) */
