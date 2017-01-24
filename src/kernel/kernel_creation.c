/*******************************************************************************
 * @file Kernel_creation.c
 * @brief creation the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "kernel_creation.h"



/***************************************************************************//**
 * @brief
 *   Create the Kernel task
 *
 * @details
 *
 * @param[in] nonepCreatedTask
 * @param[out] pCreatedTask
 *   pointer to the created task
 * @return
 * CROSSRFID_UNSPECIFIED_ERROR : the task is not created
 * CROSSRFID_SUCCESSCODE: the task is created
 ******************************************************************************/
uint8_t kernel_CreateTask ( TaskHandle_t * pCreatedTask, QueueHandle_t * pCreatedQueue )
{
Kernel_TaskParams_t parametersToTask = { 500 / portTICK_RATE_MS };
uint8_t uReturnvalue =  CROSSRFID_ERROR_UNSPECIFIED;

#if 0
	/* Macro that creates a mutex semaphore  */
	xMutexFidjiMemory = xSemaphoreCreateMutex();

	if(NULL == xMutexFidjiMemory)
	{
		/* Todo: The semaphore can't be used */
	}
#endif

	if ( pdTRUE == xTaskCreate( kernel_Dispatch ,
				(const char *) "KernelProcess",
				KERNEL_STACK_SIZE_FOR_TASK,
				&parametersToTask,
				KERNEL_TASK_PRIORITY,
				pCreatedTask))
	{
		uReturnvalue = CROSSRFID_SUCCESSCODE;
		(*pCreatedQueue) = xQueueCreate (KERNEL_NBQUEUEITEM, sizeof (Kernel_QueueItem_struct) );
	}

	watchdog_InitWatchdog (); /* the init should be done here because the dog is tickled by the kernel process*/ /* initialize and enable the watchdog*/

	return uReturnvalue;

}
