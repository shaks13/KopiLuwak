/*******************************************************************************
 * @file kernel_process.c
 * @brief the file contain the command set for the kernel
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "kernel_process.h"


/*===========================================================================================================
						public variable declaration
===========================================================================================================*/
SemaphoreHandle_t  xSemaphore = NULL;
SemaphoreHandle_t  xTimer0Semaphore = NULL;

#if (1 == USE_TEMPERATURE_EM4325)
SemaphoreHandle_t  xBinarySemphrEm4325 = NULL; /* protect the access to the EM4325. It could be used either as a RF front end and as temperature sensor*/
#endif

/*===========================================================================================================
						Private variable declaration
===========================================================================================================*/
static uint32_t ui32date;		/* use to save the board date and to be shared with the others tasks*/
static uint32_t ui32time;		/* use to save the board time and to be shared with the others tasks*/

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/* This system file contains the last shelf life computed and the state of the shelf life calculation */
static Kernel_GluePot_struct kernel_sGPGiveMeLife;
#endif

/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
static void 	kernel_FlushQueues			( void );
static uint8_t 	kernel_processWaitTimer0 	( Kernel_QueueItem_struct *pQueueItems );
/*===========================================================================================================
						Private functions definition
===========================================================================================================*/

/***************************************************************************//**
 * @brief		this function checks is the semaphore has been released by the
 * timer. When it does the sleep mode is allowed.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED: a message is posted to the serial task
 * because the UART doesn't work in EM2 and the LEUART in EM3
 ******************************************************************************/
static uint8_t kernel_processWaitTimer0 ( Kernel_QueueItem_struct *pQueueItems )
{
uint8_t status = pdFALSE;

	do {
		status = xSemaphoreTake(xTimer0Semaphore,100) ;
		if (pdTRUE == status )
		{
			kernel_AllowSleepInDefault ();		/* authorize the sleep mode */
			vSemaphoreDelete (xTimer0Semaphore);
			xTimer0Semaphore= NULL;
		}else {/* do nothing*/}

	}while (status != pdTRUE);

	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_KERNELTASKID);
	pQueueItems->ui16notification = KERNEL_GOTOSLEEPMODE;
	pQueueItems->pData = NULL;
	pQueueItems->ui16NbByte = 0;

	return ui8status;
}
/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief  This function processes the initialization of Kernel task
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 ******************************************************************************/
uint8_t kernel_ProcessInit (  Kernel_QueueItem_struct *pQueueItems  )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;


	/* Serial task initialization successful, allows the uc to enter in EM2 */
	kernel_initSleepInDefault();

	/* Initializes the calendar service */
	srvCalendar_Init();

	/* Initializes the cryotimer */
	//cryodrv_Init();

#if APP_ACTIVITYCOUNTER // APP_GLUEPOT
	/* creates a binary semaphore to protect the EM4325 access (SPI) */
	xBinarySemphrEm4325 = xSemaphoreCreateBinary();
#endif

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	/* initial glue pot GiveMeLife system file value*/
	kernel_sGPGiveMeLife.dShelfLife = (double)(1.0f);
	kernel_sGPGiveMeLife.ui16State = 0;

	srvShelfLife_ResetShelfLife();

	kernel_sGPGiveMeLife.ui16State = KERNEL_LIFE_STOP;
#endif

#if (1 == BACKUP_RESTORE_FLASH)
	/* Initializes the NVM module */
	srvFlashMemory_Init();
#endif


	//Kernel_QueueItem_struct sQueueItem;
	pQueueItems->ui16NbByte = 0;
	pQueueItems->pData = NULL;
	pQueueItems->ui16notification = KERNEL_MESSAGEID_TASKINIT_YOURSELF;

	if (sKernel_QueuePointer.pSerialQueue != NULL) /* if the task exists*/
	{
		/* Sends a message to Serial, initialization still in progress */
		pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_KERNELTASKID);
	}
	else
	{
		/* Sends a message to Serial, initialization still in progress */
		pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	}

	return ui8status;
}

/***************************************************************************//**
 * @brief This function process a reset
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
******************************************************************************/
static void kernel_FlushQueues( void )
{
UBaseType_t ui8NbMeassageWaiting =0;
QueueHandle_t QueueHandle ;
uint8_t ui8NthQueue =0 ;
uint8_t ui8NbQueue = sizeof (Kernel_QueuePointer_struct) / sizeof (QueueHandle_t) ;
Kernel_QueueItem_struct pQueueItem;

	for (ui8NthQueue=0; ui8NthQueue <ui8NbQueue ; ui8NthQueue++)
	{
		//QueueHandle = (&sKernel_QueuePointer + ui8NthQueue * sizeof (QueueHandle_t));
		if (ui8NthQueue == 0)
		{
			QueueHandle=sKernel_QueuePointer.pKernelQueue;
		}else if (ui8NthQueue == 1)
		{
			QueueHandle=sKernel_QueuePointer.pSensorQueue;
		}
		else
		{
			QueueHandle=sKernel_QueuePointer.pRFtaskQueue;
		}

		ui8NbMeassageWaiting = uxQueueMessagesWaiting (QueueHandle);
		while (ui8NbMeassageWaiting != 0)
		{

			xQueueReceive (QueueHandle,		/* handle of the task's queue*/
							&pQueueItem,							/* pointer of the object*/
							0 );
			ui8NbMeassageWaiting = uxQueueMessagesWaiting (QueueHandle);
		}
	}
}
/***************************************************************************//**
 * @brief 		This function process a reset of the FW
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 * to an other task
******************************************************************************/
uint8_t kernel_ProcessReset ( Kernel_QueueItem_struct *psQueueItem )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;


	NVIC_DisableIRQ(LETIMER0_IRQn);
	srvCalendar_DisableMeasureCCxIrq ();

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	/* disable the rtcc calendar irq which trigs the life computing */
	srvCalendar_DisableMinutLifeComputing();

	/* resets the life value */
	kernel_sGPGiveMeLife.dShelfLife = (double)(1.0f);
	kernel_sGPGiveMeLife.ui16State = 0;

	srvShelfLife_ResetShelfLife();

	kernel_sGPGiveMeLife.ui16State = KERNEL_LIFE_STOP;
#endif

	/*flush the queues*/
	kernel_FlushQueues ();

	/* Sends a message to RF task, initialization still in progress */
	psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	psQueueItem->ui16notification = KERNEL_MESSAGEID_RESET;
	psQueueItem->ui16NbByte = 0;
	psQueueItem->pData = NULL;

	/* Serial task initialization successful, allows the �C to enter in EM2 */
	kernel_AllowSleepInDefault();


	return ui8status;
}
/***************************************************************************//**
 * @brief 		This function sends a message to initialize.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void kernel_PostInitMessage ( void )
{
Kernel_QueueItem_struct sQueueItem = {	KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_KERNELTASKID),
										KERNEL_MESSAGEID_TASKINIT_YOURSELF,
										0, NULL };

	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
}

/***************************************************************************//**
 * @brief This function allows the �C to enter in EM1 mode
 * @param[in]	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void kernel_AllowSleepInEM1 ( void )
{
	/* Lets now the uc enter in EM1 */
	//sleep_SleepBlockEnd(configSLEEP_MODE+1);
	sleep_SleepBlockBegin((sleepEM1+1));
}

/***************************************************************************//**
 * @brief 		This function allows the �C to enter in the default mode
 * @param[in]	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void kernel_initSleepInDefault ( void )
{
	sleep_Init(&sleep_sleepEMxManagement, &sleep_wakeUpEMxManagement); 	/* Initialize SLEEP driver */
	sleep_SleepBlockBegin((configSLEEP_MODE+1));
}

/***************************************************************************//**
 * @brief 		This function allows the �C to enter in the default mode
 * @param[in]	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void kernel_AllowSleepInDefault ( void )
{
	/* Lets now the uc enter in EM2 */
	//sleep_SleepBlockEnd(sleepEM0+1);
	sleep_SleepBlockEnd(sleepEM1+1);
	//sleep_SleepBlockBegin((configSLEEP_MODE+1));
}


/***************************************************************************//**
 * @brief 		This function process the end of the Wake up mode . the wake
 * up mode is EM0 or EM1
 * @note		some peripheral requires to work in EM0 or EM1 like the
 * timers and the USART by e.g.. the kernel can disabled the sleep mode
 * for a given time until the peripheral has terminated their process.
 * @details
 * @param[in,out] pQueueItems : the received notification
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 ******************************************************************************/
uint8_t kernel_processEndOfWakeUpMode( Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;
/* the message aim to initialize the rf task and the spi dedicated to the EM4355 */
//Kernel_QueueItem_struct sQueueItem = {		KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_KERNELTASKID),
//											KERNEL_GOTOSLEEPMODE,
//											0,NULL};

	pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_KERNELTASKID);
	pQueueItems->ui16notification = KERNEL_GOTOSLEEPMODE;
	pQueueItems->pData = NULL;
	pQueueItems->ui16NbByte = 0;
	//xQueueSend (sKernel_QueuePointer.pSensorQueue,&sQueueItem,0);

	return ui8status;
}

/***************************************************************************//**
 * @brief 		This function updates the power management because some peripheral
 * required to stay in EM0 or EM1
 * @param[in,out] pQueueItems : the received notification
 * @return 			CROSSRFID_SUCCESSCODE :  the function is successful
 ******************************************************************************/
uint8_t kernel_processStartWakeUpMode (Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	kernel_AllowSleepInEM1 ();
	xTimer0Semaphore = xSemaphoreCreateBinary(); /* create the semaphore used for the synchronization between DataReady Irq */
	timer_inittimer0 ( 5* TIMER_WAIT1S);  /* launch the timer 0 for 1s */
	ui8status = kernel_processWaitTimer0 (pQueueItems);

	return ui8status;

}


/***************************************************************************//**
 * @brief This function reads the time of the RTCC and sends the information to
 *   the RF host
 * @details
 * @param[in] 	none
 * @param[out] 	psQueueItem : the queue item to be posted
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message to be posted
 ******************************************************************************/
uint8_t kernel_processGetTime ( Kernel_QueueItem_struct * psQueueItems )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED; /* the message aim to initialize the rf task and the spi dedicated to the EM4355 */



	 psQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	 psQueueItems->ui16notification = KERNEL_MESSAGEID_GETTIME;
	 psQueueItems->ui16NbByte = 4;
	/* read the current time*/
	srvrtcc_GetTime (& ui32time);
	psQueueItems->pData = (uint8_t *) (&ui32time);
	return ui8status ;

}

 /***************************************************************************//**
  * @brief This function writes the time of the RTCC and sends the ack to
  *   the RF host
  * @details
  * @param[in] 	pui8time :  pointer on the time to write. the format is the folowing
  * |-----------------------|-----------------------|
  * |	byte 0 (4 MSbits)	|	byte 0 (4 LSbits )	|
  * |		second tens		|		second unit		|
  * |-----------------------|-----------------------|
  * |	byte 1 (4 MSbits)	|	byte 1 (4 LSbits )	|
  * |		minute tens		|		minute unit		|
  * |-----------------------|-----------------------|
  * |	byte 2 (4 MSbits)	|	byte 2 (4 LSbits )	|
  * |		hours tens		|		hours unit		|
  * |-----------------------|-----------------------|
  * @param[out] 	psQueueItem : the queue item to be posted
  * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message to be posted
  ******************************************************************************/
uint8_t kernel_processSetTime ( Kernel_QueueItem_struct * psQueueItem )
 {
 /* the message aim to initialize the rf task and the spi dedicated to the EM4355 */
 //Kernel_QueueItem_struct sQueueItem = {	//(((KERNEL_RFFRONTENDTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the rf task */
 //										// ((KERNEL_KERNELTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,			/* the sender is the kernel end */
//		 	 	 	 	 	 	 	 	 	KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID),
// 											KERNEL_MESSAGEID_SETTIME,
// 											0,NULL}; /* the time is coded on 4 bytes */
uint8_t	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
uint8_t ui8second =  *psQueueItem->pData ;
uint8_t ui8minutes = *((psQueueItem->pData)+1);
uint8_t ui8hours = *((psQueueItem->pData)+2);

	psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	psQueueItem->ui16notification = KERNEL_MESSAGEID_SETTIME;
	psQueueItem->ui16NbByte = 0;
	/* write the current time*/
	srvCalendar_SetTime(ui8second,ui8minutes,ui8hours);
	/* send the acknowledge*/
	//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItem,0);
	return ui8status;
 }

 /***************************************************************************//**
  * @brief This function reads the date of the RTCC and sends the information to
  *   the RF host
  * @details
  * @param[in]		none
  * @param[out] 	psQueueItem : the queue item to be posted
  * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message to be posted
  ******************************************************************************/
uint8_t kernel_processGetDate ( Kernel_QueueItem_struct * psQueueItems )
{
uint8_t	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
 /* the message aim to initialize the rf task and the spi dedicated to the EM4355 */
// Kernel_QueueItem_struct sQueueItem = {	//(((KERNEL_RFFRONTENDTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the rf task */
// 										// ((KERNEL_KERNELTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,			/* the sender is the kernel end */
//		 	 	 	 	 	 	 	 	 	KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID),
//		 	 	 	 	 	 	 	 	 	KERNEL_MESSAGEID_GETDATE,
// 											4,NULL};  /* the time is coded on 4 bytes */

	psQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	psQueueItems->ui16notification = KERNEL_MESSAGEID_GETDATE;
	psQueueItems->ui16NbByte = 4;
 	/* read the current time*/
 	srvrtcc_GetDate (& ui32date);
 	psQueueItems->pData = (uint8_t *) (&ui32date);
 	//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItem,0);
 	return ui8status;

}

 /***************************************************************************//**
  * @brief This function writes the date of the RTCC and sends the ack to
  *   the RF host
  * @details
  * @param[in] 	pui8date :  pointer on the time to write. the format is the folowing
  * |-----------------------|-----------------------|
  * |	byte 0 (4 MSbits)	|	byte 0 (4 LSbits )	|
  * |		day tens		|		day unit		|
  * |-----------------------|-----------------------|
  * |	byte 1 (4 MSbits)	|	byte 1 (4 LSbits )	|
  * |		month tens		|		month unit		|
  * |-----------------------|-----------------------|
  * |	byte 2 (4 MSbits)	|	byte 2 (4 LSbits )	|
  * |		year tens		|		year unit		|
  * |-----------------------|-----------------------|
  * @param[out] 	psQueueItem : the queue item to be posted
  * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message to be posted
  ******************************************************************************/
uint8_t kernel_processSetDate ( Kernel_QueueItem_struct *pQueueItem )
 {
 /* the message aim to initialize the rf task and the spi dedicated to the EM4355 */
 //Kernel_QueueItem_struct sQueueItem = {	 	KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID),
//		 	 	 	 	 	 	 	 	 	KERNEL_MESSAGEID_SETDATE,
// 											0,NULL}; /* the time is coded on 4 bytes */
uint8_t	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
uint8_t ui8day =  *(pQueueItem->pData) ;
uint8_t ui8month = *((pQueueItem->pData)+1);
uint8_t ui8year = *((pQueueItem->pData)+2);


	pQueueItem->ui16notification = KERNEL_MESSAGEID_SETDATE;
	pQueueItem->ui16NbByte = 0;
	pQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	/* write the current time*/
	srvCalendar_SetDate ( ui8day, ui8month ,ui8year );
	/* send the acknowledge*/
	//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItem,0);
	return ui8status;
 }

 /***************************************************************************//**
 * @brief This function gets the age of the board and sends it to the Rf task
 * @param[in]	none
 * @param[out] 	psQueueItem : the queue item to be posted
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message to be posted
  ******************************************************************************/
uint8_t kernel_processGetAge ( Kernel_QueueItem_struct * psQueueItem )
{
uint8_t  ui8status = CROSSRFID_MESSAGETOBEPOSTED;
uint16_t *pui16BoardAge;
/* the message aim to initialize the rf task and the spi dedicated to the EM4355 */
//Kernel_QueueItem_struct sQueueItem = {		KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID),
//											KERNEL_MESSAGEID_GETAGE,
//											2,NULL};  /* the time is coded on 2 bytes */

	psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	psQueueItem->ui16notification = KERNEL_MESSAGEID_GETAGE;
	psQueueItem->ui16NbByte = 2;
	srvCalendar_GetBoardAge (&pui16BoardAge);
	psQueueItem->pData = (uint8_t *) (pui16BoardAge);
 	//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItem, pdFALSE);
	return ui8status;
}


#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#if 0
/* code used if we want to not save the temperature objects (in ram, nvm_config.c)
 * directly in the service measurement called by the serial task after a logical timer irq
 * (see srvtemp_MeasureTemperature) */
/***************************************************************************//**
* @brief This function updates the temperature buffer in flash and the number
* 		 of samples available.
* @param[in]	none
* @param[out] 	sQueueItem: the queue item posted
* @return 		none
 ******************************************************************************/
void kernel_UpdateTempObjectsInFlash ( Kernel_QueueItem_struct sQueueItem )
{
	uint16_t 		ui16NbMeasurement;
	int16_t * 		pi16buffer;

	/* retrieves the last stored sample in ram */
	srvtemp_GetNbMeas (&ui16NbMeasurement);
	srvtemp_GetBufferTemp ((int8_t **)&pi16buffer);

	/* updates the number of samples */
	srvFlashMemory_SetNbTempSamples(ui16NbMeasurement);

	/* updates the flash buffer to get ready for the periodic backup */
	srvFlashMemory_SaveOneTempSample((ui16NbMeasurement-1), pi16buffer[ui16NbMeasurement-1]);
}
#endif
/***************************************************************************//**
* @brief 		This function writes the temperature samples stored in RAM and
* 				the number of temperature samples available into the flash memory.
* @param[in]	none
* @param[out] 	none
* @return 		none
 ******************************************************************************/
void kernel_BackUpTempObjectsInFlash ( void )
{
	srvFlashMemory_WriteAllTempObjects();
}

/***************************************************************************//**
* @brief 		This function restores the sensor buffers and the number of
* 				samples	available.
*
* @details		It reads the data in flash and fills the sensor
* 				buffers in RAM according to the FLASH memory mapping.
*
* @param[in]	none
* @param[out] 	none
* @return 		CROSSRFID_SUCCESSCODE: successful read operation
* @return 		CROSSRFID_ERROR_NVM_READ: read operation failed
* @return 		CROSSRFID_ERROR_NVM_ISERASE: no read operation, the flash is empty
 ******************************************************************************/
uint8_t kernel_RestoreFlashToRam ( void )
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;

	ui8Status = srvtemp_RestoreFlashToRamObjects();

	return ui8Status;
}

/***************************************************************************//**
* @brief
* 		Computes the shelf life after having received a calendar event.
*
* @param[in]	psQueueItems: the message to send to serial if the life has expired
*
 * @return 		CROSSRFID_SUCCESSCODE: shelf life successfully computed
 * @return 		CROSSRFID_GP_ERROR_LIFE_COMPUT: error during the life calculation due
 * 				to the temperature acquisitions
 * @return 		CROSSRFID_MESSAGETOBEPOSTED: message to serial
 ******************************************************************************/
uint8_t kernel_ProcessShelfLife ( Kernel_QueueItem_struct * psQueueItems )
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;

	ui8Status = srvShelfLife_ComputeShelfLife();

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		kernel_sGPGiveMeLife.dShelfLife = srvShelfLife_GetShelfLife();
	}
	/* the life has expired */
	else if (CROSSRFID_GP_END_LIFE == ui8Status)
	{
		kernel_sGPGiveMeLife.dShelfLife = srvShelfLife_GetShelfLife();

		/* stops the life calculation */
		srvCalendar_DisableMinutLifeComputing();
		/* life calculation is no longer in progress */
		kernel_sGPGiveMeLife.ui16State = KERNEL_LIFE_STOP;

		/* fills the message */
		psQueueItems->ui16NbByte = 0;
		psQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_KERNELTASKID);
		psQueueItems->ui16notification = KERNEL_MESSAGEID_STOP_ACQUISITIONS;

		ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else
	{
#if (GLUEPOT_DEBUG == 1)
		/* Do not stop the irq for the test */
#else
		/* stops the life calculation */
		srvCalendar_DisableMinutLifeComputing();

		kernel_sGPGiveMeLife.ui16State = KERNEL_LIFE_STOP;
#endif
	}

	return ui8Status;
}

/***************************************************************************//**
* @brief		Enable/Disable the shelf life computing.
*
* @details		Updates the calendar irq according to the ACQ MODE sensor configuration.
*
* @param[in] 	sQueue: the serial notification to send to RF task.
* @param[out] 	none
*
* @return 		none
 ******************************************************************************/
void kernel_EnablDisablLifeComputing ( const Kernel_QueueItem_struct sQueue )
{
	uint16_t ui16sensorId;
	uint16_t u16Type;
	uint16_t u16State;

	if((sQueue.ui16NbByte > 0) && (sQueue.pData != NULL))
	{
		ui16sensorId 	= ((uint16_t*)sQueue.pData)[KERNEL_MSG_IDX_ACQ_IDX_SENSORID];
		u16Type 		= ((uint16_t*)sQueue.pData)[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
		u16State 		= ((uint16_t*)sQueue.pData)[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];

		/* the sensor id is the temperature */
		if(ui16sensorId == 0)
		{
			/* if the life calculation process was never launched previously */
			if((KERNEL_SENSOR_ACQTYPE_PERIODIC == u16Type) && (KERNEL_SENSOR_ACQMODE_START == u16State))
			{
				srvCalendar_EnableMinutLifeComputing();
				kernel_sGPGiveMeLife.ui16State = KERNEL_LIFE_START;
			}
			else if(KERNEL_SENSOR_ACQMODE_STOP == u16State)
			{
				srvCalendar_DisableMinutLifeComputing();
				kernel_sGPGiveMeLife.ui16State = KERNEL_LIFE_STOP;
			}
			else
			{
				/* do nothing */
			}
		}
	}
}

/***************************************************************************//**
* @brief This function gets the shelf life and sends it to the Rf task.
* @param[in]	none
* @param[out] 	psQueueItem : the queue item to be posted
* @return 		CROSSRFID_MESSAGETOBEPOSTED: a message to be posted
 ******************************************************************************/
uint8_t kernel_ProcessGiveMeLife ( Kernel_QueueItem_struct *  psQueue )
{
	uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED;

	/* Kernel  ====[KERNEL_MESSAGEID_GPGIVEMELIFE]====>>> RF task */
	psQueue->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	psQueue->ui16notification = KERNEL_MESSAGEID_GPGIVEMELIFE;
	psQueue->ui16NbByte = sizeof(kernel_sGPGiveMeLife);
	psQueue->pData = (uint8_t *)&kernel_sGPGiveMeLife;

	return ui8Status;
}
#endif

#if (USESWO==1)
/***************************************************************************//**

 ******************************************************************************/
void kernel_processNotificationInText ( const uint8_t ui8recvsender, const Kernel_FreeRTOSMessageId_enum eFreeRTOSMessageId, uint16_t ui16NbBytes)
{
	char cText[100] = "";
	char * pcText = &cText[0];

	switch (ui8recvsender)
	{
		case KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_SENSORTASKID):
			strcpy(pcText,"S--->K,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_KERNELTASKID):
			strcpy(pcText,"K--->S,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID):
			strcpy(pcText,"R--->K,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID):
			strcpy(pcText,"K--->R,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID):
			strcpy(pcText,"R--->S,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID):
			strcpy(pcText,"S--->R,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_KERNELTASKID):
			strcpy(pcText,"K--->K,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SENSORTASKID):
			strcpy(pcText,"S--->S,\t");
		break;

		case KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_RFFRONTENDTASKID):
			strcpy(pcText,"R--->R,\t");
		break;

		default:
			strcpy(pcText,"UKW--->UKW,\t");
		break;
	}

	switch (eFreeRTOSMessageId)
	{
		case KERNEL_MESSAGEID_ACK:
			strcat(pcText,"Acq");
		break;

		case KERNEL_MESSAGEID_NACK:
			strcat(pcText,"Nack");
		break;

		case KERNEL_MESSAGEID_QUEUEFULL:
			strcat(pcText,"Queue full");
		break;

		case KERNEL_MESSAGEID_RELEASEBUFFER:
			strcat(pcText,"Release Buffer");
		break;

		case KERNEL_MESSAGEID_TASKINIT_YOURSELF:
			strcat(pcText,"Task Init yourself");
		break;

		case KERNEL_MESSAGEID_RESET:
			strcat(pcText,"Reset");
		break;

#if (1 == USE_TEMPERATURE_EM4325)
		case KERNEL_MESSAGEID_TIMOUT_GETTEMP:
			strcat(pcText,"timeout semphr GETTEMP");
		break;
#endif

		case KERNEL_MESSAGEID_STOPFIRMWARE:
			strcat(pcText,"Stop Firmware");
		break;

		case KERNEL_MESSAGEID_CALENDAR_EVENT:
			strcat(pcText,"Calendar event");
		break;
		case KERNEL_MESSAGEID_CHANGESTATESENSOR:
			strcat(pcText,"Change sensor state");
		break;

		case KERNEL_MESSAGEID_SETLOWTEMPALARM:
			strcat(pcText,"Set Low Temp Alarm");
		break;

		case KERNEL_MESSAGEID_SETHIGHTEMPALARM:
			strcat(pcText,"Set High Temp Alarm");
		break;

		case KERNEL_MESSAGEID_SETLOWBATALARM:
			strcat(pcText,"Set Low Bat Alarm");
		break;

		case KERNEL_MESSAGEID_UPDATEALARMSTATE:
			strcat(pcText,"Update alarm state");
		break;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#if 0
		case KERNEL_MESSAGEID_UPDATEFLASH_TEMPERATURE:
			strcat(pcText,"Update Flash memory");
		break;
#endif
		case KERNEL_MESSAGEID_WRITE_TEMP_AVERAGE:
			strcat(pcText,"Write AVG temp");
		break;

		case KERNEL_MESSAGEID_GPGIVEMELIFE:
			strcat(pcText,"Give Me Life");
		break;

		case KERNEL_MESSAGEID_STOP_ACQUISITIONS:
			strcat(pcText,"Stop the acquisitions");
		break;
#endif

		case KERNEL_MESSAGEID_STARTSERIALINT:
			strcat(pcText,"Start Serial interface");
		break;

		case KERNEL_MESSAGEID_STOPSERIALINT:
			strcat(pcText,"Stop serial interface");
		break;

		case KERNEL_MESSAGEID_DATATOSEND:
			strcat(pcText,"Data to send");
		break;

		case KERNEL_MESSAGEID_DATARECEIVED:
			strcat(pcText,"Data received");
		break;

		case KERNEL_MESSAGEID_WHICHCONFIG:
			strcat(pcText,"Which config");
		break;

		case KERNEL_MESSAGEID_SPIRXIRQ:
			strcat(pcText,"Spi Rx Irq");
		break;

		case KERNEL_MESSAGEID_SPITXIRQ:
			strcat(pcText,"Spi Tx Irq");
		break;

		case KERNEL_MESSAGEID_DATAREADY:
			strcat(pcText,"data ready");
		break;

		case KERNEL_MESSAGEID_ACCELEROMETERIRQ:
			strcat(pcText,"Accelerometer Irq");
		break;

		case KERNEL_MESSAGEID_COMMBUFFERSEMAPHORE:
			strcat(pcText,"Comm Buffer Semaphore");
		break;

		case KERNEL_MESSAGEID_RFFIELDDETECTED:
			strcat(pcText,"Rf field detected");
		break;

		case KERNEL_MESSAGEID_TIMEOUT:
			strcat(pcText,"Time out");
		break;

		case KERNEL_MESSAGEID_SAMPLETOGET:
			strcat(pcText,"Sample To get");
		break;

		case KERNEL_MESSAGEID_SETALARMTOWRITE:
			strcat(pcText,"Set alarm to write");
		break;

		case KERNEL_MESSAGEID_ENABLEALARM:
			strcat(pcText,"Enable alarm");
		break;

		case KERNEL_MESSAGEID_RESETALARM:
			strcat(pcText,"Reset alarm");
		break;

		case KERNEL_MESSAGEID_STARTALARMMANAGMENT:
			strcat(pcText,"Start Alarm management");
		break;

		case KERNEL_MESSAGEID_STOPRFTASK:
			strcat(pcText,"stop RF task");
		break;

		case KERNEL_MESSAGEID_GETTEMP:
			strcat(pcText,"Get temperature");
		break;

		case KERNEL_MESSAGEID_GETAGE:
			strcat(pcText,"Get age");
		break;

		case KERNEL_MESSAGEID_GETTIME:
			strcat(pcText,"Get time");
		break;

		case KERNEL_MESSAGEID_SETTIME:
			strcat(pcText,"Set time");
		break;

		case KERNEL_MESSAGEID_GETDATE:
			strcat(pcText,"Get Date");
		break;

		case KERNEL_MESSAGEID_SETDATE:
			strcat(pcText,"Set date");
		break;

		case KERNEL_MESSAGEID_ACQMODE:
			strcat(pcText,"ACQ Mode");
		break;

		case KERNEL_MESSAGEID_ACQPERIOD:
			strcat(pcText,"ACQ Period");
		break;

		case KERNEL_MESSAGEID_ACQBEGIN:
			strcat(pcText,"ACQ Begin");
		break;

		case KERNEL_MESSAGEID_ACQEND:
			strcat(pcText,"ACQ End");
		break;

		case KERNEL_MESSAGEID_ACQTHRESHOLD:
			strcat(pcText,"ACQ threshold");
		break;

		case KERNEL_MESSAGEID_ISDATAAVAILABLE:
			strcat(pcText,"Is data available");
		break;

		case KERNEL_MESSAGEID_GIVESAMPLES:
			strcat(pcText,"Give me samples");
		break;

		case KERNEL_MESSAGEID_STARTRFMODEM:
			strcat(pcText,"Start RF Mode");
		break;

		case KERNEL_GOTOSLEEPMODE:
			strcat(pcText,"Go to sleep mode");
		break;

		case KERNEL_MESSAGEID_RX:
			strcat(pcText,"Rx");
		break;

		case KERNEL_MESSAGEID_READSYSTEMFILE:
			strcat(pcText,"Ready system file");
		break;

		case KERNEL_MESSAGEID_WRITESYSTEMFILE:
			strcat(pcText,"Write system file");
		break;

		case KERNEL_MESSAGEID_MAGNETOIRQ:
			strcat(pcText,"Magneto IRQ");
		break;

		case KERNEL_MESSAGEID_DATAREADYIRQ:
			strcat(pcText,"Data ready IRQ");
		break;
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
		case KERNEL_MESSAGEID_HOWMANYTIMEYOURUN:
			strcat(pcText,"How many time you run?");
		break;

		case KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN:
			strcat(pcText,"How much time you run?");
		break;
		case KERNEL_MESSAGEID_CALIBRATION:
			strcat(pcText,"Calibration");
		break;
		case KERNEL_MESSAGEID_LOG:
			strcat(pcText,"log");
		break;
		case KERNEL_MESSAGEID_COMPUTEFFT:
			strcat(pcText,"compute FFT");
		break;
#endif
		case KERNEL_MESSAGEID_RFEVENT:
			strcat(pcText,"RF event");
		break;

		case KERNEL_MESSAGEID_LAST:
			strcat(pcText,"Error, unknown message");
		break;

		default:
			strcat(pcText,"Error, unknown message");
		break;
	}

	printf ("%s, %X\n", cText, ui16NbBytes);
}
#endif
