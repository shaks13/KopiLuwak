/*******************************************************************************
 * @file kernel_dispatch.c
 * @brief creation the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "kernel_dispatch.h"

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
static uint8_t kernel_ashunt[KERNEL_SHUNTID_LASTELEMENT]={KERNEL_SERIALTASKID,KERNEL_SERIALTASKID};
/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
static uint8_t	kernel_ParseKernelNotification 		( Kernel_QueueItem_struct *pQueueItems );
static uint8_t 	kernel_ParseSensorNotification 		( Kernel_QueueItem_struct *pQueueItems );
static uint8_t 	kernel_ParseRFtaskNotification 		( Kernel_QueueItem_struct *pQueueItems );
static uint8_t 	kernel_ParseSerialtaskNotification 	( Kernel_QueueItem_struct *pQueueItems );
static void 	kernel_processShunt					( Kernel_QueueItem_struct * const pQueueItems);
/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   This function parses the received notification and calls the dedicated
 *   function.
 *
 * @details
 *
 * @param[in] 	dwNotifiedValue : the received notification
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 *
 ******************************************************************************/
static uint8_t kernel_ParseKernelNotification (  Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
#endif
	/* Process any events that have been latched in the notified value. */
	switch (pQueueItems->ui16notification)
	{
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_TASKINIT_YOURSELF:
		/* ******************************************************************************** */
			/* firmware initialization in progress */
			ui8status = kernel_ProcessInit(pQueueItems);
		break;

		case KERNEL_STAYWAKEUP:
			ui8status = kernel_processStartWakeUpMode (pQueueItems);
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SETLOWTEMPALARM: /*  alarm to set in system file */
		case KERNEL_MESSAGEID_SETHIGHTEMPALARM:
		case KERNEL_MESSAGEID_SETLOWBATALARM:
		/* ******************************************************************************** */
			pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
			//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,pQueueItems, pdFALSE);
			break;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_CALENDAR_EVENT: /* There is an calendar event now */
		/* ******************************************************************************** */
			if (SRVRTCC_CALENDAREVENT_HOUR_TICK == (*sQueueItems.pData))
			{
				/* compute the new life value */
				ui8Status = kernel_ProcessShelfLife(&sQueueItems);

				/* if a message must be sent: */
				if ( CROSSRFID_MESSAGETOBEPOSTED == ui8Status)
				{
					xQueueSend (sKernel_QueuePointer.pSensorQueue,&sQueueItems,0);
				}else { /*do nothing*/}

#if (1 == BACKUP_RESTORE_FLASH)
				kernel_BackUpTempObjectsInFlash();
#endif
			}
			break;

#endif

		default :
			break;
	}
	return ui8status;
}


/***************************************************************************//**
 * @brief
 *   This function parses the received notification from the serial interface
 *   task and calls the dedicated function.
 *
 * @details
 *
 * @param[in] sQueueItems : the received notification
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 *
 ******************************************************************************/
static uint8_t kernel_ParseSerialtaskNotification (  Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;


	/* Process any events that have been latched in the notified value. */
		switch (pQueueItems->ui16notification)
		{

			case KERNEL_MESSAGEID_TASKINIT_YOURSELF:

				/* Sends a message to the RF task, initialization still in progress */
				pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
				ui8status = CROSSRFID_MESSAGETOBEPOSTED;
				/* transfers the message to Rf task */
				//xQueueSend (sKernel_QueuePointer.pSensorQueue,pQueueItems, pdFALSE);
			break;

			case KERNEL_MESSAGEID_SHUNT:

				kernel_processShunt(pQueueItems);

			break;

			default:
			break;
		}

	return ui8status;

}


/***************************************************************************//**
 * @brief
 *   This function parses the received notification from the serial interface
 *   task and calls the dedicated function.
 * @param[in] sQueueItems : the received notification
 * @return 		none
 *
 ******************************************************************************/
static void kernel_processShunt(Kernel_QueueItem_struct * const pQueueItems)
{
uint8_t ui8index =0;

	ui8index = (uint8_t) *(pQueueItems->pData);
	if (ui8index<KERNEL_SHUNTID_LASTELEMENT)
	{
		kernel_ashunt[ui8index] = KERNEL_WHICH_SENDERID(pQueueItems->urecvsender);
	}
	else
	{
		/* do nothing*/
	}
}

/***************************************************************************//**
 * @brief
 *   This function parses the received notification from the serial interface
 *   task and calls the dedicated function.
 * @param[in,out] pQueueItems : the received notification
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 ******************************************************************************/
static uint8_t kernel_ParseSensorNotification (  Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
#if (1 == BACKUP_RESTORE_FLASH)
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
#endif

	/* Process any events that have been latched in the notified value. */
	switch (pQueueItems->ui16notification)
	{
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GETTEMP: /* Comm buffer message "Get temperature" 			*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ISDATAAVAILABLE: /* Comm buffer message "Data is available?" 	*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GIVESAMPLES: /* Comm buffer message "DOWNLAOD data" 			*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQMODE: /* Transfers the Serial acknowledge message to RF task */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQPERIOD: /* Transfers the Serial acknowledge message to RF task */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQBEGIN: /* Transfers the Serial acknowledge message to RF task */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQEND:/* Transfers the Serial acknowledge message to RF task */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQTHRESHOLD:/* Transfers the Serial acknowledge message to RF task */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ENABLEALARM:/* Transfers the Serial acknowledge message to RF task */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_RESETALARM: /* Transfers the content of the external EEPROM if the reset is successful */
		/* ******************************************************************************** */
#if (1 == USE_TEMPERATURE_EM4325)
		case KERNEL_MESSAGEID_TIMOUT_GETTEMP: /* Serial did not take the semaphore */
#endif
		/* ******************************************************************************** */
			/* transfers the message to Rf task */
			//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_CHANGESTATESENSOR: /* there is a calendar event (process in serial),
												rf task must change an ACQ Mode system file */
		/* ******************************************************************************** */
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
			/* Update the Calendar irq according to the temperature sensor configuration */
			kernel_EnablDisablLifeComputing(sQueueItems);
#endif
			/* transfers the message to Rf task */
			//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);*
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_UPDATEALARMSTATE: /* RF task must change the Alarm state system file			*/
												/* according to the data read in external eeprom by serial 	*/
		/* ******************************************************************************** */
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
		case KERNEL_MESSAGEID_WRITE_TEMP_AVERAGE: /* Rf task should write the temperature average
		 	 	 	 	 	 	 	 	 	 	 	 in the em4325 user memory */
		/* ******************************************************************************** */

#if (1 == BACKUP_RESTORE_FLASH)
			/* Restore process: pasting the sensor flash buffer in ram */
			ui8Status = kernel_RestoreFlashToRam();

			if(CROSSRFID_ERROR_NVM_READ != ui8Status)
			{
				/* transfers the message to Rf task */
				xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);
			}
			else
			{
				/* Do not send the KERNEL_MESSAGEID_UPDATEALARMSTATE to RF task to not
				 * write the successful firmware initialization in the register files */
			}
#else /* 1 != BACKUP_RESTORE_FLASH */
			/* transfers the message to Rf task */
			xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);
#endif
#else /* APP_CHOSEN_FLAG != APP_GLUEPOT */
			/* transfers the message to Rf task */
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
			//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,pQueueItems, pdFALSE);
#endif
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_STOPFIRMWARE:
		/* ******************************************************************************** */
			/* RF task initialization failed, �C must sleep */
			kernel_AllowSleepInDefault( );
			/* TODO: reset process ? */
		break;

		/* ********************************************************** */
		case KERNEL_MESSAGEID_DATARECEIVED:	/* data has been received */
		/* ********************************************************** */
		break;

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
		case KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN :	/* outgoing way*/
		case KERNEL_MESSAGEID_HOWMANYTIMEYOURUN :
		case KERNEL_MESSAGEID_CALIBRATION :

			/* transfers the message to Serial task, firmware initialization in progress */
			//xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
		break;
#elif (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)
		case KERNEL_MESSAGEID_ACCELEROIRQ :
			/* transfers the message to Serial task, firmware initialization in progress */
			xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);
		break;
#endif
		/* ********************************************************** */
		//case KERNEL_MESSAGEID_QUEUEFULL: /* when no message cannot be send anymore to the Serial interface */
		/* ********************************************************** */
			//xQueueReset(sKernel_QueuePointer.pSensorQueue);
			//xQueueSend (sKernel_QueuePointer.pLnaQueue,&pInitSerialQueueItem, pdFALSE); /* re initiate the Serial state machine */
		//break;

		/* ********************************************************** */
		case KERNEL_MESSAGEID_CALENDAR_EVENT:
		/* ********************************************************** */
			/* TODO */
		break;

		case KERNEL_MESSAGEID_LOG :
		case KERNEL_MESSAGEID_COMPUTEFFT :
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
			if (KERNEL_RFFRONTENDTASKID == kernel_ashunt[KERNEL_SHUNTID_LOGREADY])
			{
				pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
			}
			else
			{
				pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_SENSORTASKID);
			}
		break;
		case KERNEL_MESSAGEID_MEASUREREADY :

			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
			if (KERNEL_RFFRONTENDTASKID == kernel_ashunt[KERNEL_SHUNTID_MEASUREREADY])
			{
				pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
			}
			else
			{
				pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_SENSORTASKID);
			}
		break;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#if 0
/* code used if we want to not save the temperature objects (in ram, nvm_config.c)
 * directly in the service measurement called by the serial task after a logical timer irq
 * (see srvtemp_MeasureTemperature)*/
		/* ********************************************************** */
		case KERNEL_MESSAGEID_UPDATEFLASH_TEMPERATURE:
		/* ********************************************************** */
			kernel_UpdateTempObjectsInFlash(sQueueItems);
		break;
#endif
#endif

		default :
		break;
	}
	return ui8status;
}



/***************************************************************************//**
 * @brief
 *   This function parses the RF task notifications and calls the dedicated function.
 * @param[in,out] pQueueItems : the received notification
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 ******************************************************************************/
static uint8_t kernel_ParseRFtaskNotification (  Kernel_QueueItem_struct *pQueueItem )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (pQueueItem->ui16notification)
	{
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_TASKINIT_YOURSELF: /* transfers the message to Serial task, firmware initialization in progress */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GETTEMP: /* Comm buffer message "Get temperature" 			*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ISDATAAVAILABLE: /* Comm buffer message "Data is available?" 	*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GIVESAMPLES: /* Comm buffer message "DOWNLAOD data" 			*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQMODE: /**/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQPERIOD: /**/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQBEGIN: /**/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQEND: /**/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACQTHRESHOLD: /* sends to serial the new threshold values     */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ENABLEALARM: /* sends to serial the alarm to enable/disable 	*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_RESETALARM: /* sends to serial the alarm to reset				*/
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SETALARMTOWRITE: /* alarm to write in external eeprom memory  */
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_STARTALARMMANAGMENT: /* Serial must start to manage the alarms (message sent after the firmware init) */
		/* ******************************************************************************** */
			/* Transfers the RF task message to serial to update the configuration */
			//xQueueSend (sKernel_QueuePointer.pSensorQueue,&pQueueItem, pdFALSE);
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;

		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_STOPFIRMWARE:
		/* ******************************************************************************** */
			/* RF task initialization failed, �C must sleep */
			kernel_AllowSleepInDefault();
			/* TODO: reset process ? */
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GETAGE:
		/* ******************************************************************************** */
			 ui8status = kernel_processGetAge ( pQueueItem );
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GETTIME:
		/* ******************************************************************************** */
			ui8status = kernel_processGetTime ( pQueueItem );
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SETTIME:
		/* ******************************************************************************** */
			ui8status = kernel_processSetTime ( pQueueItem);
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GETDATE:
		/* ******************************************************************************** */
			ui8status = kernel_processGetDate (pQueueItem);
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SETDATE:
		/* ******************************************************************************** */
			ui8status = kernel_processSetDate (pQueueItem);
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_RESET:
		/* ******************************************************************************** */
			ui8status = kernel_ProcessReset ( pQueueItem);
		break;

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
		case KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN : /* incoming way*/
		case KERNEL_MESSAGEID_HOWMANYTIMEYOURUN :
		case KERNEL_MESSAGEID_CALIBRATION :
		case KERNEL_MESSAGEID_LOG :
		case KERNEL_MESSAGEID_COMPUTEFFT :
			/* transfers the message to Serial task, firmware initialization in progress */
			//xQueueSend (sKernel_QueuePointer.pSensorQueue,&pQueueItem, pdFALSE);
			ui8status = CROSSRFID_MESSAGETOBEPOSTED;
		break;
#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GPGIVEMELIFE:
		/* ******************************************************************************** */
			ui8status = kernel_ProcessGiveMeLife (&pQueueItem);
		break;
#endif
		default:
		break;
	}

	/* when a message should be posted to the RF task*/
#if 0
	if (CROSSRFID_MESSAGETOBEPOSTED == ui8status)
	{
		xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&pQueueItem, pdFALSE);
	}
#endif
	return ui8status;
}

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   This task shows bits within the RTOS task notification value being used to pass
 *	different events to the task in the same way that flags in an event group might
 *	be used for the same purpose.
 *
 * @details
 *
 * @param[in] init
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void kernel_Dispatch ( void *pvParameters )
{
Kernel_QueueItem_struct QueueItems;
BaseType_t ustatus;
uint8_t ui8status = CROSSRFID_SUCCESSCODE;


	while (1)
	{
		ustatus = xQueueReceive (	sKernel_QueuePointer.pKernelQueue,		/* handle of the task's queue*/
									&QueueItems,							/* pointer of the object*/
									10 );									/* Block during 10 ms */

		if (ustatus == pdPASS) /* an object is available*/
		{
#if (USESWO==1)
			printf("Kernel: ");
			kernel_processNotificationInText(QueueItems.urecvsender, QueueItems.ui16notification, QueueItems.ui16NbByte);
#endif
			switch ( ((QueueItems.urecvsender & KERNEL_SENDER_MASK)>>KERNEL_SENDER_SHIFT) ) /* Process any events according to the sender */
			{
				/* ********************************************************************	*/
				case KERNEL_KERNELTASKID: /* the sender is the kernel					*/
				/* ********************************************************************	*/
					ui8status = kernel_ParseKernelNotification(&QueueItems);
				break;

				/* ********************************************************************	*/
				case KERNEL_SENSORTASKID:	/* the sender is the Serial interface task 	*/
				/* ********************************************************************	*/
					ui8status = kernel_ParseSensorNotification ( &QueueItems);
				break;

				/* ********************************************************************	*/
				case KERNEL_RFFRONTENDTASKID:	/* the sender is the RF front end 		*/
				/* ********************************************************************	*/
					ui8status = kernel_ParseRFtaskNotification ( &QueueItems );
				break;

				/* ********************************************************************	*/
				case KERNEL_SERIALTASKID:	/* the sender is the RF front end 		*/
				/* ********************************************************************	*/
					kernel_ParseSerialtaskNotification ( &QueueItems );
				break;

				/* ********************************************************************	*/
				default:
				/* ********************************************************************	*/
				break;
			}

			if ( CROSSRFID_MESSAGETOBEPOSTED == ui8status) /*when the end of the process requests a messagebe posted*/
			{
				switch (KERNEL_WHICH_RECEIVERID(QueueItems.urecvsender) )
				{
					case KERNEL_SENSORTASKID:
						xQueueSend (sKernel_QueuePointer.pSensorQueue,&QueueItems, pdFALSE);
					break;

					case KERNEL_RFFRONTENDTASKID:
						xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&QueueItems, pdFALSE);
					break;

					case KERNEL_SERIALTASKID:
						xQueueSend (sKernel_QueuePointer.pSerialQueue,&QueueItems, pdFALSE);
					break;
					default:
					break;
				}
			}


		}
		else
		{ /* do nothing*/	}

		watchdog_TickleWatchdog (); /* clear the watch dog counter*/
	}
}


