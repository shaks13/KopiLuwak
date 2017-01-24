/*******************************************************************************
 * @file rftask_dispatch.c
 * @brief this function set manages the incoming queue object of the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "rftask_dispatch.h"

/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
static uint8_t rftask_ParseKernelNotification ( Kernel_QueueItem_struct *sQueueItems );
static uint8_t rftask_ParseRFtaskNotification ( Kernel_QueueItem_struct *psQueueItems );
static uint8_t rftask_ParseSensorNotification ( Kernel_QueueItem_struct *squeueItems );

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   This function parses the Kernel notifications and calls the dedicated
 *   function.
 *
 * @param[in]	pQueueItems: the received queue item
 * @param[out]	none
 *
 * @return 		CROSSRFID_SUCCESSCODE: success
 * @return  	CROSSRFID_ERROR_INITRFTASK: the initialization failed
 * @return  	CROSSRFID_MESSAGETOBEPOSTED: a message should be posted
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 ******************************************************************************/
static uint8_t rftask_ParseKernelNotification ( Kernel_QueueItem_struct * pQueueItems )
{
uint8_t ui8SystemFileAddr = 0;
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (pQueueItems->ui16notification)
	{
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_TASKINIT_YOURSELF:
		case KERNEL_MESSAGEID_RESET:
		/* ******************************************************************* */
			/* firmware initialization in progress */
			ui8status = rftask_ProcessInit(pQueueItems);
		break;

		/* ******************************************************************* 	*/
		case KERNEL_MESSAGEID_STARTRFMODEM : /* start the RF modem 				*/
		/* ******************************************************************* 	*/
		break;

		/* *************************************************************** 	*/
		case KERNEL_MESSAGEID_STOPRFTASK : /* disable the i2C interface 	*/
		/* *************************************************************** 	*/
		break;

		/* *********************************************************************************************** */
		case KERNEL_MESSAGEID_DATATOSEND : /* the kernel gives data to send to the serial interface device */
		/* *********************************************************************************************** */

		break;
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_RELEASEBUFFER:
		/* ******************************************************************* */
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_GETTIME: /* the kernel has sent the current time */
		/* ******************************************************************* */
			ui8status = srvEM4325_TransferTimeInSystemFile ((uint32_t) *((uint32_t*)pQueueItems->pData));
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_GETDATE: /* the kernel has sent the current date */
		/* ******************************************************************* */
			ui8status = srvEM4325_TransferDateInSystemFile ((uint32_t) *((uint32_t*)pQueueItems->pData));
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_GETAGE: /* the kernel has sent the age of the board */
		/* ******************************************************************* */
			ui8status = srvEM4325_TransferBoardAgeInSystemFile ((uint16_t) *((uint16_t*)pQueueItems->pData));
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_SETTIME :
		case KERNEL_MESSAGEID_SETDATE :
		/* ******************************************************************* */
			if (KERNEL_MESSAGEID_SETTIME == pQueueItems->ui16notification)
			{
				ui8SystemFileAddr = SRVEM4325_REGISTERADDR_TIME;
			}
			else
			{
				ui8SystemFileAddr = SRVEM4325_REGISTERADDR_DATE;
			}
			srvEM4325_WriteSuccessSuccesCode(ui8SystemFileAddr,0);
		break;

		/* ************************************************************************************** 	*/
		case KERNEL_MESSAGEID_SETLOWTEMPALARM: /* below temperature alarm to set in system files  	*/
		/* ************************************************************************************** 	*/
			/* message sent by the measurement service */
			ui8status = rftask_ProcessSetAlarmFromSensor(KERNEL_ALARM_ID_LOW_TEMP ,pQueueItems);
			break;

		/* *************************************************************************************** 	*/
		case KERNEL_MESSAGEID_SETHIGHTEMPALARM: /* upper temperature alarm to set in system files  	*/
		/* *************************************************************************************** 	*/
			/* message sent by the measurement service */
			ui8status = rftask_ProcessSetAlarmFromSensor(KERNEL_ALARM_ID_HIGH_TEMP,pQueueItems);
			break;

		/* ***************************************************************************************  */
		case KERNEL_MESSAGEID_SETLOWBATALARM: /* low battery alarm to set in system files  			*/
		/* *************************************************************************************** 	*/
			ui8status = rftask_ProcessSetAlarmFromSensor(KERNEL_ALARM_ID_LOW_BATTERY,pQueueItems);
			break;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
		/* ***************************************************************************************  */
		case KERNEL_MESSAGEID_GPGIVEMELIFE: /* writes the shelf life in the register file  			*/
		/* *************************************************************************************** 	*/
			ui8status = rftask_ProcessReponseGiveMeLife(*pQueueItems);
			break;
#endif

		default :
		break;
	}

	/* Writes the error code in the second register file if an error occurred */
	rftask_HookErrorCode(ui8status);

	return ui8status;

}

/***************************************************************************//**
 * @brief
 *   This function parses the RF task notifications and calls the dedicated
 *   function.
 *
 * @param[in] 	pQueueItems : the received notification
 * @param[out] 	none
 * @return 	   	CROSSRFID_SUCCESSCODE: the function is successful
 * @return 	   	CROSSRFID_MESSAGETOBEPOSTED: a queue message
 * 				should be posted
 * @return 	   	CROSSRFID_ERROR_EM4325READ: the read operation of the
 * 				register file of the EM4325 is not successful
 * @return 	   	CROSSRFID_ERROR_SYSFILE_NOT_AVAILABLE: the system file doesn't exist
 * @return 	   	CROSSRFID_ERROR_SYSFILE_READACESSNOTPERMITED: the read access
 * 				to the system file is not permitted
 * @return 	   	CROSSRFID_ERROR_SYSFILE_WRITEACESSNOTPERMITED: the write access
 * 				to the system file is not permitted
 * @return 	    CROSSRFID_MESSAGETOBEPOSTED: a message should be posted
 * @return 	    CROSSRFID_ERROR_SEMAPHR_EM4325_TIMOUT: unexpected time out to obtain the em4325 semaphore
 * @return 		CROSSRFID_READSENSORID_ERROR: error when reading the sensor id
 ******************************************************************************/
static uint8_t rftask_ParseRFtaskNotification ( Kernel_QueueItem_struct * psQueueItems )
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItems->ui16notification)
	{
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_RFFIELDDETECTED:
		/* ******************************************************************* */
				timer_initCounter();

				if ((protocol_pTagResponse->bIsImmediateReplyExpected == true) &&
					(protocol_pTagResponse->bIsResPonseExpected == true))
				{
					//timer_WaitEndOfDelay (); /* wait until T1*/
					/* send the response */
				}
				else if (protocol_pTagResponse->bIsResPonseExpected == true)
				{
					//timer_WaitEndOfDelay (); /* wait until T1*/
					/* send the response */
				}
				else { /* do nothing : restart the decoding machine*/}
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_COMMBUFFERSEMAPHORE : /* the Miso triggered an irq*/
		/* ******************************************************************* */
#if (1 == USE_TEMPERATURE_EM4325)
			ui8Status = pdTRUE;
			if( NULL != xBinarySemphrEm4325 )
		    {
				/* Semaphore "xBinarySemphrEm4325": sequence 1.1
				 * or
				 * Semaphore "xBinarySemphrEm4325": sequence 3.1 */
				/* tries to take the binary semaphore for 25 freeRTOS ticks */
				ui8Status = xSemaphoreTake(xBinarySemphrEm4325, KERNEL_BLOCK_TIME_SEMPHR_EM4325_COMBUFFER);
		    }

			/* able to obtain the semaphore and can now access the EM4325 */
			if ( pdTRUE == ui8Status )
			{
				/* Starts to process the RF command */
				ui8Status = rftask_ProcessCommBufferCommand (psQueueItems);
			}
			else
			{
				ui8Status = CROSSRFID_ERROR_SEMAPHR_EM4325_TIMOUT;
			}

#else
			ui8Status = rftask_ProcessCommBufferCommand ();
#endif
		break;

		/* ******************************************************************* */
		default :
		/* ******************************************************************* */
			/* should not happen*/
		break;
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   This function parses the Serial task notifications and calls the dedicated
 *   function.
 *
 * @param[in] 	queueItems : the received queue item
 * @param[out] 	none
 *
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : the function is successful and a message
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 * @return 		CROSSRFID_ERROR_SYSFILE_GIVEMESAMPLE_READCMDOVERFLOW: wrong index/number of sample to read, download command overflow
 ******************************************************************************/
static uint8_t rftask_ParseSensorNotification ( Kernel_QueueItem_struct *squeueItems )
{
bool bAcknowledgeFromSerial;
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (squeueItems->ui16notification)
	{
		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_UPDATEALARMSTATE:
		/* ******************************************************************************** */
			ui8status = rftask_FinishInitFirmware(squeueItems);
#if (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)
			GPIO_PinModeSet(INTERFACE_MAGNETICPWR_PORT,INTERFACE_MAGNETICPWR_PIN, gpioModePushPull, 0); /* swhitch off the led */
#endif
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_RESETALARM:
		/* ******************************************************************************** */
			/* reset performed by serial, the serial sent the content of the external EEPROM */
			if(squeueItems->ui16NbByte > 0)
			{
				/* Updates the alarm system files according to the reset by serial */
				/* the first word is the alarm state */
				rftask_UpdateSysFileFromSerial(SRVEM4325_REGISTERADDR_STATEALARM,(uint16_t*)squeueItems->pData);
				/* the other words are the RTCC date and the RTCC time when the alarms has been triggered */
				rftask_UpdateSysFileFromSerial(SRVEM4325_REGISTERADDR_DATETIMEALARM,(&((uint16_t*)squeueItems->pData)[1]));

				bAcknowledgeFromSerial = true;
			}
			else
			{
				/* The reset failed */
				bAcknowledgeFromSerial = false;
				/* serial failed to write the new reset data in the eeprom */
				ui8status = CROSSRFID_ERROR_I2C_WRITE;
			}

			/* Writes the response in the register according to the reset status from serial */
			rftask_ProcessResponseSysFile((uint8_t *)&bAcknowledgeFromSerial); /*  TODO: DO NOT return the status, we force the status to false */

		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_GETTEMP :
		/* ******************************************************************* */
#if (1 == USE_TEMPERATURE_EM4325)
			ui8status = pdTRUE;
			if( NULL != xBinarySemphrEm4325 )
		    {
				/* tries to take the binary semaphore for 25 freeRTOS ticks */
				/* Semaphore "xBinarySemphrEm4325": sequence 3.5 */
				ui8status = xSemaphoreTake(xBinarySemphrEm4325, KERNEL_BLOCK_TIME_SEMPHR_EM4325_COMBUFFER);
		    }
			/* able to obtain the semaphore and can now access the EM4325 */
			if ( pdTRUE == ui8status )
			{
				ui8status = rftask_ProcessReponseGetTemp(squeueItems->pData);
			}
			else
			{
				ui8status = CROSSRFID_ERROR_SEMAPHR_EM4325_TIMOUT;
			}

#else
			/* Gets the temperature and sends its value to RF task */
			ui8status = serial_ProcessGetTemperature (psQueueItem);
#endif
			break;

#if (1 == USE_TEMPERATURE_EM4325)
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_TIMOUT_GETTEMP : /* Serial did not take the semaphore and so not read the em4325 temperature */
		/* ******************************************************************* */
			srvEM4325_DeliverRespSysFileWriteCmd(false);
			ui8status = CROSSRFID_ERROR_SEMAPHR_EM4325_TIMOUT;
			if( NULL != xBinarySemphrEm4325 )
			{
				/* Semaphore "xBinarySemphrEm4325": sequence 3.6 */
				xSemaphoreGive( xBinarySemphrEm4325 );
			}
		break;
#endif

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ISDATAAVAILABLE :
		/* ******************************************************************* */
			ui8status = rftask_ProcessReponseDataIsAvailable((uint16_t)*((uint16_t *)(squeueItems->pData)));
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_GIVESAMPLES :
		/* ******************************************************************* */
			ui8status = rftask_ProcessReponseGiveMeSamples(squeueItems->pData);
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ACQMODE:
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ACQPERIOD:
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ACQBEGIN:
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ACQEND:
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ACQTHRESHOLD:
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_ENABLEALARM:
		/* ******************************************************************* */
			/* rf task receives the acknowledge message from serial
			 * if the configuration has been updated by serial,
			 * updates the system files and deliver the response in register files */
			ui8status = rftask_ProcessResponseSysFile(squeueItems->pData);
			break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_CHANGESTATESENSOR:
		/* ******************************************************************* */
			/* Serial task has modified the acqmode, it sends a message to update the configuration in RF task */
			rftask_UpdateSysFileFromSerial(SRVEM4325_REGISTERADDR_ACQMODE,(uint16_t*)squeueItems->pData);
			break;

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
		case KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN :

			/* transfers the message to Serial task, firmware initialization in progress */
			rftask_WriteSystemFileAccToQueueData(SRVEM4325_REGISTERADDR_HOWMUCHTIMEYOURUN,squeueItems->ui16NbByte,squeueItems->pData);
		break;
		case KERNEL_MESSAGEID_HOWMANYTIMEYOURUN :
			/* transfers the message to Serial task, firmware initialization in progress */
			rftask_WriteSystemFileAccToQueueData(SRVEM4325_REGISTERADDR_HOWMANYTIMEYOURUN,squeueItems->ui16NbByte,squeueItems->pData);
		break;
		case KERNEL_MESSAGEID_CALIBRATION :
			srvEM4325_UpdateSystemFiles (SRVEM4325_REGISTERADDR_CALIBRATION , squeueItems->pData[0], squeueItems->pData[1]);
			rftask_WriteSystemFileAccToQueueData(SRVEM4325_REGISTERADDR_CALIBRATION,squeueItems->ui16NbByte,squeueItems->pData);
		break;

		case KERNEL_MESSAGEID_LOG :
			srvEM4325_UpdateSystemFiles (SRVEM4325_REGISTERADDR_LOG , squeueItems->pData[0], squeueItems->pData[1]);
			rftask_WriteSystemFileAccToQueueData(SRVEM4325_REGISTERADDR_LOG,squeueItems->ui16NbByte,squeueItems->pData);
		break;

		case KERNEL_MESSAGEID_COMPUTEFFT :
			srvEM4325_UpdateSystemFiles (SRVEM4325_REGISTERADDR_COMPUTEFFT , squeueItems->pData[0], squeueItems->pData[1]);
			rftask_WriteSystemFileAccToQueueData(SRVEM4325_REGISTERADDR_COMPUTEFFT,squeueItems->ui16NbByte,squeueItems->pData);
		break;

#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_WRITE_TEMP_AVERAGE:
		/* ******************************************************************* */
#if (1 == USE_TEMPERATURE_EM4325)
			rftask_WriteTempAverageInUserMem( (int16_t) *((int16_t*)squeueItems->pData) );

			/* re-enable the irq for the comm buffer semaphore */
			interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);

			if( NULL != xBinarySemphrEm4325 )
			{
				/* Semaphore "xBinarySemphrEm4325": sequence 2.2" */
				xSemaphoreGive( xBinarySemphrEm4325 );
			}
#endif
			break;
#elif (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)
		case KERNEL_MESSAGEID_ACCELEROIRQ:
#if 1
			if ((squeueItems->pData[0])==  0x01)
			{
				GPIO_PinOutSet(INTERFACE_USART1EN_PORT,INTERFACE_USART1EN_PIN); /* Switch off the led */
				GPIO_PinOutSet(INTERFACE_GREENLED_PORT,INTERFACE_GREENLED_PIN); /* Switch off the led */
			}
			else
			{
				GPIO_PinOutClear(INTERFACE_GREENLED_PORT,INTERFACE_GREENLED_PIN); /* Switch off the led */
				GPIO_PinOutClear(INTERFACE_USART1EN_PORT,INTERFACE_USART1EN_PIN); /* Switch off the led */
			}
#endif
		break;

#endif


		/* ******************************************************************* */
		default:
		/* ******************************************************************* */
		break;
	}



	/* Writes the error code in the second register file if an error occurred */
	rftask_HookErrorCode(ui8status);


	return ui8status ;

}

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *  This task shows bits within the RTOS task notification value being used to pass
 *	different events to the task in the same way that flags in an event group might
 *	be used for the same purpose.
 *
 *
 * @param[in] pvParameters: parameters of the function
 * @param[out] none
 * @return none
 ******************************************************************************/
void rftask_Dispatch ( void *pvParameters )
{
Kernel_QueueItem_struct sQueueItem;
BaseType_t ustatus;
uint8_t ui8status;

	while (1)
	{

		ustatus = xQueueReceive (	sKernel_QueuePointer.pRFtaskQueue,		/* handle of the task's queue*/
									&sQueueItem,							/* pointer of the object*/
									portMAX_DELAY );						/* Block indefinitely. */

		if (ustatus == pdPASS) /* an object is available*/
		{
#if (USESWO==1)
			printf("RF    : ");
			kernel_processNotificationInText(sQueueItem.urecvsender, sQueueItem.ui16notification, sQueueItem.ui16NbByte);
#endif

			/* Process any events according to the sender */
			switch ( ((sQueueItem.urecvsender & KERNEL_SENDER_MASK)>>KERNEL_SENDER_SHIFT) )
			{
				/* ******************************************************************** */
				case KERNEL_KERNELTASKID: /* the sender is the kernel					*/
				/* ******************************************************************** */
					ui8status = rftask_ParseKernelNotification ( &sQueueItem );

					if (CROSSRFID_MESSAGETOBEPOSTED  != ui8status )
					{
						/* the process is now over reactivate the irq on miso to catch the next CommBufferSempahore*/
						//interface_EnablePinInterrupt (INTERFACE_MISO_PORT,INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
						interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
#if (1 == USE_TEMPERATURE_EM4325)
						/* the process of the RF command is completed, the binary semaphore is released */
						if ( NULL != xBinarySemphrEm4325 )
						{
							/* Semaphore "xBinarySemphrEm4325": sequence 1.2 */
							xSemaphoreGive( xBinarySemphrEm4325 );
						}
						else { /* Do nothing */ }
#endif
					}
				break;

				/* ******************************************************************** */
				case KERNEL_SENSORTASKID:	/* the sender is the Serial task 			*/
				/* ******************************************************************** */
					ui8status = rftask_ParseSensorNotification (&sQueueItem);

					/* the process is now over reactivate the irq on miso to catch the next CommBufferSempahore*/
					//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
					interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
#if (1 == USE_TEMPERATURE_EM4325)
					/* the process of the RF command is completed, the binary semaphore is released */
					if ( NULL != xBinarySemphrEm4325 )
					{
						/* Semaphore "xBinarySemphrEm4325": sequence 1.2
						 * or
						 * Semaphore "xBinarySemphrEm4325": sequence 3.6 */
						xSemaphoreGive( xBinarySemphrEm4325 );
					}
					else { /* Do nothing */ }
#endif
				break;

				/* ******************************************************************** */
				case KERNEL_RFFRONTENDTASKID: /* the sender is the rf task 				*/
				/* ******************************************************************** */
					ui8status = rftask_ParseRFtaskNotification ( &sQueueItem );
#if (1 == USE_TEMPERATURE_EM4325)
					if ( CROSSRFID_MESSAGETOBEPOSTED  != ui8status )
					{
						/* the process of the RF command is completed, the binary semaphore is released */
						if ( NULL != xBinarySemphrEm4325 )
						{
							/* Semaphore "xBinarySemphrEm4325": sequence 1.2 */
							xSemaphoreGive( xBinarySemphrEm4325 );
						}
						else { /* Do nothing */ }
					}
#endif
				break;

				/* ******************************************************************** */
				default:
				/* ******************************************************************** */
				break;
			}

			if (CROSSRFID_MESSAGETOBEPOSTED == ui8status )
			{
				xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
#if (USESWO > 1)
				printf("rf SEND in kernel queue, %X\n",sQueueItem.urecvsender);
#endif
			}
			else
			{
				/* enable the irq for the comm buffer semaphore */
				//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
				interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
			}
		}

		uint8_t ui8NbMessageWaiting = uxQueueMessagesWaiting (sKernel_QueuePointer.pRFtaskQueue);
		if (ui8NbMessageWaiting>=1)
		{
			ustatus = pdPASS;
		}
	}
}
