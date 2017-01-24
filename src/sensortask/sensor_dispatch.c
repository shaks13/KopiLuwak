/*******************************************************************************
 * @file serial_dispatch.c
 * @brief this function set manages the incoming queue object of the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "sensor_dispatch.h"

/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
static uint8_t 	sensor_ParseKernelNotification 	( Kernel_QueueItem_struct * psQueueItems );
static uint8_t 	sensor_ParseSensorNotification 	( Kernel_QueueItem_struct * psQueueItems );
static uint8_t 	sensor_ParseRFtaskNotification  ( Kernel_QueueItem_struct * psqueueItem );

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   This function parses the Kernel notifications and calls the dedicated
 *   function.
 * @details
 *
 * @param[in] pQueueItems : the received notification
 * @param[out] none
 *
 ******************************************************************************/
static uint8_t sensor_ParseKernelNotification (  Kernel_QueueItem_struct  * psQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItems->ui16notification)
	{
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_STARTSERIALINT : /* initialize the serial interface */
		/* ******************************************************************* */

			srvLIS2DH12_Init ();
			xSemaphore = xSemaphoreCreateBinary(); /* create the semaphore used for the synchronization between DataReady Irq */
			sensor_processDataAcquisition ();

		break;

		/* *************************************************************** */
		case KERNEL_MESSAGEID_STARTRFMODEM : /* start the RF modem */
		/* *************************************************************** */
			timer_initCounter();
		break;

		/* *************************************************************** */
		case KERNEL_MESSAGEID_STOPSERIALINT : /* disable the i2C interface */
		/* *************************************************************** */
		break;

		/* *********************************************************************************************** */
		case KERNEL_MESSAGEID_DATATOSEND : /* the kernel gives data to send to the serial interface device */
		/* *********************************************************************************************** */
			//sensor_senddata (pQueueItems.hwNbByte,pQueueItems.pData );
		break;

		/* *************************************************************** */
		case KERNEL_MESSAGEID_RELEASEBUFFER:
		/* *************************************************************** */

		break;

		/* ******************************************************************** */
		case KERNEL_MESSAGEID_CALENDAR_EVENT : /* There is a calendar event now */
		/* ******************************************************************** */
			//ui8status = sensor_ManageMonitoringAfterRtccEvent(psQueueItems);
		break;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
		/* ******************************************************************** */
		case KERNEL_MESSAGEID_STOP_ACQUISITIONS : /* the shelf life has expired */
		/* ******************************************************************** */
			/* stops the temperature acquisitions */
			ui8status = sensor_StopTempAcquisitions(psQueueItems);
		break;
#endif

		default :
		break;
	}

	return ui8status;

}


/***************************************************************************//**
 * @brief
 *   This function parses the Serial notifications and calls the dedicated
 *   function.
 *
 * @details
 *
 * @param[in] pQueueItems: the received notification
 * @param[out] pQueueItems
 *
 ******************************************************************************/
static uint8_t sensor_ParseSensorNotification ( Kernel_QueueItem_struct * psQueueItems )
{
	uint8_t ui8status=CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItems->ui16notification)
	{
		/* ************************************************************************** */
		case KERNEL_MESSAGEID_DATAREADY:	/* accelerometer's data must be read from */
		/* ************************************************************************** */
			srvLIS2DH12_GetGz ();
		break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_TIMEOUT:
		/* ************************************************************************** */
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SPIRXIRQ:	/* a frame has been received from the SPI interface */
		/* ******************************************************************************** */
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SAMPLETOGET:	/* sample to get */
		/* ******************************************************************************** */
			sensor_StoreNewSample ();
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ACCELEROMETERIRQ:	/* the accelerometer triggers an interupt   */
		/* ******************************************************************************** */
			ui8status = sensor_ProcessAcceleromterIRQ (psQueueItems);
		break;
		case KERNEL_MESSAGEID_DATAREADYIRQ:
			//srvlis3mdl_ProcessMagnetoIrq ();
			ui8status = sensor_ProcessMagnetoIRQ (psQueueItems);
		break;
		case  KERNEL_MESSAGEID_MAGNETOIRQ:
			//ui8status = sensor_ProcessMagnetoIRQ (psQueueItems);
		break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_GETTEMP : /* Periodic get temperature request */
		/* ******************************************************************************** */
#if (1 == USE_TEMPERATURE_EM4325)
			ui8status = pdTRUE;
			if( NULL != xBinarySemphrEm4325 )
		    {
				/* tries to take the binary semaphore for 200 freeRTOS ticks */
				/* Semaphore "xBinarySemphrEm4325": sequence 2.1 */
				ui8status = xSemaphoreTake(xBinarySemphrEm4325, KERNEL_BLOCK_TIME_SEMPHR_EM4325_GETTEMP);
		    }
			/* able to obtain the semaphore and can now access the EM4325 */
			if ( pdTRUE == ui8status )
			{
				/* Gets the temperature */
				ui8status = sensor_ProcessMeasureTemperature ( psQueueItems );

				if(CROSSRFID_MESSAGETOBEPOSTED != ui8status)
				{
					/* re-enable the irq for the comm buffer semaphore */
					//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
					interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);

					if( NULL != xBinarySemphrEm4325 )
					{
						/* Semaphore "xBinarySemphrEm4325": sequence 2.2 */
						xSemaphoreGive( xBinarySemphrEm4325 );
					}
				}
			}
			else
			{
				ui8status = CROSSRFID_MESSAGETOBEPOSTED;
				/* after the time out, re sends the message to get the temperature*/
				psQueueItems->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
				psQueueItems->ui16notification 	= KERNEL_MESSAGEID_GETTEMP;
				psQueueItems->ui16NbByte 		= 0;
				psQueueItems->pData				= NULL;
			}
#else
			srvtemp_MeasureTemperature ( );
			/* because the measurement of the temperature sends a SPI frame to the EM4325 the IRQ on MISO  is now disabled
			 * enable the irq for the comm buffer semaphore */
			//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
			interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
#endif
		break;

		case KERNEL_MESSAGEID_CALENDAR_EVENT :

			ui8status = sensor_ProcessCalendarEvent ((*psQueueItems->pData), psQueueItems);
		break;

		default :
			/* should not happen*/
		break;
	}

	return ui8status;
}
/***************************************************************************//**
 * @brief
 *   This function parses the RF task notifications and calls the dedicated
 *   function.
 *
 * @details
 *
 * @param[in] 	pQueueItems : the received notification
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be
 * posted
 *
 ******************************************************************************/
static uint8_t sensor_ParseSerialNotification ( Kernel_QueueItem_struct  * psQueueItem )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
/* Process any events that have been latched in the notified value. */
	switch (psQueueItem->ui16notification)
	{
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_SERIALREQUEST:
		/* ******************************************************************* */
			ui8status = sensor_ProcessSerialRequest ( psQueueItem);
		break;
		default:
		break;
	}

	return ui8status;
}
/***************************************************************************//**
 * @brief
 *   This function parses the RF task notifications and calls the dedicated
 *   function.
 *
 * @details
 *
 * @param[in] 	pQueueItems : the received notification
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be
 * posted
 *
 ******************************************************************************/
static uint8_t sensor_ParseRFtaskNotification ( Kernel_QueueItem_struct  * psQueueItem )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItem->ui16notification)
	{
		/* ******************************************************************* */
		case KERNEL_MESSAGEID_TASKINIT_YOURSELF:
		/* ******************************************************************* */
			sensor_ProcessInit((uint16_t *)psQueueItem->pData);
		break;

		/* ******************************************************************* */
		case KERNEL_MESSAGEID_STARTALARMMANAGMENT:
		/* ******************************************************************* */
#if (USE_TEMPERATURE_UC == 1)
			sensor_StartAlarmManagement();
#endif
		break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_GETTEMP:
		/* ************************************************************************** */
#if (1 == USE_TEMPERATURE_EM4325)
			ui8status = pdTRUE;
			if( NULL != xBinarySemphrEm4325 )
		    {
				/* tries to take the binary semaphore for 200 freeRTOS ticks */
				/* Semaphore "xBinarySemphrEm4325": sequence 3.3 */
				ui8status = xSemaphoreTake(xBinarySemphrEm4325, KERNEL_BLOCK_TIME_SEMPHR_EM4325_GETTEMP);
		    }
			/* able to obtain the semaphore and can now access the EM4325 */
			if ( pdTRUE == ui8status )
			{
				/* Gets the temperature and sends its value to RF task */
				ui8status = sensor_ProcessGetTemperature (psQueueItem);
				if( NULL != xBinarySemphrEm4325 )
				{
					/* Semaphore "xBinarySemphrEm4325": sequence 3.4 */
					xSemaphoreGive( xBinarySemphrEm4325 );
				}
			}
			else
			{
				ui8status = CROSSRFID_MESSAGETOBEPOSTED;
				/* after the time out, re sends the message to get the temperature*/
				psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
				psQueueItem->ui16notification 	= KERNEL_MESSAGEID_GETTEMP;
				psQueueItem->ui16NbByte 		= 0;
				psQueueItem->pData				= NULL;
				/* take the semaphore failed, informs rf task */
				/*ui8status = CROSSRFID_MESSAGETOBEPOSTED;
				psQueueItem->ui16NbByte 		= 0;
				psQueueItem->pData 				= NULL;
				psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
				psQueueItem->ui16notification 	= KERNEL_MESSAGEID_TIMOUT_GETTEMP;*/
			}

#else
			/* Gets the temperature and sends its value to RF task */
			ui8status = sensor_ProcessGetTemperature (psQueueItem);
#endif
		break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_ISDATAAVAILABLE:
		/* ************************************************************************** */
			ui8status = sensor_ProcessIsDataAvailable (psQueueItem);
		break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_GIVESAMPLES:
		/* ************************************************************************** */
			/* fills the message to send to rf task with the pointer to the samples buffer */
			ui8status = sensor_ProcessGiveMeSamples (psQueueItem);
		break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_ACQMODE:
		/* ************************************************************************** */
			/* changes the ACQ mode configuration according to the rf task message */
			ui8status = sensor_ProcessAcqMode (psQueueItem);
			break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_ACQPERIOD: /**/
		/* ************************************************************************** */
			/* changes the ACQ period configuration according to the rf task message */
			ui8status = sensor_ProcessAcqPeriod (psQueueItem);
			break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_ACQBEGIN: /**/
		/* ************************************************************************** */
			/* changes the ACQ begin configuration according to the rf task message */
			ui8status = sensor_ProcessAcqBegin (psQueueItem);
			break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_ACQEND: /**/
		/* ************************************************************************** */
			/* changes the ACQ end configuration according to the rf task message */
			ui8status =sensor_ProcessAcqEnd (psQueueItem);
			break;

		/* ************************************************************************** */
		case KERNEL_MESSAGEID_ACQTHRESHOLD: /**/
		/* ************************************************************************** */
			/* changes the ACQ threshold configuration according to the RF task message */
			ui8status = sensor_ProcessAcqThresholds (psQueueItem);
			break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_ENABLEALARM: /* alarm to enable/disable    					*/
		/* ******************************************************************************** */
			/* changes the EnableAlarm configuration according to the rf task message */
			ui8status = sensor_ProcessEnableDisableAlarm (psQueueItem);
			break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_RESETALARM: /* alarm to enable/disable    					*/
		/* ******************************************************************************** */
			/* Reset the alarms according to the RF task message */
			ui8status = sensor_ProcessResetAlarm (psQueueItem);
			break;

		/* ******************************************************************************** */
		case KERNEL_MESSAGEID_SETALARMTOWRITE: /* alarm to write in external eeprom memory  */
		/* ******************************************************************************** */
			sensor_StoreSetAlarmInExtMemory ( *psQueueItem);
		break;

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
		case KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN :
			srvActRec_ProcessHowMuchTimeYouRun ( (uint8_t) *(psQueueItem->pData) );
		break;
		case KERNEL_MESSAGEID_HOWMANYTIMEYOURUN :
			srvActRec_ProcessHowManyTimeYouRun ( (uint8_t) *(psQueueItem->pData) );
		break;
		case KERNEL_MESSAGEID_CALIBRATION :
			ui8status = srvActRec_ProcessCalibration ( psQueueItem);
		break;

		case KERNEL_MESSAGEID_LOG :
			ui8status = srvActRec_ProcessLog ( psQueueItem);
		break;
		case KERNEL_MESSAGEID_COMPUTEFFT :
			ui8status = srvActRec_ProcessComputeFFT ( psQueueItem);
		break;
#endif
		/* ************************************************************************** */
		default :
		/* ************************************************************************** */
			/* should not happen */
		break;
	}

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
 * @param[in] pvParameters: the parameters of the function
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void sensor_Dispatch ( void *pvParameters )
{
Kernel_QueueItem_struct sQueueItems;
BaseType_t ustatus;
uint8_t ui8status  = CROSSRFID_SUCCESSCODE;

	while (1)
	{
		ustatus = xQueueReceive (	sKernel_QueuePointer.pSensorQueue,		/* handle of the task's queue*/
									&sQueueItems,							/* pointer of the object*/
									portMAX_DELAY );						/* Block during 10 ms */


		if (ustatus == pdPASS) /* an object is available*/
		{
#if (USESWO==1)
			printf("Serial: ");
			kernel_processNotificationInText(sQueueItems.urecvsender, sQueueItems.ui16notification, sQueueItems.ui16NbByte);
#endif
			switch ( ((sQueueItems.urecvsender & KERNEL_SENDER_MASK)>>KERNEL_SENDER_SHIFT) ) /* Process any events according to the sender */
			{
				/* ******************************************************************** */
				case KERNEL_KERNELTASKID: /* the sender is the kernel					*/
				/* ******************************************************************** */
					ui8status = sensor_ParseKernelNotification ( &sQueueItems );
				break;

				/* ******************************************************************** */
				case KERNEL_SENSORTASKID:	/* the sender is the Serial task 			*/
				/* ******************************************************************** */
					ui8status = sensor_ParseSensorNotification ( &sQueueItems  );
				break;

				/* ******************************************************************** */
				case KERNEL_RFFRONTENDTASKID: /* the sender is the rf task 				*/
				/* ******************************************************************** */
					ui8status = sensor_ParseRFtaskNotification ( &sQueueItems );
				break;

				/* ******************************************************************** */
				case KERNEL_SERIALTASKID: /* the sender is the serial interface task	*/
				/* ******************************************************************** */
					ui8status = sensor_ParseSerialNotification ( &sQueueItems );
				break;

				/* ******************************************************************** */
				default:
				/* ******************************************************************** */
				break;
			}


			if ( CROSSRFID_MESSAGETOBEPOSTED == ui8status)
			{
#if 0
				if ( (KERNEL_MESSAGEID_GETTEMP == sQueueItems.ui16notification)
						&& (KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SENSORTASKID) == sQueueItems.urecvsender))
				{
					/* sends itself the get temp message to get the temperature because a "xBinarySemphrEm4325" time out occurs */
					xQueueSend (sKernel_QueuePointer.pSensorQueue,&sQueueItems,0);
				}
				else
				{

					/* sends the message to kernel */
					xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItems,0);
				}
#endif
				switch (KERNEL_WHICH_RECEIVERID(sQueueItems.urecvsender) )
				{

					case KERNEL_KERNELTASKID:
						xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItems, pdFALSE);
					break;

					case KERNEL_SENSORTASKID:
						xQueueSend (sKernel_QueuePointer.pSensorQueue,&sQueueItems, pdFALSE);
					break;

					case KERNEL_RFFRONTENDTASKID:
						xQueueSend (sKernel_QueuePointer.pRFtaskQueue,&sQueueItems, pdFALSE);
					break;

					case KERNEL_SERIALTASKID:
						xQueueSend (sKernel_QueuePointer.pSerialQueue,&sQueueItems, pdFALSE);
					break;
					default:
					break;
				}

			}else { /*do nothing*/}
		}

	}
}


