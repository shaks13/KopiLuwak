/*******************************************************************************
 * @file serial_process.c
 * @brief this function set manages the data exchange between the task and the
 * device.
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "serial_process.h"
/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
kernel_DataExchange_Type aui8QueueData [2];	/* memory to store the data send to the others tasks*/
static uint8_t aui8data[2];
static uint16_t aui16data[3];

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
/*===========================================================================================================
						Private function declaration
===========================================================================================================*/

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/


/***************************************************************************//**
 * @brief  		This function processes the initialization of serial task
 * @param[in,out] pQueueItems : the incoming message
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : the function is successful and
 * a message should be posted
 ******************************************************************************/
uint8_t serial_ProcessInit ( Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	//Kernel_QueueItem_struct sQueueItem;
	pQueueItems->ui16NbByte = 0;
	pQueueItems->pData = NULL;

	/* Sends a message to Serial, initialization still in progress */
	pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_SERIALTASKID);
	//pQueueItems->ui16notification = KERNEL_MESSAGEID_TASKINIT_YOURSELF;

	srvm2m_init();
	return ui8status;
}

#if USEM2MINTERFACE == 1 /* these IRQ handlers is dedicated to the M2M communication*/
/***************************************************************************//**
 * @brief 		This function manages the monitoring of the sensor after
 * 				receiving a calendar event.
 * @param[in]	i8CalendarEventId: the calendar event id
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void serial_processKernelM2Mack ( void )
{
	srvm2m_SendString ("\nroger \n");
	USART_IntEnable(USART0, USART_IEN_RXDATAV);
	intuart_rxBuf.wrI  = 0;
}

/***************************************************************************//**
 * @brief 		This function manages the monitoring of the sensor after
 * 				receiving a calendar event.
 * @param[in]	i8CalendarEventId: the calendar event id
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void serial_processEndOfWakeupMode ( void )
{
	USART_IntDisable(USART0, USART_IEN_RXDATAV);
	srvm2m_SendString ("\n Out \n");
	interface_EnablePinInterrupt (INTERFACELEUART_RX_PORT,INTERFACELEUART_RX_PIN,INTERFACE_RISING_EDGE);
}


/***************************************************************************//**
 * @brief 		This function updates the power management when a Rx communication
 * has been receives from a M2M host. Because the USART doesn't work on EM2 or
 * below, the sleep mode should be updated
 * @param[in,out] pQueueItems : the received notification
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 ******************************************************************************/
uint8_t serial_ProcessM2MRxmessage (Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
uint8_t ui8shuntstatus = CROSSRFID_SUCCESSCODE;
kernel_DataExchange_Type sdataobject;
Kernel_QueueItem_struct pQueueShuntItem ;


	/* identify the different fields of the serial command*/
	sdataobject.pui8data = aui8data;
	srvm2m_ProcessRxMessage ( pQueueItems->pData , &sdataobject);

	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* save the fields of the command */
		memcpy(aui8QueueData , &sdataobject, sizeof (kernel_DataExchange_Type));
		/* create the queue object */
		pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SERIALTASKID);
		pQueueItems->ui16notification = KERNEL_MESSAGEID_SERIALREQUEST;
		pQueueItems->pData = (uint8_t*) aui8QueueData; /* change the memory area to send the queue object*/
		pQueueItems->ui16NbByte= sizeof (kernel_DataExchange_Type);
		ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else
	{
		/* do nothing*/
	}

	ui8shuntstatus = srvm2m_ShuntKernelMessage ( sdataobject, &pQueueShuntItem);
	if (CROSSRFID_MESSAGETOBEPOSTED == ui8shuntstatus )
	{
		xQueueSend (sKernel_QueuePointer.pKernelQueue,&pQueueShuntItem,0);
	}
	else
	{
		/* do nothing*/
	}
	return ui8status;

}

/***************************************************************************//**
 * @brief 		This function is called after a serial request was sent to
 * the serial interface. According to the response of the serial task the
 * function sent to the serial interface the response.
 * @param[in] pQueueItems : the received notification
 * @return 		CROSSRFID_SUCCESSCODE :  the function is successful
 ******************************************************************************/
uint8_t serial_ProcessMeasureReady (Kernel_QueueItem_struct *pQueueItems )
{
kernel_DataExchange_Type *psDataVehicle = (kernel_DataExchange_Type*) pQueueItems->pData ;
int16_t *pui16AccelMeas ;
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	switch (  psDataVehicle->ui8ObjectId )
	{
		case KERNEL_OBJECTCODE_ACCELEROMETER :
			pui16AccelMeas = (int16_t *) psDataVehicle->pui8data;
			srvM2M_PrintAccelmeasurement (pui16AccelMeas[0], pui16AccelMeas[1] , pui16AccelMeas[2]);

		break;
		default :
		break;
	}

	return ui8status;
}

/***************************************************************************//**
 * @brief 		this function is called when the loop measurement is over and
 * the sensor task sent a message to inform the serial task. The task will send
 * a new message to request the FFT measurement of the data.
 * @param[in/out] pQueueItems : the received notification
 * @return 		CROSSRFID_SUCCESSCODE :  the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  the function is successful and
 * the compuationoftheFFT can be started.
 ******************************************************************************/
uint8_t serial_ProcessLogReady (Kernel_QueueItem_struct *pQueueItems )
{
uint16_t ui16sensorID = (uint16_t*)pQueueItems->pData[0]; //ui16QueueBufferMsg[0] = KERNEL_SENSOR_ID_ACCELERO;
uint16_t ui16statusID = (uint16_t*)pQueueItems->pData[1]; // ui16QueueBufferMsg[1] = (uint16_t)(srvActRec_status.aeSensorFFTStatus[KERNEL_SENSOR_ID_ACCELERO]);
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/*when the loop measurement is over*/
	if ( (KERNEL_SENSOR_ID_ACCELERO == ui16sensorID) &&
		(SRVACTREC_STATUS_DONE == ui16statusID))
{
		uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;
		aui16data [0]= KERNEL_COMMANDCODE_SET; 				/* add the write operation */
		aui16data [1]= KERNEL_SENSOR_ID_ACCELERO;			/* add the sensor ID */
		aui16data [2]= KERNEL_MEASID_XRAWAXIS ;				/* add the measurement ID */

		(*pQueueItems) = (Kernel_QueueItem_struct) {   KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SERIALTASKID),
														KERNEL_MESSAGEID_COMPUTEFFT,4, (uint8_t *)aui16data};

		srvm2m_SendString ("The measurement is now over. the FFT computation is going to start \n");

	}
	else
	{
		srvm2m_SendString ("Error: The measurement is now over but the status is not the one expected \n");
	}

	return ui8status;

}

/***************************************************************************//**
 * @brief 		This function is called after a serial request was sent to
 * the serial interface. According to the response of the serial task the
 * function sent to the serial interface the response.
 * @param[in] pQueueItems : the received notification
 * @return 		CROSSRFID_MESSAGETOBEPOSTED :  a message should be posted
 ******************************************************************************/
uint8_t serial_ProcessSensorResponse (Kernel_QueueItem_struct *pQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
kernel_DataExchange_Type *psResponse = (kernel_DataExchange_Type*) pQueueItems->pData ;

	switch (psResponse->ui8CommandId)
	{
		case KERNEL_COMMANDCODE_GET: /* when the serial request was a get command */
			if (( KERNEL_ACTIONCODE_STATE == psResponse->ui8ActionId) &&
				( KERNEL_OBJECTCODE_ACCELEROMETER == psResponse->ui8ObjectId))
			{
				switch (psResponse->pui8data[0])
				{
					case SRVACTREC_STATUS_ONGOING :
						srvm2m_SendString ("on going \n");
					break;

					default:
						srvm2m_SendString ("not active \n");
					break;
				}
			}
			else if (( KERNEL_ACTIONCODE_STATE == psResponse->ui8ActionId) &&
					( KERNEL_OBJECTCODE_ACTIVITYCOUNTER == psResponse->ui8ObjectId))
			{
				/* see srvActRec_ProcessSerialRequest */
				srvM2M_PrintActivityCounterStatus ((bool)psResponse->pui8data[0] , (uint16_t) psResponse->pui8data[1] ,(uint16_t) psResponse->pui8data[2]);
			}
			else
			{ /* do nothing*/ }

		break;
		case KERNEL_COMMANDCODE_SET: /* when the serial host request was a set command*/
			if ( CROSSRFID_SUCCESSCODE == psResponse->pui8data[0] )
			{
				srvm2m_SendString ("ok\n");
			}
			else
			{
				srvm2m_SendString ("error\n");
			}
		break;
		default:
		break;
	}

	return ui8status;
}


#endif
