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
kernel_DataExchange_Type *prtM2M_SerialCommand;

	/* identify the different fields of the serial command*/
	ui8status = srvm2m_ProcessRxMessage (  pQueueItems->pData ,prtM2M_SerialCommand );

	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* save the fields of the command */
		memcpy(aui8QueueData , prtM2M_SerialCommand, strlen (prtM2M_SerialCommand));
		/* create the queue object */
		pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SERIALTASKID);
		pQueueItems->ui16notification = KERNEL_MESSAGEID_SERIALREQUEST;
		pQueueItems->pData = aui8QueueData; /* change the memory area to send the queue object*/
		pQueueItems->ui16NbByte=strlen (prtM2M_SerialCommand);
		ui8status = CROSSRFID_MESSAGETOBEPOSTED;
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
			if ( KERNEL_ACTIONCODE_STATE == psResponse->ui8ActionId)
			{
				switch (psResponse->ui8pdata[0])
				{
					case SRVACTREC_STATUS_ONGOING :
						srvm2m_SendString ("on going \n");
					break;

					default:
						srvm2m_SendString ("not active \n");
					break;
				}
			}
			else
			{ /* do nothing*/ }

		break;
		case KERNEL_COMMANDCODE_SET: /* when the serial host request was a set command*/
			if ( CROSSRFID_SUCCESSCODE == psResponse->ui8pdata )
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
