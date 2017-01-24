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
#include "serial_dispatch.h"

/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
static uint8_t 	serial_ParseKernelNotification 	( Kernel_QueueItem_struct * psQueueItems );
static uint8_t 	serial_ParseSerialNotification 	( Kernel_QueueItem_struct * psQueueItems );
static uint8_t 	serial_ParseRFtaskNotification  ( Kernel_QueueItem_struct * psqueueItem );
static uint8_t 	serial_ParseSensorNotification ( Kernel_QueueItem_struct  * psQueueItem );
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
static uint8_t serial_ParseKernelNotification (  Kernel_QueueItem_struct  * psQueueItems )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItems->ui16notification)
	{

		case KERNEL_MESSAGEID_TASKINIT_YOURSELF : /* initialize the serial interface */
			ui8status = serial_ProcessInit(psQueueItems);
		break;

	#if USEM2MINTERFACE == 1 /* these IRQ handlers is dedicated to the M2M communication*/
		case KERNEL_GOTOSLEEPMODE:
			serial_processEndOfWakeupMode ();
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
static uint8_t serial_ParseSerialNotification ( Kernel_QueueItem_struct * psQueueItems )
{
uint8_t ui8status=CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItems->ui16notification)
	{
		/* ************************************************************************** */
		case KERNEL_MESSAGEID_DATAREADY:	/* accelerometer's data must be read from */
		/* ************************************************************************** */

		break;
		case KERNEL_MESSAGEID_RX:
			ui8status = serial_ProcessM2MRxmessage (psQueueItems);
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
static uint8_t serial_ParseRFtaskNotification ( Kernel_QueueItem_struct  * psQueueItem )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItem->ui16notification)
	{




		default :

			/* should not happen */
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
static uint8_t serial_ParseSensorNotification ( Kernel_QueueItem_struct  * psQueueItem )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Process any events that have been latched in the notified value. */
	switch (psQueueItem->ui16notification)
	{


		case KERNEL_MESSAGEID_SERIALREQUEST:
			ui8status = serial_ProcessSensorResponse (psQueueItem);
		break;

		default :

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
void serial_Dispatch ( void *pvParameters )
{
Kernel_QueueItem_struct sQueueItems;
BaseType_t ustatus;
uint8_t ui8status  = CROSSRFID_SUCCESSCODE;

	while (1)
	{
		ustatus = xQueueReceive (	sKernel_QueuePointer.pSerialQueue,		/* handle of the task's queue*/
									&sQueueItems,							/* pointer of the object*/
									portMAX_DELAY );						/* Block during 10 ms */


		if (ustatus == pdPASS) /* an object is available*/
		{
#if (USESWO==1)
			printf("Serial: ");
			kernel_processNotificationInText(sQueueItems.urecvsender, sQueueItems.ui16notification, sQueueItems.ui16NbByte);
#endif
			//switch ( ((sQueueItems.urecvsender & KERNEL_SENDER_MASK)>>KERNEL_SENDER_SHIFT) ) /* Process any events according to the sender */
			switch ( KERNEL_WHICH_SENDERID(sQueueItems.urecvsender)) /* Process any events according to the sender */
			{
				/* ******************************************************************** */
				case KERNEL_KERNELTASKID: /* the sender is the kernel					*/
				/* ******************************************************************** */
					ui8status = serial_ParseKernelNotification ( &sQueueItems );
				break;

				/* ******************************************************************** */
				case KERNEL_SERIALTASKID:	/* the sender is the Serial task 			*/
				/* ******************************************************************** */
					ui8status = serial_ParseSerialNotification ( &sQueueItems  );
				break;

				/* ******************************************************************** */
				case KERNEL_RFFRONTENDTASKID: /* the sender is the rf task 				*/
				/* ******************************************************************** */
					ui8status = serial_ParseRFtaskNotification ( &sQueueItems );
				break;

				/* ******************************************************************** */
				case KERNEL_SENSORTASKID: 	/* the sender is the sensor task			*/
				/* ******************************************************************** */
					ui8status = serial_ParseSensorNotification ( &sQueueItems );
				break;


				/* ******************************************************************** */
				default:
				/* ******************************************************************** */
				break;
			}

			if ( CROSSRFID_MESSAGETOBEPOSTED == ui8status) /*when the end of the process requests a messagebe posted*/
			{
				switch (KERNEL_WHICH_RECEIVERID(sQueueItems.urecvsender))
				{
					case KERNEL_KERNELTASKID :
						/* sends the message to kernel */
						xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItems,0);
					break;

					default:
						/* should not happen */
					break;
				}
			}else { /*do nothing*/}
		}

	}
}


