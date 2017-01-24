/*******************************************************************************
 * @file srvActivityRecorder.c
 * @brief this file defines the command set for the activity recorder
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvBubbleLevel.h"

#if (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)

static uint16_t			srvbubble_ui16QueueBufferMsg[7];	/* bufffer for queue message*/
/*===========================================================================================================
						Private variables definition
===========================================================================================================*/

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/


/*===========================================================================================================
						Public functions definition
===========================================================================================================*/

/***************************************************************************//**
 * @brief 		this function measures the magnetic field and updtaed the
 * 				threshold
 * @details		the measurement is carried out 10 times in a loop. Between
 * 				each measurement the power supply of the I2C bus is powered
 * 				down.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_INITSENSOR_ERROR : the function is not successful.
 * the measurement floor is updated to its default value
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful.
*******************************************************************************/
uint8_t srvBubble_ProcessAcceleromterIRQ ( Kernel_QueueItem_struct *sQueueItem )
{
prtadxl363_status_union pustatus;
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	sQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	sQueueItem->ui16notification = KERNEL_MESSAGEID_ACCELEROIRQ;
	sQueueItem->ui16NbByte = 1;


	prtadxl363_ReadStatus ( &(pustatus.ui8status) ); 		/* clean the IRQ from the point of view of the Adxl363*/

	if (pustatus.sstatus.bawake == 1 )						/* an activity is detected */
	{
		srvbubble_ui16QueueBufferMsg [0]= 1;
	}
	else
	{
		srvbubble_ui16QueueBufferMsg [0]= 0;

	}
	sQueueItem->pData = (uint8_t*) srvbubble_ui16QueueBufferMsg;
	return ui8status ;
}


/***************************************************************************//**
 * @brief 		this function configures the ADXL363 to act as a motion detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
uint8_t srvBubble_EnableMotionDetection ( bool bOnOrOff )
{
prtadxl363_status_union pustatus;
uint8_t status = CROSSRFID_INITSENSOR_ERROR;

	if (true == bOnOrOff)
	{
		prtadxl363_Init ();
		status = prtADXL363_CheckDeviceId ();
		if (CROSSRFID_SUCCESSCODE == status)
		{
			//letimerdrv_init ();								/* initialize the letimer to measure the activity */
			prtadxl363_InitiateMotionDetection ();			/* configure the Adxl */
			prtadxl363_EnableMotionIrq (true);					/* enable the IRQ*/
			prtadxl363_ReadStatus (&(pustatus.ui8status));	/* read the status to clear the IRQ of the adxl363*/
		}
		else
		{
			prtadxl363_Deinit ();
		}
	}
	else
	{
		interface_DisableGpioUsart1 ();
	}

	return status;
}

#endif


