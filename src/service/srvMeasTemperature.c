/*******************************************************************************
 * @file srvMeasTemperature.c
 * @brief this file defines the command set for the temperature mesurement
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvMeasTemperature.h"


/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
static int8_t srvtemp_i8CurrentTemp;
static int16_t srvtemp_i16AverageTemp;
static srvtemp_TempBuffer_struct srvtemp_TempBuffer;

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
//srvtemp_object_struct assrvtemp_temppool[DATAACQ_NBDATAPOOLOBJECT];
/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
#if USE_TEMPERATURE_EM4325 == 0
static void srvtemp_MeasureOnchipTemp (int8_t * pi8Result);
#endif
static void 	srvtemp_callbackHigh		(int8_t temp, TEMPDRV_LimitType_t limit);
static void 	srvtemp_callbackLow			(int8_t temp, TEMPDRV_LimitType_t limit);
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
static uint8_t 	srvtemp_AverageAllTemperature	(void);
#endif
/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief User callback called in case of temperature above the high threshold
 *
 * @param[in] temp: Current store temperature
 * @param[in] limit: Limit type, refer to @ref TEMPDRV_LimitType_t.
 *
 ******************************************************************************/
static void srvtemp_callbackHigh(int8_t temp, TEMPDRV_LimitType_t limit)
{
	Kernel_QueueItem_struct sQueueItem;

	/* saves the temperature which triggers the alarm */
	srvtemp_i8CurrentTemp = (int8_t)temp; /* Todo: to protect ? */

	//sQueueItem.urecvsender 		= (((KERNEL_RFFRONTENDTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) |
	//								((KERNEL_KERNELTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK));
	sQueueItem.urecvsender			= KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID);
	sQueueItem.ui16NbByte 		= 1;
	sQueueItem.pData 			= (uint8_t*)&srvtemp_i8CurrentTemp;
	sQueueItem.ui16notification 	= KERNEL_MESSAGEID_SETHIGHTEMPALARM;

	/* sends the message to the RF task */
	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
}

/***************************************************************************//**
 * @brief User callback called in case of temperature above the high threshold
 *
 * @param[in] temp: Current store temperature
 * @param[in] limit: Limit type, refer to @ref TEMPDRV_LimitType_t.
 *
 ******************************************************************************/
static void srvtemp_callbackLow(int8_t temp, TEMPDRV_LimitType_t limit)
{
	Kernel_QueueItem_struct sQueueItem;

	/* saves the temperature which triggers the alarm */
	srvtemp_i8CurrentTemp = (int8_t)temp; /* Todo: to protect ? */

	sQueueItem.urecvsender 		= (((KERNEL_RFFRONTENDTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) |
									((KERNEL_KERNELTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK));
	sQueueItem.ui16NbByte 		= 1;
	sQueueItem.pData 			= (uint8_t*)&srvtemp_i8CurrentTemp;
	sQueueItem.ui16notification 	= KERNEL_MESSAGEID_SETLOWTEMPALARM;

	/* sends the message to the RF task */
	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/***************************************************************************//**
 * @brief 	Processes the temperature average of the values in the temperature
 * 			buffer.
 *
 * @details	The average is a static variable of the service.
 *
 * @return CROSSRFID_SUCCESSCODE: Average process successful
 * @return CROSSRFID_ERROR_UNSPECIFIED: overflow during the process
 ******************************************************************************/
static uint8_t srvtemp_AverageAllTemperature (void)
{
	int16_t i16temp;
	uint8_t ui8Status;

	ui8Status = srvtemp_ComputeAvgOnSectionOfTempBuffer(0,srvtemp_TempBuffer.ui16Nbelement,&i16temp);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvtemp_i16AverageTemp = i16temp;
	}

	return ui8Status;
}
#endif

#if USE_TEMPERATURE_EM4325 == 0
/***************************************************************************//**
 * @brief 		This function measures the temperature by using the on chip sensor
 * @param[in] 	none
 * @param[out] 	pi8Result : pointer to the measured temperature
 * @return 		none
 *
 ******************************************************************************/
static void srvtemp_MeasureOnchipTemp (int8_t * pi8Result)
{
	*pi8Result = TEMPDRV_GetTemp ();
	//*pi8Result = 0x1F;
}
#endif
/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/

/***************************************************************************//**
 * @brief 		This function initializes the service to measure the temperature
 *
 * @param[in]	bEnableorNot
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvtemp_init ( bool bEnableorNot )
{
	srvtemp_TempBuffer.ui16Nbelement = 0;

	srvtemp_i16AverageTemp = 0;

	/* Starts or Stops the onchip acquisitions (stop performs a reset counter) */
	//srvtemp_StartOnchipTempAcq(bEnableorNot);
	/* initializes the temperature service of the uC */
	//srvtemp_InitOnchipTemp();
}

/***************************************************************************//**
 * @brief This function measures the temperature by using the on chip sensor
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void srvtemp_InitOnchipTemp ( void )
{
	TEMPDRV_Init();	/* initialize the temperature driver */
	NVIC_SetPriority(EMU_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);
}



/***************************************************************************//**
 * @brief This function sets the period of the onchip temperature acquisitions
 *
 * @param[in] ui16Period: the value of the period
 * @param[out] ui16Unit: the unit of the period
 * @return True if the period is set else false
 *
 ******************************************************************************/
bool srvtemp_SetPeriodOnchipTemp (const uint16_t ui16Period,const uint16_t ui16Unit)
{
	bool 						bStatus = false;
	CRYOTIMER_Period_TypeDef 	nbCycle;
	CRYOTIMER_Presc_TypeDef 	prescaler;

	/* Converts the period (time) into a number of clock cycles */
	bStatus = cryodrv_ConvPeriodToNbCycleAndPresc(ui16Period,ui16Unit,&nbCycle,&prescaler);

	if(true == bStatus)
	{
		/* Sets the nb of cycles and the prescaler */
		cryodrv_SetPrescaler(prescaler);
		CRYOTIMER_PeriodSet((uint32_t)nbCycle);
	}

	return bStatus;
}

/***************************************************************************//**
 * @brief 		This function measures the temperature
 *
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_READSENSOR_ERROR: the temperature is invalid
 * @return		CROSSRFID_SUCCESSCODE: successful measurement
 * @return 		CROSSRFID_MESSAGETOBEPOSTED: the average must be written in
 * 				the EM4325 user memory.
 *   ******************************************************************************/
uint8_t srvtemp_MeasureTemperature ( void )
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
#if USE_TEMPERATURE_EM4325 == 1
	int16_t  i16Temperature;
	ui8Status = srvEM4325_MeasureTemperature (&i16Temperature);
#else
		int8_t i8temp;
		/* gets the on chip temperature */
		srvtemp_MeasureOnchipTemp(&i8temp);
#endif

	/* Update the buffer and the number of samples available */
	if(DATAACQ_MEASBUFFER_NBELEMENT > srvtemp_TempBuffer.ui16Nbelement)
	{
		srvtemp_TempBuffer.ai16MeasBuffer[srvtemp_TempBuffer.ui16Nbelement] = i16Temperature;
		srvtemp_TempBuffer.ui16Nbelement ++;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#if (1 == BACKUP_RESTORE_FLASH)
		/* variable in ram that will allow the uc to write in flash */
		nvm_ui16TempBuffer[(srvtemp_TempBuffer.ui16Nbelement-1)]  	= i16Temperature;
		nvm_ui16NbTempSamples 										= srvtemp_TempBuffer.ui16Nbelement;
#endif
#endif
	}
	/* Overflow */
	else
	{
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
		/* average on the temperature */
		ui8Status = srvtemp_AverageAllTemperature();

		if(CROSSRFID_SUCCESSCODE == ui8Status)
		{
			/* the average must be written in the user memory */
			ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
		}

		/* clear the previous data */
		memset (srvtemp_TempBuffer.ai16MeasBuffer, PROTOCOL_EM4325_TEMERATURE_0_KELVIN , sizeof (srvtemp_TempBuffer.ai16MeasBuffer));
		srvtemp_TempBuffer.ui16Nbelement = 0;
#endif
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief This function starts or stops the cryotimer
 *
 * @param[in] bStart: Start or Stop cryotimer
 *
 ******************************************************************************/
void srvtemp_StartOnchipTempAcq (const bool bStart)
{
	CRYOTIMER_Enable(bStart);
}

/***************************************************************************//**
 * @brief This function connects the TEMPDRV Callback linked to an alarm threshold

 * @param[in] temp Temperature to trigger on.
 * @param[in] bIsHighThreshold: the limit ('1' is high, else low)
 *
 ******************************************************************************/
void srvtemp_SetTempAlarmCB (int8_t i8Thresh, bool bIsHighThreshold)
{
	if(true == bIsHighThreshold)
	{
		TEMPDRV_RegisterCallback(i8Thresh,TEMPDRV_LIMIT_HIGH,srvtemp_callbackHigh);
	}
	else
	{
		TEMPDRV_RegisterCallback(i8Thresh,TEMPDRV_LIMIT_LOW,srvtemp_callbackLow);
	}
}

/***************************************************************************//**
 * @brief This function disconnects the TEMPDRV Callback linked to an alarm threshold

 * @param[in] bIsHighThreshold: the limit ('1' is high, else low)
 *
 ******************************************************************************/
void srvtemp_RemoveTempAlarmCB (bool bIsHighThreshold)
{
	if(true == bIsHighThreshold)
	{
		TEMPDRV_UnregisterCallback(srvtemp_callbackHigh);
	}
	else
	{
		TEMPDRV_UnregisterCallback(srvtemp_callbackLow);
	}
}


/***************************************************************************//**
 * @brief 		This function is the callback for the RTCC timer for the thermometer
 * measurement
 * @note 		this function runs in a interrupt context
 * @param[in]	RTCDRV_TimerID_t : Id of the RTCC timer that triggers the IRQ
 * @param[in]	user : pointer on the user data
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
void srvtemp_ThermoCB ( void  )
{
	Kernel_QueueItem_struct sQueueItem ={ 	KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
											KERNEL_MESSAGEID_GETTEMP,
											0,
											NULL};
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* We have not woken a task at the start of the ISR.*/

#if 0
#if USE_TEMPERATURE_EM4325 == 1
int16_t  pi16Temperature;
		srvEM4325_MeasureTemperature (&pi16Temperature);
#else
		/* gets the on chip temperature */
		srvtemp_MeasureOnchipTemp((int8_t*)&(serial_sSample.tui16Data[serial_sSample.ui16Count]));
#endif


	srvtemp_TempBuffer.aui16MeasBuffer[srvtemp_TempBuffer.ui16Nbelement] = pi16Temperature;
	srvtemp_TempBuffer.ui16Nbelement ++;

#endif

	/* sends the message to the RF task */
	xQueueSendFromISR (sKernel_QueuePointer.pSensorQueue,&sQueueItem,&xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD ();
	}
}


/***************************************************************************//**
 * @brief 		This function returns the pointer on the first available
 * temperature buffer
 *
 * @param[in]	none
 * @param[out] 	ui8pdata : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvtemp_GetBufferTemp ( int8_t **ppi8pdata )
{
	(* ppi8pdata) = (int8_t*)(&srvtemp_TempBuffer.ai16MeasBuffer[0]);
}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the first avaliable
 * temperature buffer
 *
 * @param[in]	none
 * @param[out] 	ui8pdata : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvtemp_GetNbMeas ( uint16_t *ui16NbMeasurement )
{
	(* ui16NbMeasurement) = srvtemp_TempBuffer.ui16Nbelement;
}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the first avaliable
 * temperature buffer
 *
 * @param[in]	none
 * @param[out] 	ui8pdata : pointer on the buffer data of the temperature
 * @return 		true the alarm is on
 ******************************************************************************/
bool srvEM4325_IsAlarmOn ( uint8_t ui8type, uint8_t ui8IsEnable, int16_t i16CurrentTeperature , int16_t i16Threshold )
{
bool bIsAlarmOn = false ;

	if ( true == ui8IsEnable)
	{
		switch (ui8type)
		{
			case SRVTEMP_ALARMTYPE_LOW:
				if (i16CurrentTeperature<i16Threshold)
				{
					bIsAlarmOn = true;
				}
			break;
			case SRVTEMP_ALARMTYPE_HIGH:
				if (i16CurrentTeperature>i16Threshold)
				{
					bIsAlarmOn = true;
				}
			break;
		}
	}
	return bIsAlarmOn;
}

#if USE_TEMPERATURE_EM4325 == 1

/***************************************************************************//**
 * @brief 		this function sends to the RF task a message to inform it about the
 * alarm status
 * @note 	copy of srvtemp_callbacHigh
 * @param[in] 	temp: Current store temperature
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvtemp_SendHighTempAlarmState (const int16_t i16temp)
{
	Kernel_QueueItem_struct sQueueItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID),
											KERNEL_MESSAGEID_SETHIGHTEMPALARM,
											2,
											(uint8_t*) (uint32_t )i16temp };

	/* sends the message to the RF task */
	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
}

/***************************************************************************//**
 * @brief 		this function sends to the RF task a message to inform it about the
 * alarm status
 * @note 	copy of srvtemp_callbackLow
 * @param[in] 	temp: Current store temperature
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvtemp_SendLowTempAlarmState(const int16_t i16temp)
{
	Kernel_QueueItem_struct sQueueItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_KERNELTASKID),
											KERNEL_MESSAGEID_SETLOWTEMPALARM,
											2,
											(uint8_t*) (uint32_t )i16temp };

	/* sends the message to the RF task */
	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/***************************************************************************//**
* @brief 		This function pasts the flash temperature objects (buffer +
* 				number of samples in the buffer) into the ram.
*
* @details		It reads the data in flash (samples values and the number of samples)
* 				and fill the sensor	buffers in RAM according to the FLASH memory mapping.
*
* @param[in]	none
* @param[out] 	none
* @return 		CROSSRFID_SUCCESSCODE: successful read operation
* @return 		CROSSRFID_ERROR_NVM_READ: read operation failed
* @return 		CROSSRFID_ERROR_NVM_ISERASE: no read operation, the flash is empty
 ******************************************************************************/
uint8_t srvtemp_RestoreFlashToRamObjects( void )
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint8_t i8Idx;

	/* TEMPERATURE backup */
	ui8Status = srvFlashMemory_ReadAllTempObjects();

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		/* restores the number of temperature samples */
		srvtemp_TempBuffer.ui16Nbelement = nvm_ui16NbTempSamples;

		/* pasts the FLASH temperature buffer to RAM temperature buffer */
		for(i8Idx = 0; ((i8Idx < nvm_ui16NbTempSamples) && (i8Idx < DATAACQ_MEASBUFFER_NBELEMENT)); i8Idx++)
		{
			srvtemp_TempBuffer.ai16MeasBuffer[i8Idx] = nvm_ui16TempBuffer[i8Idx];
		}
	}
	else
	{
		/* do not fill the buffers in RAM */
	}

	return ui8Status;
}

/***************************************************************************//**
* @brief 		Getter of the last computed temperature average
* @return 		average on the temperature
 ******************************************************************************/
int16_t srvtemp_GetLastTempAverage( void )
{
	return srvtemp_i16AverageTemp;
}

/***************************************************************************//**
* @brief 		Computes the temperature average only on a buffer section.
*
* @param[in]	ui16IdxSample: The first sample pointed by the section cursor
* @param[in]	ui16NbSample: The number of samples in the section
* @param[out] 	pi16Avg: the average
*
* @return 		CROSSRFID_SUCCESSCODE
* @return 		CROSSRFID_ERROR_UNSPECIFIED
 ******************************************************************************/
uint8_t srvtemp_ComputeAvgOnSectionOfTempBuffer( const uint16_t ui16IdxSample, const uint16_t ui16NbSample, int16_t * pi16Avg)
{
	uint16_t ui16Idx = 0;
	uint32_t ui32Sum = 0; /* allows the function to add all samples */
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	/* saves this number in order to decrement it in a temporal variable if the temperature is invalid for some indexes,
	 * see the sum calculation below */
	uint16_t ui16NbTemp = ui16NbSample;

	/* invalid average */
	*pi16Avg = PROTOCOL_EM4325_TEMERATURE_0_KELVIN;

	/*
	 * Test the "ui32Temp" overflow before computing the average
	 *     SizeBuffer * SizeDataInBuffer16bit <= MaxSum
	 * <=> SizeBuffer <= MaxSum / SizeDataInBuffer16bit
	 * <=> SizeBuffer <= 0xFFFFFFFF / (0xFFFF/2)
	 * <=> SizeBuffer <= 0x20002
	 */
	if((ui16NbSample <= 0x20002) && (((uint32_t)ui16IdxSample+ui16NbSample) <= DATAACQ_MEASBUFFER_NBELEMENT) && (ui16NbSample > 0))
	{
		/* performs the sum */
		for(ui16Idx = ui16IdxSample; ((ui16Idx < (ui16IdxSample+ui16NbSample)) && (ui16NbTemp > 0)); ui16Idx++)
		{
			/* the data is valid */
			if(srvtemp_TempBuffer.ai16MeasBuffer[ui16Idx] != PROTOCOL_EM4325_TEMERATURE_0_KELVIN)
			{
				ui32Sum += srvtemp_TempBuffer.ai16MeasBuffer[ui16Idx];
			}
			/* else: data not take into account */
			else
			{
				ui16NbTemp--;
			}
		}
		/* Test the "srvtemp_i16AverageTemp" overflow before dividing
		 * 	   MaxSum <= SizeVariableAverageInt16bit * MaxNbSample
		 * <=> MaxSum <= (0xFFFF/2) * 0xFFFF
		 * <=> MaxSum <= 0x7FFF0000
		 */
		if ((ui32Sum <= 0x7FFF0000) && (ui16NbTemp > 0))
		{
			*pi16Avg = (int16_t) (ui32Sum / (uint32_t)ui16NbTemp);
		}
		else
		{
			ui8Status = CROSSRFID_ERROR_UNSPECIFIED;
		}
	}
	/* impossible to compute the average of the temperature */
	else
	{
		ui8Status = CROSSRFID_ERROR_UNSPECIFIED;
	}

	return ui8Status;
}
#endif

#endif
