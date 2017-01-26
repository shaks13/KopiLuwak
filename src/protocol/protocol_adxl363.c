/*******************************************************************************
 * @file protocol_adxl363.c
 * @brief this function set is codec for the adxl 363
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_adxl363.h"

static SPIDRV_Handle_t prtadxl363_psSpiHandle = NULL;

uint8_t protocol_adxl363Buffer [PROTOCOL_ADXL363_BUFFERSIZE];

static uint8_t prtADXL363_WriteADXL363Register (const uint8_t ui8Adress, const uint8_t ui8NbByteToWrite );
static void prtADXL363_ReadADXL363Register (const uint8_t ui8address,const uint8_t ui8NbByteToRead, uint8_t **ui8MemStatus);


/***************************************************************************//**
 * @brief		this function initializes the peripheral to communicate
 * with the Adxl363
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prtadxl363_Init ( void )
{

#if (USESTARTERKIT == 1)
	SPIDRV_Init_t sInitData = SPIDRV_MASTER_USART0_FORADXL363; /* Configuration data for SPI master using USART0 */
#elif (USECROSSTAG==1)
	SPIDRV_Init_t sInitData = SPIDRV_MASTER_USART1_FORADXL363; /* Configuration data for SPI master using USART0 */
#endif

	if (prtadxl363_psSpiHandle != NULL)
	{
		prtadxl363_Deinit ();
	}
	/* configure psSpiHandle thanks to prtLIS2DH12_psSpiHandle */
	prtadxl363_psSpiHandle = &(interface_sSpiHandle[INTERFACE_SENSOR2]);
	/* Initialize a instance "interface_psSpiHandle" and the USARTx according to sInitData*/
	SPIDRV_Init( prtadxl363_psSpiHandle , &sInitData);


}

/***************************************************************************//**
 * @brief 		this function deinitializes the peripheral to communicate
 * with the Adxl363
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prtadxl363_Deinit ( void )
{
	/* Deinitialize a instance "interface_psSpiHandle" and the USARTx according to sInitData*/
	SPIDRV_DeInit( prtadxl363_psSpiHandle );
	prtadxl363_psSpiHandle= NULL;

	interface_DisableGpioUsart1 ();
}

/**************************************************************************//**
 * @brief this function reads one or some bytes from a register
 * @param[in]  	ui8address : register address
 * @param[in]  	ui8NbByteToRead : NbByte to read
 * @param[out] 	pui8MemStatus: pointer on the sensor response
 * @return 		none
 *****************************************************************************/
static void prtADXL363_ReadADXL363Register (const uint8_t ui8address,const uint8_t ui8NbByteToRead, uint8_t ** pui8Response)
{

	protocol_adxl363Buffer[0] = PROTOCOL_ADXL363CMDCODE_READ;
	protocol_adxl363Buffer[1] = ui8address;

	/* start dma transfers and wait their completion */
	SPIDRV_MTransferB( 	prtadxl363_psSpiHandle,
						protocol_adxl363Buffer,
						protocol_adxl363Buffer,
						ui8NbByteToRead+PROTOCOL_ADXL363_NBBYTE_READ);

	(*pui8Response) = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_NBBYTE_READ]);
}

/**************************************************************************//**
 * @brief 		this function sends a Write to the adxl 363
 * @note 		in order to avoid a memory copy the data to be written
 * should be copied at the right index of the buffer protocol_adxl363Buffer
 * @param[in]  	ui16Adress: the address of the first byte to write
 * @param[in] 	ui8NbByteToWrite: the number of bytes to write
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the writing operation is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the function is not successful
 *****************************************************************************/
static uint8_t prtADXL363_WriteADXL363Register (const uint8_t ui8Adress, const uint8_t ui8NbByteToWrite )
{
uint8_t ui8status = CROSSRFID_INITSENSOR_ERROR;
Ecode_t estatus ;
	protocol_adxl363Buffer[0] = PROTOCOL_ADXL363CMDCODE_WRITE;
	protocol_adxl363Buffer[1] = ui8Adress;

	/* start dma Tx and wait their completion */
	estatus = SPIDRV_MTransmitB( 	prtadxl363_psSpiHandle,
						protocol_adxl363Buffer,
						ui8NbByteToWrite+PROTOCOL_ADXL363_NBBYTE_WRITE);
	if (ECODE_EMDRV_SPIDRV_OK == estatus )
	{
		ui8status = CROSSRFID_SUCCESSCODE;
	}else {/* do nothing */ }

	return ui8status;
}

/**************************************************************************//**
 * @brief 		this function checks the Id of the ADxl 363
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
uint8_t prtADXL363_CheckDeviceId ( void )
{
uint8_t *pui8Response;
uint8_t status = CROSSRFID_SUCCESSCODE;

	prtADXL363_ReadADXL363Register (PROTOCOL_ADXL363_REGISTER_DEVID_AD, 1 , &pui8Response);
	if (pui8Response[0] != PROTOCOL_ADXL363_ID)
	{
		status = CROSSRFID_INITSENSOR_ERROR;
	}
	else {/*do nothing*/}
	return status;
}

/**************************************************************************//**
 * @brief 		this function checks the Id of the ADxl363
 * @param[in]  	none
 * @param[out] 	pui8Response : pointer on the Adxl response
 * @return 		none
 *****************************************************************************/
void prtadxl363_ReadXYZaxis ( uint8_t **pui8Response )
{

	//(*pui8Response) = protocol_adxl363Buffer;
	prtADXL363_ReadADXL363Register (PROTOCOL_ADXL363_REGISTER_XDATA, PROTOCOL_REGISTER_3XYZDATA_LENGTH , pui8Response);

}

/**************************************************************************//**
 * @brief 		this function checks the Id of the ADxl363
 * @param[in]  	none
 * @param[out] 	pui8Response : pointer on the Adxl response
 * @return 		none
 *****************************************************************************/
void prtadxl363_ReadXYZaxisExtended ( uint8_t **pui8Response )
{

	//(*pui8Response) = protocol_adxl363Buffer;
	//(*pui8Response) = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_NBBYTE_READ]);
	prtADXL363_ReadADXL363Register (PROTOCOL_ADXL363_REGISTER_XDATA_L, PROTOCOL_REGISTER_3XYZDATAEXT_LENGTH , pui8Response);

}

/**************************************************************************//**
 * @brief 		this function checks the Id of the ADxl 363
 * @param[in]  	none
 * @param[out] 	pui8Response : pointer on the Adxl response
 * @return 		none
 *****************************************************************************/
void prtadxl363_ReadTemperature ( uint8_t *pui8Response )
{
	prtADXL363_ReadADXL363Register (PROTOCOL_ADXL363_REGISTER_TEMP_L, PROTOCOL_REGISTER_TEMP_LENGTH , &pui8Response);
	//pui8Response = protocol_adxl363Buffer;

}


/**************************************************************************//**
 * @brief 		this function reads the status register
 * @details		the union prtadxl363_status_union allows to retrieve the
 * different bit
 * @param[in]  	none
 * @param[out] 	pui8Response : pointer on the ADXL363 response
 * @return 		none
 *****************************************************************************/
void prtadxl363_ReadStatus ( uint8_t *pui8Response )
{
	prtADXL363_ReadADXL363Register (PROTOCOL_ADXL363_REGISTER_STATUS, PROTOCOL_REGISTER_STATUS_LENGTH , &pui8Response);
}

/**************************************************************************//**
 * @brief 		this function puts the ADXL363 to the sleep state
 * @param[in]  	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtadxl363_SleepAdxl363 ( void )
{
prtadxl363_PowerRegister_union uPowerRegisters;
uint8_t  * pui8ByteToWrite = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_COMMAND_DATA_INDEX]);

	/* fill the power register */
	uPowerRegisters.ui8power = PROTOCOL_ADXL363_REGISTER_RESET_POWER;
	uPowerRegisters.spower.bMeasure=PROTOCOL_ADXL363_MEASURE_STANDBY;
	/* copy it in the com buffer */
	pui8ByteToWrite [0] = uPowerRegisters.ui8power;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_POWER_CTL, PROTOCOL_REGISTER_POWER_LENGTH );

}

/**************************************************************************//**
 * @brief 		this function initializes the sensor to work as a accelerometer
 * log
 * @details
 * @param[in]  	bOnOrOff
 * @param[in]  	emeasfrequency :theODRfrequency of the accelerometer
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the accelerometer is correctly initialized
 * @return 		CROSSRFID_INITSENSOR_ERROR : the accelerometer is not correctly initialized
 * @return 		other : the accelerometer is not correctly initialized
 *****************************************************************************/
uint8_t prtadxl363_InitLogMeasurement ( const bool bOnOrOff , const protocol_adxl363_odr emeasfrequency)
{
uint8_t  * pui8ByteToWrite = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_COMMAND_DATA_INDEX]); /* the data are copied in the 3rd byte*/
uint8_t 	ui8status = CROSSRFID_SUCCESSCODE;
prtadxl363_registers_struct	prtadxl363_registers;

	if ( true == bOnOrOff)
	{
		/* fill the filter register */

		prtadxl363_registers.ufilter.ui8filter 			= PROTOCOL_ADXL363_REGISTER_RESET_FILTER;
		//prtadxl363_registers.ufilter.sfilter.b3ODR 		= PROTOCOL_ADXL363_ODR_50HZ;			/* Output data rate */
		prtadxl363_registers.ufilter.sfilter.b3ODR 		= emeasfrequency;
		prtadxl363_registers.ufilter.sfilter.brange 	= PROTOCOL_ADXL363_RANGE_2G;			/* measurement range */
		prtadxl363_registers.ufilter.sfilter.bHalfBw 	= PROTOCOL_ADXL363_HALFBW_ON;			/* activate the antialliasing filter */
		/* copy it in the com buffer */
		pui8ByteToWrite [0] = prtadxl363_registers.ufilter.ui8filter;
		/* Write it */
		ui8status=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_FILTER_CTL, PROTOCOL_REGISTER_FILTER_LENGTH );
		/* copy it in the com buffer */
		pui8ByteToWrite [0] = 40; // 100 * 1/50 Hz = 2 s
		/* Write it */
		ui8status|=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_TIME_ACT, PROTOCOL_REGISTER_TIMEACT_LENGTH );


		/* fill the activity/inactivity control register */
		prtadxl363_registers.uactivity.ui8activity 					= PROTOCOL_ADXL363_REGISTER_RESET_ACTIVITY;
		prtadxl363_registers.uactivity.sactivity.b2LinkLoop 		= PROTOCOL_ADXL363_LINKLOOP_DEFAULT;
		prtadxl363_registers.uactivity.sactivity.bActivityEnable 	= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.uactivity.sactivity.bInactivityEnable 	= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.uactivity.sactivity.bActivityReference = PROTOCOL_ADXL363_REFERENCEMODE;
		prtadxl363_registers.uactivity.sactivity.bInactivityReference = PROTOCOL_ADXL363_REFERENCEMODE;
		/* copy it in the com buffer */
		pui8ByteToWrite [0] = prtadxl363_registers.uactivity.ui8activity;
		/* Write it */
		ui8status|=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_ACT_INACT_CTL, PROTOCOL_REGISTER_ACTIVITY_LENGTH );

		/* enable the interruptions */
		prtadxl363_registers.uinterupt.ui8interupt 					= PROTOCOL_ADXL363_REGISTER_RESET_IRQ;
		prtadxl363_registers.uinterupt.sinterupt.bactivity 			= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.uinterupt.sinterupt.bawake 			= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.uinterupt.sinterupt.binactivity 		= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.uinterupt.sinterupt.bDataReady 		= PROTOCOL_ADXL363_ENABLE;
		/* copy it in the com buffer */
		pui8ByteToWrite [0] = prtadxl363_registers.uinterupt.ui8interupt;
		/* Write it */
		ui8status|=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_INTMAP1, PROTOCOL_REGISTER_INTERUPTMAP1_LENGTH );

		/* same for the 2nd Irq*/
		pui8ByteToWrite [0] = prtadxl363_registers.uinterupt.ui8interupt;
		/* Write it */
		ui8status|=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_INTMAP2, PROTOCOL_REGISTER_INTERUPTMAP1_LENGTH );


		/* fill the power register */
		prtadxl363_registers.upower.ui8power 			= PROTOCOL_ADXL363_REGISTER_RESET_POWER;
		prtadxl363_registers.upower.spower.bAdcEnable	= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.upower.spower.bWakeUp 		= PROTOCOL_ADXL363_DISABLE;
		prtadxl363_registers.upower.spower.b2lownoise 	= PROTOCOL_ADXL363_LOWNOISE_NORMAL;
		prtadxl363_registers.upower.spower.bMeasure 	= PROTOCOL_ADXL363_MEASURE_MEASUREMENTMODE;
		prtadxl363_registers.upower.spower.bautoSleep 	= PROTOCOL_ADXL363_DISABLE;
		/* copy it in the com buffer */
		pui8ByteToWrite [0] = prtadxl363_registers.upower.ui8power;
		/* Write it */
		ui8status|=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_POWER_CTL, PROTOCOL_REGISTER_POWER_LENGTH );

		pui8ByteToWrite [0] = 0x00;
		ui8status|=prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_SELF_TEST, PROTOCOL_REGISTER_POWER_LENGTH );
	}
	else
	{
		prtADXL363_Reset ();
	}

	return ui8status;
}

/**************************************************************************//**
 * @brief 		this function writes the sensor register to enable the
 * 				motion detection
 * @details
 * @param[in]  	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtadxl363_InitiateMotionDetection ( void )
{
uint8_t  * pui8ByteToWrite = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_COMMAND_DATA_INDEX]); /* the data are copied in the 3rd byte*/

prtadxl363_registers_struct	prtadxl363_registers;

	/* fill the filter register */
	prtadxl363_registers.ufilter.ui8filter 			= PROTOCOL_ADXL363_REGISTER_RESET_FILTER;
	prtadxl363_registers.ufilter.sfilter.b3ODR 		= PROTOCOL_ADXL363_ODR_400HZ;			/* Output data rate */
	prtadxl363_registers.ufilter.sfilter.brange 	= PROTOCOL_ADXL363_RANGE_8G;			/* measurement range */
	prtadxl363_registers.ufilter.sfilter.bHalfBw 	= PROTOCOL_ADXL363_HALFBW_ON;			/* activate the antialliasing filter */
	/* copy it in the com buffer */
	pui8ByteToWrite [0] = prtadxl363_registers.ufilter.ui8filter;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_FILTER_CTL, PROTOCOL_REGISTER_FILTER_LENGTH );


	/* copy it in the com buffer */
	pui8ByteToWrite [0] = 1; // 40 * 1/400 Hz = 0.1 s
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_TIME_ACT, PROTOCOL_REGISTER_TIMEACT_LENGTH );

	/* copy it in the com buffer */
	pui8ByteToWrite [0] = 100; // 100 * 1/400 Hz = 2 s
	pui8ByteToWrite [1] = 0;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_TIME_INACT_L, PROTOCOL_REGISTER_TIMEINACT_LENGTH );


	pui8ByteToWrite [0] = 150 ;  /* Activity threshold = 50 mg (=0xFA) */
	pui8ByteToWrite [1] = 0x00;
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_THRESH_ACT_L, PROTOCOL_REGISTER_FILTER_LENGTH );

	pui8ByteToWrite [0] = 100; /* inactivity threshold = 20 mg*/
	pui8ByteToWrite [1] = 0x00;
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_THRESH_INACT_L, PROTOCOL_REGISTER_FILTER_LENGTH );

	/* fill the activity/inactivity control register */
	prtadxl363_registers.uactivity.ui8activity 					= PROTOCOL_ADXL363_REGISTER_RESET_ACTIVITY;
	prtadxl363_registers.uactivity.sactivity.b2LinkLoop 		= PROTOCOL_ADXL363_LINKLOOP_LOOP;	/* In linked mode, each interrupt must be serviced by a host processor before the next interrupt is enabled.*/
	prtadxl363_registers.uactivity.sactivity.bActivityEnable 	= PROTOCOL_ADXL363_ENABLE;
	prtadxl363_registers.uactivity.sactivity.bInactivityEnable 	= PROTOCOL_ADXL363_ENABLE;
	prtadxl363_registers.uactivity.sactivity.bActivityReference = PROTOCOL_ADXL363_REFERENCEMODE;
	prtadxl363_registers.uactivity.sactivity.bInactivityReference = PROTOCOL_ADXL363_REFERENCEMODE;
	/* copy it in the com buffer */
	pui8ByteToWrite [0] = prtadxl363_registers.uactivity.ui8activity;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_ACT_INACT_CTL, PROTOCOL_REGISTER_ACTIVITY_LENGTH );

	/* enable the interruptions */
	prtadxl363_registers.uinterupt.ui8interupt 					= PROTOCOL_ADXL363_REGISTER_RESET_IRQ;
	prtadxl363_registers.uinterupt.sinterupt.bactivity 			= PROTOCOL_ADXL363_ENABLE;
	prtadxl363_registers.uinterupt.sinterupt.bawake 			= PROTOCOL_ADXL363_ENABLE;
	prtadxl363_registers.uinterupt.sinterupt.binactivity 		= PROTOCOL_ADXL363_ENABLE;
	prtadxl363_registers.uinterupt.sinterupt.bintlow 			= PROTOCOL_ADXL363_DISABLE;
	prtadxl363_registers.uinterupt.sinterupt.bDataReady 		= PROTOCOL_ADXL363_DISABLE;

	/* copy it in the com buffer */
	pui8ByteToWrite [0] = prtadxl363_registers.uinterupt.ui8interupt;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_INTMAP1, PROTOCOL_REGISTER_INTERUPTMAP1_LENGTH );

	/* same for the 2nd Irq*/
	//pui8ByteToWrite [0] = prtadxl363_registers.uinterupt.ui8interupt;
	/* Write it */
	//prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_INTMAP2, PROTOCOL_REGISTER_INTERUPTMAP1_LENGTH );


	/* fill the power register */
	prtadxl363_registers.upower.ui8power 			= PROTOCOL_ADXL363_REGISTER_RESET_POWER;
	prtadxl363_registers.upower.spower.bAdcEnable	= PROTOCOL_ADXL363_DISABLE;
	prtadxl363_registers.upower.spower.bWakeUp 		= PROTOCOL_ADXL363_DISABLE;
	prtadxl363_registers.upower.spower.b2lownoise 	= PROTOCOL_ADXL363_LOWNOISE_NORMAL;
	prtadxl363_registers.upower.spower.bMeasure 	= PROTOCOL_ADXL363_MEASURE_MEASUREMENTMODE;
	prtadxl363_registers.upower.spower.bautoSleep 	= PROTOCOL_ADXL363_DISABLE;
	/* copy it in the com buffer */
	pui8ByteToWrite [0] = prtadxl363_registers.upower.ui8power;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_POWER_CTL, PROTOCOL_REGISTER_POWER_LENGTH );

}

/**************************************************************************//**
 * @brief 		this function configures the adxl363 to enable the activity or
 * inactivity detection
 * @details 	in this mode the number of sample measured to detect the activity
 * or the inactivity is 1. thus the probability of false detection increases
 * @param[in]  	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtadxl363_EnableLowPowerMotionDetection ( void )
{
uint8_t  * pui8ByteToWrite = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_COMMAND_DATA_INDEX]); /* the data are copied in the 3rd byte*/

prtadxl363_registers_struct	prtadxl363_registers;

	/* fill the power register */
	prtadxl363_registers.upower.ui8power 			= PROTOCOL_ADXL363_REGISTER_RESET_POWER;
	prtadxl363_registers.upower.spower.bAdcEnable	= PROTOCOL_ADXL363_DISABLE;
	prtadxl363_registers.upower.spower.bWakeUp 		= PROTOCOL_ADXL363_ENABLE;
	prtadxl363_registers.upower.spower.b2lownoise 	= PROTOCOL_ADXL363_LOWNOISE_NORMAL;
	prtadxl363_registers.upower.spower.bMeasure 	= PROTOCOL_ADXL363_MEASURE_MEASUREMENTMODE;
	prtadxl363_registers.upower.spower.bautoSleep 	= PROTOCOL_ADXL363_ENABLE;
	/* copy it in the com buffer */
	pui8ByteToWrite [0] = prtadxl363_registers.upower.ui8power;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_POWER_CTL, PROTOCOL_REGISTER_POWER_LENGTH );

}

/**************************************************************************//**
 * @brief 		this function configures the adxl363 to enable the activity or
 * inactivity detection
 * @details 	in this mode the number of sample measured is defined by the
 * registers PROTOCOL_ADXL363_REGISTER_TIME_ACT & PROTOCOL_ADXL363_REGISTER_TIME_INACT
 * @param[in]  	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtadxl363_DisableLowPowerMotionDetection ( void )
{
uint8_t  * pui8ByteToWrite = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_COMMAND_DATA_INDEX]); /* the data are copied in the 3rd byte*/

prtadxl363_registers_struct	prtadxl363_registers;

	/* fill the power register */
	prtadxl363_registers.upower.ui8power 			= PROTOCOL_ADXL363_REGISTER_RESET_POWER;
	prtadxl363_registers.upower.spower.bAdcEnable	= PROTOCOL_ADXL363_DISABLE;
	prtadxl363_registers.upower.spower.bWakeUp 		= PROTOCOL_ADXL363_DISABLE;
	prtadxl363_registers.upower.spower.b2lownoise 	= PROTOCOL_ADXL363_LOWNOISE_NORMAL;
	prtadxl363_registers.upower.spower.bMeasure 	= PROTOCOL_ADXL363_MEASURE_MEASUREMENTMODE;
	prtadxl363_registers.upower.spower.bautoSleep 	= PROTOCOL_ADXL363_DISABLE;
	/* copy it in the com buffer */
	pui8ByteToWrite [0] = prtadxl363_registers.upower.ui8power;
	/* Write it */
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_POWER_CTL, PROTOCOL_REGISTER_POWER_LENGTH );

}


/**************************************************************************//**
 * @brief 		this function sends a reset command to the ADXL363 in order
 * to force a SW reset
 * @param[in]  	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtADXL363_Reset ( void )
{
uint8_t  * pui8ByteToWrite = &(protocol_adxl363Buffer[PROTOCOL_ADXL363_COMMAND_DATA_INDEX]); /* the data are copied in the 3rd byte*/

	pui8ByteToWrite [0] =  PROTOCOL_ADXL363_RESETCODE;
	prtADXL363_WriteADXL363Register (PROTOCOL_ADXL363_REGISTER_SOFT_RESET, 1 );
}


/**************************************************************************//**
 * @brief 		this function enables the irq for the motion detection
 * @param[in]  	bOnorOff : true to enable the irq, false to disable
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtadxl363_EnableMotionIrq ( const bool bOnorOff  )
{
	interface_InitAccelerometerIrq (bOnorOff ,  INTERFACE_FALLINGANDRISING_EDGE);
}

/**************************************************************************//**
 * @brief 		this function enables the irq for the motion detection
 * @param[in]  	bOnorOff : true to enable the irq, false to disable
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void prtadxl363_EnableDataReadyIrq ( const bool bOnorOff  )
{
	interface_InitAccelerometerIrq (bOnorOff , INTERFACE_RISING_EDGE);
}
