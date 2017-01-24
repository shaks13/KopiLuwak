/*******************************************************************************
 * @file srvlis3mdl.c
 * @brief this file defines the command set for the ADXL363 (accelerometer)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvlis3mdl.h"
/*****************************************************************************************
 ********************************* Variables definitions**********************************
 *****************************************************************************************/
static srvlis3mdl_MagnetoBuffer_struct srvtemp_MagnetoBuffer;
static srvlis3mdl_FFTBuffer_struct srvtemp_MagnetoFFTBuffer;

/*****************************************************************************************
 ********************************* Functions Definitions *********************************
 *****************************************************************************************/


/*****************************************************************************************
 ********************************* Functions *********************************************
 *****************************************************************************************/

/***************************************************************************//**
 * @brief 		this function initializes the communication with the LIS3MDL
 * device
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
*******************************************************************************/
uint8_t srvlis3mdl_Init ( void )
{
uint8_t status = CROSSRFID_INITSENSOR_ERROR;

	prtlis3mdh_init ();
	status = prtlis3mdh_CheckDeviceId ();
	srvtemp_MagnetoBuffer.ui16Nbelement = 0;
	return status;
}


/***************************************************************************//**
 * @brief 		this function configures the LIS3MDL to act as a motion detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
uint8_t srvlis3mdl_EnableMagneticDetection ( bool bEnableOrDisable )
{
uint8_t status = CROSSRFID_INITSENSOR_ERROR;
int16_t ai16Magneticfield[3];
uint8_t *pui8Statusreg;

	if (true == bEnableOrDisable)
	{
		prtlis3mdh_init ();
		status = prtlis3mdh_CheckDeviceId ();
		if (CROSSRFID_SUCCESSCODE == status)
		{
			prtlis3mdh_initDataReadypad (true);		/* init the data ready pad as an input*/
			prtlis3mdh_initIntpad (true);			/* init the irq pad as an input*/
			prtlis3mdh_InitiateMagneticFieldDetection (true);			/* configure the magnetic field detector */
			prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_STATUTREG,1,&pui8Statusreg);
			prtlis3mdh_Read3axisField (&ai16Magneticfield[0],&ai16Magneticfield[1],&ai16Magneticfield[2]);

		}
		else
		{
			/* do nothing*/
		}
	}
	else
	{
		prtlis3mdh_initIntpad (false);
		prtlis3mdh_initDataReadypad (false);
		prtlis3mdh_InitiateMagneticFieldDetection (false);
	}

	if (CROSSRFID_SUCCESSCODE != status)
	{
		prtlis3mdh_initIntpad (false);			/* deinit the irq pad as an input*/
		prtlis3mdh_initDataReadypad (false);	/* deinit the data ready pad as an input*/
	}


	return status;
}

/***************************************************************************//**
 * @brief 		this function gets the absolute value of the magnetic field
 * on the 3 axis
 * @param[in] 	none
 * @param[out] 	ui16MagneticField : the sum of the absolute value of the magnetic field
 * on the 3 axis
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return		CROSSRFID_ERROR_I2C_READ - an error has occurred.
*******************************************************************************/
uint8_t srvlis3mdl_GetAbsMagneticFieldOn3Axis ( uint16_t * ui16MagneticField )
{
uint8_t status = CROSSRFID_INITSENSOR_ERROR;
uint8_t ui8NthValue =0;
uint32_t ui32MagneticField = 0;
int16_t ai16Magneticfield[3];
uint8_t *pui8Statusreg;


	(* ui16MagneticField)= 0 ;
	status = prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_STATUTREG,1,&pui8Statusreg);
	prtlis3mdh_Read3axisField (&ai16Magneticfield[0],&ai16Magneticfield[1],&ai16Magneticfield[2]);
	if (CROSSRFID_SUCCESSCODE == status)
	{

		for (ui8NthValue =0 ; ui8NthValue <3; ui8NthValue ++)
		{
			/* get the abs value */
			if (ai16Magneticfield[ui8NthValue] <0)
			{
				ai16Magneticfield[ui8NthValue] *=-1;
			} else { /* do nothing*/}
			ui32MagneticField += (uint16_t) (ai16Magneticfield[ui8NthValue]);
		}

		if ( (ui32MagneticField & 0xFFFF0000) == 0) /* when the absolute value can be written on 16 bits*/
		{
			(* ui16MagneticField) = ui32MagneticField;
		}
		else /* otherwise return the max value*/
		{
			(* ui16MagneticField) = 0xFFFF;
		}
	}
	else { /* do nothing*/}


	return status ;
}

/***************************************************************************//**
 * @brief 		this function configures the LIS3MDL to act as a motion detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvlis3mdl_ProcessMagnetoIrq ( void )
{
int16_t ai16Magneticfield[3];
uint8_t *pui8Statusreg;

	prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_STATUTREG,1,&pui8Statusreg);
	prtlis3mdh_Read3axisField (&ai16Magneticfield[0],&ai16Magneticfield[1],&ai16Magneticfield[2]);

	printf ("Magneto : %d %d %d \n" ,ai16Magneticfield[0],ai16Magneticfield[1], ai16Magneticfield[2]);


}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the first available
 * temperature buffer
 *
 * @param[in]	none
 * @param[out] 	ui8pdata : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_GetMeasBuffer ( int16_t **i16pdata )
{
	(* i16pdata) = srvtemp_MagnetoBuffer.aui16MeasBuffer;
}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the first available
 * temperature buffer
 *
 * @param[in]	none
 * @param[out] 	ui8pdata : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_GetFFTBuffer ( uint16_t **ui16pdata )
{
	(* ui16pdata) = srvtemp_MagnetoFFTBuffer.aui16MeasBuffer;
}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the first available
 * temperature buffer
 *
 * @param[in]	none
 * @param[out] 	ui16NbMeasurement : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_GetNbMeas ( uint16_t *ui16NbMeasurement )
{
	(* ui16NbMeasurement) = srvtemp_MagnetoBuffer.ui16Nbelement;
}

/***************************************************************************//**
 * @brief 		This function returns the number of element of the FFT buffer
 *
 * @param[in]	none
 * @param[out] 	ui16NbMeasurement : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_GetNbFFTpoint ( uint16_t *ui16NbMeasurement )
{
	(* ui16NbMeasurement) = srvtemp_MagnetoFFTBuffer.ui16Nbelement;
}

/***************************************************************************//**
 * @brief 		This function set the number of element of the raw maeasurement
 *
 * @param[in]	ui16NbMeasurement : pointer on the buffer data of the temperature
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_SetNbMeas ( const uint16_t ui16NbMeasurement )
{
	 srvtemp_MagnetoBuffer.ui16Nbelement = ui16NbMeasurement;
}

/***************************************************************************//**
 * @brief 		This function set the number of element of the FFT buffer
 *
 * @param[in]	ui16NbMeasurement : pointer on the buffer data of the temperature
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_SetNbFFTMeas ( const uint16_t ui16NbMeasurement )
{
	srvtemp_MagnetoFFTBuffer.ui16Nbelement = ui16NbMeasurement;
}

/***************************************************************************//**
 * @brief 		This function is the callback for the RTCC timer for the accelerometer
 * measurement
 * @param[in]	RTCDRV_TimerID_t : Id of the RTCC timer that triggers the IRQ
 * @param[in]	user : pointer on the user data
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
void srvlis3mdl_MagnetoCB ( void )
{


}
