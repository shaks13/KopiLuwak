/*******************************************************************************
 * @file srvADXL363.c
 * @brief this file defines the command set for the ADXL363 (accelerometer)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvadxl363.h"
/*****************************************************************************************
 ********************************* Variables definitions**********************************
 *****************************************************************************************/
static srvdxl363_State_struct 	srvadxl363_state ;
static srvdxl363_MeasBuffer_struct 	srvadxl363_AccelBuffer ;
static srvdxl363_FFTBuffer_struct	srvadxl363_FFTBuffer ;
/*****************************************************************************************
 ********************************* Functions Definitions *********************************
 *****************************************************************************************/


/*****************************************************************************************
 ********************************* Functions *********************************************
 *****************************************************************************************/

/***************************************************************************//**
 * @brief 		this function initializes the communication with the ADXL363
 * @param[in] 	srvadxl36x_Mode_enum : defines the functionnality of the acceleromter
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
*******************************************************************************/
uint8_t srvadxl363_Init ( const srvadxl36x_Mode_enum emode )
{
uint8_t ui8status = CROSSRFID_INITSENSOR_ERROR;
uint8_t *pui8XYZaxis;

	prtadxl363_Init ();
	ui8status = prtADXL363_CheckDeviceId ();
	srvadxl363_AccelBuffer.ui16XaxisNbelement = 0;
	srvadxl363_AccelBuffer.ui16YaxisNbelement = 0;
	srvadxl363_AccelBuffer.ui16ZaxisNbelement = 0;
	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		switch ( emode )
		{
			case SRVADXL36X_MODE_MOTIONDETECTION:
				prtadxl363_InitiateMotionDetection();
				srvadxl363_state.ui8state = SRVADXL36X_STATE_ON;
			break;
			case SRVADXL36X_MODE_MEASUREMENT:
				ui8status = prtadxl363_InitLogMeasurement(true, PROTOCOL_ADXL363_ODR_12HZ);
				if (CROSSRFID_SUCCESSCODE == ui8status)
				{
					srvadxl363_state.ui8state = SRVADXL36X_STATE_ON;
					srvadxl363_EnableDataReadyIrq (true);
					srvadxl363_ReadAcceloMeasure (&pui8XYZaxis);		/* clean the IRQ from the point of view of the Adxl363*/
				}


			break;
			case SRVADXL36X_MODE_LOG:

			break;
			default :
				/* should not happen*/
			break;
		}

	}
	else
	{
		prtadxl363_Deinit ();
		srvadxl363_state.ui8state = SRVADXL36X_STATE_OFF;
	}
	return ui8status;
}


/***************************************************************************//**
 * @brief 		this function initializes the continuous measurement of the
 * accelerometer
 * @param[in] 	bOnOrOff
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the accelerometer is correctly initialized
 * @return 		CROSSRFID_INITSENSOR_ERROR : the accelerometer is not correctly initialized
 * @return 		other : the accelerometer is not correctly initialized
*******************************************************************************/
uint8_t srvadxl363_InitLogMeasurement (const bool bOnOrOff)
{
	return (prtadxl363_InitLogMeasurement (bOnOrOff,PROTOCOL_ADXL363_ODR_12HZ));
}

/***************************************************************************//**
 * @brief 		this function reads the 3 axis register
 * @param[in] 	none
 * @param[out] 	pui8XYZaxis pointer on the buffer response
 * @return 		none
*******************************************************************************/
void srvadxl363_ReadAcceloMeasure ( uint8_t **pui8XYZaxis )
{

	prtadxl363_ReadXYZaxisExtended ( pui8XYZaxis );
}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the first available
 * accelerometer buffer
 *
 * @param[in]	none
 * @param[out] 	i8pXdata : pointer on the buffer data of the X axis
 * @param[out] 	i8pYdata : pointer on the buffer data of the Y axis
 * @param[out] 	i8pZdata : pointer on the buffer data of the Z axis
 * @return 		none
 ******************************************************************************/
void srvadxl363_GetAxisMeasBuffer ( int16_t **i16pXdata, int16_t **i16pYdata, int16_t **i16pZdata )
{
	(* i16pXdata) =  srvadxl363_AccelBuffer.aui8XaxisBuffer;
	(* i16pYdata) =  srvadxl363_AccelBuffer.aui8YaxisBuffer;
	(* i16pZdata) =  srvadxl363_AccelBuffer.aui8ZaxisBuffer;

}

/***************************************************************************//**
 * @brief 		This function returns the pointer on the
 * accelerometer buffer
 *
 * @param[in]	none
 * @param[out] 	i8pdata : pointer on the buffer data of the requested axis
 * @return 		none
 ******************************************************************************/
void srvadxl363_GetMeasBuffer ( const Kernel_MeasurementId_enum eMeasurementId , int8_t **i8pdata )
{
	switch (eMeasurementId)
	{
		case KERNEL_MEASID_RAW:
		case KERNEL_MEASID_XRAWAXIS:
			(* i8pdata) = (int8_t *) srvadxl363_AccelBuffer.aui8XaxisBuffer;
		break;
		case KERNEL_MEASID_YRAWAXIS:
			(* i8pdata) = (int8_t *) srvadxl363_AccelBuffer.aui8YaxisBuffer;
		break;
		case KERNEL_MEASID_ZRAWAXIS:
			(* i8pdata) = (int8_t *) srvadxl363_AccelBuffer.aui8ZaxisBuffer;
		break;
		//case KERNEL_MEASID_XFFTAXIS :	/* the buffer is shared between the 3 axis */
		//case KERNEL_MEASID_YFFTAXIS :
		//case KERNEL_MEASID_ZFFTAXIS :
		case KERNEL_MEASID_FFT :
			(* i8pdata) = (int8_t *) srvadxl363_FFTBuffer.aui16Buffer;
		break;
		default:
			(* i8pdata) = NULL;
		break;
	}



}

/***************************************************************************//**
 * @brief 		This function returns the number of measurement
 * accelerometer buffer
 *
 * @param[in]	none
 * @param[out] 	ui8pdata : pointer on the buffer data of the temperature
 * @return 		none
 ******************************************************************************/
void srvadxl363_GetNbMeas ( const Kernel_MeasurementId_enum eMeasureId, uint16_t *ui16NbMeasurement )
{

	switch (eMeasureId)
	{
		case KERNEL_MEASID_RAW:
		case KERNEL_MEASID_XRAWAXIS:
			(* ui16NbMeasurement) =srvadxl363_AccelBuffer.ui16XaxisNbelement  ;
		break;
		case KERNEL_MEASID_YRAWAXIS:
			(* ui16NbMeasurement) =srvadxl363_AccelBuffer.ui16YaxisNbelement  ;
		break;
		case KERNEL_MEASID_ZRAWAXIS:
			(* ui16NbMeasurement) =srvadxl363_AccelBuffer.ui16ZaxisNbelement  ;
		break;

		//case KERNEL_MEASID_XFFTAXIS :	/* the buffer is shared between the 3 axis */
		//case KERNEL_MEASID_YFFTAXIS :
		//case KERNEL_MEASID_ZFFTAXIS :
		case KERNEL_MEASID_FFT :
			(* ui16NbMeasurement) =srvadxl363_FFTBuffer.ui16FFTNbelement  ;
		break;

		default :
			(* ui16NbMeasurement) = 0;
		break;
	}
}

/***************************************************************************//**
 * @brief 		This function returns the number of measurement
 * accelerometer buffer
 *
 * @param[in]	ui16NbMeasurement : number of measurement
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvadxl363_SetNbMeas ( const Kernel_MeasurementId_enum eMeasureId, const uint16_t ui16NbMeasurement )
{
	switch (eMeasureId)
	{
		case KERNEL_MEASID_XRAWAXIS:
			srvadxl363_AccelBuffer.ui16XaxisNbelement = ui16NbMeasurement ;
		break;
		case KERNEL_MEASID_YRAWAXIS:
			srvadxl363_AccelBuffer.ui16YaxisNbelement = ui16NbMeasurement ;
		break;
		case KERNEL_MEASID_ZRAWAXIS:
			srvadxl363_AccelBuffer.ui16ZaxisNbelement = ui16NbMeasurement ;
		break;
		case KERNEL_MEASID_XYZRAWAXIS: /* update the three axis at the same time */
			srvadxl363_AccelBuffer.ui16XaxisNbelement = ui16NbMeasurement ;
			srvadxl363_AccelBuffer.ui16YaxisNbelement = ui16NbMeasurement ;
			srvadxl363_AccelBuffer.ui16ZaxisNbelement = ui16NbMeasurement ;
		break;

		//case KERNEL_MEASID_XFFTAXIS :	/* the buffer is shared between the 3 axis */
		//case KERNEL_MEASID_YFFTAXIS :
		//case KERNEL_MEASID_ZFFTAXIS :
		case KERNEL_MEASID_FFT :
			srvadxl363_FFTBuffer.ui16FFTNbelement = ui16NbMeasurement;
		break;
		default :
		break;
	}
}

/***************************************************************************//**
* @brief 		This function is the callback for the RTCC timer for the m
 * measurement
 * @param[in]	RTCDRV_TimerID_t : Id of the RTCC timer that triggers the IRQ
 * @param[in]	user : pointer on the user data
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
void srvadxl363_AcceleroCB ( void )
{


}

/**************************************************************************//**
 * @brief 		this function enables the irq for the accelerometer
 * @param[in]  	bOnorOff : true to enable the irq, false to disable
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void srvadxl363_EnableMotionIrq ( const bool bOnorOff )
{
	prtadxl363_EnableMotionIrq (bOnorOff);
}


/**************************************************************************//**
 * @brief 		this function enables the irq for the accelerometer
 * @param[in]  	bOnorOff : true to enable the irq, false to disable
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void srvadxl363_EnableDataReadyIrq ( const bool bOnorOff )
{
	prtadxl363_EnableDataReadyIrq (bOnorOff);
}


/***************************************************************************//**
 * @brief 		this function returns true when the ADXL363 is enabled
 * @param[in] 	none
 * @param[out] 	none
 * @return 		true : the accelerometer is running
 * @return 		false : the accelerometer is not running
*******************************************************************************/
bool srvadxl363_IsEnable ( void )
{
bool bstatus = false;

	if (SRVADXL36X_STATE_ON == srvadxl363_state.ui8state)
	{
		bstatus= true;
	}
	else
	{
		bstatus = false;
	}

	return bstatus;
}
