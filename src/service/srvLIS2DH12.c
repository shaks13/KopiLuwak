/*******************************************************************************
 * @file srvEM4325.c
 * @brief this file defines the commans set for the EM4325
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvLIS2DH12.h" 	// LIS2DH12 definitions

/*****************************************************************************************
 ********************************* Variables definitions**********************************
 *****************************************************************************************/

/*****************************************************************************************
 ********************************* Functions Definitions *********************************
 *****************************************************************************************/


/*****************************************************************************************
 ********************************* Functions *********************************************
 *****************************************************************************************/

/***************************************************************************//**
 * @brief 		this function initializes the peripheral to communicate with
 * the device LIS2DH
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
uint8_t srvLIS2DH12_Init ( void )
{
uint8_t status;

	status = prtLIS2DH_Init();

	return status;
}
#if 0
/***************************************************************************//**
 * @brief Reads the value of a register.
 * @param[in] none
 * @param[out] none
 * @return none
*******************************************************************************/
uint8_t srvLIS2DH12_GetGz ( void )
{
float32_t fAcceOnZaxis = 0;
uint8_t status = CROSSRFID_SUCCESSCODE;
//arm_rfft_fast_instance_f32  sFFTinstance;
//uint8_t ui8DymmyBuffer[6] ;

	prtLIS2DH_GetGz(&fAcceOnZaxis); /* read the measurement data*/
	//LIS2DH12_GetRegisterValue (LIS2DH12_OUT_X_L,6,ui8DymmyBuffer);	/* not used*/
	af32DataPool [ui16NthElement++] = fAcceOnZaxis;

	if (ui16NthElement >= SRVLIS2DH12_DATAPOOL_NBELEMENT)
	{
		dsplib_computefft (af32DataPool,af32fft,SRVLIS2DH12_DATAPOOL_NBELEMENT);
		/* manage the buffer overflow*/
		ui16NthElement = 0;
		interface_DisablePinInterrupt (INTERFACE_SPI2INT1_PORT, INTERFACE_SPI2INT1_PIN ); /* deactivate the IRQ*/

	}else{/*do nothing*/}

	return status;
}
#else
/***************************************************************************//**
 * @brief Reads the value of a register.
 * @param[in] none
 * @param[out] none
 * @return CROSSRFID_SUCCESSCODE : the buffer is full the measure is over
 * @return CROSSRFID_INITSENSOR_ONGOING : the acquisition is over but the buffer
 * is not full
*******************************************************************************/
uint8_t srvLIS2DH12_GetGz ( void )
{
float32_t fAcceOnZaxis = 0;
uint8_t status = CROSSRFID_INITSENSOR_ONGOING;

	prtLIS2DH_GetGz(&fAcceOnZaxis); /* read the measurement data*/
	//LIS2DH12_GetRegisterValue (LIS2DH12_OUT_X_L,6,ui8DymmyBuffer);	/* not used*/

	return status;
}

#endif
