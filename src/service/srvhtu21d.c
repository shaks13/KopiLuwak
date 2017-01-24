/*******************************************************************************
 * @file srvM2M.c
 * @brief this file is the service layer for the HTU21D
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvhtu21d.h"

/***************************************************************************//**
 * @brief 		This function initializes the link to the SI7021
 * @param[in] 	bOnOrOff : enable or not the HTU21d interface
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvhtu21d_init (bool bOnOrOff)
{
	if (true == bOnOrOff)
	{
		prthtu21d_init ();
	} else {/*do nothing*/}
}

/**************************************************************************//**
 * @brief 		this function launch a temperature measurement and read it
 * @param[in]  	none
 * @param[out] 	i16Temperature : the measured temperature in degree celcius
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
uint8_t srvhtu21d_MeasureTemperature ( int16_t *i16Temperature )
{
uint8_t ui8status;

	ui8status  = prthtu21d_MeasureTemperature ( i16Temperature );
	return ui8status;
}


