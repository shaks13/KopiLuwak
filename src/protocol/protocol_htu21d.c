/*******************************************************************************
 * @file protocol_lis3Mdl.c
 * @brief this function set is codec for the Si7021 or HTU21D device
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_htu21d.h"



/*===========================================================================================================
						Public variables definitions
===========================================================================================================*/
uint8_t prthtu21d_ui8Buffer[PRTHTU21D_BUFFER_NBBYTE];



/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
static uint8_t prthtu21d_ReadRegister 					( const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield);
static uint8_t prthtu21d_ReadCurrentRegister			( const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield);
static uint8_t prthtu21d_ReadRegisterwith2AddressBytes 	( const uint16_t ui16MemoryAdr, const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield);
static uint8_t prthtu21d_WriteRegister 					( const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToWrite, uint8_t const * pui8DataToWrite);
/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief 		This function performs a random read
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[in] 	pui8Registerfield: data read in the register
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return		CROSSRFID_ERROR_I2C_READ - an error has occurred.
*******************************************************************************/
static uint8_t prthtu21d_ReadRegister(const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield)
{
int8_t ui8Status;
I2C_TransferReturn_TypeDef i8Status;

	i8Status = I2C_Read(PRTHTU21D_I2CADDRESS, ui8MemoryAdr,ui8NbByteToRead,prthtu21d_ui8Buffer);
	if (i2cTransferDone == i8Status)
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else if (i2cTransferNack == i8Status)
	{
		ui8Status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_READ;
	}

	*pui8Registerfield = prthtu21d_ui8Buffer;
	return ui8Status;
}

/***************************************************************************//**
 * @brief 		This function performs a random read
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[in] 	pui8Registerfield: data read in the register
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return		CROSSRFID_ERROR_I2C_READ - an error has occurred.
*******************************************************************************/
static uint8_t prthtu21d_ReadCurrentRegister( const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield)
{
int8_t ui8Status;
I2C_TransferReturn_TypeDef i8Status;

	i8Status = I2C_CurrentRead(PRTHTU21D_I2CADDRESS, ui8NbByteToRead,prthtu21d_ui8Buffer);
	if (i2cTransferDone == i8Status)
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else if (i2cTransferNack == i8Status)
	{
		ui8Status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_READ;
	}

	*pui8Registerfield = prthtu21d_ui8Buffer;
	return ui8Status;
}


/***************************************************************************//**
 * @brief 		This function performs a random read
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[in] 	pui8Registerfield: data read in the register
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return		CROSSRFID_ERROR_I2C_READ - an error has occurred.
*******************************************************************************/
static uint8_t prthtu21d_ReadRegisterwith2AddressBytes (const uint16_t ui16MemoryAdr, const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield)
{
uint8_t ui8Status;

	if (i2cTransferDone == I2C_Readwith2AddressBytes (PRTHTU21D_I2CADDRESS, ui16MemoryAdr,ui8NbByteToRead,prthtu21d_ui8Buffer))
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_READ;
	}

	*pui8Registerfield = prthtu21d_ui8Buffer;
	return ui8Status;
}


/***************************************************************************//**
 * @brief 		This function write in the device register
 * @param[in] 	ui8MemoryAdr: the register address
 * @param[in] 	ui8NbByteToWrite: the number of byte to write
 * @param[in] 	pui8DataToWrite: the buffer to write
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_WRITE - an error has occurred.
*******************************************************************************/
static uint8_t prthtu21d_WriteRegister ( const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToWrite, uint8_t const * pui8DataToWrite)
{
uint8_t ui8Status;

	prthtu21d_ui8Buffer [0] = ui8MemoryAdr; 							/* add the register address as the first byte*/
	memcpy(&prthtu21d_ui8Buffer [1],pui8DataToWrite, ui8NbByteToWrite);	/* append the data to write*/

	if (i2cTransferDone == I2C_Write(PRTHTU21D_I2CADDRESS, 1,ui8NbByteToWrite,prthtu21d_ui8Buffer))
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_WRITE;
	}

	return ui8Status;
}


/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief   	This function turn on or off the power supply of the external memory
 * @param[in]	OnOrOff : when true the power supplied will turn on otherwise off
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void prthtu21d_EnablePowerSuppply ( bool bOnOrOff )
{
	I2C_EnablePowerSuppply ( bOnOrOff );

}

/***************************************************************************//**
 * @brief 		This function initializes the link to the SI7021
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prthtu21d_init (void)
{
	I2C_init ( );
	sleep_DelayTask (PRTHTU21D_POWERUP_TIMEMS,configSLEEP_MODE );
}


/**************************************************************************//**
 * @brief 		this function checks the Id of the SI7021
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
uint8_t prthtu21d_CheckDeviceId ( void )
{
uint8_t *pui8Response;
uint8_t status = CROSSRFID_SUCCESSCODE;

	prthtu21d_ReadRegisterwith2AddressBytes (PRTLIS3MDL_READFWREVISION_ADDRESS, 1 , &pui8Response);
	if ((pui8Response[0] != PRTLIS3MDL_FIRMWAREREVISION_10) && (pui8Response[0] != PRTLIS3MDL_FIRMWAREREVISION_20))
	{
		status = CROSSRFID_INITSENSOR_ERROR;
	}
	else {/*do nothing*/}
	return status;
}

/**************************************************************************//**
 * @brief 		this function checks the Id of the SI7021
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE : the I2C device doesn't acknowledge
 * @return 		CROSSRFID_SUCCESSCODE : the I2C device acknowledges
 *****************************************************************************/
uint8_t prthtu21d_IsAvaialble ( void )
{

	return I2C_IsAvailable (PRTHTU21D_I2CADDRESS);
}

/**************************************************************************//**
 * @brief 		this function resets the SI7021
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
void prthtu21d_Reset ( void )
{
uint8_t *pui8Response=NULL;

	prthtu21d_WriteRegister (PRTLIS3MDL_RESET_ADDRESS, 0 , pui8Response);

}


/**************************************************************************//**
 * @brief 		this function launch a temperature measurement and read it
 * @param[in]  	none
 * @param[out] 	i16Temperature : the measured temperature in degree celcius
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
uint8_t prthtu21d_MeasureTemperature ( int16_t *i16Temperature )
{
uint8_t *pui8Response;
uint8_t ui8status ;
int16_t i16Tmp;

	//ui8status = prthtu21d_ReadRegister (PRTLIS3MDL_MEASURETEMPHOLD_ADDRESS, 2 , &pui8Response); /*PRTLIS3MDL_MEASURERHHOLD_ADDRESS*/
	ui8status = prthtu21d_WriteRegister (PRTLIS3MDL_MEASURETEMPNOTHOLD_ADDRESS, 0 , NULL);
	vTaskDelay(50);
	ui8status = prthtu21d_ReadCurrentRegister(2,&pui8Response);

	if ( CROSSRFID_SUCCESSCODE == ui8status)
	{
		//i16Tmp = (int16_t) (* (int16_t*)pui8Response);
		i16Tmp = (int16_t) ( (pui8Response [0] << 8) | (pui8Response [1]) );
		*i16Temperature = (PRTHTU21D_TEMPERATURECONVERSION_CONSTANT1 * (i16Tmp) /PRTHTU21D_TEMPERATURECONVERSION_CONSTANT2) - PRTHTU21D_TEMPERATURECONVERSION_CONSTANT3;
	}

	return ui8status;
}

/**************************************************************************//**
 * @brief 		this function launch a humidity measurement and read it
 * @param[in]  	none
 * @param[out] 	i16Humidity : the measured humidity
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
void prthtu21d_MeasureHumidity ( int16_t *i16Humidity )
{
uint8_t *pui8Response;

	prthtu21d_ReadRegister (PRTLIS3MDL_MEASURERHNOHOLD_ADDRESS, 2 , &pui8Response);

	*i16Humidity = (int16_t) (*pui8Response);
	*i16Humidity = (PRTHTU21D_HUMIDITYCONVERSION_CONSTANT1 * (*i16Humidity) /PRTHTU21D_HUMIDITYCONVERSION_CONSTANT2) - PRTHTU21D_HUMIDITYCONVERSION_CONSTANT3;

}


