/*******************************************************************************
 * @file protocol_lis3Mdl.c
 * @brief this function set is codec for the LIS3mDL
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_lis3mdl.h"



/*===========================================================================================================
						Public variables definitions
===========================================================================================================*/
uint8_t prtlis3mdl_ui8Buffer[PRTLIS3MDL_LENGTH_RX_BUFFER];



/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
static int32_t i32Sensitivity = LIS3MDL_DSENSITIVITY_4GAUSS;


/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief   	This function turn on or off the power supply of the external memory
 * @param[in]	OnOrOff : when true the power supplied will turn on otherwise off
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void prtlis3mdh_EnablePowerSuppply ( bool bOnOrOff )
{
	I2C_EnablePowerSuppply ( bOnOrOff );

}

/***************************************************************************//**
 * @brief 		This function initializes the link to the 24LC64 (64 K I2C EEPROM)
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prtlis3mdh_init (void)
{
	I2C_init ( );
}


/**************************************************************************//**
 * @brief 		this function checks the Id of the LIM3MDL
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_INITSENSOR_ERROR : the ID doens't match with the
 * sensorId of the data sheet
 *****************************************************************************/
uint8_t prtlis3mdh_CheckDeviceId ( void )
{
uint8_t *pui8Response;
uint8_t status = CROSSRFID_SUCCESSCODE;

	prtlis3mdh_ReadRegister (_LIS3MDL_REG_WHO_AM_I, 1 , &pui8Response);
	if (pui8Response[0] != LIS3MDL_DEVICE_ID)
	{
		status = CROSSRFID_INITSENSOR_ERROR;
	}
	else {/*do nothing*/}
	return status;
}

/***************************************************************************//**
 * @brief 		This function performs a random read
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[in] 	pui8Registerfield: data read in the register
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return		CROSSRFID_ERROR_I2C_READ - an error has occurred.
*******************************************************************************/
uint8_t prtlis3mdh_ReadRegister (const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield)
{
uint8_t ui8Status;
#if 0
	I2C_EnablePowerSuppply (true);
	vTaskDelay(1);	/*	dirty fix : the LIS3MDH requires a boot time > 10�S*/
#endif
	if (i2cTransferDone == I2C_Read (LIS3MDL_ADDRESS1, ui8MemoryAdr,ui8NbByteToRead,prtlis3mdl_ui8Buffer))
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_READ;
	}

	*pui8Registerfield = prtlis3mdl_ui8Buffer;
#if 0
	I2C_EnablePowerSuppply (false);
#endif
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
uint8_t prtlis3mdh_WriteRegister ( const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToWrite, uint8_t const * pui8DataToWrite)
{
uint8_t ui8Status;

	prtlis3mdl_ui8Buffer [0] = ui8MemoryAdr; 								/* add the register address as the first byte*/
	memcpy(&prtlis3mdl_ui8Buffer [1],pui8DataToWrite, ui8NbByteToWrite);	/* append the data to write*/

	if (i2cTransferDone == I2C_Write(LIS3MDL_ADDRESS1, 1,ui8NbByteToWrite,prtlis3mdl_ui8Buffer))
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_WRITE;
	}

	return ui8Status;
}



/***************************************************************************//**
 * @brief 		This function configure the LIS3MDL to work as magnetic field
 * detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void  prtlis3mdh_InitiateMagneticFieldDetection ( bool bOnOrOff )
{
prtlis3mdl_registers_struct prtlis3mdl_registers;
uint8_t 	aui8Threshold[2]={0x80,0x00};
#if 0
	I2C_EnablePowerSuppply (true);
	vTaskDelay(1);	/*	dirty fix : the LIS3MDH requires a boot time > 10�S*/
#endif


	if (true ==  bOnOrOff)
	{
		prtlis3mdl_registers.uCtrlReg2.ui8controlReg2 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg2.scontrolReg2.bSoftReset = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uCtrlReg2.scontrolReg2.bReboot = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL2, 1, &(prtlis3mdl_registers.uCtrlReg2.ui8controlReg2));

		vTaskDelay(10);

		prtlis3mdl_registers.uCtrlReg1.ui8controlReg1 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg1.scontrolReg1.b2XYoperativeMode =PRTLIS3MDL_CTRLREG1_OPMODE_LOWPOWER;
		prtlis3mdl_registers.uCtrlReg1.scontrolReg1.b3ODR =PRTLIS3MDL_CTRLREG1_ODR_1DOT25HZ;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL1, 1, &(prtlis3mdl_registers.uCtrlReg1.ui8controlReg1));



		prtlis3mdl_registers.uCtrlReg2.ui8controlReg2 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg2.scontrolReg2.b2FullScaleConfoguration = PRTLIS3MDL_CTRLREG2_FULLSCALE_16GAUSS;
		//prtlis3mdl_registers.uCtrlReg2.scontrolReg2.bSoftReset = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL2, 1, &(prtlis3mdl_registers.uCtrlReg2.ui8controlReg2));
		i32Sensitivity = LIS3MDL_DSENSITIVITY_16GAUSS;


		prtlis3mdl_registers.uCtrlReg3.ui8controlReg3 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg3.scontrolReg3.b2Operatingmode = PRTLIS3MDL_CTRLREG3_OPMODE_CONTINUOUS;
		prtlis3mdl_registers.uCtrlReg3.scontrolReg3.bLP = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL3, 1, &(prtlis3mdl_registers.uCtrlReg3.ui8controlReg3));

		prtlis3mdl_registers.uCtrlReg4.ui8controlReg4 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg4.scontrolReg4.b2ZaxiOperatingMode = PRTLIS3MDL_CTRLREG1_OPMODE_LOWPOWER;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL4, 1, &(prtlis3mdl_registers.uCtrlReg4.ui8controlReg4));

	#if 1
		prtlis3mdl_registers.uCtrlReg5.ui8controlReg5 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		//prtlis3mdl_registers.uCtrlReg5.scontrolReg5.bBloackDataUpdate = PRTLIS3MDL_ENABLE	;
		prtlis3mdl_registers.uCtrlReg5.scontrolReg5.bFastRead = PRTLIS3MDL_ENABLE	;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL5, 1, &(prtlis3mdl_registers.uCtrlReg5.ui8controlReg5));

	#endif
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_INTTHRESHOLDL, 1, aui8Threshold);

		prtlis3mdl_registers.uIntReg.ui8IntConfig = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bXintEnable = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bYintEnable = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bZintEnable = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bIntEnable = PRTLIS3MDL_DISABLE;
		//prtlis3mdl_registers.uIntReg.sIntConfig.bInteruptActive = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_INTCONFIG, 2,&(prtlis3mdl_registers.uIntReg.ui8IntConfig) );
	}
	else
	{
		prtlis3mdl_registers.uCtrlReg3.ui8controlReg3 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg3.scontrolReg3.b2Operatingmode = PRTLIS3MDL_CTRLREG3_OPMODE_POWERDOWN;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL3, 1, &(prtlis3mdl_registers.uCtrlReg3.ui8controlReg3));
	}


}



/***************************************************************************//**
 * @brief 		This function initializes the magnetometer to make a
 * continuous measurement
 * detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void  prtlis3mdh_InitiateMagneticFieldLog ( bool bOnOrOff )
{
prtlis3mdl_registers_struct prtlis3mdl_registers;
uint8_t 	aui8Threshold[2]={0x80,0x00};
#if 0
	I2C_EnablePowerSuppply (true);
	vTaskDelay(1);	/*	dirty fix : the LIS3MDH requires a boot time > 10�S*/
#endif




	if (true ==  bOnOrOff)
	{
		prtlis3mdl_registers.uCtrlReg2.ui8controlReg2 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg2.scontrolReg2.bSoftReset = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uCtrlReg2.scontrolReg2.bReboot = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL2, 1, &(prtlis3mdl_registers.uCtrlReg2.ui8controlReg2));

		vTaskDelay(10);

		prtlis3mdl_registers.uCtrlReg1.ui8controlReg1 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg1.scontrolReg1.b2XYoperativeMode =PRTLIS3MDL_CTRLREG1_OPMODE_HIGHPERFORMANCEMODE;
		prtlis3mdl_registers.uCtrlReg1.scontrolReg1.b3ODR =PRTLIS3MDL_CTRLREG1_ODR_20HZ;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL1, 1, &(prtlis3mdl_registers.uCtrlReg1.ui8controlReg1));

		prtlis3mdl_registers.uCtrlReg2.ui8controlReg2 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg2.scontrolReg2.b2FullScaleConfoguration = PRTLIS3MDL_CTRLREG2_FULLSCALE_16GAUSS;
		//prtlis3mdl_registers.uCtrlReg2.scontrolReg2.bSoftReset = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL2, 1, &(prtlis3mdl_registers.uCtrlReg2.ui8controlReg2));
		i32Sensitivity = LIS3MDL_DSENSITIVITY_16GAUSS;


		prtlis3mdl_registers.uCtrlReg3.ui8controlReg3 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg3.scontrolReg3.b2Operatingmode = PRTLIS3MDL_CTRLREG3_OPMODE_CONTINUOUS;
		prtlis3mdl_registers.uCtrlReg3.scontrolReg3.bLP = PRTLIS3MDL_DISABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL3, 1, &(prtlis3mdl_registers.uCtrlReg3.ui8controlReg3));

		prtlis3mdl_registers.uCtrlReg4.ui8controlReg4 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg4.scontrolReg4.b2ZaxiOperatingMode = PRTLIS3MDL_CTRLREG1_OPMODE_LOWPOWER;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL4, 1, &(prtlis3mdl_registers.uCtrlReg4.ui8controlReg4));

	#if 1
		prtlis3mdl_registers.uCtrlReg5.ui8controlReg5 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		//prtlis3mdl_registers.uCtrlReg5.scontrolReg5.bBloackDataUpdate = PRTLIS3MDL_ENABLE	;
		prtlis3mdl_registers.uCtrlReg5.scontrolReg5.bFastRead = PRTLIS3MDL_ENABLE	;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL5, 1, &(prtlis3mdl_registers.uCtrlReg5.ui8controlReg5));

	#endif
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_INTTHRESHOLDL, 1, aui8Threshold);

		prtlis3mdl_registers.uIntReg.ui8IntConfig = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bXintEnable = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bYintEnable = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bZintEnable = PRTLIS3MDL_ENABLE;
		prtlis3mdl_registers.uIntReg.sIntConfig.bIntEnable = PRTLIS3MDL_DISABLE;
		//prtlis3mdl_registers.uIntReg.sIntConfig.bInteruptActive = PRTLIS3MDL_ENABLE;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_INTCONFIG, 2,&(prtlis3mdl_registers.uIntReg.ui8IntConfig) );

	}
	else
	{
		prtlis3mdl_registers.uCtrlReg3.ui8controlReg3 = PRTLIS3MDL_DEFAULTREFISTERVALUE;
		prtlis3mdl_registers.uCtrlReg3.scontrolReg3.b2Operatingmode = PRTLIS3MDL_CTRLREG3_OPMODE_POWERDOWN;
		prtlis3mdh_WriteRegister ( PRTLIS3MDL_REGISTER_CONTROL3, 1, &(prtlis3mdl_registers.uCtrlReg3.ui8controlReg3));
	}

}

/***************************************************************************//**
 * @brief 		This function configures the LIS3MDL to work as magnetic field
 * detector
 * @param[in] 	none
 * @param[out] 	i16MagneticField : the magnetic field value
 * @return 		none
*******************************************************************************/
void  prtlis3mdh_ReadXaxisField ( int16_t *i16MagneticField)
{
uint8_t *pui8DataRead ;

	prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_XAXISLSB, 2 , &pui8DataRead);
	*i16MagneticField = (int16_t) ((int32_t) (pui8DataRead[0])/i32Sensitivity);

}
/***************************************************************************//**
 * @brief 		This function configures the LIS3MDL to work as magnetic field
 * detector
 * @param[in] 	none
 * @param[out] 	i16MagneticField : the magnetic field value
 * @return 		none
*******************************************************************************/
void  prtlis3mdh_ReadYaxisField ( int16_t *i16MagneticField)
{
uint8_t *pui8DataRead ;

	prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_YAXISLSB, 2 , &pui8DataRead);
	*i16MagneticField = (int16_t) ((int32_t) (pui8DataRead[0])/i32Sensitivity);
}

/***************************************************************************//**
 * @brief 		This function configures the LIS3MDL to work as magnetic field
 * detector
 * @param[in] 	none
 * @param[out] 	i16MagneticField : the magnetic field value
 * @return 		none
*******************************************************************************/
void  prtlis3mdh_ReadZaxisField ( int16_t *i16MagneticField)
{
uint8_t *pui8DataRead;

	prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_ZAXISLSB, 2 , &pui8DataRead);
	*i16MagneticField = (int16_t) ((int32_t) (pui8DataRead[0])/i32Sensitivity);
}

/***************************************************************************//**
 * @brief 		This function configures the LIS3MDL to work as magnetic field
 * detector
 * @param[in] 	none
 * @param[out] 	i16XmagneticField : the magnetic field value on the X axis
 * @param[out] 	i16YmagneticField : the magnetic field value on the y axis
 * @param[out] 	i16ZmagneticField : the magnetic field value on the Z axis
 * @return 		none
*******************************************************************************/
void  prtlis3mdh_Read3axisField ( int16_t *i16XmagneticField, int16_t *i16YmagneticField ,int16_t *i16ZmagneticField )
{
uint8_t *pui8DataRead;
int32_t i32magneticField;

	prtlis3mdh_ReadRegister (PRTLIS3MDL_REGISTER_XAXISLSB, 6 , &pui8DataRead);
	i32magneticField = (int32_t )( *((int16_t*) (pui8DataRead)) )*1000;
	*i16XmagneticField = (int16_t) (i32magneticField/i32Sensitivity);
	i32magneticField = (int32_t ) ( *((int16_t*) (pui8DataRead+2)) )*1000;
	*i16YmagneticField = (int16_t) (i32magneticField/i32Sensitivity);
	i32magneticField = (int32_t ) ( *((int16_t*) (pui8DataRead+4)) )*1000;
	*i16ZmagneticField = (int16_t) (i32magneticField/i32Sensitivity);
}

/***************************************************************************//**
 * @brief 		This function enable  or diasble the data ready pad
 * detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prtlis3mdh_initDataReadypad ( const bool bEnableOrDisable )
{
	interface_EnableInputPad (INTERFACE_MAGNETODATAREADY_PORT,INTERFACE_MAGNETODATAREADY_PIN , bEnableOrDisable);
	if (true == bEnableOrDisable)
	{
		interface_EnablePinInterrupt (INTERFACE_MAGNETODATAREADY_PORT, INTERFACE_MAGNETODATAREADY_PIN, INTERFACE_RISING_EDGE);
	}
	else
	{
		interface_DisablePinInterrupt (INTERFACE_MAGNETODATAREADY_PORT, INTERFACE_MAGNETODATAREADY_PIN);
	}
}
/***************************************************************************//**
 * @brief 		This function enable  or disable the irq pad
 * detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prtlis3mdh_initIntpad ( const bool bEnableOrDisable )
{
	interface_EnableInputPad (INTERFACE_MAGNETOIRQ_PORT,INTERFACE_MAGNETOIRQ_PIN , bEnableOrDisable);
	if (true == bEnableOrDisable)
	{
		interface_EnablePinInterrupt (INTERFACE_MAGNETOIRQ_PORT, INTERFACE_MAGNETOIRQ_PIN, INTERFACE_RISING_EDGE);
	}
	else
	{
		interface_DisablePinInterrupt (INTERFACE_MAGNETOIRQ_PORT, INTERFACE_MAGNETOIRQ_PIN);
	}
}
