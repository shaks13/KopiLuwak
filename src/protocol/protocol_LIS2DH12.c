/****************************************************************************************
 * @file LIS2DH12
 * @brief Header file of LIS2DH12 Driver
 * @autor
******************************************************************************************
 *
 * */

/*****************************************************************************************
 ********************************* Includes files ****************************************
 *****************************************************************************************/

#include "protocol_LIS2DH12.h" 	// LIS2DH12 definitions

/*****************************************************************************************
 ********************************* Variables definitions**********************************
 *****************************************************************************************/
static SPIDRV_Handle_t 	prtLIS2DH12_psSpiHandle;	/* pointer of the handle of the SPI interface*/
/* Global variable : Axis data buffer */
LIS2DH12_AxisData_Union LIS2DH12_uAxisData;

/* Manages the access of the axis data buffer */
static bool 	LIS2DH12_bLockAxisData;
/* Measurement range */
static uint8_t 	protocol_selectedRange     = 0;
/* Resolution */
static uint8_t 	protocol_fullResolutionSet = 0;
/* contains RX & TX SPI data*/
static uint8_t 	protocol_auCommSPIBuffer [PROTOCOL_LIS2DH12_BUFFERSIZE];

/*****************************************************************************************
 ********************************* Functions Definitions *********************************
 *****************************************************************************************/

static uint8_t SPI_Read (const uint8_t registerAddress, uint8_t ui8NbByteToRead );
static uint8_t SPI_Write (uint8_t registerAddress, uint8_t *pdataBuffer, uint8_t ui8NbByteToWrite );
//static void LIS2DH12_Getz( int16_t* z);
static void LIS2DH12_GetXyz(uint16_t* x, uint16_t* y, uint16_t* z);
/*****************************************************************************************
 ********************************* Functions *********************************************
 *****************************************************************************************/

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param[in] registerAddress - register addresss to read the data
 * @param[in] ui8NbByteToRead - number of byte to read
 * @return none
*******************************************************************************/

static uint8_t SPI_Read (const uint8_t registerAddress, uint8_t ui8NbByteToRead )
{
#if USEDMA | 1
	uint8_t status;

	if (ui8NbByteToRead > 1 )	/* when more than 1 bytes should be read*/
	{
		protocol_auCommSPIBuffer[0] = LIS2DH12_SPI_READ | LIS2DH12_SPI_MB |registerAddress;
	}
	else
	{
		protocol_auCommSPIBuffer[0] = LIS2DH12_SPI_READ | LIS2DH12_SPI_SINGLEBYTE | registerAddress;
	}

	/* start dma transfers and wait their completion */
	status= SPIDRV_MTransferB( 	prtLIS2DH12_psSpiHandle,
								protocol_auCommSPIBuffer,
								protocol_auCommSPIBuffer,
								ui8NbByteToRead+PROTOCOL_LIS2DH12_NBBYTE_READ);

	if (status == ECODE_EMDRV_SPIDRV_OK)
	{
		status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		status = CROSSRFID_READSENSOR_ERROR;
	}
	return status;

#else
	//uNbBytereceived = 0;
	USART_IntEnable (USART1,USART_IEN_RXDATAV);
	GPIO_PinOutClear (INTERFACE_SPI2CS_PORT, INTERFACE_SPI2CS_PIN);	 				/* activate Spi bus*/
	interface_SendSpiBuffer( pdataBuffer, PROTOCOL_LIS2DH12_NBCMDBYTE);		/* send the first byte */
	interface_SendSpiBuffer( NULL, uNbbyte);
	GPIO_PinOutSet (INTERFACE_SPI2CS_PORT, INTERFACE_SPI2CS_PIN);					/* disable Spi bus*/
	USART_IntDisable (USART1,USART_IEN_RXDATAV);
#endif
}


static uint8_t SPI_Readloop (const uint8_t registerAddress, uint8_t ui8NbByteToRead, uint16_t ui16nbloop )
{
#if USEDMA | 1
	uint8_t status;

	if (ui8NbByteToRead > 1 )	/* when more than 1 bytes should be read*/
	{
		protocol_auCommSPIBuffer[0] = LIS2DH12_SPI_READ | LIS2DH12_SPI_MB |registerAddress;
	}
	else
	{
		protocol_auCommSPIBuffer[0] = LIS2DH12_SPI_READ | LIS2DH12_SPI_SINGLEBYTE | registerAddress;
	}

	/* start dma transfers and wait their completion */
	status= SPIDRV_MTransferBloop( 	prtLIS2DH12_psSpiHandle,
										protocol_auCommSPIBuffer,
										protocol_auCommSPIBuffer,
										ui8NbByteToRead+PROTOCOL_LIS2DH12_NBBYTE_READ,
										ui16nbloop);

	if (status == ECODE_EMDRV_SPIDRV_OK)
	{
		status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		status = CROSSRFID_READSENSOR_ERROR;
	}
	return status;
#endif
}
/*#################################################################################*/


/***************************************************************************//**
 * @brief Writes the value of a register.
 *
 * @param uSlaveId - Id of the SPI slave  (not used)
 * @param pdataBuffer - pointer of the data
 * @param uNbbyte - number of byte to write
 *
 * @return none
*******************************************************************************/

static uint8_t SPI_Write (uint8_t registerAddress, uint8_t *pdataBuffer, uint8_t ui8NbByteToWrite )
{

uint8_t status = CROSSRFID_SUCCESSCODE;
#if (USESTARTERKIT==1)
	if (ui8NbByteToWrite > 1 )	/* when more than 1 bytes should be read*/
	{
		protocol_auCommSPIBuffer[0] = LIS2DH12_SPI_WRITE | LIS2DH12_SPI_MB |registerAddress;
		memcpy(&(protocol_auCommSPIBuffer[1]),pdataBuffer,ui8NbByteToWrite );
	}
	else
	{
		protocol_auCommSPIBuffer[0] = LIS2DH12_SPI_WRITE | LIS2DH12_SPI_SINGLEBYTE | registerAddress;
		protocol_auCommSPIBuffer[1] =  *pdataBuffer;
	}


	/* start dma Tx and wait their completion */
	status = SPIDRV_MTransmitB( 	prtLIS2DH12_psSpiHandle,
									protocol_auCommSPIBuffer,
									ui8NbByteToWrite+PROTOCOL_LIS2DH12_NBBYTE_WRITE);

	if (status == ECODE_EMDRV_SPIDRV_OK)
	{
		status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		status = CROSSRFID_READSENSOR_ERROR;
	}
#elif (USECROSSTAG==1)

#endif
	return status;

}

/*#################################################################################*/

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param[in] ui8registerAddress - Address of the register.
 * @param[in] egisterAddress - Address of the register.
 *
 * @return registerValue  - Value of the register.
*******************************************************************************/
uint8_t prtLIS2DH_GetRegisterValue(const uint8_t ui8registerAddress,const uint8_t ui8NbByteToRead, uint8_t *ui8registerValue)
{
uint8_t status = CROSSRFID_SUCCESSCODE;

#if (USESTARTERKIT==1)
		status = SPI_Read(ui8registerAddress, ui8NbByteToRead);
		(*ui8registerValue) = protocol_auCommSPIBuffer[1];
#elif (USECROSSTAG==1)

#endif

	return status;
}

/*#################################################################################*/



/***************************************************************************//**
 * @brief 		this function deinitializes the peripheral to communicate
 * with the LIS2DH
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
uint8_t prtLIS2DH_Init ( void )
{
uint8_t status = 0;
#if USESTARTERKIT==1
SPIDRV_Init_t sInitData = SPIDRV_MASTER_USART0_FORLIS2DH; /* Configuration data for SPI master using USART0 */

#elif USECROSSTAG==1

#else

#endif


#if (USESTARTERKIT==1)
	/* configure psSpiHandle thanks to prtLIS2DH12_psSpiHandle */
	prtLIS2DH12_psSpiHandle = &(interface_sSpiHandle[INTERFACE_SENSOR1]);

	/* Initialize a instance "interface_psSpiHandle" and the USARTx according to sInitData*/
	SPIDRV_Init( prtLIS2DH12_psSpiHandle , &sInitData);

	status = LIS2DH12_Configure();	/*initialize the device*/
	if (CROSSRFID_SUCCESSCODE == status)
	{
		interface_InitLIS2DHIntGPIO (); /* initializes the GPIO for the IRQ and the IRQ*/
		//prsdrv_setupPrs ();
		prtLIS2DH_GetGzloop ( 3 );
		//LDMA_Init( &sInitDma );
	}
	else
	{
		prtLIS2DH_Deinit ();
	}
#elif (USECROSSTAG==1)

	/* todo init the I2C interface*/
#endif
	return status;
}

/***************************************************************************//**
 * @brief 		this function deinitializes the peripheral to communicate
 * with the LIS2DH
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void prtLIS2DH_Deinit ( void )
{
#if (USESTARTERKIT==1)
	SPIDRV_DeInit( prtLIS2DH12_psSpiHandle );
#elif (USECROSSTAG==1)

#endif


}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue   - Data value to write.
 *
 * @return None.
*******************************************************************************/

void prtLIS2DH_SetRegisterValue(unsigned char registerAddress,uint8_t registerValue){


#if (LIS2DH12_SPI_COMM == 1)
		// SPI Communication
		SPI_Write(registerAddress, &registerValue, 1);

#else
		// I2C Communication
		if(SAO_CONFIG == 0){
			protocol_auCommSPIBuffer[0] = registerAddress;
			protocol_auCommSPIBuffer[1] = registerValue;
			I2C_Write(LIS2DH12_SA00_I2C_WRITE, protocol_auCommSPIBuffer, 2, 1);
		}
		else{
			protocol_auCommSPIBuffer[0] = registerAddress;
			protocol_auCommSPIBuffer[1] = registerValue;
			I2C_Write(LIS2DH12_SA01_I2C_WRITE, protocol_auCommSPIBuffer, 2, 1);
		}
#endif

}

/*#################################################################################*/

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the LIS2DH12
 *		  part is present.
 *
 * @param commProtocol - SPI or I2C protocol.
 *                       Example: LIS2DH12_SPI_COMM
 *                                LIS2DH12_I2C_COMM
 *
 * @return status      - Result of the initialization procedure.
 * CROSSRFID_INITSENSOR_ERROR - I2C/SPI peripheral was not initialized or
 * LIS2DH12 part is not present.
 * CROSSRFID_SUCCESSCODE - I2C/SPI peripheral is initialized and
 *  LIS2DH12 part is present.
*******************************************************************************/
uint8_t LIS2DH12_Configure ( void )
{
uint8_t status = CROSSRFID_SUCCESSCODE;
uint8_t ui8registerValue =0;
uint8_t ui8DymmyBuffer[6] ;
//uint8_t idx = 0;

    /* Axis data buffer available */
    LIS2DH12_bLockAxisData = false;

    /* Checks the LIS2DH12 ID */
    status = prtLIS2DH_GetRegisterValue(LIS2DH12_WHO_AM_I ,1,&ui8registerValue);
    if((status != CROSSRFID_SUCCESSCODE ) || (ui8registerValue != LIS2DH12_ID))
    {
        status = CROSSRFID_INITSENSOR_ERROR;
    }
    else
    {
    	//LIS2DH12_SetRegisterValue(LIS2DH12_TEMP_CFG_REG,LIS2DH12_TEMPERATURE_ENABLE);  	/* Temperature enable */
    	prtLIS2DH_SetRegisterValue(LIS2DH12_CTRL_REG1,LIS2DH12_REG1_100HZ);			  	/* 5376 Hz Data output rate configuration */
    	prtLIS2DH_SetRegisterValue(LIS2DH12_CTRL_REG2,LIS2DH12_REG2_ENABLEHF);		  	/* enable the High pass filter*/
    	prtLIS2DH_SetRegisterValue(LIS2DH12_CTRL_REG3,LIS2DH12_REG3_DATAREADY);			/* Dataready IRQ enable*/
    	prtLIS2DH_SetRegisterValue(LIS2DH12_CTRL_REG4,LIS2DH12_REG4_4G);				   	/* +- 4g resolution */
    	prtLIS2DH_SetRegisterValue(LIS2DH12_CTRL_REG6,0x02);				   				/* +- 4g resolution */
    	//LIS2DH12_SetRegisterValue(LIS2DH12_CTRL_REG5,LIS2DH12_REG5_ENABLEFIFO);			/* enable fifo */
    	//LIS2DH12_SetRegisterValue(LIS2DH12_FIFO_CTRL_REG,0x80);							/* enable fifo */
    	prtLIS2DH_SetRegisterValue(LIS2DH12_INT1_CFG,0x30);
    	//LIS2DH12_SetRegisterValue(LIS2DH12_INT1_THS,0x00);
    	prtLIS2DH_SetRegisterValue(LIS2DH12_INT1_DURATION,0x00);

    	/* dummy read the register output*/
    	//LIS2DH12_GetRegisterValue (LIS2DH12_STATUS_REG,1,ui8DymmyBuffer);	/* not used*/
    	prtLIS2DH_GetRegisterValue (LIS2DH12_OUT_Z_L,2,ui8DymmyBuffer);	/* not used*/

    }
    /* Measurement Range: +/- 2g (reset default). */
    protocol_selectedRange = 4;
    protocol_fullResolutionSet = 0;

	return status;
}



/*#################################################################################*/
/***************************************************************************//**
 * @brief Reads the raw output data of each axis.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/

static void LIS2DH12_GetXyz(uint16_t* x, uint16_t* y, uint16_t* z){
uint8_t *readBuffer   = protocol_auCommSPIBuffer ;

    SPI_Read(LIS2DH12_OUT_X_L,  6);
    /* x = ((LIS2DH12_OUT_X_H) << 8) + LIS2DH12_OUT_X_L */
    *x = ((short)readBuffer[2] << 8) + readBuffer[1];
    /* y = ((LIS2DH12_OUT_Y_H) << 8) + LIS2DH12_OUT_Y_L */
    *y = ((short)readBuffer[4] << 8) + readBuffer[3];
    /* z = ((LIS2DH12_OUT_Z_H) << 8) + LIS2DH12_OUT_Z_L */
    *z = ((short)readBuffer[6] << 8) + readBuffer[5];
}
/*#################################################################################*/
/***************************************************************************//**
 * @brief Reads the raw output data of z axis.
 *
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
#if 0
static void LIS2DH12_Getz( int16_t* z)
{
int8_t *readBuffer   = protocol_auCommSPIBuffer ;

    SPI_Read(LIS2DH12_OUT_Z_L,  2);
    (*z) = ( ( (int16_t) (readBuffer[2] << 8) & 0xFF00) | readBuffer[1] );
}
#endif
/*#################################################################################*/
/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to mg.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/

void prtLIS2DH_GetGxyz(float32_t* x,float32_t* y,float32_t* z){

uint16_t xData = 0;
uint16_t yData = 0;
uint16_t zData = 0;

	LIS2DH12_GetXyz(&xData,&yData,&zData);
    *x = (float32_t)(xData * LIS2DH12_SCALE_FACTOR * (protocol_selectedRange));
    *y = (float32_t)(yData * LIS2DH12_SCALE_FACTOR * (protocol_selectedRange));
    *z = 1024 - (float32_t)(zData * LIS2DH12_SCALE_FACTOR * (protocol_selectedRange));

}

/*#################################################################################*/
/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to mg.
 *
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/

void prtLIS2DH_GetGz(float32_t * z)
{
int16_t zData = 0;
float32_t f32ScaleFactor = LIS2DH12_SCALE_FACTOR * (protocol_selectedRange);
int8_t *readBuffer   = (int8_t*)protocol_auCommSPIBuffer ;
	//LIS2DH12_Getz(&zData);

	SPI_Read(LIS2DH12_OUT_Z_L,  2);
	(*z) = ( ( (int16_t) (readBuffer[2] << 8) & 0xFF00) | readBuffer[1] );
    (*z) = /*1024 -*/ ((float32_t)zData * f32ScaleFactor);
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to mg.
 *
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/

void prtLIS2DH_GetGzloop(uint16_t ui16nbloop)
{

	SPI_Readloop(LIS2DH12_OUT_Z_L,  2,   ui16nbloop);

}


/*#################################################################################*/

/***************************************************************************//**
 * @brief Reads the raw output data of temperature.
 *
 * @param temp - temperature output data.
 *
 * @return None.
*******************************************************************************/

void prtLIS2DH_GetTemp(short* temp){

	unsigned char *readBuffer   = protocol_auCommSPIBuffer ;

    SPI_Read(LIS2DH12_OUT_TEMP_L,  2);
	*temp = ((short)readBuffer[2] << 8) | readBuffer[1];
}

/*#################################################################################*/

/**************************************************************************//**
 * @brief Plug the Tx/Rx buffer for the SPI communication
 *****************************************************************************/
#if 0
void LIS2DH12_Plugbuffer (void){

interface_SetTxRxSPIbuffer (0,protocol_auCommSPIBuffer,0,protocol_auCommSPIBuffer);
}
#endif
/*#################################################################################*/

/***************************************************************************//**
 * @brief Saves the raw output data of each axis into the Axis Data buffer.
 *
 * @param None.
 *
 * @return CROSSRFID_SUCCESSCODE : the read operation is successful.
 *		   CROSSRFID_READSENSOR_ERROR : the read operation is not successful.
*******************************************************************************/

uint8_t prtLIS2DH_SaveAxisData(void)
{
	uint8_t					ustatus = CROSSRFID_READSENSOR_ERROR;
	unsigned char 			*readBuffer   = protocol_auCommSPIBuffer ;
	Kernel_QueueItem_struct pQueueItem = {((( KERNEL_SENSORTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | /* the receiver is the Kernel */
										((KERNEL_SENSORTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,	/* the sender is the Serial task */
											KERNEL_MESSAGEID_DATARECEIVED, 	/* the message is that a accelerometer's data has been received */
										0,NULL};

	/* Checks if the LIS2DH12_bLockAxisData is available */
	if(false == LIS2DH12_bLockAxisData)
	{

		SPI_Read(LIS2DH12_OUT_X_L , 6);
		/* x = ((ADXL345_DATAX1) << 8) + ADXL345_DATAX0 */
		LIS2DH12_uAxisData.axis.x = ((short)readBuffer[2] << 8) + readBuffer[1];
		/* y = ((ADXL345_DATAY1) << 8) + ADXL345_DATAY0 */
		LIS2DH12_uAxisData.axis.y = ((short)readBuffer[4] << 8) + readBuffer[3];
		/* z = ((ADXL345_DATAZ1) << 8) + ADXL345_DATAZ0 */
		LIS2DH12_uAxisData.axis.z = ((short)readBuffer[6] << 8) + readBuffer[5];

		/* posts a message to the kernel */
		pQueueItem.ui16NbByte = 6;
		pQueueItem.pData = &LIS2DH12_uAxisData.aui8Axis[0];
		xQueueSend (sKernel_QueuePointer.pKernelQueue,&pQueueItem, pdFALSE);
		/* Buffer no longer available */
		LIS2DH12_bLockAxisData = true;
		/* successful read data */
		ustatus = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		/* Do nothing */
		__NOP();
	}

	return ustatus;
}

/*#################################################################################*/

/***************************************************************************//**
 * @brief Releases the Axis Data buffer.
 * @param None.
 * @return None.
*******************************************************************************/
void prtLIS2DH_ReleaseAxixDataBuffer(void)
{
	LIS2DH12_bLockAxisData = false;
}

/*#################################################################################*/

/*#################################################################################*/

/***************************************************************************//**
 * @brief this function cleans the ADXL memory in order to activate the irq
 *
 * @param[in] None.
 * @param[out] none
 * @return None.
 *****************************************************************************/
void prtLIS2DH_CleanBuffer(void){
uint16_t x;
uint16_t y;
uint16_t z;

	LIS2DH12_GetXyz(&x,&y, &z);

}

/*#################################################################################*/

