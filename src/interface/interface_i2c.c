/*******************************************************************************
 * @file interface_i2c.c
 * @brief this files contains the function set for the management of the I2C
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "interface_i2c.h"

/*===========================================================================================================
						Private variables declarations
===========================================================================================================*/
//static uint16_t i2c_ui16deviceAddress = INTERFACEI2C_DEFAULT_ADDRESS;

/* Transmission flags */
volatile static bool i2c_bRxInProgress;


/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief This function initializes the link to the 24LC64 (64 K I2C EEPROM)
 * @param[in] 	ui16I2CdeviceAddress - the address of the I2C slave
 * @param[out] 	none
 * @return 		CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE : the I2C device doesn't acknowledge
 * @return 		CROSSRFID_SUCCESSCODE : the I2C device acknowledges
*******************************************************************************/
uint8_t I2C_IsAvailable ( const uint8_t ui8address )
{
I2C_TransferSeq_TypeDef 	seq;
I2C_TransferReturn_TypeDef	i8Status;
uint8_t				ui8status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;
I2C_TypeDef 		*i2c 	= I2C0;

		/* fills the combined write/read sequence */
		seq.addr 		= (int16_t ) ui8address;
		seq.flags 		= I2C_FLAG_WRITE;
		seq.buf[0].len 	= 0;
		seq.buf[0].data = NULL;
		seq.buf[1].len 	= 0;
		seq.buf[1].data	= NULL;

		i8Status = I2C_TransferInit (I2C0,  &seq); /* initialized and start the I2C transfer */
		if (i2cTransferNack  == i8Status)
		{
			ui8status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;
		}
		else if (i2cTransferDone  == i8Status)
		{
			ui8status = CROSSRFID_SUCCESSCODE;
		}
		i2c->CMD        = I2C_CMD_STOP;

#if 0
I2C_TypeDef 		*i2c 	= I2C0;
uint32_t			ui32IrqPending=0 ;
uint8_t				ui8status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;

	/* Check if in busy state. Since this SW assumes single master, we can */
	/* just issue an abort. The BUSY state is normal after a reset. */
	if (i2c->STATE & I2C_STATE_BUSY)
	{
	  i2c->CMD = I2C_CMD_ABORT;
	}

	/* Prepare for a transfer */
	transfer->state   = i2cStateStartAddrSend;
	transfer->result  = i2cTransferInProgress;
	transfer->offset  = 0;
	transfer->bufIndx = 0;
	transfer->seq     = seq;

	//I2C_IntDisable(i2c, _I2C_IEN_MASK);
	i2c->CMD = I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
	//i2c->CMD = I2C_CMD_ABORT;
	/* send a start and the device address */
	i2c->TXDATA     = ui8address; /* Data not transmitted until START sent */
	i2c->CMD        = I2C_CMD_START;
    ui32IrqPending = I2C_IntGet (i2c);
    if (I2C_IF_ACK == (ui32IrqPending & I2C_IF_ACK))
    {
    	ui8status = CROSSRFID_SUCCESSCODE;
    }
    else
    {
    	ui8status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;
    }
    i2c->CMD        = I2C_CMD_STOP;
#endif

    return ui8status;

}

/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief This function initializes the link to the 24LC64 (64 K I2C EEPROM)
 * @param[in] 	ui16I2CdeviceAddress - the address of the I2C slave
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void I2C_init ( void )
{
I2C_Init_TypeDef sI2CConfiguration = I2C_INIT_DEFAULT;

	/* Saves the software address of the device into the i2c driver */
	//i2c_ui16deviceAddress = ui16I2CdeviceAddress;

	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(INTERFACE_I2CPWR_PORT, INTERFACE_I2CPWR_PIN, gpioModePushPull, 1);
	GPIO_PinOutSet (INTERFACE_I2CPWR_PORT, INTERFACE_I2CPWR_PIN);	/* power up the I2c devices*/
	GPIO_PinModeSet(INTERFACE_I2CSCL_PORT, INTERFACE_I2CSCL_PIN, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(INTERFACE_I2CSDA_PORT, INTERFACE_I2CSDA_PIN, gpioModeWiredAnd, 1);

	I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN ;
	I2C0->ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC15 | I2C_ROUTELOC0_SCLLOC_LOC15 ;

	/* default driver initialization */
	I2C_Init(I2C0, &sI2CConfiguration);

	/* Exit the BUSY state. The I2C will be in this state out of RESET. */
	if (I2C0->STATE & I2C_STATE_BUSY)
	{
		I2C0->CMD = I2C_CMD_ABORT;
	}else {/*do nothing*/}

	/* Enable the Clock Low Timeout counter */
	I2C0->CTRL = (I2C0->CTRL & ~_I2C_CTRL_CLTO_MASK) | I2C_CTRL_CLTO_160PCC;

	/* Enable error interrupts */
	I2C0->IEN |= I2C_IEN_ARBLOST | I2C_IEN_BUSERR | I2C_IEN_CLTO;

	/* Enable interrupts in NVIC */
	/*NVIC_ClearPendingIRQ(I2C0_IRQn);
	NVIC_EnableIRQ(I2C0_IRQn);*/
}

/***************************************************************************//**
 * @brief 		This function performs a random read with an address on 1 byte
 * @param[in] 	i2c_ui16deviceAddress: address of the I2C device
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[out] 	pui8ReadData: read buffer
 * @param[in] 	i2c_ui16deviceAddress: address of the I2C device
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead - number of byte to read
 * @return none
*******************************************************************************/
I2C_TransferReturn_TypeDef I2C_Read (const uint8_t i2c_ui16deviceAddress, const uint8_t ui8MemoryAdr, uint8_t ui8NbByteToRead, uint8_t *pui8ReadData)
{
I2C_TransferSeq_TypeDef 	seq;
I2C_TransferReturn_TypeDef	i8Status;
uint8_t data[1];

	data [0]= ui8MemoryAdr;
	/* fills the combined write/read sequence */
	seq.addr 		= i2c_ui16deviceAddress;
	seq.flags 		= I2C_FLAG_WRITE_READ;
	seq.buf[0].len 	= 1;
	seq.buf[0].data = data;
	seq.buf[1].len 	= ui8NbByteToRead;
	seq.buf[1].data	= pui8ReadData;

	i8Status = I2C_TransferInit (I2C0,  &seq); /* initialized and start the I2C transfer */
	while (i8Status == i2cTransferInProgress)  /* Do a polled transfer*/
	{
		i8Status = I2C_Transfer(I2C0);
	}

	return i8Status;
}


/***************************************************************************//**
 * @brief 		This function performs a current read with an address on 1 byte
 * @details		the address regsiter where the data is read shound be sent before
 * @param[in] 	i2c_ui16deviceAddress: address of the I2C device
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[out] 	pui8ReadData: read buffer
 * @param[in] 	i2c_ui16deviceAddress: address of the I2C device
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead - number of byte to read
 * @return none
*******************************************************************************/
I2C_TransferReturn_TypeDef I2C_CurrentRead (const uint8_t i2c_ui16deviceAddress , uint8_t ui8NbByteToRead, uint8_t *pui8ReadData)
{
I2C_TransferSeq_TypeDef 	seq;
I2C_TransferReturn_TypeDef	i8Status;

	/* fills the combined write/read sequence */
	seq.addr 		= i2c_ui16deviceAddress;
	seq.flags 		= I2C_FLAG_READ;
	seq.buf[0].len 	= ui8NbByteToRead;
	seq.buf[0].data = pui8ReadData;
	seq.buf[1].len 	= 0;
	seq.buf[1].data	= NULL;

	i8Status = I2C_TransferInit (I2C0,  &seq); /* initialized and start the I2C transfer */
	while (i8Status == i2cTransferInProgress)  /* Do a polled transfer*/
	{
		i8Status = I2C_Transfer(I2C0);
	}

	return i8Status;
}

/***************************************************************************//**
 * @brief 		This function performs a random read with an address on 2bytes
 * @param[in] 	i2c_ui16deviceAddress: address of the I2C device
 * @param[in] 	ui16MemoryAdr: the first address to read in the memory
 * @param[in] 	ui8NbByteToRead: number of byte to read
 * @param[out] 	pui8ReadData: read buffer
 * @return
 *   @li #i2cTransferInProgress - indicates that transfer not finished.
 *   @li #i2cTransferDone - transfer completed successfully.
 *   @li otherwise some sort of error has occurred.
 *
*******************************************************************************/
I2C_TransferReturn_TypeDef I2C_Readwith2AddressBytes (const uint8_t i2c_ui16deviceAddress, const uint16_t ui16MemoryAdr, uint8_t ui8NbByteToRead, uint8_t *pui8ReadData)
{
	I2C_TransferSeq_TypeDef seq;
	uint8_t 				ui8Address[2];
	I2C_TransferReturn_TypeDef	i8Status;

	/* TxData are the address */
	ui8Address[0] = (uint8_t)INTERFACEI2C_GETMSB(ui16MemoryAdr);
	ui8Address[1] = (uint8_t)INTERFACEI2C_GETLSB(ui16MemoryAdr);

	/* fills the combined write/read sequence */
	seq.addr 		= i2c_ui16deviceAddress;
	seq.flags 		= I2C_FLAG_WRITE_READ;
	seq.buf[0].len 	= 2;
	seq.buf[0].data = &ui8Address[0];
	seq.buf[1].len 	= ui8NbByteToRead;
	seq.buf[1].data	= pui8ReadData;

	i8Status = I2C_TransferInit (I2C0,  &seq); /* initialized and start the I2c transfer */
	while (i8Status == i2cTransferInProgress)  /* Do a polled transfer*/
	{
		i8Status = I2C_Transfer(I2C0);
	}

	return i8Status;
}

/***************************************************************************//**
 * @brief This function performs a 1byte write operation with an address on
 * 2 bytes
 * @param[in] i2c_ui16deviceAddress: address of the I2C device
 * @param[in] ui16MemoryAdr: the first address to read in the memory
 * @param[out] ui8NbByteTowrite: number of byte to read
 * @return none
*******************************************************************************/
void I2C_WriteByteWith2AddressBytes (const uint8_t i2c_ui16deviceAddress, const uint16_t ui16MemoryAdr, const uint8_t ui8ByteToWrite)
{
	I2C_TransferSeq_TypeDef seq;
	uint8_t 				ui8TxData[3];
	int8_t 					i8Status;

	ui8TxData[0] = (uint8_t)INTERFACEI2C_GETMSB(ui16MemoryAdr);
	ui8TxData[1] = (uint8_t)INTERFACEI2C_GETLSB(ui16MemoryAdr);
	ui8TxData[2] = ui8ByteToWrite;

	/* fills the combined write/read sequence */
	seq.addr 		= i2c_ui16deviceAddress;
	seq.flags 		= I2C_FLAG_WRITE;
	seq.buf[0].len 	= 2+1;
	seq.buf[0].data = &ui8TxData[0];

	i8Status = I2C_TransferInit (I2C0,  &seq); /* initialized and start the I2c transfer */
	while (i8Status == i2cTransferInProgress)  /* Do a polled transfer*/
	{
		i8Status = I2C_Transfer(I2C0);
	}
}

/***************************************************************************//**
 * @brief This function initiates a write command
 * @param[in] i2c_ui16deviceAddress: address of the I2C device
 * @param[in] ui8NbBytePerAddress: the number of byte in the memory address
 * @param[in] ui16NbByteToWrite: the number of byte to read
 * @param[in] pui8DataToWrite: the buffer to write
 * @return
 *   @li #i2cTransferInProgress - indicates that transfer not finished.
 *   @li #i2cTransferDone - transfer completed successfully.
 *   @li #i2cTransferUsageFault -  buffer overflow
 *   @li otherwise some sort of error has occurred.
 *
*******************************************************************************/
I2C_TransferReturn_TypeDef I2C_Write (const uint8_t i2c_ui16deviceAddress, const uint8_t ui8NbBytePerAddress, const uint16_t ui16NbByteToWrite, uint8_t const * pui8DataToWrite)
{
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef i8Status;

	if((uint32_t)(ui8NbBytePerAddress + ui16NbByteToWrite) > 0xFFFF)
	{
		i8Status = i2cTransferUsageFault;
	}
	else
	{
		/* fills the combined write/read sequence */
		seq.addr 		= i2c_ui16deviceAddress;
		seq.flags 		= I2C_FLAG_WRITE;
		seq.buf[0].len 	= (uint16_t)(ui8NbBytePerAddress+ui16NbByteToWrite);
		seq.buf[0].data = (uint8_t*)pui8DataToWrite;
		seq.buf[1].len 	= 0;
		seq.buf[1].data = NULL;

		i8Status = I2C_TransferInit (I2C0,  &seq); /* initialized and start the I2c transfer */
		while (i8Status == i2cTransferInProgress)  /* Do a polled transfer*/
		{
			i8Status = I2C_Transfer(I2C0);
		}
	}

	return i8Status;
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C0_IRQHandler(void)
{
uint32_t ui32Flag;

  ui32Flag = I2C0->IF;

	if (ui32Flag & I2C_IF_ADDR) /* Address Match */
	{
		/* Indicating that reception is started */
		i2c_bRxInProgress = true;
		I2C0->RXDATA;
		I2C_IntClear(I2C0, I2C_IFC_ADDR);
	}
	if (ui32Flag & I2C_IF_RXDATAV) /* Data received */
	{
		//i2c_rxBuffer[i2c_rxBufferIndex] = I2C0->RXDATA;
		//i2c_rxBufferIndex++;
	}
	if (ui32Flag & ( I2C_IF_ARBLOST |   I2C_IF_BUSERR))
	{
		I2C_IntClear(I2C0, (I2C_IF_ARBLOST|  I2C_IF_BUSERR));
	}

	if(ui32Flag & I2C_IEN_SSTOP)
	{
		/* Stop received, reception is ended */
		I2C_IntClear(I2C0, I2C_IEN_SSTOP);
		i2c_bRxInProgress = false;
		//i2c_rxBufferIndex = 0;
	}

}

/***************************************************************************//**
 * @brief
 *   This function turn on or off the power supply of the external memory
 * @param[in]	OnOrOff : when true the power supplied will turn on otherwise off
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void I2C_EnablePowerSuppply ( bool bOnOrOff )
{
	if (true == bOnOrOff)
	{
		GPIO_PinOutSet(INTERFACE_I2CPWR_PORT, INTERFACE_I2CPWR_PIN);
		GPIO_PinModeSet(INTERFACE_I2CSCL_PORT, INTERFACE_I2CSCL_PIN, gpioModeWiredAnd, 1);
		GPIO_PinModeSet(INTERFACE_I2CSDA_PORT, INTERFACE_I2CSDA_PIN, gpioModeWiredAnd, 1);
	}
	else
	{
		GPIO_PinOutClear(INTERFACE_I2CPWR_PORT, INTERFACE_I2CPWR_PIN);
		GPIO_PinModeSet(INTERFACE_I2CSCL_PORT, INTERFACE_I2CSCL_PIN, gpioModeDisabled, 0);
		GPIO_PinModeSet(INTERFACE_I2CSDA_PORT, INTERFACE_I2CSDA_PIN, gpioModeDisabled, 0);
	}
}
