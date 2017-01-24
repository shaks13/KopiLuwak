/*******************************************************************************
 * @file protocol_24LC64.c
 * @brief this files contains the function set for the 24LC64 (64 K I2C EEPROM)
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "protocol_24LC64.h"

/*===========================================================================================================
						Private functions declarations
===========================================================================================================*/
#if 0
static void prt24LC64_Read (const uint16_t ui16MemoryAdr, uint8_t ui8NbByteToRead, uint8_t *pui8ReadData );
#endif
/*===========================================================================================================
						Public variables declarations
===========================================================================================================*/
/* i2c Buffer */
uint8_t prt24lc64_ui8Buffer[PRT24LC64_LENGTH_RX_BUFFER];

/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief This function initializes the link to the 24LC64 (64 K I2C EEPROM)
 * @param[in] none
 * @param[out] none
 * @return none
*******************************************************************************/
void prt24LC64_init (void)
{
	I2C_init ( );
}

/***************************************************************************//**
 * @brief This function performs a random read
 * @param[in] ui16MemoryAdr: the first address to read in the memory
 * @param[in] ui8NbByteToRead: number of byte to read
 * @return 	CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * 			CROSSRFID_ERROR_I2C_READ - an error has occurred.
*******************************************************************************/
uint8_t prt24LC64_RandomRead (const uint16_t ui16MemoryAdr, uint8_t ui8NbByteToRead)
{
	uint8_t ui8Status;

	if (i2cTransferDone == I2C_Readwith2AddressBytes (PRT24LC64_ADDRESS, ui16MemoryAdr,ui8NbByteToRead,&prt24lc64_ui8Buffer[0]))
	{
		ui8Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_I2C_READ;
	}

#if 0
	I2C_EnablePowerSuppply (false);
#endif
	return ui8Status;
}
#if 0
/***************************************************************************//**
 * @brief This function write 1 byte
 * @param[in] ui16MemoryAdr: the first address to read in the memory
 * @param[in] ui8ByteToWrite: the byte to write
 * @return none
*******************************************************************************/
void prt24LC64_WriteByte (const uint16_t ui16MemoryAdr, const uint8_t ui8ByteToWrite)
{
	I2C_WriteByteWith2AddressBytes (ui16MemoryAdr,ui8ByteToWrite);
}
#endif
/***************************************************************************//**
 * @brief This function initiates the write command
 *
 * @detail The max write cycle time is 5 ms (byte or page).
 *
 * @param[in] ui16NbByteToWrite: the number of byte to read
 * @param[in] pui8DataToWrite: the buffer to write
 * @return 	CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * 			CROSSRFID_ERROR_I2C_WRITE - an error has occurred.
*******************************************************************************/
uint8_t prt24LC64_Write (const uint16_t ui16NbByteToWrite, uint8_t const * pui8DataToWrite)
{
	uint8_t ui8Status;

	if (i2cTransferDone == I2C_Write(PRT24LC64_ADDRESS,2,ui16NbByteToWrite,pui8DataToWrite))
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
 * @brief
 *   This function turn on or off the power supply of the external memory
 * @param[in]	OnOrOff : when true the power supplied will turn on otherwise off
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void prt24LC64_Enable24LC64 ( bool bOnOrOff )
{
	I2C_EnablePowerSuppply ( bOnOrOff );

}

