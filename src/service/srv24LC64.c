/*******************************************************************************
 * @file srv24LC64.c
 * @brief this files contains the function set for the 24LC64 (64 K I2C EEPROM)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "srv24LC64.h"

/*===========================================================================================================
						Public variable declaration
===========================================================================================================*/
srv24LC_MemoryContent_struct srv24LC_MemoryContent;

/*===========================================================================================================
						Private variable declaration
===========================================================================================================*/

/*===========================================================================================================
						Private function declaration
===========================================================================================================*/
static uint8_t srv24lc64_Write16bitData 			( const uint16_t ui16Address, const uint16_t ui16NbByteToWrite, uint8_t const * pui8DataToWrite );
static uint8_t srv24lc64_Read16bitData 				( const uint16_t ui16Address, const uint16_t ui16NbByteToRead, uint16_t ** ppui16ReadData );



/***************************************************************************//**
 * @brief This function write data at an address in the memory
 *
 * @detail 	this function is going to prepare the bytes to write
 * 			in 16bit format (address and data) then starting the transfer.
 * 			The max write cycle time is 5 ms (byte or page).
 *
 * @param[in] ui16Address: the address where the data will be written
 * @param[in] ui16NbByteToWrite: the number of byte to write (only data)
 * @param[in] pui8DataToWrite: the data to write
 * @return 	CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 	CROSSRFID_ERROR_I2C_WRITE - an error has occurred during the i2c tansfer.
 * @return 	CROSSRFID_ERROR_24LC64_INVALID_ADRS - invalid 24lc64 address
*******************************************************************************/
static uint8_t srv24lc64_Write16bitData ( const uint16_t ui16Address, const uint16_t ui16NbByteToWrite, uint8_t const * pui8DataToWrite )
{
	uint16_t ui16Idx = 0;
	uint16_t ui8Status;

	if( ui16Address < PRT24LC64_LIMIT_INVALID_ADRS)
	{
		/* prepares the bytes to write in 16bit format (address and data) */
		prt24lc64_ui8Buffer[0] = (ui16Address & 0xFF00) >> 8;
		prt24lc64_ui8Buffer[1] = ui16Address & 0x00FF;
		for(ui16Idx = 2; ui16Idx < ui16NbByteToWrite+2 ; ui16Idx ++) /* nb bytes + 2 address bytes */
		{
			prt24lc64_ui8Buffer[ui16Idx] = *pui8DataToWrite;
			pui8DataToWrite++;
		}

#if 0
		srv24LC_Enable24LC64(true);
#endif
		/* writes the buffer in memory*/
		ui8Status =  prt24LC64_Write(ui16NbByteToWrite,&prt24lc64_ui8Buffer[0]);
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_EXTEEPROM_INVALID_ADRS;
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief 		This function read at an address in the memory
 * @note 		the protocol layer has its own buffer to save the I2C data. thaht's why
 * 				a point to pointer is used
 * @param[in] 	ui16Address: the address where the data will be read
 * @param[in] 	ui16NbByteToRead: the number of byte to read (only data)
 * @param[in] 	ppui16ReadData: the address of the read buffer
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_READ - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_24LC64_INVALID_ADRS - invalid 24lc64 address
*******************************************************************************/
static uint8_t srv24lc64_Read16bitData ( const uint16_t ui16Address, const uint16_t ui16NbByteToRead, uint16_t ** ppui16ReadData )
{
	uint8_t ui8Status;

	if( ui16Address < PRT24LC64_LIMIT_INVALID_ADRS)
	{
#if 0
		srv24LC_Enable24LC64(true);
#endif
		ui8Status = prt24LC64_RandomRead(ui16Address,ui16NbByteToRead);
		*ppui16ReadData = (uint16_t*) prt24lc64_ui8Buffer;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_EXTEEPROM_INVALID_ADRS;
	}

	return ui8Status;
}

/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/

/***************************************************************************//**
 * @brief This function initializes the service layer of the 24LC64
 * (64 K I2C EEPROM)external memory
 * @param[in] none
 * @param[out] none
 * @return none
*******************************************************************************/
void srv24LC64_init (void)
{
	prt24LC64_init ();
}

#if 0
/***************************************************************************//**
 * @brief This function initializes the service layer of the 24LC64
 * (64 K I2C EEPROM)external memory
 * @param[in] none
 * @param[out] none
 * @return none
*******************************************************************************/
void srv24LC64_GetAlarmStatus (uint16_t *ui16AlarmStatus )
{
	(*ui16AlarmStatus) = serial_tui16AlarmDataInEeprom[SERIAL_ADDRESS_24LC64_ALARMBITFIELD];
}
#endif



/***************************************************************************//**
 * @brief
 *   This function reads the stored data in external eeprom memory about alarms
 *
 * @detail the external memory map is the following
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Bit field of the states alarm 			|
 * 	|	0x0001		|		2			|	Date of the "below temp" One shot alarm	|
 * 	|	0x0003		|		2			|	Time of the "below temp" One shot alarm	|
 * 	|	0x0005		|		2			|	Date of the "upper temp" One shot alarm	|
 * 	|	0x0007		|		2			|	Time of the "upper temp" One shot alarm	|
 * 	|	0x0009		|		2			|	Date of the "low bat" One shot alarm	|
 * 	|	0x000B		|		2			|	Time of the "low bat" One shot alarm	|
 * 	|	0x000D		|		1			|	CRC16 CCITT 							|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in]	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_READ - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_CRC_ALARM_EXTEEPROM - the crc is wrong
 ******************************************************************************/
uint8_t srv24LC_ReadAlarmFields ( uint16_t **ppui16AlarmStatus )
{
	uint8_t 			ui8Status;

	/* reads the data in the external eeprom */
	ui8Status = srv24lc64_Read16bitData(SRV24LC_ADDRESS_ALARMBITFIELD,(SRV24LC_ADDRESS_24LC64_SIZE_16BITBUFFER*2),ppui16AlarmStatus);

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   This function reads the stored data in external eeprom memory about alarms
 *
 * @detail the external memory map is the following
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Bit field of the states alarm 			|
 * 	|	0x0001		|		2			|	Date of the "below temp" One shot alarm	|
 * 	|	0x0003		|		2			|	Time of the "below temp" One shot alarm	|
 * 	|	0x0005		|		2			|	Date of the "upper temp" One shot alarm	|
 * 	|	0x0007		|		2			|	Time of the "upper temp" One shot alarm	|
 * 	|	0x0009		|		2			|	Date of the "low bat" One shot alarm	|
 * 	|	0x000B		|		2			|	Time of the "low bat" One shot alarm	|
 * 	|	0x000D		|		1			|	CRC16 CCITT 							|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in]	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_READ - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_CRC_ALARM_EXTEEPROM - the crc is wrong
 ******************************************************************************/
uint8_t srv24LC_WriteAlarmFields ( uint16_t *pui16NewAlarmFieldToWrite )
{
	uint8_t 			ui8Status;

	/* Writes the new alarm buffer in the eeprom */
	ui8Status = srv24lc64_Write16bitData(SRV24LC_ADDRESS_ALARMBITFIELD,(SRV24LC_ADDRESS_24LC64_SIZE_16BITBUFFER*2),(uint8_t*)(pui16NewAlarmFieldToWrite) );

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   This function reads the different version stored in the memory and checks
 *   the CRC of this field is correct
 *
 * @detail the external memory map is the following in the block 7
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Firmware version						|
 * 	|	0x0001		|		1			|	Hardware version						|
 * 	|	0x0002		|		1			|	Test version							|
 * 	|	0x0003		|		1			|	...										|
 * 	|	0x0004		|		1			|	...										|
 * 	|	0x000D		|		1			|	CRC16 CCITT 							|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in]	pui16BoardVersion : pointer on the read data in the EEPROM
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_READ - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_CRC_EXTEEPROM - the crc is wrong
 ******************************************************************************/
uint8_t srv24LC_ReadBoardVersion( uint16_t  **pui16BoardVersion  )
{
uint8_t 			ui8Status;
bool 				bIsRightResidue = false;


	/* reads the data in the external EEPROM */
	ui8Status = srv24lc64_Read16bitData(SRV24LC_ADDRESS_FIRMWAREVERSION,(SRV24LC_NBBYTE_BOARDVERSION),pui16BoardVersion);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		bIsRightResidue = srvCRC_IsRightResidue (SRV24LC_NBBYTE_BOARDVERSION/2,*pui16BoardVersion);
		if(true == bIsRightResidue)
		{
			/* return success code */
			ui8Status = CROSSRFID_SUCCESSCODE ;
		}
		else
		{
			ui8Status = CROSSRFID_ERROR_EXTEEPROM_CRC;
		}
	}
	else
	{
		/* error during i2c read: returns a bad status */
	}
	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   This function reads the different version stored in the memory
 *
 * @detail the external memory map is the following in the block 7
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Firmware version						|
 * 	|	0x0001		|		1			|	Hardware version						|
 * 	|	0x0002		|		1			|	Test version							|
 * 	|	0x0003		|		1			|	...										|
 * 	|	0x0004		|		1			|	...										|
 * 	|	0x000D		|		1			|	CRC16 CCITT 							|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in]	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_WRITE - an error has occurred during the i2c tansfer.
 * @return 		CROSSRFID_ERROR_24LC64_INVALID_ADRS - invalid 24lc64 address
 ******************************************************************************/
uint8_t srv24LC_WriteBoardVersion( void )
{
uint8_t ui8Status;
uint8_t ui8NthWord=0;
uint16_t aui16DataToWrite [SRV24LC_NBBYTE_BOARDVERSION/2];


	memset (aui16DataToWrite,0x00,SRV24LC_NBBYTE_BOARDVERSION);
	aui16DataToWrite[ui8NthWord++] = COMMON_VERSION_FIRMWARE;
	aui16DataToWrite[ui8NthWord++] = COMMON_VERSION_HARDWARE;
	aui16DataToWrite[ui8NthWord++] = COMMON_VERSION_TEST;

	srvCRC_InsertCrc16CCITT (SRV24LC_NBBYTE_BOARDVERSION/2 ,aui16DataToWrite);
    /* writes the data in the external eeprom */
	ui8Status = srv24lc64_Write16bitData ( SRV24LC_ADDRESS_FIRMWAREVERSION, SRV24LC_NBBYTE_BOARDVERSION, (uint8_t*) aui16DataToWrite );

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   This function turn on or off the power supply of the external memeory
 * @details 	when the power supply is switched on a boot time should be added
 * to let the power supply to stabilize and the I2C device to boot
 * @param[in]	OnOrOff : when true the power supplied will turn on otherwise off
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srv24LC_Enable24LC64 ( bool bOnOrOff )
{
	prt24LC64_Enable24LC64 ( bOnOrOff );
	if (true == bOnOrOff)
	{
		sleep_SleepBlockBegin(sleepEM1+1);			/* because the systick is clocked by the HF clock*/
		vTaskDelay(PRT24LC64_BOOTTIMEMS);			/* block the task during 1ms*/
		sleep_SleepBlockEnd(sleepEM1+1);
	}
	else { /* do nothing*/}
}
