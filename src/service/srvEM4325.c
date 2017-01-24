/*******************************************************************************
 * @file srvEM4325.c
 * @brief this file defines the command set for the EM4325
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvEM4325.h"

/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
/* Comm buffer command */
static srvEM4325_CmdFirstWord_union srvEM4325_uCmdFirstWord; /* when the combuffer semaphore us fired then the IRQ is disabled and this field cannot be overwritten */
/* Comm buffer response */
static srvEM4325_RespFirstWord_union srvEM4325_uRespFirstWord;

/* System files values */
static srvEM4325_Systemfile_struct 	srvEM4325_sSysFile;

static srvEM4325_download_struct	srvEM4325_sdownload;

/*
 * @brief When the comm buffer command is a 'write' operation relating to firmware parameters not managed by the RF task,
 * it sends a message to the in question task and stores them in srvEM4325_tui16RegFileBuffer.
 * The system files will be update thanks to this buffer after receiving a 'write successful status'. */
static uint16_t srvEM4325_tui16RegFileBuffer[SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ];

/* Used as an address pointed by the "pui8Data" free RTOS notification pointer */
static uint16_t srvEM4325_ui16DataToSendInTaskQueue[7];

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
srvEM4325_configuration_struct 	srvEM4325_sConfiguration;
srvEM4325_commParams_Type 		srvEM4325_sCommParams;
srvEM4325_Systemfile_struct*	srvEM4325_psSysFile;

/*===========================================================================================================
						Private Functions declaration
===========================================================================================================*/
//static void srvEM4325_GetCommandCode 		(const uint8_t * const puFrame, uint16_t *eCommandCode);
static void 	srvEM4325_InitSpiBus 				( const bool bIsMaster  );
static bool 	srvEM4325_IsSucessCode 				( void );
static void 	srvEM4325_WriteErrorStatus 			( const uint8_t pui8RegAddress );
static uint8_t 	srvEM4325_UpdateSysFileCmd 			( void );
static uint8_t 	srvEM4325_IsAvailableSysFileAddress ( const uint8_t ui8RegisterAdress );
static uint8_t 	srvEM4325_ReadSensorId 				( uint16_t *  pui16IdSensor );
static uint8_t 	srvEM4325_ReadAlarmId 				( uint16_t * pui16IdAlarm );
static uint8_t 	srvEM4325_FillMsgToUpdateAcqSysFile ( uint8_t * purecvsender,
													  Kernel_FreeRTOSMessageId_enum * peMsg,
													  uint8_t ** ppui8Data);
static uint8_t 	srvEM4325_FillMsgToUpdateEnableAlarm( uint8_t * purecvsender,
													  Kernel_FreeRTOSMessageId_enum * peMsg,
													  uint8_t ** ppui8Data);
static uint8_t 	srvEM4325_FillMsgToResetAlarm		( uint8_t * purecvsender,
													  Kernel_FreeRTOSMessageId_enum * peMsg,
													  uint8_t ** ppui8Data);
static uint8_t 	srvEM4325_TransferWhoAreYou			( void );
static uint8_t 	srvEM4325_TransferAcqSysFile 		( void );
static uint8_t 	srvEM4325_TransferStateAlarm		( void );
static uint8_t 	srvEM4325_TransferEnableAlarm		( void );
static uint8_t 	srvEM4325_TransferDateTimeAlarm		( void );
static uint8_t 	srvEM4325_ProcessGiveMeSamples 		( Kernel_QueueItem_struct *psQueueitem  );
static uint8_t 	srvEM4325_ProcessIdxSample 			( void );
static uint8_t  srvEM4325_ProcessTime 				( Kernel_QueueItem_struct * psQueueitem );
static uint8_t  srvEM4325_ProcessDate 				( Kernel_QueueItem_struct *  psQueueitem );
static uint8_t 	srvEM4325_ProcessNbSamplesToGive 	( void );
static uint8_t 	srvEM4325_ProcessSelectMemory 		( void );
static void 	srvEM4325_EnableAuxPad 				( void );
static uint8_t 	srvEM4325_WriteRegFileWord			(const uint16_t ui16WordAdrs, uint16_t ui16RegisterFileData);
static uint8_t 	srvEM4325_ReadRegFileWord			(const uint16_t ui16WordAdrs, uint16_t *pui16RegisterFileData);
static uint8_t 	srvEM4325_GetNewSensorData			( uint16_t * pui16ReadSensorDataMSW, uint16_t * pui16ReadSensorDataLSW );
static uint8_t 	srvEM4325_Reset 					( Kernel_QueueItem_struct *  psQueueitem );
static uint8_t 	srvEM4325_ProcessIsDataAvailable 	( Kernel_QueueItem_struct *psQueueitem  );
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
static uint8_t srvEM4325_ProcessHowMuchTimeYouRun 	( Kernel_QueueItem_struct *sQueueitem  );
static uint8_t srvEM4325_ProcessHowManyTimeYouRun 	( Kernel_QueueItem_struct *sQueueitem  );
static uint8_t srvEM4325_ProcessLog 				( Kernel_QueueItem_struct *sQueueitem  );
static uint8_t srvEM4325_ProcessCalibration 		( Kernel_QueueItem_struct *sQueueitem  );
static uint8_t srvEM4325_ProcessComputeFFT 			( Kernel_QueueItem_struct *sQueueitem  );
static uint8_t srvEM4325_ProcessEnableActivity 		( Kernel_QueueItem_struct *sQueueitem  );
#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
static uint8_t srvEM4325_AskGiveMeLifeToKernel 		( Kernel_QueueItem_struct * psQueueitem );
#endif

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/

/**************************************************************************//**
 * @brief 		Write a word of the EM4325 Register file by using the SPI command
 * "SPIWriteRegisterFileWord".
 * @param[in] 	ui8WordNum: Register File Word (between 0x00 (word 1)and 0x7 (word 8 ))
 * @param[in] 	ui16WordToWrite: Word to write
 * @return 		CROSSRFID_SUCCESSCODE : the write operation is successful
 * @return 		CROSSRFID_ERROR_EM4325WRITE : the write operation is not successful
 *****************************************************************************/
static uint8_t srvEM4325_WriteRegFileWord(const uint16_t ui16WordAdrs, uint16_t ui16RegisterFileData)
{
uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325WRITE;
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_WRITEREGFILEWORD,&ui16WordAdrs,(uint16_t *)(&ui16RegisterFileData));
	/* Test the status */
	if(true == srvEM4325_IsSucessCode())
	{
		ui8EM4325Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		/* Do nothing */
	}
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief 		Read a word in the EM4325 Register file by using the Spi command
 * "SPIReadRegisterFileWord".
 * @param[in] 	ui8WordAdrs: Register File Word (between 0x00 (word 1)and 0x7 (word 8 ))
 * @param[out]	pui16RegisterFileData: pointer of the 2 bytes of the Register file
 * @return 		CROSSRFID_SUCCESSCODE : the read operation is successful
 * @return 		CROSSRFID_ERROR_EM4325READ : the read operation is not successful
 *****************************************************************************/
static uint8_t srvEM4325_ReadRegFileWord(const uint16_t ui16WordAdrs, uint16_t *pui16RegisterFileData)
{
uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325READ;
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_READREGFILEWORD,&ui16WordAdrs,NULL);
	/* Test the status */
	if(true == srvEM4325_IsSucessCode())
	{
		protocol_ReadSpiResponse(PROTOCOL_EM4325_SPI_CMD_READREGFILEWORD,pui16RegisterFileData,NULL,NULL);
		ui8EM4325Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		/* Do nothing */
	}
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief this function sends the SPI command GetSensorData (New sample)
 * @param[in]  none
 * @param[out] pui16ReadSensorDataMSW: the sensor data MSW word
 * @param[out] pui16ReadSensorDataLSW: the sensor data LSW word
 * @return CROSSRFID_SUCCESSCODE: the operation is successful
 * @return CROSSRFID_ERROR_EM4325GETNEWSAMPLE: the operation is not successful
 *****************************************************************************/
static uint8_t srvEM4325_GetNewSensorData(uint16_t * pui16ReadSensorDataMSW, uint16_t * pui16ReadSensorDataLSW )
{
uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325GETNEWSAMPLE;
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	*pui16ReadSensorDataMSW = 0;
	*pui16ReadSensorDataLSW = 0;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_GETNEWSENSORDATA,NULL,NULL);

	if(true == srvEM4325_IsSucessCode())
	{
		ui8EM4325Status = CROSSRFID_SUCCESSCODE;
		protocol_ReadSpiResponse(PROTOCOL_EM4325_SPI_CMD_GETNEWSENSORDATA,pui16ReadSensorDataMSW,pui16ReadSensorDataLSW,NULL); /* note: UTC Time Stamp data not read */
	}
	else
	{
		/* Do nothing */
	}
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief this function return true when the status of the EM4325 response is
 * a sucess
 * @note This function should be called after sending a SPI command
 *   else the returned value is not relevant.
 * @detail parsing of the status byte
 * 0b0 : disabled
 * 0b1 : enabled
 * @param[out] eDeviceState status of the command
 * 0b000 : Ready/listen
 * 0b001 : Arbitrate
 * 0b010 : Reply/TagMsg
 * 0b011 : Acknowledged
 * 0b100 : open
 * 0b101 : secured
 * 0b110 : killed
 * 0b111 : sleep
 * @param[out] bMemoryState status of the memory
 * 0b0 : Memory not busy
 * 0b1 : Memory busy
 * @param[out] eCommandState status of the command
 * 0b00 : Command executed
 * 0b01 : invalid command
 * 0b10 : command failed
 * 0b10 : command failed
 * @param[in]  pdata point on the EM4325 response
 * @param[out] bTransponderState status of the EM4325
 * 0b0 : disabled
 * 0b1 : enabled
 * @param[out] eDeviceState status of the command
 * 0b000 : Ready/listen
 * 0b001 : Arbitrate
 * 0b010 : Reply/TagMsg
 * 0b011 : Acknowledged
 * 0b100 : open
 * 0b101 : secured
 * 0b110 : killed
 * 0b111 : sleep
 * @param[out] bMemoryState status of the memory
 * 0b0 : Memory not busy
 * 0b1 : Memory busy
 *
 * @param[in]  	none
 * @param[out] 	none
 * @return 		true when the command execution is successful
 * false otherwise
 *****************************************************************************/
static bool srvEM4325_IsSucessCode (void)
{
	if (SRVEM4325_SUCCESS_CODE  == (protocol_GetEm4325Status() & (SRVEM4325_MASK_STATUS_TRANSPONDER | SRVEM4325_MASK_STATUS_RESPONSE)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**************************************************************************//**
 * @brief this function write the error code of the in the first word f the
 * register file
 * @param[in]  	pui8RegAddress: address of the system file
 * @return 		none
 *****************************************************************************/
static void srvEM4325_WriteErrorStatus (const uint8_t pui8RegAddress)
{

	/* Response: Writes the first word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = pui8RegAddress;
	srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

}

/**************************************************************************//**
 * @brief this function updates the comm buffer command
 * @note
 * @param[in]  	none
 * @param[out] 	none
 * @return CROSSRFID_SUCCESSCODE : the read operation is successful
 * @return CROSSRFID_ERROR_EM4325READ : the read operation is not successful
 *****************************************************************************/
static uint8_t srvEM4325_UpdateSysFileCmd ( void )
{
	uint16_t ui16FirstWordCmd;
	uint8_t ui8Status;

	/* Reads the first word of the register file to know the command parameters */
	ui8Status = srvEM4325_ReadRegFileWord(0x00,&ui16FirstWordCmd);

	if((CROSSRFID_SUCCESSCODE == ui8Status) && ((ui16FirstWordCmd & SRVEM4325_MASK_COMMBUFFERSEMAPHORE) == SRVEM4325_MASK_COMMBUFFERSEMAPHORE) )
	{
		/* Updates the bit fields of the command */
		srvEM4325_uCmdFirstWord.ui16Value = ui16FirstWordCmd;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_EM4325READ;
		srvEM4325_uCmdFirstWord.sBitsField.ui16Handshake = 0;
		srvEM4325_uCmdFirstWord.sBitsField.ui16NbWordsInParams = 0;
		srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite = 0;
		srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_UNEXPECTED;
	}

	return ui8Status;
}

/**************************************************************************//**
 * @brief this function returns the status of the access to the system file
 *
 * @note
 *  Before calling this function, the structure srvEM4325_uCmdFirstWord
 *  should be filled by reading the first register file of the EM4325
 *
 * @param[in] ui8RegisterAdress: the register address
 * @param[in] bRWaccess: the read or write access request
 * @param[out] none
 * @return CROSSRFID_SUCCESSCODE : the access to the register ia authorized
 * @return CROSSRFID_ERROR_SYSFILE_NOT_AVAILABLE : the access to the register is not authorized
 * @return CROSSRFID_ERROR_SYSFILE_READACESSNOTPERMITED : the read access to the register is not authorized
 * @return CROSSRFID_ERROR_SYSFILE_WRITEACESSNOTPERMITED : the write access to the register is not authorized
 *
 *****************************************************************************/
static uint8_t srvEM4325_IsAvailableSysFileAddress (const uint8_t ui8RegisterAdress )
{
	uint8_t ui8status = CROSSRFID_SUCCESSCODE;
	/* Updates the bit fields of the command */
	bool bRWaccess = srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite;

	if (ui8RegisterAdress <0x20) /* generic access file*/
	{
		if ( ((1 << ui8RegisterAdress) & SRVEM4325_SYSTEMFILE_ISAVAILABLE) == 0 ) /* check if this register file exits */
		{
			 ui8status = CROSSRFID_ERROR_SYSFILE_NOT_AVAILABLE;
		}
		else
		{
			if ( 	(bRWaccess == SRVEM4325_SYSTEMFILE_READACCESSBITFIELD) && 	/* check if the read access to this register file is permitted */
					((1 << ui8RegisterAdress) & SRVEM4325_SYSTEMFILE_READACCESS) == false )
			{
				ui8status = CROSSRFID_ERROR_SYSFILE_READACESSNOTPERMITED;
			}
			else if ( 	(bRWaccess == SRVEM4325_SYSTEMFILE_WRITEACCESSBITFIELD) && 	/* check if the write access to this register file is permitted */
						((1 << ui8RegisterAdress) & SRVEM4325_SYSTEMFILE_WRITEACCESS) == false )
			{
				ui8status = CROSSRFID_ERROR_SYSFILE_WRITEACESSNOTPERMITED;
			}
		}
	}
	else /* from the address 0x20 the system file are dedicated to specific application and may be dependent of the targeted application*/
	{
		if ( ((1 << (ui8RegisterAdress-0x20)) & SRVEM4325_SYSTEMFILE_APPISAVAILABLE) == 0 ) /* check if this register file exits */
		{
			 ui8status = CROSSRFID_ERROR_SYSFILE_NOT_AVAILABLE;
		}
		else
		{
			if ( 	(bRWaccess == SRVEM4325_SYSTEMFILE_READACCESSBITFIELD) && 	/* check if the read access to this register file is permitted */
					((1 << (ui8RegisterAdress-0x20)) & SRVEM4325_SYSTEMFILE_APPREADACCESS) == false )
			{
				ui8status = CROSSRFID_ERROR_SYSFILE_READACESSNOTPERMITED;
			}
			else if ( 	(bRWaccess == SRVEM4325_SYSTEMFILE_WRITEACCESSBITFIELD) && 	/* check if the write access to this register file is permitted */
						((1 << (ui8RegisterAdress-0x20)) & SRVEM4325_SYSTEMFILE_APPWRITEACCESS) == false )
			{
				ui8status = CROSSRFID_ERROR_SYSFILE_WRITEACESSNOTPERMITED;
			}
		}
	}

	return ui8status;
}

/**************************************************************************//**
 * @brief Reads the id of the sensor for ACQ**** system file commands
 *
 * @param[in]  	pui16IdSensor: the sensor id
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE: the function is successful
 * @return 		CROSSRFID_READSENSORID_ERROR: error when reading the sensor id
 *****************************************************************************/
static uint8_t srvEM4325_ReadSensorId ( uint16_t * pui16IdSensor )
{
	uint16_t ui16IdSensor;
	uint8_t ui8Status;
	*pui16IdSensor = KERNEL_SENSOR_ID_NBSENSOR;

	/* Reads the sensor id */
	ui8Status = srvEM4325_ReadRegFileWord(0x01,&ui16IdSensor);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		/* expected sensor */
		if (ui16IdSensor < KERNEL_SENSOR_ID_NBSENSOR)
		{
			*pui16IdSensor = ui16IdSensor;
		}
		else
		{
			ui8Status = CROSSRFID_READSENSORID_ERROR;
		}
	}
	else
	{	/* reading Sensor Id failed */	}

	return ui8Status;
}

/**************************************************************************//**
 * @brief Reads the id of the alarm for DateTimeAlarm system file commands
 *
 * @param[in]  	pui16IdAlarm: the alarm id
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE: the function is successful
 * @return 		CROSSRFID_READALARMID_ERROR: error when reading the alarm id
 *****************************************************************************/
static uint8_t srvEM4325_ReadAlarmId ( uint16_t * pui16IdAlarm )
{
	uint16_t ui16IdAlarm;
	uint8_t ui8Status;
	*pui16IdAlarm = KERNEL_ALARM_ID_LAST;

	/* Reads the sensor id */
	ui8Status = srvEM4325_ReadRegFileWord(0x01,&ui16IdAlarm);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		/* expected alarm id */
		if (ui16IdAlarm < KERNEL_ALARM_ID_LAST)
		{
			*pui16IdAlarm = ui16IdAlarm;
		}
		else
		{
			ui8Status = CROSSRFID_READALARMID_ERROR;
		}
	}
	else
	{	/* reading Alarm Id failed */	}

	return ui8Status;
}

/**************************************************************************//**
 * @brief 		Fills the message which will inform serial or kernel task to manage new acquisition setup
 * @param[out]  purecvsender: the freeRTOS sender and the receiver
 * @param[out]  peMsg: the id of the freeRTOS message
 * @param[out]  ppui8Data: the new acquisition setup
 * @return 		CROSSRFID_SUCCESSCODE: the function is successful
 * @return		CROSSRFID_ERROR_SYSFILE_ACQ_PARAMETER: one or several acq parameters in register files are wrong
 * @return		CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER: wrong parameter number
 * @return 		CROSSRFID_ERROR_EM4325READ: error when reading the sensor id
 *****************************************************************************/
static uint8_t srvEM4325_FillMsgToUpdateAcqSysFile (uint8_t * purecvsender, Kernel_FreeRTOSMessageId_enum * peMsg, uint8_t ** ppui8Data)
{
	uint8_t 	ui8status;
	uint16_t 	ui16IdSensor;

	*ppui8Data 		= 0;
	*peMsg 			= KERNEL_MESSAGEID_LAST;
	*purecvsender 	= 0;

	/* Reads the sensor id */
	ui8status = srvEM4325_ReadSensorId(&ui16IdSensor);

	/* Read success */
	if(CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* the first word of the message is the sensor id */
		srvEM4325_tui16RegFileBuffer[0] = ui16IdSensor;

		/* TODO: test the sensor id to know which is the receiver */
		*purecvsender =  KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);

		/* Reads the new values that will update the system files */
		/* 4 parameters read + sensor id: 	Param2 (1) + Param2 (0) + Param3 (1) + Param3 (0) */
		if(srvEM4325_uCmdFirstWord.sBitsField.ui16NbWordsInParams == SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQEND)
		{
			ui8status |= srvEM4325_ReadRegFileWord(0x02,&srvEM4325_tui16RegFileBuffer[KERNEL_MSG_IDX_ACQ_DATE_LSW]);
			ui8status |= srvEM4325_ReadRegFileWord(0x03,&srvEM4325_tui16RegFileBuffer[KERNEL_MSG_IDX_ACQ_DATE_MSW]);
			ui8status |= srvEM4325_ReadRegFileWord(0x04,&srvEM4325_tui16RegFileBuffer[KERNEL_MSG_IDX_ACQ_TIME_LSW]);
			ui8status |= srvEM4325_ReadRegFileWord(0x05,&srvEM4325_tui16RegFileBuffer[KERNEL_MSG_IDX_ACQ_TIME_MSW]);
		}
		/* 2 parameters read + sensor id: 	Param2 (0) + Param3 (0) */
		else if(srvEM4325_uCmdFirstWord.sBitsField.ui16NbWordsInParams == SRVEM4325_SYSFILE_NBPARAM_ACQMODE_ACQPERIOD)  /* or SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQMODE*/
		{
			ui8status |= srvEM4325_ReadRegFileWord(0x02,&srvEM4325_tui16RegFileBuffer[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD]);
			ui8status |= srvEM4325_ReadRegFileWord(0x03,&srvEM4325_tui16RegFileBuffer[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD]);
		}
		else
		{
			/* Wrong number of parameters */
			ui8status = CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER;
		}

		/* Fills the free RTOS notification according to the system file */
		switch (srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress)
		{
			case SRVEM4325_REGISTERADDR_ACQMODE:
				if((srvEM4325_tui16RegFileBuffer[1] < KERNEL_SENSOR_ACQTYPE_LAST) && (srvEM4325_tui16RegFileBuffer[2] < KERNEL_SENSOR_ACQSTATE_LAST))
				{
					*peMsg = KERNEL_MESSAGEID_ACQMODE;
				}
				else
				{
					/* wrong parameter */
					ui8status = CROSSRFID_ERROR_SYSFILE_ACQ_PARAMETER;
				}
				break;

			case SRVEM4325_REGISTERADDR_ACQPERIOD:
				if( (srvEM4325_tui16RegFileBuffer[2] < KERNEL_SENSOR_ACQUNIT_LAST) &&
					(srvEM4325_tui16RegFileBuffer[1] != 0) &&	/* the value should be different to 0*/
					(srvEM4325_tui16RegFileBuffer[1] <= 0x2000)) 	/* the value should be less than to 10000  ( arbitraty value) TODO : choose a relevant value*/
				{
					*peMsg = KERNEL_MESSAGEID_ACQPERIOD;
				}
				else
				{
					/* wrong parameter */
					ui8status = CROSSRFID_ERROR_SYSFILE_ACQ_PARAMETER;
				}
				break;

			case SRVEM4325_REGISTERADDR_ACQBEGIN:
				*peMsg = KERNEL_MESSAGEID_ACQBEGIN;
				break;

			case SRVEM4325_REGISTERADDR_ACQEND:
				*peMsg = KERNEL_MESSAGEID_ACQEND;
				break;

			case SRVEM4325_REGISTERADDR_ACQTHRESHOLD:
				*peMsg = KERNEL_MESSAGEID_ACQTHRESHOLD;
				break;

			default:
				break;
		}
	}
	else
	{ /* Reading the sensor id failed */ }

	if(CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* serial will receive these data to update the configuration */
		*ppui8Data = (uint8_t*)&srvEM4325_tui16RegFileBuffer[0];
	}


	return ui8status;
}

/**************************************************************************//**
 *  @brief  	Fills the message which will inform serial to manage the new value of
 * the enable alarm system file
 *
 * @param[out]  purecvsender: the freeRTOS sender and the receiver
 * @param[out]  peMsg: the id of the freeRTOS message
 * @param[out]  ppui8Data: the new EnableAlarm value
 *
 * @return 		CROSSRFID_SUCCESSCODE: the function is successful
 * @return 		CROSSRFID_ERROR_EM4325READ: impossible to read the enableAlarm value
 * written by the reader in register files
 *****************************************************************************/
static uint8_t srvEM4325_FillMsgToUpdateEnableAlarm (uint8_t * purecvsender, Kernel_FreeRTOSMessageId_enum * peMsg, uint8_t ** ppui8Data)
{
	uint8_t 	ui8Status;
	uint16_t 	ui16EnableAlarmValue;

	*ppui8Data 		= 0;
	*peMsg 			= KERNEL_MESSAGEID_LAST;
	*purecvsender 	= 0;

	ui8Status = srvEM4325_ReadRegFileWord(0x01,&ui16EnableAlarmValue);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		/* the first word of the message is the enableAlarm value */
		srvEM4325_tui16RegFileBuffer[0] = ui16EnableAlarmValue;

		/* serial will receive these data to update the configuration */
		*ppui8Data = (uint8_t*)&srvEM4325_tui16RegFileBuffer[0];

		*purecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);

		*peMsg = KERNEL_MESSAGEID_ENABLEALARM;
	}
	else
	{
		/* read operation failed */
	}

	return ui8Status;
}

/**************************************************************************//**
 * @brief  	Fills the message which will inform serial to manage the reset of the alarm
 *
 * @param[out]  purecvsender: the freeRTOS sender and the receiver
 * @param[out]  peMsg: the id of the freeRTOS message
 * @param[out]  ppui8Data: the new ResetAlarm value
 *
 * @return CROSSRFID_SUCCESSCODE: the function is successful
 * @return CROSSRFID_ERROR_EM4325READ: impossible to read the resetAlarm value
 * written by the reader in register files
 *****************************************************************************/
static uint8_t srvEM4325_FillMsgToResetAlarm (uint8_t * purecvsender, Kernel_FreeRTOSMessageId_enum * peMsg, uint8_t ** ppui8Data)
{
	uint8_t 	ui8Status;
	uint16_t 	ui16ResetAlarmValue;

	*ppui8Data 		= 0;
	*peMsg 			= KERNEL_MESSAGEID_LAST;
	*purecvsender 	= 0;

	ui8Status = srvEM4325_ReadRegFileWord(0x01,&ui16ResetAlarmValue);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		/* the first word of the message is ResetAlarm value */
		srvEM4325_tui16RegFileBuffer[0] = ui16ResetAlarmValue;

		/* serial will receive these data to update the configuration */
		*ppui8Data = (uint8_t*)&srvEM4325_tui16RegFileBuffer[0];

		*purecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);

		*peMsg = KERNEL_MESSAGEID_RESETALARM;
	}
	else
	{
		/* read operation failed */
	}

	return ui8Status;
}

/**************************************************************************//**
 * @brief
 * @param[in]  	none
 * @param[out]  none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_TransferWhoAreYou(void)
{
uint8_t ui8Status;

	/* Response: Writes the configuration */
	ui8Status = srvEM4325_WriteRegFileWord(SRVEM4325_PARAMETER_INDEX,srvEM4325_sSysFile.WhoAreYou);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_WHOAREYOU;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write status of the whoAreYou configuration */
	return ui8Status;
}

/**************************************************************************//**
 * @brief this function transfers the system file ACQmode or ACQPeriod or
 * ACQBegin or ACQEnd according to the address read in the command
 * @param[in]  	none
 * @param[out]  none
 * @return 		the write operation in the register file word 1
 * 				CROSSRFID_ERROR_EM4325WRITE: configuration write status in register file word
 * 				CROSSRFID_READSENSOR_ERROR: wrong sensor id
 * 				CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_TransferAcqSysFile ( void )
{
	uint8_t ui8status, ui8TempNbParam = 0;
	uint16_t ui16IdSensor;
	uint32_t ui32Param2, ui32Param3;

	/* Reads the sensor id */
	ui8status = srvEM4325_ReadSensorId(&ui16IdSensor);

	/* Read success */
	if(CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* Transfers the system files in the register files */
		switch (srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress)
		{
			case SRVEM4325_REGISTERADDR_ACQMODE:
				/* 3 register files transfered */
				ui8TempNbParam = SRVEM4325_SYSFILE_NBPARAM_ACQMODE_ACQPERIOD;
				ui32Param2 = (uint32_t)srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].Type;
				ui32Param3 = (uint32_t)srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].State;
				break;

			case SRVEM4325_REGISTERADDR_ACQPERIOD:
				/* 3 register files transfered */
				ui8TempNbParam = SRVEM4325_SYSFILE_NBPARAM_ACQMODE_ACQPERIOD;
				ui32Param2 = (uint32_t)srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].PeriodValue;
				ui32Param3 = (uint32_t)srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].PeriodUnit;
				break;

			case SRVEM4325_REGISTERADDR_ACQBEGIN:
				/* 5 register files transfered */
				ui8TempNbParam = SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQEND;
				ui32Param2 = srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].BeginDateValue;
				ui32Param3 = srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].BeginTimeValue;
				break;

			case SRVEM4325_REGISTERADDR_ACQEND:
				/* 5 register files transfered */
				ui8TempNbParam = SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQEND;
				ui32Param2 = srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].EndDateValue;
				ui32Param3 = srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].EndTimeValue;
				break;

			case SRVEM4325_REGISTERADDR_ACQTHRESHOLD:
				/* 3 register files transfered */
				ui8TempNbParam = SRVEM4325_SYSFILE_NBPARAM_ACQMODE_ACQTHRESHOLD;
				ui32Param2 = srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].LowThreshold;
				ui32Param3 = srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[ui16IdSensor].HighThreshold;
				break;

			default:
				break;
		}
		/* Response: writes the system file values in register files */
		if(ui8TempNbParam == SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQEND) /* 4 parameters written + sensor id: Param2 (1) + Param2 (0) + Param3 (1) + Param3 (0) */
		{
			ui8status |= srvEM4325_WriteRegFileWord(0x02,(uint16_t)( ui32Param2 & 0x0000FFFF));
			ui8status |= srvEM4325_WriteRegFileWord(0x03,(uint16_t)((ui32Param2 & 0xFFFF0000) >> 16));
			ui8status |= srvEM4325_WriteRegFileWord(0x04,(uint16_t)( ui32Param3 & 0x0000FFFF));
			ui8status |= srvEM4325_WriteRegFileWord(0x05,(uint16_t)((ui32Param3 & 0xFFFF0000) >> 16));
		}
		else /* 2 parameters written + sensor id: Param2 (0) + Param3 (0) */
		{
			ui8status |= srvEM4325_WriteRegFileWord(0x02,(uint16_t)( ui32Param2 & 0x0000FFFF));
			ui8status |= srvEM4325_WriteRegFileWord(0x03,(uint16_t)( ui32Param3 & 0x0000FFFF));
		}
	}
	else
	{ /* Reading the sensor id failed */ }

	/* read or write operations failed */
	if(CROSSRFID_SUCCESSCODE != ui8status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		ui8TempNbParam = 0;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = ui8TempNbParam;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress;
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write operation status of the acquisition parameters */
	return ui8status;
}

/**************************************************************************//**
 * @brief this function writes in the register file the stateAlarm bit field
 * @note
 * @param[in]  	none
 * @param[out] 	none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_TransferStateAlarm(void)
{
uint8_t ui8Status;

	/* Response: Writes the AreYouWarningMe system file */
	ui8Status = srvEM4325_WriteRegFileWord(0x01,srvEM4325_sSysFile.StateAlarm.ui16Value);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_STATEALARM;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write operation status of the AreYouWarningMe parameter */
	return ui8Status;
}

/**************************************************************************//**
 * @brief this function writes in the register file the EnableAlarm bit field
 * @note
 * @param[in]  	none
 * @param[out] 	none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_TransferEnableAlarm(void)
{
uint8_t ui8Status;

	/* Response: Writes the EnableAlarm system file */
	ui8Status = srvEM4325_WriteRegFileWord(0x01,srvEM4325_sSysFile.EnableAlarm.ui16Value);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_ENABLEALARM;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write operation status of the EnableAlarm parameter */
	return ui8Status;
}

/**************************************************************************//**
 * @brief this function writes in the register file the DateTimeAlarm system
 * file (dates and times when the alarm has been triggered according to the
 * alarm id).
 *
 * @details the register files will be written following this mapping:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Alarm Id 								|
 * 	|	0x0001		|		2			|	Date of the One shot alarm				|
 * 	|	0x0003		|		2			|	Time of the One shot alarm				|
 * @param[in]  	none
 * @param[out] 	none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_TransferDateTimeAlarm ( void )
{
	uint8_t ui8status;
	uint16_t ui16IdAlarm, ui16WordToWrite;

	/* Reads the sensor id */
	ui8status = srvEM4325_ReadAlarmId(&ui16IdAlarm);

	/* Read success and sensor id available */
	if((CROSSRFID_SUCCESSCODE == ui8status) && (ui16IdAlarm < KERNEL_ALARM_ID_LAST))
	{
		/* Response: Writes the Date when the alarm has been triggered */
		/* 1. writes the LSW */
		ui16WordToWrite = (uint16_t)(srvEM4325_psSysFile->DateTimeAlarm[ui16IdAlarm].Date & 0x0000FFFF);
		ui8status |= srvEM4325_WriteRegFileWord(0x02,ui16WordToWrite);
		/* 2. writes the MSW */
		ui16WordToWrite = (uint16_t)((srvEM4325_psSysFile->DateTimeAlarm[ui16IdAlarm].Date & 0xFFFF0000) >> 16);
		ui8status |= srvEM4325_WriteRegFileWord(0x03,ui16WordToWrite);

		/* Response: Writes the Time when the alarm has been triggered */
		/* 1. writes the LSW */
		ui16WordToWrite = (uint16_t)(srvEM4325_psSysFile->DateTimeAlarm[ui16IdAlarm].Time & 0x0000FFFF);
		ui8status |= srvEM4325_WriteRegFileWord(0x04,ui16WordToWrite);
		/* 2. writes the MSW */
		ui16WordToWrite = (uint16_t)((srvEM4325_psSysFile->DateTimeAlarm[ui16IdAlarm].Time & 0xFFFF0000) >> 16);
		ui8status |= srvEM4325_WriteRegFileWord(0x05,ui16WordToWrite);
	}
	else
	{
		/* Reading the alarm id failed */
	}

	/* Previous read or write operations failed */
	if(CROSSRFID_SUCCESSCODE != ui8status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 5;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress;
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write operation status of the DateTimeAlarm parameters */
	return ui8status;
}

/**************************************************************************//**
 * @brief this function checks is a data transfer with the RF host is on going
 * If there is no transfer on going the message is transfered to the Serial
 * task to get the data pointer
 *
 * @note
 * This system file allows the reader to define the where in the sample buffer
 * the read session will begin. After setting this sys file and the 'NbSamplesToGive'
 * system file,the reader could read the desired samples by sending a 'GiveMeSample'
 * reg file request.
 * Each 'GiveMeSample' read session can give 7 words (limitation due to the
 * em4325 register files). If 'NbSampleToGive' is under this value, the reader
 * should send several 'GiveMeSample' sessions.
 * 'NbSamplesToGive' and 'IdxSample' system files are autonomously decreased/increased
 * between each session.
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE: process GiveMeSample Ok
 * @return 		CROSSRFID_MESSAGETOBEPOSTED:
 * @return 		CROSSRFID_READSENSORID_ERROR: error when reading the sensor id
 *****************************************************************************/
static uint8_t srvEM4325_ProcessGiveMeSamples (  Kernel_QueueItem_struct *psQueueitem )
{
uint8_t status = CROSSRFID_SUCCESSCODE;
uint16_t ui16IdSensor = 0;
uint16_t ui16MeasurementId = 0;


	if (false == srvEM4325_sdownload.IsOnGoing )		/* the previous data transfer is now over request the new data pointer of the Serial task*/
	{
		/* Reads the sensor id */
		status = srvEM4325_ReadSensorId(&ui16IdSensor);
		status |= srvEM4325_ReadRegFileWord(0x02,&ui16MeasurementId);

		psQueueitem->ui16notification = KERNEL_MESSAGEID_GIVESAMPLES;
		psQueueitem->ui16NbByte = 4;
		psQueueitem->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
		/* Data to send: the sensor id */
		srvEM4325_ui16DataToSendInTaskQueue[0] = ui16IdSensor;
		srvEM4325_ui16DataToSendInTaskQueue[1] = ui16MeasurementId;
		psQueueitem->pData = (uint8_t*)(srvEM4325_ui16DataToSendInTaskQueue);
		status  = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else												/* a data transfer is on going use the data pointer save in the EM4325 service*/
	{
		status = srvEM4325_TransferGiveMeSamples( srvEM4325_sdownload.pSerialData );
		status  = CROSSRFID_SUCCESSCODE;
	}


	return status;
}

/**************************************************************************//**
 * @brief this function processes the operations on the IdxSample System file
 *
 * @note
 * This system file allows the reader to define the where in the sample buffer
 * the read session will begin. After setting this sys file and the 'NbSamplesToGive'
 * system file,the reader could read the desired samples by sending a 'GiveMeSample'
 * reg file request.
 * Each 'GiveMeSample' read session can give 7 words (limitation due to the
 * em4325 register files). If 'NbSampleToGive' is under this value, the reader
 * should send several 'GiveMeSample' sessions.
 * 'NbSamplesToGive' and 'IdxSample' system files are autonomously decreased/increased
 * between each session.
 * @param[in]  	none
 * @param[out] 	none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_ProcessIdxSample ( void )
{
uint8_t ui8RegFileAdrs = 0x01;
uint8_t ui8EM4325Status;
uint16_t ui16tempIdxSample;

	/* Read "IdxSample" System File */
	if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite )
	{
		/* Transfers the IdxSample system file */
		ui8EM4325Status = srvEM4325_WriteRegFileWord(ui8RegFileAdrs,srvEM4325_psSysFile->IdxSample);
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	/* Write "IdxSample" System File */
	else
	{
		/* Reads the IdxSample system file new value */
		ui8EM4325Status = srvEM4325_ReadRegFileWord(ui8RegFileAdrs,&ui16tempIdxSample);
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
		if(	(CROSSRFID_SUCCESSCODE == ui8EM4325Status)
				/*  &&(ui16tempIdxSample < KERNEL_COMMBUFFER_LENGTH) ||*/	)
		{
			/* Updates the system file in case of write op */
			srvEM4325_psSysFile->IdxSample = ui16tempIdxSample;
			srvEM4325_sdownload.IsOnGoing = false;				/* stop the data transfer*/
			srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		}
		else
		{
			srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		}
	}

	/* Fills the first fields of the first register file word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_IDXSAMPLE;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* First address of the register file */
	ui8RegFileAdrs = 0x00;
	/* Write the status at the first address of the register file */
	srvEM4325_WriteRegFileWord(ui8RegFileAdrs,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write operation status of the IdxSample parameter */
	return ui8EM4325Status;
}


/**************************************************************************//**
 * @brief this function processes the read or write operation
 * @note
 * @param[in]  	none
 * @param[out] 	psQueueitem : pointer of the received queue message. It may be
 * updated by this function
 * @return 		the read operations in the register file words in case of read time parameters
 * @return 		CROSSRFID_ERROR_EM4325READ: configuration write status in register file word in case of read time parameters
 * @return 		CROSSRFID_SUCCESSCODE: successful read operations in case of read time parameters
 * @return 		CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER : the number of parameeters int he right one
 *****************************************************************************/
static uint8_t srvEM4325_ProcessTime ( Kernel_QueueItem_struct *  psQueueitem )
{
uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
#if 0
Kernel_QueueItem_struct sQueueItem = {	(((KERNEL_KERNELTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the kernel */
										 ((KERNEL_RFFRONTENDTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,	/* the sender is the kernel */
											KERNEL_MESSAGEID_GETTIME,
											0, NULL};
#endif
	psQueueitem->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID);

	/* when the RF host request a read access */
	if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
	{
		psQueueitem->ui16notification = KERNEL_MESSAGEID_GETTIME;
	}
	else
	{
		if (srvEM4325_uCmdFirstWord.sBitsField.ui16NbWordsInParams == SRVEM4325_COMMBUFFER_TIME_NBWORD  )
		{
			/* read the two next words*/
			ui8Status = srvEM4325_ReadRegFileWord(0x01,&srvEM4325_tui16RegFileBuffer[1]);
			ui8Status |= srvEM4325_ReadRegFileWord(0x02,&srvEM4325_tui16RegFileBuffer[2]);
			psQueueitem->ui16NbByte = SRVEM4325_COMMBUFFER_TIME_NBWORD * SRVEM4325_NBBYTEPERWORDBYTE;		/* the date is stored on 2 words*/
			psQueueitem->pData = (uint8_t *) (&(srvEM4325_tui16RegFileBuffer[SRVEM4325_PARAMETER_INDEX]));
			psQueueitem->ui16notification = KERNEL_MESSAGEID_SETTIME;
			ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
		}
		else
		{
			ui8Status = CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER;
			//srvEM4325_WriteErrorStatus ((uint8_t)(srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress));
		}
	}

	//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);

	return ui8Status;
}

/**************************************************************************//**
 * @brief 		this function processes the read or write operation
 * @note
 * @param[in]  	none
 * @param[out] 	psQueueitem : pointer of the received queue message. It may be
 * updated by this function
 * @return 		the read operations in the register file words in case of read date parameters
 * @return 		CROSSRFID_ERROR_EM4325READ: configuration write status in register file word in case of read date parameters
 * @return 		CROSSRFID_SUCCESSCODE: successful read operations in case of read date parameters
 * @return 		CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER : the number of parameeters int he right one
 *****************************************************************************/
static uint8_t srvEM4325_ProcessDate ( Kernel_QueueItem_struct *  psQueueitem )
{
	uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
	#if 0
	Kernel_QueueItem_struct sQueueItem = {	(((KERNEL_KERNELTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the kernel */
											 ((KERNEL_RFFRONTENDTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,	/* the sender is the kernel */
												KERNEL_MESSAGEID_GETTIME,
												0, NULL};
	#endif
	psQueueitem->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID);

	/* when the RF host request a read access */
	if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
	{
		psQueueitem->ui16notification = KERNEL_MESSAGEID_GETDATE;
	}
	else
	{
		if (srvEM4325_uCmdFirstWord.sBitsField.ui16NbWordsInParams == SRVEM4325_COMMBUFFER_DATE_NBWORD  )
		{
			/* read the two next words*/
			ui8Status = srvEM4325_ReadRegFileWord(0x01,&srvEM4325_tui16RegFileBuffer[1]);
			ui8Status |= srvEM4325_ReadRegFileWord(0x02,&srvEM4325_tui16RegFileBuffer[2]);
			psQueueitem->ui16NbByte = SRVEM4325_COMMBUFFER_DATE_NBWORD * SRVEM4325_NBBYTEPERWORDBYTE;		/* the date is stored on 2 words*/
			psQueueitem->pData = (uint8_t *) (&(srvEM4325_tui16RegFileBuffer[SRVEM4325_PARAMETER_INDEX]));
			psQueueitem->ui16notification = KERNEL_MESSAGEID_SETDATE;
			ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
		}
		else
		{
			ui8Status = CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER;
			//srvEM4325_WriteErrorStatus ((uint8_t)(srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress));
		}
	}

	//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);

	return ui8Status;
}
/**************************************************************************//**
 * @brief 		this function processes the operations on the NbSamplesToGive
 * System file.
 *
 * @note
 * This system file allows the reader to define the number of samples to read
 * in the sample buffer. After setting this sys file and the 'IdxSample' sys file,
 * the reader could read the desired samples by sending a 'GiveMeSample'reg file
 * request.
 * Each 'GiveMeSample' read session can give 7 words (limitation due to the
 * em4325 register files). If 'NbSampleToGive' is under this value, the reader
 * should send several 'GiveMeSample' sessions.
 * 'NbSamplesToGive' and 'IdxSample' system files are autonomously decreased/increased
 * between each session.
 * @param[in]  	none
 * @param[out] 	none
 * @return 		the write operation in the register file word 1
 * 				CROSSRFID_ERROR_EM4325WRITE: configuration write status in register file word
 * 				CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
static uint8_t srvEM4325_ProcessNbSamplesToGive ( void )
{
uint8_t ui8RegFileAdrs = 0x01;
uint8_t ui8EM4325Status;
uint16_t ui16tempNbSamplesToGive;

	/* Read "NbSamplesToGive" System File */
	if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
	{
		/* Transfers the NbSamplesToGive system file */
		ui8EM4325Status = srvEM4325_WriteRegFileWord(ui8RegFileAdrs,srvEM4325_psSysFile->NbSamplesToGive);
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	/* Write "NbSamplesToGive" System File */
	else
	{
		/* Reads the IdxSample system file new value */
		ui8EM4325Status = srvEM4325_ReadRegFileWord(ui8RegFileAdrs,&ui16tempNbSamplesToGive);
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;

		/* if the operation is successful and the number of samples to read not exceeds the length of the sample buffer in case of write operation */
		if(		(CROSSRFID_SUCCESSCODE == ui8EM4325Status)
					/*&& ((ui16tempNbSamplesToGive) <= KERNEL_COMMBUFFER_LENGTH) ||*/)
		{
			srvEM4325_psSysFile->NbSamplesToGive = ui16tempNbSamplesToGive;
			srvEM4325_sdownload.IsOnGoing = false;				/* stop the data transfer*/
			srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		}
		else
		{
			srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		}
	}


	/* Fills the first fields of the first register file word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_NBSAMPLETOGIVE;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Write the status at the first address of the register file */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	/* return the write operation status of the NbSamplesToGive parameter */
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief 		this function processes the operations on the SelectMemory
 * System file.
 * @note
 * This system file define where the samples will be transfered during a
 * 'GiveMeSamples' operation
 *
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_ERROR_UNSPECIFIED: unknown error
 * @return 		CROSSRFID_SUCCESSCODE: the function is successful
 * @return 		CROSSRFID_ERROR_EM4325READ : the read operation is not successful
 *****************************************************************************/
static uint8_t srvEM4325_ProcessSelectMemory ( void )
{
	uint8_t ui8RegFileAdrs = 0x01;
	uint8_t ui8EM4325Status = CROSSRFID_ERROR_UNSPECIFIED;
	uint16_t ui16tempSelectMemory;

	/* Read "SelectMemory" System File */
	if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
	{
		/* Transfers the NbSamplesToGive system file */
		ui8EM4325Status = srvEM4325_WriteRegFileWord(ui8RegFileAdrs,srvEM4325_psSysFile->SelectMemory);
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	/* Write "SelectMemory" System File */
	else
	{
		/* Reads the IdxSample system file new value */
		ui8EM4325Status = srvEM4325_ReadRegFileWord(ui8RegFileAdrs,&ui16tempSelectMemory);
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	/* if the operation is successful and the chosen memory is available in case of write op */
	if(		(CROSSRFID_SUCCESSCODE == ui8EM4325Status) &&
			(	(srvEM4325_psSysFile->SelectMemory < SRVEM4325_SYSFILE_SELECTMEM_LAST) ||
				(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)	)	)
	{
		/* Updates the system file in case of write operation */
		if(SRVEM4325_SYSTEMFILE_WRITEACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
		{
			srvEM4325_psSysFile->SelectMemory = ui16tempSelectMemory;
			srvEM4325_sdownload.IsOnGoing = false;				/* stop the data transfer*/
		}
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
	}

	/* Fills the first fields of the first register file word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_SELECTMEMORY;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Write the status at the first address of the register file */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief 		this function reinitializes the firmware
 * @param[in]  	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE: the function is successful
 * @return 		CROSSRFID_ERROR_SYSFILE_RESET_PARAMETER: 2nd words contains an unexpected value
 *****************************************************************************/
static uint8_t srvEM4325_Reset ( Kernel_QueueItem_struct *  psQueueitem )
{
uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
uint16_t  ui16SecondWordCmd;

	ui8Status = srvEM4325_ReadRegFileWord(0x01,&ui16SecondWordCmd);
	if ( (ui8Status == CROSSRFID_SUCCESSCODE ) && (ui16SecondWordCmd == SRVEM4325_RESET_VALUE)) /* process to the reset when the 2nd word contains the expected value*/
	{
		psQueueitem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID);
		psQueueitem->ui16notification = KERNEL_MESSAGEID_RESET;
		ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
		//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
	}
	else
	{
		srvEM4325_WriteErrorStatus ((uint8_t)(srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress));		/* the SPI communication is now over enable again the IRQ to catch the next comm buffer semaphore*/
		//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
		interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
		ui8Status = CROSSRFID_ERROR_SYSFILE_RESET_PARAMETER;

	}

	return ui8Status;
}


/**************************************************************************//**
 * @brief 		this functions setups a USART as SPI interface
 * @param[in]	bIsMaster true when the mcu is the master, false for the slave
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static void srvEM4325_InitSpiBus ( const bool bIsMaster  )
{
	prtEM4325_InitSpiBus (bIsMaster);
}

/**************************************************************************//**
 * @brief 		this functions initializes the AUX pad in order to synchronize the
 * data transfer with the RF host.
 * @note 		this function should be called when the RF host has chosen this
 * king  of working mode thanks to the srvEM4325_psSysFile->SelectMemory
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static void srvEM4325_EnableAuxPad ( void  )
{
	if ( SRVEM4325_SYSFILE_SELECTMEM_REGFILEOPTIMIZE == srvEM4325_psSysFile->SelectMemory ) /* when the data transger goes though the register file without the handshake*/
	{
		interface_InitAuxPad();
		interface_EnablePinInterrupt (INTERFACE_AUX_PORT, INTERFACE_AUX_PIN,INTERFACE_FALLING_EDGE ); /* activate the falling edge on the AUX pad to clock the data transfer*/
	}
	else
	{
		interface_DisablePinInterrupt (INTERFACE_AUX_PORT, INTERFACE_AUX_PIN);
	}
}


#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/

/**************************************************************************//**
 * @brief 		this function read or write how much the machine is running.
 * the unit is hour.
 * @note 		developed for the activity counter (APP_ACTIVITYCOUNTER )
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static uint8_t srvEM4325_ProcessHowManyTimeYouRun ( Kernel_QueueItem_struct *sQueueitem  )
{
uint8_t status = CROSSRFID_MESSAGETOBEPOSTED;

	/* fill the message to be posted */
	sQueueitem->urecvsender 	= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);//((KERNEL_SENSORTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK);	/* the recipient is the serial task*/
	sQueueitem->ui16notification 	= KERNEL_MESSAGEID_HOWMANYTIMEYOURUN ;
	sQueueitem->ui16NbByte 		= 1 ;
	if ( SRVEM4325_SYSTEMFILE_READACCESSBITFIELD ==srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite) /* in order to avoid the cast warning*/
	{
		sQueueitem->pData 			= (uint8_t *) (SRVEM4325_SYSTEMFILE_READACCESSBITFIELD);
	}
	else
	{
		sQueueitem->pData 			= (uint8_t *) (SRVEM4325_SYSTEMFILE_WRITEACCESSBITFIELD);
	}

	return status;

}

/**************************************************************************//**
 * @brief 		this function process the calibration of the sensors. because
 * the calibration of the sensor could take a while the status of the response
 * will be directly written and a message would be send to the Serial task.
 * @note 		developed for the activity counter (APP_ACTIVITYCOUNTER )
 * @details 	the register files will be written following this mapping:
 *  from the RF host to the device
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  accelerometer					|
 * 	|				|					|	0x0001 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 *
 *  from device to the the RF host
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  accelerometer					|
 * 	|				|					|	0x0001 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0002		|		1			|	status  								|
 * 	|				|					|srvActivityRecorder_CalibrationStatus_enum|
 * 	|				|					|	0x0000 : not calibrated					|
 * 	|				|					|	0x0001 : on going						|
 * 	|				|					|	0x0002 : calibrated						|
 * 	|				|					|	0x00FF : error							|
 *  |---------------|-------------------|-------------------------------------------|
 * @param[in]  	none
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static uint8_t srvEM4325_ProcessCalibration ( Kernel_QueueItem_struct *sQueueitem  )
{
uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
uint8_t ui8CalStatus = 0x0001; /* on going*/

	ui8Status = srvEM4325_ReadRegFileWord(0x01,&srvEM4325_tui16RegFileBuffer[0]);
	srvEM4325_tui16RegFileBuffer [1]= srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite;
	/* fill the message to be posted */
	sQueueitem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);	/* the recipient is the serial task*/
	sQueueitem->ui16notification 	= KERNEL_MESSAGEID_CALIBRATION ;
	sQueueitem->ui16NbByte 			= 4 ;
	sQueueitem->pData 				= (uint8_t *) (srvEM4325_tui16RegFileBuffer);

	if ( CROSSRFID_SUCCESSCODE ==  ui8Status )
	{
		ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
	} else {/* do nothing*/}


	/* Response: Writes the status word */
	srvEM4325_WriteRegFileWord(SRVEM4325_PARAMETER_INDEX+1,ui8CalStatus);
	/* fill the response */
	srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_CALIBRATION;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);


	return ui8Status;

}

/**************************************************************************//**
 * @brief 		this function process the calibration of the sensors. because
 * the calibration of the sensor could take a while the status of the response
 * will be directly written and a message would be send to the Serial task.
 * @note 		developed for the activity counter (APP_ACTIVITYCOUNTER )
 * @details 	the register files will be written following this mapping:
 *  from the RF host to the device
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  thermometer					|
 * 	|				|					|	0x0001 :  accelerometer					|
 * 	|				|					|	0x0002 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 *
 *  from device to the the RF host
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  thermometer					|
 * 	|				|					|	0x0001 :  accelerometer					|
 * 	|				|					|	0x0002 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0002		|		1			|	status  								|
 * 	|				|					|srvActivityRecorder_CalibrationStatus_enum|
 * 	|				|					|	0x0000 : not running					|
 * 	|				|					|	0x0001 : on going						|
 * 	|				|					|	0x0002 : calibrated						|
 * 	|				|					|	0x00FF : error							|
 *  |---------------|-------------------|-------------------------------------------|
 * @param[in]  	none
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static uint8_t srvEM4325_ProcessLog ( Kernel_QueueItem_struct *sQueueitem  )
{
uint8_t ui8Status = CROSSRFID_ERROR;

	srvEM4325_tui16RegFileBuffer [0]= srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite;
	ui8Status = srvEM4325_ReadRegFileWord(0x01,&srvEM4325_tui16RegFileBuffer[1]);

	/* fill the message to be posted */
	sQueueitem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);	/* the recipient is the serial task*/
	sQueueitem->ui16notification 	= KERNEL_MESSAGEID_LOG ;
	sQueueitem->ui16NbByte 			= 2 ;
	sQueueitem->pData 				= (uint8_t *) (srvEM4325_tui16RegFileBuffer);

	if ( CROSSRFID_SUCCESSCODE ==  ui8Status )
	{
		ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
	} else {/* do nothing*/}

	return ui8Status;

}

/**************************************************************************//**
 * @brief 		this function process the request of the RF host to get the
 * status or launch the computation of a FFT.
 * the required RAM to compute the FFT is substantial and should be
 * shared between the different service. The type of the array for the
 * computation of the FFT is float_32.
 * @note 		developed for the activity counter (APP_ACTIVITYCOUNTER )
 * @details 	the register files will be written following this mapping:
 *  from the RF host to the device
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  thermometer					|
 * 	|				|					|	0x0001 :  accelerometer					|
 * 	|				|					|	0x0002 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0002		|		1			|	Measurement Id 							|
 * 	|				|					|	(Kernel_MeasurementId_enum)				|
 * 	|				|					|	0x0000 :  raw							|
 * 	|				|					|	0x0001 :  FFT 		(should not be used)|
 * 	|				|					|	0x0002 :  X axis 						|
 * 	|				|					|	0x0003 :  Y axis	 					|
 * 	|				|					|	0x0004 :  Z axis	 					|
 * 	|				|					|	0x0080 :  XYZZaxis (should not be used)	|
 *  |---------------|-------------------|-------------------------------------------|
 *
 *  from device to the the RF host
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  thermometer					|
 * 	|				|					|	0x0001 :  accelerometer					|
 * 	|				|					|	0x0002 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0002		|		1			|	status  								|
 * 	|				|					|srvActivityRecorder_CalibrationStatus_enum	|
 * 	|				|					|	0x0000 : not running					|
 * 	|				|					|	0x0001 : on going						|
 * 	|				|					|	0x0002 : computed						|
 * 	|				|					|	0x00FF : error							|
 *  |---------------|-------------------|-------------------------------------------|
 * @param[in]  	none
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static uint8_t srvEM4325_ProcessComputeFFT ( Kernel_QueueItem_struct *sQueueitem  )
{
uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED;

	srvEM4325_tui16RegFileBuffer [0]= srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite; 	/* add the read of write opeartion */
	ui8Status |= srvEM4325_ReadRegFileWord(0x01,&srvEM4325_tui16RegFileBuffer[1]);			/* add the sensot ID */
	ui8Status |= srvEM4325_ReadRegFileWord(0x02,&srvEM4325_tui16RegFileBuffer[2]);			/* add the measurement ID */

	/* fill the message to be posted */
	sQueueitem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);	/* the recipient is the serial task*/
	sQueueitem->ui16notification 	= KERNEL_MESSAGEID_COMPUTEFFT ;
	sQueueitem->ui16NbByte 			= 4 ;
	sQueueitem->pData 				= (uint8_t *) (srvEM4325_tui16RegFileBuffer);

	if ( CROSSRFID_SUCCESSCODE ==  ui8Status )
	{
		ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
	} else {/* do nothing*/}

	return ui8Status;

}


/**************************************************************************//**
 * @brief 		this function process the request of the RF host to get the
 * status or launch the computation of a FFT.
 * the required RAM to compute the FFT is substantial and should be
 * shared between the different service. The type of the array for the
 * computation of the FFT is float_32.
 * @note 		developed for the activity counter (APP_ACTIVITYCOUNTER )
 * @details 	the register files will be written following this mapping:
 *  from the RF host to the device
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  thermometer					|
 * 	|				|					|	0x0001 :  accelerometer					|
 * 	|				|					|	0x0002 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0002		|		1			|	Measurement Id 							|
 * 	|				|					|	(Kernel_MeasurementId_enum)				|
 * 	|				|					|	0x0000 :  raw							|
 * 	|				|					|	0x0001 :  FFT 		(should not be used)|
 * 	|				|					|	0x0002 :  X axis 						|
 * 	|				|					|	0x0003 :  Y axis	 					|
 * 	|				|					|	0x0004 :  Z axis	 					|
 * 	|				|					|	0x0080 :  XYZZaxis (should not be used)	|
 *  |---------------|-------------------|-------------------------------------------|
 *
 *  from device to the the RF host
 *  |---------------|-------------------|-------------------------------------------|
 *  |	address		|	number of words	|	definition								|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0001		|		1			|	sensor Id 								|
 * 	|				|					|	0x0000 :  thermometer					|
 * 	|				|					|	0x0001 :  accelerometer					|
 * 	|				|					|	0x0002 :  magnetometer					|
 *  |---------------|-------------------|-------------------------------------------|
 * 	|	0x0002		|		1			|	status  								|
 * 	|				|					|srvActivityRecorder_CalibrationStatus_enum	|
 * 	|				|					|	0x0000 : not running					|
 * 	|				|					|	0x0001 : on going						|
 * 	|				|					|	0x0002 : computed						|
 * 	|				|					|	0x00FF : error							|
 *  |---------------|-------------------|-------------------------------------------|
 * @param[in]  	none
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
static uint8_t srvEM4325_ProcessEnableActivity ( Kernel_QueueItem_struct *sQueueitem  )
{
uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED;

	srvEM4325_tui16RegFileBuffer [0]= srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite; 	/* add the read of write opeartion */
	ui8Status |= srvEM4325_ReadRegFileWord(0x01,&srvEM4325_tui16RegFileBuffer[1]);			/* add the sensot ID */
	ui8Status |= srvEM4325_ReadRegFileWord(0x02,&srvEM4325_tui16RegFileBuffer[2]);			/* add the measurement ID */

	/* fill the message to be posted */
	sQueueitem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);	/* the recipient is the serial task*/
	sQueueitem->ui16notification 	= KERNEL_MESSAGEID_COMPUTEFFT ;
	sQueueitem->ui16NbByte 			= 4 ;
	sQueueitem->pData 				= (uint8_t *) (srvEM4325_tui16RegFileBuffer);

	if ( CROSSRFID_SUCCESSCODE ==  ui8Status )
	{
		ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
	} else {/* do nothing*/}

	return ui8Status;

}


/**************************************************************************//**
 * @brief 		this function read or write how many times the machine has been
 * started
 * @note 		developed for the activity counter (APP_ACTIVITYCOUNTER )
 * @param[in] 	none
 * @param[out] 	sQueueitem
 * return 		CROSSRFID_MESSAGETOBEPOSETED : a message should be posted to
 * an another task
 *****************************************************************************/
static uint8_t srvEM4325_ProcessHowMuchTimeYouRun ( Kernel_QueueItem_struct *sQueueitem  )
{
#if 0
Kernel_QueueItem_struct sQueueItem = {	(((KERNEL_SENSORTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the kernel */
										 ((KERNEL_RFFRONTENDTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,		/* the sender is the kernel */
											KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN,
											1,	/* 1 byte of data*/
											NULL};

	sQueueItem.pData = srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite; /* copy in the data the Read or Write access required*/
	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
#else
uint8_t status = CROSSRFID_MESSAGETOBEPOSTED;

	/* fill the message to be posted */
	sQueueitem->urecvsender 	= KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
	sQueueitem->ui16notification 	= KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN ;
	sQueueitem->ui16NbByte 		= 1 ;
	if ( SRVEM4325_SYSTEMFILE_READACCESSBITFIELD ==srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite) /* in order to avoid the cast warning*/
	{
		sQueueitem->pData 			= (uint8_t *) (SRVEM4325_SYSTEMFILE_READACCESSBITFIELD);
	}
	else
	{
		sQueueitem->pData 			= (uint8_t *) (SRVEM4325_SYSTEMFILE_WRITEACCESSBITFIELD);
	}

	return status;
#endif
}
#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
/**************************************************************************//**
 * @brief 		Asks the shelf life to kernel.
 * @param[in] 	none
 * @param[out] 	psQueueitem: the message to send to kernel
 * return 		CROSSRFID_MESSAGETOBEPOSETED : a message should be posted to
 * an another task
 *****************************************************************************/
static uint8_t srvEM4325_AskGiveMeLifeToKernel ( Kernel_QueueItem_struct * psQueueitem )
{
	uint8_t status = CROSSRFID_MESSAGETOBEPOSTED;

	/* RF -> KERNEL */
	psQueueitem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID);
	/* give me life system file to get */
	psQueueitem->ui16notification 	= KERNEL_MESSAGEID_GPGIVEMELIFE;
	/* No data to send */
	psQueueitem->ui16NbByte 		= 0 ;

	return status;
}
#endif
/**************************************************************************//**
 * @brief 		this function update the queue message to send the queue message
 * to the serial task
 * @param[in] 	none
 * @param[out] 	sQueueitem
 * return 		CROSSRFID_MESSAGETOBEPOSETED : a message should be posted to
 * an another task
 * @return 		CROSSRFID_READSENSORID_ERROR: error when reading the sensor id
 *****************************************************************************/
static uint8_t srvEM4325_ProcessIsDataAvailable ( Kernel_QueueItem_struct *psQueueitem  )
{
uint16_t ui16IdSensor = 0 ;
uint16_t ui16MeasId = 0 ;
uint8_t ui8Status =CROSSRFID_MESSAGETOBEPOSTED;

	/* Reads the sensor id */
	ui8Status = srvEM4325_ReadSensorId(&ui16IdSensor);
	ui8Status |= srvEM4325_ReadRegFileWord(0x02,&ui16MeasId);
	if (CROSSRFID_SUCCESSCODE == ui8Status)
	{
		psQueueitem->ui16notification = KERNEL_MESSAGEID_ISDATAAVAILABLE;
		psQueueitem->urecvsender =  KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
		psQueueitem->ui16NbByte = 4;
		srvEM4325_ui16DataToSendInTaskQueue[0] = ui16IdSensor;
		srvEM4325_ui16DataToSendInTaskQueue[1] = ui16MeasId;
		psQueueitem->pData = (uint8_t *) srvEM4325_ui16DataToSendInTaskQueue;
		ui8Status =CROSSRFID_MESSAGETOBEPOSTED;
	}
	else { /*do nothing*/ }


	return ui8Status ;
}

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/



/**************************************************************************//**
 * @brief 		this functions initializes the �c to communicate with the
 * EM4325
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
void srvEM4325_Init ( void )
{
	srvEM4325_InitSpiBus ( true );		/* initialize the bus to communicate with the EM4325 */
	interface_InitAuxPad ();
	interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN, INTERFACE_RISING_EDGE);	/* configure the MISO pad */
	srvEM4325_InitSwModule();			/* Init the module EM4325*/
}


/**************************************************************************//**
 * @brief 		this function initializes the software global variables of the EM4325
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void srvEM4325_InitSwModule ( void )
{
	/* initiate the configuration of the EM4325*/

	/* (1) The SPI configuration of the EM4325*/
	srvEM4325_sConfiguration.eSpiConfig = SRVEM4325_SPICONFIG_SPISLAVE;

	/* (2) The I/O Control word of the EM43254 */
	srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_DEFAULT_IOCONTROLWORD;

	/* (3) The battery management words of the EM4325 */
	srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word1 = SRVEM4325_BAPCTRLWORD1_CROSSSENSITIV;
	srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word2 = SRVEM4325_BAPCTRLWORD2_AUXEVENTRFSTATE;

	/* (4) The initial mode of the EM4325 */
	srvEM4325_sConfiguration.eMode = SRVEM4325_SIGNALING_COMM_BUFFER;

	/* initialize the system files */
	srvEM4325_psSysFile = &srvEM4325_sSysFile;

	memset (srvEM4325_psSysFile, 0x00 , sizeof (srvEM4325_sSysFile));
	memset (&srvEM4325_tui16RegFileBuffer[0], 0x0000 , sizeof (srvEM4325_tui16RegFileBuffer));
	memset (&srvEM4325_sdownload, 0x00 , sizeof (srvEM4325_sdownload));

}
#if COMMON_VERSION_FIRMWARE == 0x0121
/**************************************************************************//**
 * @brief this function initializes the v1.2.1 system files
 * @param[in] ui32BeginDate: Date (ACQBEGIN System file) when the temperature acquisitions has started
 * @param[in] ui32BeginTime: Time (ACQBEGIN System file) when the temperature acquisitions has started
 * @return none
 *****************************************************************************/
void srvEM4325_InitSystemFiles (const uint32_t ui32BeginDate, const uint32_t ui32BeginTime)
{
	srvEM4325_sSysFile.WhoAreYou = (	((srvEM4325_sConfiguration.eMode 	 << SRVEM4325_SHIFT_CROSSRFID_CONFIG) 	& SRVEM4325_MASK_CROSSRFID_CONFIG) 	|
										((SRVEM4325_SYSTEMFILE_FW_MAJOR_VERS << SRVEM4325_SHIFT_FW_MAJOR_VERS) 		& SRVEM4325_MASK_FW_MAJOR_VERS) 	|
										((SRVEM4325_SYSTEMFILE_FW_MINOR_VERS << SRVEM4325_SHIFT_FW_MINOR_VERS) 		& SRVEM4325_MASK_FW_MINOR_VERS));

	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].Type 		= KERNEL_SENSOR_ACQTYPE_PERIODIC;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].State 		= KERNEL_SENSOR_ACQSTATE_BUSY;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].PeriodValue = 30;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].PeriodUnit 	= KERNEL_SENSOR_ACQUNIT_S;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].BeginDateValue = ui32BeginDate;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].BeginTimeValue = ui32BeginTime;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].EndDateValue = 0;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].EndTimeValue = 0;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].LowThreshold = SRVEM4325_SYSTEMFILE_LOWTEMPTHRESHOLD_DEF_VALUE;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].HighThreshold = SRVEM4325_SYSTEMFILE_HIGHTEMPTHRESHOLD_DEF_VALUE;
	/* enables the alarms */
	srvEM4325_sSysFile.EnableAlarm.ui16Value = SRVEM4325_SYSTEMFILE_ENABLEALARM_DEF_VALUE;
}
#elif COMMON_VERSION_FIRMWARE == 0x0122
/**************************************************************************//**
 * @brief this function initializes the v1.2.2 system files
 * @param[in] ui32BeginDate: Date (ACQBEGIN System file) when the temperature acquisitions has started
 * @param[in] ui32BeginTime: Time (ACQBEGIN System file) when the temperature acquisitions has started
 * @return none
 *****************************************************************************/
void srvEM4325_InitSystemFiles (const uint32_t ui32BeginDate, const uint32_t ui32BeginTime)
{
	srvEM4325_sSysFile.WhoAreYou = (	((srvEM4325_sConfiguration.eMode 	 << SRVEM4325_SHIFT_CROSSRFID_CONFIG) 	& SRVEM4325_MASK_CROSSRFID_CONFIG) 	|
										((SRVEM4325_SYSTEMFILE_FW_MAJOR_VERS << SRVEM4325_SHIFT_FW_MAJOR_VERS) 		& SRVEM4325_MASK_FW_MAJOR_VERS) 	|
										((SRVEM4325_SYSTEMFILE_FW_MINOR_VERS << SRVEM4325_SHIFT_FW_MINOR_VERS) 		& SRVEM4325_MASK_FW_MINOR_VERS));

	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].Type 		= KERNEL_SENSOR_ACQTYPE_PERIODIC;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].State 		= KERNEL_SENSOR_ACQSTATE_FREE;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].PeriodValue = 30;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].PeriodUnit 	= KERNEL_SENSOR_ACQUNIT_S;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].BeginDateValue = ui32BeginDate;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].BeginTimeValue = ui32BeginTime;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].EndDateValue = KERNEL_ACQ_BEGIN_END_INFINITE_VALUE;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].EndTimeValue = KERNEL_ACQ_BEGIN_END_INFINITE_VALUE;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].LowThreshold = SRVEM4325_SYSTEMFILE_LOWTEMPTHRESHOLD_DEF_VALUE;
	srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_TEMPERATURE].HighThreshold = SRVEM4325_SYSTEMFILE_HIGHTEMPTHRESHOLD_DEF_VALUE;
	/* enables the alarms */
	srvEM4325_sSysFile.EnableAlarm.ui16Value = SRVEM4325_SYSTEMFILE_ENABLEALARM_DEF_VALUE;
}
#endif
/**************************************************************************//**
 * @brief this function writes the em4325 registers to configure it as either
 * comm buffer signaling ( BAP + IRQ when the RF host writes the register files)
 * RF modem Bypass ( the EM4325 behaves as an AFE)
 * RF modem shared limited ( the em4325 is a BAP until the open state and AFE after)
 * RF modem shared full ( the em4325 is a BAP until the open state and AFE after)
 *
 * @param[in] srvEM4325_SpiSlaveExtensionMode_enum: The mode of
 * the Spi slave extension.
 * @param[out] none
 *
 * @return CROSSRFID_SUCCESSCODE : the function is successful
 * @return CROSSRFID_EM4325GENERIC_ERROR : the function is not successful
 *****************************************************************************/
uint8_t srvEM4325_ConfigSpiSlaveExt(srvEM4325_SpiSlaveExtensionMode_enum eFunctionality)
{
	uint16_t ui16TempRegisterValue;
	uint8_t ui8Status;
	bool bIsWriteRequired = false;

	/* Reads the register value of the I/O control word */
	ui8Status = srvEM4325_ReadWord(SRVEM4325_MEMMAP_IOCONTROLWORD, &ui16TempRegisterValue);

	if(CROSSRFID_SUCCESSCODE != ui8Status)
	{
		ui8Status = CROSSRFID_ERROR_EM4325GENERIC;
	}
	else
	{
		switch(eFunctionality)
		{
			case SRVEM4325_SIGNALING_COMM_BUFFER:
				/* If the register value is not in the desired configuration */
				if(srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord  != ui16TempRegisterValue)
				{
					srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_IOCTRLWORD_SIGNALINGCOMBUF;
					bIsWriteRequired =true;
				}else{/* do nothing */}
				break;

			case SRVEM4325_RF_MODEM_BYPASS:
				/* If the register value is not in the desired configuration */
				if(srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord  != ui16TempRegisterValue)
				{
					srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_IOCTRLWORD_RFMDMBYPASS;
					bIsWriteRequired =true;
				}else{/* do nothing */}
				break;

			case SRVEM4325_RF_MODEM_SHARED_LIMITED:
				/* If the register value is not in the desired configuration */
				if(srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord  != ui16TempRegisterValue)
				{
					srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_IOCTRLWORD_RFMDMSHAREDLIMITED;
					srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word2 = SRVEM4325_BAPCTRLWORD2_AUXEVENTRFSTATE;
					ui8Status = srvEM4325_WriteWord(	SRVEM4325_MEMMAP_BATTERYMGMTWORD2,
														srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word2);
					bIsWriteRequired =true;
				}else{/* do nothing */}
				break;

			case SRVEM4325_RF_MODEM_SHARED_FULL:
				/* If the register value is not in the desired configuration */
				if(srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord  != ui16TempRegisterValue)
				{
					bIsWriteRequired =true;
					srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_IOCTRLWORD_RFMDMSHAREDFULL;
				}else{/* do nothing */}
				break;

			case SRVEM4325_UID_SHARED:
				/* If the register value is not in the desired configuration */
				if(srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord  != ui16TempRegisterValue)
				{
					bIsWriteRequired =true;
					srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_IOCTRLWORD_EPCSHARED;
				}else{/* do nothing */}
			break;
			default:
				bIsWriteRequired =true;
				srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord = SRVEM4325_IOCTRLWORD_SPISLAVE;
				break;
		}

		/* write the new configuration when is is different from the current one stored in the EM4325*/
		if (true == bIsWriteRequired)
		{
			ui8Status = srvEM4325_WriteWord(	SRVEM4325_MEMMAP_IOCONTROLWORD,
												srvEM4325_sConfiguration.uIOcontrol.ui16IOcontrolWord);

		}
		else{ /* do nothing*/}
	}

	return ui8Status;
}
#if 0/* not used anymore*/
/**************************************************************************//**
 * @brief this function sends the SPI command SpiRequestStatus
 * @param[in]  none
 * @param[out] pui8EM4325Status : the EM4325 status
 * @return none
 *****************************************************************************/
void srvEM4325_RequestStatus (uint8_t * pui8EM4325Status)
{
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_REQUESTSTATUS,NULL,NULL);
	*pui8EM4325Status = protocol_GetEm4325Status();
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
}
#endif
/**************************************************************************//**
 * @brief this function sends the SPI command SpiReadWord
 * @param[in]  ui8Address : physical address to read
 * 				cf. = SRVEM4325_ADRESS_REGFILE_x
 * @param[out] pui16ReadWord :Value of the the EM4325 user memory word
 * @return CROSSRFID_SUCCESSCODE : the read operation is successful
 * @return CROSSRFID_ERROR_EM4325READ : the read operation is not successful
 *****************************************************************************/
uint8_t srvEM4325_ReadWord(const uint16_t ui16Address, uint16_t * pui16ReadWord )
{
uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325READ;
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	*pui16ReadWord = 0;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_READWORD,&ui16Address,NULL);

	if(true == srvEM4325_IsSucessCode())
	{
		ui8EM4325Status = CROSSRFID_SUCCESSCODE;
		protocol_ReadSpiResponse(PROTOCOL_EM4325_SPI_CMD_READWORD,pui16ReadWord,NULL,NULL);
	}
	else
	{
		/* Do nothing */
	}
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief this function sends the SPI command  SpiWriteWord
 * @param[in]  ui8address physical address to read
 * @param[in]  ui16WordToWrite represents the value to write at the "ui8address"
 * @param[out] none
 * @return CROSSRFID_SUCCESSCODE : the write operation is successful
 * @return CROSSRFID_ERROR_EM4325WRITE : the write operation is not successful
 *****************************************************************************/
uint8_t srvEM4325_WriteWord(const uint16_t ui16Address, uint16_t ui16WordToWrite)
{
uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325WRITE;
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_WRITEWORD,&ui16Address,(uint16_t *)(&ui16WordToWrite));

	if(true == srvEM4325_IsSucessCode())
	{
		ui8EM4325Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		/* Do nothing */
	}
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief Reboots the EM4325 by using the SPI command "SPIBoot".
 * @detail The Boot Sequence will complete in less than 2 ms and is performed
 * after the reply status has been sent to the SPI Master.
 * @param[in] none
 * @param[out] none
 * @return CROSSRFID_SUCCESSCODE : the write operation is successful
 * @return CROSSRFID_ERROR_EM4325BOOT : the boot operation is not successful
 *****************************************************************************/
uint8_t srvEM4325_Boot	(void)
{
uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325BOOT;
uint8_t ui8previousconfiguration = srvEM4325_sConfiguration.eMode;

	srvEM4325_sConfiguration.eMode = SRVEM4325_SPIMASTER;  /* change the configuration to select the MSIO IRQ*/
	protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_BOOT,NULL,NULL);
	/* Test the status */
	if(true == srvEM4325_IsSucessCode())
	{
		ui8EM4325Status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		/* Do nothing */
	}
	srvEM4325_sConfiguration.eMode = ui8previousconfiguration; /* Now the SPI com is over come back to the previous one*/
	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief this function sends the SPI command SpiSetCommParams to EM4325.
 * The �C set BAP mode sensitivity, BLF clock used by the SPI Master,
 * and some air interface protocol settings.
 * @param[in] ui16Params: Comm parameters
 * @return CROSSRFID_SUCCESSCODE : the SetComParams  operation is successful
 * @return CROSSRFID_ERROR_EM4325SETCOMMPARAMS : the SetComParams  operation
 * is not successful
 *****************************************************************************/
uint8_t srvEM4325_SetComParams (const uint16_t ui16Params)
{
	uint8_t ui8EM4325Status = CROSSRFID_ERROR_EM4325SETCOMMPARAMS;

	if(SRVEM4325_RF_MODEM_SHARED_LIMITED == srvEM4325_sConfiguration.eMode)
	{
		protocol_SendSpiCommand(PROTOCOL_EM4325_SPI_CMD_SETCOMMPARAMS,(uint16_t *)(&ui16Params),NULL);

		if(true == srvEM4325_IsSucessCode())
		{
			//srvEM4325_FillCommParams();
			ui8EM4325Status = CROSSRFID_SUCCESSCODE;
		}
		else
		{
			/* Do nothing */
		}
	}
	else
	{
		/* Do nothing */
	}

	return ui8EM4325Status;
}

/**************************************************************************//**
 * @brief this function writes in the register file the current configuration
 * of the crossrfid
 * @param[in]  	none
 * @param[out]  none
 * @return 		none
 *****************************************************************************/
void srvEM4325_TransferHowOldAreyou(void)
{
uint8_t ui8Status;

	/* Response: Writes the lifetime */
	ui8Status = srvEM4325_WriteRegFileWord(0x01,srvEM4325_sSysFile.HowOldAreYou);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_HOWOLDAEYOU;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);
}

/**************************************************************************//**
 * @brief this function measures the temperature and write the response in
 * the register file
 * @param[in]  	pui8Data: pointer to the temperature value
 * @param[out]  none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
uint8_t srvEM4325_TransferMeasureTemp (uint8_t const * pui8Data)
{
uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
int16_t i16Temp;

	i16Temp = (int16_t)(*((int16_t*)pui8Data));

	/* Response: Writes the onchip temperature */
	ui8Status = srvEM4325_WriteRegFileWord(0x01,(uint16_t)i16Temp);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_HOWWARMISIT;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8Status;
}

/**************************************************************************//**
 * @brief this function writes the board time in the system file
 * @param[in]  pui32time: time of the �c
 * @param[out]  none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
uint8_t srvEM4325_TransferTimeInSystemFile (const uint32_t pui32time)
{
uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
uint16_t ui16time =0x0000;

	memcpy(&(srvEM4325_psSysFile->time) ,&pui32time,4 );

	/* Response: Writes the time */
	ui16time = pui32time & 0xFFFF;
	ui8Status = srvEM4325_WriteRegFileWord(0x01,ui16time);

	ui16time = (pui32time >> 16) & 0xFFFF;
	ui8Status |= srvEM4325_WriteRegFileWord(0x02,ui16time);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 2;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_TIME;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8Status;
}

/**************************************************************************//**
 * @brief this function writes the board date in the system file
 * @param[in]  	pui32date: date of the �C
 * @param[out]  none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
uint8_t srvEM4325_TransferDateInSystemFile (const uint32_t pui32date)
{
uint8_t ui8Status;
uint16_t ui16date =0x0000;

	memcpy(&(srvEM4325_psSysFile->Date) ,&pui32date,4 );

	/* Response: Writes the date */
	ui16date = pui32date & 0xFFFF;
	ui8Status = srvEM4325_WriteRegFileWord(0x01,ui16date);

	ui16date = (pui32date >> 16) & 0xFFFF;
	ui8Status |= srvEM4325_WriteRegFileWord(0x02,ui16date);

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 2;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_DATE;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Response: Writes the first word */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8Status;
}

/**************************************************************************//**
 * @brief this function writes the board age in the system file
 * @param[in]  	pui32BoardAge: age of the board
 * @param[out]  none
 * @return 		the write operation in the register file word 1
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
uint8_t srvEM4325_TransferBoardAgeInSystemFile (const uint16_t pui32BoardAge)
{
uint8_t ui8RegFileAdrs = 0x01;
uint8_t ui8Status;

	/* first write the board age*/
	ui8Status = srvEM4325_WriteRegFileWord(ui8RegFileAdrs,pui32BoardAge);
	ui8RegFileAdrs -- ;
	memcpy(&(srvEM4325_psSysFile->HowOldAreYou) ,&pui32BoardAge,2 ); /* and save it*/

	/* Response: fill the first word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_HOWOLDAEYOU;
	srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;	/* the date size is two words */
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;
	/* Writes the first word */
	srvEM4325_WriteRegFileWord(ui8RegFileAdrs,srvEM4325_uRespFirstWord.ui16Value);

	return ui8Status;
}
/**************************************************************************//**
 * @brief this function gets the number of data available and write the response
 * in the register file
 *
 * @note
 * This system file allows the reader to read samples. The 'NbSamplesToGive' and
 * 'IdxSamples' system files must be set before sending the first 'GiveMeSample'
 * session.
 * Each 'GiveMeSample' read session can give 7 words (limitation due to the
 * em4325 register files). If 'NbSampleToGive' is under this value, the reader
 * should send several 'GiveMeSample' sessions.
 * 'NbSamplesToGive' and 'IdxSample' system files are autonomously decreased/increased
 * between each session.
 *
 * @param[in]  	ui16Data: the number of data available
 * @param[out]  none
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 *****************************************************************************/
uint8_t srvEM4325_TransferIsDataAvailable (const uint16_t ui16Data)
{
	uint8_t ui8RegFileAdrs = 0x01;
	uint8_t ui8EM4325Status = CROSSRFID_SUCCESSCODE;

	/* Read "IsDataAvailable" System File */
	if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
	{
		srvEM4325_psSysFile->IsDataAvailable = ui16Data;

		/* Transfers the IsDataAvailable system file */
		ui8EM4325Status = srvEM4325_WriteRegFileWord(ui8RegFileAdrs,srvEM4325_psSysFile->IsDataAvailable);

		if(CROSSRFID_SUCCESSCODE == ui8EM4325Status)
		{
			srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 1;
			srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		}
		else
		{
			srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
			srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		}
	}
	/* Write "IsDataAvailable" System File */
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
	}

	/* Fills the first fields of the first register file word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_ISDATAAVAILABLE;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Write the status at the first address of the register file */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8EM4325Status;
}



/**************************************************************************//**
 * @brief this function saves the data pointer on the measurement samples
 * sent by the serial task.
 *
 * @param[in]  	pui16Data : pointer on the samples
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void  srvEM4325_SaveDataPointer (uint16_t const * pui16Data )
{
	srvEM4325_sdownload.pSerialData = (uint16_t*) pui16Data ;
	srvEM4325_sdownload.IsOnGoing = true;
}

/**************************************************************************//**
 * @brief this function get the parameters of the DOWNLOAD command and write
 * the data in the register files according to the command
 *
 * @param[in]  	pui16Data: pointer on the samples
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE: give me sample operation successful
 * @return 		CROSSRFID_ERROR_SYSFILE_GIVEMESAMPLE_READCMDOVERFLOW: wrong index/number of sample to read, download command overflow
 *****************************************************************************/
uint8_t  srvEM4325_TransferGiveMeSamples (uint16_t const * pui16Data )
{
uint8_t uiStatus = CROSSRFID_SUCCESSCODE;
uint8_t ui8Idx;
uint8_t ui8Adrs = 0x00;
uint8_t ui8NbWordsToWriteInRegFile = 0x00; /*field "Number of words following" of the first register file*/
static uint8_t ui8BlockNumber = 0;

	/* Writes a part of the first word of the register file */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	if (SRVEM4325_SYSFILE_SELECTMEM_REGFILEOPTIMIZE != srvEM4325_psSysFile->SelectMemory)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_GIVESAMPLES;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = ((ui8BlockNumber++)<<5 || SRVEM4325_REGISTERADDR_GIVESAMPLES);
		if (ui8BlockNumber == 4)
		{
			ui8BlockNumber = 0;
		}
	}

	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

#if 0 /* not used anymore*/
	if((srvEM4325_psSysFile->NbSamplesToGive + srvEM4325_psSysFile->IdxSample > KERNEL_COMMBUFFER_LENGTH))
	{
		/* Response: Nb packets in command too large */
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
		srvEM4325_WriteRegFileWord(ui8Adrs,srvEM4325_uRespFirstWord.ui16Value);
		uiStatus = CROSSRFID_ERROR_SYSFILE_GIVEMESAMPLE_READCMDOVERFLOW;
	}
	else /* success case */
#endif
	{
		/* check where the samples must be transfered */
		switch(srvEM4325_psSysFile->SelectMemory)
		{
			/* ******************************* */
			case SRVEM4325_SYSFILE_SELECTMEM_REGFILE:
			case SRVEM4325_SYSFILE_SELECTMEM_REGFILEOPTIMIZE:
			/* ******************************* */

				/* --- Updates the field "Number of words following" of the first register file --- */
#if 1 /* Optimized algorithm taking care of the number of data available */
				if((srvEM4325_psSysFile->IsDataAvailable - srvEM4325_psSysFile->IdxSample) <= 0)
				{
					ui8NbWordsToWriteInRegFile = 0;
				}
				else
				{
					/* Nb > (IsDataAvailable - Idx): ui8NbWordsToWriteInRegFile should be limited by IsDataAvailable- Idx */
					if(srvEM4325_psSysFile->NbSamplesToGive > (srvEM4325_psSysFile->IsDataAvailable - srvEM4325_psSysFile->IdxSample))
					{
						/* 7 <= (IsDataAvailable - Idx) */
						if( SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ <= (srvEM4325_psSysFile->IsDataAvailable - srvEM4325_psSysFile->IdxSample) )
						{
							ui8NbWordsToWriteInRegFile = SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ;
						}
						/* 7 > (IsDataAvailable - Idx) */
						else
						{
							ui8NbWordsToWriteInRegFile = (srvEM4325_psSysFile->IsDataAvailable - srvEM4325_psSysFile->IdxSample);
							srvEM4325_sdownload.IsOnGoing =false ; /* now the download is over*/
						}
					}
					/* Nb <= (IsDataAvailable - Idx) */
					else
					{
						if(SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ < srvEM4325_psSysFile->NbSamplesToGive)
						{
							ui8NbWordsToWriteInRegFile = SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ;
						}
						else
						{
							ui8NbWordsToWriteInRegFile = srvEM4325_psSysFile->NbSamplesToGive;
						}
					}
				}
#else
				if(SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ < srvEM4325_psSysFile->NbSamplesToGive)
				{
					ui8NbWordsToWriteInRegFile = SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ;
				}
				else
				{
					ui8NbWordsToWriteInRegFile = srvEM4325_psSysFile->NbSamplesToGive;
				}
#endif

				/* Response: Transfers the words to read in the register file */
				for(ui8Idx = 0; ui8Idx < ui8NbWordsToWriteInRegFile ; ui8Idx ++)
				{
					/* Increments the register file address */
					ui8Adrs++;

					/* WriteRegisterFileWord command send to the EM4325 */
					srvEM4325_WriteRegFileWord(	ui8Adrs,
												pui16Data[(uint16_t)(srvEM4325_psSysFile->IdxSample + ui8Idx)]);
				}
				break;

			/* ******************************* */
			case SRVEM4325_SYSFILE_SELECTMEM_USERMEM:
			/* ******************************* */
				ui8NbWordsToWriteInRegFile = srvEM4325_psSysFile->NbSamplesToGive;

				/* Set the address of the EM4325 user buffer: first address + index */
				ui8Adrs = SRVEM4325_MEMMAP_USER + srvEM4325_psSysFile->IdxSample;

				/* Response: Transfers the words to read in the register file */
				for(ui8Idx = 0; ui8Idx < ui8NbWordsToWriteInRegFile ; ui8Idx ++)
				{
					srvEM4325_WriteWord(	ui8Adrs,
											pui16Data[(uint16_t)(srvEM4325_psSysFile->IdxSample + ui8Idx)]);
					/* Increments the user memory address */
					ui8Adrs++;
				}
				break;

			default:
			break;
		}/*switch(srvEM4325_psSysFile->SelectMemory)*/

		/* updates the "IdxSample" system file according to the number of data transfered in the register file buffer */
		srvEM4325_psSysFile->IdxSample += ui8NbWordsToWriteInRegFile;
		srvEM4325_psSysFile->NbSamplesToGive -= ui8NbWordsToWriteInRegFile;


		/* Response: Writes the first word of the register file */
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = ui8NbWordsToWriteInRegFile;
		srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);
	}

	return uiStatus;
}

/**************************************************************************//**
 * @brief This function informs that the firmware initialization
 * is successful by writing the OK status at SRVEM4325_REGISTERADDR_RESET.
 *
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
void srvEM4325_WriteSuccessInitFwStatus ( void )
{
	/* Response: Writes the first word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_RESET;
	srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);
}

/**************************************************************************//**
 * @brief This function writes a success code in the regsiter file
 * @param[in]  	ui8Register : system file address
 * @param[in]  	ui8Nbword : number of word folowing in the register file
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void srvEM4325_WriteSuccessSuccesCode ( const uint8_t ui8Register, const uint8_t ui8Nbword )
{
	/* Response: Writes the first word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = ui8Register;
	srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = ui8Nbword;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);
}

/**************************************************************************//**
 * @brief This function writes an error code in the second register file if
 * an error occurred.
 * @param[in]  	ui8Error: the error code
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void srvEM4325_HookErrorCode ( const CrossRfid_Status_enum ui8Error )
{
	if(ui8Error > CROSSRFID_ERROR)
	{
		srvEM4325_WriteErrorStatus ((uint8_t)(srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress));
		srvEM4325_WriteRegFileWord(SRVEM4325_PARAMETER_INDEX,(uint16_t)(ui8Error));
		/* enable the irq for the comm buffer semaphore */
		//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
		interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
	}else {/*do nothing*/}
}

/***************************************************************************//**
 * @brief		This function writes in the register files the data given in the
 * input parameters
 * @brief		the max number of byte to be written is 14
 * @param[in] 	ui8SysFileAdrs: the address of the system file
 * @param[in] 	ui8NbByte: number of byte to be written between 0 to 14 bytes
 * @param[in] 	pui8Data: pointer of the data to be written
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvEM4325_WriteSystemFileAccToQueueData ( const uint8_t ui8SysFileAdrs,  const  uint8_t ui8NbByte  , uint8_t const * pui8Data )
{
uint8_t ui8NthByte =0;
uint16_t ui16WordToWrite;


	if (ui8NbByte <= (SRVEM4325_NB_MAX_REGFILE_WORDS-1)*2 )
	{
		/* write the data in the register file*/
		for (ui8NthByte = 0 ; ui8NthByte< ui8NbByte; ui8NthByte+=2 )
		{
			ui16WordToWrite = (pui8Data[ui8NthByte] | (pui8Data[ui8NthByte+1]<<8));
			srvEM4325_WriteRegFileWord ( ui8NthByte/2 +1 , ui16WordToWrite );
		}
	}
	else { /* do nothing there is a mistakje about the number of byte to wrie*/}


	srvEM4325_WriteSuccessSuccesCode (ui8SysFileAdrs,ui8NbByte/2); /* the first word of the register file is the last word to write*/


}


/**************************************************************************//**
 * @brief This function writes the comm buffer response according to the
 * status sent by serial task (after having modify or not the configuration
 * of the firmware)
 *
 * @param[in] bAcknowledge : The state of the other task acknowledge to know
 * if the system files must be updated
 *
 * @return CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE: serial returns a wrong status
 * @return CROSSRFID_SUCCESSCODE: serial returns a good status
 *****************************************************************************/
uint8_t srvEM4325_DeliverRespSysFileWriteCmd (const bool bAcknowledge)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;

	/* builds the register file status */
	srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress;
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* The status returned by the other task is SUCCESS */
	if(true == bAcknowledge)
	{
		/* Updates the system files according to the last 'write' command sytem file address */
		srvEM4325_UpdateSysFileFromOtherTask(srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress,&srvEM4325_tui16RegFileBuffer[0]);

		/* builds the register file status */
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	}
	/* The status returned by the other task is FAILED */
	else
	{
		/* builds the register file status */
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
		/* */
		ui8Status = CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE;
	}

	/* writes the register file status */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   This function updates the system files according to new registered configuration
 *   by the other task
 *
 * @param[in] eSysFileAdrs: the address of the system file
 * @param[in] pui16Data: the configuration
 * @param[out] none
 *
 ******************************************************************************/
void srvEM4325_UpdateSysFileFromOtherTask ( const srvEM4325_RegisterAddress_enum eSysFileAdrs, uint16_t const * pui16Data )
{
	uint8_t ui8Idx;

	/* Update the system file according to this address */
	switch (eSysFileAdrs)
	{
		case SRVEM4325_REGISTERADDR_ACQMODE:
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].Type = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].State = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
			break;

		case SRVEM4325_REGISTERADDR_ACQPERIOD:
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].PeriodValue = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].PeriodUnit = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
			break;

		case SRVEM4325_REGISTERADDR_ACQBEGIN:
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].BeginDateValue =
					(uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] |
					(((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16));
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].BeginTimeValue =
					(uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_TIME_LSW] |
					(((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_TIME_MSW] & 0x0000FFFF) << 16));
			break;

		case SRVEM4325_REGISTERADDR_ACQEND:
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].EndDateValue =
					(uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] |
					(((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16));
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].EndTimeValue =
					(uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_TIME_LSW] |
					(((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_TIME_MSW] & 0x0000FFFF) << 16));
			break;

		case SRVEM4325_REGISTERADDR_ACQTHRESHOLD:
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].LowThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
			srvEM4325_psSysFile->srvEM4325_sAcqSysFiles[pui16Data[0]].HighThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
			break;

		case SRVEM4325_REGISTERADDR_STATEALARM:
			/*  pui16Data[0] is the new value of StateAlarm */
			srvEM4325_psSysFile->StateAlarm.ui16Value = pui16Data[0];
			break;

		case SRVEM4325_REGISTERADDR_ENABLEALARM:
			/*  pui16Data[0] is the new value of EnableAlarm */
			srvEM4325_psSysFile->EnableAlarm.ui16Value = pui16Data[0];
			break;

		case SRVEM4325_REGISTERADDR_DATETIMEALARM:

			for ( ui8Idx = 0; ui8Idx < KERNEL_ALARM_ID_LAST; ui8Idx ++ )
			{
				/* pui16Data[idx*4+x]: idx is the alarm id, x the LSW or MSW of the value (date: 0,1 / time: 2,3)*/
				srvEM4325_psSysFile->DateTimeAlarm[ui8Idx].Date = (uint32_t)(pui16Data[(ui8Idx*4)] | (((uint32_t)(pui16Data[(ui8Idx*4)+1])) << 16));
				srvEM4325_psSysFile->DateTimeAlarm[ui8Idx].Time = (uint32_t)(pui16Data[(ui8Idx*4)+2] | (((uint32_t)(pui16Data[(ui8Idx*4)+3])) << 16));
			}
			break;

		case SRVEM4325_REGISTERADDR_RESETALARM:
			/* Do Nothing */
			break;

		default:
			break;
	}
}

/***************************************************************************//**
 * @brief
 *   This function sets the State Alarm system file according to the alarm triggered
 *
 * @param[in] eAlarmId: the alarm id
 *
 ******************************************************************************/
void srvEM4325_SetStateAlarmFromSensor(const Kernel_AlarmId_enum eAlarmId)
{
	/* Update the bit field alarm system file according to the alarm id */
	switch (eAlarmId)
	{
		case KERNEL_ALARM_ID_LOW_TEMP:
			srvEM4325_psSysFile->StateAlarm.sBitField.ui16BelowTemp = 1;
			break;

		case KERNEL_ALARM_ID_HIGH_TEMP:
			srvEM4325_psSysFile->StateAlarm.sBitField.ui16UpperTemp = 1;
			break;

		case KERNEL_ALARM_ID_LOW_BATTERY:
			srvEM4325_psSysFile->StateAlarm.sBitField.ui16LowBattery = 1;
			break;

		default:
			break;
	}
}

/***************************************************************************//**
 * @brief
 *   This function updates the dateTimeAlarm system file according to the alarm triggered
 *
 * @param[in] eAlarmId: the alarm id
 * @param[in] ui32Date: the rtcc date
 * @param[in] ui32Time: the rtcc time
 ******************************************************************************/
void srvEM4325_SetDateTimeAlarmFromSensor(const Kernel_AlarmId_enum eAlarmId, const uint32_t ui32Date, const uint32_t ui32Time)
{
	srvEM4325_psSysFile->DateTimeAlarm[eAlarmId].Date = ui32Date;
	srvEM4325_psSysFile->DateTimeAlarm[eAlarmId].Time = ui32Time;
}

/***************************************************************************//**
 * @brief
 *   This function gets the value of the enable Alarm system file
 *
 * @param[out] pui16EnableAlarm: the value of the system file
 ******************************************************************************/
void srvEM4325_GetEnableAlarmSysFile ( uint16_t * pui16EnableAlarm )
{
	*pui16EnableAlarm = srvEM4325_psSysFile->EnableAlarm.ui16Value;
}

/***************************************************************************//**
 * @brief
 *   This function gets the value of the ACQ system files
 *
 * @param[out] pui16OutBuffer: the returned values of the system files
 * @param[out] pui8NbWordsRead: the number of words read
 ******************************************************************************/
void srvEM4325_GetAcqSysFiles ( uint16_t * pui16OutBuffer, uint8_t * pui8NbWordsRead )
{
	*pui8NbWordsRead 	= (uint16_t)(sizeof(srvEM4325_sSysFile.srvEM4325_sAcqSysFiles) / 2); /* byte / 2: in Word */
	/* pasts the system files into the out buffer */
	memcpy (&pui16OutBuffer[0], &srvEM4325_sSysFile.srvEM4325_sAcqSysFiles[0].Type , sizeof(srvEM4325_sSysFile.srvEM4325_sAcqSysFiles));
}

/**************************************************************************//**
 * @brief this function processes the incoming queue object
 *
 * @param[in]  pQueueItems: pointer on the message
 * @param[out] pQueueItems: pointer on the message
 * @return 	   CROSSRFID_SUCCESSCODE : the function is successful
 * @return 	   CROSSRFID_MESSAGETOBEPOSTED : a queue message
 * should be posted
 * @return 	   CROSSRFID_ERROR_EM4325READ : the read operation of the
 * register file of the EM4325 is not successful
 * @return 	   CROSSRFID_ERROR_SYSFILE_NOT_AVAILABLE : the system file doesn't exist
 * @return 	   CROSSRFID_ERROR_SYSFILE_READACESSNOTPERMITED : the read access
 * to the system file is not permitted
 * @return 	   CROSSRFID_ERROR_SYSFILE_WRITEACESSNOTPERMITED : the write access
 * to the system file is not permitted
 * @return 	CROSSRFID_READSENSORID_ERROR: error when reading the sensor id
 *****************************************************************************/
uint8_t srvEM4325_ProcessCommBufferCommand ( Kernel_QueueItem_struct * psQueueitem )
{
uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
//Kernel_FreeRTOSMessageId_enum eMessageId;


	/* Updates the information about the received command (code command, read or write ?, etc.) */
	ui8Status = srvEM4325_UpdateSysFileCmd();

	if( CROSSRFID_SUCCESSCODE == ui8Status )
	{
		/* check if the access to the system file is authorized*/
		ui8Status = srvEM4325_IsAvailableSysFileAddress(srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress);
	}
	else { /* do nothing*/}

#if (USESWO > 1)
		if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
		{
			printf("Read ");
		}
		else
		{
			printf("Write ");
		}
		printf("at %d\n",srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress);
#endif

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		/* according to the received register address */
		switch (srvEM4325_uCmdFirstWord.sBitsField.ui16RegisterAddress)
		{
			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_WHOAREYOU:	/* the SPI command received is to retrieve the configuration information */
			/* *******************************************************************************************************/
				/* the SPI communication is now over enable again the IRQ to catch the next comm buffer semaphore*/
				ui8Status = srvEM4325_TransferWhoAreYou();
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_HOWOLDAEYOU:
			/* *******************************************************************************************************/
				psQueueitem->urecvsender =  KERNEL_CREATE_RECANDSEND(KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID);
				psQueueitem->ui16notification = KERNEL_MESSAGEID_GETAGE;
				ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_HOWWARMISIT:	/* the SPI command received is to measure a temperature		 */
			/* *******************************************************************************************************/
				psQueueitem->urecvsender =  KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
				psQueueitem->ui16notification = KERNEL_MESSAGEID_GETTEMP;
				ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_TIME:	/* the SPI command received is to get the time of the board 		 */
			case SRVEM4325_REGISTERADDR_TIME2:	/* the SPI command received is to get the time of the board 		 */
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_ProcessTime ( psQueueitem );
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_DATE:	/* the SPI command received is to get the date of the board 		 */
			case SRVEM4325_REGISTERADDR_DATE2:	/* the SPI command received is to get the date of the board 		 */
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_ProcessDate ( psQueueitem );
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_ACQMODE: 	/* Configure or get the acquisition mode for one sensor			 */
			case SRVEM4325_REGISTERADDR_ACQPERIOD:	/* Configure or get the acquisition period for one sensor 		 */
			case SRVEM4325_REGISTERADDR_ACQBEGIN: 	/* Configure or get the beginning of acquisitions for one sensor */
			case SRVEM4325_REGISTERADDR_ACQEND:		/* Configure or get the end of acquisitions for one sensor 	 	 */
			case SRVEM4325_REGISTERADDR_ACQTHRESHOLD:/* Configure or get the alarm thresholds for one sensor	 	 */
			/* *******************************************************************************************************/
				/* "Read" request */
				if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
				{
					/* Fills the register files with the system file values */
					ui8Status = srvEM4325_TransferAcqSysFile();
				}
				else
				{
					/* Fills the message which will inform serial or kernel task to manage new acquisition setup */
					ui8Status = srvEM4325_FillMsgToUpdateAcqSysFile(&(psQueueitem->urecvsender),&(psQueueitem->ui16notification),&(psQueueitem->pData));

					if(CROSSRFID_SUCCESSCODE == ui8Status)
					{
						/* sends the message */
						ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
					}
					else
					{
						/* writes unsuccessful status in register file */
						srvEM4325_DeliverRespSysFileWriteCmd(false); /* TODO: DO NOT return the status, we force the status to false */
					}
				}
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_STATEALARM:	/* the command is to transfer the state alarm system file	 	 */
			/* *******************************************************************************************************/
				/* Fills the register files with the system file values */
				ui8Status = srvEM4325_TransferStateAlarm();
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_ENABLEALARM:	/* the command is to transfer the enable alarm system file	 */
			/* *******************************************************************************************************/
				/* "Read" request */
				if(SRVEM4325_SYSTEMFILE_READACCESSBITFIELD == srvEM4325_uCmdFirstWord.sBitsField.ui16ReadOrWrite)
				{
					ui8Status = srvEM4325_TransferEnableAlarm();
				}
				else
				{
					/* Fills the message which will inform serial or kernel task to manage new acquisition setup */
					ui8Status = srvEM4325_FillMsgToUpdateEnableAlarm(&(psQueueitem->urecvsender),&(psQueueitem->ui16notification),&(psQueueitem->pData));

					if(CROSSRFID_SUCCESSCODE == ui8Status)
					{
						/* sends the message */
						ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
					}
					else
					{
						/* writes unsuccessful status in register file */
						srvEM4325_DeliverRespSysFileWriteCmd( false );
					}
				}
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_RESETALARM:	/* the command is to reset the AreYouWarningMe and DateAndTime */
			/* *******************************************************************************************************/
				/* Fills the message which will inform serial or kernel task to manage new acquisition setup */
				ui8Status = srvEM4325_FillMsgToResetAlarm(&(psQueueitem->urecvsender),&(psQueueitem->ui16notification),&(psQueueitem->pData));

				if(CROSSRFID_SUCCESSCODE == ui8Status)
				{
					/* sends the message */
					ui8Status = CROSSRFID_MESSAGETOBEPOSTED;
				}
				else
				{
					/* writes unsuccessful status in register file */
					srvEM4325_DeliverRespSysFileWriteCmd(false);
				}
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_DATETIMEALARM:	/* to inform the RF host when the alarm has been triggered 	 */
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_TransferDateTimeAlarm();
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_ISDATAAVAILABLE:	/* to inform the RF host if some data are available		 */
			/* *******************************************************************************************************/
				/* fills the message to send to serial with the "sensor id" */
				ui8Status = srvEM4325_ProcessIsDataAvailable (psQueueitem);
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_GIVESAMPLES:
			/* *******************************************************************************************************/
				/* fills the message to send to serial with the "sensor id" if it is the first givemesample session */
				ui8Status  = srvEM4325_ProcessGiveMeSamples (psQueueitem );
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_IDXSAMPLE:
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_ProcessIdxSample();
			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_NBSAMPLETOGIVE:
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_ProcessNbSamplesToGive();
				break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_SELECTMEMORY:
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_ProcessSelectMemory();

				if ( CROSSRFID_SUCCESSCODE == ui8Status)  /* when the data transfer goes though the register file without the handshake*/
				{
					srvEM4325_EnableAuxPad();
				}else {/*do nothing*/}

			break;

			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_RESET:
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_Reset( psQueueitem);
			break;

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
			/* specific for the DNCS project */
			case SRVEM4325_REGISTERADDR_HOWMUCHTIMEYOURUN:
				ui8Status = srvEM4325_ProcessHowMuchTimeYouRun ( psQueueitem );

			break;

			case SRVEM4325_REGISTERADDR_HOWMANYTIMEYOURUN:
				ui8Status =  srvEM4325_ProcessHowManyTimeYouRun (psQueueitem);
			break;

			case SRVEM4325_REGISTERADDR_CALIBRATION:
				ui8Status =  srvEM4325_ProcessCalibration (psQueueitem);
			break;
			case SRVEM4325_REGISTERADDR_LOG:
				ui8Status =  srvEM4325_ProcessLog (psQueueitem);
			break;
			case SRVEM4325_REGISTERADDR_COMPUTEFFT :
				ui8Status =  srvEM4325_ProcessComputeFFT (psQueueitem);
			break;
			case SRVEM4325_REGISTERADDR_ENABLEACTIVITY :
				ui8Status =  srvEM4325_ProcessEnableActivity (psQueueitem);
			break;
#elif  (APP_CHOSEN_FLAG == APP_GLUEPOT)
			/* *******************************************************************************************************/
			case SRVEM4325_REGISTERADDR_GPGIVEMELIFE: /* The rf host asks the value of the shelf life 				 */
			/* *******************************************************************************************************/
				ui8Status = srvEM4325_AskGiveMeLifeToKernel(psQueueitem);
				break;
#endif
			default :
				/* should not happen*/
			break;
		}
	}

	srvEM4325_HookErrorCode(ui8Status);

	return ui8Status;
}

/**************************************************************************//**
 * @brief 		this functions updates the UID of the devices through the
 * register file
 * @param[in] 	ui8Nbbyte : nb byte of the UID to write
 * @param[in] 	pui8Uid : pointer on the UID
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
void srvEM4325_WriteUid( const uint8_t ui8Nbbyte , const uint8_t * pui8Uid  )
{
uint8_t ui8NbWord = 0;
uint16_t ui16Wordtowrite;

	for (ui8NbWord = 0 ;ui8NbWord < (ui8Nbbyte/2); ui8NbWord++  )
	{
		ui16Wordtowrite = (pui8Uid [ui8NbWord*2]<<8) | pui8Uid [ui8NbWord*2+1];
		srvEM4325_WriteRegFileWord (ui8NbWord  , ui16Wordtowrite );
	}

}

/**************************************************************************//**
 * @brief 		this functions measures and reads the temperature in the EM4325
 * @note 		should not be used in an IRQ context
 * @param[in] 	none
 * @param[out] 	pi16Temperature : the temperature multiplied by 1000
 * in degree Celcius returned by the EM4325
 * return 		CROSSRFID_READSENSOR_ERROR : the temperature is invalid
 * return 		CROSSRFID_SUCCESSCODE : the temperature is valid
 *****************************************************************************/
uint8_t srvEM4325_MeasureTemperature( int16_t  * pi16Temperature  )
{
	uint16_t ui16ReadSensorDataMSW, ui16ReadSensorDataLSW;
	uint8_t ui8status;

	/* SensorDataMSW (em4325 register) contains the temperature but not translated */
	ui8status = srvEM4325_GetNewSensorData(&ui16ReadSensorDataMSW,&ui16ReadSensorDataLSW);
	//uint16_t ui16Word = PROTOCOL_EM4325_MEASURETEMERATURE;

	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		if ( PROTOCOL_EM4325_TEMERATURE_INVALID == (ui16ReadSensorDataMSW & PROTOCOL_EM4325_MASK_TEMERATURE) )
		{
			(*pi16Temperature ) = PROTOCOL_EM4325_TEMERATURE_0_KELVIN;
			ui8status = CROSSRFID_READSENSOR_ERROR;
		}
		else
		{
			/* Tests the sign of the temperature */
			if(1 == ((ui16ReadSensorDataMSW & PROTOCOL_EM4325_MASK_NEG_TEMERATURE) >> PROTOCOL_EM4325_SHIFT_NEG_TEMERATURE))
			{
				/* the format of the temperature is (2's complement MSB  = 4�C / 1 LSB = 0.25�C) */
				(*pi16Temperature) =  (int16_t)(((~ui16ReadSensorDataMSW) & 0x00FF) >> 2);
				(*pi16Temperature) *=  PROTOCOL_EM4325_COEF_NEG_TEMERATURE;
			}
			else
			{
				/* the format of the temperature is (1 MSB  = 4�C / 1 LSB = 0.25�C) */
				(*pi16Temperature) =  (int16_t)(((ui16ReadSensorDataMSW & PROTOCOL_EM4325_MASK_MSB_TEMERATURE) >> PROTOCOL_EM4325_SHIFT_MSB_TEMERATURE)
										* PROTOCOL_EM4325_COEF_MSB_TEMERATURE);
				(*pi16Temperature) += (int16_t)(((ui16ReadSensorDataMSW & PROTOCOL_EM4325_MASK_LSB_TEMERATURE) >> PROTOCOL_EM4325_SHIFT_LSB_TEMERATURE)
										* PROTOCOL_EM4325_COEF_LSB_TEMERATURE);
			}
			ui8status = CROSSRFID_SUCCESSCODE;
		}
	}
	else
	{
		(*pi16Temperature ) = PROTOCOL_EM4325_TEMERATURE_0_KELVIN;
		ui8status = CROSSRFID_ERROR_EM4325GETNEWSAMPLE;
	}


	return ui8status  ;
}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/**************************************************************************//**
 * @brief		This function write the shelf life in the register files.
 *
 * @param[in]  	sQueue: the kernel message
 * @param[out] 	none
 *
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 *****************************************************************************/
uint8_t srvEM4325_TransferGiveMeLife ( const Kernel_QueueItem_struct sQueue )
{
	uint8_t ui8EM4325Status = CROSSRFID_SUCCESSCODE;

	/* if kernel sends a  message */
	if((sQueue.ui16NbByte > 0) && (sQueue.pData != NULL))
	{
		/* writes the parameters in the register file */
		/* the shelf life */
		ui8EM4325Status = srvEM4325_WriteRegFileWord((SRVEM4325_STATUS_INDEX+1),  ((uint16_t*)sQueue.pData)[0]);
		ui8EM4325Status |= srvEM4325_WriteRegFileWord((SRVEM4325_STATUS_INDEX+2), ((uint16_t*)sQueue.pData)[1]);
		ui8EM4325Status |= srvEM4325_WriteRegFileWord((SRVEM4325_STATUS_INDEX+3), ((uint16_t*)sQueue.pData)[2]);
		ui8EM4325Status |= srvEM4325_WriteRegFileWord((SRVEM4325_STATUS_INDEX+4), ((uint16_t*)sQueue.pData)[3]);
		/* the state of the shelf life calculation */
		ui8EM4325Status |= srvEM4325_WriteRegFileWord((SRVEM4325_STATUS_INDEX+5), ((uint16_t*)sQueue.pData)[4]);
	}
	else
	{
		ui8EM4325Status = CROSSRFID_ERROR_EM4325WRITE;
	}

	/* the parameter has been written */
	if(CROSSRFID_SUCCESSCODE == ui8EM4325Status)
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 5;
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSOK;
	}
	else
	{
		srvEM4325_uRespFirstWord.sBitsField.ui16NbWordsInParams = 0;
		srvEM4325_uRespFirstWord.sBitsField.ui16Status = SRVEM4325_REGFILE_STATUSKO;
	}

	/* Fills the first fields of the first register file word */
	srvEM4325_uRespFirstWord.sBitsField.ui16Handshake = SRVEM4325_HANDSHAKE_CROSSTORFHOST;
	srvEM4325_uRespFirstWord.sBitsField.ui16RegisterAddress = SRVEM4325_REGISTERADDR_GPGIVEMELIFE;
	srvEM4325_uRespFirstWord.sBitsField.ui16Rfu = 0;

	/* Write the status at the first address of the register file */
	srvEM4325_WriteRegFileWord(SRVEM4325_STATUS_INDEX,srvEM4325_uRespFirstWord.ui16Value);

	return ui8EM4325Status;
}
#elif (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
/**************************************************************************//**
 * @brief this function initializes the v1.2.2 system files
 * @param[in] ui16RegisterAddress: Date (ACQBEGIN System file) when the temperature acquisitions has started
 * @param[in] ui16SensorId: id of the sensor to be updated
 * @param[in] ui16NewField: new field to be written
 * @return none
 *****************************************************************************/
void srvEM4325_UpdateSystemFiles (const srvEM4325_RegisterAddress_enum ui16RegisterAddress , const Kernel_Sensor_Id_enum ui16SensorId, const uint16_t ui16NewField)
{
	switch (ui16RegisterAddress)
	{
		case SRVEM4325_REGISTERADDR_CALIBRATION:
			if (ui16SensorId<KERNEL_SENSOR_ID_NBSENSOR)
			{
				srvEM4325_sSysFile.ui16Calibration[ui16SensorId]= ui16NewField;
			}else {/*do nothing*/}
		break;
		case SRVEM4325_REGISTERADDR_LOG:
			if (ui16SensorId<KERNEL_SENSOR_ID_NBSENSOR)
			{
				srvEM4325_sSysFile.ui16Log[ui16SensorId]= ui16NewField;
			}else {/*do nothing*/}
		break;
		default :
			/*nothing to do for the others registers*/
		break;
	}

}
#endif

