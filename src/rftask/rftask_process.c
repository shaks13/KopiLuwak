/*******************************************************************************
 * @file rftask_process.c
 * @brief this function set manages the data exchange between the task and the
 * device.
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "rftask_process.h"

/* Free rtos message to send to serial:

 * 1. to update the serial configuration during the init process:
		AcqSysFiles system file structure
		EnableAlarm system file structure
 * 2. to update the triggered alarms in the serial configuration
		16 bit : alarm field
		32 bit : Date
		32 bit : Time
*/
static uint16_t rftask_ui16MessageToSerial[RFTASK_NB_WORDS_MESSAGE_TOSERIAL];

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
static void 		rftask_FillInitMessageToSerial 	( void );
static uint8_t 		rftask_init 					( void );

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   This function initializes the RF task and the SPI used for the
 *   communication with the RF front end
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return CROSSRFID_SUCCESSCODE : the initialization is successful
 * @return CROSSRFID_ERROR_INITRFTASK: the initialization failed
 ******************************************************************************/
static uint8_t rftask_init ( void )
{
uint8_t ui8Status;
uint16_t ui16Word;
uint32_t ui32BeginDate,ui32BeginTime;

	srvEM4325_Init( );				/* Init the �C SPI in master mode */
	srvrtcc_GetDate(&ui32BeginDate);
	srvrtcc_GetTime(&ui32BeginTime);
	srvEM4325_InitSystemFiles(ui32BeginDate,ui32BeginTime);

	memset (rftask_ui16MessageToSerial, 0x00 , sizeof (rftask_ui16MessageToSerial));

	/* Init the EM4325 registers to perform comm buffer operations */
	ui8Status = srvEM4325_ConfigSpiSlaveExt(SRVEM4325_SIGNALING_COMM_BUFFER);

	if (CROSSRFID_SUCCESSCODE != ui8Status)
	{
		interface_DeInitSPI ();
		interface_DisableAuxPad ();
		srvEM4325_sConfiguration.eSpiConfig = SRVEM4325_SPICONFIG_DISABLED;
	}
	else /* success case */
	{
		/* ----------- Configures the BATTERY MGMT WORD 1 of the EM4325 --------------------------- */
		ui8Status = srvEM4325_ReadWord(SRVEM4325_MEMMAP_BATTERYMGMTWORD1, &ui16Word);
		if (ui16Word != srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word1)
		{
			ui8Status |= srvEM4325_WriteWord(SRVEM4325_MEMMAP_BATTERYMGMTWORD1, (uint16_t)(srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word1));
		}
		else {/* do nothing*/}

		/* ----------- Configures the BATTERY MGMT WORD 2 of the EM4325 --------------------------- */
		ui8Status = srvEM4325_ReadWord(SRVEM4325_MEMMAP_BATTERYMGMTWORD2, &ui16Word);
		if (ui16Word != srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word2)
		{
			ui8Status |= srvEM4325_WriteWord(SRVEM4325_MEMMAP_BATTERYMGMTWORD2, (uint16_t)(srvEM4325_sConfiguration.uBatteryMgmt.sBatteryMgmt.u32Word2));
		}
		else {/* do nothing*/}

		/* ----------- when the writing operations are successful --------------------------------- */
		if ( CROSSRFID_SUCCESSCODE == ui8Status )
		{
			/* Reboots the EM4325 */
			ui8Status = srvEM4325_Boot();

			if(CROSSRFID_SUCCESSCODE == ui8Status)
			{
				/* wait 2 ms */
				/* TODO EM2 delay */
				TIMER1_Init();
				TIMER1_DelayAt13Mhz(2);
				TIMER1_DeInit();
			}
			else {/* do nothing*/}

			/* Enables the irq on MISO port */
			srvEM4325_sConfiguration.eMode = SRVEM4325_SIGNALING_COMM_BUFFER;  /* change the configuration to select the MSIO IRQ*/
			//interface_EnablePinInterrupt (INTERFACE_MISO_PORT,INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
			interface_ActivatePinInterrupt(INTERFACE_MISO_PIN,true);
		}
		else
		{
			ui8Status = CROSSRFID_ERROR_INITRFTASK;
		}
	}
	return ui8Status;
}
/***************************************************************************//**
 * @brief
 *   This function fills the message for serial task to update
 *   its configuration during the init process of the firmware.
 *
 * @param[in] none
 * @param[out] none
 * @return none
 ******************************************************************************/
static void rftask_FillInitMessageToSerial ( void )
{
	uint8_t ui8NbWordsRead;

	/* fills this buffer with the 'ACQ' system files values*/
	srvEM4325_GetAcqSysFiles		(&rftask_ui16MessageToSerial[0],&ui8NbWordsRead);
	/* fills this buffer with the 'enableAlarm' system file value */
	srvEM4325_GetEnableAlarmSysFile	(&rftask_ui16MessageToSerial[ui8NbWordsRead]);
}
/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief  This function processes the initialization of RF task
 * @param[in] 	none
 * @param[out] 	none
 * @return 	CROSSRFID_SUCCESSCODE : the initialization is successful
 * @return  CROSSRFID_ERROR_INITRFTASK: the initialization failed
 * @return  CROSSRFID_MESSAGETOBEPOSTED: a message should be posted
 ******************************************************************************/
uint8_t rftask_ProcessInit ( Kernel_QueueItem_struct *pQueueItems  )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	/* if the RF task initialization failed, notifies the kernel */
	if (CROSSRFID_SUCCESSCODE != rftask_init())
	{
		pQueueItems->ui16NbByte = 0;
		pQueueItems->pData = NULL;
		/* sends a message to kernel to sleep the uc */
		pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_KERNELTASKID,KERNEL_RFFRONTENDTASKID);
		pQueueItems->ui16notification = KERNEL_MESSAGEID_STOPFIRMWARE;
	}
	else
	{
		rftask_FillInitMessageToSerial();
		pQueueItems->ui16NbByte = RFTASK_NB_WORDS_MESSAGE_TOSERIAL*2;
		pQueueItems->pData = (uint8_t*)&rftask_ui16MessageToSerial[0];
		/* sends a message to serial to continue the firmware initialization */
		pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
		pQueueItems->ui16notification = KERNEL_MESSAGEID_TASKINIT_YOURSELF;
	}

	return ui8status;
}


/***************************************************************************//**
 * @brief
 *   This function writes the status of the firmware initialization in the
 *   em4325 register file 0
 *
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void rftask_WriteSuccessInitFw ( void )
{
	/* Writes the status of the initialization */
	srvEM4325_WriteSuccessInitFwStatus();
}

/***************************************************************************//**
 * @brief  		This function reads the data of the register file
 * @details 	the EM4325 should have been previously configured
 * @param[in] 	none
 * @param[out] 	none
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
 * @return 	   CROSSRFID_MESSAGETOBEPOSTED: a message should be posted
 * @return 		CROSSRFID_READSENSORID_ERROR: error when reading the sensor id
 ******************************************************************************/
uint8_t rftask_ProcessCommBufferCommand ( Kernel_QueueItem_struct * psQueueItems )
{
uint8_t status = CROSSRFID_SUCCESSCODE;

	/* the service will process the command*/
	status = srvEM4325_ProcessCommBufferCommand (psQueueItems);

#if (1 == USE_TEMPERATURE_EM4325)
	/* Exception: the HowWarmIsIt? command is not completed but the semaphore should be released
	 * to allow serial to access the em4325 in order to get the temperature */
	if( KERNEL_MESSAGEID_GETTEMP == psQueueItems->ui16notification )
	{
		if ( NULL != xBinarySemphrEm4325 )
		{
			/* Semaphore "xBinarySemphrEm4325": sequence 3.2 */
			xSemaphoreGive( xBinarySemphrEm4325 );
		}
		else { /* Do nothing */ }
	}
#endif

	return status;
}

/***************************************************************************//**
 * @brief
 *   This function writes the response to the comm buffer "GETTEMP" command
 *
 * @details
 *
 * @param[in] pui8Data: pointer to the temperature value (from serial task)
 * @param[out] 	none
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 ******************************************************************************/
uint8_t rftask_ProcessReponseGetTemp ( uint8_t const * pui8Data )
{
	return srvEM4325_TransferMeasureTemp (pui8Data);
}

/***************************************************************************//**
 * @brief
 *   This function writes the response to the comm buffer "DATAISAVAILABLE" command
 *
 * @details
 *
 * @param[in] pui8Data: pointer to the number of data available (from serial task)
 * @param[out] none
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 ******************************************************************************/
uint8_t rftask_ProcessReponseDataIsAvailable ( const uint16_t  ui16Data )
{
	return srvEM4325_TransferIsDataAvailable (ui16Data);
}

/***************************************************************************//**
 * @brief
 *   This function writes the response to the comm buffer "DOWNLOAD" command
 *
 * @details
 *
 * @param[in] pui8Data: pointer to the samples (from serial task)
 * @param[out] none
 * @return 		CROSSRFID_SUCCESSCODE: give me sample operation successful
 * @return 		CROSSRFID_ERROR_SYSFILE_GIVEMESAMPLE_READCMDOVERFLOW: wrong index/number of sample to read, download command overflow
 ******************************************************************************/
uint8_t  rftask_ProcessReponseGiveMeSamples ( uint8_t const * pui8Data )
{
	/* saves the first address of the sample buffer */
	srvEM4325_SaveDataPointer ((uint16_t *) pui8Data);
	return srvEM4325_TransferGiveMeSamples((uint16_t *)pui8Data);
}

/***************************************************************************//**
 * @brief
 *   This function writes the response to the "write" command on system files
 *   that change the configuration in the serial task
 *
 * @details
 *
 * @param[in] 	pui8Data: pointer to the acquittal of other task
 * @param[out] 	none
 * @return CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE: serial returns a wrong status
 * @return CROSSRFID_SUCCESSCODE: serial returns a good status
 ******************************************************************************/
uint8_t  rftask_ProcessResponseSysFile ( uint8_t const * pui8Data )
{
	return srvEM4325_DeliverRespSysFileWriteCmd((bool)(*pui8Data));
}

/***************************************************************************//**
 * @brief		This function updates the system files according to new registered configuration
 *   by the other task
 * @param[in] 	eSysFileAdrs: the address of the system file
 * @param[in] 	pui16Data: the configuration
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void rftask_UpdateSysFileFromSerial ( const srvEM4325_RegisterAddress_enum eSysFileAdrs, uint16_t const * pui16Data )
{
	srvEM4325_UpdateSysFileFromOtherTask(eSysFileAdrs,pui16Data);
}

/***************************************************************************//**
 * @brief
 *   This function updates the State Alarm system file according to the alarm triggered
 *   and sends a message to the serial task with the data to write in the
 *   external memory
 *
 * @detail the message sent to serial
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	the alarm id	 						|
 * 	|	0x0001		|		2			|	Date of the One shot alarm				|
 * 	|	0x0003		|		2			|	Time of the One shot alarm				|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in] 	eAlarmId: the alarm id
 * @param[out] 	eAlarmId: the alarm id
 * @return		CROSSRFID_SUCCESSCODE  : the function is successful
 * @return		CROSSRFID_MESSAGETOBEPOSTED  : the function is successful & a
 * message should be posted
 *
 ******************************************************************************/
uint8_t rftask_ProcessSetAlarmFromSensor ( const Kernel_AlarmId_enum eAlarmId , Kernel_QueueItem_struct * psQueueItem  )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;
uint32_t ui32Date, ui32Time;

	/* Sets the bit field alarm according to the triggered alarm */
	srvEM4325_SetStateAlarmFromSensor (eAlarmId);

	/* fills the message to send to serial to write the new value in the external eeprom */
	rftask_ui16MessageToSerial[0] = (uint16_t)eAlarmId;
	srvrtcc_GetDate((uint32_t*)&rftask_ui16MessageToSerial[1]);
	srvrtcc_GetTime((uint32_t*)&rftask_ui16MessageToSerial[3]);

	ui32Date = (uint32_t)(rftask_ui16MessageToSerial[1] | (((uint32_t)(rftask_ui16MessageToSerial[2])&0x0000FFFF) << 16));
	ui32Time = (uint32_t)(rftask_ui16MessageToSerial[3] | (((uint32_t)(rftask_ui16MessageToSerial[4])&0x0000FFFF) << 16));

	/* Updates the system file according to the triggered alarm */
	srvEM4325_SetDateTimeAlarmFromSensor(eAlarmId,ui32Date,ui32Time);

	/* Sends the message */
	psQueueItem->ui16NbByte = 10;
	psQueueItem->pData = (uint8_t*)&rftask_ui16MessageToSerial[0];
	psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
	psQueueItem->ui16notification = KERNEL_MESSAGEID_SETALARMTOWRITE;

	return ui8status;
}

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
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
void rftask_WriteSystemFileAccToQueueData ( const uint8_t ui8SysFileAdrs,  const  uint8_t ui8NbByte  , uint8_t const * pui8Data )
{
	srvEM4325_WriteSystemFileAccToQueueData (ui8SysFileAdrs,  ui8NbByte  ,  pui8Data);
}
#endif

/**************************************************************************//**
 * @brief This function sends an error code to the host if an error occurred.
 * @param[in]  	ui8Error: the error code
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void rftask_HookErrorCode ( const CrossRfid_Status_enum ui8Error )
{
	srvEM4325_HookErrorCode(ui8Error);
}
#if 0 /* not used anymore*/
/***************************************************************************//**
 * @brief		This function sends a message to an other task
 * @details		the tasks should be created and the schedulter running
 *
 * @param[in] 	eReveiver: eRecipient of the mesage. it should be a member
 * of the Kernel_TaskId_enum
 * @param[in] 	eMessageId: message to be sent it should be a member of
 * the enum Kernel_FreeRTOSMessageId_enum
 * @param[in] 	ui8NbByte: number of byte of pui8Data
 * @param[in] 	pui8Data: pointer to the data should be sent
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void rftask_SendMessage ( const uint8_t eRecipient , const uint8_t eMessageId , const uint8_t  ui8NbByte ,  uint8_t const * pui8Data )
{
Kernel_QueueItem_struct sQueueItem = {	(((eRecipient << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the kernel */
										 ((KERNEL_RFFRONTENDTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,		/* the sender is the kernel */
											eMessageId,
											ui8NbByte,	/* 1 byte of data*/
											(uint8_t *)pui8Data};

	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
}
#endif
/***************************************************************************//**
 * @brief		This function performs the last actions to ended the firmware
 * 				initialization.
 * @details		It updates the areYouWarningMe and the DateTimleAlarm system files.
 * 				It sends a free RTOS message to serial to enable the alarms.
 * @param[in] 	none
 * @param[out] 	psQueueItems : queue message received and will be updated
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : the function is successful and a message
 * should be poster
 ******************************************************************************/
uint8_t rftask_FinishInitFirmware ( Kernel_QueueItem_struct * psQueueItems)
{
	uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Updates the alarm system files according to the data read in the external eeprom by serial during the init process */
	/* the first word is the alarm state*/
	rftask_UpdateSysFileFromSerial(SRVEM4325_REGISTERADDR_STATEALARM,(uint16_t*)psQueueItems->pData);
	/* the other words are the rtcc date and the rtcc time when the alarms has been triggered */
	rftask_UpdateSysFileFromSerial(SRVEM4325_REGISTERADDR_DATETIMEALARM,(&((uint16_t*)psQueueItems->pData)[1]));
	/* END OF THE FIRMWARE INITIALIZATION */
	/* writes initialization firmware successful in the register file 0 of the em4325 */
	rftask_WriteSuccessInitFw();

#if (USE_TEMPERATURE_UC == 1)
	ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	/* Sends the message to serial to start the alarm management */
	psQueueItems->ui16NbByte = 0;
	psQueueItems->pData = NULL;
	psQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_RFFRONTENDTASKID);
	psQueueItems->ui16notification = KERNEL_MESSAGEID_STARTALARMMANAGMENT;
	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sSerialQueueItem, pdFALSE);
#endif
	return ui8status;
}
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/***************************************************************************//**
 * @brief Writes the temperature average in the em4325 user memory.
 *
 * @param[in] i16Average: the temperature average to write

 ******************************************************************************/
void rftask_WriteTempAverageInUserMem (const int16_t i16Average)
{
	srvEM4325_WriteWord(SRVEM4325_MEMMAP_USER, (uint16_t)i16Average);
}

/***************************************************************************//**
 * @brief 		Writes the shelf life in the em4325 user memory.
 *
 * @param[in] 	sQueue: the kernel message
 *
 * @return		CROSSRFID_SUCCESSCODE: successful write operations
 * @return		CROSSRFID_ERROR_EM4325WRITE: write operation error of system file parameters in register files
 ******************************************************************************/
uint8_t rftask_ProcessReponseGiveMeLife ( const Kernel_QueueItem_struct sQueue )
{
	return srvEM4325_TransferGiveMeLife(sQueue);
}

#endif

