/*******************************************************************************
 * @file common_statuscode.h
 * @brief contains the status code
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef COMMON_STATUSCODE_H
#define COMMON_STATUSCODE_H


#define CROSSRFID_MASK_ERROR_DECODING_EPC_CMD 	(0xF0)
#define CROSSRFID_BASE_ERROR_DECODING_EPC_CMD 	(0x70)

/** @eneum CrossRfid_Status_enum
 *  @brief This enum contains the status code sent by the functions
 */
typedef enum
{
	CROSSRFID_SUCCESSCODE = 0x00,			/*!< Success */
	CROSSRFID_MESSAGETOBEPOSTED ,			/*!< Success case but a message to an another task should be posted */

	CROSSRFID_ERROR , 						/* -------------- error codes -------------- */

	CROSSRFID_TASKNOTCREATED=0x10, 			/*!< the task is not created */
	CROSSRFID_QUEUEFULL, 					/*!< the queue is full */
	CROSSRFID_ERROR_UNSPECIFIED,			/*!< unspecified error code  */
	CROSSRFID_ERROR_WRONGPARAMETER,			/*!< one parameter is erroneous  */
	CROSSRFID_ERROR_SERIAL_ACKNOWLEDGE,
	CROSSRFID_ERROR_SEMAPHR_EM4325_TIMOUT,	/*!< unexpected time out to obtain a em4325 semaphore */

	/* these error codes are specific to the EM4325 protocol */
	CROSSRFID_ERROR_EM4325GENERIC=0x20,
	CROSSRFID_ERROR_EM4325WRITE,
	CROSSRFID_ERROR_EM4325READ,
	CROSSRFID_ERROR_EM4325GETCOMMPARAMS,
	CROSSRFID_ERROR_EM4325SETCOMMPARAMS,
	CROSSRFID_ERROR_EM4325SETHANDLE,
	CROSSRFID_ERROR_EM4325REQNEWHANDLE,
	CROSSRFID_ERROR_EM4325BOOT,
	CROSSRFID_ERROR_EM4325GETNEWSAMPLE,

	/* these error codes are specific to the I2C interface */
	CROSSRFID_ERROR_I2CUNKNOWN=0x30,
	CROSSRFID_ERROR_I2COVERFLOW,
	CROSSRFID_ERROR_I2C_READ,
	CROSSRFID_ERROR_I2C_WRITE,

	/* these error codes are specific to the Sensor */
	CROSSRFID_INITSENSOR_ERROR=0x40,
	CROSSRFID_INITSENSOR_ONGOING,
	CROSSRFID_READSENSOR_ERROR,
	CROSSRFID_READALARMID_ERROR,
	CROSSRFID_READSENSORID_ERROR,

	/* these error codes are specific to the external eeprom */
	CROSSRFID_ERROR_EXTEEPROM_CRC=0x50,
	CROSSRFID_ERROR_EXTEEPROM_INVALID_ADRS,

	/* these error codes are specific to the RF task */
	CROSSRFID_ERROR_INITRFTASK= 0x60,
	CROSSRFID_ERROR_SYSFILE_NOT_AVAILABLE,  				/*!< system file not available */
	CROSSRFID_ERROR_SYSFILE_ACQ_NB_PARAMETER,				/*!< wrong number of acq parameters according to the acq system file */
	CROSSRFID_ERROR_SYSFILE_ACQ_PARAMETER,					/*!< wrong acq parameters according to the acq system file */
	CROSSRFID_ERROR_SYSFILE_READACESSNOTPERMITED,			/*!< the read access is not authorized */
	CROSSRFID_ERROR_SYSFILE_WRITEACESSNOTPERMITED,			/*!< the write access is not authorized  */
	CROSSRFID_ERROR_SYSFILE_GIVEMESAMPLE_READCMDOVERFLOW,	/*!< the download sample operation has wrong index and number sample parameters  */
	CROSSRFID_ERROR_SYSFILE_RESET_PARAMETER,				/*!< 2nd words in register file contains an unexpected value  */

	/* these error codes are specific to the epc protocol	 */
	CROSSRFID_ERROR_WAITMOREBIT=0x70,
	CROSSRFID_ERROR_PARAMNETERNOTFOUND,
	CROSSRFID_WAIT_TIMEOUT,
	CROSSRFID_WAIT_RFFIELDOFF,
	CROSSRFID_ERROR_WRONG_PREAMBLE, 	/* Preamble error: from "PROTOCOL_DECODEPREAMBLE" */
	CROSSRFID_ERROR_OVERFLOW_RX_CMD, 		/* Command not found: error from "PROTOCOL_WAITPARAMETER" & "PROTOCOL_WAITCRC"*/
	CROSSRFID_ERROR_COMMANDNOTIDENTIFIED, 	/* Command not found: error from "PROTOCOL_DECODECOMMANDCODE" */
	CROSSRFID_ERROR_UNKNOWN_EPC_CMDID, 		/* Command not found: error from "PROTOCOL_DECODEPARAMAETER" */
	CROSSRFID_ERROR_CRC16_RX_CMD, 			/* CRC16 error found: error from "PROTOCOL_DECODECRC" */
	CROSSRFID_ERROR_WRONG_SYMBOL_TIMOUT,
	CROSSRFID_ERROR_RX_BUFFER_OVERFLOW,
	CROSSRFID_ERROR_UNKNOWNPARAMETER,		/*!< one parameter is erroneous */
	/* --- Note:  End of error code dedicated to the decoding state machine (EPC) --- */

	/* these error codes are specific to the non volatile memory of the uC */
	CROSSRFID_ERROR_NVM_INIT = 0x80,
	CROSSRFID_ERROR_NVM_READ,
	CROSSRFID_ERROR_NVM_WRITE,
	CROSSRFID_ERROR_NVM_OVERFLOW,
	CROSSRFID_ERROR_NVM_ISERASE,

	/* these error codes are specific to the serial communication*/
	CROSSRFID_ERROR_SERIAL_WRONGCOMMANDID = 0x90,
	CROSSRFID_ERROR_SERIAL_WRONGOBJECTID,
	CROSSRFID_ERROR_SERIAL_WRONGOPERATIONID,
	CROSSRFID_ERROR_SERIAL_WRONGACTIONID,

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	,
	CROSSRFID_GP_ERROR_LIFE_COMPUT = (0x90),
	CROSSRFID_GP_END_LIFE
#endif

}CrossRfid_Status_enum;

#endif


