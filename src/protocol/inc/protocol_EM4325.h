/*******************************************************************************
 * @file protocol_EM4325.h
 * @brief this function set is codec for the EM4325 device
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef PROTOCOL_EM4325_H
#define PROTOCOL_EM4325_H

#include "common_library.h"
#include "interface_spi.h"
//#include "serial_process.h"


/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define PROTOCOL_EM4325_SPIBUFFERSIZE					(8+1) 			/* the number of byte of the SendSPI command is 8 bytes*/

/* the folowing constants defines the number of byte of the commands according to the EM4325 datasheet*/
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_REQUESTSTATUS	0x01			/*nb of bytes of the SPIRequestStatus command*/
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_BOOT				0x01			/*nb of bytes of the SPIBoot command*/
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_READWORD			0x02			/*nb of bytes of the SPIReadWord command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_WRITEWORD		0x04			/*nb of bytes of the SPIWriteWord command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_READPAGE			0x02			/*nb of bytes of the SPIReadPage command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_READREGFILEWORD	0x02			/*nb of bytes of the SPIWRegisterFileWord command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_WRITEREGFILEWORD	0x04			/*nb of bytes of the SPIWriteRegisterFileWord command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_REQRN			0x01			/*nb of bytes of the SPIReqNewHandle command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_REQNEWHANDLE		0x02			/*nb of bytes of the SPIReqNewHandle command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_SETHANDLE		0x04			/*nb of bytes of the SPISetHandle command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_SETCOMMPARAMS	0x04			/*nb of bytes of the SPISetCommParams command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_GETCOMMPARAMS	0x02			/*nb of bytes of the SPIGetCommParams command */
#define PROTOCOL_EM4325_SPI_CMD_NBBYTE_GETNEWSENSORDATA	0x01			/*nb of bytes of the SPIGetSensorData command (new sample case) */

/* the folowing constants defines the number of byte of the responses according to the EM4325 datasheet*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_REQUESTSTATUS	0x01			/*nb of bytes of the response to the command SPIRequestStatus*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_BOOT				0x01			/*nb of bytes of the response to the command SPIBoot*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_READWORD			0x03			/*nb of bytes of the response to the command SPIReadWord*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_WRITEWORD		0x01			/*nb of bytes of the response to the command SPIWriteWord*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_READPAGE			0x09			/*nb of bytes of the SPIReadPage response*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_READREGFILEWORD	0x03			/*nb of bytes of the response to the command SPIReadRegisterFileWord*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_WRITEREGFILEWORD	0x01			/*nb of bytes of the response to the command SPIWriteRegisterFileWord*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_REQRN			0x03			/*nb of bytes of the response to the command SPIReqRN*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_REQNEWHANDLE		0x06			/*nb of bytes of the response to the command SPIReqNewHandle*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_SETHANDLE		0x01			/*nb of bytes of the response to the command SPISetHandle*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_SETCOMMPARAMS	0x01			/*nb of bytes of the response to the command SPISetCommParams*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_GETCOMMPARAMS	0x05			/*nb of bytes of the response to the command SPIGetCommParams*/
#define PROTOCOL_EM4325_SPI_RES_NBBYTE_GETNEWSENSORDATA	0x09			/*nb of bytes of the response to the command SPIGetSensorData (new sample case) */

#define PROTOCOL_EM4325_SPIMASK_REGISTERFILEWORD		0x07			/* mask of the Register file word parameter of the SPI readRegister fileWord command*/
#define PROTOCOL_EM4325_SPI_REGISTERFILENBWORD			0x08			/* numeber of word of the register file*/


#define PROTOCOL_EM4325_TRANSPONDERSTATE_MASK			0x40			/* transponder field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_TRANSPONDERSTATE_SHIFT			0x06			/* transponder field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_DEVICESTATESTATE_MASK			0x38			/* device state  field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_DEVICESTATESTATE_SHIFT			0x3				/* device state field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_MEMORYSTATE_MASK				0x04			/* memory state  field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_MEMORYSTATE_SHIFT				0x2				/* memory state field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_COMMMANDSTATE_MASK				0x03			/* command state  field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_COMMMANDSTATE_SHIFT				0x0				/* command state field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_REGISTERADDDRESS_MASK			0x04			/* register address field of the response to the SPIREquestStatus */
#define PROTOCOL_EM4325_REGISTERADDDRESS_SHIFT			0x3F			/* register address field of the response to the SPIREquestStatus */


#define PROTOCOL_EM4325_MEASURETEMERATURE				(0x0400)
/* masks of the temperature in the "sensor data" MSW */
#define PROTOCOL_EM4325_MASK_TEMERATURE					(0x01FF)
#define PROTOCOL_EM4325_MASK_MSB_TEMERATURE				(0x00F0)
#define PROTOCOL_EM4325_MASK_LSB_TEMERATURE				(0x000F)
#define PROTOCOL_EM4325_MASK_NEG_TEMERATURE				(0x0100)
/* shifts of the temperature in the "sensor data" MSW */
#define PROTOCOL_EM4325_SHIFT_MSB_TEMERATURE			(4)
#define PROTOCOL_EM4325_SHIFT_LSB_TEMERATURE			(0)
#define PROTOCOL_EM4325_SHIFT_NEG_TEMERATURE			(8)
/* coefficients of the temperature in the "sensor data" MSW */
#define PROTOCOL_EM4325_COEF_MSB_TEMERATURE				(4)
#define PROTOCOL_EM4325_COEF_LSB_TEMERATURE				(0.25)
#define PROTOCOL_EM4325_COEF_NEG_TEMERATURE				(-1)
/* invalid measurement) */
#define PROTOCOL_EM4325_TEMERATURE_INVALID				(0b100000000)

#define PROTOCOL_EM4325_TEMERATURE_0_KELVIN				(-273)
/*===========================================================================================================
						Enum definition
===========================================================================================================*/
/**
 *  @enum protocol_em4325SPIcommandCode_enum
 *  @brief this enum contains EM4325 command set
 */
typedef enum {
	PROTOCOL_EM4325_SPI_CMD_REQUESTSTATUS =			0xE0,			/*!< command code of the SPIRequestStatus */
	PROTOCOL_EM4325_SPI_CMD_BOOT =					0xE1,			/*!< command code of the SPI BOOT */
	PROTOCOL_EM4325_SPI_CMD_ENABLETRANSPONDER =		0xE2,			/*!< command code of the SPITransponder */
	PROTOCOL_EM4325_SPI_CMD_DISABLETRANSPONDER =	0xE3,			/*!< command code of the SPITransponder */
	PROTOCOL_EM4325_SPI_CMD_GETSENSORDATA		 =	0xE4,			/*!< command code of the GetSensorData */
	PROTOCOL_EM4325_SPI_CMD_GETSENSORDATAAFTERNEW =	0xE5,			/*!< command code of the GetSensorDataAfter new sample */
	PROTOCOL_EM4325_SPI_CMD_READPAGE =				0xE9,			/*!< command code of the SPIReadPage */
	PROTOCOL_EM4325_SPI_CMD_GETNEWSENSORDATA =		0xE5,			/*!< command code of the Get sensor data after new sample */
	PROTOCOL_EM4325_SPI_CMD_SETFLAGS =				0xE6,			/*!< command code of the SPISetFlags*/
	PROTOCOL_EM4325_SPI_CMD_READWORD =				0xE7,			/*!< command code of the SPIReadWord*/
	PROTOCOL_EM4325_SPI_CMD_WRITEWORD =				0xE8,			/*!< command code of the SPIWriteWord*/
	PROTOCOL_EM4325_SPI_CMD_READREGFILEWORD =		0xEE,			/*!< command code of the SPIReadRegisterFileWord*/
	PROTOCOL_EM4325_SPI_CMD_WRITEREGFILEWORD =		0xEF,			/*!< command code of the SPIWriteRegisterFileWord*/
	PROTOCOL_EM4325_SPI_CMD_REQRN =					0xF0,			/*!< command code of the SPIReqRN*/
	PROTOCOL_EM4325_SPI_CMD_REQNEWHANDLE =			0xF1,			/*!< command code of the SPIReqNewHandle*/
	PROTOCOL_EM4325_SPI_CMD_SETHANDLE =				0xF2,			/*!< command code of the SPISetHandle*/
	PROTOCOL_EM4325_SPI_CMD_SETCOMMPARAMS =			0xF3,			/*!< command code of the SPISetCommParams*/
	PROTOCOL_EM4325_SPI_CMD_GETCOMMPARAMS =			0xF4,			/*!< command code of the SPIGetCommParams*/
}protocol_em4325SPIcommandCode_enum;




/**
 *  @enum procotol_bufferstate_enum
 *  @brief this enum contains the Id of the state of the em4325 buffer
 *
 */
typedef enum {
	PROTOCOL_BUFFER_LOCKED = false,		/*!< This Id to define when the buffer is locked */
	PROTOCOL_BUFFER_UNLOCKED = true		/*!< This Id to define when the buffer is unlocked */
}procotol_bufferstate_enum;

/*===========================================================================================================
						struct definition
===========================================================================================================*/


/*===========================================================================================================
						public variables declaration
===========================================================================================================*/

/*===========================================================================================================
						prototype
===========================================================================================================*/
void prtEM4325_InitSpiBus 					( const bool bIsMaster  );
void protocol_SendSpiCommand				(const uint8_t ui8IdCmd, uint16_t const * pui16Param1, uint16_t const * pui16Param2);
void protocol_ReadSpiResponse 				(uint8_t ui8IdCmd, uint16_t * pui16Param1, uint16_t * pui16Param2, uint16_t * pui16Param3);
uint8_t protocol_GetEm4325Status			(void);
#endif
