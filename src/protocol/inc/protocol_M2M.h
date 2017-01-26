/*******************************************************************************
 * @file protocol_m2m.h
 * @brief this file is the service layer for the M2M communication
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef PROTOCOLM2M_H_
#define PROTOCOLM2M_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "interface_leuart.h"
#include "srvEM4325.h"
#include "srv24LC64.h"




/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define SRVM2M_COMMANDCODE_AVAILABLE						0x0C00
#define SRVM2M_MEMBANK_AVAILABLE							0x0003


#define SRVM2M_COMMANDCODE_MASK								0x0400
#define SRVM2M_COMMANDCODE_SHIFT							10
#define SRVM2M_MEMBANK_MASK									0x0F
#define SRVM2M_MEMBANK_SHIFT								0x0
#define SRVM2M_SYSTEMFILEADDRESS_MASK						0x03F8
#define SRVM2M_SYSTEMFILEADDRESS_SHIFT						0x3
#define SRVM2M_NBWORD_MASK									0x07
#define SRVM2M_NBWORD_SHIFT									0x0

#define SRVM2M_MAXNBWORDTOREAD								8
/*===========================================================================================================
						Enum definition
===========================================================================================================*/


/**
 *  @enum 	srvEM4325_Handshake_enum
 *  @brief 	this enum defines the communication way and it's used as
 *  a handshake
 */
typedef enum {
	PRTM2M_HANDSHAKE_RFHOSTTOCROSS = 0b0,	/*!< the communication way is from the RF host to the �c*/
	PRTM2M_HANDSHAKE_CROSSTORFHOST ,			/*!< the communication way is from the �c to the RF host*/
}prtM2M_Handshake_enum;


/**
 *  @enum 	srvEM4325_status_enum
 *  @brief	this enum define the status of the register file
 *
 */
typedef enum {
	PRTM2M_REGFILE_STATUSKO = 0b0,		/*!< the status is an successful code*/
	PRTM2M_REGFILE_STATUSOK,				/*!< the status is not successful code*/
}prtm2m_status_enum;


/**
 *  @enum srvm2m_commandFormat_enum
 *  @brief this enum contains the available command code
  * @note		the frame format is the next one
 * 	|---------------|---------------|-----------|-----------------------|
 * 	|	1st byte 	| 2nd byte		| 3rd byte	|	4th byte			|
 * 	|---------------|---------------|-----------|-----------------------|
 * 	| Command code	| address		| NbByte	|	data (optional)		|
 * 	|& memory bank	|				|			|						|
 * 	|---------------|---------------|-----------|-----------------------|
 * 	|0xAX : Read	| 				| 			|						|
 * 	|0xBX : Write	| 				| 			|						|
 * 	|0xX0:E�		| 				| 			|						|
 * 	|0xX1:Systemfile|				| 			| 						|
 * 	|---------------|---------------|-----------|-----------------------|
 * @param[in] 	pui8Rxdata : pointer of the RX message
 *
 */
typedef enum{
	PRTM2M_COMMANDFORMAT_HEADER	= 0x00,				/*!< the first data is the header */
	PRTM2M_COMMANDFORMAT_CMDCODE	,				/*!< the first data of the command code & membank is the command code  */
	PRTM2M_COMMANDFORMAT_NBWORD 	,				/*!< the 2nd byte of the command is the number of byte following */
	PRTM2M_COMMANDFORMAT_SYSTEMFILEADRESS =0x01	,	/*!< the first byte of the command is the memory bank code */
	PRTM2M_COMMANDFORMAT_DATA 		= 0x04,			/*!< the 3rd byte of the command is the first data byte (optional)*/
}srvm2m_commandFormat_enum;


/**
 *  @enum 	srvm2m_HearderCode_enum
 *  @brief 	this enum contains the available header
 *
 */
typedef enum{
	PRTM2M_COMMANDECODE			= 0x0,				/*!< Not defined */
	PRTM2M_ACCESSSYSTEMFILE		= 'F',		/*!< the M2M host requests an access to the system file  */

}srvm2m_HearderCode_enum;



/**
 *  @enum srvm2m_commandcode_enum
 *  @brief this enum contains the available command code
 *
 */
typedef enum{
	PRTM2M_READCOMMANDCODE		= 0b0000,		/*!< the M2M host request a read access  */
	PRTM2M_WRITEOMMANDCODE 		= 0b0100,		/*!< the M2M host request a write access */
}srvm2m_commandcode_enum;

/**
 *  @enum srvm2m_memorybank_enum
 *  @brief this enum contains the available memory bank
 *
 */
typedef enum{
	PRTM2M_EXTERNALMEMORY		= '0',		/*!< the M2M host request an operation on the external memory */
	PRTM2M_SYSTEMFILE	 		= '1'			/*!< the M2M host request an operation on the system file */
}srvm2m_memorybank_enum;



/** @struct prtM2M_ResponseBitField_Type
 *  @var prtM2M_ResponseBitField_Type::ui16Rfu
 *  Member 'ui16Rfu' is not usable
 *  @var prtM2M_ResponseBitField_Type::ui16Handshake
 *  Member 'ui16Handshake' is the sender [0 (RF reader), 1 (Tag)]
 *  @var prtM2M_ResponseBitField_Type::ui16Status
 *  Member 'ui16Status' is the status of the process [0 (process failed), 1 (successful process)]
 *  @var prtM2M_ResponseBitField_Type::ui16RegisterAddress
 *  Member 'ui16RegisterAddress' is the logical address of the register
 *  @var prtM2M_ResponseBitField_Type::ui16NbWordsInParams
 *  Member 'ui16NbWordsInParams' is the number of words in the parameters of the command
 */
typedef struct {
	uint16_t ui16NbWordsInParams : 3;	/* LSB */
	uint16_t ui16RegisterAddress: 7;
	uint16_t ui16Status : 1;
	uint16_t ui16Handshake : 1;
	uint16_t ui16Header : 4;				/* MSB */
}prtM2M_ResponseBitField_Type;

/** @union prtM2M_RespFirstWord_union
 *
 */
typedef union {
	prtM2M_ResponseBitField_Type sBitsField;
	uint16_t ui16Value;
}prtM2M_RespFirstWord_union;





/*===========================================================================================================
						globale variable definition
===========================================================================================================*/



/*===========================================================================================================
						prototype
===========================================================================================================*/

void 	prtm2m_init 					( void );
bool 	prtm2m_IsRightCommandcode 		( uint8_t * const pui8RxData );
uint8_t prtm2m_ProcessRxMessage 		( uint8_t * const pui8Rxdata, kernel_DataExchange_Type *psdataobject );
void 	prtm2m_SendString				( const char* const str );
void 	prtm2m_Hex2ascii				( const uint8_t* const pui8HexArray, uint8_t ui8Nbelement , uint8_t *AsciiString);
void 	prtm2m_i16ToStr				( int16_t bin, unsigned char ui8Nbelement, uint8_t *AsciiString);

#endif /* PRTM2M_H_ */
