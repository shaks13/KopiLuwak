/*******************************************************************************
 * @file protocol_epctag.h
 * @brief this function set is codec for the EM4325 device
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef PROTOCOL_EPCTAG_H
#define PROTOCOL_EPCTAG_H

#include "common_library.h"
#include "common_statuscode.h"
#include "protocol_epc.h"
#include "timerdrv.h" /* todo: used just for "timer0_uElapsed" */
#include "interface_gpio.h" /* todo: used just for "gpio_IsFirstEdge" */
#include "crc16.h"
#include "bsp.h"

/*===========================================================================================================
						Defines
===========================================================================================================*/
#define PROTOCOL_BLFCOUNTER_SIZE 					100
#define PROTOCOL_MAXNBCMDBIT_SIZE 					(PROTOCOL_BLFCOUNTER_SIZE) /* Todo: define this value or find another method to stop the state machine if nbsymbols > nbMAX */
#define PROTOCOL_SMALLRESPONSE_NBBIT				41 /* Todo: define this value or find another method to stop the state machine if nbsymbols > nbMAX */
#define PROTOCOL_NB_FRAMESYNC_SYMBOL 				3 /*Delimiter & Tari & RtCal */
#define PROTOCOL_NB_PREAMBLE_SYMBOL 				4 /*Delimiter & Tari & RtCal & TrCal */

/* constat of the tag error code */
#define PROTOCOL_TAGERRORCODE_NBBIT 				8


#define PROTOCOL_ENCODING_RESETEDGE 				0
#define PROTOCOL_ENCODING_FIRSTEDGE 				1
#define PROTOCOL_ENCODING_NBPREAMBLEBIT				(4+6)
#define PROTOCOL_ENCODING_NBPILOTTONEBIT			(12)
#define PROTOCOL_ENCODING_NBEDGEMILLER2				4
#define PROTOCOL_ENCODING_NBEDGEMILLER4				8
#define PROTOCOL_ENCODING_NBEDGEMILLER8				16

/* the size of the different elements of the EPC standard*/
#define PROTOCOL_MAXCOMMAND_SIZE 					16	/* 16 bit*/

/* Offset (in bit) to read the length of EBV in the SELECT command */
#define PROTOCOL_OFFSET_NBBIT_EBV_SELECT 			(12)
#define PROTOCOL_OFFSET_NBBIT_EBV_READ 				(10)

/* Parameters to define the TIMER1 top value in order to detect wrong symbols in the Reader commands */
//#define PROTOCOL_DEFAULT_VALUE_RTCAL				(40)
#define PROTOCOL_DEFAULT_VALUE_RTCAL				(0x6A) /* todo: initial RTCAL value to define ! */
#define PROTOCOL_DEFAULT_VALUE_FACTOR_RTCAL			(3)
#define PROTOCOL_MIN_VALUE_FACTOR_RTCAL				(1)


#define PROTOCOL_CONVERT_DR(x)						((x==0) ? (8) : (64.0/3))

/*===========================================================================================================
						Enumerators
===========================================================================================================*/
/**
 *  @enum protcol_DecodingState_enum
 *  @brief this enum contains the different state of the decoding state machien
 */
typedef enum {
  PROTOCOL_WAITPREAMBLE =0x00,		/*!< Wait to receive 3 symbol */
  PROTOCOL_DECODEPREAMBLE,		/*!< the decoding of the preamble (Tari, RtCal, TrCal) */
  PROTOCOL_WAITCOMMANDCODE,		/*!< Wait to receive 3 symbol */
  PROTOCOL_DECODECOMMANDCODE,	/*!< the decoding of the command code */
  PROTOCOL_WAITPARAMETER,		/*!< Wait to receive 3 symbol */
  PROTOCOL_DECODEPARAMAETER,		/*!< the decoding of the data */
  PROTOCOL_CHECK_NO_RECETPION_DURING_TIMEOUT, /*!< during this state, no symbols has to be received */
  PROTOCOL_WAITCRC,				/*!< Wait to receive the CRC */
  PROTOCOL_DECODECRC,			/*!< the decoding of the data */
  PROTOCOL_COMPUTEDATA,			/*!< the decoding of the data */
  PROTOCOL_COMPUTECRC,			/*!< the computation of the received CRC*/
  PROTOCOL_WAITT1,				/*!< the computation of the received CRC*/
} protcol_DecodingState_enum;


/**
 *  @enum 	protcol_TagErrorCode_enum
 *  @brief 	this enum contains the index of the array of the available Tag error code.
 *  the error code are stored in the array protocol_auErrorcode
 */
typedef enum {
  PROTOCOL_TAGERROR_OTHER,						/*!< other error */
  PROTOCOL_TAGERROR_NOTSUPPORTED,				/*!< Not supported*/
  PROTOCOL_TAGERROR_INSUFFICIENTPRIVILEGES,		/*!< Insufficient privileges*/
  PROTOCOL_TAGERROR_MEMORYOVERRUN,				/*!< Memory overrun*/
  PROTOCOL_TAGERROR_MEMORYLOCKED,				/*!< Memory locked*/
  PROTOCOL_TAGERROR_CRYPTOSUITE,				/*!< Crypto suite error*/
  PROTOCOL_TAGERROR_COMMANDNOTENCAPSULATED,	 	/*!< Command not encapsulated*/
  PROTOCOL_TAGERROR_RESPONSEBUFFEROVERFLOW,		/*!< ResponseBuffer overflow*/
  PROTOCOL_TAGERROR_SECURITYTIMEOUT,			/*!< Security timeout*/
  PROTOCOL_TAGERROR_INSUFFICIENTPOWER,			/*!< Insufficient power*/
  PROTOCOL_TAGERROR_NONSPECIFIC,				/*!< Non-specific error*/
} protcol_TagErrorCode_enum;


/**
 *  @enum	protcol_ResponseHeader_enum
 *  @brief 	this enum contains the two available header of the tag response
 */
typedef enum {
  PROTOCOL_HEADER_SUCESS =0x0,		/*!<sucessful header  */
  PROTOCOL_HEADER_ERROR =0x1		/*!<error code header  */
} protcol_ResponseHeader_enum;

/*===========================================================================================================
						Private structures declarations
===========================================================================================================*/
/** @struct protocol_parametersManagementInCmd_struct
 *  @brief this structure contains the length of the parameters that are not fixed in the EPC Gen2
 *  @var protocol_parametersManagementInCmd_struct::ui8EBV
 *  Member 'ui8EBV' is the unit such as (ui8EBV*8) = number of bit in the 'EBV' parameter
 *  @var protocol_parametersManagementInCmd_struct::ui8Variable
 *  Member 'ui8Variable' is the number of bit in the parameter with a size = 'Variable'
 *  @var protocol_parametersManagementInCmd_struct::bParameterUpdated
 *  Member 'bParameterUpdated' is a flag to know if the length of parameters has been updated or not
 */
typedef struct
{
	uint8_t ui8EBV;					/* possible values: 1 to 3 bytes */
	uint8_t ui8Variable;			/* possible values: 0 to 255 bit */
	bool	bParameterUpdated;
}protocol_parametersManagementInCmd_struct;

/* @struct protocol_selectCommandFields_struct */
typedef struct
{
	bool * pTarget;
	bool * pAction;
	bool * pMemBank;
	bool * pPointer;
	bool * pLength;
	bool * pMask;
	bool * pTrucate;
	bool * pCrc16;
}protocol_selectCommandFields_struct;

/* @struct protocol_challengeCommandFields_struct */
typedef struct
{
	bool * pIncRepLen;
	bool * pImmed;
	bool * pCsi;
	bool * pLength;
	bool * pMessage;
	bool * pCrc16;
}protocol_challengeCommandFields_struct;

/* @struct protocol_queryCommandFields_struct */
typedef struct
{
	bool * pDr;
	bool * pM;
	bool * pTRext;
	bool * pSel;
	bool * pSession;
	bool * pTarget;
	bool * pQ;
	bool * pCrc;
}protocol_queryCommandFields_struct;

/* @struct protocol_ackCommandFields_struct */
typedef struct
{
	bool * pRn;
}
protocol_ackCommandFields_struct;

/* @struct protocol_reqRnCommandFields_struct */
typedef struct
{
	bool * pRn;
	bool * pCrc16;
}protocol_reqRnCommandFields_struct;

/* todo: struct for all cmd to do*/
/* ... */

/* @struct protocol_readCommandFields_struct */
typedef struct
{
	bool * pMemBank;
	bool * pWordPtr;
	bool * pWordCount;
	bool * pRN;
	bool * pCrc16;
}protocol_readCommandFields_struct;

typedef union
{
	protocol_selectCommandFields_struct sSelect;
	protocol_challengeCommandFields_struct sChallenge;
	protocol_queryCommandFields_struct sQuery;
	protocol_ackCommandFields_struct sAck;
	protocol_reqRnCommandFields_struct sReqRn;
	/* todo: struct for all cmd to do*/
	/* ... */
	protocol_readCommandFields_struct sRead;
	/* ... */
}protocol_commandsParamFields_union;




/*===========================================================================================================
						Public variables definitions
===========================================================================================================*/
extern volatile uint8_t 		auClockCounterPerSymbol [PROTOCOL_BLFCOUNTER_SIZE];
extern volatile uint8_t 		uNbSymbol;
extern protocol_preamble_struct protocol_preamble;							/* this sructure contains the value of the preamble*/
extern uint8_t 					uNbBitDecodedinCmd ;						/* number of bit decoded in the  auCmdDecoded array*/
extern bool 					auCmdDecoded [PROTOCOL_MAXNBCMDBIT_SIZE];	/* this array contains the cmd decoded*/
extern uint8_t 					uDecodingMachineState ;						/* state of the state machine*/
extern protocol_response_struct *protocol_pTagResponse;					/* pointer structure of the tag response */
/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
uint8_t protocol_decodeEpcCmd (void);
void protocol_initDecodingmachine(void);
void protocol_resetDecodingmachine(void);
void protocol_EncodeData (protocol_response_struct * sresponse_struct );
void protocol_appendCRC16(const uint16_t uiCRC16, bool *pEndofReponse);
#endif
