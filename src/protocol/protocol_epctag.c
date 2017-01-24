/*******************************************************************************
 * @file protocol_epctag.c
 * @brief this function set is codec for the EM4325 device
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_epctag.h"

/*===========================================================================================================
						Private variables declarations
===========================================================================================================*/
/*  structure managing the length of the not fixed parameters in command */
static protocol_parametersManagementInCmd_struct 	sParamManag;
/* Number of symbols to wait to go to the next state of the state machine in protocol_decodeEpcCmd() */
static uint8_t 										uNbSymbolToWait;
/* Id of the command found in the protocol_decodeEpcCmd() state machine */
static uint8_t 										uCommandId;
/* Set of pointers allowing to separate the parameters of each command */
static protocol_commandsParamFields_union 			uCommandParamFields;
/* ui8PreviousRTcal is used in the process allowing to detect a wrong symbol during the reception of a command.
 * It represents the RTCAL value found in the previous valid command.
 * The timer0 top value is defined by this value (and not by the current RTCAL) as soon as the first symbol is received. */
static uint8_t 										ui8PreviousRTcal = PROTOCOL_DEFAULT_VALUE_RTCAL;
/* (ui8RTcalFactor * ui8PreviousRTcal) = top value of the timer 0 overflow */
static uint8_t 										ui8RTcalFactor 	 = PROTOCOL_DEFAULT_VALUE_FACTOR_RTCAL;
static const bool 	protocol_commandcode[][10] = {		/* take care the index of this table should be aligned with protocol_epccommandId_enum*/
		{1,0,1,0},				/* select*/
		{1,1,0,1,0,1,0,0},		/* Challenge */
		{1,0,0,0},				/* query*/
		{1,0,0,1},				/* queryadj*/
		{0,0},					/* query rep*/
		{0,1},					/* Ack*/
		{1,1,0,0,0,0,0,0},		/* Nack*/
		{1,1,0,0,0,0,0,1},		/* ReqRn*/
		{1,1,0,0,0,0,1,0},		/* Read*/
		{1,1,0,0,0,0,1,1}		/* Write*/
};
#if WARNING_v120_NOT_USED
static const bool 	protocol_auErrorcode[][10] = {
		{0,0,0,0,0,0,0,0},		/* other error*/
		{0,0,0,0,0,0,0,1},		/* Not supported*/
		{0,0,0,0,0,0,1,0},		/* Insufficient privileges*/
		{0,0,0,0,0,0,1,1},		/* Memory overrun*/
		{0,0,0,0,0,1,0,0},		/* Memory locked*/
		{0,0,0,0,0,1,0,1},		/* Crypto suite error*/
		{0,0,0,0,0,1,1,0},		/* Command not encapsulated*/
		{0,0,0,0,0,1,1,1},		/* ResponseBuffer overflow*/
		{0,0,0,0,1,0,0,0},		/* Security timeout*/
		{0,0,0,0,1,0,1,1},		/* Insufficient power*/
		{0,0,0,0,1,1,1,1},		/* Non-specific error*/
};
#endif
static bool	protocol_auReqRNResponse[] = {
		1,1,0,1,0,0,0,0,1,1,0,1,0,0,0,0,		/* handle*/
		1,1,0,1,0,0,0,0,1,1,0,1,0,0,0,0			/*CRC16*/
};
/*===========================================================================================================
						Public variables declarations
===========================================================================================================*/
volatile uint8_t 	auClockCounterPerSymbol [PROTOCOL_BLFCOUNTER_SIZE];/* this array contains the number of BLF period between 2 RX Rising edges*/
bool 				auCmdDecoded [PROTOCOL_MAXNBCMDBIT_SIZE];		/* this array contains the cmd decoded*/
bool 				auSmallTagResponse	 [PROTOCOL_SMALLRESPONSE_NBBIT];	/* this array contains the small tag response (without data)*/
volatile uint8_t 	uNbSymbol = 0;									/* number of relevant element auBLFCounter array*/
uint8_t 			uNbBitDecodedinCmd = 0;							/* number of bit decoded in the  auCmdDecoded array*/
uint8_t 			uDecodingMachineState = 0;						/* state of the state machine*/
uint8_t				ui8NbBitFirstSymbols = PROTOCOL_NB_FRAMESYNC_SYMBOL; /* Number of symbols in the preamble or frame sync at the beginning of a command */
protocol_preamble_struct protocol_preamble;							/* this structure contains the value of the preamble*/
protocol_response_struct protocol_TagResponse;						/* structure of the tag response */
protocol_response_struct *protocol_pTagResponse;					/* pointer structure of the tag response */


/*===========================================================================================================
						Private functions declarations
===========================================================================================================*/
static uint8_t 		protocol_decodePreamble 		(void);
static void 		protocol_decodeEPCsymbols		(void);
static uint8_t 		protocol_GetNbParameterSymbol 	(const uint8_t uCommandId,const uint8_t uXParameter,const uint8_t uYParameter, uint8_t *uNbSymbolToWait);
static uint8_t 		protocol_interpretCmdCode		(uint8_t *pwNbSymbolToWait, uint8_t * puCommandId);
static uint8_t 		protocol_GetNbCRCSymbol 		(const uint8_t uCommandId, uint8_t *uNbSymbolToWait);
static void			protocol_enableEpcTimeout1		(void);
static void			protocol_enableTimeoutWrongSymbolDetection	(void);
static bool 		protocol_IsReponseExpected (const uint8_t uCommandId );
static bool  		protocol_IsImmediateReplyExpected (const uint8_t uCommandId );
#if WARNING_v120_NOT_USED
static uint8_t 		protocol_GetHandle (const uint8_t uCommandId, uint16_t *ui16handle);
static void  		protocol_appendTagResponse (const uint8_t uCommandId , protocol_response_struct *pTagResponse);
static void __attribute__((optimize("O2"))) protocol_appendTagResponse (const uint8_t uCommandId , protocol_response_struct *pTagResponse);
static uint8_t __attribute__((optimize("O2"))) protocol_GetHandle (const uint8_t uCommandId, uint16_t *ui16handle);
#endif
/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
/**************************************************************************//**
 * @brief this function computes the preambles fields of the command
 * @param[in]  none
 * @param[out] none
 * @return "CROSSRFID_ERROR_WRONG_PREAMBL" if an anomaly is detected in the preamble
 * @return "CROSSRFID_SUCCESSCODE" if the preamble is valid
 *****************************************************************************/
static uint8_t protocol_decodePreamble (void)
{
	uint8_t status = CROSSRFID_ERROR_WRONG_PREAMBLE;

	/* if RTcal >= Tari whereas the EPC gen2 specifies 2.5*Tari <= RTcal <= 3.0*Tari */
	if( (auClockCounterPerSymbol[2] < (auClockCounterPerSymbol[1]<<1)) ||  (auClockCounterPerSymbol[2] > (auClockCounterPerSymbol[1]<<2)))
	{
		status = CROSSRFID_ERROR_WRONG_PREAMBLE;
	}
	else
	{
		protocol_preamble.uDelimiter = auClockCounterPerSymbol[0];
		protocol_preamble.uTari = auClockCounterPerSymbol[1];
		protocol_preamble.uRtCal = auClockCounterPerSymbol[2];

		/* the TrCal field is only present in the query command and its value is between 1.1 and 3.0 RtCal*/
		if (auClockCounterPerSymbol[2] < auClockCounterPerSymbol[3])
		{
			//protocol_preamble.uTrCal = auClockCounterPerSymbol[3];
			ui8NbBitFirstSymbols = PROTOCOL_NB_PREAMBLE_SYMBOL;
		}
		else
		{
			//protocol_preamble.uTrCal = 0;
			ui8NbBitFirstSymbols = PROTOCOL_NB_FRAMESYNC_SYMBOL;

		}
		status = CROSSRFID_SUCCESSCODE;
	}

	return status;
}

/**************************************************************************//**
 * @brief this function decodes the incoming EPC command according to the
 * pivot (= RtCal/2). the table auClockCounterPerSymbol is updated
 * by the RX IRQ.
 * @detail the EPC gen2 specification defines a data 0 a symbol < pivot
 * and a data 1 > pivot
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
static __attribute__((optimize("O2"))) void protocol_decodeEPCsymbols (void)
{
uint8_t upivot = protocol_preamble.uRtCal >> 1;
uint8_t ui8Idx = 0;
uint8_t ui8NbBitTemp = 0;

	/* stores the previous nb bit decoded */
	ui8NbBitTemp = uNbBitDecodedinCmd;

	/* decodes the previous bit not processed yet */
	for(ui8Idx = ui8NbBitTemp+ui8NbBitFirstSymbols; ui8Idx < uNbSymbol; ui8Idx++)
	{
		if (auClockCounterPerSymbol[ui8Idx] < upivot)
		{
			auCmdDecoded[uNbBitDecodedinCmd++] = (_Bool)0;
		}
		else
		{
			auCmdDecoded[uNbBitDecodedinCmd++] = (_Bool)1;
		}
		/* Updates the CRC 16 for each bit */
		CRC16_update_crc16_computation(auCmdDecoded[uNbBitDecodedinCmd-1]);
	}
}

#if 0
/**************************************************************************//**
 * @brief This function checks if the received crc16 of a reader cmd is valid
 * @param[in]  none
 * @param[out] none
 * @return CROSSRFID_ERROR_CRC16_RX_CMD : the crc16 is not valid
 * @return CROSSRFID_SUCCESSCODE : the crc16 is valid
 *****************************************************************************/
static uint8_t  protocol_CheckCRC16 (void)
{
	uint8_t status = CROSSRFID_ERROR_CRC16_RX_CMD;
	uint16_t ui16CRCcomputed = 0;
	/* CRC computed */
	ui16CRCcomputed = get_raw_crc16_without_xor();

	if(ui16CRCcomputed == PROTOCOL_RESIDUCRC16)
	{
		status = CROSSRFID_SUCCESSCODE;
	}
	else
	{
		status = CROSSRFID_ERROR_CRC16_RX_CMD;
	}

	return status;
}
#endif

/**************************************************************************//**
 * @brief this function returns true when a response is expected according
 * to the EPC standard and set the pointer of the memory response
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @return true : a response is expected
 * @return false : the �c shall shall not send any response
 *****************************************************************************/
static bool __attribute__((optimize("O0"))) protocol_IsReponseExpected (const uint8_t uCommandId )
{
uint8_t bIsResponseExpected  = true;

	switch (uCommandId)
	{
		case PROTOCOL_EPC_CMDID_SELECT:		/* acc to the epc standard these commands doesn't wait any response*/
		case PROTOCOL_EPC_CMDID_NACK:
		case PROTOCOL_EPC_CMDID_UNKNOWN:
			bIsResponseExpected = false;
		break;
		case PROTOCOL_EPC_CMDID_QUERY:		/* these commands are managed by the EM4325 and not by the �c*/
		case PROTOCOL_EPC_CMDID_QUERYADJ:
		case PROTOCOL_EPC_CMDID_QUERYREP:
		case PROTOCOL_EPC_CMDID_ACK:
			bIsResponseExpected = false;
		break;

		case PROTOCOL_EPC_CMDID_REQRN:		/* these commands are managed by the EM4325 and not by the �c*/
			bIsResponseExpected = true;
			protocol_pTagResponse->pResponse = protocol_auReqRNResponse;
		break;
		default :
			bIsResponseExpected = false;
		break;
	}
	return bIsResponseExpected ;
}

#if WARNING_v120_NOT_USED
/**************************************************************************//**
 * @brief this function appends to the tag response the handle. For some specific
 * response like error code or the reqrn the response can be fully completed fill in
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @param[out]  pTagResponseEndOfdata : the pointer of the response after the data
 * @return none
 *****************************************************************************/
static void __attribute__((optimize("O2"))) protocol_appendTagResponse (const uint8_t uCommandId , protocol_response_struct *pTagResponse)
{
uint8_t uNthBit = 0;
uint8_t uIdxHandle = PROTOCOL_SMALLRESPONSE_NBBIT- PROTOCOL_HANDLE_NBBIT - PROTOCOL_CRCON16BITS;
uint8_t uIdxErrorCode = uIdxHandle - PROTOCOL_TAGERRORCODE_NBBIT;
bool ui16MyHandle [] = {1,1,0,1,0,0,0,0,1,1,0,1,0,0,0,0}; /*0xD0D0 */

	switch (uCommandId)
	{
		case PROTOCOL_EPC_CMDID_REQRN: /* REqRN response : handle + CRC16*/
			pTagResponse->pResponse = auSmallTagResponse;
			pTagResponse->uNbBit = PROTOCOL_HANDLE_NBBIT ;
			memcpy(pTagResponse->pResponse,ui16MyHandle,PROTOCOL_HANDLE_NBBIT); /* copy the handle */
		break;

		case PROTOCOL_EPC_CMDID_READ:


		break;

		break;
		default : /* error code response : header + ErrorCode + Handke +CRC16*/
			pTagResponse->uNbBit = PROTOCOL_TAGERRORCODE_NBBIT+ PROTOCOL_HEADER_NBBIT + PROTOCOL_HANDLE_NBBIT + PROTOCOL_CRCON16BITS;
			pTagResponse->pResponse = auSmallTagResponse;
			pTagResponse->pResponse[uNthBit++] = PROTOCOL_HEADER_ERROR;	/* copy the header */
			memcpy(&(pTagResponse->pResponse[uIdxErrorCode]),protocol_auErrorcode[PROTOCOL_TAGERROR_NONSPECIFIC],PROTOCOL_TAGERRORCODE_NBBIT); /* copy the error code*/
			memcpy(&(pTagResponse->pResponse[uIdxHandle]),(uint8_t *)(pTagResponse->abHandle),PROTOCOL_HANDLE_NBBIT);	   /* copy the handle*/

		break;
	}

}
#endif

/**************************************************************************//**
 * @brief this function returns true when a response is expected according
 * to the EPC standard
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @return true : a response is expected
 * @return false : the �c shall shall not send any response
 *****************************************************************************/
static bool __attribute__((optimize("O2"))) protocol_IsImmediateReplyExpected (const uint8_t uCommandId )
{
uint8_t bIsImmediateReply = false;

switch (uCommandId)
{

	case PROTOCOL_EPC_CMDID_QUERY:
	case PROTOCOL_EPC_CMDID_QUERYREP:
	case PROTOCOL_EPC_CMDID_QUERYADJ:
	case PROTOCOL_EPC_CMDID_ACK:
	case PROTOCOL_EPC_CMDID_REQRN:
	case PROTOCOL_EPC_CMDID_READ:
	case PROTOCOL_EPC_CMDID_ACCESS:
	case PROTOCOL_EPC_CMDID_FILEOPEN:
	case PROTOCOL_EPC_CMDID_READBUFFER:
		bIsImmediateReply = true;
	break;
	case PROTOCOL_EPC_CMDID_WRITE:
	case PROTOCOL_EPC_CMDID_KILL:
	case PROTOCOL_EPC_CMDID_LOCK:
	case PROTOCOL_EPC_CMDID_BLOCKWRITE:
	case PROTOCOL_EPC_CMDID_BLOCKERASE:
	case PROTOCOL_EPC_CMDID_BLOCKPERMALOCK:
	case PROTOCOL_EPC_CMDID_AUTHENTICATE:
	case PROTOCOL_EPC_CMDID_AUTHCOMM:
	case PROTOCOL_EPC_CMDID_SECURECOMM:
	case PROTOCOL_EPC_CMDID_KEYUPDATE:
	case PROTOCOL_EPC_CMDID_TAGPRIVILEGE:
	case PROTOCOL_EPC_CMDID_UNTRACEABLE:
	case PROTOCOL_EPC_CMDID_FILELIST:
	case PROTOCOL_EPC_CMDID_FILEPRIVILEGE:
	case PROTOCOL_EPC_CMDID_FILESETUP:
	case PROTOCOL_EPC_CMDID_SELECT:
	case PROTOCOL_EPC_CMDID_NACK:
		bIsImmediateReply = false;
	break;
	default :
		bIsImmediateReply = false;
	break;
}
	return bIsImmediateReply ;
}



/**************************************************************************//**
 * @brief this function returns the number of parameter of the EPC command.
 * this value depends of the command and of the parameters it self. by e.g.
 * the BlockWrite command could write 1 or multiple words and its parameter length
 * is not a constant. The input parameters uXParameter and uYParameter are the
 * parameters to compute the parameter length.
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @param[in]  uXParameter :
 * @param[out] wNbSymbolToWait : the number of symbol to be received.
 * @return CROSSRFID_SUCCESSCODE : the number of parameter has been succesfully computed
 * @return CROSSRFID_COMMANDDECODING_ERROR : the command could not be identified
 * @return CROSSRFID_WAITMOREBIT_ERROR : some more bit is required to identify
 * the command
 *****************************************************************************/
static uint8_t __attribute__((optimize("O2"))) protocol_GetNbCRCSymbol (const uint8_t uCommandId, uint8_t *uNbSymbolToWait)
{
uint8_t status  = CROSSRFID_SUCCESSCODE;

	switch (uCommandId)
	{
		case PROTOCOL_EPC_CMDID_SELECT:
		case PROTOCOL_EPC_CMDID_REQRN:
		case PROTOCOL_EPC_CMDID_READ:
		case PROTOCOL_EPC_CMDID_WRITE:
		case PROTOCOL_EPC_CMDID_KILL:
		case PROTOCOL_EPC_CMDID_LOCK:
		case PROTOCOL_EPC_CMDID_ACCESS:
		case PROTOCOL_EPC_CMDID_BLOCKWRITE:
		case PROTOCOL_EPC_CMDID_BLOCKERASE:
		case PROTOCOL_EPC_CMDID_BLOCKPERMALOCK:
		case PROTOCOL_EPC_CMDID_AUTHENTICATE:
		case PROTOCOL_EPC_CMDID_AUTHCOMM:
		case PROTOCOL_EPC_CMDID_SECURECOMM:
		case PROTOCOL_EPC_CMDID_KEYUPDATE:
		case PROTOCOL_EPC_CMDID_TAGPRIVILEGE:
		case PROTOCOL_EPC_CMDID_READBUFFER:
		case PROTOCOL_EPC_CMDID_UNTRACEABLE:
		case PROTOCOL_EPC_CMDID_FILEOPEN:
		case PROTOCOL_EPC_CMDID_FILELIST:
		case PROTOCOL_EPC_CMDID_FILEPRIVILEGE:
		case PROTOCOL_EPC_CMDID_FILESETUP:
			(*uNbSymbolToWait) = PROTOCOL_CRCON16BITS;
		break;
		case PROTOCOL_EPC_CMDID_QUERY:
			(*uNbSymbolToWait) = PROTOCOL_CRCON5BITS;
		break;
		case PROTOCOL_EPC_CMDID_QUERYREP:
		case PROTOCOL_EPC_CMDID_QUERYADJ:
		case PROTOCOL_EPC_CMDID_ACK:
		case PROTOCOL_EPC_CMDID_NACK:
			(*uNbSymbolToWait) = PROTOCOL_NOCRC;
		break;
		default :
			status = CROSSRFID_ERROR_UNKNOWN_EPC_CMDID;
		break;
	}
	return status ;
}

#if WARNING_v120_NOT_USED
/**************************************************************************//**
 * @brief this function returns the number of parameter of the EPC command.
 * this value depends of the command and of the parameters it self. by e.g.
 * the BlockWrite command could write 1 or multiple words and its parameter length
 * is not a constant. The input parameters uXParameter and uYParameter are the
 * parameters to compute the parameter length.
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @param[in]  uXParameter :
 * @param[out] wNbSymbolToWait : the number of symbol to be received.
 * @return CROSSRFID_SUCCESSCODE : the number of parameter has been succesfully computed
 * @return CROSSRFID_COMMANDDECODING_ERROR : the command could not be identified
 * @return CROSSRFID_WAITMOREBIT_ERROR : some more bit is required to identify
 * the command
 *****************************************************************************/
static uint8_t __attribute__((optimize("O2"))) protocol_GetHandle (const uint8_t uCommandId, uint16_t *ui16handle)
{
uint8_t status  = CROSSRFID_ERROR_PARAMNETERNOTFOUND;

	switch (uCommandId)
	{
		case PROTOCOL_EPC_CMDID_READ:
			(*ui16handle) = (uint16_t) (uCommandParamFields.sRead.pRN);
			status = CROSSRFID_SUCCESSCODE;
		break;
		case PROTOCOL_EPC_CMDID_WRITE:
		case PROTOCOL_EPC_CMDID_KILL:
		case PROTOCOL_EPC_CMDID_LOCK:
		case PROTOCOL_EPC_CMDID_ACCESS:
		case PROTOCOL_EPC_CMDID_BLOCKWRITE:
		case PROTOCOL_EPC_CMDID_BLOCKERASE:
		case PROTOCOL_EPC_CMDID_BLOCKPERMALOCK:
		case PROTOCOL_EPC_CMDID_AUTHENTICATE:
		case PROTOCOL_EPC_CMDID_AUTHCOMM:
		case PROTOCOL_EPC_CMDID_SECURECOMM:
		case PROTOCOL_EPC_CMDID_KEYUPDATE:
		case PROTOCOL_EPC_CMDID_TAGPRIVILEGE:
		case PROTOCOL_EPC_CMDID_READBUFFER:
		case PROTOCOL_EPC_CMDID_UNTRACEABLE:
		case PROTOCOL_EPC_CMDID_FILEOPEN:
		case PROTOCOL_EPC_CMDID_FILELIST:
		case PROTOCOL_EPC_CMDID_FILEPRIVILEGE:
		case PROTOCOL_EPC_CMDID_FILESETUP:
		break;
		default :
		break;
	}
	return status ;
}
#endif

/**************************************************************************//**
 * @brief this function identifies the command
 * machine
 * @param[in]  none
 * @param[out] uCommandId : the id of the command
 * @return CROSSRFID_SUCCESSCODE : the command has been successfully identified
 * @return CROSSRFID_COMMANDNOTIDENTIFIED_ERROR : the command has not
 * been successfully identified
 *****************************************************************************/
static bool __attribute__((optimize("O2"))) protocol_GetCommandCode(uint8_t *uCommandId)
{
uint8_t status = CROSSRFID_ERROR_COMMANDNOTIDENTIFIED;

	/* ********************************* > 16 bit decoded ********************** */
	if(uNbBitDecodedinCmd > 16)
	{
		/* TODO : to be completed*/
	}
	/* ********************** 16 bit decoded ********************** */
	else if(uNbBitDecodedinCmd == 16)
	{
		/* TODO : to be completed*/
	}
	/* ********************** 8 to 15 bit decoded ********************** */
	else if(uNbBitDecodedinCmd < 16 && uNbBitDecodedinCmd >= 8)
	{
		/* TODO : to be completed*/
		if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_REQRN],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_REQRN;
		}
		else if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_READ],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_READ;
		}
		/*else if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_WRITE],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_WRITE;
		}*/
		else if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_NACK],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_NACK;
		}
		else
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_UNKNOWN;
		}
	}
	/* ********************** 4 to 7 bit decoded ********************** */
	else if(uNbBitDecodedinCmd < 8 && uNbBitDecodedinCmd >= 4)
	{
		if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_SELECT],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_SELECT;
		}
		else if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_QUERY],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_QUERY;
		}
		/*else if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_QUERYADJ],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_QUERYADJ;
		}*/
		else
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_UNKNOWN;
		}
	}
	/* ********************** 2 or 3 bit decoded ********************** */
	else if(uNbBitDecodedinCmd < 4 && uNbBitDecodedinCmd >= 2)
	{
		/*if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_QUERYREP],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_QUERYREP;
		}
		else */if (!memcmp(auCmdDecoded,protocol_commandcode[PROTOCOL_EPC_CMDID_ACK],uNbBitDecodedinCmd))
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_ACK;
		}
		else
		{
			(*uCommandId) =PROTOCOL_EPC_CMDID_UNKNOWN;
		}
	}
	/* ********************** 2 or 3 bit decoded ********************** */
	else if(uNbBitDecodedinCmd < 2)
	{
		/* TODO : to be completed*/
	}
	else
	{
		/* TODO : to be completed*/
	}

	/* updates the status */
	if ((*uCommandId) !=PROTOCOL_EPC_CMDID_UNKNOWN)
	{
		status = CROSSRFID_SUCCESSCODE;
	}
	else if (((*uCommandId) !=PROTOCOL_EPC_CMDID_UNKNOWN) && (uNbBitDecodedinCmd==16))
	{
		status = CROSSRFID_ERROR_WAITMOREBIT;
	}
	else
	{
		status = CROSSRFID_ERROR_COMMANDNOTIDENTIFIED;
	}
	return status;
}

/**************************************************************************//**
 * @brief this function returns the number of parameter of the EPC command.
 * this value depends of the command and of the parameters it self. by e.g.
 * the BlockWrite command could write 1 or multiple words and its parameter length
 * is not a constant. The input parameters uXParameter and uYParameter are the
 * parameters to compute the parameter length.
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @param[in]  uXParameter :
 * @param[out] wNbSymbolToWait : the number of symbol to be received.
 * @return CROSSRFID_SUCCESSCODE : the number of required symbol have been received
 * @return CROSSRFID_COMMANDDECODING_ERROR : the command could not be identified
 * @return CROSSRFID_WAITMOREBIT_ERROR : some more bit is required to identify
 * the command
 *****************************************************************************/
static uint8_t __attribute__((optimize("O2"))) protocol_UpdateNbParameterSymbol (const uint8_t uCommandId,uint8_t *uNbSymbolToWait)
{
uint8_t status  = CROSSRFID_SUCCESSCODE;

		switch (uCommandId)
		{
			/* ********************************** SELECT ********************************** */
			case PROTOCOL_EPC_CMDID_SELECT:
			/* **************************************************************************** */

				/* if the MSB of the current EBV byte is = 1 */
				if(sParamManag.bParameterUpdated == false)
				{
					/* updates EBV length */
					if(true == (auCmdDecoded[PROTOCOL_OFFSET_NBBIT_EBV_SELECT+(sParamManag.ui8EBV-1)*8]) && (sParamManag.ui8EBV <= PROTOCOL_EPC_NBMAXBYTE_EBVPARAMETER))
					{
						sParamManag.ui8EBV ++;
						(*uNbSymbolToWait) += 8;
						status = CROSSRFID_ERROR_WAITMOREBIT;
					}
					/* updates Variable length */
					else
					{
						/* reads the length */
						/*----------*/
						/* todo: (bArray2int) may be re code a function which start from MSB */
						uint8_t mask = 1;
						for (uint8_t bnthElement = (PROTOCOL_OFFSET_NBBIT_EBV_SELECT+sParamManag.ui8EBV*8) + 7 ; bnthElement >= (PROTOCOL_OFFSET_NBBIT_EBV_SELECT+sParamManag.ui8EBV*8) ; bnthElement--)
						{
							if(true == auCmdDecoded[bnthElement])
							{
								sParamManag.ui8Variable += mask;
							}
							mask <<= 1;
						}
						/*----------*/

						/* updates the number of parameter to wait */
						(*uNbSymbolToWait) += sParamManag.ui8Variable;
						sParamManag.bParameterUpdated = true;

						/* Updates the pointers allowing to separate the parameters of this command */
						uCommandParamFields.sSelect.pTarget 	= &auCmdDecoded[PROTOCOL_EPC_CMDCODESIZE_SELECT];
						uCommandParamFields.sSelect.pAction 	= uCommandParamFields.sSelect.pTarget+3;
						uCommandParamFields.sSelect.pMemBank 	= uCommandParamFields.sSelect.pAction+3;
						uCommandParamFields.sSelect.pPointer 	= uCommandParamFields.sSelect.pMemBank+2;
						uCommandParamFields.sSelect.pLength 	= uCommandParamFields.sSelect.pPointer+(8*sParamManag.ui8EBV);
						uCommandParamFields.sSelect.pMask 		= uCommandParamFields.sSelect.pLength+8;
						uCommandParamFields.sSelect.pTrucate 	= uCommandParamFields.sSelect.pMask+sParamManag.ui8Variable;
						uCommandParamFields.sSelect.pCrc16 		= uCommandParamFields.sSelect.pTrucate+1;

						if(sParamManag.ui8Variable > 0)
						{
							status = CROSSRFID_ERROR_WAITMOREBIT;
						}
						else
						{
							status = CROSSRFID_SUCCESSCODE;
						}
					}
				}
				else
				{
					status = CROSSRFID_SUCCESSCODE;
				}

			break;

			/* *********************************** QUERY *********************************** */
			case PROTOCOL_EPC_CMDID_QUERY:
			/* **************************************************************************** */
				uCommandParamFields.sQuery.pDr 		= &auCmdDecoded[PROTOCOL_EPC_CMDCODESIZE_QUERY];
				uCommandParamFields.sQuery.pM 		= uCommandParamFields.sQuery.pDr+1;
				uCommandParamFields.sQuery.pTRext 	= uCommandParamFields.sQuery.pM+2;
				uCommandParamFields.sQuery.pSel 	= uCommandParamFields.sQuery.pTRext+1;
				uCommandParamFields.sQuery.pSession	= uCommandParamFields.sQuery.pSel+2;
				uCommandParamFields.sQuery.pTarget 	= uCommandParamFields.sQuery.pSession+2;
				uCommandParamFields.sQuery.pQ 		= uCommandParamFields.sQuery.pTarget+1;
				uCommandParamFields.sQuery.pCrc 	= uCommandParamFields.sQuery.pQ+4;
			break;

			/* *********************************** ACK *********************************** */
			case PROTOCOL_EPC_CMDID_ACK:
			/* **************************************************************************** */
				uCommandParamFields.sAck.pRn = &auCmdDecoded[PROTOCOL_EPC_CMDCODESIZE_ACK];
			break;

			/* ********************************** NACK ********************************** */
			case PROTOCOL_EPC_CMDID_NACK:
			/* *************************************************************************** */

			break;

			/* *********************************** REQRN *********************************** */
			case PROTOCOL_EPC_CMDID_REQRN:
			/* **************************************************************************** */
				uCommandParamFields.sReqRn.pRn = &auCmdDecoded[PROTOCOL_EPC_CMDCODESIZE_REQRN];
			break;

			/* *********************************** READ *********************************** */
			case PROTOCOL_EPC_CMDID_READ:
			/* **************************************************************************** */
				/* if the MSB of the current EBV byte is = 1 */
				if(true == (auCmdDecoded[PROTOCOL_OFFSET_NBBIT_EBV_READ+(sParamManag.ui8EBV-1)*8]) && (sParamManag.ui8EBV <= PROTOCOL_EPC_NBMAXBYTE_EBVPARAMETER))
				{
					/* updates EBV length */
					sParamManag.ui8EBV ++;
					(*uNbSymbolToWait) += 8;
					status = CROSSRFID_ERROR_WAITMOREBIT;
				}
				/* updates Variable length */
				else
				{
					/* Updates the pointers allowing to separate the parameters of this command */
					uCommandParamFields.sRead.pMemBank 		= &auCmdDecoded[PROTOCOL_EPC_CMDCODESIZE_READ];
					uCommandParamFields.sRead.pWordPtr 		= uCommandParamFields.sSelect.pTarget+2;
					uCommandParamFields.sRead.pWordCount 	= uCommandParamFields.sRead.pWordPtr+(8*sParamManag.ui8EBV);
					uCommandParamFields.sRead.pRN 			= uCommandParamFields.sRead.pWordCount+8;
					uCommandParamFields.sRead.pCrc16		= uCommandParamFields.sRead.pRN+16;
					status = CROSSRFID_SUCCESSCODE;
				}
			break;

			/* ********************************** WRITE ********************************** */
			case PROTOCOL_EPC_CMDID_WRITE:
			/* *************************************************************************** */
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_WRITE(sParamManag.ui8EBV); /* Todo: EBV in protocol_UpdateNbParameterSymbol */
			break;

			case PROTOCOL_EPC_CMDID_BLOCKWRITE:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_BLOCKWRITE(sParamManag.ui8EBV,sParamManag.ui8Variable);
			break;

			case PROTOCOL_EPC_CMDID_BLOCKERASE:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_BLOCKERASE(sParamManag.ui8EBV);
			break;

			case PROTOCOL_EPC_CMDID_BLOCKPERMALOCK:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_BLOCKPERMALOCK(sParamManag.ui8EBV,sParamManag.ui8Variable);
			break;

			case PROTOCOL_EPC_CMDID_AUTHENTICATE:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_AUTHENTICATE(sParamManag.ui8EBV);
			break;

			case PROTOCOL_EPC_CMDID_AUTHCOMM:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_AUTHCOMM(sParamManag.ui8EBV);
			break;

			case PROTOCOL_EPC_CMDID_SECURECOMM:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_SECURECOMM(sParamManag.ui8EBV);
			break;
			case PROTOCOL_EPC_CMDID_KEYUPDATE:
				//(*uNbSymbolToWait) += PROTOCOL_EPC_NBPARAMETER_KEYUPDATE(sParamManag.ui8EBV);
			break;

			default :
				status = CROSSRFID_ERROR_UNKNOWNPARAMETER;
			break;
		}
	return status ;
}

/**************************************************************************//**
 * @brief this function returns the number of parameter of the EPC command.
 * this value depends of the command and of the parameters itself. by e.g.
 * the BlockWrite command could write 1 or multiple words and its parameter length
 * is not a constant. The input parameters uXParameter and uYParameter are the
 * parameters to compute the parameter length.
 * @param[in]  uCommandId : the id of the EPC gen2 command (protocol_epccommandId_enum)
 * @param[in]  uXParameter :
 * @param[out] wNbSymbolToWait : the number of symbol to be received.
 * @return CROSSRFID_SUCCESSCODE : the number of parameter has been succesfully computed
 * @return CROSSRFID_COMMANDDECODING_ERROR : the command could not be identified
 * @return CROSSRFID_WAITMOREBIT_ERROR : some more bit is required to identify
 * the command
 *****************************************************************************/
static uint8_t __attribute__((optimize("O2"))) protocol_GetNbParameterSymbol (const uint8_t uCommandId,const uint8_t uXParameter,const uint8_t uYParameter, uint8_t *uNbSymbolToWait)
{
uint8_t status  = CROSSRFID_SUCCESSCODE;

	switch (uCommandId)
	{
		case PROTOCOL_EPC_CMDID_SELECT:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_SELECT(uXParameter,uYParameter) + PROTOCOL_EPC_CMDCODESIZE_SELECT;
		break;
		case PROTOCOL_EPC_CMDID_QUERY:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_QUERY + PROTOCOL_EPC_CMDCODESIZE_QUERY;
		break;
		case PROTOCOL_EPC_CMDID_QUERYADJ:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_QUERYADJ + PROTOCOL_EPC_CMDCODESIZE_QUERYADJ;
		break;
		case PROTOCOL_EPC_CMDID_QUERYREP:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_QUERYREP + PROTOCOL_EPC_CMDCODESIZE_QUERYREP;
		break;
		case PROTOCOL_EPC_CMDID_ACK:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_ACK + PROTOCOL_EPC_CMDCODESIZE_ACK;
		break;
		case PROTOCOL_EPC_CMDID_NACK:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_NACK + PROTOCOL_EPC_CMDCODESIZE_NACK;
		break;
		case PROTOCOL_EPC_CMDID_REQRN:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_REQRN + PROTOCOL_EPC_CMDCODESIZE_REQRN;
		break;
		case PROTOCOL_EPC_CMDID_READ:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_READ(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_READ;
		break;
		case PROTOCOL_EPC_CMDID_WRITE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_WRITE(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_WRITE;
		break;
		case PROTOCOL_EPC_CMDID_KILL:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_KILL + PROTOCOL_EPC_CMDCODESIZE_KILL;
		break;
		case PROTOCOL_EPC_CMDID_LOCK:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_LOCK + PROTOCOL_EPC_CMDCODESIZE_LOCK ;
		break;
		case PROTOCOL_EPC_CMDID_ACCESS:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_ACCESS + PROTOCOL_EPC_CMDCODESIZE_ACCESS;
		break;
		case PROTOCOL_EPC_CMDID_BLOCKWRITE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_BLOCKWRITE(uXParameter,uYParameter) + PROTOCOL_EPC_CMDCODESIZE_BLOCKWRITE;
		break;
		case PROTOCOL_EPC_CMDID_BLOCKERASE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_BLOCKERASE(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_BLOCKERASE;
		break;
		case PROTOCOL_EPC_CMDID_BLOCKPERMALOCK:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_BLOCKPERMALOCK(uXParameter,uYParameter) + PROTOCOL_EPC_CMDCODESIZE_BLOCKPERMALOCK;
		break;
		case PROTOCOL_EPC_CMDID_AUTHENTICATE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_AUTHENTICATE(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_AUTHENTICATE;
		break;
		case PROTOCOL_EPC_CMDID_AUTHCOMM:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_AUTHCOMM(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_AUTHCOMM;
		break;
		case PROTOCOL_EPC_CMDID_SECURECOMM:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_SECURECOMM(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_SECURECOMM;
		break;
		case PROTOCOL_EPC_CMDID_KEYUPDATE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_KEYUPDATE(uXParameter) + PROTOCOL_EPC_CMDCODESIZE_KEYUPDATE;
		break;
		case PROTOCOL_EPC_CMDID_TAGPRIVILEGE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_TAGPRIVILEGE + PROTOCOL_EPC_CMDCODESIZE_TAGPRIVILEGE;
		break;
		case PROTOCOL_EPC_CMDID_READBUFFER:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_READBUFFER + PROTOCOL_EPC_CMDCODESIZE_READBUFFER;
		break;
		case PROTOCOL_EPC_CMDID_UNTRACEABLE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_UNTRACEABLE + PROTOCOL_EPC_CMDCODESIZE_UNTRACEABLE;
		break;
		case PROTOCOL_EPC_CMDID_FILEOPEN:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_FILEOPEN + PROTOCOL_EPC_CMDCODESIZE_FILEOPEN;
		break;
		case PROTOCOL_EPC_CMDID_FILELIST:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_FILELIST + PROTOCOL_EPC_CMDCODESIZE_FILELIST;
		break;
		case PROTOCOL_EPC_CMDID_FILEPRIVILEGE:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_FILEPRIVILEGE + PROTOCOL_EPC_CMDCODESIZE_FILEPRIVILEGE;
		break;
		case PROTOCOL_EPC_CMDID_FILESETUP:
			(*uNbSymbolToWait) = PROTOCOL_EPC_NBPARAMETER_FILESETUP + PROTOCOL_EPC_CMDCODESIZE_FILESETUP;
		break;
		default :
			status = CROSSRFID_ERROR_UNKNOWNPARAMETER;
		break;
	}
	return status ;
}

/**************************************************************************//**
 * @brief Sets the top value of TIMER0 counter and enables the TIMER0
 * interruption (T1 time out) in order to know when the �C should send the
 * response to the reader.
 *
 * @detail The TIMER0 is configured in capture/compare on the EM4325 AUX pin
 * to count each front of the BLF clock. The TIMER0 CNT register is reset on
 * MISO rising edge (Rx pin). As soon as the reader command is finished, the
 * TIMER0 will no longer reset. The �C will send the response to the reader
 * when the overflow interruption will occur.
 *
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
static void	protocol_enableEpcTimeout1	(void)
{
	uint16_t u1610Tpri;
	uint16_t ui16TimeToWait;
	uint16_t ui16ProcessingDelay = 14; /* this timind is the dealy of the �c between the IRQ and the first resply edge
	 	 	 	 	 	 	 	 	 	 4 * HFfrequency / prescaler*/

	/* test the value of DR to find Tpri */
	if(*(uCommandParamFields.sQuery.pDr) != 0)		/* DR = 64/3*/
	{
		/* 10 x (Trcal/ (64/3)) = 10 x Tpri */
		u1610Tpri= 30 * protocol_preamble.uTrCal >> 6;
	}
	else	/* DR = 8*/
	{
		/* 10 x (Trcal / 8) = 10 x Tpri */
		u1610Tpri = 10 * protocol_preamble.uTrCal >> 3;
	}

	/* RTCAL > 10 Tpri */
	if(protocol_preamble.uRtCal > u1610Tpri)
	{
		/* todo : re configure the timer 0 on CC1 when the rf field is lost */
		/* todo: test: Timer 0 no longer on BLF but on HFRC0 */
		//TIMER0->CTRL = (TIMER0->CTRL & (~_TIMER_CTRL_CLKSEL_MASK)) | TIMER_CTRL_CLKSEL_PRESCHFPERCLK;

		/* epc's T1 = RTcal */

		/* for this case the timer 0 */
		ui16TimeToWait = protocol_preamble.uRtCal - ui16ProcessingDelay;
	}
	else
	{
		/* T1 = 10 Tpri */
		//ui16TimeToWait = 20;
		ui16TimeToWait = u1610Tpri - ui16ProcessingDelay;
	}

	TIMER_TopSet(TIMER0, ui16TimeToWait);

}

/**************************************************************************//**
 * @brief Sets the top value of TIMER1 counter and enables the TIMER1
 * interruption in order detect a wrong symbol during the reception of a
 * reader command.
 *
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
static void	protocol_enableTimeoutWrongSymbolDetection	(void)
{
	/* symbol > RTCAL * factor -> is a wrong symbol */
	TIMER_TopSet(TIMER1, ((uint16_t)(ui8PreviousRTcal)*(uint16_t)(ui8RTcalFactor)));
	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);
}

/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/**************************************************************************//**
 * @brief this function send on to the EM4325's AFE the Tag response.
 * @detail in the worst case when the BLF is 640kHz the Tx should toggle
 * on each half period (1.28 Mhz) so the number of cycles should be
 * less than HF frequency / 1.28 MHz
 * @detail this function works for the miller and not for the FM0
 * @param[in]  bIsPilotTone true is the pilot tone should be added false not
 * @param[in]  uNbBit nb bit to send (Data + CRC)
 * @param[in]  pointer of the data & CRC to send which is the tag response
 * @param[out] none
 * @return none
 *****************************************************************************/
void __attribute__((optimize("O3"))) protocol_EncodeData (protocol_response_struct * sresponse_struct )
{
bool uNextCheck=0;
bool IsToggleOnNextEdge=true;
uint8_t uNthEdge=PROTOCOL_ENCODING_RESETEDGE;
uint8_t uNbMiddleEdge = PROTOCOL_ENCODING_NBEDGEMILLER2/2  ; 	/* 1 for FM0, 2 for Miller 2 4 for Miller 4 and 8 for Miller 8*/
uint8_t uLastEdge = PROTOCOL_ENCODING_NBEDGEMILLER2;
uint8_t *pTagResposne = (uint8_t*) sresponse_struct->pResponse ; 			/* the header and the dummy symbol is included*/
uint8_t *pEndTagResposne = (uint8_t*) (sresponse_struct->pResponse+sresponse_struct->uNbBit + 1); 		/* the dummy symbol is included*/
uint8_t aStartofFrameWoPilotTone[] =   {0,0,0,0,0,1,0,1,1,1};
uint8_t aStartofFrameWithPilotTone[] = {0,0,0,0,
										0,0,0,0,
										0,0,0,0,
										0,0,0,0,0,1,0,1,1,1};
uint8_t *pStartOfFrame;
uint8_t *pEndOfStartOfFrame;

	/* choose the right start of frame*/
	if (sresponse_struct->bIsPilotTone == true)
	{
		pStartOfFrame =aStartofFrameWithPilotTone;
		pEndOfStartOfFrame =pStartOfFrame+PROTOCOL_ENCODING_NBPREAMBLEBIT+PROTOCOL_ENCODING_NBPILOTTONEBIT;
	}
	else
	{
		pStartOfFrame =aStartofFrameWoPilotTone;
		pEndOfStartOfFrame =pStartOfFrame+PROTOCOL_ENCODING_NBPREAMBLEBIT;
	}

	/*add the dummy symbol at the end*/
	*pEndTagResposne  = 1;


	/* Wait the epc's T1 expires.*/
	while(false == timer0_bElapsed );



	/* send the pilot tone and the start of frame
	 * the miller preamble contains some 0 data but unlike of data 0 of the response
	 * the TX should toggle on each 1/2 Tpri  */
	do{
		while (GETDATA_PA1 () == false); /* wait the rising edge*/
		if (true == IsToggleOnNextEdge)
		{
			TOGGLE_PC0();
		}else {
			IsToggleOnNextEdge = true;
		}

		uNthEdge ++;
		//IsToggleOnNextEdge = true;
		if (uNthEdge == PROTOCOL_ENCODING_FIRSTEDGE)	/* on the first edge */
		{
			uNextCheck = (bool) (*pStartOfFrame);		/* get the current data*/
		}
		else if ((uNthEdge == uNbMiddleEdge) 	&& 		/* on the second edge */
				(uNextCheck == 1 ))
		{
			IsToggleOnNextEdge = false;
		}
		else if (uNthEdge == 3 )						/* on the third edge */
		{
			/* nothing to do*/
		}
		else if (uNthEdge == uLastEdge ) 				/* on the last edge */
		{
			uNthEdge = PROTOCOL_ENCODING_RESETEDGE;
			pStartOfFrame++;
		}else { /* do nothing*/}
		//while (GETDATA_PA1 () == true); /* wait the LowLevel state */
	}while (pStartOfFrame<pEndOfStartOfFrame); /* loop of the response to send*/

	uNthEdge = PROTOCOL_ENCODING_RESETEDGE;
	IsToggleOnNextEdge=true;

	/* send the data */
	do {
		while (GETDATA_PA1 () == false); /* wait the rising edge*/
		if (true == IsToggleOnNextEdge)
		{
			TOGGLE_PC0();
		}else { /* do nothing*/}

		uNthEdge ++;
		IsToggleOnNextEdge = true;
		if (uNthEdge == PROTOCOL_ENCODING_FIRSTEDGE)	/* on the first edge */
		{
			uNextCheck = (bool) (*sresponse_struct->pResponse);		/* get the current data*/
		}
		else if ((uNthEdge == uNbMiddleEdge) 	&& 		/* on the second edge */
				(uNextCheck == 1 ))
		{
			IsToggleOnNextEdge = false;
		}
		else if (uNthEdge == 3 )						/* on the third edge */
		{
			pTagResposne++;
			uNextCheck |=(bool) (*pTagResposne);   		/* get the next data*/
		}
		else if (uNthEdge == uLastEdge ) 				/* on the last edge */
		{
			uNthEdge = PROTOCOL_ENCODING_RESETEDGE;
			if (uNextCheck == 0) 					/* 2 data 0 following*/
			{
				IsToggleOnNextEdge = false;
			} else { /* do nothing*/}
		}else { /* do nothing*/}

		//while (GETDATA_PA1 () == true); /* wait the LowLevel state */

	}while (pTagResposne<pEndTagResposne); /* loop of the response to send*/

	/*because the loop exits on the 3rd edge of the loop
	 * the two last edges should be set outside the loop*/
	while (GETDATA_PA1 () == false); /* wait the rising edge*/
	if (!GET_PC0())
	{
		SET_PC0();
	}  else {}

	while (GETDATA_PA1 () == true); /* wait the rising edge*/
	while (GETDATA_PA1 () == false); /* wait the LowLevel state */
	CLEAR_PC0 (); /* the low level should be the final state*/


}
#if 0
/**************************************************************************//**
 * @brief this function sendon to the EM4325's AFE the Tag response.
 * @detail in the worst case when the BLF is 640kHz the Tx should toogle
 * on each half period (1.28 Mhz) so the number of cycles should be
 * less than HF frequency / 1.28 MHz
 * @param[in]  uNbBit nb bit to send
 * @param[in]  pointer of the data to send which is the tag response
 * @param[in] none
 *
 * @return none
 *****************************************************************************/
void __attribute__((optimize("O3"))) protocol_EncodeDataWithoutBLF (const uint8_t uNbBit, uint8_t * pTagResposne)
{
bool uNextCheck=0;
uint8_t uNthEdge=0;
bool IsToggleOnNextEdge=true;
//uint8_t uNbEdgeByBit = 4; /* 2 for FM0, 4 for Miller 2 8 for Miller 4 and 16 for Miller 8*/
//uint8_t uNbMiddleEdge = uNbEdgeByBit/2  ; /* 1 for FM0, 2 for Miller 2 4 for Miller 4 and 8 for Miller 8*/
//uint8_t uLastEdge = uNbEdgeByBit; /* 1 for FM0, 2 for Miller 2 4 for Miller 4 and 8 for Miller 8*/
uint8_t *pEndTagResposne = pTagResposne+uNbBit;

	do {

		while (GETDATA_PA1 () == false); /* wait the rising edge*/
		if (true == IsToggleOnNextEdge)
		{
			TOGGLE_PC0();
		}//else { /* do nothing*/}

		uNthEdge ++;
		IsToggleOnNextEdge = true;
		if (uNthEdge == 1)							/* on the first edge */
		{
			uNextCheck = (bool) (*pTagResposne);	/* get the current data*/
		}
		else if ((uNthEdge == 2) 	&& 				/* on the second edge */
				(uNextCheck == 1 ))
		{
			IsToggleOnNextEdge = false;
		}
		else if (uNthEdge == 3 )					/* on the third edge */
		{
			pTagResposne++;
			uNextCheck |=(bool) (*pTagResposne);   /* get the next data*/
		}
		else if (uNthEdge == 4 ) 					/* on the last edge */
		{
			uNthEdge = 0;
			if (uNextCheck == 0 )					/* 2 data 0 following*/
			{
				IsToggleOnNextEdge = false;
			} else { /* do nothing*/}
		}else { /* do nothing*/}

		while (GETDATA_PA1 () == true); /* wait the LowLevel state */

	}while (pTagResposne<pEndTagResposne); /* loop of the response to send*/
}

#endif

#if 0
/**************************************************************************//**
 * @brief this function sendon to the EM4325's AFE the Tag response.
 * @detail in the worst case when the BLF is 640kHz the Tx should toogle
 * on each half period (1.28 Mhz) so the number of cycles should be
 * less than HF frequency / 1.28 MHz
 * @param[in]  uNbBit nb bit to send
 * @param[in]  pointer of the data to send which is the tag response
 * @param[in] none
 *
 * @return none
 *****************************************************************************/
void __attribute__((optimize("O2"))) protocol_EncodeDataOptimize (const uint8_t uNbBit,  uint8_t * pTagResposne)
{
uint8_t uNthBit=0;
uint8_t uNthEdge=4;
uint8_t ushift =0;
uint8_t auToogleMask[] ={0x7,0xD,0xF,0xD};	/* miller 2*/
uint32_t uToogleMask =0;


	for ( uNthBit = 0 ; uNthBit < uNbBit ; uNthBit++) /* loop of the response to send*/
	{
		if (uNthEdge==4)	 /* when the four edges have process*/
		{
			uNthEdge =1;
			//uBitCouple = (*pTagResposne) & (*(pTagResposne+1) << 1); /* load the couple of bit */
			uToogleMask = auToogleMask[(((uint32_t) (*pTagResposne)>>ushift) & 0x03 )];
			if (ushift++==31)
				pTagResposne++;
		}

		while (GETDATA_PA1 () == false); /* wait the rising edge*/
		if (true == (uToogleMask << uNthEdge++) )
		{
			TOGGLE_PC0();
		}
		while (GETDATA_PA1 () == true); /* wait the LowLevel state */

	}/* loop of the response to send*/

}
#endif

/**************************************************************************//**
 * @brief this function appends to the tag response the computed CRC16
 * machine
 * @param[in] uiCRC16 : CRC16 to be appended to the tag response
 * @param[in] pEndofReponse : last bit of the tag response
 * @param[out] none
 * @return none
 *****************************************************************************/
void __attribute__((optimize("O0"))) protocol_appendCRC16 ( const uint16_t uiCRC16, bool *pEndofReponse )
{
bool * pCRC16 = pEndofReponse ;
bool * pEndofCRC16 = pEndofReponse + PROTOCOL_CRCON16BITS ;
uint16_t mask=0x8000 ;

	do
	{
		if (uiCRC16 & mask)
		{
			(*pCRC16) = 1;
		}
		else
		{
			(*pCRC16) = 0;
		}
		pCRC16++;
		mask>>=1;
	} while (pCRC16 <pEndofCRC16 );
}
/**************************************************************************//**
 * @brief this function initializes the global variables of the decoding
 * machine
 * @param[in]  none
 * @param[in] none
 * @return none
 *****************************************************************************/
void protocol_initDecodingmachine(void)
{
uint16_t uiCRC16;
#if 0
	uNbSymbol 						= 0; /* no symbols */
	uNbBitDecodedinCmd 				= 0; /* no bit decoded */
	ui8NbBitFirstSymbols 			= PROTOCOL_NB_FRAMESYNC_SYMBOL;
	uNbSymbolToWait					= 2; /* 2 bit minimum per command code */
	uCommandId 						= PROTOCOL_EPC_CMDID_UNKNOWN; /* Command not found */

	/* Parameters with a length not fixed */
	sParamManag.ui8Variable 		= 0; /* minimum parameter value */
	sParamManag.ui8EBV 				= 1; /* minimum parameter value */
	sParamManag.bParameterUpdated 	= false; /* Parameter length not found */

	gpio_IsFirstEdge 				= true; /* First edge to detect the delimiter */

	/* Clears arrays */
	memset (auClockCounterPerSymbol,0,PROTOCOL_BLFCOUNTER_SIZE);
	memset (auCmdDecoded,0,PROTOCOL_MAXNBCMDBIT_SIZE);
#endif

	timer0_bElapsed 				= false;	 /* reset overflow timer 0 */
	protocol_pTagResponse=&protocol_TagResponse; /* initialize the pointer */
	protocol_pTagResponse->bIsHandleFound = false;

	interface_DisablePinInterrupt(INTERFACE_AUX_PORT,INTERFACE_AUX_PIN);

	timer_initCounter ();												/* init timer 1 to count on HF edge & timer 0 to count on BLF*/
	protocol_enableTimeoutWrongSymbolDetection();						/* enable timer 1 overflow in order to detect a wrong symbol inside the commands */


	GPIO_PinOutSet (INTERFACE_CLK_PORT,INTERFACE_CLK_PIN);				/* Set to HL P2_SCLK to activate the BYPASSSIGNAL*/
	GPIO_PinOutSet (INTERFACE_CS_PORT,INTERFACE_CS_PIN);				/* Set to HL P2_CS to get out the BLF clock on aux pad*/


	GPIO_PinModeSet(INTERFACE_AUX_PORT,INTERFACE_AUX_PIN, gpioModeInput,  0);  	/* Aux as input the configration of the others pad are done during the init of the SPI*/

	CMU_HFRCOBandSet(cmuHFRCOFreq_32M0Hz);

	/* compute the CRC of the handle response*/
	uiCRC16 = crc16_ccitt_bit2 ((uint8_t*)protocol_auReqRNResponse,PROTOCOL_HANDLE_NBBIT);
	protocol_appendCRC16(uiCRC16,  &(protocol_auReqRNResponse[PROTOCOL_HANDLE_NBBIT]));
#if 0
	/* enables the falling edge IRQ on the RX to detect the next delimiter */
	interface_EnablePinInterrupt(INTERFACE_RX_PORT, INTERFACE_RX_PIN, INTERFACE_FALLINGANDRISING_EDGE);
	/* State machine at the initial state */
	uDecodingMachineState 			= PROTOCOL_WAITPREAMBLE;
#endif
}

/**************************************************************************//**
 * @brief this function reset the glocbal variables of the decoding
 * machine
 * @param[in]  none
 * @param[in] none
 * @return none
 *****************************************************************************/
void protocol_resetDecodingmachine (void)
{
	/* avoids the overflow, wait for a new falling edge */
	TIMER1->CMD = TIMER_CMD_STOP;
	TIMER0->CMD = TIMER_CMD_STOP;

	uNbSymbol 						= 0; /* no symbols */
	uNbBitDecodedinCmd 				= 0; /* no bit decoded */
	ui8NbBitFirstSymbols 			= PROTOCOL_NB_FRAMESYNC_SYMBOL;
	uNbSymbolToWait					= 2; /* 2 bit minimum per command code */
	uCommandId 						= PROTOCOL_EPC_CMDID_UNKNOWN; /* Command not found */

	/* Parameters with a length not fixed */
	sParamManag.ui8Variable 		= 0; /* minimum parameter value */
	sParamManag.ui8EBV 				= 1; /* minimum parameter value */
	sParamManag.bParameterUpdated 	= false; /* Parameter length not found */

	gpio_IsFirstEdge 				= true; /* First edge to detect the delimiter */
	timer1_bElapsed 				= false;	/* reset overflow timer 0 */

	/* updates the value of the timer 1 overflow according to the previous valid command parameters */
	TIMER_TopSet(TIMER1, ((uint16_t)(ui8PreviousRTcal)*(uint16_t)(ui8RTcalFactor)));

	/* enables the falling edge IRQ on the RX to detect the next delimiter */
	interface_EnablePinInterrupt(INTERFACE_RX_PORT, INTERFACE_RX_PIN, INTERFACE_FALLINGANDRISING_EDGE);

	/* State machine at the initial state */
	uDecodingMachineState 			= PROTOCOL_WAITPREAMBLE;

	protocol_pTagResponse->bIsResPonseExpected = false;
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

/**************************************************************************//**
 * @brief This function interprets the decoded command code. When there are not
 * sufficient symbols to identify the command code, the function
 * returns a specific status and updates the number of symbols to receive.
 * @param[in]  none
 * @param[out] wNbSymbolToWait : the number of symbol to received to identify
 * the current command
 * @param[out] puCommandId : Code of the current command
 * the command code (2 or 4 or 8 or 16)
 * @return CROSSRFID_SUCCESSCODE : the command has been identified
 * @return CROSSRFID_COMMANDDECODING_ERROR : the command could not be identified
 * @return CROSSRFID_WAITMOREBIT_ERROR : some more bit is required to identify
 * the command
 *****************************************************************************/
static uint8_t __attribute__((optimize("O2"))) protocol_interpretCmdCode(uint8_t *pwNbSymbolToWait, uint8_t * puCommandId)
{
	uint8_t status  = CROSSRFID_SUCCESSCODE;

	/* Cmd not found with current NbSymbolToWait */
	if (CROSSRFID_SUCCESSCODE != protocol_GetCommandCode (puCommandId))
	{
		if ((*pwNbSymbolToWait)<PROTOCOL_MAXCOMMAND_SIZE)
		{
			/* try to receive more symbols to identify the cmd */
			(*pwNbSymbolToWait) = (*pwNbSymbolToWait)*2;	/* the min command size is 2 bits, then 4 then 8 and 16*/
			status = CROSSRFID_ERROR_WAITMOREBIT;
		}
		else
		{
			/* Unknown cmd found */
			status = CROSSRFID_ERROR_COMMANDNOTIDENTIFIED;
		}
	}
	return status;
}

/**************************************************************************//**
 * @brief this function decodes the incoming command according to the epc
 * standard
 * @details this function is called as a background task and the irq
 * on GPIO (RX data) fills the array auClockCounterPerSymbol.
 * When the number of symbol reachs the expected value the data
 * are first decoded then interpreted and the number of symbol expected
 * updated. The EPC stadard doesn't define a end of frame for the EPC
 * command. The decoding process decode and counts the number of expected
 * symbols during the command reception.
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
uint8_t __attribute__((optimize("O0"))) protocol_decodeEpcCmd (void)
{
uint8_t status 	= CROSSRFID_ERROR_WAITMOREBIT;
uint8_t uNbSymbolInCRC = 0;
/*uint8_t unthloop = 0;
uint16_t ui16handle=0;*/


	/* if a wrong symbol (> 2*RTcal) has been received*/
	if((PROTOCOL_CHECK_NO_RECETPION_DURING_TIMEOUT != uDecodingMachineState) && (0 != timer1_bElapsed))
	{
		GPIO_PinOutClear (INTERFACE_CLK_PORT,INTERFACE_CLK_PIN);
		if ((GET_EM4325_MISO() == false) && (false == GET_EM4325_AUXSTATE()))	/* when the RX is at low state*/
		{
			status 	= CROSSRFID_WAIT_RFFIELDOFF;
			ui8RTcalFactor = PROTOCOL_DEFAULT_VALUE_FACTOR_RTCAL ;	/* reset the Rtcal factor in order to be able to decode the next query (TrCal > 1 * RtCal) */
		}
#if 0
		else if ((auClockCounterPerSymbol[0] > 170) ) /* first symbol 100 * 1.7 �s*/
		{
			status 	= CROSSRFID_WAIT_RFFIELDOFF;
		}
#endif
		else/* when the RX is at high state*/
		{
			/* === error === */
			status 	= CROSSRFID_ERROR_WRONG_SYMBOL_TIMOUT;
		}
	}

	else
	{
		if(uNbSymbol >= PROTOCOL_MAXNBCMDBIT_SIZE) /* Todo : find the value of PROTOCOL_MAXNBCMDBIT_SIZE */
		{
			status 	= CROSSRFID_ERROR_RX_BUFFER_OVERFLOW;
		}
		else
		{
			switch (uDecodingMachineState)
			{
				/* --------------------------------------------------------------- */
				case PROTOCOL_WAITPREAMBLE:
				/* --------------------------------------------------------------- */
					/* wait to receive the first 4 symbols Tari, RTCal & may be TrCal (Tested in the protocol_decodePreamble function) */
					if (uNbSymbol >= (ui8NbBitFirstSymbols+1))
					{
						uDecodingMachineState = PROTOCOL_DECODEPREAMBLE;
					}
					else
					{
						/* nothing to do*/
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_DECODEPREAMBLE:
				/* --------------------------------------------------------------- */
					status = protocol_decodePreamble ();
					if (CROSSRFID_SUCCESSCODE == status)
					{
						status = CROSSRFID_ERROR_WAITMOREBIT; /* in order to continue the state machine */
						/* starts to compute the CRC */
						CRC16_init_crc16_computation();
						uDecodingMachineState = PROTOCOL_WAITCOMMANDCODE;
					}
					else
					{
						/* === error === */
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_WAITCOMMANDCODE:
				/* --------------------------------------------------------------- */
					/* wait to receive the number of symbol corresponding to a command code */
					if (uNbSymbol >(ui8NbBitFirstSymbols + uNbSymbolToWait-1))
					{
						uDecodingMachineState = PROTOCOL_DECODECOMMANDCODE;
					}
					else {/*wait more data*/}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_DECODECOMMANDCODE:
				/* --------------------------------------------------------------- */
					/* decode the previous received symbols */
					protocol_decodeEPCsymbols();
					status = protocol_interpretCmdCode(&uNbSymbolToWait,&uCommandId); /* get the command id */
					if (CROSSRFID_SUCCESSCODE == status) /* when the command has been identified*/
					{
						status = CROSSRFID_ERROR_WAITMOREBIT; /* in order to continue the state machine */
						protocol_GetNbParameterSymbol (uCommandId,1,0, &uNbSymbolToWait);	/* get the number of parameter to wait */
						protocol_pTagResponse->bIsResPonseExpected = protocol_IsReponseExpected (uCommandId);
						if (true == protocol_pTagResponse->bIsResPonseExpected)	/* when a reply is expected */
						{
							protocol_pTagResponse->bIsImmediateReplyExpected = protocol_IsImmediateReplyExpected (uCommandId);
							timer0_bElapsed = false;
							TIMER0->CMD = TIMER_CMD_START;							/* the timer 0 should be starterd to compute the EPC's T1*/
							GPIO_PinOutSet (INTERFACE_CLK_PORT,INTERFACE_CLK_PIN); 	/* the EM4325's CLK should be set to have the BLF clock*/

						} else {/* do nothing*/}
						uDecodingMachineState = PROTOCOL_WAITPARAMETER;
					}
					else if (status == CROSSRFID_ERROR_WAITMOREBIT)
					{
						uDecodingMachineState = PROTOCOL_WAITCOMMANDCODE;
					}
					else if (status == CROSSRFID_ERROR_COMMANDNOTIDENTIFIED)
					{
						/* === error === */
					}
					else
					{
						/* do nothing*/
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_WAITPARAMETER:
				/* --------------------------------------------------------------- */
					if (uNbBitDecodedinCmd > (uNbSymbolToWait-1))
					{
						uDecodingMachineState = PROTOCOL_DECODEPARAMAETER;
					}
					else /*wait more data*/
					{
						/* decode the previous received symbols */
						protocol_decodeEPCsymbols();
						uDecodingMachineState = PROTOCOL_WAITPARAMETER;
					}

					if(uNbSymbol >= PROTOCOL_MAXNBCMDBIT_SIZE)
					{
						/* === error === */
						status = CROSSRFID_ERROR_OVERFLOW_RX_CMD;
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_DECODEPARAMAETER:
				/* --------------------------------------------------------------- */
					status = protocol_UpdateNbParameterSymbol(uCommandId,&uNbSymbolToWait);
					if (CROSSRFID_SUCCESSCODE==status)
					{
						protocol_GetNbCRCSymbol (uCommandId,&uNbSymbolInCRC);
						uNbSymbolToWait += uNbSymbolInCRC;
						status = CROSSRFID_ERROR_WAITMOREBIT; /* in order to continue the state machine */
						if(0 == uNbSymbolInCRC)
						{
							/* In the case where the command doesn't have crc */
							uDecodingMachineState = PROTOCOL_CHECK_NO_RECETPION_DURING_TIMEOUT;
						}
						else
						{
#if 0							/*retrieve the handle */

							if ( CROSSRFID_SUCCESSCODE == protocol_GetHandle (uCommandId,&ui16handle))
							{
								protocol_pTagResponse->bIsHandleFound = true;
								protocol_pTagResponse->ui16Handle = ui16handle ;
							} else {/*do nothing*/}

							protocol_appendTagResponse (uCommandId,protocol_pTagResponse);
#endif
							/* In the case where the command has a crc */
							uDecodingMachineState = PROTOCOL_WAITCRC;
						}
					}
					else if (CROSSRFID_ERROR_WAITMOREBIT==status)
					{
						uDecodingMachineState = PROTOCOL_WAITPARAMETER;
					}
					else
					{
						/* === error === */
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_CHECK_NO_RECETPION_DURING_TIMEOUT:
				/* --------------------------------------------------------------- */
					/*
					 * Command without CRC:
					 * No symbols has to be received during the previous timeout
					 * implemented (= T1 because the command has been found,
					 * see protocol_enableEpcTimeout1()).
					 */

					/* check if no symbol received */
					if(uNbBitDecodedinCmd == (uNbSymbol-ui8NbBitFirstSymbols+1))
					{
						/* === error === */
						status = CROSSRFID_ERROR_COMMANDNOTIDENTIFIED;
					}
					else
					{
						/* time out occurs */
						if(timer1_bElapsed != 0)
						{
							/* todo: method to avoid the overflow (wrong symbol) during RTCAL after QUERY */
							if(PROTOCOL_MIN_VALUE_FACTOR_RTCAL == ui8RTcalFactor)
							{
								protocol_preamble.uRtCal += 10;
							}
							/* saves the previous valid RTcal value */
							ui8PreviousRTcal = protocol_preamble.uRtCal;

							/* --------------------------*/
							/* ----- Command VALID  -----*/
							status = CROSSRFID_SUCCESSCODE;
							/* --------------------------*/
							/* --------------------------*/
						}
						else
						{
							/* wait timeout */
							status = CROSSRFID_WAIT_TIMEOUT;
						}
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_WAITCRC:
				/* --------------------------------------------------------------- */
					if (uNbBitDecodedinCmd > (uNbSymbolToWait-1))
					{
						uDecodingMachineState = PROTOCOL_DECODECRC;
						if ( PROTOCOL_EPC_CMDID_QUERY == uCommandId ) // TODO : just for the QUERY, CRC5 to check
						{
							/* after query, the timeout is define by 1*rtcal for the detection of wrong symbols*/
							ui8RTcalFactor = PROTOCOL_MIN_VALUE_FACTOR_RTCAL;
							/* saves the previous valid RTcal value */
							ui8PreviousRTcal = protocol_preamble.uRtCal + 10;
							/* save the used parameters fo the response encoding*/
							protocol_pTagResponse->bIsPilotTone = uCommandParamFields.sQuery.pTRext;
							protocol_pTagResponse->protocol_encoding = (uint8_t) (*(uCommandParamFields.sQuery.pM)<<1);
							protocol_pTagResponse->protocol_encoding += (uint8_t) (*((uCommandParamFields.sQuery.pM)+1));
							protocol_preamble.uTrCal = auClockCounterPerSymbol[3]; /* save the TrCal value that is just available in the query command*/
							protocol_pTagResponse->uTpri = 10.0 * protocol_preamble.uTrCal / PROTOCOL_CONVERT_DR( uCommandParamFields.sQuery.pDr);
							protocol_enableEpcTimeout1();  /* this time out will occur after the end of the reception (cpt reset on miso rising edge) */
							status = CROSSRFID_SUCCESSCODE;
						}
						else {/* Do nothing*/}

					}
					else /*wait more data*/
					{
						/* decode the previous received symbols */
						protocol_decodeEPCsymbols();
						uDecodingMachineState = PROTOCOL_WAITCRC;
					}


					if(uNbSymbol >= PROTOCOL_MAXNBCMDBIT_SIZE)
					{
						/* === error === */
						status = CROSSRFID_ERROR_OVERFLOW_RX_CMD;
					}
				break;

				/* --------------------------------------------------------------- */
				case PROTOCOL_DECODECRC:
				/* --------------------------------------------------------------- */

				if(true == CRC16_IsRightResidue ())
				{
					/* todo: method to avoid the overflow (wrong symbol) during RTCAL after QUERY */
					if(PROTOCOL_MIN_VALUE_FACTOR_RTCAL == ui8RTcalFactor)
					{
						protocol_preamble.uRtCal += 10;
					}

					/* saves the previous valid RTcal value */
					ui8PreviousRTcal = protocol_preamble.uRtCal;

					/* --------------------------*/
					/* ----- Command VALID  -----*/
					status = CROSSRFID_SUCCESSCODE;
					uDecodingMachineState = PROTOCOL_WAITT1;
					NVIC_DisableIRQ(GPIO_ODD_IRQn);		/* disable the Irq of the GPIO. activate again in the reset function*/
					if (true == protocol_pTagResponse->bIsResPonseExpected)
					{
						NVIC_DisableIRQ(TIMER1_IRQn);		/* disable the timer 1 when the command is correct and when a response is expected*/
					} else {/*do nothing*/}
					/* --------------------------*/
					/* --------------------------*/
					}
					else
					{
						/* === error === */
						status = CROSSRFID_ERROR_CRC16_RX_CMD;
					}
				break;

				/* --------------------------------------------------------------- */
				default:
				/* --------------------------------------------------------------- */
					status = CROSSRFID_SUCCESSCODE; /* end of the decoding */
					//protocol_initDecodingmachine();
					protocol_resetDecodingmachine();
				break;
			}
		}
	}
	return status;
}


