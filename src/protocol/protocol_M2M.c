/*******************************************************************************
 * @file protocol_M2M.c
 * @brief this file is the service layer for the M2M communication
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_M2M.h"

#if USEM2MINTERFACE == 1 /* these IRQ handlers is dedicated to the M2M communication*/
/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
static 	uint8_t *pui8_SystemFile =  NULL;  /* pointer on the System file structure*/
static 	uint8_t *pui8_ExternalMemory =  NULL;  /* pointer on the System file structure*/

//static prtM2M_RespFirstWord_union prtm2m_uRespFirstWord;
//static kernel_DataExchange_Type prtM2M_SerialCommand;

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
/*===========================================================================================================
						Private function declaration
===========================================================================================================*/
#if 0
static void 	prtm2m_ProcessReadMessage 			( uint8_t * const pui8Rxdata );
static void 	prtm2m_ProcessWriteMessage 			( uint8_t * const pui8Rxdata );
static void 	prtm2m_ProcessReadSystemFile 		( const uint8_t ui8SystemFileAddress , const uint8_t ui8NbWord );
static void 	prtm2m_ProcessReadExternalMemmory	( uint8_t * const pui8Rxdata );

static uint32_t prtm2m_hexstr2int					( const char* const hex_str, size_t hex_str_length);
static void 	prtm2m_Hex2ascii(const uint8_t* const pui8HexArray, uint8_t ui8Nbelement , uint8_t *AsciiString);
#endif
//static void 	prtm2m_GetSuccessSuccesCode 		( const uint8_t ui8Register, const uint8_t ui8Nbword , uint16_t *ui16SuccessCode);
static uint8_t 	prtm2m_ParseRxMessage 				( uint8_t * const pui8Rxdata, kernel_DataExchange_Type *psdataobject );

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
#if 0

/***************************************************************************//**
 * @brief 		This function process to the reception of the data
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
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void prtm2m_ProcessWriteMessage ( uint8_t * const pui8Rxdata )
{


}



/***************************************************************************//**
 * @brief 		This function process to the read command
 * @note		the frame format is the next one
 * 	|---------------|---------------|-----------|-----------------------|
 * 	|	1st byte 	| 2nd byte		| 3rd byte	|	4th byte			|
 * 	|---------------|---------------|-----------|-----------------------|
 * 	| header code	| R/W & address	| address	|	address & NbWord	|
  	|---------------|---------------|-----------|-----------------------|
 * 	|0xF : System	| 0bX0XX : Read | 			|0baXXX:address			|
 * 	|file access	| 0bX1XX : Write| 			|0baWWW : nb word 		|
 * 	|				| 0bXXaa:address| 			| ( 1 word = 2 bytes)	|
 * 	|---------------|---------------|-----------|-----------------------|
 * @param[in] 	pui8Rxdata : pointer of the RX message
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void prtm2m_ProcessReadSystemFile ( const uint8_t ui8SystemFileAddress , const uint8_t ui8NbWord )
{
uint8_t pcAsciiString[(ui8NbWord+1)*4 +1 ];	/* each word requires 4 characters to be coded (by e.g. 0xD0D0) */
uint16_t ui16SuccessCode = 0;

	prtm2m_GetSuccessSuccesCode (ui8SystemFileAddress,ui8NbWord,&ui16SuccessCode);
	prtm2m_Hex2ascii ((uint8_t *) &ui16SuccessCode,2,pcAsciiString);	/* add the success code*/
	/* send the response to the M2M host */
	prtm2m_Hex2ascii ( pui8_SystemFile + ui8SystemFileAddress,(uint32_t) (ui8NbWord*2),&(pcAsciiString[4])); /* append the response*/
	intuart_PutData ( pcAsciiString, (ui8NbWord+1)*4  );
}


/**************************************************************************//**
 * @brief 		This function process to the read command
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
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void prtm2m_ProcessReadExternalMemmory ( uint8_t * const pui8Rxdata )
{
uint32_t ui32NbByte = 0 ;

	ui32NbByte = prtm2m_hexstr2int((const char* const) &pui8Rxdata [PRTM2M_COMMANDFORMAT_DATA],2);
	/* send the response to the M2M host */
	intuart_PutData ( pui8_ExternalMemory, ui32NbByte);
}
#endif

#if 0
/**************************************************************************//**
 * @brief converts the given hex string of size hex_str_length to a uint32_t.
 * 			e.g.
 * 			hex_str_to_int("ABCD", 4) -> 0xABCD
 * 			hex_str_to_int("ABCD", 2) -> 0xAB
 * @param[in] 	hex_str string to convert
 * @param[in] 	hex_str_length length of the hex string we want to convert to int.
 * 			Useful for limiting sscanf application scope.
 *			hex_str_length must be smaller than 4 so the result fits in a uint32_t
 * @param[out] 	none
 * @return
 *****************************************************************************/
static uint32_t prtm2m_hexstr2int(const char* const hex_str, size_t hex_str_length)
{
	uint32_t result;
	char tmp_str[8+1]; // don't forget the end NULL char

	if ((hex_str_length * 4) > 32)
	{
		return 0;
	}
	memcpy(tmp_str, hex_str, hex_str_length);
	tmp_str[hex_str_length] = 0;
	sscanf(tmp_str, "%x", (int*)&result);
	return result;
}
#endif

/**************************************************************************//**
* @brief converts the given hex array to an ASCII string
* @param[in] 	pui8HexArray : array of hexadecimal element to convert
* @param[in] 	ui8Nbelement : number of element of the pui8HexArray
* @param[out] 	AsciiString : the ASCII string
* @return	none
 *****************************************************************************/
void prtm2m_Hex2ascii(const uint8_t* const pui8HexArray, uint8_t ui8Nbelement , uint8_t *AsciiString)
{
	char aAscii [16] = 	{'0','1','2','3','4','5','6','7', '8','9','A','B','C','D','E','F'};
	uint8_t ui8NthHextab=0;
	uint8_t ui8NthString=0;

	for (ui8NthHextab=ui8Nbelement;ui8NthHextab>0;ui8NthHextab--)
	{
		AsciiString[ui8NthString++] = aAscii [(pui8HexArray[ui8NthHextab-1]&0xF0)>>4];
		AsciiString[ui8NthString++] = aAscii [pui8HexArray[ui8NthHextab-1]&0x0F];
	}

	AsciiString[ui8NthString] ='\r';

}

/**************************************************************************//**
* @brief converts the given hex array to an ASCII string
* @param[in] 	pui8HexArray : array of hexadecimal element to convert
* @param[in] 	ui8Nbelement : number of element of the pui8HexArray
* @param[out] 	AsciiString : the ASCII string
* @return	none
 *****************************************************************************/
void prtm2m_i16ToStr (int16_t bin, unsigned char ui8Nbelement, uint8_t *AsciiString)
{

	if (bin < 0)
    {
		bin *= -1;
		//IsNegative = true;
		*(AsciiString) = '-';
    }
	else
	{
		*(AsciiString) = ' ';
	}


	AsciiString += ui8Nbelement+1;
    *AsciiString = '\0';

	while (ui8Nbelement--)
	{
		*--AsciiString = (bin % 10) + '0';
		bin /= 10;
	}

}


/***************************************************************************//**
 * @brief 		This function parses the received command from the serail interface
 * @note		the frame format is the next one
 * 	|---------------|---------------|-----------------------|
 * 	| Command code	|  object		| data (optional)		|
 * 	|				|				|						|
 * 	|---------------|---------------|-----------------------|
 * 	| set			| 	accel		|		on				|
 * 	| set			| 	accel		|		off				|
 * 	| get			| 	accel		| 		state			|
 * 	|---------------|---------------|-----------------------|
 * @param[in] 	pui8Rxdata : pointer of the RX message
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the command has been successful parsed
 * @return 		CROSSRFID_ERROR_SERIAL_WRONGCOMMANDID : the command has been recognized
 * @return 		CROSSRFID_ERROR_SERIAL_WRONGOBJECTID : the object has been recognized
 * @return 		CROSSRFID_ERROR_SERIAL_WRONGACTIONID : the action has been recognized
 ******************************************************************************/
static uint8_t prtm2m_ParseRxMessage ( uint8_t * const pui8Rxdata, kernel_DataExchange_Type *psdataobject )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
uint8_t * pui8ObjectId = &(pui8Rxdata[4]); /* the command is set or get so the next field is here*/
uint8_t * pui8OperationId ;


	/*get the command code Id */
	if (!memcmp(pui8Rxdata,"get ",3))
	{
		psdataobject->ui8CommandId = KERNEL_COMMANDCODE_GET;
	}
	else if (!memcmp(pui8Rxdata,"set ",3))
	{
		psdataobject->ui8CommandId = KERNEL_COMMANDCODE_SET;
	}
	else
	{
		psdataobject->ui8CommandId = KERNEL_COMMANDCODE_UNKNOWN;
		ui8status = CROSSRFID_ERROR_SERIAL_WRONGCOMMANDID;
	}

	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* retrieve the object id */
		if (!memcmp(pui8ObjectId,"accel",strlen("accel")))
		{
			psdataobject->ui8ObjectId = KERNEL_OBJECTCODE_ACCELEROMETER;

			/* retrieve the action id */
			pui8OperationId = pui8ObjectId + strlen("accel") +1;
			if (!memcmp(pui8OperationId,"on",strlen("on")))
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_ON;
			}
			else if (!memcmp(pui8OperationId,"off",strlen("off")))
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_OFF;
			}
			else if (!memcmp(pui8OperationId,"state",strlen("state")))
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_STATE;
			}
			else
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_UNKNOWN;
				ui8status = CROSSRFID_ERROR_SERIAL_WRONGACTIONID;
			}

		}
		/* retrieve the object id */
		else if (!memcmp(pui8ObjectId,"actcount",strlen("actcount")))
		{
			psdataobject->ui8ObjectId = KERNEL_OBJECTCODE_ACTIVITYCOUNTER;
			/* retrieve the operation id */
			pui8OperationId = pui8ObjectId + strlen("actcount") +1;
			if (!memcmp(pui8OperationId,"on",strlen("on")))
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_ON;
			}
			else if (!memcmp(pui8OperationId,"off",strlen("off")))
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_OFF;
			}
			else if (!memcmp(pui8OperationId,"state",strlen("state")))
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_STATE;
			}
			else
			{
				psdataobject->ui8ActionId = KERNEL_ACTIONCODE_UNKNOWN;
				ui8status = CROSSRFID_ERROR_SERIAL_WRONGACTIONID;
			}


		}
		else
		{
			psdataobject->ui8CommandId = KERNEL_COMMANDCODE_UNKNOWN;
			ui8status = CROSSRFID_ERROR_SERIAL_WRONGOBJECTID;
		}
	}

	return ui8status;
}



#if 0
/**************************************************************************//**
 * @brief 		This function writes a success code in the register file
 * @param[in]  	ui8Register : system file address
 * @param[in]  	ui8Nbword : number of word following in the register file
 * @param[out] 	ui16SuccessCode : the success code to be sent
 * @return 		none
 *****************************************************************************/
static void prtm2m_GetSuccessSuccesCode ( const uint8_t ui8Register, const uint8_t ui8Nbword , uint16_t *ui16SuccessCode)
{
	/* Response: Writes the first word */
	prtm2m_uRespFirstWord.sBitsField.ui16Handshake = PRTM2M_HANDSHAKE_CROSSTORFHOST;
	prtm2m_uRespFirstWord.sBitsField.ui16RegisterAddress = ui8Register;
	prtm2m_uRespFirstWord.sBitsField.ui16Status = PRTM2M_REGFILE_STATUSOK;
	prtm2m_uRespFirstWord.sBitsField.ui16NbWordsInParams = ui8Nbword;
	prtm2m_uRespFirstWord.sBitsField.ui16Header = 0;

	(*ui16SuccessCode) = prtm2m_uRespFirstWord.ui16Value;
}
#endif
/*===========================================================================================================
						Public functions definition
===========================================================================================================*/

/***************************************************************************//**
 * @brief 		This function checks the command code and the me
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
bool prtm2m_IsRightCommandcode (  uint8_t * const pui8RxData )
{
bool	status = true;

	if ( (!memcmp(pui8RxData,"set",3 )) || (!memcmp(pui8RxData,"get",3 )))
	//if ( PRTM2M_ACCESSSYSTEMFILE != ui8HeaderCode )
	{
		status = false;
	}


	return status;
}






/***************************************************************************//**
 * @brief 		This function initializes the M2M interface
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void prtm2m_init ( void )
{
	pui8_SystemFile 	=  ( uint8_t * ) srvEM4325_psSysFile; 				/* initialize the pointer on the system file*/
	pui8_ExternalMemory =  ( uint8_t * ) srv24LC_MemoryContent.version; 	/* initialize the pointer on the external memory*/ /* TODO : replace by an array of pointer*/

	intleuart_init ();

}


/***************************************************************************//**
 * @brief 		This function process to the reception of the data
 * @note		the frame format is the next one
 * 	|---------------|---------------|-----------------------|
 * 	| Command code	|  object		| data (optional)		|
 * 	|				|				|						|
 * 	|---------------|---------------|-----------------------|
 * 	| set			| 	accel		|		on				|
 * 	| set			| 	accel		|		off				|
 * 	| get			| 	accel		| 						|
 * 	|---------------|---------------|-----------------------|
 * @param[in] 	pui8Rxdata : pointer of the RX message
 * @param[out] 	prtM2M_SerialCommand : the Id of the different fields of the serial command
 * @return 		CROSSRFID_SUCCESSCODE : the command has been successful parsed
 * @return 		CROSSRFID_ERROR_SERIAL_WRONGCOMMANDID : the command has been recognized
 * @return 		CROSSRFID_ERROR_SERIAL_WRONGOBJECTID : the object has been recognized
 * @return 		CROSSRFID_ERROR_SERIAL_WRONGACTIONID : the action has been recognized
 ******************************************************************************/
uint8_t prtm2m_ProcessRxMessage ( uint8_t * const pui8Rxdata, kernel_DataExchange_Type *psdataobject )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	ui8status = prtm2m_ParseRxMessage (pui8Rxdata, psdataobject );

	return ui8status;
}


/***************************************************************************//**
 * @brief		helper wrapper that performs the strlen before calling uartPutData()
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	str, null terminated string
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void prtm2m_SendString(const char* const str)
{
	intuart_PutString( str );
}

#endif

