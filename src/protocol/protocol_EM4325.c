/*******************************************************************************
 * @file protocol_EM4325.c
 * @brief this function set is codec for the EM4325 device
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_EM4325.h"

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
/* buffer for the SPI RX & TX frames dedicated to the EM4325 */
static uint8_t 							protocol_BufferEM4325 [PROTOCOL_EM4325_SPIBUFFERSIZE] ;

/*===========================================================================================================
						Private functions declaration
===========================================================================================================*/
static void protocol_UpdateEmBufBeforeSendSpiCmd(const uint8_t ui8IdCmd, uint8_t const * pui8Param1, uint8_t const * pui8Param2, uint8_t * pui8NbByteInCmdResp);
#if WARNING_v120_NOT_USED
static void protocol_ProcessSpiRequestStatusResponse ( const uint8_t pReplyStatus ,
												bool *bTransponderState ,
												uint8_t *eDeviceState,
												bool *bMemoryState ,
												uint8_t *eCommandState );
#endif

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
#if WARNING_v120_NOT_USED
/**************************************************************************//**
 * @brief this function parses the status byte of the response em4325
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
 * @param[out] eCommandState status of the command
 * 0b00 : Command executed
 * 0b01 : invalid command
 * 0b10 : command failed
 * 0b10 : command failed
 * @return none
 *****************************************************************************/
static void protocol_ProcessSpiRequestStatusResponse ( const uint8_t pReplyStatus ,
												bool *bTransponderState ,
												uint8_t *eDeviceState,
												bool *bMemoryState ,
												uint8_t *eCommandState )
{

	(*bTransponderState) = (pReplyStatus & PROTOCOL_EM4325_TRANSPONDERSTATE_MASK) 	>> PROTOCOL_EM4325_TRANSPONDERSTATE_SHIFT;
	(*eDeviceState) 	 = (pReplyStatus & PROTOCOL_EM4325_DEVICESTATESTATE_MASK) 	>> PROTOCOL_EM4325_DEVICESTATESTATE_SHIFT;
	(*bMemoryState) 	 = (pReplyStatus & PROTOCOL_EM4325_MEMORYSTATE_MASK) 		>> PROTOCOL_EM4325_MEMORYSTATE_SHIFT;
	(*eCommandState) 	 = (pReplyStatus & PROTOCOL_EM4325_COMMMANDSTATE_MASK) 		>> PROTOCOL_EM4325_COMMMANDSTATE_SHIFT;

}
#endif

/**************************************************************************//**
 * @brief this function update the parameters and the Tx buffer
 * before sending a SPI command to EM4325.
 *
 * @param[in] ui8IdCmd: The id of the SPI command to send
 * @param[in] pui8Param1: Parameter 1 according to the command
 * @param[in] pui8Param2: Parameter 2 according to the command
 * @param[out] pui8NbByteInCmdResp[0]: Nb of byte in the command
 * @param[out] pui8NbByteInCmdResp[1]: Nb of byte in the response
 *
 * @return none
 *****************************************************************************/
static void protocol_UpdateEmBufBeforeSendSpiCmd(const uint8_t ui8IdCmd, uint8_t const * pui8Param1, uint8_t const * pui8Param2, uint8_t * pui8NbByteInCmdResp)
{
	switch(ui8IdCmd)
	{
		case PROTOCOL_EM4325_SPI_CMD_REQUESTSTATUS:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_REQUESTSTATUS;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_REQUESTSTATUS;
			break;

			/* TODO : these cmds */

		case PROTOCOL_EM4325_SPI_CMD_BOOT:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_BOOT;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_BOOT;
			break;
#if 0
		case PROTOCOL_EM4325_SPI_CMD_ENABLETRANSPONDER:
			*pui8NbByteInResponse = ;
			*pui8NbByteInCommand = ;
			break;

		case PROTOCOL_EM4325_SPI_CMD_DISABLETRANSPONDER:
			*pui8NbByteInResponse = ;
			*pui8NbByteInCommand = ;
			break;

		case PROTOCOL_EM4325_SPI_CMD_READPAGE:
			*pui8NbByteInResponse = PROTOCOL_EM4325_SPI_RES_NBBYTE_READPAGE;
			*pui8NbByteInCommand = PROTOCOL_EM4325_SPI_CMD_NBBYTE_READPAGE;
			/* in (7 bits): page number */
			protocol_BufferEM4325 [1] = *pui8Param1;
			/* out (64 bits): page */
			break;
#endif
		case PROTOCOL_EM4325_SPI_CMD_GETNEWSENSORDATA:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_GETNEWSENSORDATA;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_GETNEWSENSORDATA;
			/* in (): */
			/* Nothing */
			break;
#if 0
		case PROTOCOL_EM4325_SPI_CMD_SETFLAGS:
			*pui8NbByteInResponse = ;
			*pui8NbByteInCommand = ;
			break;
#endif

		case PROTOCOL_EM4325_SPI_CMD_READWORD:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_READWORD;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_READWORD;
			/* in (7 bits): Address */
			protocol_BufferEM4325[1] = *pui8Param1;
			/* out (16 bits): Word */
			break;

		case PROTOCOL_EM4325_SPI_CMD_WRITEWORD:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_WRITEWORD;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_WRITEWORD;
			/* in (7 bits): Address */
			protocol_BufferEM4325 [1] = *pui8Param1;
			/* in (16 bits): Data */
			protocol_BufferEM4325 [3] = *pui8Param2; /* inversion due to a cast uint16* to uint8* */
			pui8Param2++;
			protocol_BufferEM4325 [2] = *pui8Param2;
			break;

		case PROTOCOL_EM4325_SPI_CMD_READREGFILEWORD:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_READREGFILEWORD;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_READREGFILEWORD;
			/* in (3 bits) Register File Word */
			protocol_BufferEM4325 [1] = *pui8Param1;
			/* out (16 bits): Word */
			break;

		case PROTOCOL_EM4325_SPI_CMD_WRITEREGFILEWORD:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_WRITEREGFILEWORD;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_WRITEREGFILEWORD;
			/* in (3 bits) Register File Word */
			protocol_BufferEM4325 [1] = *pui8Param1;
			/* in (16 bits): Data */
			protocol_BufferEM4325 [3] = *pui8Param2; /* inversion due to a cast uint16* to uint8* */
			pui8Param2++;
			protocol_BufferEM4325 [2] = *pui8Param2;
			break;
#if 0
		case PROTOCOL_EM4325_SPI_CMD_REQRN:
			*pui8NbByteInResponse = PROTOCOL_EM4325_SPI_RES_NBBYTE_REQRN;
			*pui8NbByteInCommand = PROTOCOL_EM4325_SPI_CMD_NBBYTE_REQRN;
			/* out 16 : Random number */
			break;

		case PROTOCOL_EM4325_SPI_CMD_REQNEWHANDLE:
			*pui8NbByteInResponse = PROTOCOL_EM4325_SPI_RES_NBBYTE_REQNEWHANDLE;
			*pui8NbByteInCommand = PROTOCOL_EM4325_SPI_CMD_NBBYTE_REQNEWHANDLE;
			/* in 8 : dummy */
			protocol_BufferEM4325 [1] = (uint8_t)0;
			/* out 16 : new handle*/
			/* out 8 : backscatter settings */
			/* out 16 RN16 or handle */
			break;


		case PROTOCOL_EM4325_SPI_CMD_SETHANDLE:
			*pui8NbByteInResponse = PROTOCOL_EM4325_SPI_RES_NBBYTE_SETHANDLE;
			*pui8NbByteInCommand = PROTOCOL_EM4325_SPI_CMD_NBBYTE_SETHANDLE;
			/* in 16 : handle */
			protocol_BufferEM4325 [1] = *pui8Param1;
			/* in 8 : dummy */
			protocol_BufferEM4325 [2] = (uint8_t)0;
			break;
#endif
		case PROTOCOL_EM4325_SPI_CMD_SETCOMMPARAMS:
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_CMD_NBBYTE_SETCOMMPARAMS;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = PROTOCOL_EM4325_SPI_RES_NBBYTE_SETCOMMPARAMS;
			/* in 16 : */
			protocol_BufferEM4325 [2] = *pui8Param1;
			pui8Param1++;
			protocol_BufferEM4325 [1] = *pui8Param1;
			break;
#if 0
		case PROTOCOL_EM4325_SPI_CMD_GETCOMMPARAMS:
			*pui8NbByteInResponse = PROTOCOL_EM4325_SPI_RES_NBBYTE_GETCOMMPARAMS;
			*pui8NbByteInCommand = PROTOCOL_EM4325_SPI_CMD_NBBYTE_GETCOMMPARAMS;
			/* in 8 : dummy */
			protocol_BufferEM4325 [1] = (uint8_t)0;
			/* out 8 : Backscatter Settings */
			/* out 8 : Flag Settings */
			/* out 16 : Tag Handle */
			break;
#endif
		default:
			*pui8NbByteInCmdResp = 0;
			pui8NbByteInCmdResp++;
			*pui8NbByteInCmdResp = 0;
			break;
	}
}
/*===========================================================================================================
						Public functions definition
===========================================================================================================*/



/**************************************************************************//**
 * @brief 		this functions setups a USART as SPI interface
 * @param[in] 	bIsMaster true when the mcu is the master, false for the slave
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
void prtEM4325_InitSpiBus ( const bool bIsMaster  )
{
	interface_InitSPI (bIsMaster);
}
/**************************************************************************//**
 * @brief this function sends a SPI command to EM4325.
 *
 * @param[in] ui8IdCmd: The id of the Spi command to send
 * @param[in] pui8Param1: Parameter 1 according to the command
 * @param[in] pui8Param2: Parameter 1 according to the command
 *
 * @return none
 *****************************************************************************/
void protocol_SendSpiCommand (const uint8_t ui8IdCmd, uint16_t const * pui16Param1, uint16_t const * pui16Param2)
{
uint8_t tui8NbByteInCmdResp[2]; /* [0]: nb Byte in command */
								/* [1]: nb Byte in response */


USART_TypeDef *spi = INTERFACE_USART;

	/* fill the TxBuffer*/
	protocol_BufferEM4325 [0] 	= ui8IdCmd;

	protocol_UpdateEmBufBeforeSendSpiCmd(ui8IdCmd,(uint8_t*)pui16Param1,(uint8_t*)pui16Param2,&tui8NbByteInCmdResp[0]);

	/* activate the CS*/
	GPIO_PinOutClear (INTERFACE_CS_PORT, INTERFACE_CS_PIN);

	/* Starts the SPI reception */
	interface_StartSPIRx(&protocol_BufferEM4325[0],tui8NbByteInCmdResp[1]);

	/* send the command*/
	interface_SendSpiBuffer (protocol_BufferEM4325,tui8NbByteInCmdResp[0]);

	/* wait the first response byte according to the cmd id */
	if(PROTOCOL_EM4325_SPI_CMD_WRITEWORD == ui8IdCmd)
	{
		interface_WaitEm4325SpiResponse(0xFF, 8); /* 8 ms */
	}
	else if(PROTOCOL_EM4325_SPI_CMD_GETNEWSENSORDATA == ui8IdCmd)
	{
		interface_WaitEm4325SpiResponse(0xFF, 20); /* 20 ms before receiving a new sample */
	}
	else
	{
		interface_WaitResponseReadySignal(0xFF);
	}

	/* send dummy bytes in order to get the data*/
	interface_ClearRx();

	interface_SendSpiBuffer (NULL,tui8NbByteInCmdResp[1] );

	USART_IntDisable(spi,USART_IEN_RXDATAV);

	/* Disable the CS*/
	GPIO_PinOutSet (INTERFACE_CS_PORT, INTERFACE_CS_PIN);
}

/**************************************************************************//**
 * @brief this function update the parameters and the Tx buffer
 * before sending a SPI command to EM4325.
 *
 * @param[in] ui8IdCmd: The id of the Spi command to send
 * @param[out] pui16Param1: out parameter 1
 * @param[out] pui16Param3: out parameter 2
 * @param[out] pui16Param2: out parameter 3
 *
 * @return none
 *****************************************************************************/
void protocol_ReadSpiResponse (uint8_t ui8IdCmd, uint16_t * pui16Param1, uint16_t * pui16Param2, uint16_t * pui16Param3)
{
	switch(ui8IdCmd)
	{
		case PROTOCOL_EM4325_SPI_CMD_REQUESTSTATUS:
			break;

			/* TODO : these cmds */
#if 0
		case PROTOCOL_EM4325_SPI_CMD_BOOT:
			break;

		case PROTOCOL_EM4325_SPI_CMD_ENABLETRANSPONDER:
			break;

		case PROTOCOL_EM4325_SPI_CMD_DISABLETRANSPONDER:
			break;

		case PROTOCOL_EM4325_SPI_CMD_READPAGE:
			/* out (64 bits): page */
			//*pui16Param1 = ((uint16_t)(protocol_BufferEM4325[1]) << 8) | protocol_BufferEM4325[2];
			break;
#endif
		case PROTOCOL_EM4325_SPI_CMD_GETNEWSENSORDATA:
			/* out (16 bits): Sensor Data MSW */
			*pui16Param1 = ((uint16_t)(protocol_BufferEM4325[1]) << 8) | protocol_BufferEM4325[2];
			/* out (16 bits): Sensor Data LSW */
			*pui16Param2 = ((uint16_t)(protocol_BufferEM4325[3]) << 8) | protocol_BufferEM4325[4];
			/* out (16 bits): UTC Time Stamp MSW */
			*pui16Param3 = ((uint16_t)(protocol_BufferEM4325[5]) << 8) | protocol_BufferEM4325[6];
			/* note: UTC Time Stamp LSW not read */
			break;
#if 0
		case PROTOCOL_EM4325_SPI_CMD_SETFLAGS:
			break;
#endif
		case PROTOCOL_EM4325_SPI_CMD_READWORD:
			/* out (16 bits): Word */
			*pui16Param1 = ((uint16_t)(protocol_BufferEM4325[1]) << 8) | protocol_BufferEM4325[2];
			break;

		case PROTOCOL_EM4325_SPI_CMD_READREGFILEWORD:
			/* out (16 bits): Word */
			*pui16Param1 = ((uint16_t)(protocol_BufferEM4325[1]) << 8) | protocol_BufferEM4325[2];
			break;
#if 0
		case PROTOCOL_EM4325_SPI_CMD_REQRN:
			/* out 16 : Random number */
			*pui16Param1 = ((uint16_t)(protocol_BufferEM4325[1]) << 8) | protocol_BufferEM4325[2];
			break;

		case PROTOCOL_EM4325_SPI_CMD_REQNEWHANDLE:
			/* out 16 : new handle*/
			*pui16Param1 = ((uint16_t)(protocol_BufferEM4325[1]) << 8) | protocol_BufferEM4325[2];
			/* out 8 : backscatter settings */
			*pui16Param2 = protocol_BufferEM4325[3];
			/* out 16 RN16 or handle */
			*pui16Param3 = ((uint16_t)(protocol_BufferEM4325[4]) << 8) | protocol_BufferEM4325[5];
			break;

		case PROTOCOL_EM4325_SPI_CMD_GETCOMMPARAMS:
			/* out 8 : Backscatter Settings */
			*pui16Param2 = protocol_BufferEM4325[1];
			/* out 8 : Flag Settings */
			*pui16Param2 = protocol_BufferEM4325[2];
			/* out 16 : Tag Handle */
			*pui16Param3 = ((uint16_t)(protocol_BufferEM4325[3]) << 8) | protocol_BufferEM4325[4];
			break;
#endif

		default:
			break;
	}
}

/**************************************************************************//**
 * @brief this function returned the status of the EM4325
 *
 * @note
 * This function should called after sending a SPI command
 *
 * @param[in] ui8IdCmd: The id of the Spi command to send
 * @param[out] pui16Param1: out parameter 1
 * @param[out] pui16Param3: out parameter 2
 * @param[out] pui16Param2: out parameter 3
 *
 * @return none
 *****************************************************************************/
uint8_t protocol_GetEm4325Status (void)
{
	return protocol_BufferEM4325[0];
}


