/*******************************************************************************
 * @file protocol_extMemory.c
 * @brief this function set is coded for the external memory device
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "protocol_extMemory.h"

static SPIDRV_Handle_t 	prtExtMem_psSpiHandle;	/* pointer of the handle of the SPI interface*/

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
uint8_t protocol_BufferExtMemory [PROTOCOL_EXTMEMORY_SPIBUFFERSIZE] ;

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/


/**************************************************************************//**
 * @brief Init the external memory DMA
 * @param[in]  sSpiHAndle
 * @param[out] none
 * @return none
 *****************************************************************************/
void protocol_InitExtMem( void )
{
/* Configuration data for SPI master using USART1 */
SPIDRV_Init_t sInitData = SPIDRV_MASTER_USART1_FOREXTMEMORY;

	/* configure psSpiHandle thanks to interface_psSpiHandle */
	prtExtMem_psSpiHandle = &(interface_sSpiHandle[INTERFACE_EXTERNALMEMORY]);

	/* Initialize a instance "interface_psSpiHandle" and the USARTx according to sInitData*/
	/* DMA Signals :	- USART1_TXBL
	 * 					- USART1_RXDATAV */
	SPIDRV_Init(prtExtMem_psSpiHandle, &sInitData);


#if 0	/* not used anymore */
	/* /WP: Write Protection enable */
	GPIO_PinModeSet(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP, gpioModePushPull, 0);
	/* /HOLD: Hold disable */
	GPIO_PinModeSet(PROTOCOL_EXTMEMORY_PORT_HOLD,PROTOCOL_EXTMEMORY_PIN_HOLD, gpioModePushPull, 1);
#endif

	interface_InitSpiMasterUsingDMA ();

}

/**************************************************************************//**
 * @brief Sends a Request Status to the external memory
 * @param[in]  none
 * @param[out] pui8MemStatus: the external memory status
 * @return none
 *****************************************************************************/
void protocol_ReqStatusExtMem (uint8_t * pui8MemStatus)
{
	protocol_BufferExtMemory[0] = PROTOCOL_EXTMEMORY_CMD_REQUEST_STATUS;
	protocol_BufferExtMemory[1] = (uint8_t)0;

	/* start dma transfers and wait their completion */
	SPIDRV_MTransferB( 	interface_psSpiHandle,
						protocol_BufferExtMemory,
						protocol_BufferExtMemory,
						PROTOCOL_EXTMEMORY_NBBYTE_REQUEST_STATUS+1);

	/* return the status of EM4325*/
	(*pui8MemStatus) =  protocol_BufferExtMemory[0];
}

/**************************************************************************//**
 * @brief Sends a Read to the external memory
 * @param[in]  ui16Adress: the address of the first byte to read
 * @param[in] ui8NbByteToRead: the number of bytes to read
 * @param[out] none
 * @return none
 *****************************************************************************/
void protocol_ReadExtMem (const uint16_t ui16Adress, const uint8_t ui8NbByteToRead)
{
	protocol_BufferExtMemory[0] = PROTOCOL_EXTMEMORY_CMD_READ;
	protocol_BufferExtMemory[1] = (uint8_t)((ui16Adress & 0xFF00) >> 8);
	protocol_BufferExtMemory[2] = (uint8_t)(ui16Adress & 0x00FF);
	memset (&protocol_BufferExtMemory[3],0,ui8NbByteToRead);

	/* start dma transfers and wait their completion */
	SPIDRV_MTransferB( 	interface_psSpiHandle,
						protocol_BufferExtMemory,
						protocol_BufferExtMemory,
						ui8NbByteToRead+PROTOCOL_EXTMEMORY_NBBYTE_READ);
}

/**************************************************************************//**
 * @brief Sends a Write to the external memory
 * @param[in]  ui16Adress: the address of the first byte to write
 * @param[in] ui8NbByteToWrite: the number of bytes to write
 * @param[in] pui8ByteToWrite: the buffer containing the bytes to write
 * @param[out] none
 * @return none
 *****************************************************************************/
void protocol_WriteExtMem (const uint16_t ui16Adress, const uint8_t ui8NbByteToWrite, uint8_t const * pui8ByteToWrite)
{
	protocol_BufferExtMemory[0] = PROTOCOL_EXTMEMORY_CMD_WRITE;
	protocol_BufferExtMemory[1] = (uint8_t)((ui16Adress & 0xFF00) >> 8);
	protocol_BufferExtMemory[2] = (uint8_t)(ui16Adress & 0x00FF);
	memcpy(&protocol_BufferExtMemory[3],pui8ByteToWrite,ui8NbByteToWrite);

#if 0	/* not used anymore */
	/* Disable write protection */
	GPIO_PinOutSet(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP);
#endif
	/* start dma Tx and wait their completion */
	SPIDRV_MTransmitB( 	interface_psSpiHandle,
						protocol_BufferExtMemory,
						ui8NbByteToWrite+PROTOCOL_EXTMEMORY_NBBYTE_WRITE);
#if 0	/* not used anymore */
	/* Enable write protection */
	GPIO_PinOutClear(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP);
#endif
}

/**************************************************************************//**
 * @brief Sends a command to enable the write latch of the external memory
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
void protocol_EnableWriteLatchExtMem ( void )
{
	protocol_BufferExtMemory[0] = PROTOCOL_EXTMEMORY_CMD_ENABLE_WRITELATCH;

#if 0	/* not used anymore */
	/* Disable write protection */
	GPIO_PinOutSet(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP);
#endif

	/* start dma Tx and wait their completion */
	SPIDRV_MTransmitB( 	interface_psSpiHandle,
						protocol_BufferExtMemory,
						PROTOCOL_EXTMEMORY_NBBYTE_ENABLE_WRITELATCH);

#if 0	/* not used anymore */
	/* Enable write protection */
	GPIO_PinOutClear(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP);
#endif
}

/**************************************************************************//**
 * @brief Sends a command to disable the write latch of the external memory
 * @param[in]  none
 * @param[out] none
 * @return none
 *****************************************************************************/
void protocol_DisableWriteLatchExtMem ( void )
{
	protocol_BufferExtMemory[0] = PROTOCOL_EXTMEMORY_CMD_DISABLE_WRITELATCH;

#if 0	/* not used anymore */
	/* Disable write protection */
	GPIO_PinOutSet(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP);
#endif

	/* start dma Tx and wait their completion */
	SPIDRV_MTransmitB( 	interface_psSpiHandle,
						protocol_BufferExtMemory,
						PROTOCOL_EXTMEMORY_NBBYTE_DISABLE_WRITELATCH);

#if 0	/* not used anymore */
	/* Enable write protection */
	GPIO_PinOutClear(PROTOCOL_EXTMEMORY_PORT_WP,PROTOCOL_EXTMEMORY_PIN_WP);
#endif
}
