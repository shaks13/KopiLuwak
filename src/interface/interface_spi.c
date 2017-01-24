/*******************************************************************************
 * @file interface_spi.c
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "interface_spi.h"



static void interface_SetRoutePen (USART_TypeDef *usart, const uint32_t flags);
static void interface_SetCmd (USART_TypeDef *usart, const uint32_t flags);
#if 0
static void interface_SetCtrl (USART_TypeDef *usart, const uint32_t flags);
#endif


/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
static T_sSserial_bufferconfig 	sBufferConfig;
static bool 					interface_bStoreRx;
volatile uint8_t 				uBufferIndex;
//bool 							txActive;
//bool 							rxActive;
/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
volatile uint8_t 	interface_uIdxSpiByteReceived;/* Index of received bytes */
/* when the em4325 is a slave this boolean means the em4325 has just started emiting the response*/
volatile bool 		interface_bEm4325SpiDataReady;
volatile bool 		interface_bIntCommBuffer;
/*  SPI handle containing information such as peripherals that can trigger UDMA transfers, Tx/Rx buffers @, etc. */
SPIDRV_HandleData_t interface_sSpiHandle[INTERFACE_SPI_NBSPISLAVE];	 /* allocate the memory for the handle of the different SPI slave*/



#if 0
/*****************************************************************************
 * @brief  this function configures the CTRL register of the SPI
 * @detail this function doesn't exit in the emlib
 * @param[in] flag : flag to be set
 * @param[out] none
 * return none
 *****************************************************************************/
static void interface_SetCtrl (USART_TypeDef *usart, const uint32_t flags)
{
	usart->CTRL |= flags ;
}
#endif
/*****************************************************************************
 * @brief  this function configures the USART's I/O Routing Pin Enable Register
 * @detail allocate the hardware flow control to GPIOs
 * @param[in] flag : flag to be set
 * @param[out] none
 * return none
 *****************************************************************************/
static void interface_SetRoutePen (USART_TypeDef *usart, const uint32_t flags)
{
	usart->ROUTEPEN = flags ;
}
/*****************************************************************************
 * @brief  this function configures the routing of the CMD register
 * @detail this function doesn't exit in the emlib
 * @param[in] flag : flag to be set
 * @param[out] none
 * return none
 *****************************************************************************/
static void interface_SetCmd (USART_TypeDef *usart, const uint32_t flags)
{
	usart->CMD = flags ;
}


/******************************************************************************
 * @brief sends data using USART2
 * @param txBuffer points to data to transmit
 * @param bytesToSend bytes will be sent
 *****************************************************************************/
void interface_SendSpiBuffer(uint8_t* pui8TxBuffer, const uint8_t ui8BytesToSend)
{
USART_TypeDef *spi = INTERFACE_USART;


uint8_t        ui8NthByte;

	/* Sending the data */
	for (ui8NthByte = 0; ui8NthByte < ui8BytesToSend;  ui8NthByte++)
	{
		/* Waiting for the usart to be ready */
		while (!(USART_StatusGet (spi) & USART_STATUS_TXBL)) ;
		if (pui8TxBuffer != NULL)
		{
			/* Writing next byte to USART */
			spi->TXDATA = *pui8TxBuffer;
			pui8TxBuffer++;
		}
		else
		{
			spi->TXDATA = 0; /* force the MOSI to the low state */
		}
	}

	/*Waiting for transmission of last byte */
	while (!(USART_StatusGet (spi) & USART_STATUS_TXC)) ;
}

/******************************************************************************
 * @brief this functions waits the signal coming from slave meaning the
 * response is ready
 * @param[in] bnbMax : Number of clock
 *****************************************************************************/
void interface_WaitResponseReadySignal ( uint8_t bnbMax )
{
USART_TypeDef *spi = INTERFACE_USART;

	do
	{
			/* Tx dummy bytes */
			spi->TXDATA = 0;
			/* Waiting for the transmit buffer is empty */
			//while (false == (USART_StatusGet (spi) & USART_STATUS_TXBL)) ;
		/* Waiting for transmission has completed */
		while (false == (USART_StatusGet (spi) & USART_STATUS_TXC)) ;

	/* Waiting for RX data */
	} while ((interface_bEm4325SpiDataReady == false) && (bnbMax-->0));

	/* Waiting for transmission has completed */
	//while (false == (USART_StatusGet (spi) & USART_STATUS_TXC)) ;

}

/******************************************************************************
 * @brief this functions waits the signal coming from slave meaning the
 * response is ready
 * @param[in] bnbMax : Number of clock
 * @param[in] ui8Ms : delay in millisecond before receiving the response
 *****************************************************************************/
void interface_WaitEm4325SpiResponse ( const uint8_t ui8nbMax, const uint8_t ui8Ms )
{
	sleep_DelayTask (ui8Ms, configSLEEP_MODE);
	interface_WaitResponseReadySignal(ui8nbMax);
}

/**************************************************************************//**
 * @brief Starts the spi reception
 * @param pui8RxBuffer: Rx Buffer
 * @param ui8NbByteToReceived: Number of byte to receive
 * @return none
 *****************************************************************************/
void interface_StartSPIRx(uint8_t * pui8RxBuffer, const uint8_t ui8NbByteToReceived)
{
	interface_bStoreRx = false;
	/* Data not available (USART1 interrupt)*/
	interface_bEm4325SpiDataReady 	= false;
	//interface_EnablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN, INTERFACE_RISING_EDGE);
	interface_ActivatePinInterrupt (INTERFACE_MISO_PIN, true);
	interface_EnableSPIRxInt (pui8RxBuffer,ui8NbByteToReceived);
}

/*****************************************************************************
 * @brief  this function configures the routing of the CMD register
 * @detail this function doesn't exit in the emlib
 * @param[in] 	none
 * @param[out] 	none
 * return 		none
 *****************************************************************************/
void interface_ClearRx ( void )
{
	/* Enable Rx int */
	USART_RxDataGet (INTERFACE_USART);
	USART_IntEnable (INTERFACE_USART,USART_IEN_RXDATAV);
	interface_bStoreRx = true;
}

/**************************************************************************//**
 * @brief USART1 RX IRQ Handler Setup
 * @param receiveBuffer points to where to place received data
 * @param receiveBufferSize indicates the number of bytes to receive
 *****************************************************************************/
void interface_EnableSPIRxInt(uint8_t* receiveBuffer, const uint8_t bbytesToReceive)
{
USART_TypeDef *spi = INTERFACE_USART;


	/* Setting up pointer and indexes */
	sBufferConfig.pRxData      = receiveBuffer;
	sBufferConfig.uNbRxByte  = bbytesToReceive;
	interface_uIdxSpiByteReceived = 0;

	/* Clear receive buffer and the RX shift register. */
	interface_SetCmd (spi,USART_CMD_CLEARRX);
#if 0
	USART_IntClear(spi,_USART_IFC_MASK);
	/* Enable Rx int */
	USART_IntEnable (spi,USART_IEN_RXDATAV);
#endif
}

/**************************************************************************//**
 * @brief Disable the USAR1 RX Interruptions
 *****************************************************************************/
void interface_DisableSPIRxInt(void)
{
USART_TypeDef *spi = INTERFACE_USART;

	USART_IntDisable(spi,USART_IEN_RXDATAV);
}

/**************************************************************************//**
 * @brief this functions setups a USART as SPI interface
 * @param[in] bIsMaster true when the mcu is the master, false for the slave
 * @param[out] none
 * return none
 *****************************************************************************/
void interface_InitSPI ( const bool bIsMaster )
{

	USART_TypeDef *spi = INTERFACE_USART;
	USART_InitSync_TypeDef init = {	false, /** Specifies whether TX and/or RX shall be enabled when init completed. */
									0, /** USART/UART reference clock assumed when configuring baudrate setup. Set
										 * it to 0 if currently configurated reference clock shall be used. */
									INTERFACE_SPIBAUDRATE, /** Desired baudrate. */
									usartDatabits8,/** Number of databits in frame. */
									bIsMaster, /** Select if to operate in master or slave mode. */
									true, /** Select if to send most or least significant bit first. */
									usartClockMode0,  /** Clock idle high, sample on rising edge. */
									false,
									usartPrsRxCh0,
									false,
									false,
									0,
									0};

		/* Enabling clock to USART 1 */
		CMU_ClockEnable(INTERFACE_USARTCLOCK, true);
		USART_InitSync ( spi, &init);

		NVIC_ClearPendingIRQ(INTERFACE_RXIRQ);
		NVIC_SetPriority(INTERFACE_RXIRQ, (configKERNEL_INTERRUPT_PRIORITY >> ((8 - __NVIC_PRIO_BITS)) & 0x7));
		NVIC_EnableIRQ(INTERFACE_RXIRQ);
		/* Clear receive buffer and the RX shift register. */
		interface_SetCmd (spi,USART_CMD_CLEARRX | USART_CMD_CLEARTX);
		USART_IntDisable (spi, _USART_IFC_MASK);
		USART_IntClear (spi,_USART_IFC_MASK);
		/* Enable Transmitter & Receiver */
		interface_SetCmd (spi,USART_CMD_TXEN | USART_CMD_RXEN);
		/* Enabling pins and setting location */
		interface_SetRoutePen (spi, USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_CLKPEN /*| USART_ROUTEPEN_CSPEN*/);

		spi->ROUTELOC0 = USART_ROUTELOC0_TXLOC_LOC30 | USART_ROUTELOC0_RXLOC_LOC30 | USART_ROUTELOC0_CLKLOC_LOC30 /*| USART_ROUTELOC0_CSLOC_LOC11*/;

		interface_InitSpiGPIO(bIsMaster);
		/* No Data Available from EM4325 */
		interface_bEm4325SpiDataReady  = false;

}

/**************************************************************************//**
 * @brief this functions setups a USART as SPI interface and configure the
 * associated DMA Channel
 * @param[in] none
 * @param[out] none
 * return none
 *****************************************************************************/
void interface_InitSpiMasterUsingDMA (void)
{

	NVIC_ClearPendingIRQ(INTERFACE_RXIRQ );
	//NVIC_SetPriority(USART1_RX_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> ((8 - __NVIC_PRIO_BITS)) & 0x7));
	NVIC_EnableIRQ(INTERFACE_RXIRQ);

	/* Enable GPIO interrupts in the NVIC interrupt controller */
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief this functions deinits the SPI interface
 * @param[out] none
 * return none
 *****************************************************************************/
void interface_DeInitSPI (void)
{
USART_TypeDef *spi = INTERFACE_USART;

	NVIC_ClearPendingIRQ(INTERFACE_RXIRQ);
	NVIC_DisableIRQ(INTERFACE_RXIRQ);

	USART_Enable(spi, usartDisable);
	/* Clear receive buffer and the RX shift register. */
	interface_SetCmd (spi,USART_CMD_CLEARRX | USART_CMD_CLEARTX);
	USART_IntDisable (spi, _USART_IFC_MASK);
	USART_IntClear (spi,_USART_IFC_MASK);
	/* Clear transmit buffer and the TX shift register */
	interface_SetCmd (spi,USART_CMD_TXEN | USART_CMD_RXEN);
	/* Disabling pins / locations */
	interface_SetRoutePen (spi, USART_ROUTEPEN_TXPEN_DEFAULT | USART_ROUTEPEN_RXPEN_DEFAULT | USART_ROUTEPEN_CLKPEN_DEFAULT | USART_ROUTEPEN_CSPEN_DEFAULT);
	/* No Data Available from EM4325 */
	interface_bEm4325SpiDataReady  = false;
	/* Disabling clock to USART 1 */
	CMU_ClockEnable(INTERFACE_USARTCLOCK, false);


}

/**************************************************************************//**
 * @brief this functions deinits the SPI interface which used the dma
 * @param[out] none
 * return none
 *****************************************************************************/
void interface_DeInitSpiUsingDMA (void) /* Todo: check this function */
{
	SPIDRV_DeInit(interface_psSpiHandle);
	NVIC_ClearPendingIRQ(INTERFACE_RXIRQ);
	NVIC_DisableIRQ(INTERFACE_RXIRQ);

}

/**************************************************************************//**
 * @brief 		USART0 RX IRQ Handler
 * @detail 		the device connected to the SPI bus (USART#1) is the EM4325
 * @param[in]  	none
 * @param[out]  none
 * @return 		none
 *****************************************************************************/
void RFFRONTEND_RX_IRQHANDLER ( void )
{
uint32_t	wIntFlags;
USART_TypeDef *spi = INTERFACE_USART;
Kernel_QueueItem_struct pQueueItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_SERIALTASKID,KERNEL_SERIALTASKID),
										KERNEL_MESSAGEID_RX,
										0,NULL};
BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* We have not woken a task at the start of the ISR.*/


	wIntFlags = USART_IntGet (spi);
	USART_IntClear (spi,wIntFlags);

	if (wIntFlags & _USART_IF_RXDATAV_MASK)
	{
		{
			if ((interface_uIdxSpiByteReceived < INTERFACE_SPIBUFFERSIZE) /*&& (interface_uIdxSpiByteReceived < sBufferConfig.uNbRxByte)*/)
			{
				if(interface_bStoreRx /*interface_bEm4325SpiDataReady*/ == true)
				{
	#if (USESTARTERKIT == 1)
					/* Read EM4325 Data */
					sBufferConfig.pRxData[interface_uIdxSpiByteReceived++] = USART_RxDataGet (USART1);
	#elif (USECROSSTAG==1)
					/* Read EM4325 Data */
					sBufferConfig.pRxData[interface_uIdxSpiByteReceived++] = USART_RxDataGet (USART0);
	#endif
				}
				else
				{
					/* Buffer is cleared on read access. */
					USART_RxDataGet (spi);
				}
			}
			else
			{
				/* the buffer is full */
				USART_IntDisable(spi,USART_IEN_RXDATAV);
			}
		}

	}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD ();
	}

}


/**************************************************************************//**
 * @brief USART1 TX IRQ Handler
 * @detail the device connected to the SPI bus (USART#1) is the EM4325
 * @param[in]  none
 * @param[out]  none
 * @return none
 *****************************************************************************/
void RFFRONTEND_TX_IRQHANDLER(void)
{

}


