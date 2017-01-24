/*******************************************************************************
 * @file interface_uart.c
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "interface_leuart.h"

volatile intleuart_circularBuffer_struct intuart_rxBuf = { {0}, 0, 0, 0,INTERFACELEUART_COMMAND_NBMINBYTE, false };
volatile intleuart_circularBuffer_struct intuart_txBuf = { {0}, 0, 0, 0,INTERFACELEUART_COMMAND_NBMINBYTE, false };


/*===========================================================================================================
						Private function declaration
===========================================================================================================*/
static void intleuart_initmodule ( void );

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/


/***************************************************************************//**
 * @brief 		this function initializes the UART peripheral
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void intleuart_initmodule ( void )
{
LEUART_Init_TypeDef leuart1Init =LEUART_INIT_DEFAULT;
LEUART_TypeDef *leuart = INTERFACELEUART_M2M_LEUART;

	/* Enable clock  */
	CMU_ClockEnable(INTERFACELEUART_M2M_LEUARTCLOCK, true);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO); 		/* Select LFRCO as clock source for LFBCLK */


	/* Reseting and initializing LEUART */
	LEUART_Reset(leuart);
	LEUART_Init(leuart, &leuart1Init);

	/* Prepare UART Rx and Tx interrupts */
	LEUART_IntClear(leuart, _LEUART_IEN_MASK);
	LEUART_IntEnable(leuart, LEUART_IEN_RXDATAV); /* don't enable the RXdataV because it should be process when the Kernel has disabled the sleep mode*/

	NVIC_ClearPendingIRQ(INTERFACELEUART_M2M_IRQHANDLER);
	/* because this interrupt handler 'll call a FreeRtos function*/
	NVIC_SetPriority(INTERFACELEUART_M2M_IRQHANDLER, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);

	NVIC_EnableIRQ(INTERFACELEUART_M2M_IRQHANDLER);


	/* Route LEUART0 RX pin to DMA location 0 */
	leuart->ROUTELOC0 = LEUART_ROUTELOC0_TXLOC_LOC3 | LEUART_ROUTELOC0_RXLOC_LOC3;
	leuart->ROUTEPEN = LEUART_ROUTEPEN_RXPEN | LEUART_ROUTEPEN_TXPEN;

	/* Finally enable it */
	LEUART_Enable(leuart, leuartEnable);

}

/***************************************************************************//**
 * @brief 		this function initializes the UART peripheral
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void intleuart_init ( void )
{

	/* configure the GPIO*/
	GPIO_PinModeSet(INTERFACELEUART_TX_PORT, INTERFACELEUART_TX_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(INTERFACELEUART_RX_PORT, INTERFACELEUART_RX_PIN, gpioModeInput, 0);
	interface_EnablePinInterrupt (INTERFACELEUART_RX_PORT, INTERFACELEUART_RX_PIN, INTERFACE_FALLING_EDGE);
	intleuart_initmodule();
	intuart_PutString ("KopiLuwak , Come in\n");
}


/***************************************************************************//**
 * @brief 		this function disables the UART peripheral
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is mage thanje the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void intleuart_disable ( void )
{
LEUART_TypeDef *leuart = INTERFACELEUART_M2M_LEUART;

	/* Disable UART */
	LEUART_Enable(leuart, leuartDisable);

	GPIO_PinModeSet(INTERFACELEUART_TX_PORT, INTERFACELEUART_TX_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(INTERFACELEUART_RX_PORT, INTERFACELEUART_RX_PIN, gpioModeDisabled, 0);

	NVIC_DisableIRQ(INTERFACELEUART_M2M_IRQHANDLER);
	/* disable clock for USART */
	CMU_ClockEnable(INTERFACELEUART_M2M_LEUARTCLOCK, false);

}


/***************************************************************************//**
 * @brief		this function is the handler of the TX interruption of the USART 0
 * USEM2MINTERFACE
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void LEUART0_IRQHandler ( void )
{
LEUART_TypeDef *leuart = INTERFACELEUART_M2M_LEUART;
uint32_t	wIntFlags;
Kernel_QueueItem_struct pQueueItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_SERIALTASKID,KERNEL_SERIALTASKID),
										KERNEL_MESSAGEID_RX,
										0,NULL};
BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* We have not woken a task at the start of the ISR.*/


#if (USEM2MINTERFACE == 1)

	wIntFlags = LEUART_IntGet (leuart);
	/* Check for RX data valid interrupt */
	if (leuart->IF & LEUART_IF_RXDATAV)		/* a new character has been received */
	{
		LEUART_IntClear (leuart,LEUART_IF_RXDATAV);
		/* this IRq handler is available for the M2M management*/
		/* when the uc wakes up from the sleep mode */
	/*	if (false == sleep_IsWakeup())
		{
			USART_Rx(INTERFACELEUART_M2M_USART);
		}
		else
		{
	*/
		timer_ResetCounterTimer0 (); /* a new character has been received. This timer ensure the uc doesn't go below EM1*/
		intuart_rxBuf.data[intuart_rxBuf.wrI] = LEUART_Rx(INTERFACELEUART_M2M_LEUART);
		//intuart_rxBuf.wrI             = (intuart_rxBuf.wrI + 1) % INTERFACELEUART_BUFFER_NBBYTE;
		//intuart_rxBuf.pendingBytes++;
		intuart_rxBuf.wrI++ ;
		if ( '\r' == intuart_rxBuf.data[intuart_rxBuf.wrI-1] ) /* if the last character of the command */
		{
			pQueueItem.pData = (uint8_t *) (intuart_rxBuf.data);
			pQueueItem.ui16NbByte = intuart_rxBuf.wrI;
			//intuart_rxBuf.pendingBytes = 0;
			intuart_rxBuf.wrI = 0;
			xQueueSendFromISR (sKernel_QueuePointer.pSerialQueue,&pQueueItem, &xHigherPriorityTaskWoken);
		}
	}
	else if (leuart->IF & LEUART_IF_TXBL)   /* Check TX buffer level status */
	{
		LEUART_IntClear (leuart,LEUART_IF_TXBL);
		if (intuart_txBuf.pendingBytes > 0)
		{
		  /* Transmit pending character */
		  LEUART_Tx(INTERFACELEUART_M2M_LEUART, intuart_txBuf.data[intuart_txBuf.rdI]);
		  intuart_txBuf.rdI = (intuart_txBuf.rdI + 1) % INTERFACELEUART_BUFFER_NBBYTE;
		  intuart_txBuf.pendingBytes--;
		}

		/* Disable Tx interrupt if no more bytes in queue */
		if (intuart_txBuf.pendingBytes == 0)
		{
		  LEUART_IntDisable(INTERFACELEUART_M2M_LEUART, LEUART_IEN_TXBL);
		}
	}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD ();
	}

#endif
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
void intuart_PutString(const char* const str)
{
	intuart_PutData((const uint8_t* const)str, strlen(str)+1);
}


/***************************************************************************//**
 * @brief
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanke the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	dataPtr : pointer on the Ascii string to be sent
 * @param[in] 	dataLen : number ASCII characters to be sent
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void intuart_PutData(const uint8_t* const dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;
  LEUART_TypeDef *leuart = INTERFACELEUART_M2M_LEUART;

  /* Check if buffer is large enough for data */
  if (dataLen <= INTERFACELEUART_BUFFER_NBBYTE)
  {
	  /* Check if buffer has room for new data */
	  if ((intuart_txBuf.pendingBytes + dataLen) > INTERFACELEUART_BUFFER_NBBYTE)
	  {
		/* Wait until room */
		while ((intuart_txBuf.pendingBytes + dataLen) > INTERFACELEUART_BUFFER_NBBYTE) ;
	  }

	  /* Fill dataPtr[0:dataLen-1] into txBuffer */
	  while (i < dataLen)
	  {
		intuart_txBuf.data[intuart_txBuf.wrI] = *(dataPtr + i);
		intuart_txBuf.wrI             = (intuart_txBuf.wrI + 1) % INTERFACELEUART_BUFFER_NBBYTE;
		i++;
	  }

	  /* Increment pending byte counter */
	  intuart_txBuf.pendingBytes += dataLen;

	  /* Enable interrupt on USART TX Buffer*/
	  LEUART_IntEnable(leuart, LEUART_IEN_TXBL);
  }
  else
  {/* Buffer can never fit the requested amount of data */}
}




/***************************************************************************//**
 * @brief
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanke the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	dataPtr : pointer of the received data
 * @param[in] 	dataLen : number of expected data
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
uint32_t intuart_GetData ( uint8_t * dataPtr, uint16_t dataLen )
{
  uint32_t i = 0;

  /* Wait until the requested number of bytes are available */
  if (intuart_rxBuf.pendingBytes < dataLen)
  {
    while (intuart_rxBuf.pendingBytes < dataLen) ;
  }

  if (dataLen == 0)
  {
    dataLen = intuart_rxBuf.pendingBytes;
  }

  /* Copy data from Rx buffer to dataPtr */
  while (i < dataLen)
  {
    *(dataPtr + i) = intuart_rxBuf.data[intuart_rxBuf.rdI];
    intuart_rxBuf.rdI      = (intuart_rxBuf.rdI + 1) % INTERFACELEUART_BUFFER_NBBYTE;
    i++;
  }

  /* Decrement pending byte counter */
  intuart_rxBuf.pendingBytes -= dataLen;

  return i;
}

