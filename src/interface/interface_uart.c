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
#include "interface_uart.h"


/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
#if 0
static void 	intuart_init		( void );

static uint8_t 	intuart_GetChar		( void );
static void 	intuart_PutChar		( uint8_t ch );
#endif
/*===========================================================================================================
						Private function declaration
===========================================================================================================*/
#if USEM2MINTERFACE == 1 /* this IRQ handler is dedicated to the M2M communication*/


#if 0
/***************************************************************************//**
 * @brief 		this function disables the UART peripheral
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @note 		that if there are no pending characters in the receive buffer, this
 *  function return 0.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static uint8_t intuart_GetChar( )
{
  uint8_t ch;

  /* Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data */
  if (intuart_rxBuf.pendingBytes < 1)
  {
    return 0;
  }

  /* Copy data from buffer */
  ch        = intuart_rxBuf.data[intuart_rxBuf.rdI];
  intuart_rxBuf.rdI = (intuart_rxBuf.rdI + 1) % INTERFACEUSART_BUFFER_NBBYTE;

  /* Decrement pending byte counter */
  intuart_rxBuf.pendingBytes--;

  return ch;
}

/***************************************************************************//**
 * @brief 		this function disables the UART peripheral
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @note 		that if there are no pending characters in the receive buffer, this
 *  function return 0.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void intuart_PutChar(uint8_t ch)
{
  /* Check if Tx queue has room for new data */
  if ((txBuf.pendingBytes + 1) > INTERFACEUSART_BUFFER_NBBYTE)
  {
    /* Wait until there is room in queue */
    while ((txBuf.pendingBytes + 1) > INTERFACEUSART_BUFFER_NBBYTE) ;
  }

  /* Copy ch into txBuffer */
  txBuf.data[txBuf.wrI] = ch;
  txBuf.wrI             = (txBuf.wrI + 1) % INTERFACEUSART_BUFFER_NBBYTE;

  /* Increment pending byte counter */
  txBuf.pendingBytes++;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(USART0, USART_IEN_TXBL);
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
	intuart_PutData((const uint8_t* const)str, strlen(str));
}

#endif


/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/


#if 0
/***************************************************************************//**
 * @brief 		this function initializes the UART peripheral
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void intuart_Enable(void)
{
	/* Configure the GPIO*/
	GPIO_PinModeSet (INTERFACEUSART_TX_PORT,INTERFACEUSART_TX_PIN,gpioModePushPull,1);
	GPIO_PinModeSet (INTERFACEUSART_RX_PORT,INTERFACEUSART_RX_PIN,gpioModeInput,0);
	interface_EnablePinInterrupt (INTERFACEUSART_RX_PORT, INTERFACEUSART_RX_PIN, INTERFACE_FALLING_EDGE);
	intuart_init();
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
void intuart_Disable(void)
{
	/* Disable UART */
	USART_Enable(INTERFACEUSART_M2M_USART, usartDisable);

	GPIO_PinModeSet(INTERFACEUSART_TX_PORT, INTERFACEUSART_TX_PIN, gpioModeDisabled, 0); /* PE 10 UART0_TX*/
	GPIO_PinModeSet(INTERFACEUSART_RX_PORT, INTERFACEUSART_RX_PIN, gpioModeDisabled, 0); /* PE 11 UART0_RX */

	NVIC_DisableIRQ(INTERFACEUSART_M2M_RXIRQHANDLER);
	NVIC_DisableIRQ(INTERFACEUSART_M2M_TXIRQHANDLER);
	/* disable clock for USART */
	CMU_ClockEnable(INTERFACEUSART_M2M_USARTCLOCK, false);

}
#endif

#endif





