/*******************************************************************************
 * @file interface_uart.h
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef INTERFACE_USART_H
#define INTERFACE_USART_H

#include "em_usart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "common_library.h"
#include "kernel_common.h"
#include "kernel_process.h"
#include "drv_sleep.h"
#include "timerdrv.h"



/*===========================================================================================================
						constant definition
===========================================================================================================*/



//#define INTERFACEUSART_COMMAND_NBMINBYTE		    6				/* command code + address + NbByte following*/
//#define INTERFACEUSART_COMMANDINDEX_NBBYTE	    	4				/* command code + address + NbByte following*/



/*===========================================================================================================
						structure definition
===========================================================================================================*/


//extern volatile intuart_circularBuffer_struct intuart_rxBuf;
//extern volatile intuart_circularBuffer_struct intuart_txBuf;




/*===========================================================================================================
						prototype
===========================================================================================================*/
#if USEM2MINTERFACE == 1 /* these IRQ handlers is dedicated to the M2M communication*/
//void 	intuart_Enable		( void );
//void 	intuart_Disable		( void );



//void 	UART0_RX_IRQHandler	(void);
//void 	UART0_TX_IRQHandler	(void);
#endif


#endif

