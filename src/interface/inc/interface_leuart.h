/*******************************************************************************
 * @file interface_leuart.h
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef INTERFACE_LEUART_H
#define INTERFACE_LEUART_H

#include "em_leuart.h"
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
#define INTERFACELEUART_BUFFER_NBBYTE         		64				/* length of the circular buffer */

#define INTERFACELEUART_RX_PORT		     	 	    gpioPortA
#define INTERFACELEUART_TX_PORT		      		    gpioPortA
#define INTERFACELEUART_RX_PIN		      		    4
#define INTERFACELEUART_TX_PIN		       		 	3
#define INTERFACELEUART_M2M_LEUART					LEUART0
#define INTERFACELEUART_M2M_LEUARTCLOCK				cmuClock_LEUART0


#define INTERFACELEUART_M2M_IRQHANDLER			LEUART0_IRQn

#define INTERFACELEUART_COMMAND_NBMINBYTE		    6				/* command code + address + NbByte following*/
#define INTERFACELEUART_COMMANDINDEX_NBBYTE	    	4				/* command code + address + NbByte following*/


/*===========================================================================================================
						structure definition
===========================================================================================================*/
typedef struct
{
  uint8_t  data[INTERFACELEUART_BUFFER_NBBYTE];  /* data buffer */
  uint16_t rdI;               /* read index */
  uint16_t wrI;               /* write index */
  uint16_t pendingBytes;      /* count of how many bytes are not yet handled */
  uint16_t ExpectedBytes;     /* count the number of expected bytes */
  bool     overflow;          /* buffer overflow indicator */
}intleuart_circularBuffer_struct;


/*===========================================================================================================
						public variable
===========================================================================================================*/
extern volatile intleuart_circularBuffer_struct intuart_rxBuf;
extern volatile intleuart_circularBuffer_struct intuart_txBuf;

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	intleuart_init 			( void );
void 	intleuart_disable		( void );
void 	intuart_PutString		( const char* const dataPtr );
void 	intuart_PutData		( const uint8_t* const dataPtr, uint32_t dataLen );
uint32_t uartGetData		( uint8_t * dataPtr, uint32_t dataLen );

#endif
