/**
 * @file interface_spi.h
 * @author
 *
 * @section DESCRIPTION
 *
 * This file represents the slave spi interface.
 */

#ifndef _INTERFACE_SPI_H_
#define _INTERFACE_SPI_H_



#include "common_library.h"
#include "em_device.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "spidrv.h"
#include "interface_gpio.h"
#include "timerdrv.h"
#include "drv_sleep.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/

#define INTERFACE_SPI_NBSPISLAVE					3

#define INTERFACE_SPIBAUDRATE						4000000		/* 4 MHz */
#define INTERFACE_SPIBUFFERSIZE						16

#if USESTARTERKIT
#define INTERFACE_AUX_PORT							gpioPortC
#define INTERFACE_AUX_PIN							10
#define INTERFACE_MISO_PORT							gpioPortC
#define INTERFACE_MISO_PIN							7
#define INTERFACE_RX_PORT							INTERFACE_MISO_PORT
#define INTERFACE_RX_PIN							INTERFACE_MISO_PIN
#define INTERFACE_MOSI_PORT							gpioPortC
#define INTERFACE_MOSI_PIN							6
#define INTERFACE_CS_PORT							gpioPortC
#define INTERFACE_CS_PIN							9
#define INTERFACE_CLK_PORT							gpioPortC
#define INTERFACE_CLK_PIN							8
#define INTERFACE_USART								USART1
#define INTERFACE_RXIRQ								USART1_RX_IRQn
#define INTERFACE_TXIRQ								USART1_RX_IRQn
#define INTERFACE_USARTCLOCK						cmuClock_USART1
#define RFFRONTEND_RX_IRQHANDLER							USART1_RX_IRQHandler

/* Pin configuration */
#define INTERFACE_SPI2INT1_PORT						gpioPortA
#define INTERFACE_SPI2INT1_PIN						4
#define INTERFACE_SPI2INT2_PORT						gpioPortA
#define INTERFACE_SPI2INT2_PIN						5

#elif (USECROSSTAG==1)

#define INTERFACE_AUX_PORT							gpioPortB
#define INTERFACE_AUX_PIN							13
#define INTERFACE_MISO_PORT							gpioPortF
#define INTERFACE_MISO_PIN							7
#define INTERFACE_RX_PORT							INTERFACE_MISO_PORT
#define INTERFACE_RX_PIN							INTERFACE_MISO_PIN
#define INTERFACE_MOSI_PORT							gpioPortF
#define INTERFACE_MOSI_PIN							6
#define INTERFACE_CS_PORT							gpioPortA
#define INTERFACE_CS_PIN							1
#define INTERFACE_CLK_PORT							gpioPortA
#define INTERFACE_CLK_PIN							0
#define INTERFACE_USART								USART0
#define INTERFACE_RXIRQ								USART0_RX_IRQn
#define INTERFACE_TXIRQ								USART0_RX_IRQn
#define INTERFACE_USARTCLOCK						cmuClock_USART0
#define RFFRONTEND_RX_IRQHANDLER					USART0_RX_IRQHandler
#define RFFRONTEND_TX_IRQHANDLER					USART0_TX_IRQHandler


/* Pin configuration */
#define INTERFACE_LIS2DHINT1_PORT					(gpioPortA)
#define INTERFACE_LIS2DHINT1_PIN					(3)
#define INTERFACE_LIS2DHINT2_PORT					(gpioPortA)
#define INTERFACE_LIS2DHINT2_PIN					(4)
#define INTERFACE_LIS2DHEN_PORT						(gpioPortD)
#define INTERFACE_LIS2DHEN_PIN						(12)

#define ADXL363_RX_IRQHANDLER						USART1_RX_IRQHandler
#define ADXL363_TX_IRQHANDLER						USART1_TX_IRQHandler


#else
#define INTERFACE_AUX_PORT							gpioPortC
#define INTERFACE_AUX_PIN							10
#define INTERFACE_MISO_PORT							gpioPortA
#define INTERFACE_MISO_PIN							3
#define INTERFACE_RX_PORT							INTERFACE_MISO_PORT
#define INTERFACE_RX_PIN							INTERFACE_MISO_PIN
#define INTERFACE_MOSI_PORT							gpioPortA
#define INTERFACE_MOSI_PIN							2
#define INTERFACE_CLK_PORT							gpioPortA
#define INTERFACE_CLK_PIN							4
#define INTERFACE_CS_PORT							gpioPortA
#define INTERFACE_CS_PIN							5
#define INTERFACE_USART								USART0
#define INTERFACE_RXIRQ								USART0_RX_IRQn
#define INTERFACE_TXIRQ								USART0_RX_IRQn
#define INTERFACE_USARTCLOCK						cmuClock_USART0
#define RFFRONTEND_RX_IRQHANDLER							USART0_RX_IRQHandler


/* Pin configuration */
#define INTERFACE_SPI2INT1_PORT						gpioPortB
#define INTERFACE_SPI2INT1_PIN						12

/* ADXL435 interruption data ready */
#define INTERFACE_ADXL345INT2_PORT					gpioPortB
#define INTERFACE_ADXL345INT2_PIN					13

#endif



/*===========================================================================================================
						enum definition
===========================================================================================================*/
typedef enum
{
	INTERFACE_EXTERNALMEMORY =0x00,
	INTERFACE_SENSOR1 ,
	INTERFACE_SENSOR2 ,
}interface_slaveID;


/*===========================================================================================================
						structure definition
===========================================================================================================*/
/** @struct serial_DMACongig_struct
 *  @brief This structure contains the configuration of the DMA transfer
 *  @var serial_DMACongig_struct::bNbByte
 *  Member 'bNbByte' is the number of byte to be transfered
 *  @var serial_DMACongig_struct::pdata
 *  Member 'pdata' points on the data buffer
 */
typedef struct {
	uint8_t uNbRxByte;
	uint8_t * pRxData;
	uint8_t uNbTxByte;
	uint8_t * pTxdata;
} T_sSserial_bufferconfig;

/*===========================================================================================================
						glabal definition
===========================================================================================================*/
extern volatile uint8_t interface_uIdxSpiByteReceived;/* Index of received bytes */
extern volatile bool 	interface_bEm4325SpiDataReady;
extern volatile bool 	interface_bIntCommBuffer;
extern SPIDRV_Handle_t 	interface_psSpiHandle ;
extern SPIDRV_HandleData_t interface_sSpiHandle[];

/*===========================================================================================================
						prototypes
===========================================================================================================*/
void 	interface_InitSPI 						( const bool bIsMaster);
void 	interface_InitSpiMasterUsingDMA 		( void );
void 	interface_DeInitSPI 					( void );
void 	interface_DeInitSpiUsingDMA			( void );
void 	interface_SendSpiBuffer				(uint8_t* pui8TxBuffer, const uint8_t ui8BytesToSend);
void 	interface_StartSPIRx					(uint8_t * pui8RxBuffer, const uint8_t ui8NbByteToReceived);
void 	interface_EnableSPIRxInt				(uint8_t* receiveBuffer, const uint8_t bbytesToReceive);
void 	interface_DisableSPIRxInt				( void );
void 	interface_ClearRx 						( void );
void 	interface_WaitEm4325SpiResponse 	    ( const uint8_t ui8nbMax, const uint8_t ui8Ms );
void 	interface_WaitResponseReadySignal 		( uint8_t bnbMax );

#endif // #ifndef _INTERFACE_SPI_H_
