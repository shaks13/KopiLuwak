/*******************************************************************************
 * @file protocol_extMemory.h
 * @brief this function set is coded for the Ext Memory device
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef PROTOCOL_EXTMEMORY_H_
#define PROTOCOL_EXTMEMORY_H_

#include "common_library.h"
#include "interface_spi.h"
#include "spidrv.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define PROTOCOL_EXTMEMORY_SPIBUFFERSIZE 				(256)

/* Pin */
#define PROTOCOL_EXTMEMORY_PIN_WP						(0)
#define PROTOCOL_EXTMEMORY_PIN_HOLD						(1)

/* Port */
#define PROTOCOL_EXTMEMORY_PORT_WP						(gpioPortA)
#define PROTOCOL_EXTMEMORY_PORT_HOLD					(gpioPortA)

/* Commands */
#define PROTOCOL_EXTMEMORY_CMD_REQUEST_STATUS 			(0b00000101)
#define PROTOCOL_EXTMEMORY_CMD_READ 					(0b00000011)
#define PROTOCOL_EXTMEMORY_CMD_WRITE 					(0b00000010)
#define PROTOCOL_EXTMEMORY_CMD_ENABLE_WRITELATCH		(0b00000110)
#define PROTOCOL_EXTMEMORY_CMD_DISABLE_WRITELATCH		(0b00000100)

/* Nb Byte in Commands */
#define PROTOCOL_EXTMEMORY_NBBYTE_REQUEST_STATUS 		(1)
#define PROTOCOL_EXTMEMORY_NBBYTE_READ 					(3)
#define PROTOCOL_EXTMEMORY_NBBYTE_WRITE 				(3)
#define PROTOCOL_EXTMEMORY_NBBYTE_ENABLE_WRITELATCH		(1)
#define PROTOCOL_EXTMEMORY_NBBYTE_DISABLE_WRITELATCH	(1)

/*===========================================================================================================
						extern definition
===========================================================================================================*/
/* SPI RX/TX buffer dedicated to the external memory */
extern uint8_t protocol_BufferExtMemory [PROTOCOL_EXTMEMORY_SPIBUFFERSIZE] ;

/*===========================================================================================================
						prototype
===========================================================================================================*/
void protocol_InitExtMem				( void );
void protocol_ReqStatusExtMem 			(uint8_t * pui8MemStatus);
void protocol_ReadExtMem 				(const uint16_t ui16Adress, const uint8_t ui8NbByteToRead);
void protocol_WriteExtMem 				(const uint16_t ui16Adress, const uint8_t ui8NbByteToWrite, uint8_t const * pui8ByteToWrite);
void protocol_EnableWriteLatchExtMem	(void);
void protocol_DisableWriteLatchExtMem	(void);

#endif /* PROTOCOL_EXTMEMORY_H_ */
