/*******************************************************************************
 * @file protocol_24LC64
 * @brief this files contains the function set for the 24LC64 (64 K I2C EEPROM)
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef PROTOCOL_24LC64_H
#define PROTOCOL_24LC64_H
#include "common_library.h"
#include "interface_i2c.h"
#include "common_statuscode.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
/* I2C address of the device*/
#define PRT24LC64_ADDRESS						(0xA0)

#define PRT24LC64_LENGTH_RX_BUFFER				(128)

#define PRT24LC64_LIMIT_INVALID_ADRS			(0x2000)
#define PRT24LC64_WRITECYCLEMS					(5)
#define PRT24LC64_BOOTTIMEMS					(1)
/*===========================================================================================================
						Public variables definitions
===========================================================================================================*/
/* i2c Buffer */
extern uint8_t prt24lc64_ui8Buffer[PRT24LC64_LENGTH_RX_BUFFER];

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	prt24LC64_init 			( void );
uint8_t prt24LC64_RandomRead 	( const uint16_t ui16MemoryAdr, uint8_t ui8NbByteToRead );
void 	prt24LC64_WriteByte 	( const uint16_t ui16MemoryAdr, const uint8_t ui8ByteToWrite );
uint8_t prt24LC64_Write 		( const uint16_t ui16NbByteToWrite, uint8_t const * pui8DataToWrite );
void 	prt24LC64_Enable24LC64 	( bool OnOrOff );
#endif // #PROTOCOL_24LC64_H
