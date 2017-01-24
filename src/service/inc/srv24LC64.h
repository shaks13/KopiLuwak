/*******************************************************************************
 * @file srv24LC64.h
 * @brief this files contains the function set for the 24LC64 (64 K I2C EEPROM)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SRV24LC64_H_
#define SRV24LC64_H_

#include "common_library.h"
#include "common_macro.h"
#include "common_version.h"
#include "common_statuscode.h"
#include "protocol_24LC64.h"
#include "srvCRC.h"
#include "drv_sleep.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define SRV24LC_ADDRESS_ALARM							0x0000
#define SRV24LC_ADDRESS_VERSION							0x1C00


#define SRV24LC_ADDRESS_24LC64_SIZE_16BITBUFFER 		0x000E
#define SRV24LC_NBBYTE_BOARDVERSION		 				0x0010

#define SRV24LC_ALARM_RESET				 				0x0000
/*===========================================================================================================
						enum definition
===========================================================================================================*/
/**
 *  @enum this enum contains the address to write in the 24LC64 memory
 *  to update the bit field alarm
 *  @note take care this enum should be aligned with serial_Adrs24LCMemory_enum
 */
typedef enum{
	SRV24LC_ADDRESS_ALARMBITFIELD 		= SRV24LC_ADDRESS_ALARM 	+ 0x0000,	/*!< the address of the alarm bit field status  */
	SRV24LC_ADDRESS_ALARMLOWTEMP  		= SRV24LC_ADDRESS_ALARM 	+ 0x0001,	/*!< the uint16_t address of the low temp alarm */
	SRV24LC_ADDRESS_ALARMHIGHTEMP 		= SRV24LC_ADDRESS_ALARM 	+ 0x0005,	/*!< the uint16_t address of the high temp alarm */
	SRV24LC_ADDRESS_ALARMLOWBAT 		= SRV24LC_ADDRESS_ALARM 	+ 0x0009,	/*!< the uint16_t address of the low bat alarm */
	SRV24LC_ADDRESS_ALARMCRC			= SRV24LC_ADDRESS_ALARM 	+ 0x000D,	/*!< the uint16_t address of the crc */
	SRV24LC_ADDRESS_FIRMWAREVERSION 	= SRV24LC_ADDRESS_VERSION 	+ 0x0000,	/*!< the address of the alarm bit field status  */
	SRV24LC_ADDRESS_HARDWAREVERSION  	= SRV24LC_ADDRESS_VERSION 	+ 0x0002,	/*!< the uint16_t address of the low temp alarm */
	SRV24LC_ADDRESS_TESTWAREVERSION 	= SRV24LC_ADDRESS_VERSION 	+ 0x0004,	/*!< the uint16_t address of the high temp alarm */
	SRV24LC_ADDRESS_CRCVERSION			= SRV24LC_ADDRESS_VERSION 	+ 0x000E,	/*!< the uint16_t address of the crc */
}srv24LC_Adrs24LCMemory_enum;

/*===========================================================================================================
						struct definition
===========================================================================================================*/

/** @struct srv24LC_MemoryContent_struct
 *  @brief this structure is a copy of the external memory
 *  @var srv24LC_MemoryContent_struc::pui16alarm
 *  Member 'pui16alarm' is the pointer on the alarms status
 *  @var srv24LC_MemoryContent_struc::version
 *  Member 'version' is the pointer on the version
 */
typedef struct {
	uint16_t  * pui16alarm;
	uint16_t  * version;
}srv24LC_MemoryContent_struct;


/*===========================================================================================================
						public definition definition
===========================================================================================================*/
extern srv24LC_MemoryContent_struct srv24LC_MemoryContent;

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	srv24LC64_init 							( void );
uint8_t srv24LC_ReadAlarmFields 				( uint16_t **ppui16AlarmStatus );
uint8_t srv24LC_WriteAlarmFields 				( uint16_t *pui16NewAlarmFieldToWrite );
uint8_t srv24LC_ReadBoardVersion				( uint16_t **pui16BoardVersion );
uint8_t	srv24LC_WriteBoardVersion				( void );
void 	srv24LC_Enable24LC64 					( bool OnOrOff );
void 	srv24LC64_GetAlarmStatus 				( uint16_t *ui16AlarmStatus );

#endif
