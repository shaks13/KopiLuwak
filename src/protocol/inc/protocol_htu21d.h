/*******************************************************************************
 * @file protocol_LIS3MDL
 * @brief this files contains the function set for the HTU21D
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef _PROTOCOL_HTU21D_H_
#define _PROTOCOL_HTU21D_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "common_macro.h"
#include "interface_i2c.h"
#include "interface_gpio.h"
#include "kernel_common.h"
#include "drv_sleep.h"



/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define PRTHTU21D_I2CADDRESS  						(0x80)
#define PRTHTU21D_BUFFER_NBBYTE						(16)

#define PRTHTU21D_TEMPERATURECONVERSION_CONSTANT1	(175.72)
#define PRTHTU21D_TEMPERATURECONVERSION_CONSTANT2	(65536)
#define PRTHTU21D_TEMPERATURECONVERSION_CONSTANT3	(46.85)


#define PRTHTU21D_HUMIDITYCONVERSION_CONSTANT1		(125)
#define PRTHTU21D_HUMIDITYCONVERSION_CONSTANT2		(65536)
#define PRTHTU21D_HUMIDITYCONVERSION_CONSTANT3		(6)


#define PRTHTU21D_POWERUP_TIMEMS					(15)

/*===========================================================================================================
						Enum definition
===========================================================================================================*/
/**
 * @enum protocol_Si7021_memoryMap_enum
 * @brief this enum defines the mapping of the SI7021
 */
typedef enum {
	PRTLIS3MDL_READHEATER_ADDRESS 				= 0x11	,				/*!< Read Heater Control Register */
	PRTLIS3MDL_WRITEHEATER_ADDRESS 				= 0x51	,				/*!< Write Heater Control Register */
	PRTLIS3MDL_READTEMP_ADDRESS 				= 0xE0	,				/*!< Read Temperature Value from Previous RH Measurement */
	PRTLIS3MDL_MEASURETEMPHOLD_ADDRESS 			= 0xE3	,				/*!< Measure Temperature, Hold Master Mode */
	PRTLIS3MDL_MEASURERHHOLD_ADDRESS 			= 0xE5	,				/*!< Measure Relative Humidity, Hold Master Mode */
	PRTLIS3MDL_WRITERHTUSERREAGISTER_ADDRESS 	= 0xE6	,				/*!< Write RH/T User Register 1  */
	PRTLIS3MDL_READRHTUSERREAGISTER_ADDRESS 	= 0xE7	,				/*!< Read RH/T User Register 1  */
	PRTLIS3MDL_MEASURETEMPNOTHOLD_ADDRESS 		= 0xF3	,				/*!< Measure Temperature, No Hold Master Mode */
	PRTLIS3MDL_MEASURERHNOHOLD_ADDRESS 			= 0xF5	,				/*!< Measure Relative Humidity, No Hold Master Mode */
	PRTLIS3MDL_RESET_ADDRESS 					= 0xFE	,				/*!< Reset */
	PRTLIS3MDL_READID1_ADDRESS 					= 0xFA0F	,			/*!< Read Electronoic Id 1st byte*/
	PRTLIS3MDL_READID2_ADDRESS 					= 0xFCC9	,			/*!< Read Electronoic Id 1st byte*/
	PRTLIS3MDL_READFWREVISION_ADDRESS 			= 0x84B8				/*!< Read Firmware Revision*/
}protocol_Si7021_memoryMap_enum;

/**
 * @enum protocol_Si7021_FirwmareRevision_enum
 * @brief this enum defines the firmware revision
 */
typedef enum {
	PRTLIS3MDL_FIRMWAREREVISION_10				= 0xFF	,				/*!< Firmware version 1.0 */
	PRTLIS3MDL_FIRMWAREREVISION_20 				= 0x20	,				/*!< Firmware version 2.0 */
}protocol_Si7021_FirwmareRevision_enum;

/*===========================================================================================================
						structure definition
===========================================================================================================*/
/** @struct prtlis3mdl_controlregister1
 *  @brief this structure is the 8 bits control register 1
 *  @var prtlis3mdl_controlregister1::bSelfTestenable
 *  Member 'bSelfTestenable' Self-test enable.
 *  @var prtlis3mdl_controlregister1::bFastODRenable
 *  Member 'bFastODRenable'FAST_ODR enables data rates higher than 80 Hz
 *  @var prtlis3mdl_controlregister1::b3ODR
 *  Member 'b3ODR' Output data rate selection.
 *  @var prtlis3mdl_controlregister1::b2XYoperativeMode
 *  Member 'b2XYoperativeMode' X and Y axes operative mode selection.
 *  @var prtlis3mdl_controlregister1::btempEnable
 *  Member 'btempEnable' Temperature sensor enable.
 */
typedef struct {
	uint8_t bSelfTestenable					: 1;
	uint8_t bFastODRenable					: 1;
	uint8_t b3ODR							: 3;
	uint8_t b2XYoperativeMode				: 2;
	uint8_t btempEnable						: 1;
}prthtu21d_controlregister1_structure;


/*===========================================================================================================
						public function
===========================================================================================================*/
void 	prthtu21d_init 					( void );
uint8_t prthtu21d_CheckDeviceId 		( void );
uint8_t prthtu21d_IsAvaialble 			( void );
void 	prthtu21d_Reset 				( void );
void 	prthtu21d_EnablePowerSuppply 	( bool bOnOrOff );
uint8_t	prthtu21d_MeasureTemperature 	( int16_t *i16Temperature );
void 	prthtu21d_MeasureHumidity 		( int16_t *i16Humidity );

#endif
