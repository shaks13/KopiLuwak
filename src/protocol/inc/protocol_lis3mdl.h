/*******************************************************************************
 * @file protocol_LIS3MDL
 * @brief this files contains the function set for the 24LC64 (64 K I2C EEPROM)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef _PROTOCOL_LIS3MDL_H_
#define _PROTOCOL_LIS3MDL_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "common_macro.h"
#include "interface_i2c.h"
#include "interface_gpio.h"
#include "kernel_common.h"
#include "kernel_common.h"



/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define PRTLIS3MDL_LENGTH_RX_BUFFER						(16)
#define PRTLIS3MDL_DEFAULTREFISTERVALUE					(0x00)

#define _LIS3MDL_REG_CTL_1_TEMP_EN 						0b10000000

#define _LIS3MDL_REG_CTL_2_RESET   						0b00000100

#define _LIS3MDL_REG_WHO_AM_I     						0x0F
#define _LIS3MDL_REG_CTL_1        						0x20
#define _LIS3MDL_REG_CTL_2        						0x21
#define _LIS3MDL_REG_CTL_3        						0x22
#define _LIS3MDL_REG_CTL_4        						0x23
#define _LIS3MDL_REG_STATUS      						0x27
#define _LIS3MDL_REG_OUT_X_L      						0x28
#define _LIS3MDL_REG_OUT_X_H     	 					0x29
#define _LIS3MDL_REG_OUT_Y_L      						0x2A
#define _LIS3MDL_REG_OUT_Y_H      						0x2B
#define _LIS3MDL_REG_OUT_Z_L      						0x2C
#define _LIS3MDL_REG_OUT_Z_H      						0x2D
#define _LIS3MDL_REG_OUT_TEMP_L   						0x2E
#define _LIS3MDL_REG_OUT_TEMP_H   						0x2F
#define _LIS3MDL_REG_INT_CONFIG  						0x30
#define _LIS3MDL_REG_INT_SOURCE  						0x31
#define _LIS3MDL_REG_THRESHOLD_L 						0x32
#define _LIS3MDL_REG_THRESHOLD_H 						0x33

#define LIS3MDL_ADDRESS1  								(0b0011110 << 1)
#define LIS3MDL_ADDRESS2  								(0b0011100 << 1)

#define LIS3MDL_PERFORMANCE_LOW_POWER  					0b00
#define LIS3MDL_PERFORMANCE_MEDIUM     					0b01
#define LIS3MDL_PERFORMANCE_HIGH       					0b10
#define LIS3MDL_PERFORMANCE_ULTRA_HIGH 					0b11

#define LIS3MDL_DATA_RATE_0_625_HZ 						0b000
#define LIS3MDL_DATA_RATE_1_25_HZ  						0b001
#define LIS3MDL_DATA_RATE_2_5_HZ   						0b010
#define LIS3MDL_DATA_RATE_5_HZ     						0b011
#define LIS3MDL_DATA_RATE_10_HZ    						0b100
#define LIS3MDL_DATA_RATE_20_HZ    						0b101
#define LIS3MDL_DATA_RATE_40_HZ    						0b110
#define LIS3MDL_DATA_RATE_80_HZ    						0b111

#define LIS3MDL_MODE_CONTINUOUS    						0b00
#define LIS3MDL_MODE_SINGLE        						0b01
#define LIS3MDL_MODE_POWER_DOWN    						0b11

#define LIS3MDL_SCALE_4_GAUSS      						0b00
#define LIS3MDL_SCALE_8_GAUSS      						0b01
#define LIS3MDL_SCALE_12_GAUSS     						0b10
#define LIS3MDL_SCALE_16_GAUSS     						0b11

#define LIS3MDL_AXIS_X          						0
#define LIS3MDL_AXIS_Y          						1
#define LIS3MDL_AXIS_Z           						2

#define LIS3MDL_STATUS_ZYXOR       						0b10000000
#define LIS3MDL_STATUS_ZOR     						    0b01000000
#define LIS3MDL_STATUS_YOR      				   		0b00100000
#define LIS3MDL_STATUS_XOR        						0b00010000
#define LIS3MDL_STATUS_ZYXDA    					   	0b00001000
#define LIS3MDL_STATUS_ZDA       					  	0b00000100
#define LIS3MDL_STATUS_YDA     					    	0b00000010
#define LIS3MDL_STATUS_XDA      					   	0b00000001

#define LIS3MDL_DEVICE_ID          						0b00111101

#define LIS3MDL_DSENSITIVITY_4GAUSS						6842 /* value given by the datasheet*/
#define LIS3MDL_DSENSITIVITY_8GAUSS						3421 /* value given by the datasheet*/
#define LIS3MDL_DSENSITIVITY_12GAUSS					2281 /* value given by the datasheet*/
#define LIS3MDL_DSENSITIVITY_16GAUSS					1711 /* value given by the datasheet*/


/*===========================================================================================================
						Enum definition
===========================================================================================================*/
/**
 * @enum protocol_LIS3MDL_Enable_enum
 * @brief this enum defines the values to enable or disable acc to the LIS3MDL datasheet
 */
typedef enum {
	PRTLIS3MDL_DISABLE 	= 0x00,				/*!< the value to disable a functionality in a register */
	PRTLIS3MDL_ENABLE  	,					/*!< the value to enable a functionality in a register */
}protocol_LIS3MDL_Enable_enum;

/**
 * @enum protocol_LIS3MDL_memorymap_enum
 * @brief this enum defines the memory map according to the datasheet of the device
 */
typedef enum {
	PRTLIS3MDL_REGISTER_WHOAMI 			= _LIS3MDL_REG_WHO_AM_I,			/*!< who am: I */
	PRTLIS3MDL_REGISTER_CONTROL1 		= _LIS3MDL_REG_CTL_1,				/*!< control register 1.*/
	PRTLIS3MDL_REGISTER_CONTROL2 		,									/*!< control register 2.*/
	PRTLIS3MDL_REGISTER_CONTROL3 		,									/*!< control register 3.*/
	PRTLIS3MDL_REGISTER_CONTROL4 		,									/*!< control register 4.*/
	PRTLIS3MDL_REGISTER_CONTROL5 		,									/*!< control register 5.*/
	PRTLIS3MDL_REGISTER_STATUTREG 		= _LIS3MDL_REG_STATUS,				/*!< status register */
	PRTLIS3MDL_REGISTER_XAXISLSB		= _LIS3MDL_REG_OUT_X_L,				/*!< X axis LSB register */
	PRTLIS3MDL_REGISTER_XAXISMSB		,									/*!< X axis MSB register */
	PRTLIS3MDL_REGISTER_YAXISLSB		= _LIS3MDL_REG_OUT_Y_L,				/*!< Y axis LSB register */
	PRTLIS3MDL_REGISTER_YAXISMSB		,									/*!< Y axis MSB register */
	PRTLIS3MDL_REGISTER_ZAXISLSB		= _LIS3MDL_REG_OUT_Z_L,				/*!< Z axis LSB register */
	PRTLIS3MDL_REGISTER_ZAXISMSB		,									/*!< Z axis MSB register */
	PRTLIS3MDL_REGISTER_INTCONFIG		= _LIS3MDL_REG_INT_CONFIG,			/*!< Interruption config register */
	PRTLIS3MDL_REGISTER_INTSOURCE		= _LIS3MDL_REG_INT_SOURCE,			/*!< Interruption source register */
	PRTLIS3MDL_REGISTER_INTTHRESHOLDL	= _LIS3MDL_REG_THRESHOLD_L,			/*!< Interruption threshold low register */
	PRTLIS3MDL_REGISTER_INTTHRESHOLDH	,									/*!< Interruption threshold high register */


}protocol_LIS3MDL_memorymap_enum;


/**
 * @enum protocol_LIS3MDL_XYaxisOperativeMode_enum
 * @brief this enum defines the values of the operating mode
 */
typedef enum {
	PRTLIS3MDL_CTRLREG1_OPMODE_LOWPOWER 	= 0x00,				/*!< low power mode */
	PRTLIS3MDL_CTRLREG1_OPMODE_MEDIUMPERFORMANCEMODE,			/*!< medium performance mode */
	PRTLIS3MDL_CTRLREG1_OPMODE_HIGHPERFORMANCEMODE,				/*!< high performance mode */
	PRTLIS3MDL_CTRLREG1_OPMODE_ULTRAHIGHPERFORMANCEMODE,		/*!< ultra high performance mode */
}protocol_LIS3MDL_XYaxisOperativeMode_enum;


/**
 * @enum protocol_LIS3MDL_ODR_enum
 * @brief this enum defines the values of the Output data rate
 */
typedef enum {
	PRTLIS3MDL_CTRLREG1_ODR_DOT625HZ 	= 0x00,					/*!< 0.625 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_1DOT25HZ 	,						/*!< 1.25 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_2DOT5HZ 	,						/*!< 2.5 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_5HZ 		,						/*!< 5 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_10HZ 		,						/*!< 10 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_20HZ 		,						/*!< 20 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_40HZ 		,						/*!< 40 HZ */
	PRTLIS3MDL_CTRLREG1_ODR_80HZ 								/*!< 80 HZ */
}protocol_LIS3MDL_ODR_enum;


/**
 * @enum protocol_LIS3MDL_FullSacale_enum
 * @brief this enum defines the values of the full scale
 */
typedef enum {
	PRTLIS3MDL_CTRLREG2_FULLSCALE_4GAUSS 	= 0x00	,				/*!< +/- 4 gauss */
	PRTLIS3MDL_CTRLREG2_FULLSCALE_8GAUSS 			,				/*!< +/- 8 gauss */
	PRTLIS3MDL_CTRLREG2_FULLSCALE_12GAUSS 			,				/*!< +/- 12 gauss */
	PRTLIS3MDL_CTRLREG2_FULLSCALE_16GAUSS 			,				/*!< +/- 16 gauss */
}protocol_LIS3MDL_FullSacale_enum;


/**
 * @enum protocol_LIS3MDL_SystemOperatingMode_enum
 * @brief this enum defines the values of thesystem operating mode
 */
typedef enum {
	PRTLIS3MDL_CTRLREG3_OPMODE_CONTINUOUS 	= 0x00	,				/*!< Continuous-conversion mode */
	PRTLIS3MDL_CTRLREG3_OPMODE_SINGLE 				,				/*!< Single-conversion mode Single-conversion mode has to be used with sampling frequency from 0.625 Hz to 80Hz.*/
	PRTLIS3MDL_CTRLREG3_OPMODE_POWERDOWN 			,				/*!< Power-down mode */
	PRTLIS3MDL_CTRLREG3_OPMODE_POWERDOWN2 							/*!< Power-down mode */
}protocol_LIS3MDL_SystemOperatingMode_enum;



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
}prtlis3mdl_controlregister1_structure;


/** @union prtlis3mdl_controlregister1_union
 *
 */
typedef union {
	uint8_t ui8controlReg1;
	prtlis3mdl_controlregister1_structure scontrolReg1;
}prtlis3mdl_controlregister1_union;


/** @struct prtlis3mdl_controlregister2
 *  @brief this structure is the 8 bits control register 2
 *  @var prtlis3mdl_controlregister2::b2RFUb0b1
 *  Member 'b2RFUb0b1' must be set to 0
 *  @var prtlis3mdl_controlregister2::bSoftReset
 *  Member 'bSoftReset' Configuration registers and user register reset function.
 *  @var prtlis3mdl_controlregister2::bReboot
 *  Member 'bReboot' Reboot memory content.
 *  @var prtlis3mdl_controlregister2::bRFUb4
 *  Member 'bRFUb4' must be set to 0
 *  @var prtlis3mdl_controlregister2::b2FullScaleConfoguration
 *  Member 'b2FullScaleConfoguration' Full-scale configuration.
 *  @var prtlis3mdl_controlregister2::bRFUb7
 *  Member 'bRFUb7' must be set to 0
 */
typedef struct {
	uint8_t b2RFUb0b1						: 2;
	uint8_t bSoftReset						: 1;
	uint8_t bReboot							: 1;
	uint8_t bRFUb4							: 1;
	uint8_t b2FullScaleConfoguration		: 2;
	uint8_t bRFUb7							: 1;
}prtlis3mdl_controlregister2_structure;


/** @union prtlis3mdl_controlregister2_union
 *
 */
typedef union {
	uint8_t ui8controlReg2;
	prtlis3mdl_controlregister2_structure scontrolReg2;
}prtlis3mdl_controlregister2_union;


/** @struct prtlis3mdl_controlregister3
 *  @brief this structure is the 8 bits control register 3
 *  @var prtlis3mdl_controlregister3::b2Operatingmode
 *  Member 'b2Operatingmode' Operating mode selection.
 *  @var prtlis3mdl_controlregister3::bSPImodeSelection
 *  Member 'bSPImodeSelection' SPI serial interface mode selection.
 *  @var prtlis3mdl_controlregister3::bRFUb3b4
 *  Member 'bRFUb3b4' must be set to 0
 *  @var prtlis3mdl_controlregister3::bLP
 *  Member 'bLP' low power mode configuration
 *  @var prtlis3mdl_controlregister3::b2RFUb6b7
 *  Member 'b2RFUb6b7'  must be set to 0
 */
typedef struct {
	uint8_t b2Operatingmode					: 2;
	uint8_t bSPImodeSelection				: 1;
	uint8_t b2RFUb3b4						: 2;
	uint8_t bLP								: 1;
	uint8_t b2RFUb6b7						: 2;
}prtlis3mdl_controlregister3_structure;


/** @union prtlis3mdl_controlregister3_union
 *
 */
typedef union {
	uint8_t ui8controlReg3;
	prtlis3mdl_controlregister3_structure scontrolReg3;
}prtlis3mdl_controlregister3_union;

/** @struct prtlis3mdl_controlregister4
 *  @brief this structure is the 8 bits control register 4
 *  @var prtlis3mdl_controlregister4::bRFUb0
 *  Member 'bRFUb0' must be set to 0
 *  @var prtlis3mdl_controlregister4::BigLittleEndianSelection
 *  Member 'BigLittleEndianSelection' Big/Little Endian data selection.
 *  @var prtlis3mdl_controlregister4::b2ZaxiOperatingMode
 *  Member 'b2ZaxiOperatingMode' Z-axis operating mode selection
 *  @var prtlis3mdl_controlregister4::b4RFUb4
 *  Member 'b4RFUb4' must be set to 0
 */
typedef struct {
	uint8_t bRFUb0							: 1;
	uint8_t BigLittleEndianSelection		: 1;
	uint8_t b2ZaxiOperatingMode				: 2;
	uint8_t b4RFUb4							: 4;
}prtlis3mdl_controlregister4_structure;


/** @union prtlis3mdl_controlregister4_union
 *
 */
typedef union {
	uint8_t ui8controlReg4;
	prtlis3mdl_controlregister4_structure scontrolReg4;
}prtlis3mdl_controlregister4_union;


/** @struct prtlis3mdl_controlregister5
 *  @brief this structure is the 8 bits control register 4
 *  @var prtlis3mdl_controlregister5::bRFUb0
 *  Member 'bRFUb0' must be set to 0
 *  @var prtlis3mdl_controlregister5::bBloackDataUpdate
 *  Member 'bBloackDataUpdate' Block data update for magnetic data. Default value: 0
 *  @var prtlis3mdl_controlregister5::bFastRead
 *  Member 'bFastRead' FAST READ allows reading the high part of DATA OUT only in order to increase
reading efficiency.
 */
typedef struct {
	uint8_t b6RFUb0							: 6;
	uint8_t bBloackDataUpdate				: 1;
	uint8_t bFastRead						: 1;
}prtlis3mdl_controlregister5_structure;


/** @union prtlis3mdl_controlregister5_union
 *
 */
typedef union {
	uint8_t ui8controlReg5;
	prtlis3mdl_controlregister5_structure scontrolReg5;
}prtlis3mdl_controlregister5_union;


/** @struct prtlis3mdl_IntConfig_structure
 *  @brief this structure is the 8 bits interupt register
 *  @var prtlis3mdl_IntConfig_structure::bIntEnable
 *  Member 'bIntEnable' Interrupt enable on INT pin. Default value 0.
 *  @var prtlis3mdl_IntConfig_structure::bLatchRequest
 *  Member 'bLatchRequest' enables the latch interrupt request.
 *  @var prtlis3mdl_IntConfig_structure::bInteruptActive (0: low; 1:high)
 *  Member 'bInteruptActive' interrupt active configuration on INT.
 *  @var prtlis3mdl_IntConfig_structure::b2RFU
 *  Member 'b2RFU' must be set to 0
 *  @var prtlis3mdl_IntConfig_structure::bZintEnable
 *  Member 'bZintEnable' Enable interrupt generation on Z-axis.
 *  @var prtlis3mdl_IntConfig_structure::bYintEnable
 *  Member 'bZintEnable' Enable interrupt generation on Y-axis.
 *  @var prtlis3mdl_IntConfig_structure::bXintEnable
 *  Member 'bZintEnable' Enable interrupt generation on X-axis.
 */
typedef struct {
	uint8_t bIntEnable						: 1;
	uint8_t bLatchRequest					: 1;
	uint8_t bInteruptActive					: 1;
	uint8_t b2RFU							: 2;
	uint8_t bZintEnable						: 1;
	uint8_t bYintEnable						: 1;
	uint8_t bXintEnable						: 1;
}prtlis3mdl_IntConfig_structure;


/** @union prtlis3mdl_IntConfig_union
 *
 */
typedef union {
	uint8_t ui8IntConfig;
	prtlis3mdl_IntConfig_structure sIntConfig;
}prtlis3mdl_IntConfig_union;


/** @struct prtlis3mdl_registers_struct
 *  @brief this structure contains the different register used by the LIS3MDL
 *  @note for further detail please refer to its data sheet
 *  @var prtlis3mdl_registers_struct::uCtrlReg1
 *  Member 'uCtrlReg1' is the control register 1
 *  @var prtlis3mdl_registers_struct::uCtrlReg2
 *  Member 'uCtrlReg2' is the control register 2
 *  @var prtlis3mdl_registers_struct::uCtrlReg3
 *  Member 'uCtrlReg3' is the control register 3
 *  @var prtlis3mdl_registers_struct::uCtrlReg4
 *  Member 'uCtrlReg4' is the control register 4
 *  @var prtlis3mdl_registers_struct::uCtrlReg5
 *  Member 'uCtrlReg5' is the control register 5

 *   */
typedef struct {
	prtlis3mdl_controlregister1_union 					uCtrlReg1;
	prtlis3mdl_controlregister2_union					uCtrlReg2;
	prtlis3mdl_controlregister3_union					uCtrlReg3;
	prtlis3mdl_controlregister4_union					uCtrlReg4;
	prtlis3mdl_controlregister5_union					uCtrlReg5;
	prtlis3mdl_IntConfig_union							uIntReg;
}prtlis3mdl_registers_struct;

/*===========================================================================================================
						public function
===========================================================================================================*/
void 	prtlis3mdh_init 							( void );
void 	prtlis3mdh_EnablePowerSuppply 				( bool bOnOrOff );
uint8_t prtlis3mdh_CheckDeviceId 					( void );

uint8_t prtlis3mdh_ReadRegister 					( const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToRead , uint8_t **pui8Registerfield);
uint8_t prtlis3mdh_WriteRegister 					( const uint8_t ui8MemoryAdr, const uint8_t ui8NbByteToWrite, uint8_t const * pui8DataToWrite);
void  	prtlis3mdh_InitiateMagneticFieldDetection 	( bool bOnOrOff );
void  	prtlis3mdh_InitiateMagneticFieldLog 		( bool bOnOrOff );

void  	prtlis3mdh_ReadXaxisField 					( int16_t *i16MagneticField);
void  	prtlis3mdh_ReadYaxisField 					( int16_t *i16MagneticField);
void  	prtlis3mdh_ReadZaxisField 					( int16_t *i16MagneticField);
void  	prtlis3mdh_Read3axisField 					( int16_t *i16XmagneticField, int16_t *i16YmagneticField, int16_t *i16ZmagneticField);
void 	prtlis3mdh_initDataReadypad 				( const bool bEnableOrDisable );
void 	prtlis3mdh_initIntpad		 				( const bool bEnableOrDisable );

#endif
