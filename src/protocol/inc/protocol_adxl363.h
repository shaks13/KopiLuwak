/*******************************************************************************
 * @file protocol_adxl363.h
 * @brief this function set is codec for the adxl363 device
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef PROTOCOL_ADXL363_H_
#define PROTOCOL_ADXL363_H_

#include "common_library.h"
#include "interface_spi.h"
#include "spidrv.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define PROTOCOL_ADXL363_BUFFERSIZE									0x10
#define PROTOCOL_ADXL363_NBBYTE_READ 								(2)
#define PROTOCOL_ADXL363_NBBYTE_WRITE 								(2)


/** ADXL362 Commands ***************************************************/

// page 19 of datasheet
#define PROTOCOL_ADXL363_COMMAND_WRITE_REGISTER						(0x0A)
#define PROTOCOL_ADXL363_COMMAND_READ_REGISTER						(0x0B)
#define PROTOCOL_ADXL363_COMMAND_READ_FIFO							(0x0D)


#define PROTOCOL_ADXL363_COMMAND_DATA_INDEX							(0x02)
#define PROTOCOL_ADXL363_RESETCODE									(0x52)


/** ADXL362 Registers **************************************************/

// page 23
#define PROTOCOL_ADXL363_REGISTER_DEVID_AD							(0x00)
#define PROTOCOL_ADXL363_REGISTER_DEVID_MST							(0x01)
#define PROTOCOL_ADXL363_REGISTER_PARTID							(0x02)
#define PROTOCOL_ADXL363_REGISTER_REVID								(0x03)
#define PROTOCOL_ADXL363_REGISTER_XDATA								(0x08)
#define PROTOCOL_ADXL363_REGISTER_YDATA								(0x09)
#define PROTOCOL_ADXL363_REGISTER_ZDATA								(0x0A)
#define PROTOCOL_ADXL363_REGISTER_STATUS							(0x0B)
#define PROTOCOL_ADXL363_REGISTER_FIFO_ENTRIES_L					(0x0C)
#define PROTOCOL_ADXL363_REGISTER_FIFO_ENTRIES_H					(0x0D)
#define PROTOCOL_ADXL363_REGISTER_XDATA_L							(0x0E)
#define PROTOCOL_ADXL363_REGISTER_XDATA_H							(0x0F)
#define PROTOCOL_ADXL363_REGISTER_YDATA_L							(0x10)
#define PROTOCOL_ADXL363_REGISTER_YDATA_H							(0x11)
#define PROTOCOL_ADXL363_REGISTER_ZDATA_L							(0x12)
#define PROTOCOL_ADXL363_REGISTER_ZDATA_H							(0x13)
#define PROTOCOL_ADXL363_REGISTER_TEMP_L							(0x14)
#define PROTOCOL_ADXL363_REGISTER_TEMP_H							(0x15)
// reserved
#define PROTOCOL_ADXL363_REGISTER_SOFT_RESET						(0x1F)
#define PROTOCOL_ADXL363_REGISTER_THRESH_ACT_L						(0x20)
#define PROTOCOL_ADXL363_REGISTER_THRESH_ACT_H						(0x21)
#define PROTOCOL_ADXL363_REGISTER_TIME_ACT							(0x22)
#define PROTOCOL_ADXL363_REGISTER_THRESH_INACT_L					(0x23)
#define PROTOCOL_ADXL363_REGISTER_THRESH_INACT_H					(0x24)
#define PROTOCOL_ADXL363_REGISTER_TIME_INACT_L						(0x25)
#define PROTOCOL_ADXL363_REGISTER_TIME_INACT_H						(0x26)
#define PROTOCOL_ADXL363_REGISTER_ACT_INACT_CTL						(0x27)
#define PROTOCOL_ADXL363_REGISTER_FIFO_CONTROL						(0x28)
#define PROTOCOL_ADXL363_REGISTER_FIFO_SAMPLES						(0x29)
#define PROTOCOL_ADXL363_REGISTER_INTMAP1							(0x2A)
#define PROTOCOL_ADXL363_REGISTER_INTMAP2							(0x2B)
#define PROTOCOL_ADXL363_REGISTER_FILTER_CTL						(0x2C)
#define PROTOCOL_ADXL363_REGISTER_POWER_CTL							(0x2D)
#define PROTOCOL_ADXL363_REGISTER_SELF_TEST							(0x2E)
// 0x2F - 0x3F are reserved for factory use

/** ADXL362 Id **************************************************/
#define PROTOCOL_REGISTER_XDATA_LENGTH								(0x02)
#define PROTOCOL_REGISTER_YDATA_LENGTH								(0x02)
#define PROTOCOL_REGISTER_ZDATA_LENGTH								(0x02)
#define PROTOCOL_REGISTER_TEMP_LENGTH								(0x02)
#define PROTOCOL_REGISTER_3XYZDATA_LENGTH							(0x03)
#define PROTOCOL_REGISTER_3XYZDATAEXT_LENGTH						(0x06)
#define PROTOCOL_REGISTER_THRESHOLD_LENGTH							(0x02)

#define PROTOCOL_REGISTER_POWER_LENGTH								(0x01)
#define PROTOCOL_REGISTER_FILTER_LENGTH								(0x01)
#define PROTOCOL_REGISTER_ACTIVITY_LENGTH							(0x01)
#define PROTOCOL_REGISTER_INTERUPTMAP1_LENGTH						(0x01)
#define PROTOCOL_REGISTER_INTERUPTMAP2_LENGTH						(0x01)
#define PROTOCOL_REGISTER_STATUS_LENGTH								(0x01)
#define PROTOCOL_REGISTER_TIMEACT_LENGTH							(0x01)
#define PROTOCOL_REGISTER_TIMEINACT_LENGTH							(0x02)


/** ADXL362 Id **************************************************/
#define PROTOCOL_ADXL363_ID											(0xAD)


/** ADXL362 Register Bit Masks *****************************************/

#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_ERR_USER_REGS			(1<<7)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_AWAKE					(1<<6)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_INACT					(1<<5)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_ACT					(1<<4)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_FIFO_OVERRUN			(1<<3)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_FIFO_WATERMARK		(1<<2)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_FIFO_READY			(1<<1)
#define PROTOCOL_ADXL363_REGISTER_MASK_STATUS_DATA_READY			(1<<0)

#define PROTOCOL_ADXL363_REGISTER_MASK_FIFO_ENTRIES_H				0x03 // 00000011_b

#define PROTOCOL_ADXL363_REGISTER_MASK_XDATA_H						0x0F // 00001111_b

#define PROTOCOL_ADXL363_REGISTER_MASK_YDATA_H						0x0F // 00001111_b

#define PROTOCOL_ADXL363_REGISTER_MASK_ZDATA_H						0x0F // 00001111_b

#define PROTOCOL_ADXL363_REGISTER_MASK_TEMP_H						0x0F // 00001111_b

#define PROTOCOL_ADXL363_REGISTER_MASK_THRESH_ACT_H					0x07 // 00000111_b

#define PROTOCOL_ADXL363_REGISTER_MASK_THRESH_INACT_H				0x07 // 00000111_b

#define PROTOCOL_ADXL363_REGISTER_MASK_ACT_INACT_CTL_RES			0xC0 // 110000_b
#define PROTOCOL_ADXL363_REGISTER_MASK_ACT_INACT_CTL_LINKLOOP		0x30 // 001100_b
#define PROTOCOL_ADXL363_REGISTER_MASK_ACT_INACT_CTL_INACT_REF		(1<<3)
#define PROTOCOL_ADXL363_REGISTER_MASK_ACT_INACT_CTL_INACT_EN		(1<<2)
#define PROTOCOL_ADXL363_REGISTER_MASK_ACT_INACT_CTL_ACT_REF		(1<<1)
#define PROTOCOL_ADXL363_REGISTER_MASK_ACT_INACT_CTL_ACT_EN			(1<<0)

#define PROTOCOL_ADXL363_REGISTER_MASK_FIFO_CONTROL_AH				(1<<3)
#define PROTOCOL_ADXL363_REGISTER_MASK_FIFO_CONTROL_FIFO_TEMP		(1<<2)
#define PROTOCOL_ADXL363_REGISTER_MASK_FIFO_CONTROL_FIFO_MODE		0x03 // 0000011_b

#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_INT_LOW				(1<<7)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_AWAKE					(1<<6)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_INACT					(1<<5)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_ACT					(1<<4)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_FIFO_OVERRUN			(1<<3)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_FIFO_WATERMARK		(1<<2)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_FIFO_READY			(1<<1)
#define PROTOCOL_ADXL363_REGISTER_MASK_INTMAP_DATA_READY			(1<<0)

#define PROTOCOL_ADXL363_REGISTER_MASK_FILTER_CTL_RANGE				0xC0 // 11000000_b
#define PROTOCOL_ADXL363_REGISTER_MASK_FILTER_CTL_RES				(1<<5)
#define PROTOCOL_ADXL363_REGISTER_MASK_FILTER_CTL_HALF_BW			(1<<4)
#define PROTOCOL_ADXL363_REGISTER_MASK_FILTER_CTL_EXT_SAMPLE		(1<<3)
#define PROTOCOL_ADXL363_REGISTER_MASK_FILTER_CTL_ODR				0x05 // 00000111_b

#define PROTOCOL_ADXL363_REGISTER_RESET_POWER						(0x00)
#define PROTOCOL_ADXL363_REGISTER_MASK_POWER_CTL_RES				(1<<7)
#define PROTOCOL_ADXL363_REGISTER_MASK_POWER_CTL_EXT_CLK			(1<<6)
#define PROTOCOL_ADXL363_REGISTER_MASK_POWER_CTL_LOW_NOISE			0x30 // 00110000_b
#define PROTOCOL_ADXL363_REGISTER_MASK_POWER_CTL_WAKEUP				(1<<3)
#define PROTOCOL_ADXL363_REGISTER_MASK_POWER_CTL_AUTOSLEEP			(1<<2)
#define PROTOCOL_ADXL363_REGISTER_MASK_POWER_CTL_MEASURE			0x03 // 0000011_b

#define PROTOCOL_ADXL363_REGISTER_RESET_FILTER						(0x00)
#define PROTOCOL_ADXL363_REGISTER_RESET_ACTIVITY					(0x00)
#define PROTOCOL_ADXL363_REGISTER_RESET_IRQ							(0x00)

/*===========================================================================================================
						Enum definition
===========================================================================================================*/

/**
 * @enum protocol_adxl363_lownoise
 * @brief this enum defines the low noise configuration of the power register
 */
typedef enum {
	PROTOCOL_ADXL363_LOWNOISE_NORMAL 	= 0x00,				/*!< Normal operation (reset default). */
	PROTOCOL_ADXL363_LOWNOISE_LOWNOISE ,					/*!< Low noise mode.*/
	PROTOCOL_ADXL363_LOWNOISE_ULTRALOWNOISE ,				/*!< Ultra low noise mode.*/
	PROTOCOL_ADXL363_LOWNOISE_RESERVED						/*!< reserved*/
}protocol_adxl363_lownoise;

/**
 * @enum protocol_adxl363_lownoise
 * @brief this enum defines the measure configuration of the power register
 */
typedef enum {
	PROTOCOL_ADXL363_MEASURE_STANDBY 	= 0x00,				/*!< Standby. */
	PROTOCOL_ADXL363_MEASURE_RESERVED ,						/*!< reserved*/
	PROTOCOL_ADXL363_MEASURE_MEASUREMENTMODE ,				/*!< Measurement mode.*/
	PROTOCOL_ADXL363_MEASURE_RESERVED2						/*!< reserved*/
}protocol_adxl363_measure;

/**
 * @enum protocol_adxl363_range
 * @brief this enum defines the measurement range selection available
 */
typedef enum {
	PROTOCOL_ADXL363_RANGE_2G 	= 0x00,						/*!< �2 g (reset default)*/
	PROTOCOL_ADXL363_RANGE_4G 	,							/*!< �4 g */
	PROTOCOL_ADXL363_RANGE_8G 								/*!< �8 g */
}protocol_adxl363_range;

/**
 * @enum protocol_adxl363_HalfBandwidth
 * @brief this enum defines the half band width
 */
typedef enum {
	PROTOCOL_ADXL363_HALFBW_OFF 	= 0x00,					/*!< the bandwidth of the filters is set to � the ODR for a wider bandwidth.*/
	PROTOCOL_ADXL363_HALFBW_ON 	,							/*!< the bandwidth of the antialiasing filters is set to � the output data rate (ODR) for more conservative filtering. */
}protocol_adxl363_HalfBandwidth;


/**
 * @enum protocol_adxl363_odr
 * @brief this enum defines the output data rates
 */
typedef enum {
	PROTOCOL_ADXL363_ODR_12HZ 	= 0x00,						/*!< 12.5 Hz*/
	PROTOCOL_ADXL363_ODR_25HZ 	,							/*!< 25 Hz */
	PROTOCOL_ADXL363_ODR_50HZ 	,							/*!< 50 Hz */
	PROTOCOL_ADXL363_ODR_100HZ 	,							/*!< 100 Hz*/
	PROTOCOL_ADXL363_ODR_200HZ	,							/*!< 200 Hz */
	PROTOCOL_ADXL363_ODR_400HZ 	,							/*!< 400 Hz */
}protocol_adxl363_odr;



/**
 * @enum protocol_adxl363_linkloop
 * @brief this enum defines the Link/Loop Mode Enable.
 */
typedef enum {
	PROTOCOL_ADXL363_LINKLOOP_DEFAULT 	= 0x00,				/*!< Activity and inactivity detection are both enabled, and their interrupts (if mapped) must be acknowledged by the host processor by reading the status register. Autosleep is disabled in this mode. Use this mode for free fall detection applications.*/
	PROTOCOL_ADXL363_LINKLOOP_LINKED  	,					/*!< Activity and inactivity detection are linked sequentially such that only one is enabled at a time. Their interrupts (if mapped) must be acknowledged by the host processor by reading the status register. */
	PROTOCOL_ADXL363_LINKLOOP_LOOP 	=0x03,					/*!< Activity and inactivity detection are linked sequentially such that only one is enabled at a time, and their interrupts are internally acknowledged (do not need to be serviced by the host processor). */
}protocol_adxl363_linkloop;

/**
 * @enum protocol_adxl363_Enable
 * @brief this enum defines the values to enable or disable acc to the ADL363 datasheet
 */
typedef enum {
	PROTOCOL_ADXL363_DISABLE 	= 0x00,				/*!< the value to disable a functionality in a register */
	PROTOCOL_ADXL363_ENABLE  	,					/*!< the value to enable a functionality in a register */
}protocol_adxl363_Enable;

/**
 * @enum protocol_adxl363_Enable
 * @brief this enum defines the values to select the reference or absolute mode acc to the ADL363 datasheet
 */
typedef enum {
	PROTOCOL_ADXL363_ABSOLUTEMODE 	= 0x00,				/*!< the value to disable a functionality in a register */
	PROTOCOL_ADXL363_REFERENCEMODE  	,				/*!< the value to enable a functionality in a register */
}protocol_adxl363_AsoluteOrReferenceMode;




/**
 * @enum protocol_adxl363_memorymap
 * @brief this enum defines the memory map of the adxl 363
 */
typedef enum {
	PROTOCOL_ADXL363MAP_DEVID_AD 	= 0x00,		/*!< This register contains the Analog Device */
	PROTOCOL_ADXL363MAP_DEVID_MST ,				/*!< This register contains the Analog Devices MEMS device ID*/
	PROTOCOL_ADXL363MAP_DEVID ,					/*!< This register contains the device ID*/
	PROTOCOL_ADXL363MAP_REVID ,					/*!< This register contains the product revision ID, beginning with 0x01 and incrementing for each subsequent revision*/
	PROTOCOL_ADXL363MAP_XDATA 		= 0x08 ,	/*!< This register holds the eight most significant bits of the x-axis acceleration data. */
	PROTOCOL_ADXL363MAP_YDATA ,					/*!< This register holds the eight most significant bits of the y-axis acceleration data. */
	PROTOCOL_ADXL363MAP_ZDATA ,					/*!< This register holds the eight most significant bits of the z-axis acceleration data. */
	PROTOCOL_ADXL363MAP_STATUS 		= 0x0B,		/*!< This register includes the following bits that describe various conditions of the ADXL363.*/
	PROTOCOL_ADXL363MAP_TEMPL		= 0x14,		/*!< These two registers contain the twos complement, sign extended (SX) temperature sensor output data. TEMP_L contains the eight LSBs,.*/
	PROTOCOL_ADXL363MAP_TEMPH,					/*!< and TEMP_H contains the four MSBs of the 12-bit value.*/
	PROTOCOL_ADXL363MAP_ADCL		= 0x16,		/*!< These two registers contain the twos complement, sign extended (SX) output of the auxiliary ADC*/
	PROTOCOL_ADXL363MAP_ADCH,					/*!< The sign extension bits (B[15:12], denoted as SX in the ADC_DATA_H bit map that follows) have the same value as the MSB (B11).*/
	PROTOCOL_ADXL363MAP_SOFTRESET  	= 0x1F,		/*!< Writing Code 0x52 (representing the letter, R, in ASCII or unicode) to this register immediately resets the ADXL363.*/
}protocol_adxl363_memorymap;

/**
 *  @enum protocol_adxl363_commandcode
 *	@brief this enum contains the Id of the command code of the ADXL 363
 */
typedef enum {
	PROTOCOL_ADXL363CMDCODE_WRITE = 0x0A,		/*!< This ID defines the write command*/
	PROTOCOL_ADXL363CMDCODE_READ = 0x0B,		/*!< This ID defines the read command*/
	PROTOCOL_ADXL363CMDCODE_FIFOREAD = 0x0D,	/*!< This ID defines the FIFO read command*/
}protocol_adxl363_commandcode;


/*===========================================================================================================
						structure definition
===========================================================================================================*/

/** @struct prtadxl363_power_struct
 *  @brief this structure is the 8 bits power register as defined in the ADXL363 datasheet
 *  @var prtadxl363_power_struct::bMeasure
 *  Member 'bMeasure' Selects Measurement Mode or Standby
 *  @var prtadxl363_power_struct::bautoSleep
 *  Member 'bautoSleep' Autosleep. Activity and inactivity detection must be in linked
 *  mode or loop mode (LINK/LOOP bits in ACT_INACT_CTL register) to enable autosleep;
 *  otherwise, the bit is ignored. See the Motion Detection section for details.
 *  @var prtadxl363_power_struct::bWakeUp
 *  Member 'bWakeUp' See the Operating Modes section for a detailed description of wake-up mode.
 *  @var prtadxl363_power_struct::b2lownoise
 *  Member 'b2lownoise' Selects Power vs. Noise Tradeoff:
 *  @var prtadxl363_power_struct::bExtClk
 *  Member 'bExtClk' External Clock.the accelerometer runs off the external clock provided on the INT1 pin.
 *  @var prtadxl363_power_struct::bAdcEnable
 *  Member 'bAdcEnable' ADC is enabled. The signal on the ADC input pin is converted and the corresponding
 *   digital value is made available in the ADC_DATA_H and ADC_DATA_L registers. Data in these registers
 *   is updated at the ODR selected in the FILTER_CTL register
 */
typedef struct {
	uint8_t bMeasure						: 2;
	uint8_t bautoSleep						: 1;
	uint8_t bWakeUp							: 1;
	uint8_t b2lownoise						: 2;
	uint8_t bExtClk							: 1;
	uint8_t bAdcEnable						: 1;
}prtadxl363_power_struct;


/** @struct prtadxl363_filter_struct
*  @brief this structure is the 8 bits filter register as defined in the ADXL363 datasheet
 *  @var prtadxl363_filter_struct::b3ODR
 *  Member 'b3ODR' Output Data Rate. Selects the ODR and configures internal filters to
 *  a bandwidth of � or � the selected ODR, depending on the HALF_BW bit setting.
 *  @var prtadxl363_filter_struct::bExtsample
 *  Member 'bExtsample' External Sampling Trigger. 1 = the INT2 pin is used for external conversion timing control.
 *  @var prtadxl363_filter_struct::bHalfBw
 *  Member 'bHalfBw' Halved Bandwidth.
 *  @var prtadxl363_filter_struct::bRFU
 *  Member 'bRFU' reserved for future use
 *  @var prtadxl363_filter_struct::brange
 *  Member 'brange' Measurement Range Selection.
 */
typedef struct {
	uint8_t b3ODR							: 3;
	uint8_t bExtsample						: 1;
	uint8_t bHalfBw							: 1;
	uint8_t bRFU							: 1;
	uint8_t brange							: 2;
}prtadxl363_filter_struct;

/** @struct prtadxl363_ActivityInactivityRegister_struct
*  @brief this structure is the 8 bits filter register as defined in the ADXL363 datasheet
 *  @var prtadxl363_ActivityInactivityRegister_struct::bActivityEnable
 *  Member 'bActivityEnable' enables the activity (overthreshold) functionality.
 *  @var prtadxl363_ActivityInactivityRegister_struct::bActivityReference
 *  Member 'bActivityReference' Activity Referenced/Absolute Select.
 *  @var prtadxl363_ActivityInactivityRegister_struct::bInactivityEnable
 *  Member 'bInactivityEnable' Inactivity Enable.
 *  @var prtadxl363_ActivityInactivityRegister_struct::bInactivityReference
 *  Member 'bInactivityReference' Inactivity Referenced/Absolute Select.
 *  @var prtadxl363_ActivityInactivityRegister_struct::b2LinkLoop
 *  Member 'b2LinkLoop' Link/Loop Mode Enable.
 *  @var prtadxl363_ActivityInactivityRegister_struct::b2RFU
 *  Member 'b2RFU' reserved for future used
 */
typedef struct {
	uint8_t bActivityEnable					: 1;
	uint8_t bActivityReference				: 1;
	uint8_t bInactivityEnable				: 1;
	uint8_t bInactivityReference			: 1;
	uint8_t b2LinkLoop						: 2;
	uint8_t b2RFU							: 2;
}prtadxl363_ActivityInactivityRegister_struct;


/** @struct prtadxl363_InteruptionRegister_struct
 *  @brief this structure is the 8 bits filter register as defined in the ADXL363 datasheet
 *  @var prtadxl363_InteruptionRegister_struct::bDataReady
 *  Member 'bDataReady' Data Ready Interrupt. 1 = maps the data ready status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::bfifoready
 *  Member 'bfifoready' FIFO Ready Interrupt. 1 = maps the FIFO ready status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::bfifowatermark
 *  Member 'bfifowatermark' FIFO Watermark Interrupt. 1 = maps the FIFO watermark status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::bfifooverrun
 *  Member 'bfifooverrun' FIFO Overrun Interrupt. 1 = maps the FIFO overrun status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::binactivity
 *  Member 'binactivity' Activity Interrupt. 1 = maps the activity status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::binacttivity
 *  Member 'binacttivity' Activity Interrupt. 1 = maps the inactivity status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::bawake
 *  Member 'bawake' Awake Interrupt. 1 = maps the awake status to INT1 pin.
 *  @var prtadxl363_InteruptionRegister_struct::bintlow
 *  Member 'bintlow' Interrupt Active Low. 1 = INT1 pin is active low.
 */
typedef struct {
	uint8_t bDataReady						: 1;
	uint8_t bfifoready						: 1;
	uint8_t bfifowatermark					: 1;
	uint8_t bfifooverrun					: 1;
	uint8_t bactivity						: 1;
	uint8_t binactivity						: 1;
	uint8_t bawake							: 1;
	uint8_t bintlow							: 1;
}prtadxl363_InteruptionRegister_struct;


/** @union prtadxl363_FilterRegister_union
 *
 */
typedef union {
	uint8_t ui8power;
	prtadxl363_power_struct spower;
}prtadxl363_PowerRegister_union;

/** @union prtadxl363_FilterRegister_union
 *
 */
typedef union {
	uint8_t ui8filter;
	prtadxl363_filter_struct sfilter;
}prtadxl363_FilterRegister_union;

/** @union prtadxl363_ActivityInactivityRegister_union
 *
 */
typedef union {
	uint8_t ui8activity;
	prtadxl363_ActivityInactivityRegister_struct sactivity;
}prtadxl363_ActivityInactivityRegister_union;

/** @union prtadxl363_InteruptionRegister_union
 *
 */
typedef union {
	uint8_t ui8interupt;
	prtadxl363_InteruptionRegister_struct sinterupt;
}prtadxl363_InteruptionRegister_union;


/** @struct prtadxl363_registers_struct
 *  @brief this structure contains the different register used by the ADXL
 *  @note for further detail please refer to its data sheet
 *  @var srvEM4325_configuration_struct::ui8power
 *  Member 'ui8power' is the power register
 *  @var srvEM4325_configuration_struct::ui8filter
 *  Member 'ui8filter' is the filter register
 */
typedef struct {
	prtadxl363_PowerRegister_union 					upower;
	prtadxl363_FilterRegister_union					ufilter;
	prtadxl363_ActivityInactivityRegister_union		uactivity;
	prtadxl363_InteruptionRegister_union			uinterupt;
}prtadxl363_registers_struct;



/** @struct prtadxl363_StatusRegister_struct
 *  @brief this structure is the 8 bits filter register as defined in the ADXL363 datasheet
 *  @var prtadxl363_StatusRegister_struct::berruserregs
 *  Member 'berruserregs' Error Detect. 1 indicates one of two conditions: either an SEU event,
 *   such as an alpha particle of a power glitch, has disturbed a user register setting or the
 *   ADXL363 is not configured. This bit is high upon both startup and soft reset, and resets as
 *    soon as any register write commands are performed.
 *  @var prtadxl363_StatusRegister_struct::bawake
 *  Member 'bawake' Indicates whether the accelerometer is in an active (awake = 1) or inactive
 *  (awake = 0) state, based on the activity and inactivity functionality. To enable autosleep,
 *   activity and inactivity detection must be in linked mode or loop mode (LINKLOOP bits in
 *   the ACT_INACT_CTL register); otherwise, this bit defaults to 1.
 *  @var prtadxl363_StatusRegister_struct::binactivity
 *  Member 'binactivity' Inactivity. 1 indicates that the inactivity detection function
 *   has detected an inactivity or a free fall condition.
 *  @var prtadxl363_StatusRegister_struct::bactivity
 *  Member 'bactivity' Activity. 1 indicates that the activity detection function
 *  has detected an overthreshold condition.
 *  @var prtadxl363_StatusRegister_struct::bfifooverrun
 *  Member 'bfifooverrun' FIFO Overrun. 1 indicates that the FIFO has overrun or overflowed,
 *  such that new data replaces unread data. See the Using FIFO Interrupts section for details.
 *  @var prtadxl363_StatusRegister_struct::bfifowatermark
 *  Member 'bfifowatermark' FIFO Watermark. 1 indicates that the FIFO contains at least the
 *  desired number of samples, as set in the FIFO_SAMPLES register. See the Using FIFO
 *   Interrupts section for details.
 *  @var prtadxl363_StatusRegister_struct::bfifoready
 *  Member 'bfifoready' FIFO Ready. 1 indicates that there is at least one sample available in the
 *  FIFO output buffer. See the Using FIFO Interrupts section for details.
 *  @var prtadxl363_StatusRegister_struct::bdataready
 *  Member 'bdataready' Data Ready. 1 indicates that a new valid sample is available to be read.
 *  This bit clears when a FIFO read is performed. See the Data Ready Interrupt section for more details.
 */
typedef struct {
	uint8_t bdataready						: 1;
	uint8_t bfifoready						: 1;
	uint8_t bfifowatermark					: 1;
	uint8_t bfifooverrun					: 1;
	uint8_t bactivity						: 1;
	uint8_t binactivity						: 1;
	uint8_t bawake							: 1;
	uint8_t berruserregs					: 1;
}prtadxl363_StatusRegister_struct;

/** @union prtadxl363_status_union
 *
 */
typedef union {
	uint8_t ui8status;
	prtadxl363_StatusRegister_struct sstatus;
}prtadxl363_status_union;

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/


/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
//void 	protocol_InitADXL363DMA	 	( void );
void 	prtadxl363_InitiateMotionDetection 			( void );
uint8_t prtadxl363_InitLogMeasurement			 	( const bool bOnOrOff , const protocol_adxl363_odr emeasfrequency);
void 	prtadxl363_EnableLowPowerMotionDetection 	( void );
void 	prtadxl363_DisableLowPowerMotionDetection 	( void );
void 	prtadxl363_EnableMotionIrq 					( const bool bOnorOff  );
void 	prtadxl363_EnableDataReadyIrq 				( const bool bOnorOff  );
void 	prtadxl363_Init 		 					( void );
void 	prtadxl363_Deinit 							( void );
uint8_t prtADXL363_CheckDeviceId 					( void );
void 	prtadxl363_ReadXYZaxis 						( uint8_t **pui8Response );
void 	prtadxl363_ReadXYZaxisExtended 				( uint8_t **pui8Response );
void 	prtadxl363_ReadTemperature 					( uint8_t *pui8Response );
void 	prtadxl363_ReadStatus 						( uint8_t *pui8Response );
void 	prtadxl363_SleepAdxl363 					( void );
void 	prtADXL363_Reset 							( void );

#endif /* PROTOCOL_ADXL363_H_ */
