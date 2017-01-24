/*******************************************************************************
 * @file srvEM4325.h
 * @brief this file defines the command set for the EM4325
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVEM4325_H_
#define SRVEM4325_H_

#include "common_library.h"
#include "protocol_EM4325.h"
#include "kernel_common.h"
#include "common_version.h"
#include "srvMeasTemperature.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
/* EM4325 Status */
#define SRVEM4325_MASK_STATUS_RESPONSE				(0x03)
#define SRVEM4325_MASK_STATUS_TRANSPONDER			(0x80)

#define SRVEM4325_COMMANDCODEBYTE					(0)
#define SRVEM4325_NBBYTEBYTE						(0)


#define SRVEM4325_SUCCESS_CODE						(0x80)
/* -------------- definition of the Read Write access to the memory  --------------------------------------- */
#if (COMMON_VERSION_FIRMWARE == 0x0120 )
#define SRVEM4325_SYSTEMFILE_ISAVAILABLE					0x07FD			/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_READACCESS						0x07FD			/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_WRITEACCESS					0x0000			/* defines the write access right for the system file */
#elif (COMMON_VERSION_FIRMWARE == 0x0121 )
#define SRVEM4325_SYSTEMFILE_ISAVAILABLE					0x03F1FFFD		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_READACCESS						0x03E17FFD		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_WRITEACCESS					0x03F0BF50		/* defines the write access right for the system file */
#elif (COMMON_VERSION_FIRMWARE == 0x0122 )
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#define SRVEM4325_SYSTEMFILE_ISAVAILABLE					0x03F1FFFD		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_READACCESS						0x03E17FFD		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_WRITEACCESS					0x03F0B650		/* defines the write access right for the system file */
#else /* (APP_CHOSEN_FLAG!= APP_GLUEPOT) */
#define SRVEM4325_SYSTEMFILE_ISAVAILABLE					0x03F1FFFD		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_READACCESS						0x03E17FFD		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_WRITEACCESS					0x03F0BF50		/* defines the write access right for the system file */
#endif /* end APP_CHOSEN_FLAG */
#else /* else (COMMON_VERSION_FIRMWARE ! = 0x0120 != 0x0121 != 0x0122)*/
#define SRVEM4325_SYSTEMFILE_ISAVAILABLE					0x0000			/* only the first four register exists*/
#define SRVEM4325_SYSTEMFILE_READACCESS						0x0000			/* read access for the first 4 register */
#define SRVEM4325_SYSTEMFILE_WRITEACCESS					0x0000			/* No Write access */
#endif	/* end COMMON_VERSION_FIRMWARE */

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
#define SRVEM4325_SYSTEMFILE_APPISAVAILABLE					0x0000003F		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_APPREADACCESS					0x0000003F		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_APPWRITEACCESS					0x0000003F		/* defines the write access right for the system file */
#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
#define SRVEM4325_SYSTEMFILE_APPISAVAILABLE					0x00010000		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_APPREADACCESS					0x00010000		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_APPWRITEACCESS					0x00000000		/* defines the write access right for the system file */
#elif (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)
#define SRVEM4325_SYSTEMFILE_APPISAVAILABLE					0x00000000		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_APPREADACCESS					0x00000000		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_APPWRITEACCESS					0x00000000		/* defines the write access right for the system file */
#else
#define SRVEM4325_SYSTEMFILE_APPISAVAILABLE					0x00000000		/* defines which word exist in the system file*/
#define SRVEM4325_SYSTEMFILE_APPREADACCESS					0x00000000		/* defines the read access right for the system file */
#define SRVEM4325_SYSTEMFILE_APPWRITEACCESS					0x00000000		/* defines the write access right for the system file */
#endif

/* -------------- Comm buffer -------------------------------------------------- */
#define SRVEM4325_NB_MAX_REGFILE_WORDS						(8)
#define SRVEM4325_NBBYTEPERWORDBYTE							(2)
#define SRVEM4325_STATUS_INDEX								(0x00)			/* index of the register file where the status word is located*/
#define SRVEM4325_PARAMETER_INDEX							(0x01)				/* index of the register file where the first word of parameter is located*/
#define SRVEM4325_COMMBUFFER_NB_MAX_PACKETS_TO_READ			(SRVEM4325_NB_MAX_REGFILE_WORDS - 1)

#define SRVEM4325_COMMBUFFER_SIZE_PACKET					(16) 			/* 1 packet = 1 word */
#define SRVEM4325_COMMBUFFER_BYTE_FACTOR_PACKET				(SRVEM4325_COMMBUFFER_SIZE_PACKET/8) /* */

/* "WHO ARE YOU" register : mask and shift */
#define SRVEM4325_SYSTEMFILE_FW_MAJOR_VERS					((COMMON_VERSION_FIRMWARE & 0x00F0) >> 4) /*0x02*/			/* Major version number */
#define SRVEM4325_SYSTEMFILE_FW_MINOR_VERS					((COMMON_VERSION_FIRMWARE & 0x000F) >> 0) /*0x01*/			/* Minor version number */

#define SRVEM4325_MASK_FW_MAJOR_VERS						(0x00F0)
#define SRVEM4325_MASK_FW_MINOR_VERS						(0x000F)
#define SRVEM4325_MASK_CROSSRFID_CONFIG						(0xF000)
#define SRVEM4325_SHIFT_FW_MAJOR_VERS						(4)
#define SRVEM4325_SHIFT_FW_MINOR_VERS						(0)
#define SRVEM4325_SHIFT_CROSSRFID_CONFIG					(12)


/* -------------- "Date and time" register  -------------------------------------------------- */
#define SRVEM4325_COMMBUFFER_DATE_NBWORD					(2)		/* the date is coded on 2 words*/
#define SRVEM4325_COMMBUFFER_TIME_NBWORD					(2)		/* the time is coded on 2 words*/

/* Nb parameters in the Acquisition system files */
#define SRVEM4325_SYSFILE_NBPARAM_ACQMODE_ACQPERIOD			(3)
#define SRVEM4325_SYSFILE_NBPARAM_ACQMODE_ACQTHRESHOLD		(3)
#define SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQEND			(5)
#define SRVEM4325_SYSFILE_NBPARAM_ACQBEGIN_ACQMODE			(3)


#define SRVEM4325_MASK_COMMBUFFERSEMAPHORE					(0xF000)

/* -------------- EM4325 Reset value -------------------------------------- */
#define SRVEM4325_RESET_VALUE								(0xB001)

/* -------------- EM4325 registers values -------------------------------------- */
#define SRVEM4325_DEFAULT_IOCONTROLWORD						(0x2660)			/* default value of the iO contol word (SRVEM4325_IOCTRLWORD_SIGNALINGCOMBUF)*/

/* TODO : 0x2xxx (Disable pull resistors) or 0xAxxx (Enable pull resistors) ?  on P3_CS, P2_SCLK, P1_MISO, and P0_MOSI */
#define SRVEM4325_IOCTRLWORD_SPISLAVE						(0x2600)			/* I/0 control word register = SPI Slave */
#define SRVEM4325_IOCTRLWORD_SIGNALINGCOMBUF				(0x2660)			/* I/0 control word register = signaling comm buffer semaphore */
#define SRVEM4325_IOCTRLWORD_RFMDMBYPASS					(0x2680)			/* I/0 control word register = RF MODEM Bypass */
#define SRVEM4325_IOCTRLWORD_RFMDMSHAREDLIMITED				(0x26C0)			/* I/0 control word register = RF MODEM Shared / Limited */
#define SRVEM4325_IOCTRLWORD_RFMDMSHAREDFULL				(0x26E0)			/* I/0 control word register = RF MODEM Shared / Full */
#define SRVEM4325_IOCTRLWORD_EPCSHARED						(0x2618)			/* I/0 control word register = RF MODEM Shared / Full */

/* BAP word management word 1 */
#define SRVEM4325_BAPCTRLWORD1_DEFAULTSENSITIV 				(0xE001)
#define SRVEM4325_BAPCTRLWORD1_CROSSSENSITIV				(0xC000)

/* BAP word management word 2 for RF modem shared */
#define SRVEM4325_BAPCTRLWORD2_AUXEVENTRFSTATE				(0x0003)

#define SRVEM4325_SYSTEMFILE_ENABLEALARM_DEF_VALUE  		(uint16_t)(0x0007)
#define SRVEM4325_SYSTEMFILE_LOWTEMPTHRESHOLD_DEF_VALUE  	(uint16_t)(0)
#define SRVEM4325_SYSTEMFILE_HIGHTEMPTHRESHOLD_DEF_VALUE  	(uint16_t)(100)
/*===========================================================================================================
						Enum definition
===========================================================================================================*/
/**
 *  @enum protocol_em4225byte_enum
 *  @brief
 */
typedef enum {
	SRVEM4325_STATUS_SUCCCESS			=	0x00,			/*!<  ACK (command executed) */
	SRVEM4325_STATUS_INVALID_CMD 		=	0x01,			/*!<  NACK (invalid command) */
	SRVEM4325_STATUS_CMD_FAILED 		=	0x02,			/*!<  NACK (command failed) */
	SRVEM4325_STATUS_MEMORY_LOCKED 		=	0x03			/*!<  NACK (memory locked) */
}protocol_em4225byte_enum;

/**
 *  @enum srvEM4325_ReadWriteAccess_enum
 *  @brief this enum define the Read  or Write Acesss fo the system file
 */
typedef enum {
	SRVEM4325_SYSTEMFILE_READACCESSBITFIELD = 0b0,			/*!< the read access of the word of the system file */
	SRVEM4325_SYSTEMFILE_WRITEACCESSBITFIELD ,				/*!< the write access of the word of the system file  */
}srvEM4325_ReadWriteAccess_enum;

/**
 *  @enum 	srvEM4325_Handshake_enum
 *  @brief 	this enum defines the communication way and it's used as
 *  a handshake
 */
typedef enum {
	SRVEM4325_HANDSHAKE_RFHOSTTOCROSS = 0b0,	/*!< the communication way is from the RF host to the �c*/
	SRVEM4325_HANDSHAKE_CROSSTORFHOST ,			/*!< the communication way is from the �c to the RF host*/
}srvEM4325_Handshake_enum;

/**
 *  @enum 	srvEM4325_SpiSlaveExtensionMode_enum
 *  @brief	this enum contains the functionality Id of the EM4325 when it is
 *  configured in SPI Slave Extension.
 *
 */
typedef enum {
	SRVEM4325_SIGNALING_COMM_BUFFER = 0,	/*!< The EM4325 behaves as a tag and a specific IRQ wuhen the RF host writes in the register file*/
	SRVEM4325_SPISLAVE ,					/*!< The EM4325 behaves as a tag and as SPI slave*/
	SRVEM4325_SPIMASTER ,					/*!< The EM4325 behaves as a tag and as SPI master*/
	SRVEM4325_RF_MODEM_BYPASS,				/*!< The EM4325 behaves as an AFE*/
	SRVEM4325_RF_MODEM_SHARED_LIMITED,		/*!< The EM4325 behaves as a tag until the open state and an AFE after */
	SRVEM4325_RF_MODEM_SHARED_FULL,			/*!< The EM4325 behaves as a tag until the open state and an AFE after */
	SRVEM4325_UID_SHARED,					/*!< The EM4325 behaves as a tag with the EPC memory shared */
	SRVEM4325_ASBAP							/*!< The EM4325 behaves as a tag */
}srvEM4325_SpiSlaveExtensionMode_enum;

/**
 *  @enum	srvEM4325_SPImembank_enum
 *  @brief 	this enum contains the SPI memory map
 */
typedef enum {
	SRVEM4325_MEMMAP_RESERVED =				0x00,			/*!< reserved bank */
	SRVEM4325_MEMMAP_TID =					0x04,			/*!< TID bank */
	SRVEM4325_MEMMAP_EPC =					0x14,			/*!< EPC bank */
	SRVEM4325_MEMMAP_USER =					0x2C,			/*!< User bank */
	SRVEM4325_MEMMAP_IOCONTROLWORD =		0xF0,			/*!< I/O Control Word */
	SRVEM4325_MEMMAP_BATTERYMGMTWORD1 =		0xF1,			/*!< Battery Management Word 1 */
	SRVEM4325_MEMMAP_BATTERYMGMTWORD2 =		0xF2,			/*!< Battery Management Word 2 */
	SRVEM4325_MEMMAP_SENSORDATA 	=		0x100,			/*!< SesnsorData Word 1 */
	SRVEM4325_MEMMAP_SENSORDATAW2 				 ,			/*!< SesnsorData Word 2 */
	SRVEM4325_MEMMAP_REGISTERFILE =			0x104,			/*!< register file */
}srvEM4325_SPImembank_enum;

/**
 *  @enum 	srvEM4325_SpiConfig_enum
 *  @brief	this enum contains the different configuration of the EM4325's SPI
 *  configured in SPI Slave Extension.
 *
 */
typedef enum {
	SRVEM4325_SPICONFIG_DISABLED = 0,					/*!< The SPI of the EM4325 behaves is disabled*/
	SRVEM4325_SPICONFIG_SPISLAVE,						/*!< The SPI of the EM4325 behaves is configures as a slave*/
	SRVEM4325_SPICONFIG_SPIMASTER_FULLDUPLEX,			/*!< The SPI of the EM4325 behaves is configures as a master using full-duplex communications */
	SRVEM4325_SPICONFIG_SPIMASTER_HALFDUPLEX,			/*!< The SPI of the EM4325 behaves is configures as a master using full-duplex communications */
}srvEM4325_SpiConfig_enum;


/**
 *  @enum 	srvEM4325_status_enum
 *  @brief	this enum define the status of the register file
 *
 */
typedef enum {
	SRVEM4325_REGFILE_STATUSKO = 0b0,		/*!< the status is an successful code*/
	SRVEM4325_REGFILE_STATUSOK,				/*!< the status is not successful code*/
}srvEM4325_status_enum;

/**
 *  @enum	srvEM4325_auxpadconfiguration_enum
 *  @brief	this enum contains the different configuration of the AUX pad
 *  configured in SPI Slave Extension.
 *
 */
typedef enum {
	SRVEM4325_AUXPAD_RFFIELD = 0b00,			/*!< field is present meaning the device state is not Sleep*/
	SRVEM4325_AUXPAD_INVENTORYONGOING,			/*!<Device is participating in the current inventory round meaning the device state is Arbitrate, Reply/TagMsg, Acknowledged, Open, or Secured*/
	SRVEM4325_AUXPAD_SINGULATED,				/*!< Device is singulated meaning the device state is Acknowledged, Open, or Secured */
	SRVEM4325_AUXPAD_FLAGSET,					/*!< Device is selected meaning the Select Flag is set. The signal is gated in Sleep state and during the Boot Sequence. */
}srvEM4325_auxpadconfiguration_enum;

/**
 *  @enum 	srvEM4325_statuscode_enum
 *  @brief 	this enum contains the Id of the status code
  *
 */
typedef enum {
	SRVEM4325_STATUSCODE_SUCCESS = 0x00,			/*!< This Id is the success status code */
	SRVEM4325_ERRORCODE_GENERIC					/*!< This Id is last one of this enum*/
}srvEM4325_statuscode_enum;

/**
 *  @enum	srvEM4325_SelectMemoryCode_enum
 *  @brief 	this enum contains the Id of the SelectMemory register file that defines
 *  where the samples will be send during a download operation
  *
 */
typedef enum {
	SRVEM4325_SYSFILE_SELECTMEM_REGFILE = 0x00,		/*!< To select the RegisterFile memory */
	SRVEM4325_SYSFILE_SELECTMEM_USERMEM,			/*!< To select the UserMemory memory */
	SRVEM4325_SYSFILE_SELECTMEM_REGFILEOPTIMIZE,	/*!< To select the RegisterFile memory without the handshake  */
	SRVEM4325_SYSFILE_SELECTMEM_LAST
}srvEM4325_SelectMemoryCode_enum;

/**
 *  @enum 	srvEM4325_RegisterAddress_enum
 *  @brief	this enum contains the Id of the operation code ( command or response)
  *
 */
typedef enum {
	SRVEM4325_REGISTERADDR_WHOAREYOU	= (0x00),		/*!< This Id is the register address to retrieve the configuration of the application */
	SRVEM4325_REGISTERADDR_HOWAREYOU	,				/*!< This Id is the register address to retrieve the battery state of the application */
	SRVEM4325_REGISTERADDR_HOWOLDAEYOU,					/*!< This Id is the register address to retrieve the lifetime of the board */
	SRVEM4325_REGISTERADDR_HOWWARMISIT,					/*!< This Id is the register address to measure the temperature*/
	SRVEM4325_REGISTERADDR_TIME ,						/*!< This Id is the register address to measure the current time */
	SRVEM4325_REGISTERADDR_TIME2,						/*!< This Id is the register address to measure the current time */
	SRVEM4325_REGISTERADDR_DATE,						/*!< This Id is the register address to measure the current date */
	SRVEM4325_REGISTERADDR_DATE2,						/*!< This Id is the register address to measure the current date */
	SRVEM4325_REGISTERADDR_ACQMODE,						/*!< This Id is the register address to configure or get the acquisition mode for one sensor */
	SRVEM4325_REGISTERADDR_ACQPERIOD,					/*!< This Id is the register address to configure or get the acquisition period for one sensor */
	SRVEM4325_REGISTERADDR_ACQBEGIN,					/*!< This Id is the register address to configure or get the beginning of acquisitions for one sensor */
	SRVEM4325_REGISTERADDR_ACQEND,						/*!< This Id is the register address to configure or get the end of acquisitions for one sensor */
	SRVEM4325_REGISTERADDR_ACQTHRESHOLD,				/*!< This Id is the register address to configure or get the alarm thresholds */
	SRVEM4325_REGISTERADDR_ENABLEALARM ,				/*!< This Id is the register address to know if an alarm is enable */
	SRVEM4325_REGISTERADDR_STATEALARM,					/*!< This Id is the register address to know the alarm status */
	SRVEM4325_REGISTERADDR_RESETALARM,					/*!< This Id is the register address to reset the alarm status */
	SRVEM4325_REGISTERADDR_DATETIMEALARM,				/*!< This Id is the register address to know the rtcc dates/times when the alarms have been triggered */
	SRVEM4325_REGISTERADDR_RESET = (0x14) ,				/*!< This Id is the register address to reset the board and load the default configuration*/
	SRVEM4325_REGISTERADDR_ISDATAAVAILABLE,				/*!< This Id is the register address to know how many data is available*/
	SRVEM4325_REGISTERADDR_GIVESAMPLES , 				/*!< This Id is the register address to send to the RF host some stored data in the application (RAM or external mememory)*/
	SRVEM4325_REGISTERADDR_IDXSAMPLE , 					/*!< This Id is the register address to define the address of the first sample to download in the GiveMeData system file */
	SRVEM4325_REGISTERADDR_NBSAMPLETOGIVE ,				/*!< This Id is the register address to define the number of samples to download in the GiveMeData system file */
	SRVEM4325_REGISTERADDR_SELECTMEMORY ,				/*!< This Id is the register address to define the memory which going to store the samples */
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
	SRVEM4325_REGISTERADDR_HOWMUCHTIMEYOURUN = (0x20),	/*!< This Id is the register address to read or write the number of hour the system is running*/
	SRVEM4325_REGISTERADDR_HOWMANYTIMEYOURUN ,			/*!< This Id is the register address to read or write the number of time the system has started*/
	SRVEM4325_REGISTERADDR_CALIBRATION,					/*!< This Id is the register address to read or write the calibration of the sensor*/
	SRVEM4325_REGISTERADDR_LOG,							/*!< This Id is the register address to start a continuous measurement or the status of a continuous measurement*/
	SRVEM4325_REGISTERADDR_COMPUTEFFT,					/*!< This Id is the register address to compute a FFT*/
	SRVEM4325_REGISTERADDR_ENABLEACTIVITY,				/*!< This Id is the register address to set or get the activity counters*/
#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
	SRVEM4325_REGISTERADDR_GPGIVEMELIFE = (0x30),		/*!< This Id is the system file address to read the absolute shelf life */
#endif
	SRVEM4325_REGISTERADDR_LAST, 						/*!< This Id is last one of this enum*/
	SRVEM4325_REGISTERADDR_UNEXPECTED =(0x7F)
}srvEM4325_RegisterAddress_enum;
/*===========================================================================================================
						struct definition
===========================================================================================================*/

/** @struct srvEM4325_BatteryMgmtBitField_struct
 *  @brief this union regroups the 2 words (16 bits) of the the battery management
 *  @var protocol_BatteryMgmt_struct::b2RFfieldDetectotDutyCycle
 *  Member 'b2RFfieldDetectotDutyCycle' is the percentage of time where the rf detector is on
 *  0b00 : 100 % (always on)
 *  0b01 : 50 %
 *  0b10 : 25 %
 *  0b11 : 12.5 %
 *  @var srvEM4325_BatteryMgmtBitField_struct::b2RFfade
 *  Member 'b2RFfade' Amount of time that the field is no longer detected before an Active to
 *  Sleep transition will occur. Field detection for RF fade control is only performed when the
 *  device is not processing an RF command and the timing operation is reset with every RF command.
 *  RF fade control times :
 *  0b00 : 125 �s
 *  0b01 : 1 ms
 *  0b10 : 10 ms
 *  0b11 : 100ms
 *  @var srvEM4325_BatteryMgmtBitField_struct::b4InitialIdleTimeout
 *  Member 'b4InitialIdleTimeout'
 *  @var srvEM4325_BatteryMgmtBitField_struct::b2IdeTimeoutUnit
 *  Member 'b2IdeTimeoutUnit'
 *  0b00 : 10 ms
 *  0b01 : 1 s
 *  0b10 : 4 s
 *  0b11 : 64 s
 *  @var srvEM4325_BatteryMgmtBitField_struct::b2BAPsensitivity
 *  Member 'b2BAPsensitivity'
 *  0b00: best (maximum) sensitivity,
 *  0b01: default sensitivity,
 *  0b10: degraded sensitivity,
 *  0b11: most degraded (minimum) sensitivity
 *  @var srvEM4325_BatteryMgmtBitField_struct::b2Auxevent
 *  Member 'b2Auxevent'
 *  0b00: RF field is present meaning the device state is not Sleep
 *  0b01: Device is participating in the current inventory round meaning the device state is Arbitrate, Reply/TagMsg, Acknowledged, Open, or Secured
 *  0b10: Device is singulated meaning the device state is Acknowledged, Open, or Secured
 *  0b11: Device is selected meaning the Select Flag is set. The signal is gated in Sleep state and during the Boot Sequence.
 *  @var srvEM4325_BatteryMgmtBitField_struct::b1RTFIdleTimeout
 *  Member 'b1RTFIdleTimeout'
 *  0b0: RTF Idle timeout is disabled,
 *  0b1: RTF Idle timeout is enabled
 *  @var srvEM4325_BatteryMgmtBitField_struct::b3Totalmute
 *  Member 'b3Totalmute' When TOTAL is enabled,
 *  0b0: TOTAL Mute timeout is disabled,
 *  0b1: TOTAL Mute timeout is enabled
 *  @var srvEM4325_BatteryMgmtBitField_struct::b3SleepTimout
 *  Member 'b3SleepTimout' when BAP Mode is enabled and Idle Timeout is non-zero,
 *  0b000: Duty cycle control disabled,
 *  0b001: Duty cycle control enabled, Sleep Timeout = Idle Timeout,
 *  0b010: Duty cycle control enabled, Sleep Timeout = 2X Idle Timeout,
 *  0b011: Duty cycle control enabled, Sleep Timeout = 4X Idle Timeout,
 *  0b100: Duty cycle control enabled, Sleep Timeout = 8X Idle Timeout,
 *  0b101: Duty cycle control enabled, Sleep Timeout = 16X Idle Timeout,
 *  0b110: Duty cycle control enabled, Sleep Timeout = 32X Idle Timeout,
 *  0b111: Duty cycle control enabled, Sleep Timeout = 64X Idle Timeout
 *  @var protocol_BatteryMgmt_struct::b6nbTotalTagMsg
 *  Member 'b6nbTotalTagMsg' Number of TagMsg�s to transmit before self muting occurs.
 *  @var protocol_BatteryMgmt_struct::b1AlarmBlink
 *  Member 'b1AlarmBlink' When SPI Config is �0� and Alarms Out is �1� and the I/O pins are enabled for outputs,
 *  0b0: Alarm outputs are continuous and active low signals,
 *  0b1: Alarm outputs are active low pulses approximately 40 ms in duration and occurring approximately every 8 seconds
 *  @var srvEM4325_BatteryMgmtBitField_struct::b1LDBlevel
 *  Member 'b1LDBlevel'
 *  0b0: LBD level = 1.3V
 *  0b1: LBD level = 2.2V
 *  @var srvEM4325_BatteryMgmtBitField_struct::b1BAPctrl
 *  Member 'b1BAPctrl'
 *  0b0: BAP control disabled
 *  0b1: BAP control enabled
 */
typedef struct {
	uint32_t b1BAPctrl									: 1;
	uint32_t b1LDBlevel									: 1;
	uint32_t b1AlarmBlink								: 1;
	uint32_t b6nbTotalTagMsg							: 6;
	uint32_t b3SleepTimout								: 3;
	uint32_t b3Totalmute								: 1;
	uint32_t b1RTFIdleTimeout							: 1;
	uint32_t b2Auxevent									: 2;
	uint32_t b2BAPsensitivity 							: 2;
	uint32_t b4IdelTimoutActiveToSelep 					: 4;
	uint32_t b2IdeTimeoutUnit 							: 2;
	uint32_t b4InitialIdleTimeout 						: 4;
	uint32_t b2RFfade 									: 2;
	uint32_t b2RFfieldDetectotDutyCycle 				: 2;
}srvEM4325_BatteryMgmtBitField_struct;

/** @struct srvEM4325_BatteryMgmtWordx_struct
 *  @brief this union regroups the 2 words (16 bits) of the the battery management
 *  @var srvEM4325_BatteryMgmtWordx_struct::u32Word1
 *  @var srvEM4325_BatteryMgmtWordx_struct::u32Word2
 */
typedef struct {
	uint32_t u32Word2						: 16;
	uint32_t u32Word1						: 16;
}srvEM4325_BatteryMgmtWordx_struct;

/** @struct protocol_IOcontrol_struct
 *  @brief this union regroups the 2 words (16 bits) of the the battery managemen
 *  @var protocol_IOcontrol_struct::b1PullEnable
 *  Member 'b1PullEnable'
 *  0b0: Disable pull resistors on P3_CS, P2_SCLK, P1_MISO, and P0_MOSI,
 *  0b1: Enable pull resistors on P3_CS, P2_SCLK, P1_MISO, and P0_MOSI when they are enabled as inputs
 *  @var protocol_IOcontrol_struct::b2SpiConfig
 *  Member 'b2SpiConfig'
 *  0b0: SPI interface disabled,
 *  0b1: SPI interface enabled as SPI Slave using half-duplex communications,
 *  0b10: SPI interface enabled as SPI Master using full-duplex communications,
 *  0b11: SPI interface enabled as SPI Master using half-duplex communications
 *  @var protocol_IOcontrol_struct::b1SpiCPOL
 *  Member 'b1SpiCPOL'
 *  The SPI CPOL bit and the SPI CPHA bit are used to define the behaviour of SCLK
 *  and when data is latched with respect to SCLK. If the phase of the clock is zero (CPHA is �0�),
 *   data is latched at the rising edge of SCLK when CPOL is �0� and at the falling edge of SCLK when CPOL is �1�.
 *   If the phase of the clock is one (CPHA is �1�), data is latched at the rising edge of SCLK when CPOL is �1� and at
 *   the falling edge of SCLK when CPOL is �0�. The combination of the two bits is also known as the SPI Mode and defined
 *   as follows:
 *   <table>
 *   <caption id="multi_row">version</caption>
 *   <tr><th>SPI mode                      <th>CPOL        <th>CPHA
 *   <tr><td>0<td>0<td>0
 *   <tr><td>1<td>0<td>1
 *   <tr><td>2<td>1<td>0
 *   <tr><td>3<td>1<td>1
 *  @var protocol_IOcontrol_struct::b1SpiCPHA
 *  Member 'b1SpiCPHA'
 *  @var protocol_IOcontrol_struct::b1AuxEn
 *  Member 'b1AuxEn'
 *  0b0: Aux function is for tamper detection when device is not an SPI Slave and tamper test signal is output on MOSI pin and input on AUX pin,
 *  0b1: Aux function is for an RF event condition and output on AUX pin
 *  @var protocol_IOcontrol_struct::b1Auxout
 *  Member 'b1Auxout'
 *  0b0: Aux function disabled (HI-Z state on AUX pin)
 *  0b1: Aux function enabled
 *  @var protocol_IOcontrol_struct::b1Alarmout
 *  Member 'b1Alarmout'
 *  When SPI Config is �0� and an I/O pin is enabled for output:
 *  0b0: Output for the I/O pin is from the I/O Word,
 *  0b1: Output for the I/O pin is for an alarm condition
 *  P3 = Temperature Alarm (Under Temp OR Over Temp),
 *  P2 = Aux Alarm,
 *  P1 = No Alarms,
 *  P0 = Tamper test signal when Aux function enabled for tamper detection,
 *  Output in I/O Word when Aux function not enabled for tamper detection
*/
typedef struct {
	uint16_t b8SpiSlaveconfig 							: 8;
	uint16_t b1Alarmout			 						: 1;
	uint16_t b1Auxout			 						: 1;
	uint16_t b1AuxEn			 						: 1;
	uint16_t b1SpiCPHA			 						: 1;
	uint16_t b1SpiCPOL			 						: 1;
	uint16_t b2SpiConfig 								: 2;
	uint16_t b1PullEnable			 					: 1;
}srvEM4325_IOcontrol_struct;

/** @union srvEM4325_IOcontrol_union
 *
 */
typedef union {
	uint16_t ui16IOcontrolWord;
	srvEM4325_IOcontrol_struct sIOcontrol;
}srvEM4325_IOcontrol_union;


/** @struct srvEM4325_BatteryMgmt_union
 *
 */
typedef union {
	srvEM4325_BatteryMgmtWordx_struct sBatteryMgmt;
	srvEM4325_BatteryMgmtBitField_struct sbBatteryMgmtbitfield;
}srvEM4325_BatteryMgmt_union;

/** @struct srvEM4325_configuration_struct
 *  @brief this structure contains software global variables
 *  according to the EM4325 configuration
 *  @var srvEM4325_configuration_struct::eMode
 *  Member 'eMode' is the mode of the EM4325 ( tag or AFE or a mix)
 *  @var srvEM4325_configuration_struct::eSpiConfig
 *  Member 'eSpiConfig' is the EM4325's spi configuration
 */
typedef struct {
	srvEM4325_SpiSlaveExtensionMode_enum 	eMode;
	srvEM4325_SpiConfig_enum 				eSpiConfig;
	srvEM4325_BatteryMgmt_union 			uBatteryMgmt;
	srvEM4325_IOcontrol_union 				uIOcontrol;
}srvEM4325_configuration_struct;

/** @struct srvEM4325_commParams_Type
 *  @brief this structure contains the RFID communication parameters
 *  @var srvEM4325_commParams_Type::ui8BackscatterSettings
 *  Backscatter settings are valid meaning the device is participating in the current inventory round
 *  @var srvEM4325_commParams_Type::ui8Session
 *  Member 'ui8FlagSettings' is the Session
 *  @var srvEM4325_commParams_Type::ui8TRtext
 *  Using a the Pilot tone or not
 *  @var srvEM4325_commParams_Type::ui8DataEncoding
 *  Member 'ui8DataEncoding' is the Session Miller Data Encoding
 *  @var srvEM4325_commParams_Type::ui8FlagSettings
 *  Member 'ui8FlagSettings' is
 *  @var srvEM4325_commParams_Type::ui16RN16
 *  Member 'ui16RN16' is the RN16
 *  @var srvEM4325_commParams_Type::ui16Handle
 *  Member 'ui16Handle' is the handle
 */
typedef struct {
	uint8_t ui8BackscatterSettings;
	uint8_t ui8Session;
	uint8_t ui8TRtext;
	uint8_t ui8DataEncoding;
	uint8_t ui8FlagSettings;
	uint16_t ui16RN16;
	uint16_t ui16Handle;
}srvEM4325_commParams_Type;

/** @struct srvEM4325_CommandBitField_Type
 *  @var srvEM4325_CommandBitField_Type::ui16Rfu
 *  Member 'ui16Rfu' is not usable
 *  @var srvEM4325_CommandBitField_Type::ui16Handshake
 *  Member 'ui16Handshake' is the sender [0 (RF reader), 1 (Tag)]
 *  @var srvEM4325_CommandBitField_Type::ui16Handshake
 *  Member 'ui16ReadOrWrite' is the type of operation [0 (Read), 1 (Write)]
 *  @var srvEM4325_CommandBitField_Type::ui16RegisterAddress
 *  Member 'ui16RegisterAddress' is the logical address of the register
 *  @var srvEM4325_CommandBitField_Type::ui16NbWordsInParams
 *  Member 'ui16NbWordsInParams' is the number of words in the parameters of the command
 */
typedef struct {
	uint16_t ui16NbWordsInParams : 3;	/* LSB */
	uint16_t ui16RegisterAddress: 7;
	uint16_t ui16ReadOrWrite: 1;
	uint16_t ui16Handshake : 1;
	uint16_t ui16Rfu : 4;				/* MSB */
}srvEM4325_CommandBitField_Type;

/** @union srvEM4325_CmdFirstWord_union
 *
 */
typedef union {
	srvEM4325_CommandBitField_Type sBitsField;
	uint16_t ui16Value;
}srvEM4325_CmdFirstWord_union;

/** @struct srvEM4325_ResponseBitField_Type
 *  @var srvEM4325_CommandBitField_Type::ui16Rfu
 *  Member 'ui16Rfu' is not usable
 *  @var srvEM4325_CommandBitField_Type::ui16Handshake
 *  Member 'ui16Handshake' is the sender [0 (RF reader), 1 (Tag)] ( see srvEM4325_ReadWriteAccess_enum)
 *  @var srvEM4325_CommandBitField_Type::ui16Status
 *  Member 'ui16Status' is the status of the process [0 (process failed), 1 (successful process)]
 *  @var srvEM4325_ResponseBitField_Type::ui16RegisterAddress
 *  Member 'ui16RegisterAddress' is the logical address of the register
 *  @var srvEM4325_ResponseBitField_Type::ui16NbWordsInParams
 *  Member 'ui16NbWordsInParams' is the number of words in the parameters of the command
 */
typedef struct {
	uint16_t ui16NbWordsInParams : 3;	/* LSB */
	uint16_t ui16RegisterAddress: 7;
	uint16_t ui16Status : 1;
	uint16_t ui16Handshake : 1;
	uint16_t ui16Rfu : 4;				/* MSB */
}srvEM4325_ResponseBitField_Type;

/** @union srvEM4325_RespFirstWord_union
 *
 */
typedef union {
	srvEM4325_ResponseBitField_Type sBitsField;
	uint16_t ui16Value;
}srvEM4325_RespFirstWord_union;

/**
 * @struct this structure is the contain of the acquisition system files
 *  @var srvEM4325_Acq_struct::Type
 *  Member 'Type' defines if the monitoring is running  in periodic or one shot mode
 *  @var srvEM4325_Acq_struct::State
 *  Member 'State' defines if the monitoring is running or not
 *  @var srvEM4325_Acq_struct::Period
 *  Member 'Period' defines the value of the period between two acquisitions
 *  @var srvEM4325_Acq_struct::UnitPeriod
 *  Member 'UnitPeriod' defines the unit of 'Period' member
 *  @var srvEM4325_Acq_struct::BeginDateValue
 *  Member 'BeginDateValue' defines the value of the date when the acquisitions will start/has been launched
 *  @var srvEM4325_Acq_struct::EndDateValue
 *  Member 'EndDateValue' defines the unit of 'BeginDateValue' member
 *  @var srvEM4325_Acq_struct::BeginTimeValue
 *  Member 'BeginTimeValue' defines the value of the time when the acquisitions will start/has been launched
 *  @var srvEM4325_Acq_struct::EndTimeValue
 *  Member 'EndTimeValue' defines the unit of 'BeginTimeValue' member
 *  @var srvEM4325_Acq_struct::LowThreshold
 *  Member 'LowThreshold' defines the value of the LowThreshold
 *  @var srvEM4325_Acq_struct::HighThreshold
 *  Member 'HighThreshold' defines the value of the HighThreshold
 */
typedef struct {
	/* 	system file - ACQMODE 	*/
	uint16_t Type;
	uint16_t State;
	/* 	system file - ACQPERIOD	*/
	uint16_t PeriodValue;
	uint16_t PeriodUnit;
	/*	system file - ACQBEGIN 	*/
	uint32_t BeginDateValue;
	uint32_t BeginTimeValue;
	/* 	system file - ACQEND 	*/
	uint32_t EndDateValue;
	uint32_t EndTimeValue;
	/* 	system file - ACQThreshold 	*/
	int16_t LowThreshold;
	int16_t HighThreshold;
	/* 	the sensor id is defined by the srvEM4325_Sensor_enum */
}srvEM4325_Acq_struct;

/** @struct srvEM4325_Alarm_struct
 * @var srvEM4325_Alarm_struct::ui16BelowTemp to know if an alarm is set
 *  Member 'ui16BelowTemp' defines the bit for the upper temperature alarm
 * @var srvEM4325_Alarm_struct::ui16UpperTemp
 *  Member 'ui16UpperTemp' defines the bit for the below temperature alarm
 * @var srvEM4325_Alarm_struct::ui16LowBattery
 *  Member 'ui16LowBattery' defines the bit for the low battery alarm
 * @var srvEM4325_Alarm_struct::ui16Rfu
 *  Member 'ui16Rfu' defines bit field reserved for use
 */
typedef struct {
	uint16_t ui16BelowTemp 		: 1;	/* LSB */
	uint16_t ui16UpperTemp 		: 1;
	uint16_t ui16LowBattery		: 1;
	uint16_t ui16Rfu			: 13;
}srvEM4325_Alarm_struct;

/** @struct srvEM4325_Alarm_union
 * @var srvEM4325_Alarm_union::sBitField
 *  Member 'sBitField' defines the value of the bit field
 * @var srvEM4325_Alarm_union::ui16Value
 *  Member 'ui16Value' defines the value of the AreYouWarningMe system file
 */
typedef union {
	srvEM4325_Alarm_struct sBitField;
	uint16_t ui16Value;
}srvEM4325_Alarm_union;

/** @struct srvEM4325_DateTimeAlarm_struct
 * @var srvEM4325_DateTimeAlarm_struct::Date
 *  Member 'Date' defines the rtcc date when the alarm occurred
 * @var srvEM4325_DateTimeAlarm_struct::Time
 *  Member 'Time' defines the rtcc time when the alarm occurred
 */
typedef struct {
	uint32_t Date;
	uint32_t Time;
}srvEM4325_DateTimeAlarm_struct;



/** @struct srvEM4325_download_struct
 * @var srvEM4325_download_struct::IsOnGoing
 *  Member 'IsOnGoing' defines if an data transfer between the RF host and the is on going
 */
typedef struct {
	bool IsOnGoing;
	uint16_t * pSerialData;
}srvEM4325_download_struct;

/**
  * @struct this structure is the contain of the system file
  * @detail this structure is linked to the enum srvEM4325_RegisterAddress_enum
  * that defines the address of the register
 */
typedef struct {
uint16_t 	WhoAreYou	;					/*!< This variable is the configuration of the application */
uint16_t	HowAreYou	;					/*!< This variable is the battery state of the application */
uint16_t	HowOldAreYou;					/*!< This variable is the lifetime of the board */
uint16_t	HowWarmIsIt;					/*!< This variable is the measure the temperature*/
uint16_t	time ;							/*!< This variable is the current time word 1*/
uint16_t	time2;							/*!< This variable is the current time word 2 */
uint16_t	Date;							/*!< This variable is the current date word 1*/
uint16_t	Date2;							/*!< This variable is the current date word 2*/
srvEM4325_Acq_struct srvEM4325_sAcqSysFiles[KERNEL_SENSOR_ID_NBSENSOR];	/*!< This variable is to configure or get the acquisition configuration */
srvEM4325_Alarm_union EnableAlarm;			/*!< This variable is to know the enabled alarms */
srvEM4325_Alarm_union StateAlarm;			/*!< This variable is to know the alarm status */
srvEM4325_Alarm_union ResetAlarm;			/*!< This variable is to reset the alarm status */
srvEM4325_DateTimeAlarm_struct DateTimeAlarm[KERNEL_ALARM_ID_LAST]; /*!< This variable is to know the rtcc dates and times when alarms have been triggered */
uint16_t	Reset ;							/*!< This variable is to reset the board and load the default configuration*/
uint16_t	IsDataAvailable;				/*!< This variable is to know how many data is available*/
uint16_t	GiveMeSamples ; 				/*!< This variable is to send to the RF host some stored samples in the application (RAM or external mememory)*/
uint16_t	IdxSample ; 					/*!< This variable defines the address of the first sample to download in the GiveMeData system file */
uint16_t	NbSamplesToGive  ; 				/*!< This variable defines the number of samples to download in the GiveMeData system file */
uint16_t	SelectMemory ; 					/*!< This variable defines the memory which going to store the samples */
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
uint16_t	ui16HowMuchYouRun ; 					/*!< This variable stores the number of start of the machine */
uint16_t	ui16HowTimeYouRun ; 					/*!< This variable stores the number of time of the activity of the machine */
uint16_t	ui16Calibration [KERNEL_SENSOR_ID_NBSENSOR];/*!< This variable stores the request or the status of the calibration of the sensor */
uint16_t	ui16Log [KERNEL_SENSOR_ID_NBSENSOR];		/*!< This variable stores the request or the status of the continuous measurement of the sensor */
#endif
}srvEM4325_Systemfile_struct;

/*===========================================================================================================
						Public variables definitions
===========================================================================================================*/
/* Parameters of the RFID communication in progress */
extern srvEM4325_commParams_Type 		srvEM4325_sCommParams;
extern srvEM4325_configuration_struct 	srvEM4325_sConfiguration;
extern srvEM4325_Systemfile_struct*		srvEM4325_psSysFile;

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	srvEM4325_Init 							( void );
void 	srvEM4325_InitSwModule 					( void );
void	srvEM4325_InitSystemFiles			( const uint32_t ui32BeginDate, const uint32_t ui32BeginTime );
uint8_t srvEM4325_ConfigSpiSlaveExt				( srvEM4325_SpiSlaveExtensionMode_enum eFunctionality );

/* Spi commands */
void 	srvEM4325_RequestStatus 				( uint8_t * pui8EM4325Status );
uint8_t srvEM4325_ReadWord						( const uint16_t ui16Address, uint16_t * pui16ReadWord );
uint8_t srvEM4325_WriteWord						( const uint16_t ui16Address, uint16_t ui16WordToWrite );

uint8_t srvEM4325_Boot							( void );
uint8_t srvEM4325_SetComParams 					( const uint16_t ui16Params );

/* Comm buffer process */
uint8_t srvEM4325_ProcessCommBufferCommand 		( Kernel_QueueItem_struct * psQueueitem );
uint8_t	srvEM4325_TransferMeasureTemp 			( uint8_t const * pui8Data );
uint8_t	srvEM4325_TransferDateInSystemFile 		( const uint32_t pui32date );
uint8_t	srvEM4325_TransferBoardAgeInSystemFile 	( const uint16_t pui32BoardAge );
uint8_t srvEM4325_TransferTimeInSystemFile 		( const uint32_t pui32time );
uint8_t	srvEM4325_TransferIsDataAvailable		( const uint16_t  ui16Data );
void  	srvEM4325_SaveDataPointer 				( uint16_t const * pui16Data );
uint8_t	srvEM4325_TransferGiveMeSamples 		( uint16_t const * pui16Data);
void 	srvEM4325_ProcessAcqSysFile 			( void );
uint8_t	srvEM4325_DeliverRespSysFileWriteCmd	( bool bAcknowledge );
void 	srvEM4325_WriteSuccessInitFwStatus		( void );
void 	srvEM4325_UpdateSysFileFromOtherTask	( const srvEM4325_RegisterAddress_enum eSysFileAdrs, uint16_t const * pui16Data );
void 	srvEM4325_SetStateAlarmFromSensor 		( const Kernel_AlarmId_enum eAlarmId );
void 	srvEM4325_SetDateTimeAlarmFromSensor	( const Kernel_AlarmId_enum eAlarmId, const uint32_t ui32Date, const uint32_t ui32Time );
void 	srvEM4325_GetEnableAlarmSysFile 		( uint16_t * pui16EnableAlarm );
void 	srvEM4325_GetAcqSysFiles 				( uint16_t * pui16OutBuffer, uint8_t * pui8NbWordsRead );
void 	srvEM4325_WriteSuccessSuccesCode 		( const uint8_t ui8Register, const uint8_t ui8Nbword );
void 	srvEM4325_HookErrorCode 				( const CrossRfid_Status_enum ui8Error );
void 	srvEM4325_WriteSystemFileAccToQueueData ( const uint8_t ui8SysFileAdrs,  const  uint8_t ui8NbByte  , uint8_t const * pui8Data );
void 	srvEM4325_WriteUid						( const uint8_t ui8Nbbyte , const uint8_t * pui8Uid  );
uint8_t srvEM4325_MeasureTemperature			( int16_t  * pi16Temperature );

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
uint8_t srvEM4325_TransferGiveMeLife 			( const Kernel_QueueItem_struct sQueue );
#elif (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
void srvEM4325_UpdateSystemFiles 				(const srvEM4325_RegisterAddress_enum ui16RegisterAddress , const Kernel_Sensor_Id_enum ui16SensorId, const uint16_t ui16NewField);
#endif
#endif /* SRVEM4325_H_ */
