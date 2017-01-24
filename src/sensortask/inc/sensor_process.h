/*******************************************************************************
 * @file sensor_process.h
 * @brief this function set manages the data exchange between the task and the
 * device.
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SENSOR_PROCESS_H
#define SENSOR_PROCESS_H

#include "common_library.h"
#include "kernel_common.h"
#include "common_version.h"
#include "srvMeasTemperature.h"
#include "srvLIS2DH12.h"
#include "srv24LC64.h"
#include "srvCalendar.h"
#include "srvCRC.h"
#include "srvadxl363.h"
#include "srvActivityRecorder.h"
#include "srvlis3mdl.h"
#include "srvM2M.h"
#include "drv_sleep.h"
#include "srvhtu21d.h"
#include "logicaltimer.h"
#include "srvBubbleLevel.h"

#define SENSOR_DEVICEID							(0xA0)			/* device ID for the I2C device*/

/* index (word uin16_t format) of the RF task configuration translated to Serial configuration */
#define  SENSOR_IDX_SFILE_ACQ 					(0)
#define  SENSOR_IDX_SFILE_ALARM_ENABLE 			(42)

/* masks of the enable alarm configuration */
#define SENSOR_ALARM_MASK_BELOWTEMP				(uint16_t)(0x0001)
#define SENSOR_ALARM_MASK_UPPERTEMP				(uint16_t)(0x0002)
#define SENSOR_ALARM_MASK_LOWBAT				(uint16_t)(0x0004)


#define SENSOR_MEASUREMENT_ISREADY				(0x0F)
#define SENSOR_MEASUREMENT_RESET				(0x00)

/* shifts of the enable alarm configuration */
#define SENSOR_ALARM_SHIFT_BELOWTEMP			(0)
#define SENSOR_ALARM_SHIFT_UPPERTEMP			(1)
#define SENSOR_ALARM_SHIFT_LOWBAT				(2)

/* define the number of calendar event to plug on a capture compare channel */
#define SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG			(KERNEL_SENSOR_ID_NBSENSOR )
/* value to notify that the channel is free */
#define SENSOR_FREE_CALENDAR_EVENT_CHANNEL			(-1)
/* shifts of the enable alarm configuration */
//#define SENSOR_ALARM_NB_BYTES_BUFFER_EXTMEM		(28)
//#define SENSOR_ALARM_NB_WORDS_BUFFER_EXTMEM		(SENSOR_ALARM_NB_BYTES_BUFFER_EXTMEM/2)


/*===========================================================================================================
						enum definition
===========================================================================================================*/
/**
 *  @enum	sensor_serialchannel_enum
 *  @brief  this enum contain the available RTCC channel for the measurement to start or stop a measurement
 *  @details the channel CC1 is dedicated to the calendar
 */
typedef enum{
	SENSOR_RTCCCHANNEL_0 = 0,	/*!< the first channel available    */
	SENSOR_RTCCCHANNEL_1 ,		/*!< Do not use this channel. CC1 is dedicated to the calendar */
	SENSOR_RTCCCHANNEL_2 ,		/*!< the 2nd channel available is the third one  */
}sensor_serialchannel_enum;


/**
 *  @enum 	sensor_direction_enum
 *  @brief 	this enum contains the direction used by the serial interface task (slave or master)
 */
typedef enum{
	SENSOR_DIRECTION_SLAVE = 0,	/*!< the MCU is a slave  */
	SENSOR_DIRECTION_MASTER		/*!< the MCU is the master  */
}sensor_direction_enum;

/**
 *  @enum sensor_i2cdatarate_enum
 *  @brief this enum contains the available data rates of the I2C interface
 */
typedef enum{
	SENSOR_I2CFREQUENCY_1k = 0,		/*!< the clock frequency is 1k  */
	SENSOR_I2CFREQUENCY_10k ,		/*!< the clock frequency is 10k  */
	SENSOR_I2CFREQUENCY_100k ,		/*!< the clock frequency is 100k  */
	SENSOR_I2CFREQUENCY_400k 		/*!< the clock frequency is 400k  */
}sensor_i2cdatarate_enum;

/**
 *  @enum this enum contains the protocol used by the serial interface task
 *  @brief the task can work as a salve or as a master depending of the sensor_direction_enum
 *
 */
typedef enum{
	SENSOR_PROTOCOL_SPI = 0,	/*!< SPI protocol  */
	SENSOR_PROTOCOL_UART ,		/*!< UART protocol  */
	SENSOR_PROTOCOL_I2C			/*!< I2C protocol  */
}sensor_prococol_enum;


/*===========================================================================================================
						enum definition
===========================================================================================================*/

/**
 *  @enum sensor_AlarmStatusIndex_enum
 *  @brief this enum contains the address to write in the 24LC64 memory
 *  to update the bit field alarm
 *
 */
typedef enum{
	SENSOR_ALARMBITFIELD		= 0x0000,	/*!< the address of the alarm bit field status  */
	SENSOR_ALARMLOWTEMP 		= 0x0001,	/*!< the uint16_t address of the low temp alarm */
	SENSOR_ALARMHIGHTEMP 		= 0x0005,	/*!< the uint16_t address of the high temp alarm */
	SENSOR_ALARMLOWBAT 			= 0x0009,	/*!< the uint16_t address of the low bat alarm */
	SENSOR_ALARMCRC				= 0x000D,	/*!< the uint16_t address of the crc */
	SENSOR_SIZE_16BITBUFFER
}sensor_AlarmStatusIndex_enum;



/**
 *  @enum sensor_VersionIndex_enum
 *  @brief this enum contains the index of the different versions of the board
 *
 */
typedef enum{
	SENSOR_FIRMAWAREVERSION		= 0x0000,	/*!< the address of the alarm bit field status  */
	SENSOR_HARDWAREVERSION 		= 0x0002,	/*!< the uint16_t address of the low temp alarm */
	SENSOR_TESTVERSION 			= 0x0004,	/*!< the uint16_t address of the high temp alarm */
}sensor_VersionIndex_enum;


/**
 *  @enum sensor_CalendarEventType_enum
 *  @brief this enum contains the index of the different versions of the board
 *
 */
typedef enum{
	SENSOR_CALENDAREVENT_STOPMEAS		= 0x00	,	/*!< the type  of the calendar event is start measurement  */
	SENSOR_CALENDAREVENT_STARTMEAS				,	/*!< the type  of the calendar event is stop measurement  */
}sensor_CalendarEventType_enum;
/*===========================================================================================================
						structure definition
===========================================================================================================*/

/** @struct sensor_i2cconfiguration_struct
 *  @brief this structure contains the specific configuration for the I2C
 *  @var sensor_i2cconfiguration_struct::ui2cAddress
 *  Member 'ui2cAddress' defines the I2C address (familly and device on 8 bits)
 *  @var sensor_i2cdatarate_enum::sI2cdatarates
 *  Member 'sI2cdatarates' defines the I2C datarates
 */
typedef struct{
	uint8_t ui2cAddress  ;
	sensor_i2cdatarate_enum sI2cdatarates ;
}sensor_i2cconfiguration_struct;

/** @struct sensor_configuration_struct
 *  @brief This structure contains the configuration of the serial interface
 *  @var sensor_configuration_struct::sensor_SlaveOrMaster
 *  Member 'sensor_SlaveOrMaster' defines if the MCU acts as a master or a slave
 *  @var sensor_configuration_struct::sensor_Protocol
 *  Member 'sensor_Protocol' defines the used protocol for the serial interface task
 */
typedef struct{
	sensor_direction_enum sensor_SlaveOrMaster ;
	sensor_prococol_enum sensor_Protocol;
}sensor_configuration_struct;

#if 0
/** @struct sensor_SampleData_struct
 *  @brief This structure contains the configuration and data of the samples monitored
 *  @var sensor_SampleData_struct::ui16Count
 *  Member 'ui16Count' defines the number of samples collected
 *  @var sensor_SampleData_struct::bOverFlow
 *  Member 'bOverFlow' defines if a buffer overflow has occurred
 *  @var sensor_SampleData_struct::tui16Data
 *  Member 'tui16Data' defines data
 */
typedef struct{
	uint16_t 	ui16Count ;
	bool 		bOverFlow ;
	int16_t 	ti16Data[KERNEL_COMMBUFFER_LENGTH];
}sensor_SampleData_struct;
#endif


/** @struct sensor_SensorCongiga_struct
 *  @brief This structure contains the configuration for a measurement
 *  @var sensor_SensorCongiga_struct::IsStartDefined
 *  Member 'IsStartDefined' defines the number of samples collected
 *  @var sensor_SensorCongiga_struct::IsEndDefined
 *  Member 'IsEndDefined' defines if a buffer overflow has occurred
 *  @var sensor_SensorCongiga_struct::IsPeriodDefined
 *  Member 'IsPeriodDefined' defines data
 *  @var sensor_SensorCongiga_struct::IsModeDefined
 *  Member 'IsModeDefined' defines data
 */
typedef struct{
	uint8_t 	bIsStartDefined 	: 1;
	uint8_t 	bIsEndDefined 		: 1;
	uint8_t 	bIsPeriodDefined 	: 1;
	uint8_t 	bIsModeDefined 		: 1;
	uint8_t 	b4RFU 				: 4;
}sensor_SensorCongiga_struct;

/** @struct sensor_SensorCongiga_union
 *  @brief This structure contains the configuration and data of the samples monitored
 *  @var sensor_SensorCongiga_struct::IsConfigured
 *  Member 'IsConfigured' is the
 *  @var sensor_SensorCongiga_struct::bitfield
 *  Member 'bitfield' access bit to bit
 */
typedef union{
	uint8_t 	ui8field ;
	sensor_SensorCongiga_struct bitfield;
}sensor_SensorCongiga_union;
/**
 * @struct this structure is the contain of the sensor configuration
 *  @var sensor_SensorConfig_struct::Type
 *  Member 'Type' defines if the monitoring is running in periodic or one shot mode
 *  @var sensor_SensorConfig_struct::IsRunning
 *  Member 'IsRunning' defines if the monitoring is running or not
 *  @var sensor_SensorConfig_struct::IsConfigured
 *  Member 'IsConfigured' is updated when the RF host configured the measurement
 *  @var sensor_SensorConfig_struct::Period
 *  Member 'Period' defines the value of the period between two acquisitions
 *  @var sensor_SensorConfig_struct::UnitPeriod
 *  Member 'UnitPeriod' defines the unit of 'Period' member
 *  @var sensor_SensorConfig_struct::BeginDateValue
 *  Member 'BeginDateValue' defines the value of the date when the acquisitions will start/has been launched
 *  @var sensor_SensorConfig_struct::EndDateValue
 *  Member 'EndDateValue' defines the unit of 'BeginDateValue' member
 *  @var sensor_SensorConfig_struct::BeginTimeValue
 *  Member 'BeginTimeValue' defines the value of the time when the acquisitions will start/has been launched
 *  @var sensor_SensorConfig_struct::EndTimeValue
 *  Member 'EndTimeValue' defines the unit of 'BeginTimeValue' member
 *  @var sensor_SensorConfig_struct::LowThreshold
 *  Member 'LowThreshold' defines the value of the low alarm threshold
 *  @var sensor_SensorConfig_struct::HighThreshold
 *  Member 'HighThreshold' defines the unit of the high alarm threshold
 */
typedef struct {
	uint8_t ui8Type;
	bool 	IsRunning;
	sensor_SensorCongiga_union IsConfigured;
	uint16_t ui16PeriodValue;
	uint16_t ui16PeriodUnit;
	bool 	 bStartDateTimeIsPlugged;
	uint32_t ui32BeginDateValue;
	uint32_t ui32BeginTimeValue;
	bool 	 bEndDateTimeIsPlugged;
	uint32_t ui32EndDateValue;
	uint32_t ui32EndTimeValue;
	int16_t i16LowThreshold;
	int16_t i16HighThreshold;
	uint8_t ui8TimerId;
//	int8_t *pi8MeasBuffer;
	/* 	the sensor id is defined by the kernel */
}sensor_SensorConfig_struct;

/** @struct sensor_EnableAlarm_struct
 * @brief this structure contains software global variables to know if an alarm is enable
 * @var sensor_EnableAlarm_struct::ui16BelowTemp
 *  Member 'ui16BelowTemp' defines the bit for the upper temperature alarm
 * @var sensor_EnableAlarm_struct::ui16UpperTemp
 *  Member 'ui16UpperTemp' defines the bit for the below temperature alarm
 * @var sensor_EnableAlarm_struct::ui16LowBattery
 *  Member 'ui16LowBattery' defines the bit for the low battery alarm
 * @var sensor_EnableAlarm_struct::ui16Rfu
 *  Member 'ui16Rfu' defines bit field reserved for use
 */
typedef struct {
	uint16_t ui16BelowTemp 		: 1;	/* LSb */
	uint16_t ui16UpperTemp 		: 1;
	uint16_t ui16LowBattery		: 1;
	uint16_t ui16Rfu			: 13;
}sensor_EnableAlarm_struct;

/** @struct srvEM4325_StateAlarm_struct
 * @brief this structure contains software global variables to know if an alarm is enable
 * @var sensor_StateAlarm_union::sBitField
 *  Member 'sBitField' defines the value of the bit field
 * @var sensor_StateAlarm_union::ui16Value
 *  Member 'ui16Value' defines the value of the AreYouWarningMe system file
 */
typedef union {
	sensor_EnableAlarm_struct sBitField;
	uint16_t ui16Value;
}sensor_EnableAlarm_union;

/** @struct sensor_StateAlarm_struct
 * @brief 	this structure contains the alarm state (1 means tha alarms fired 0 not)
 * @var sensor_StateAlarm_struct::ui16BelowTemp to know if an alarm is set
 *  Member 'ui16BelowTemp' defines the bit for the upper temperature alarm
 * @var sensor_StateAlarm_struct::ui16UpperTemp
 *  Member 'ui16UpperTemp' defines the bit for the below temperature alarm
 * @var sensor_StateAlarm_struct::ui16LowBattery
 *  Member 'ui16LowBattery' defines the bit for the low battery alarm
 * @var sensor_StateAlarm_struct::ui16Rfu
 *  Member 'ui16Rfu' defines bit field reserved for use
 */
typedef struct {
	uint16_t ui16BelowTemp 		: 1;	/* LSb */
	uint16_t ui16UpperTemp 		: 1;
	uint16_t ui16LowBattery		: 1;
	uint16_t ui16Rfu			: 13;
}sensor_StateAlarm_struct;

/** @struct sensor_StateAlarm_union
 * @brief this union contains the alarm state (1 means tha alarms fired 0 not)
 * @var sensor_StateAlarm_union::sBitField
 *  Member 'sBitField' defines the value of the bit field
 * @var sensor_StateAlarm_union::ui16Value
 *  Member 'ui16Value' defines the value of the AreYouWarningMe system file
 */
typedef union {
	sensor_StateAlarm_struct sBitField;
	uint16_t ui16Value;
}sensor_StateAlarm_union;

/** @struct sensor_AckStateAlarm_struct
 * @brief 	this structure contains the state aboout the acknowledge of the RF task
 * @var sensor_StateAlarm_struct::ui16BelowTemp to know if an alarm is set
 *  Member 'ui16BelowTemp' true when the RF task has aknowledge the new state
 * @var sensor_StateAlarm_struct::ui16UpperTemp
 *  Member 'ui16UpperTemp' true when the RF task has aknowledge the new state
 * @var sensor_StateAlarm_struct::ui16LowBattery
 *  Member 'ui16LowBattery' true when the RF task has aknowledge the new state
 * @var sensor_StateAlarm_struct::ui16Rfu
 *  Member 'ui16Rfu' defines bit field reserved for use
 */
typedef struct {
	uint16_t ui16BelowTemp 		: 1;	/* LSb */
	uint16_t ui16UpperTemp 		: 1;
	uint16_t ui16LowBattery		: 1;
	uint16_t ui16Rfu			: 13;
}sensor_AckStateAlarm_struct;

/** @struct sensor_StateAlarm_union
 * @brief this union contains the alarm state (1 means tha alarms fired 0 not)
 * @var sensor_StateAlarm_union::sBitField
 *  Member 'sBitField' defines the value of the bit field
 * @var sensor_StateAlarm_union::ui16Value
 *  Member 'ui16Value' defines the value of the AreYouWarningMe system file
 */
typedef union {
	sensor_AckStateAlarm_struct sBitField;
	uint16_t ui16Value;
}sensor_AckStateAlarm_union;


/** @struct sensor_CalendarEvent_struct
 *  @brief This structure allows the serial task to manage the calendar events (CCxdate/CCxTime)
 *  and the actions to perform after receiving a calendar notification (eg: Start or Stop
 *  the monitoring of a sensor).
  *  @var sensor_CalendarEvent_struct::bIsEnable
 *  Member 'bIsEnable' is true when the event is used by the RTCC
 *  @var sensor_CalendarEvent_struct::i8CCxChNumber
 *  Member 'i8CCxChNumber' defines the capture compare channel number linked to the event.
 *  @var sensor_CalendarEvent_struct::ui8SensorId
 *  Member 'ui8SensorId' defines the sensor id affected to the calendar event Id.
 *  @var sensor_CalendarEvent_struct::bIsBegin
 *  Member 'ui8type' defines the type of the calendar event ( see sensor_CalendarEventType_enum)
 *  @var sensor_CalendarEvent_struct::ui32Date
 *  Member 'ui32Date' defines the date to set in the capture compare of the rtcc channel.
 *  @var sensor_CalendarEvent_struct::ui32Time
 *  Member 'ui32Time' defines the time to set in the capture compare of the rtcc channel.
 */
typedef struct{
	bool 		bIsEnable;
	int8_t 		i8CCxChNumber;
	uint8_t 	ui8SensorId;
	uint8_t		ui8type;
	uint32_t	ui32Date;
	uint32_t	ui32Time;
}sensor_CalendarEvent_struct;



/** @struct sensor_CalendarEvent_struct
 *  @brief This structure allows the serial task to manage the calendar events (CCxdate/CCxTime)
 *  and the actions to perform after receiving a calendar notification (eg: Start or Stop
 *  the monitoring of a sensor).
  *  @var sensor_CalendarEvent_struct::bIsEnable
 *  Member 'bIsEnable' is true when the event is used by the RTCC
 *  @var sensor_CalendarEvent_struct::i8CCxChNumber
 *  Member 'i8CCxChNumber' defines the capture compare channel number linked to the event.
 *  @var sensor_CalendarEvent_struct::ui8SensorId
 *  Member 'ui8SensorId' defines the sensor id affected to the calendar event Id.
 *  @var sensor_CalendarEvent_struct::bIsBegin
 *  Member 'ui8type' defines the type of the calendar event ( see sensor_CalendarEventType_enum)
 *  @var sensor_CalendarEvent_struct::ui32Date
 *  Member 'ui32Date' defines the date to set in the capture compare of the rtcc channel.
 *  @var sensor_CalendarEvent_struct::ui32Time
 *  Member 'ui32Time' defines the time to set in the capture compare of the rtcc channel.
 */
typedef struct{
	sensor_StateAlarm_union 		StateAlarm;
	sensor_EnableAlarm_union		EnableAlarm;
	sensor_AckStateAlarm_union		AckStateAlarm;
}sensor_Alarm_struct;

/*===========================================================================================================
						Public variables definition
===========================================================================================================*/
//extern sensor_datapool_struct sensor_sDdatapool;

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
void 	sensor_ProcessInit 						( uint16_t * pui16Data );
#if (USE_TEMPERATURE_UC == 1)
void 	sensor_StartAlarmManagement 			( void );
#endif
uint8_t sensor_ProcessGetTemperature 			( Kernel_QueueItem_struct  * psQueueItem  );
uint8_t sensor_ProcessIsDataAvailable 			( Kernel_QueueItem_struct *sQueueItem );
uint8_t	sensor_ProcessGiveMeSamples 			( Kernel_QueueItem_struct * psQueueItem );
uint8_t sensor_ProcessEnableDisableAlarm 		( Kernel_QueueItem_struct * psQueueItem );
uint8_t	sensor_ProcessResetAlarm 				( Kernel_QueueItem_struct * psQueueItem );
uint8_t sensor_ProcessAcqMode 					( Kernel_QueueItem_struct * psQueueItem );
uint8_t	sensor_ProcessAcqPeriod					( Kernel_QueueItem_struct * psQueueItem );
uint8_t	sensor_ProcessAcqBegin					( Kernel_QueueItem_struct * psQueueItem );
uint8_t	sensor_ProcessAcqEnd					( Kernel_QueueItem_struct * psQueueItem );
uint8_t	sensor_ProcessAcqThresholds				( Kernel_QueueItem_struct * psQueueItem);
void 	sensor_StoreNewSample					( void );
void 	sensor_StoreSetAlarmInExtMemory			( const Kernel_QueueItem_struct queueItems );
void 	sensor_processDataAcquisition 			( void );
uint8_t	sensor_ProcessAcceleromterIRQ 			( Kernel_QueueItem_struct *sQueueItem  );
uint8_t sensor_ProcessMagnetoIRQ 				( Kernel_QueueItem_struct *sQueueItem  );
uint8_t sensor_ProcessCalendarEvent 			( const uint8_t CalendarEventId ,  Kernel_QueueItem_struct *pQueueItems);
uint8_t sensor_ProcessMeasureTemperature 		( Kernel_QueueItem_struct*  psQueueItem );
uint8_t sensor_ProcessSerialRequest 			( Kernel_QueueItem_struct *sQueueItem );

void 	serial_processEndOfWakeupMode 			( void );
uint8_t serial_ProcessSensorResponse 			(Kernel_QueueItem_struct *pQueueItems );
void 	sensor_GetBufferTemp 					( int8_t **i8pdata );
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
uint8_t	sensor_StopTempAcquisitions 			(Kernel_QueueItem_struct  * psQueueItems);
#endif

#endif

