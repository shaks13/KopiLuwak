/*******************************************************************************
 * @file kernel_common.h
 * @brief creation the task
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef KERNEL_COMMON_H
#define KERNEL_COMMON_H

#include "common_library.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "common_statuscode.h"
#include "semphr.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/

#define KERNEL_DONTCLEARINCOMINGNTF					0x0000
#define KERNEL_RESETTHEOUTGOINGNTF					0x0000

/* constants of the Queue objects */
#define KERNEL_NBQUEUEITEM							4
/* @brief these mask are used to get or set the notification field exchaged between 2 tasks*/
#define KERNEL_SENDER_MASK 							0x0F
#define KERNEL_RECEIVER_MASK 						0xF0
#define KERNEL_MESSAGE_MASK 						0x00FFFF00
#define KERNEL_QUEUEID_MASK							0x000000F0
#define KERNEL_RFU_MASK 							0x0000000F
#define KERNEL_RECEIVER_SHIFT						4
#define KERNEL_SENDER_SHIFT							0
#define KERNEL_MESSAGE_SHIFT						8
#define KERNEL_QUEUEID_SHIFT						4
#define KERNEL_RFU_SHIFT							0

/* Common constants between RF task and Serial Task */
#define KERNEL_COMMBUFFER_LENGTH					(256)

/* index to synchronize rf task and serial configurations concerning the sensor */
/* 		sensor id index */
#define KERNEL_MSG_IDX_ACQ_IDX_SENSORID				(0)
/* 		index if system files on 32bit */
#define KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD			(1)
#define KERNEL_MSG_IDX_ACQ_MODE						(2)
#define KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD		(2)
/* 		index if system files on 32bit */
#define KERNEL_MSG_IDX_ACQ_DATE_LSW   				(1)
#define KERNEL_MSG_IDX_ACQ_DATE_MSW   				(2)
#define KERNEL_MSG_IDX_ACQ_TIME_LSW					(3)
#define KERNEL_MSG_IDX_ACQ_TIME_MSW					(4)

#define KERNEL_NB_WORDS_IN_SENSOR_CONFIG			(14)

#define KERNEL_CREATE_RECANDSEND(receipt,sender)	(((receipt << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | \
													((sender << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK))
#define KERNEL_WHICH_SENDERID(urecvsender) 			((urecvsender & KERNEL_SENDER_MASK)>>KERNEL_SENDER_SHIFT)
#define KERNEL_WHICH_RECEIVERID(urecvsender) 		((urecvsender & KERNEL_RECEIVER_MASK)>>KERNEL_RECEIVER_SHIFT)

/* The time in ticks to wait for the binary semaphore preserve the em4325 access to become available. */
#define KERNEL_BLOCK_TIME_SEMPHR_EM4325_COMBUFFER	(25)  /* ms */
#define KERNEL_BLOCK_TIME_SEMPHR_EM4325_GETTEMP		(10) /* ms *//* TODO: find the critical value */

/* To perform an infinite acquisition or disable calendar event */
#define KERNEL_ACQ_BEGIN_END_INFINITE_VALUE			(0)

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#if (GLUEPOT_DEBUG == 1)
/* change these defines in second tick for example to compute the life each second */
#define KERNEL_RTCC_IFC_LIFE_CALCULATION_TICK 		(RTCC_IFC_CNTTICK)
#define KERNEL_RTCC_IF_LIFE_CALCULATION_TICK  		(RTCC_IF_CNTTICK)
#define KERNEL_RTCC_IEN_LIFE_CALCULATION_TICK 		(RTCC_IEN_CNTTICK)
#define KERNEL_RTCC_IEN_MASK_LIFE_CALCULATION_TICK  (_RTCC_IEN_CNTTICK_MASK)
#define KERNEL_RTCC_IEN_SHIFT_LIFE_CALCULATION_TICK (_RTCC_IEN_CNTTICK_SHIFT)
#else
/* minute tick */
/*#define KERNEL_RTCC_IFC_LIFE_CALCULATION_TICK 		(RTCC_IFC_MINTICK)
#define KERNEL_RTCC_IF_LIFE_CALCULATION_TICK  		(RTCC_IF_MINTICK)
#define KERNEL_RTCC_IEN_LIFE_CALCULATION_TICK 		(RTCC_IEN_MINTICK)
#define KERNEL_RTCC_IEN_MASK_LIFE_CALCULATION_TICK 	(_RTCC_IEN_MINTICK_MASK)
#define KERNEL_RTCC_IEN_SHIFT_LIFE_CALCULATION_TICK (_RTCC_IEN_MINTICK_SHIFT)*/
/* hour tick */
#define KERNEL_RTCC_IFC_LIFE_CALCULATION_TICK 		(RTCC_IFC_HOURTICK)
#define KERNEL_RTCC_IF_LIFE_CALCULATION_TICK  		(RTCC_IF_HOURTICK)
#define KERNEL_RTCC_IEN_LIFE_CALCULATION_TICK 		(RTCC_IEN_HOURTICK)
#define KERNEL_RTCC_IEN_MASK_LIFE_CALCULATION_TICK 	(_RTCC_IEN_HOURTICK_MASK)
#define KERNEL_RTCC_IEN_SHIFT_LIFE_CALCULATION_TICK (_RTCC_IEN_HOURTICK_SHIFT)
#endif

#endif
/*===========================================================================================================
						Enum definition
===========================================================================================================*/

/**
 *  @enum Kernel_TaskId_enum
 *  @brief this enum contains the Id of the tasks. these messages are received or sent to the tasks.
 *  these values are used by the fields usender and  horeceiver by the structure
 *  Kernel_TaskNotification_struct
 * */
typedef enum
{
	KERNEL_KERNELTASKID = 0x0,		/*!< Id of the Kernel task */
	KERNEL_SENSORTASKID,			/*!< Id of the sensor task */
	KERNEL_SERIALTASKID,			/*!< Id of the serial task */
	KERNEL_RFFRONTENDTASKID,		/*!< Id of the RF front end task */
	KERNEL_UNKNOWNTASK 				/*!< Unknown */
}Kernel_TaskId_enum;

/**
 *  @enum Kernel_QueueId_enum
 *  @brief this enum contains the Id of the queue. these messages are received or sent to the tasks.
 *  These values are used by the fields usender and  horeceiver by the structure Kernel_TaskNotification_struct
 *
 */
typedef enum
{
	KERNEL_KERNELID,					/*!< Id of the kernel queue */
	KERNEL_SERIALINTERFACEQUEUEID,		/*!< Id of the serialinterface queue */
	KERNEL_RFFRONTENDQUEUEID,			/*!< Id of the Rf front end queue */
	KERNEL_UNKNOWNQUEUEID				/*!< Unknown */
}Kernel_QueueId_enum;

/**
 *  @enum Kernel_FreeRTOSMessageId_enum
 *  @brief this enum contains the messages dedicated to the NLA front end. These messages
 *  are received or sent to the tasks. these values are used by the field wnotification
 *  by the structure Kernel_TaskNotification_struct
 *
 */
typedef enum{
	/* ==== message can be used by any tasks ==== */
	KERNEL_MESSAGEID_ACK = 0x00,			/*!< this message is sent to notify that the acknowledgement */
	KERNEL_MESSAGEID_NACK,					/*!< this message is sent to notify that the non-acknowledgement */
	KERNEL_MESSAGEID_QUEUEFULL,				/*!< this message is sent to notify the queue is full*/
	KERNEL_MESSAGEID_RELEASEBUFFER,			/*!< this message is sent by notify that the buffer can be released*/
	KERNEL_MESSAGEID_TASKINIT_YOURSELF,		/*!< this message is sent to notify that the task has to initialize itself */
	KERNEL_MESSAGEID_RESET,					/*!< this message is sent to notify that a reset has been requested */
#if (1 == USE_TEMPERATURE_EM4325)
	KERNEL_MESSAGEID_TIMOUT_GETTEMP,
#endif
	/* ==== message specific to the Kernel task ==== */
	KERNEL_MESSAGEID_STOPFIRMWARE,			/*!< this message is sent to notify that the firmware must stop */
	KERNEL_MESSAGEID_CALENDAR_EVENT,		/*!< this message is sent to notify that there is a calendar event */
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	//KERNEL_MESSAGEID_UPDATEFLASH_TEMPERATURE,/*!< this message is sent to notify that the buffer in flash must be update */
#endif
	/* ==== message specific to the RF task ==== */
	KERNEL_MESSAGEID_CHANGESTATESENSOR,		/*!< this message is sent to notify that the system file must be update concerning the state of acquisition */
	//KERNEL_MESSAGEID_WRITEINITOK,			/*!< this message is sent to notify that the RF task must write the firmware initialization status in em4325 register files */
	KERNEL_MESSAGEID_SETLOWTEMPALARM,		/*!< this message is sent to notify that the RF task must update the below temperature bit in the AreYouWarningMe system file */
	KERNEL_MESSAGEID_SETHIGHTEMPALARM,		/*!< this message is sent to notify that the RF task must update the upper temperature bit in the AreYouWarningMe system file */
	KERNEL_MESSAGEID_SETLOWBATALARM,		/*!< this message is sent to notify that the RF task must update the below battery bit in the AreYouWarningMe system file */
	KERNEL_MESSAGEID_UPDATEALARMSTATE,		/*!< this message is sent to notify that the RF task must update the AlarmState system file */
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	KERNEL_MESSAGEID_WRITE_TEMP_AVERAGE,	/*!< this message is sent to notify that the RF task must write the temperature average in the em4325 user memory */
#endif
	/* ==== message specific to the serial interface task ==== */
	KERNEL_MESSAGEID_STARTSERIALINT,		/*!< this message is sent to activate the serial interface interface */
	KERNEL_MESSAGEID_STOPSERIALINT,			/*!< this message is sent to disable the serial interface interface  */
	KERNEL_MESSAGEID_DATATOSEND,			/*!< this message is sent to notify that data should be send by the serial interface task */
	KERNEL_MESSAGEID_DATARECEIVED,			/*!< this message is sent to notify that data has been received by the serial interface task */
	KERNEL_MESSAGEID_WHICHCONFIG,			/*!< this message is sent to notify that the kernel asks the used configuration of the serial interface task */
	KERNEL_MESSAGEID_SPIRXIRQ,				/*!< this message is sent to notify that the RX interruption has been triggered */
	KERNEL_MESSAGEID_SPITXIRQ,				/*!< this message is sent to notify that the TX interruption has been triggered */
	KERNEL_MESSAGEID_DATAREADY,				/*!< this message is sent to notify that accelerometer's data must be read */
	KERNEL_MESSAGEID_ACCELEROMETERIRQ,		/*!< this message is sent to notify that accelerometer's data must be read */
	KERNEL_MESSAGEID_COMMBUFFERSEMAPHORE,	/*!< this message is sent to notify that accelerometer's data must be read */
	KERNEL_MESSAGEID_RFFIELDDETECTED,		/*!< this message is sent to notify that accelerometer's data must be read */
	KERNEL_MESSAGEID_TIMEOUT,				/*!< this message is sent to notify that accelerometer's data must be read */
	KERNEL_MESSAGEID_SAMPLETOGET,
	KERNEL_MESSAGEID_SETALARMTOWRITE,		/*!< this message is sent to notify that the serial task must write the new alarm bit field in the external memory */
	KERNEL_MESSAGEID_ENABLEALARM,           /*!< this message is sent to notify that the serial task must enable/disable alarms */
	KERNEL_MESSAGEID_RESETALARM,           	/*!< this message is sent to notify that the serial task must reset alarms */
	KERNEL_MESSAGEID_STARTALARMMANAGMENT,	/*!< this message is sent after the firmware initialization to notify that the serial task must start to manage the alarms */
	/* ==== message specific to the EM4325  ==== */
	KERNEL_MESSAGEID_STOPRFTASK,			/*!< this message is sent to activate the RF front end interface */
	KERNEL_MESSAGEID_GETTEMP,				/*!< this message is sent to get the temperature */
	KERNEL_MESSAGEID_GETAGE,				/*!< this message is sent to get the age of the board */
	KERNEL_MESSAGEID_GETTIME,				/*!< this message is sent to get the time */
	KERNEL_MESSAGEID_SETTIME,				/*!< this message is sent to get the time */
	KERNEL_MESSAGEID_GETDATE,				/*!< this message is sent to get the date */
	KERNEL_MESSAGEID_SETDATE,				/*!< this message is sent to get the date */
	KERNEL_MESSAGEID_ACQMODE,				/*!< set or get the acquisition mode for one sensor */
	KERNEL_MESSAGEID_ACQPERIOD,				/*!< this message is sent to configure or get the acquisition period for one sensor */
	KERNEL_MESSAGEID_ACQBEGIN,				/*!< this message is sent to configure or get the beginning of acquisitions for one sensor */
	KERNEL_MESSAGEID_ACQEND,				/*!< this message is sent to configure or get the end of acquisitions for one sensor */
	KERNEL_MESSAGEID_ACQTHRESHOLD,			/*!< this message is sent to configure or get the alarm thresholds for one sensor*/
	KERNEL_MESSAGEID_ISDATAAVAILABLE,		/*!< this message is sent to check if new data must be read */
	KERNEL_MESSAGEID_GIVESAMPLES,			/*!< this message is sent to notify that sensor's data must be read */
	KERNEL_MESSAGEID_STARTRFMODEM,			/*!< this message is sent to activate the serial interface interface */
	/* ==== message specific to the power Management  ==== */
	KERNEL_GOTOSLEEPMODE,
	KERNEL_STAYWAKEUP,
	/* ==== message specific to the M2M communication  ==== */
	KERNEL_MESSAGEID_RX,					/*!< this message is sent to inform that a RX frame has been received */
	KERNEL_MESSAGEID_READSYSTEMFILE,		/*!< this message is sent to request a read to the system file*/
	KERNEL_MESSAGEID_WRITESYSTEMFILE,		/*!< this message is sent to request a read to the system file*/
	/* ==== message specific to the M2M communication  ==== */
	KERNEL_MESSAGEID_MAGNETOIRQ,			/*!< this message is sent to inform that an IRq from the magnetometer */
	KERNEL_MESSAGEID_ACCELEROIRQ,			/*!< this message is sent to inform that an IRq from the accelerometer */
	KERNEL_MESSAGEID_DATAREADYIRQ,			/*!< this message is sent to inform that an IRq from the magnetometer */
	/* ==== message specific to the the application  ==== */
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)	/* this application counts the number of time and hours*/
	KERNEL_MESSAGEID_HOWMANYTIMEYOURUN,		/*!< this message is sent to activate the RF front end interface */
	KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN,		/*!< this message is sent to get the temperature */
	KERNEL_MESSAGEID_CALIBRATION,			/*!< this message is sent to set or get the calibration process */
	KERNEL_MESSAGEID_SERIALREQUEST,				/*!< this message is sent to set or get the measurement mode */
	KERNEL_MESSAGEID_LOG,					/*!< this message is sent to set or get the log */
	KERNEL_MESSAGEID_COMPUTEFFT,			/*!< this message is sent to set or get the computation of the FFT */
#elif (APP_CHOSEN_FLAG == APP_GLUEPOT)
	KERNEL_MESSAGEID_GPGIVEMELIFE,			/*!< this message is sent to get the shelf life */
	KERNEL_MESSAGEID_STOP_ACQUISITIONS,		/*!< this message is sent to stop the temperature acquisitions */
#endif
	KERNEL_MESSAGEID_RFEVENT,				/*!< this message is sent to activate the serial interface interface */
	KERNEL_MESSAGEID_LAST

}Kernel_FreeRTOSMessageId_enum;

#if 0
/**
 *  @enum Kernel_PoolObjectType_enum
 *  @brief this enum contains the Id of the different pool object.
 *  These Ids are used by the structure Kernel_PoolObject_structs
 *
 */
typedef enum{
	KERNEL_OBJECTTYPEID_TEMP = 0x00,			/*!< This Id to define the temperature object type */
	KERNEL_OBJECTTYPEID_HUMIDITY,				/*!< This Id to define the humidity object type */
	KERNEL_OBJECTTYPEID_UNKNOWN 				/*!< This Id to define the unknown object type */
}Kernel_PoolObjectType_enum;

/**
 *  @enum Kernel_Interface_enum
 *  @brief this enum contains the Id of the different interface.
 *  these Ids are used by the structure Kernel_TaskParams_t
 *
 */
typedef enum{
	KERNEL_INTERFACE_I2C = 0x00,				/*!< This Id to define the I2C interface */
	KERNEL_INTERFACE_SPI,						/*!< This Id to define the SPI (USART) interface */
	KERNEL_INTERFACE_UART 						/*!< This Id to define the UART interface */
}Kernel_Interface_enum;
#endif
/**
 *  @enum Kernel_Sensor_AcqType_enum
 *  @brief this enum contains the acquisition mode id of sensor
 */
typedef enum {
	KERNEL_SENSOR_ACQTYPE_ONESHOT = 0x00,		/*!< To select the periodic mode */
	KERNEL_SENSOR_ACQTYPE_PERIODIC,				/*!< To select the one shot mode */
	KERNEL_SENSOR_ACQTYPE_LAST
}Kernel_Sensor_AcqType_enum;

/**
 *  @enum	serial_Acqmode_enum
 *  @brief  this enum contain the different mode of the word 2 of acqmode
 */
typedef enum{
	KERNEL_SENSOR_ACQMODE_STOP = 0,				/*!< the sensor should stop */
	KERNEL_SENSOR_ACQMODE_START ,				/*!< the sensor should start */
	KERNEL_SENSOR_ACQMODE_WAITCALENDAREVANT ,	/*!< the sensor 'll start on the calendar event */
}serial_Acqmode_enum;

/**
 *  @enum Kernel_Sensor_AcqState_enum
 *  @brief this enum contains the state of acquisition
 */
typedef enum {
	KERNEL_SENSOR_ACQSTATE_FREE = 0x00,		/*!< the sensor is free to use / stop the acquisition command*/
	KERNEL_SENSOR_ACQSTATE_BUSY,			/*!< the sensor performs acquisitions / start the acquisition command */
	KERNEL_SENSOR_ACQSTATE_WAITCALENDAREVENT,			/*!< the sensor performs acquisitions / start the acquisition command */
	KERNEL_SENSOR_ACQSTATE_LAST
}Kernel_Sensor_AcqState_enum;

/**
 *  @enum Kernel_Sensor_AcqUnitPeriod_enum
 *  @brief this enum contains the unit of acquisition period
 */
typedef enum {
	KERNEL_SENSOR_ACQUNIT_US = 0x00,	/*!< select microsecond unit */
	KERNEL_SENSOR_ACQUNIT_MS,			/*!< select millisecond unit */
	KERNEL_SENSOR_ACQUNIT_S,			/*!< select second unit */
	KERNEL_SENSOR_ACQUNIT_MIN,			/*!< select minute unit */
	KERNEL_SENSOR_ACQUNIT_HOUR,			/*!< select hour unit */
	KERNEL_SENSOR_ACQUNIT_DAY,			/*!< select day unit */
	KERNEL_SENSOR_ACQUNIT_LAST
}Kernel_Sensor_AcqUnitPeriod_enum;

/**
 *  @enum Kernel_Sensor_Id_enum
 *  @brief this enum contains the id sensor
 */
typedef enum {
	KERNEL_SENSOR_ID_TEMPERATURE = 0x00,	/*!< To select the temperature sensor */
	KERNEL_SENSOR_ID_ACCELERO,				/*!< To select the accelerometer sensor */
	KERNEL_SENSOR_ID_MAGNETO,				/*!< To select the magnetometer sensor */
	KERNEL_SENSOR_ID_NBSENSOR
}Kernel_Sensor_Id_enum;

/**
 *  @enum this enum contains the Id of the alarm
 */
typedef enum
{
	KERNEL_ALARM_ID_LOW_TEMP,			/*!< Id of the low temperature alarm */
	KERNEL_ALARM_ID_HIGH_TEMP,			/*!< Id of the high temperature alarm */
	KERNEL_ALARM_ID_LOW_BATTERY,		/*!< Id of the low battery alarm */
	KERNEL_ALARM_ID_LAST
}Kernel_AlarmId_enum;

/**
 *  @enum this enum contains the Id of the EnableAlarm configuration
 */
typedef enum
{
	KERNEL_ALARM_ID_DISABLE = 0,	/*!< Id of the enable alarm configuration */
	KERNEL_ALARM_ID_ENABLE			/*!< Id of the disable alarm configuration */
}Kernel_EnableAlarmId_enum;


/**
 *  @enum Kernel_MeasurementId_enum
 *  @brief this enum contains the id of the measurement
 */
typedef enum {
	KERNEL_MEASID_RAW			 = 0x00,	/*!< To select the raw Measurement */
	KERNEL_MEASID_FFT,						/*!< To select the FFT Measurement */
	KERNEL_MEASID_XRAWAXIS,					/*!< To select the raw measurement on the X axis */
	KERNEL_MEASID_YRAWAXIS,					/*!< To select the raw measurement on the Y axis */
	KERNEL_MEASID_ZRAWAXIS,					/*!< To select the raw measurement on the Z axis */
	KERNEL_MEASID_XFFTAXIS,					/*!< To select the raw measurement on the X axis */
	KERNEL_MEASID_YFFTAXIS,					/*!< To select the raw measurement on the Y axis */
	KERNEL_MEASID_ZFFTAXIS,					/*!< To select the raw measurement on the Z axis */
	KERNEL_MEASID_XYZRAWAXIS	= 0x80,		/*!< To select the raw measurement on the 3 axis X,Y and Z */
}Kernel_MeasurementId_enum;




/**
 *  @enum kernel_commandId_enum
 *  @brief this enum contains the available command code
 *
 */
typedef enum{
	KERNEL_COMMANDCODE_UNKNOWN	= 0x00,		/*!< the M2M host request a get access  */
	KERNEL_COMMANDCODE_GET		= 0x01,		/*!< the M2M host request a get access  */
	KERNEL_COMMANDCODE_SET 		= 0x02,		/*!< the M2M host request a set access */
}kernel_commandId_enum;


/**
 *  @enum kernel_ObjectId_enum
 *  @brief this enum contains the available command code
 *
 */
typedef enum{
	KERNEL_OBJECTCODE_UNKNOWN			= 0x00	,		/*!< unknown object code */
	KERNEL_OBJECTCODE_ACCELEROMETER				,		/*!< the M2M host request an action on the accelerometer */
	KERNEL_OBJECTCODE_MAGNETOMETER				,		/*!< the M2M host request an action on the magnetometer */
	KERNEL_OBJECTCODE_ACTIVITYCOUNTER 			,		/*!< the M2M host request an action on the activity counter */
}kernel_ObjectId_enum;

/**
 *  @enum kernel_ActionId_enum
 *  @brief this enum contains the available action code
 *
 */
typedef enum{
	KERNEL_ACTIONCODE_UNKNOWN			= 0x00	,	/*!< unknown action code   */
	KERNEL_ACTIONCODE_ON						,	/*!< the M2M host request a activate action  */
	KERNEL_ACTIONCODE_OFF 						,	/*!< the M2M host request a deactivate action  */
	KERNEL_ACTIONCODE_STATE 					,	/*!< the M2M host request a state action  */
}kernel_ActionId_enum;




/** @struct kernel_DataExchange_Type
 *  @var kernel_DataExchange_Type::ui8CommandId
 *  Member 'ui8CommandId' is the command ID of the received command
 *  @var kernel_DataExchange_Type::ui8ObjectId
 *  Member 'ui8ObjectId' is the object of the operation
 *  @var kernel_DataExchange_Type::ui8ActionId
 *  Member 'ui8ActionId' is the action requested by the user (by e.g. "on" or "off")
 *  @var kernel_DataExchange_Type::ui16RegisterAddress
 *  Member 'ui16RegisterAddress' is the logical address of the register
 *  @var kernel_DataExchange_Type::ui16NbWordsInParams
 *  Member 'ui16NbWordsInParams' is the number of words in the parameters of the command
 */
typedef struct {
	kernel_commandId_enum ui8CommandId ;
	kernel_ObjectId_enum ui8ObjectId ;
	kernel_ActionId_enum ui8ActionId ;
	uint8_t *ui8pdata;
}kernel_DataExchange_Type;

/*===========================================================================================================
						structure definition
===========================================================================================================*/

/* Structure with parameters for serial interface */
typedef struct
{
	bool bIsMaster ;
	//Kernel_Interface_enum uInterface ;
} Kernel_TaskParams_t;

#if 0
/** @struct Kernel_PoolObject_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var Kernel_PoolObject_struct::ObjectId
 *  Member 'ObjectId' is the Id of the pool object.
 *  @var Kernel_PoolObject_struct::ObjectType
 *  Member 'ObjectType' is the type of the pool object. it allow to identifure the type of the pool data and its structure
 *  @var Kernel_PoolObject_struct::hwObjectSize
 *  Member 'hwObjectSize' is the number of byte of Data pointer by pObjectData
 *  @var Kernel_PoolObject_struct::pObjectData
 *  Member 'pObjectData' is a pointer on the data
 */
typedef struct
{
	uint8_t uObjectId;
	Kernel_PoolObjectType_enum eObjectType;
	uint16_t hwObjectSize;
	void *pObjectData;
} Kernel_PoolObject_struct;
#endif


/** @struct Kernel_TaskPointer_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var Kernel_TaskPointer_struct::pKernelTask
 *  Member 'pKernelTask' is he handle of the kernel task
 *  @var Kernel_TaskPointer_struct::pSensorTask
 *  Member 'pSensorTask' is he handle of the sensor task
 *  @var Kernel_TaskPointer_struct::pLnaTask
 *  Member 'pLnaTask' is he handle of the fidji task
 */
typedef struct
{
	TaskHandle_t pKernelTask;
	TaskHandle_t pSerialTask;
	TaskHandle_t pSensorTask;
	TaskHandle_t pRFtask;
}Kernel_TaskPointer_struct;




/** @struct Kernel_QueuePointer_struct
 *  @brief This structure contains the pointer to the queues handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var Kernel_TaskPointer_struct::pKernelTask
 *  Member 'pKernelTask' is he handle of the kernel task
 *  @var Kernel_TaskPointer_struct::pFidjiTask
 *  Member 'pFidjiTask' is he handle of the fidji task
 *  @var Kernel_TaskPointer_struct::pLnaTask
 *  Member 'pLnaTask' is he handle of the fidji task
 */
typedef struct
{
	QueueHandle_t pKernelQueue;
	QueueHandle_t pSerialQueue;
	QueueHandle_t pSensorQueue;
	QueueHandle_t pRFtaskQueue;
}Kernel_QueuePointer_struct;



/** @struct Kernel_QueueItem_struct
 *  @brief This structure contains the different field of the notification
 *  @var Kernel_QueueItem_struct::urecvsender
 *  Member 'urecvsender' contains the taksId of the sender and the receiver
 *  @var Kernel_QueueItem_struct::hwnotification
 *  Member 'hwnotification' contains the notification Id (see Kernel_LNAfrontendMessageId_enum)
 *  @var Kernel_QueueItem_struct::hwNbByte
 *  Member 'hwNbByte' is the number of bytes of data
 *  @var Kernel_QueueItem_struct::pData
 *  Member 'pData' is the the pointer of data
 */
typedef struct
{
	uint8_t urecvsender ;								/*!< Id of the sender & receiver tasks */
	Kernel_FreeRTOSMessageId_enum ui16notification ;   	/*!< Id of the notification */
	uint16_t ui16NbByte;									/*!< number of byte of data */
	uint8_t * pData;									/*!< pointer on the data */
}Kernel_QueueItem_struct;


#define KERNEL_MESSAGE_DEFAULT                                                              \
{																							\
	(((KERNEL_SENSORTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 				\
	((KERNEL_RFFRONTENDTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)),     			\
	KERNEL_MESSAGEID_LAST, 																	\
	0, 																						\
	NULL																					\
}

#if 0
typedef union
{
	uint32_t wvalue;
	Kernel_TaskNotification_struct sTaskNtf;
}Kernel_TaskNotification_union;
#endif

/*===========================================================================================================
						Binary semaphore
===========================================================================================================*/
extern SemaphoreHandle_t  xSemaphore ;
extern SemaphoreHandle_t  xTimer0Semaphore ;

/*===========================================================================================================
						Mutexes
===========================================================================================================*/
#if (1 == USE_TEMPERATURE_EM4325)
/* binary semaphore protecting the EM4325 resource
 * The sequences Take/Give in the Free RTOS are:
 *      -------------------------					-----------------------------
 *  1.1	| 	Sequence [1] 		| 1.2			2.1	| 	Sequence [2] 			| 2.2
 * 	Take| Comm buffer command 	| Give			Take| Periodic Get temp (serial)| Give
 *      -------------------------					-----------------------------
 *
 *      -------------------------------------------------
 * 2.1"	| 	Sequence [2"] 								| 2.2"
 * 	Take| Periodic Get temp + average temp to write (RF)| Give
 *      -------------------------------------------------
 *
 *      -------------------------			-----------------------------			----------------------------
 *  3.1	| 	Sequence [3]		| 3.2	3.3	|	Sequence [3]			| 3.4	3.5	| 	Sequence [3]			| 3.6
 * 	Take| HowWarmIsIt command 	| Give	Take| Get One Shot temp (serial)| Give	Take| HowWarmIsIt end (Rf task)	| Give
 *      -------------------------			-----------------------------			-----------------------------
 *
 * The sequences are specified in the code (e.g. Semaphore "xBinarySemphrEm4325": sequence 3.1) */
extern SemaphoreHandle_t  xBinarySemphrEm4325 ;
#endif

/*===========================================================================================================
						prototypes
===========================================================================================================*/

/* @brief contains the task handles*/
Kernel_TaskPointer_struct sKernel_TaskPointer;
Kernel_QueuePointer_struct sKernel_QueuePointer;
#if 0
uint8_t kernel_GetpTask ( const Kernel_TaskId_enum TaskId, TaskHandle_t *pTaskToNotify  );
#endif
#endif

