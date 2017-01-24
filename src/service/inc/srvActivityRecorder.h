/*******************************************************************************
 * @file srvADXL363.c
 * @brief this file defines the command set for the ADXL363 (accelerometer)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRV_ACTIVITYRECORDER_H_
#define SRV_ACTIVITYRECORDER_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "srvadxl363.h"
#include "kernel_process.h"
#include "srvlis3mdl.h"
#include "drv_sleep.h"
#include "srvfft.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define SRVACTIVITYREC_MAGNETICFIELDTHRESHOLD 				0x0100
#define SRVACTIVITYREC_NBMEASUREFORTHECALIBRATION 			100
#define SRVACTIVITYREC_FACTORFORTHECALIBRATION 				1.2
#define SRVACTIVITYREC_ACCELEROMETERBUFFERNBELMEMENT		1024
#define SRVACTIVITYREC_MAGNETOBUFFERNBELMEMENT				128

#define SRVACTIVITYREC_DEFAULTSTATUS           			   {0,0,0,0,{0,0,0},{0,0,0},0}
/*===========================================================================================================
						Enum definition
===========================================================================================================*/

/**
 *  @enum srvActivityRecorder_ReadWriteAccess_enum
 *  @brief this enum defines the Read  or Write Acesss of the statiuc variables
 */
typedef enum {
	SRVACTREC_SYSTEMFILE_READACCESSBITFIELD = 0b0,			/*!< the read access of the word of the system file */
	SRVACTREC_SYSTEMFILE_WRITEACCESSBITFIELD ,				/*!< the write access of the word of the system file  */
}srvActivityRecorder_ReadWriteAccess_enum;

#if 0
/**
 *  @enum srvActivityRecorder_SensorID_enum
 *  @brief this enum defines the sensor Id
 */
typedef enum {
	SRVACTREC_SENSORID_THERMOMETER = 0x00,					/*!< the sensor id for the thermometer */
	SRVACTREC_SENSORID_ACCELEROMETER ,						/*!< the sensor id for the accelerometer */
	SRVACTREC_SENSORID_MAGNETOMETER ,						/*!< the sensor id for the magnetometer */
	SRVACTREC_SENSORID_NBSENSOR ,							/*!< the number of sensor available */
}srvActivityRecorder_SensorID_enum;
#endif


/**
 *  @enum srvActivityRecorder_CalibrationStatus_enum
 *  @brief this enum defines the action requested by an another tasks by e.g.
 *  the serial task can ask some action to the sensor task (srvm2m_actionId_enum)
 */
typedef enum {
	SRVACTREC_ACTION_ON = 0x00,				/*!< the calibration has not be carried out */
	SRVACTREC_ACTION_OFF,					/*!< the calibration process is on going*/
	SRVACTREC_ACTION_STATE,					/*!< the sensor is calibrated*/
}srvActivityRecorder_ActionId_enum;


/**
 *  @enum srvActivityRecorder_CalibrationStatus_enum
 *  @brief this enum defines the calibration status
 */
typedef enum {
	SRVACTREC_STATUS_NOTDONE = 0x00,				/*!< the calibration has not be carried out */
	SRVACTREC_STATUS_ONGOING,					/*!< the calibration process is on going*/
	SRVACTREC_STATUS_DONE,						/*!< the sensor is calibrated*/
	SRVACTREC_STATUS_ERROR=0xFF,					/*!< the calibration process returns an error code*/
}srvActivityRecorder_CalibrationStatus_enum;




/*===========================================================================================================
						struct definition
===========================================================================================================*/

/** @struct 	srvActivityRecorder_status_struct
 *  @brief 		This structure contains the status of the service about the activity recorder
 *  @var srvActivityRecorder_status_struct::ui16Activitytime
 *  Member 'ui16Activitytime' is the number of hours of activity
 *  @var srvActivityRecorder_status_struct::ui16NbActivity
 *  Member 'ui16NbActivity' is the number of confirmed activity detection
 *  @var srvActivityRecorder_status_struct::ui8FalseDetection
 *  Member 'ui8FalseDetection' is the number of false detection fired by the ADXL363
 *  @var srvActivityRecorder_status_struct::aeSensorCalStatus
 *  Member 'aeSensorCalStatus' is the status of the calibration of the sensor
 *  @var srvActivityRecorder_status_struct::aeSensorLogStatus
 *  Member 'aeSensorLogStatus' is the status of the log mode. The sensor measures
 *  until an array is full and the measure is stopped.
 *  @var srvActivityRecorder_status_struct::aeSensorContinuousStatus
 *  Member 'aeSensorContinuousStatus' is the status of the continuous mode. The measurement
 *  is enable to run continuously and the data is send to an another task.
 */
typedef struct
{
	uint16_t 	ui16Activitytime;
	uint16_t 	ui16NbActivity ;
	uint8_t 	ui8FalseDetection;
	bool 		IsAdxlLowpowerMode;
	srvActivityRecorder_CalibrationStatus_enum 		aeSensorCalStatus[KERNEL_SENSOR_ID_NBSENSOR];
	srvActivityRecorder_CalibrationStatus_enum 		aeSensorLogStatus[KERNEL_SENSOR_ID_NBSENSOR];
	srvActivityRecorder_CalibrationStatus_enum 		aeSensorContinuousStatus[KERNEL_SENSOR_ID_NBSENSOR];
	srvActivityRecorder_CalibrationStatus_enum 		eFFTcomputationStatus;
}srvActivityRecorder_status_struct;

/** @struct 	srvActivityRecorder_AxisBuffer_struct
 *  @brief 		This structure contains the buffer for the thre axis
 *  @var srvActivityRecorder_AxisBuffer_struct::Xaxis
 *  Member 'Xaxis' is the buffer for the X axis
 *  @var srvActivityRecorder_AxisBuffer_struct::Xaxis
 *  Member 'yaxis' is the buffer for the Y axis
 *  @var srvActivityRecorder_AxisBuffer_struct::Xaxis
 *  Member 'Zaxis' is the buffer for the Z axis
 */
typedef struct
{
	int16_t 	*Xaxis;
	int16_t 	*Yaxis;
	int16_t 	*Zaxis;
}srvActivityRecorder_AxisBuffer_struct;


/*===========================================================================================================
						extern definition
===========================================================================================================*/

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	srvActRec_InitActivityRecorder 		( void );
void 	srvActRec_IncreaseActivityTime 		( void );
void 	srvActRec_IncreaseActivityCounter	( void );
void 	srvActRec_GetActivityTime 			( uint16_t  *pui16ActivityRecorder);
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
void 	srvActRec_ProcessHowMuchTimeYouRun 	( uint8_t ui8ReadOrWriteAccess );
void	srvActRec_ProcessHowManyTimeYouRun 	( uint8_t ui8ReadOrWriteAccess );
uint8_t srvActRec_ProcessCalibration 		( Kernel_QueueItem_struct * psQueueItem );
uint8_t srvActRec_ProcessLog 				( Kernel_QueueItem_struct * psQueueItem );
uint8_t srvActRec_ProcessComputeFFT			( Kernel_QueueItem_struct * psQueueItem );
uint8_t srvActRec_ProcessAcceleromterIRQ 	( Kernel_QueueItem_struct *sQueueItem );
uint8_t srvActRec_ProcessMagnetoIRQ 		( Kernel_QueueItem_struct *psQueueItem );
#endif
uint8_t srvActRec_EnableMotionDetection 	( bool bOnOrOff );
uint8_t srvActRec_CalibrateAcceleroDetector ( void );
uint8_t srvActRec_IsCalibrationIsonGoing 	( void );
#endif /* SRV_LIS2DH12_H_ */
