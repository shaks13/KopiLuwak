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
#ifndef SRV_ADXL363_H_
#define SRV_ADXL363_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "protocol_adxl363.h"
#include "letimerdrv.h"


/*===========================================================================================================
						constant definition
===========================================================================================================*/
#if APP_ACTIVITYCOUNTER == APP_CHOSEN_FLAG
#define SRVADXL363_BUFFER_NBELEMENT			(1024)
#else
#define SRVADXL363_BUFFER_NBELEMENT			10
#endif
/*===========================================================================================================
						Enum definition
===========================================================================================================*/
/**
 *  @enum srvadxl36x_AxisID_enum
 *  @brief this enum defines the axis Id
 */
typedef enum {
	SRVADXL36X_AXISID_X = 0x00,					/*!< the axis id for the the X axis */
	SRVADXL36X_AXISID_Y ,						/*!< the axis id for the the Y axis */
	SRVADXL36X_AXISID_Z ,						/*!< the axis id for the the Z axis */
}srvadxl36x_AxisID_enum;


/**
 *  @enum srvadxl36x_State_enum
 *  @brief this enum defines the state of the accelerometer
 */
typedef enum {
	SRVADXL36X_STATE_ON = 0x00,					/*!< the acceleromter is on*/
	SRVADXL36X_STATE_OFF ,						/*!< the acceleromter is off*/
}srvadxl36x_State_enum;

/**
 *  @enum srvadxl36x_Mode_enum
 *  @brief this enum defines the axis Id
 */
typedef enum {
	SRVADXL36X_MODE_MOTIONDETECTION = 0x00,					/*!< the mode motion detection is selected */
	SRVADXL36X_MODE_LOG ,									/*!< the mode log is selected */
	SRVADXL36X_MODE_MEASUREMENT ,							/*!< the mode measurement is selected */
	SRVADXL36X_MODE_OFF ,							/*!< the sensor is deactivated */
}srvadxl36x_Mode_enum;



/*===========================================================================================================
						struct definition
===========================================================================================================*/

/** @struct srvdxl363_Status_struct
 *  @brief This structure contains the state of the accelerometer
 *  @var srvdxl363_Status_struct::ui8state
 *  Member 'ui8state' is the state of the accelerometer (activate or not)
 */
typedef struct
{
	srvadxl36x_State_enum ui8state;
	srvadxl36x_Mode_enum ui8mode;

}srvdxl363_State_struct;

/** @struct srvtemp_TempBuffer_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var srvtemp_TempBuffer_struct::ui16Nbelement
 *  Member 'ui16Nbelement' is the number of measure saved in the buffer
 *  @var srvtemp_TempBuffer_struct::aui8MeasBuffer
 *  Member 'aui8MeasBuffer' is the temperature measurement in celcius
 */
typedef struct
{
	uint16_t ui16XaxisNbelement;
	uint16_t ui16YaxisNbelement;
	uint16_t ui16ZaxisNbelement;
	int16_t aui8XaxisBuffer [SRVADXL363_BUFFER_NBELEMENT];
	int16_t aui8YaxisBuffer [SRVADXL363_BUFFER_NBELEMENT];
	int16_t aui8ZaxisBuffer [SRVADXL363_BUFFER_NBELEMENT];
}srvdxl363_MeasBuffer_struct;

/** @struct srvtemp_TempBuffer_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var srvtemp_TempBuffer_struct::ui16Nbelement
 *  Member 'ui16Nbelement' is the number of measure saved in the buffer
 *  @var srvtemp_TempBuffer_struct::aui8MeasBuffer
 *  Member 'aui8MeasBuffer' is the temperature measurement in celcius
 */
typedef struct
{
	uint16_t ui16FFTNbelement;
	uint16_t aui16Buffer [SRVADXL363_BUFFER_NBELEMENT/2];
}srvdxl363_FFTBuffer_struct;
/*===========================================================================================================
						extern definition
===========================================================================================================*/

/*===========================================================================================================
						prototype
===========================================================================================================*/
uint8_t srvadxl363_Init 					( const srvadxl36x_Mode_enum emode );
void 	srvadxl363_ReadAcceloMeasure 		( uint8_t ** pui8XYZaxis );
void 	srvadxl363_ReadStatus 				( uint8_t *pui8Response );
void	 srvadxl363_GetAxisMeasBuffer 		( int16_t **i16pXdata, int16_t **i16pYdata, int16_t **i16pZdata );
void 	srvadxl363_GetMeasBuffer	 		( const Kernel_MeasurementId_enum eMeasurementId , int8_t **i8pdata );
void 	srvadxl363_GetNbMeas 				( const Kernel_MeasurementId_enum eMeasId, uint16_t *ui16NbMeasurement );
void 	srvadxl363_SetNbMeas 				( const Kernel_MeasurementId_enum eMeasId, const uint16_t ui16NbMeasurement );
void 	srvadxl363_AcceleroCB 				( void );
void 	srvadxl363_EnableMotionIrq 			( const bool bOnorOff  );
void 	srvadxl363_EnableDataReadyIrq 			( const bool bOnorOff  );
uint8_t	srvadxl363_InitLogMeasurement 		( const bool bOnOrOff );
bool 	srvadxl363_IsEnable 				( void );
#endif /* SRV_LIS2DH12_H_ */
