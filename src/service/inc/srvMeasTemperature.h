/*******************************************************************************
 * @file srvMeasTemperature.h
 * @brief this file defines the commans set for the temperature mesurement
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVMEASTEMPERTURE_H_
#define SRVMEASTEMPERTURE_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "interface_adc.h"
#include "tempdrv.h"
#include "rtcdriver.h"
#include "cryodrv.h"
#include "srvEM4325.h"
#include "srvFlashMemory.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#define DATAACQ_MEASBUFFER_NBELEMENT				(NVM_SIZE_SAMPLE_BUFFER_IN_WORD)
#else
#define DATAACQ_MEASBUFFER_NBELEMENT				(10)
#endif

/*===========================================================================================================
						Enum definition
===========================================================================================================*/

/**
 *  @enum srvtemp_tempAlarmType_enum
 *  @brief this enum contains the different types of the alarms
 */
typedef enum {
	SRVTEMP_ALARMTYPE_LOW =			0x00,			/*!< type for the low threshold */
	SRVTEMP_ALARMTYPE_HIGH 				,			/*!< type for the low threshold */

}srvtemp_tempAlarmType_enum;

/*===========================================================================================================
						structure definition
===========================================================================================================*/

/** @struct srvtemp_TempBuffer_struct
 *  @brief This structure contains the temperature buffer ad the number of
 *  samples in the buffer.
 *  @var srvtemp_TempBuffer_struct::ui16Nbelement
 *  Member 'ui16Nbelement' is the number of measure saved in the buffer
 *  @var srvtemp_TempBuffer_struct::aui8MeasBuffer
 *  Member 'aui8MeasBuffer' is the temperature measurement in celcius
 */
typedef struct
{
	uint16_t ui16Nbelement;
	int16_t ai16MeasBuffer [DATAACQ_MEASBUFFER_NBELEMENT];
}srvtemp_TempBuffer_struct;

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	srvtemp_init 					( bool bEnableorNot );
void 	srvtemp_MeasureOnchipTemp 		(int8_t * pi8Result);
//void srvtemp_ReleaseDatapoolObject (uint8_t *pdatapoolobject);
void 	srvtemp_InitOnchipTemp 			( void );

void 	srvtemp_getOnchipTempRawData	(uint16_t *hwTempRawData);
void 	srvtemp_convertToCelsius		(const uint16_t hwAdcSample, int16_t *hw16Result);
uint8_t srvtemp_MeasureTemperature 		( void );
bool 	srvtemp_SetPeriodOnchipTemp 	(const uint16_t ui16Period, const uint16_t ui16Unit);
void 	srvtemp_StartOnchipTempAcq 		( const bool bStart);
void 	srvtemp_SetTempAlarmCB 			( int8_t i8Thresh, bool bIsHighThreshold);
void 	srvtemp_RemoveTempAlarmCB 		( bool bIsHighThreshold);
void 	srvtemp_ThermoCB 				( void  );
void 	srvtemp_GetBufferTemp 			( int8_t **ppi8pdata );
void 	srvtemp_GetNbMeas 				( uint16_t *ui16NbMeasurement );
bool 	srvEM4325_IsAlarmOn 			( uint8_t ui8type, uint8_t ui8IsEnable, int16_t i16CurrentTeperature , int16_t i16Threshold );

void 	srvtemp_SendLowTempAlarmState	( const int16_t i16temp);
void 	srvtemp_SendHighTempAlarmState 	( const int16_t i16temp);
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
uint8_t	srvtemp_RestoreFlashToRamObjects( void );
int16_t srvtemp_GetLastTempAverage		( void );
uint8_t srvtemp_ComputeAvgOnSectionOfTempBuffer ( const uint16_t ui16IdxSample, const uint16_t ui16NbSample, int16_t * pi16Avg);
#endif

#endif /* SRVMEASTEMPERTURE_H_ */
