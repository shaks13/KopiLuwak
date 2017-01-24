/*******************************************************************************
 * @file srvlis3mdl.c
 * @brief this file defines the command set for the LISM3MDL (magnetometer)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRV_LIS3MDL_H_
#define SRV_LIS3MDL_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "protocol_lis3mdl.h"



/*===========================================================================================================
						constant definition
===========================================================================================================*/
#if APP_ACTIVITYCOUNTER == APP_CHOSEN_FLAG
#define SRVLIS3MDH_BUFFER_NBELEMENT					128
#else
#define SRVLIS3MDH_BUFFER_NBELEMENT					10
#endif

/*===========================================================================================================
						Enum definition
===========================================================================================================*/

/*===========================================================================================================
						struct definition
===========================================================================================================*/

/** @struct srvtemp_TempBuffer_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var srvtemp_TempBuffer_struct::ui16Nbelement
 *  Member 'ui16Nbelement' is the number of measure saved in the buffer
 *  @var srvtemp_TempBuffer_struct::aui8MeasBuffer
 *  Member 'aui16MeasBuffer' is the magnetometer measurement
 */
typedef struct
{
	uint16_t ui16Nbelement;
	int16_t aui16MeasBuffer [SRVLIS3MDH_BUFFER_NBELEMENT];
}srvlis3mdl_MagnetoBuffer_struct;

/** @struct srvlis3mdl_FFTBuffer_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var srvlis3mdl_FFTBuffer_struct::ui16Nbelement
 *  Member 'ui16Nbelement' is the number of measure saved in the buffer
 *  @var srvlis3mdl_FFTBuffer_struct::aui8MeasBuffer
 *  Member 'aui16MeasBuffer' is the magnetometer measurement
 */
typedef struct
{
	uint16_t ui16Nbelement;
	uint16_t aui16MeasBuffer [SRVLIS3MDH_BUFFER_NBELEMENT];
}srvlis3mdl_FFTBuffer_struct;

/*===========================================================================================================
						extern definition
===========================================================================================================*/

/*===========================================================================================================
						prototype
===========================================================================================================*/
uint8_t srvlis3mdl_Init 						( void );
uint8_t srvlis3mdl_EnableMagneticDetection 		( bool bEnableOrDisable );
void 	srvlis3mdl_ProcessMagnetoIrq 			( void );
void 	srvlis3mdl_GetMeasBuffer 				( int16_t **i16pdata );
void 	srvlis3mdl_GetFFTBuffer 				( uint16_t **ui16pdata );
void 	srvlis3mdl_GetNbMeas 					( uint16_t *ui16NbMeasurement );
void 	srvlis3mdl_GetNbFFTpoint 				( uint16_t *ui16NbMeasurement );
void 	srvlis3mdl_SetNbMeas 					( const uint16_t ui16NbMeasurement);
void 	srvlis3mdl_SetNbFFTMeas 				( const uint16_t ui16NbMeasurement );
void 	srvlis3mdl_MagnetoCB 					( void );
uint8_t srvlis3mdl_GetAbsMagneticFieldOn3Axis 	( uint16_t * ui16MagneticField );
#endif /* SRV_LIS2DH12_H_ */
