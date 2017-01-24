/*******************************************************************************
 * @file srvCRC.h
 * @brief this file defines the API of the CRC service
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVFFT_H_
#define SRVFFT_H_
#include "common_library.h"
#include "common_statuscode.h"
#include "dsplib_processsignal.h"

/*===========================================================================================================
						Constants
===========================================================================================================*/
#define SRVFFT_BUFFER_NBELEMENT			(1024)

/*===========================================================================================================
						structure definition
===========================================================================================================*/
/** @struct srvfft_FFTBuffer_struct
 *  @brief This structure contains the pointer to the task handle.
 *  it should be aligned on the Kernel_TaskId_enum
 *  @var srvfft_FFTBuffer_struct::ui16Nbelement
 *  Member 'ui16Nbelement' is the number of measure saved in the buffer
 *  @var srvfft_FFTBuffer_struct::aui8MeasBuffer
 *  Member 'aui16MeasBuffer' is the magnetometer measurement
 */
typedef struct
{
	uint16_t ui16Nbelement;
	float32_t aui16MeasBuffer [SRVFFT_BUFFER_NBELEMENT];
}srvfft_FFTBuffer_struct;

/*===========================================================================================================
						Public functions declarations
===========================================================================================================*/
uint8_t srvfft_CopyFFTData 			( uint16_t *pui16RawData , const uint16_t ui16Nbelemt );
uint8_t srvfft_CopyRawData 			( int16_t const *pi16RawData , const uint16_t ui16Nbdata );
uint8_t srvfft_ComputeFFT 			( void );
void 	srvfft_UpdateNbMeas 			( uint16_t *ui16Nbelemt );

#endif /* SRVCRC_H_ */
