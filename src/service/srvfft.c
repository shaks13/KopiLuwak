/*******************************************************************************
 * @file stvfft.c
 * @brief this services computes the FFT
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)

#include "srvfft.h"


/*===========================================================================================================
						Private variables definition
===========================================================================================================*/
static 	srvfft_FFTBuffer_struct af32RawData ;
static 	srvfft_FFTBuffer_struct af32FFTbuffer ;
/*===========================================================================================================
						Private functions definition
===========================================================================================================*/

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief		this function copies the raw data in the raw FFT buffer
 * @param[in] 	pi16RawData : pointer on the raw data to be copied in the
 * raw data pool
 * @param[in] 	ui16Nbdata : number of element to be copied
 * @return CROSSRFID_SUCCESSCODE : the function is successful
 * @return CROSSRFID_ERROR_WRONGPARAMETER : one parameter is erroneous
 ******************************************************************************/
uint8_t srvfft_CopyRawData (int16_t const *pi16RawData , const uint16_t ui16Nbdata )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
uint16_t ui16Nthelemt;

	if (ui16Nbdata>SRVFFT_BUFFER_NBELEMENT)
	{
		ui8status  = CROSSRFID_ERROR_WRONGPARAMETER;
	}
	else
	{
		for (ui16Nthelemt = 0 ; ui16Nthelemt <ui16Nbdata ; ui16Nthelemt++)
		{
			af32RawData.aui16MeasBuffer[ui16Nthelemt] =  (float32_t) pi16RawData[ui16Nthelemt];
		}
		af32RawData.ui16Nbelement = ui16Nbdata;
	}
	return ui8status;
}

/***************************************************************************//**
 * @brief		this function copies the fft result in the given array. the
 * data are casted as uin16-t because the result is the magnitude of the
 * FFT and divided by the number of point.
 * @param[out] 	pui16fftData : pointer where the data willl be copy
 * @param[out] 	ui16Nbelemt : number of element copied
 * @return CROSSRFID_SUCCESSCODE : the function is successful
 * @return CROSSRFID_ERROR_WRONGPARAMETER : one parameter is erroneous
 ******************************************************************************/
uint8_t srvfft_CopyFFTData (uint16_t *pui16fftData , const uint16_t ui16Nbelemt )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
uint16_t ui16Nthelemt=0;

	af32FFTbuffer.ui16Nbelement	= af32RawData.ui16Nbelement/2;
	for (ui16Nthelemt = 0 ; ui16Nthelemt <af32FFTbuffer.ui16Nbelement ; ui16Nthelemt++)
	{
		pui16fftData[ui16Nthelemt] = (uint16_t) (af32FFTbuffer.aui16MeasBuffer[ui16Nthelemt]/af32RawData.ui16Nbelement);
	}

	return ui8status;
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 * @param[in] none
 * @param[out] none
 * @return none
*******************************************************************************/
uint8_t srvfft_ComputeFFT ( void )
{
uint8_t status = CROSSRFID_SUCCESSCODE;


	dsplib_computefft (af32RawData.aui16MeasBuffer,af32FFTbuffer.aui16MeasBuffer,af32RawData.ui16Nbelement);

	return status;
}

/***************************************************************************//**
 * @brief 			this function updated the number of element to be computed.
 * @param[in,out] ui16Nbelemt : the number of element to be computed
 * @param[out] none
 * @return none
*******************************************************************************/
void srvfft_UpdateNbMeas ( uint16_t *ui16Nbelemt )
{

	if ((*ui16Nbelemt)>SRVFFT_BUFFER_NBELEMENT )
	{
		(*ui16Nbelemt) = SRVFFT_BUFFER_NBELEMENT;
	}else {/* do nothing*/}

	if ((*ui16Nbelemt) >= 2048u)
	{
		(*ui16Nbelemt) = 2048u;
	}
	else if ((*ui16Nbelemt) >= 1024u)
	{
		(*ui16Nbelemt) = 1024u;
	}
	else if ((*ui16Nbelemt) >= 512u)
	{
		(*ui16Nbelemt) = 512u;
	}
	else if ((*ui16Nbelemt) >= 256u)
	{
		(*ui16Nbelemt) = 256u;
	}
	else if ((*ui16Nbelemt) >= 128u)
	{
		(*ui16Nbelemt) = 128u;
	}
	else if ((*ui16Nbelemt) >= 64u)
	{
		(*ui16Nbelemt) = 64u;
	}
	else if ((*ui16Nbelemt) >= 32u)
	{
		(*ui16Nbelemt) = 32u;
	}
	else if ((*ui16Nbelemt) >= 16u)
	{
		(*ui16Nbelemt) = 16u;
	}
	else
	{
		(*ui16Nbelemt) = 0;
	}

}



#endif /* APP_ACTIVITYCOUNTER */
