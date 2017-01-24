/*******************************************************************************
 * @file dsplib_processsignal.c
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "dsplib_processsignal.h"

static void dsplib_multipyByHanningWindows ( float32_t *pf32DataIn , float32_t *pf32DataOut , const uint16_t ui16NbData);

/***************************************************************************//**
 * @brief this function multipplies point by point the input array to the hanning
 * window.
 * @detail the formula of the hanning window is 1/2 * (1-cos (2*PI*n/N-1)) with
 * 	n the index of the current data
 * 	N the number of input data
 * @param[in] pf16DataIn 	: pointer on the input data
 * @param[in] ui16NbData 	: number of data of the input data
 * @param[out] pf16DataOut 	: pointer on the fft result
 * @return none
*******************************************************************************/
static void dsplib_multipyByHanningWindows ( float32_t *pf32DataIn , float32_t *pf32DataOut , const uint16_t ui16NbData)
{
uint16_t ui16NthPoint=0;
const float32_t f32constant = 2*PI/(ui16NbData-1);

	do{
		pf32DataOut[ui16NthPoint] = pf32DataIn[ui16NthPoint] * (0.5*(1-cos(f32constant*ui16NthPoint)));
	}while (ui16NthPoint++<ui16NbData);

}


/***************************************************************************//**
 * @brief compute the absolute value of the real part of the FFT
 * @param[in] pf16DataIn 	: pointer on the input data
 * @param[in] ui16NbData 	: number of data of the input data
 * @param[out] pf16DataOut 	: pointer on the fft result
 * @return none
*******************************************************************************/
void dsplib_computefft ( float32_t *pf32DataIn , float32_t *pf32DataOut , const uint16_t ui16NbData)
{
arm_rfft_fast_instance_f32  sFFTinstance;

	arm_rfft_fast_init_f32 (&sFFTinstance,ui16NbData);
	/* compute the fft*/
	arm_rfft_fast_f32 (&sFFTinstance,pf32DataIn,pf32DataOut,false);
	/*
	 * The CMSIS function arm_abs_f32 computes the absolute value of a real number(s). The output samples of FFT are always complex numbers.
	 * "Real FFT" means the input samples are real-valued but the output samples are still complex-valued. Hence, you should use arm_cmplx_mag_f32 instead.
	 * https://community.arm.com/thread/9532
	*/
	arm_cmplx_mag_f32 (pf32DataOut,pf32DataOut,ui16NbData/2);
}


/***************************************************************************//**
 * @brief compute the absolute value of the real part of the FFT. First the input
 * data are multiplied by the Hanning window
 * @param[in] pf16DataIn 	: pointer on the input data
 * @param[in] ui16NbData 	: number of data of the input data
 * @param[out] pf16DataOut 	: pointer on the fft result
 * @return none
*******************************************************************************/
void dsplib_computeHannfft ( float32_t *pf32DataIn , float32_t *pf32DataOut , const uint16_t ui16NbData)
{
	dsplib_multipyByHanningWindows (pf32DataIn , pf32DataOut ,  ui16NbData); 	/* multiply the input data by the Hanning window*/
	dsplib_computefft ( pf32DataOut , pf32DataOut , ui16NbData);				/* compute the FFT*/
}
