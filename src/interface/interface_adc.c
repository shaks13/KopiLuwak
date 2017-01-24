/*******************************************************************************
 * @file interface_adc.c
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "interface_adc.h"

#if 0
void interfaceADC_setup(void)
{
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

  init.ovsRateSel = adcOvsRateSel2;
  init.lpfMode    = adcLPFilterBypass;
  init.warmUpMode = adcWarmupNormal;
  init.timebase   = ADC_TimebaseCalc(0);
  init.prescale   = ADC_PrescaleCalc(7000000, 0);
  init.tailgate   = 0;

  ADC_Init(ADC0, &init);

  /* Initialize a single sample conversion. */
  /* To start a conversion, use ADC_Start(). */
  /* Conversion result can be read with ADC_DataSingleGet(). */
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  initSingle.prsSel     = adcPRSSELCh0;
  initSingle.acqTime    = adcAcqTime16;
  initSingle.reference  = adcRefVDD;
  initSingle.resolution = adcRes12Bit;
  initSingle.input      = adcSingleInpCh4;
  initSingle.diff       = 0;
  initSingle.prsEnable  = 0;
  initSingle.leftAdjust = 0;
  initSingle.rep        = 0;

  ADC_InitSingle(ADC0, &initSingle);
}





#endif

/***************************************************************************//**
 * @brief
 *   This function initializes the ADC to measure continuously the RSSI.
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
uint8_t interfaceADC_IsMeasuring (void)
{
uint8_t status;

	if ( interfaceADC_GetStatus (ADC0 ,_ADC_STATUS_SINGLEACT_MASK) != 0 )
	{
		status = true;
	}
	else
	{
		status = false;
	}
	return status;
}

/***************************************************************************//**
 * @brief
 *   This function returns true is a Single conversion data is available
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
uint8_t interfaceADC_IsDataAvailaible (void)
{
uint8_t status;

	if ( interfaceADC_GetStatus (ADC0 ,_ADC_STATUS_SINGLEDV_MASK) != 0 )
	{
		status = true;
	}
	else
	{
		status = false;
	}
	return status;
}

/***************************************************************************//**
 * @brief
 *   This function enables the ADC interrupt when the conversion is over
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void interfaceADC_EnableIrqConversionOver (void)
{
	ADC_IntEnable (ADC0, ADC_IEN_SINGLE);
}

/***************************************************************************//**
 * @brief
 *   This function enables the ADC interrupt when the conversion is over
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void interfaceADC_DisableIrqConversionOver (void)
{
	ADC_IntDisable  (ADC0, ADC_IEN_SINGLE);
}


/***************************************************************************//**
 * @brief
 *   This function initializes the ADC for the internal temperature measurement
 *
 * @details the forumla is the next one
 * 	TCELSIUS=CAL_TEMP_0-(ADC0_TEMP_0_READ_1V25- ADC_result)�Vref/(4096�TGRAD_ADCTH)
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void interfaceADC_initSingleInternalTemp(void)
{
  /* Initialize a single sample conversion. */
  /* To start a conversion, use ADC_Start(). */
  /* Conversion result can be read with ADC_DataSingleGet(). */
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  initSingle.prsSel     = adcPRSSELCh0;
  initSingle.acqTime    = adcAcqTime16;
  initSingle.reference  = _ADC_SINGLECTRL_REF_1V25;
  initSingle.resolution = adcRes12Bit;
  initSingle.posSel		= adcPosSelTEMP;
  initSingle.negSel		= 0;
  initSingle.diff       = 0;
  initSingle.prsEnable  = 0;
  initSingle.leftAdjust = 0;
  initSingle.rep        = 0;
  initSingle.singleDmaEm2Wu = false;
  initSingle.fifoOverwrite = false;

  ADC_InitSingle(ADC0, &initSingle);
}

/***************************************************************************//**
 * @brief
 *   This function initializes the ADC for the external temperature measurement
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void interfaceADC_init_single_XTEMP(void)
{
	/* Initialize a single sample conversion. */
	/* To start a conversion, use ADC_Start(). */
	/* Conversion result can be read with ADC_DataSingleGet(). */
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	initSingle.acqTime    = adcAcqTime16;
	initSingle.reference  = adcRefVDD;
	initSingle.resolution = adcRes12Bit;
	initSingle.posSel		= adcPosSelAPORT0XCH6;
	initSingle.negSel		= 0;
	initSingle.diff       = 0;
	initSingle.prsEnable  = 0;
	initSingle.leftAdjust = 0;
	initSingle.rep        = 0;
	initSingle.singleDmaEm2Wu = false;
	initSingle.fifoOverwrite = false;

	ADC_InitSingle(ADC0, &initSingle);
}


#if 0 /* not used anymore*/
/* Function : Convert ADC data (channel 6 : external temperature sensor) to Celsius */
int16_t extTempConvertToCelsius(const int32_t i32AdcSample,const int8_t ucGainSelectSensor)
{
	float 		f32Coef 	= (float)0.0;
	float		f32Tempor	= (float)0.0;
	uint16_t 	ui16Offset 	= (uint16_t)0;
	int16_t 	i16Result 	= (int16_t)0;
	/* select linear law (Voltage[mV] = f(temperature)) according to LM94022 */
	if(ucGainSelectSensor == 0)
	{
		f32Coef = INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS00;
		ui16Offset = INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS00;
	}
	else if(ucGainSelectSensor == 1)
	{
		f32Coef = INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS01;
		ui16Offset = INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS01;
	}
	else if(ucGainSelectSensor == 2)
	{
		f32Coef = INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS10;
		ui16Offset = INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS10;
	}
	else
	{
		f32Coef = INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS11;
		ui16Offset = INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS11;
	}

	/* calculate temperature = f(ADC sample)*/
	/* Vadc[mV] = ADC * Vref[mV] / 2^12 */
	f32Tempor = (float)(i32AdcSample * BSP_ADC0_REFVDD_MV / INTERFACEADC_MAXVALUE_RESOLUTION_12BIT);
	/* Temperature[�C] = (Vadc[mV] - B[mV]) / A[mV/�C] */
	f32Tempor = (float)((f32Tempor - ui16Offset) / f32Coef);
	/* x10 for TEST MODE DISPLAY (ex: 25.4�C -> 0x000254) */
	i16Result = (int16_t)f32Tempor;
	return i16Result;
}
#endif

void interfaceADC_init_single_ACQ1(void)
{
  /* Initialize a single sample conversion. */
  /* To start a conversion, use ADC_Start(). */
  /* Conversion result can be read with ADC_DataSingleGet(). */
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  initSingle.prsSel     = adcPRSSELCh0;
  initSingle.acqTime    = adcAcqTime16;
  initSingle.reference  = adcRefVDD;
  initSingle.resolution = adcRes12Bit;
  initSingle.posSel		= adcPosSelAPORT0XCH6;
  initSingle.negSel		= 0;
  initSingle.diff       = 0;
  initSingle.prsEnable  = 0;
  initSingle.leftAdjust = 0;
  initSingle.rep        = 0;
  initSingle.singleDmaEm2Wu = false;
  initSingle.fifoOverwrite = false;

  ADC_InitSingle(ADC0, &initSingle);
}

void interfaceADC_init_single_VBAT(void)
{
  /* Initialize a single sample conversion. */
  /* To start a conversion, use ADC_Start(). */
  /* Conversion result can be read with ADC_DataSingleGet(). */
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  initSingle.prsSel     = adcPRSSELCh0;
  initSingle.acqTime    = adcAcqTime16;
  initSingle.reference  = adcRefVDD;
  initSingle.resolution = adcRes12Bit;
  initSingle.posSel		= adcPosSelAPORT0XCH5;
  initSingle.negSel		= 0;
  initSingle.diff       = 0;
  initSingle.prsEnable  = 0;
  initSingle.leftAdjust = 0;
  initSingle.rep        = 0;
  initSingle.singleDmaEm2Wu = false;
  initSingle.fifoOverwrite = false;

  ADC_InitSingle(ADC0, &initSingle);
}

/* Function : Get acquisition value on monitoring board's CHANNEL 1 */
uint16_t interfaceADC_get_acq1_value (void)
{
	uint16_t acq_value= 0;
#if 0
	//Single acq1 measure
	/* Activate ACQ channel 1 power */
	interfaceacq_enableChannel1();
	/* Wait ACQ channel stabilization */
	/* TODO : should be reworked to remove this delay function*/
	//Delay_ms(1);
	interfaceADC_init_single_ACQ1();
	ADC_Start(ADC0, adcStartSingle);
	/* Wait while conversion is active */
	while (ADC_GetStatus (ADC0, ADC_STATUS_SINGLEACT));
	/* Get ADC result */
	acq_value = ADC_DataSingleGet(ADC0);
	interfaceacq_disableChannel1();
	interfaceADC_init_single_RSSI();
#endif
	return (acq_value);

}

/***************************************************************************//**
 * @brief
 *   This function manages the ADC interuptions
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 *
 ******************************************************************************/
void ADC0_IRQHandler (void)
{

}


/***************************************************************************//**
* @brief
*   This function loads the configuration of the ADCO
*
* @details this function was not available n the emlib
*
* @param[in] adc Pointer to ADC peripheral register block.
* @param[out] wScanRegister :  value of the scan register
* @param[out] wSingleRegister :  value of the single register
* @return none
*
******************************************************************************/
/*__INLINE*/ void interfaceADC_uploadConfig (ADC_TypeDef *adc, uint32_t *wSingleRegister, uint32_t *wScanRegister )
{
      adc->CMD = ADC_CMD_SINGLESTOP;
      (*wSingleRegister) = adc->SINGLECTRL ;
      (*wScanRegister) = adc->SCANCTRL ;
}



#if 0 /* not used anymore*/
/***************************************************************************//**
* @brief
*   Get the enabled interupt
*
* @param[in] adc Pointer to ADC peripheral register block.
*
* @return
*   ADC enabled interrupt.
******************************************************************************/
uint32_t interfaceADC_GetInt(ADC_TypeDef *adc)
{
  return adc->IEN;
}


/***************************************************************************//**
* @brief
*   returns the temperature reading at 1V25 reference
*
* @param[in] none
* @param[out] the temperature at 1.25 V
* @return none
******************************************************************************/
void interfaceADC_GetTempAt125 (uint16_t *hwTemperAt1V25)
{
      (*hwTemperAt1V25) = (uint16_t)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_NEGSEOFFSET2XVDD_MASK)    >> _DEVINFO_ADC0CAL2_NEGSEOFFSET2XVDD_SHIFT);
      // Before efm32g222: (*hwTemperAt1V25) = (uint16_t)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)    >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
}

/***************************************************************************//**
* @brief
*   returns the calibration temperature
*
* @param[in] none
* @param[out] the temperature at 1.25 V
* @return none
******************************************************************************/
void interfaceADC_GetCalibrationTemp (uint16_t *hwCalibTemper)
{
      (*hwCalibTemper) = (uint16_t)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
}



#endif


/***************************************************************************//**
* @brief
*   returns the status
*
* @note
*   Check data valid flag before calling this function.
*
* @param[in] adc
*   Pointer to ADC peripheral register block.
*
* @return
*   Scan conversion data.
******************************************************************************/
/*__INLINE*/ uint32_t interfaceADC_GetStatus (ADC_TypeDef *adc , const uint32_t flags )
{
  return  (adc->STATUS) & flags;
}





