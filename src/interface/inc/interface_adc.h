/*******************************************************************************
 * @file interface_adc.h
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef INTERFACEADC0_H
#define INTERFACEADC0_H


#include "em_adc.h"
#include "em_cmu.h"
//#include "em_prs.h"
//#include "em_timer.h"
//#include "bsp.h"

#include "common_library.h"
//#include "kernel_common.h"


/* Elements of the external temperature sensor linear law according to Table 1. LM94022 Transfer Table */
/* for Gain Select = "00" */
#define INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS00 	(float)-5.5
#define INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS00	(uint16_t)1034
/* for Gain Select = "01" */
#define INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS01	(float)-8.2
#define INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS01	(uint16_t)1567
/* for Gain Select = "10" */
#define INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS10	(float)-10.9
#define INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS10	(uint16_t)2100
/* for Gain Select = "11" */
#define INTERFACEADC_EXT_TEMP_COEF_SLOPE_GS11	(float)-13.6
#define INTERFACEADC_EXT_TEMP_COEF_OFFSET_GS11	(uint16_t)2633
/* ADC0 12bit resolution */
#define INTERFACEADC_MAXVALUE_RESOLUTION_12BIT	(uint16_t)4096
/* logic level on 2 inputs GS0 and GS1 of external temperature sensor (cf. LM94022) */
#define INTERFACEADC_GAIN_SELECT_EXT_TEMP_SENSOR (uint8_t)0 /* min = 0, max = 3 */
/* Thermometer output gradient (ADCcode/�C), cf. reference manual (Table 3.14. ADC) */
#define INTERFACEADC_THERMO_GRAD 				(float)-6.3

#define INTERFACEADC_BSP_ADC0_REFVDD_MV	(uint16_t)2100

/***************************************************************************//**
 * @brief
 *   Initializes ADC0.
 ******************************************************************************/
void interfaceADC_setup(void);
#if 0
void ADC0_init_single_RSSI(void);
void ADC0_init_RSSItriggeredbytimer (void);
void adc0_InitScan(void);
void ADC0_Enable_RSSItriggeredbytimer(void);
void ADC0_Enable_StartRssiMeas(void);
#endif
uint8_t interfaceADC_IsMeasuring (void);
uint8_t interfaceADC_IsDataAvailaible (void);

void interfaceADC_initSingleInternalTemp (void);
void interfaceADC_init_single_XTEMP(void);
int16_t interfaceADC_convertToCelsius(int32_t adcSample);
void interfaceADC_init_single_ACQ1(void);
void interfaceADC_init_single_VBAT(void);
/* Get channel 1 acquisition on monitoring board */
uint16_t interfaceADC_get_acq1_value (void);
/* Returned the ADC0 raw data corresponding to the temperature sensor channel */
int16_t interfaceADC_getRawData_XTemp(void);
/* Converted raw data  from the external temperature sensor into degrees Celsius */
//int16_t extTempConvertToCelsius(const int32_t i32AdcSample,const int8_t ucGainSelectSensor);
void interfaceADC_EnableIrqConversionOver (void);
void interfaceADC_DisableIrqConversionOver (void);


/*__INLINE*/ extern uint32_t interfaceADC_GetStatus (ADC_TypeDef *adc , const uint32_t flags );
extern void interfaceADC_GetTempAt125 (uint16_t *hwTemperAt1V25);
extern void interfaceADC_GetCalibrationTemp (uint16_t *hwCalibTemper);
extern uint32_t interfaceADC_GetInt(ADC_TypeDef *adc);
/*__INLINE*/ extern void interfaceADC_uploadConfig (ADC_TypeDef *adc, uint32_t *wSingleRegister, uint32_t *wScanRegister );
/*__INLINE*/ extern void interfaceADC_downloadConfig (ADC_TypeDef *adc, uint32_t wSingleRegister, uint32_t wScanRegister );


#endif

