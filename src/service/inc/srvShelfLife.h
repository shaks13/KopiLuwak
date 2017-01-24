/*******************************************************************************
 * @file srvShelfLife.h
 * @brief ShelfLife service API
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SRVGLUEPOT_H_
#define SRVGLUEPOT_H_

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)

#include "common_library.h"
#include "kernel_common.h"
#include "protocol_EM4325.h"
#include "srvMeasTemperature.h"

#define		SRV_SHELFLIFE_TENTH_MAX_TEMPERATURE 		(6)
#define		SRV_SHELFLIFE_TENTH_MIN_TEMPERATURE 		(-5)

/* tenth max - tenth min + 1 (for the 0 degree) */
#define		SRV_SHELFLIFE_NB_TEMPERATURE_IN_2D_ARRAY 	(SRV_SHELFLIFE_TENTH_MAX_TEMPERATURE - SRV_SHELFLIFE_TENTH_MIN_TEMPERATURE + 1)

#define		SRV_SHELFLIFE_NB_SHELFLIFER_IN_2D_ARRAY 	(11)

/* helps to find the temperature index in the 2D array because idx = 0 is not the 0 degree */
#define		SRV_SHELFLIFE_OFFSET_IDX_TEMPERATURE 		(5)

/* initial shelf life of the product in day */
#define 	SRV_SHELFLIFE_INITIAL_LIFE_IN_DAY 			(260)

#define		SRV_SHELFLIFE_INIALI_ABSOLUTE_LIFE			((double)(1.0f))

#if (GLUEPOT_DEBUG == 0)
/* unit: 1 minute in absolute format */
#define		SRV_SHELFLIFE_TIME_DELTA_BETWEEN_CALCULATION (double)(SRV_SHELFLIFE_INIALI_ABSOLUTE_LIFE/((uint32_t)24))
#else
/* unit: 1 second in absolute format */
#define		SRV_SHELFLIFE_REAL_TIME_DELTA_BETWEEN_CALCULATION (double)(SRV_SHELFLIFE_INIALI_ABSOLUTE_LIFE/((uint32_t)/*SRV_SHELFLIFE_INITIAL_LIFE_IN_DAY* */24*60*60))
/* speed up the test: 1s in real is 10 days in the process */
#define		SRV_SHELFLIFE_TIME_DELTA_BETWEEN_CALCULATION (double)(SRV_SHELFLIFE_REAL_TIME_DELTA_BETWEEN_CALCULATION*24*60*60*10)
#endif
/*===========================================================================================================
						Public functions declarations
===========================================================================================================*/
uint8_t srvShelfLife_ComputeShelfLife 	(void);
double 	srvShelfLife_GetShelfLife		(void);
void 	srvShelfLife_ResetShelfLife		(void);
#endif /* APP_GLUEPOT */

#endif /* SRVGLUEPOT_H_ */
