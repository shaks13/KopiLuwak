/***************************************************************************//**
 * @file cryodrv.h
 * @brief CRYO TIMER API definition.
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef CRYODRV_H_
#define CRYODRV_H_

#include "common_library.h"
#include "em_cryotimer.h"
#include "em_cmu.h"
#include "em_device.h"
#include "FreeRTOSConfig.h"
#include "kernel_common.h"

/*===========================================================================================================
						constants
===========================================================================================================*/
#define CRYODRV_PRESC_LFRCO (1) /* Must be consistent with the selected CRYOTIMER_Presc_TypeDef */
#define CRYODRV_LFRCO_FREQ_WITH_PRESC  (32768UL/CRYODRV_PRESC_LFRCO)

/** EM4 CRYOTIMER init structure, interruption period = 30.5s. */
#define CRYOTIMER_INIT_EM4											\
{																	\
  true,                 /* Start counting when init done.    	*/ 	\
  false,                /* Disable CRYOTIMER during debug halt.	*/ 	\
  true,                 /* Enable EM4 wakeup.             		*/ 	\
  cryotimerOscLFXO,    /* Select Low Frequency RC Oscillator.	*/ 	\
  cryotimerPresc_1,     /* LF Oscillator frequency undivided.	*/ 	\
  cryotimerPeriod_1m, 	/* Wakeup event after 1000000 pre-scaled clock cycles. */ \
}

#define CRYODRV_USING_FREE_RTOS_TO_MONITOR /* Todo removed, define if prs+dma not used for the sample monitoring  */

#define CRYODRV_NB_ENUM_PERIOD (cryotimerPeriod_4096m+1)
/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/
void cryodrv_Init(void);
void cryodrv_SetPrescaler(const CRYOTIMER_Presc_TypeDef prescaler);
bool cryodrv_ConvPeriodToNbCycleAndPresc(const uint16_t ui16Period, const uint16_t ui16unit, CRYOTIMER_Period_TypeDef * pNbCycle, CRYOTIMER_Presc_TypeDef * pPrescaler);
#endif /* CRYODRV_H_ */
