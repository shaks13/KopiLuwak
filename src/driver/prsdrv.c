/*******************************************************************************
 * @file prddrv.c
 * @brief this function set manages the PRS
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "prsdrv.h"

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/


/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief this function initializes the PRS
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return		none
 ******************************************************************************/
void prsdrv_SetCryoTimerCh ( void )
{
	CMU_ClockEnable(cmuClock_PRS, true);

	/* PRS producer:
	 * source: CRYO TIMER
	 * signal: cryo timer PERIOD interruption */
	PRS_SourceAsyncSignalSet(PRSDRV_NUM_CHAN_CRYO, PRS_CH_CTRL_SOURCESEL_CRYOTIMER, PRS_CH_CTRL_SIGSEL_CRYOTIMERPERIOD);
}


/***************************************************************************//**
 * @brief this function initializes the PRS
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return		none
 ******************************************************************************/
void prsdrv_setupPrs ( void )
{

	CMU_ClockEnable(cmuClock_PRS, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	//GPIO_PinModeSet(INTERFACE_GPIOIN_PORT,INTERFACE_GPIOIN_PIN, gpioModeInput, 1);	/* PB12*/
	//interface_EnablePinInterrupt (INTERFACE_GPIOIN_PORT, INTERFACE_GPIOIN_PIN , INTERFACE_FALLING_EDGE); /* enable the IRQ */
	//GPIO_PinModeSet(INTERFACE_GPIOOUT_PORT,INTERFACE_GPIOOUT_PIN, gpioModePushPull, 1); /*PF7*/

	PRS_SourceAsyncSignalSet(1, PRS_CH_CTRL_SOURCESEL_GPIOL, PRS_CH_CTRL_SIGSEL_GPIOPIN4); /* the source is PA4*/


	#if 0
	PRS->ROUTELOC0 = 	PRS_ROUTELOC0_CH1LOC_LOC6 ; /* channel 1 loc6 = PF7*/
	PRS->ROUTEPEN = PRS_ROUTEPEN_CH1PEN ;
	#endif
	PRS->DMAREQ0= PRS_DMAREQ0_PRSSEL_PRSCH1; /* */
}
