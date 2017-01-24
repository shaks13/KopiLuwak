/*******************************************************************************
 * @file letimerdrv.c
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "letimerdrv.h"


static void letimerdrv_Clear(LETIMER_TypeDef *letimer);
volatile bool bLeTimerElapsed = false ;

/***************************************************************************//**
 * @brief	This function clears the LE timer
 * @details
 * @param[in] 	letimer
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void letimerdrv_Clear(LETIMER_TypeDef *letimer)
{
  /* Make sure disabled first, before resetting other registers */
  letimer->CMD = LETIMER_CMD_CLEAR;

}


/**************************************************************************//**
 * @brief   this function configures and starts the LETIMER
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void letimerdrv_init( void )
{
/* Set configurations for LETIMER 0 */
const LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_ACTIVITYCOUNTER ;

	LETIMER_Enable(LETIMER0, false);
	/* Enable necessary clocks */
	CMU_ClockPrescSet (cmuClock_LETIMER0, cmuClkDiv_16384/*LETIMER_CLOCKDIVISION*/);	/* the LEtimer period is 1s*/
	/* The CORELE clock is also necessary for the RTC and all
	 low energy peripherals, but since this function
	 is called before RTC_setup() the clock enable
	 is only included here */
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	LETIMER_CompareSet (LETIMER0,0,LETIMER_COMPARESET); /* set Comp0 to 3600 s*/
	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	LETIMER_IntClear (LETIMER0, _LETIMER_IFC_MASK); /* Enable underflow interrupt */
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF); /* Enable underflow interrupt */
	NVIC_EnableIRQ(LETIMER0_IRQn); /* Enable LETIMER0 interrupt vector in NVIC*/
}


/***************************************************************************//**
 * @brief 		This function initializes the RSSI measurement. The Timer TM0 runs
 *   continuously and via the PRS system triggers an ADC measurement
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void letimerdrv_SetTime (uint16_t dwTime)
{
//uint32_t dwCount = (uint32_t) (dwTimeus / 1000000.0 * LETIMER_LFRCO_FREQ) + 1;

	//LETIMER_IntClear(LETIMER0, (LETIMER_IF_UF | LETIMER_IF_COMP0 | LETIMER_IF_COMP1 | LETIMER_IF_REP0| LETIMER_IF_REP1));
	letimerdrv_Clear (LETIMER0);
	LETIMER_CompareSet (LETIMER0,0,dwTime);
	//LETIMER_RepeatSet (LETIMER0,0,1); /* one shoot mode*/
}

#if USELOGICALTIMER == 0
/***************************************************************************//**
 * @brief		This function manages the LEtimer interruption
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ***********************
 ******************************************************************************/
void LETIMER0_IRQHandler(void)
{
uint32_t dwInterupt;

	dwInterupt = LETIMER_IntGet ( LETIMER0 );
	/* Clear flag interrupt */
	LETIMER_IntClear(LETIMER0, dwInterupt);

	if ( (dwInterupt & LETIMER_IF_COMP0) == LETIMER_IF_COMP0) /**< the time is elapsed*/
	{
		bLeTimerElapsed = true;
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
		srvActRec_IncreaseActivityTime ();
#endif
	}

}
#endif

/***************************************************************************//**
 * @brief 		This function enable or disable the LEtimer
 * @details
 * @param[in] 	enable true to enable counting, false to disable
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void letimerdrv_enable (const bool enable)
{
	LETIMER_Enable(LETIMER0, enable);
}



