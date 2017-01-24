/*******************************************************************************
 * @file timerdrv.c
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "timerdrv.h"

static void timer_WaitEndOfDelay( timer_timerId_enum timerId ) ; /* used for the test mode*/

volatile uint8_t timer1_bElapsed;
volatile bool 	timer0_bElapsed;


/***************************************************************************//**
 * @brief
 *   This function initializes the timer 0 to run one time and fires an IRQ when
 *   the time is elapsed
 * @details
 * @param[in] 	dwTimems : time in ms
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_inittimer0(uint32_t ui32dwTimeus)
{

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
	  .enable	  = true,  				/* don't start after initialization*/
	  .debugRun   = false,  		 		/* don't run during debug */
	  .prescale   = timerPrescale1024,  	/* scale factor  = 512  (divide by 512)*/
	  .clkSel	  = timerClkSelHFPerClk,  	/* peripheral high frequency clock */
	  .fallAction = timerInputActionNone, 	/* no action on the rising input*/
	  .riseAction = timerInputActionNone, 	/* no action on the falling input*/
	  .mode 	  = timerModeUp,		  	/* increase counter*/
	  .dmaClrAct  = false,				  	/* no DMA feature */
	  .quadModeX4 = false,				  	/* quadrature feature disable */
	  .oneShot	  = true,				  	/* counting up one time*/
	  .sync 	  = false,				  	/* No start/stop/reload by other timers */
	};

	/* Enable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);
	/* because this interupt handler 'll call a FreeRtos function*/
	NVIC_SetPriority(TIMER0_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);
	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);

	/* Configure TIMER */
	TIMER_Init(TIMER0, &timerInit);

	/* Set TIMER Top value */
	timer_SetTimetimer0( ui32dwTimeus );
}

/**************************************************************************//**
 * @brief  this function configures the timers 0 & 1. the timer 0 works as a
 * counter. its clock input is TIM0_CC1.
 * The timer 1 (HFRC0/presc) is the clock to decode
 * the symbols in the reader command (demodulated by the EM4325)
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void timer_initCounter (void)
{

	/* ---------------- Select TIMER1 parameters ---------------- */
	TIMER_Init_TypeDef timer1Init =
	{
	  .enable	  = false,  				/* don't start after initialization*/
	  .debugRun   = false,  		 		/* don't run during debug */
	  .prescale   = timerPrescale16,  		/* scale factor  = 16  (divide by 16)*/
	  .clkSel	  = timerClkSelHFPerClk,	/* peripheral high frequency clock */
	  .fallAction = timerInputActionNone, 	/* no action on the rising input*/
	  .riseAction = timerInputActionNone, 	/* no action on the falling input*/
	  .mode 	  = timerModeUp,		  	/* increase counter*/
	  .dmaClrAct  = false,				  	/* no DMA feature */
	  .quadModeX4 = false,				  	/* quadrature feature disable */
	  .oneShot	  = true,				  	/* counting up one time*/
	  .sync 	  = false,				  	/* No start/stop/reload by other timers */
	};


#if 0
	/* ----------------Select TIMER0 parameters ---------------- */
	TIMER_Init_TypeDef timer0Init =
	{
	  .enable	  = false,  				/* don't start after initialization*/
	  .debugRun   = false,  		 		/* don't run during debug */
	  .prescale   = timerPrescale1,  		/* scale factor  = 1  (divide by 1)*/
	  .clkSel	  = timerClkSelCC1,			/* peripheral high frequency clock */
	  .fallAction = timerInputActionNone, 	/* no action on the rising input*/
	  .riseAction = timerInputActionNone, 	/* no action on the falling input*/
	  .mode 	  = timerModeUp,		  	/* increase counter*/
	  .dmaClrAct  = false,				  	/* no DMA feature */
	  .quadModeX4 = false,				  	/* quadrature feature disable */
	  .oneShot	  = true,				  	/* counting up one time*/
	  .sync 	  = false,				  	/* No start/stop/reload by other timers */
	};

	/* Select CC channel parameters */
	TIMER_InitCC_TypeDef timer0CCInit =
	{
	.eventCtrl  = timerEventRising,
	.edge       = timerEdgeRising,
	.prsSel     = timerPRSSELCh0,
	.cufoa      = timerOutputActionNone,
	.cofoa      = timerOutputActionNone,
	.cmoa       = timerOutputActionNone,
	.mode       = timerCCModeCapture,
	.filter     = true,
	.prsInput   = false, /* select CCinput*/
	.coist      = false,
	.outInvert  = false,
	};
#else
	/* ----------------Select TIMER0 parameters ---------------- */
	TIMER_Init_TypeDef timer0Init =
	{
	  .enable	  = false,  				/* don't start after initialization*/
	  .debugRun   = false,  		 		/* don't run during debug */
	  .prescale   = timerPrescale16,  		/* scale factor  = 1  (divide by 1)*/
	  .clkSel	  = timerClkSelHFPerClk,			/* peripheral high frequency clock */
	  .fallAction = timerInputActionNone, 	/* no action on the rising input*/
	  .riseAction = timerInputActionNone, 	/* no action on the falling input*/
	  .mode 	  = timerModeUp,		  	/* increase counter*/
	  .dmaClrAct  = false,				  	/* no DMA feature */
	  .quadModeX4 = false,				  	/* quadrature feature disable */
	  .oneShot	  = true,				  	/* counting up one time*/
	  .sync 	  = false,				  	/* No start/stop/reload by other timers */
	};
#endif


	/* === configure TIMER 0 ===*/
	CMU_ClockEnable(cmuClock_TIMER0, true); /* Enable clock for TIMER0 module */
	/* because this interupt handler 'll call a FreeRtos function*/
	NVIC_SetPriority(TIMER0_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);
	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);

	//TIMER0->ROUTE = TIMER_ROUTE_CC1PEN;
	TIMER0->ROUTEPEN = TIMER_ROUTEPEN_CC1PEN;

	TIMER_IntEnable(TIMER0, TIMER_IF_OF);
	TIMER_Init(TIMER0, &timer0Init); /* Configure TIMER 0 */
#if 0
	TIMER_InitCC(TIMER0, 1, &timer0CCInit); /* Configure CC channel 1 */
#endif
	/* === configure TIMER 1 ===*/
	CMU_ClockEnable(cmuClock_TIMER1, true); /* Enable clock for TIMER1 module */
	TIMER_Init(TIMER1, &timer1Init); 		/* Configure TIMER 1 */
	/* because this interupt handler 'll call a FreeRtos function*/
	NVIC_SetPriority(TIMER1_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);
	/* Enable TIMER1 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER1_IRQn);
	TIMER_IntEnable(TIMER1, TIMER_IF_OF);
}

/***************************************************************************//**
 * @brief
 *   This function writes the TOp value of the counter. when the counter
 *   reaches this value the IRQ OF is triggered
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_SetCounterValue (uint32_t dwCount)
{
	TIMER_TopSet(TIMER0, dwCount); /* Set TIMER Top value */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF); 	/* Enable overflow interrupt */
	NVIC_EnableIRQ(TIMER0_IRQn); /* Enable TIMER0 interrupt vector in NVIC */
}


/***************************************************************************//**
 * @brief		this function resets the counter of the timer 0
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_ResetCounterTimer0 ( void )
{
	TIMER_CounterSet(TIMER0, 0);
}



/***************************************************************************//**
 * @brief
 *   This function initializes the RSSI measurement. The Timer TM0 runs
 *   continously and via the PRS system triggers an ADC measurement
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_inittimerToCountContinously (uint32_t dwTimeus)
{
	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
	  .enable	  = true,  					/* start after initialization*/
	  .debugRun   = false,  		 		/* don't run during debug */
	  .prescale   = timerPrescale512,  		/* scale factor  = 512  (divide by 512)*/
	  .clkSel	  = timerClkSelHFPerClk,  	/* peripheral high frequency clock */
	  .fallAction = timerInputActionNone, 	/* no action on the rising input*/
	  .riseAction = timerInputActionNone, 	/* no action on the falling input*/
	  .mode 	  = timerModeUp,		  	/* increase counter*/
	  .dmaClrAct  = false,				  	/* no DMA feature */
	  .quadModeX4 = false,				  	/* quadrature feature disable */
	  .oneShot	  = false,				  	/* counting up continously */
	  .sync 	  = false,				  	/* No start/stop/reload by other timers */
	};

	CMU_ClockEnable(cmuClock_TIMER0, true); /* Enable clock for TIMER0 module */
	TIMER_Init(TIMER0, &timerInit); /* Configure TIMER */
	timer_SetTimetimer0(dwTimeus);	/* Set TIMER Top value */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF); 	/* Enable overflow interrupt */
	NVIC_EnableIRQ(TIMER0_IRQn); /* Enable TIMER0 interrupt vector in NVIC */


}


/***************************************************************************//**
 * @brief		this function computes the number of clock of the Timer0 to
 * wait
 * @details
 * @param[in] 	dwTimeus : the time in us to wait
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_SetTimetimer0(uint32_t ui32dwTimeus)
{
uint32_t dwCount = (uint32_t) (ui32dwTimeus / 1024.0 / 1000000 * SystemHFClockGet ()); /*/1024 because timerPrescale512 */

	if ( (dwCount & 0xFFFF0000) != 0 ) /* when the counter cannot be writtent on 16 bits*/
	{
		dwCount = 0x0000FFFF; 			/* pick the max*/
	}
	else {/* do nothing */ }

	TIMER_TopSet(TIMER0, dwCount); /* Set TIMER Top value */
}

/***************************************************************************//**
 * @brief		This function manages the timer 0 interruption
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void TIMER0_IRQHandler(void)
{
uint32_t dwInterupt;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	dwInterupt = TIMER_IntGet ( TIMER0 );
	/* Clear flag interrupt */
	TIMER_IntClear(TIMER0, dwInterupt);
	if ( (dwInterupt & _TIMER_IF_OF_MASK) == TIMER_IF_OF) /**< the time is elapsed*/
	{
		timer0_bElapsed = true;
		if (xTimer0Semaphore != NULL)
		{
			xSemaphoreGiveFromISR(xTimer0Semaphore,&xHigherPriorityTaskWoken);
		} else { /*do nothing */}
	}

	if (pdTRUE == xHigherPriorityTaskWoken )
	{
		taskYIELD ();
	}

}

/***************************************************************************//**
 * @brief 		This function manages the timer 1 interruption
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{
uint32_t dwInterupt;
	dwInterupt = TIMER_IntGet ( TIMER1 );
	/* Clear flag interrupt */
	TIMER_IntClear(TIMER1, dwInterupt);

	if ( (dwInterupt & _TIMER_IF_OF_MASK) == TIMER_IF_OF) /**< the time is elapsed*/
	{
		timer1_bElapsed = 1;
	}
}


/***************************************************************************//**
 * @brief
 *   This function starts one timer
 * @details
 * @param[in] 	timer_timerId_enum : Id of the timer (0 for Timer0, 1  for timer1, ...)
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void timer_WaitEndOfDelay( timer_timerId_enum timerId )
{
	timer0_bElapsed = false;
	if (timerId == TIMER_TIMER0ID)
	{
		while (timer0_bElapsed == false);
	}

}

/***************************************************************************//**
 * @brief 		This function block the code execution during a specified time
 * @details  	this function shall be used only for the test function. In this case the
 *	mcu works in the state EM0 only.
 * @param[in] 	dwTimeus : time to wait in �s
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_delay (uint32_t dwTimeus)
{
	timer_inittimer0(dwTimeus);
	timer_start (TIMER_TIMER0ID);
	timer_WaitEndOfDelay (TIMER_TIMER0ID);
}


/***************************************************************************//**
 * @brief This function starts one timer
 * @details
 * @param[in] timer_timerId_enum : Id of the timer (0 for Timer0, 1  for timer1, ...)
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_start( timer_timerId_enum timerId )
{
	if (timerId == TIMER_TIMER0ID)
	{
		/* Set TIMER Top value */
		TIMER_CounterSet(TIMER0,0);
		//cross_rfid_flags.timer0_end=0;
		TIMER_Enable(TIMER0,true);
	}
	else if (timerId == TIMER_TIMER1ID)
	{
		TIMER_Enable(TIMER1,true);
	}
	//else if (timerId == TIMER_TIMER2ID)
	//{
		//TIMER_Enable(TIMER2,true);
	//}
	else
	{
		/* do noting */
	}
}
/***************************************************************************//**
 * @brief
 *   This function stops one timer
 * @details
 * @param[in] 	timer_timerId_enum : Id of the timer (0 for Timer0, 1  for timer1, ...)
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void timer_stop ( timer_timerId_enum timerId )
{

	if (timerId == TIMER_TIMER0ID)
	{
		TIMER_Enable(TIMER0,false);
	}
	else if (timerId == TIMER_TIMER1ID)
	{
		TIMER_Enable(TIMER1,false);
	}
	else
	{
		/* do noting */
	}

}

/***************************************************************************//**
 * @brief
 *   This function gets the counter value
 * @details
 * @param[in] timer_timerId_enum : Id of the timer (0 for Timer0, 1  for timer1, ...)
 * @param[out] wCounter : the counter value
 * @return 		none
 ******************************************************************************/
__INLINE void timer_GetCounterVal (timer_timerId_enum timerId, uint32_t *wCounter)
{
	switch (timerId)
	{
		case  TIMER_TIMER0ID:
			(*wCounter)=TIMER_CounterGet(TIMER0);
		break;
		default:
		break;
	}
}

/***************************************************************************//**
 * @brief
 *    This function sets the counter value
 * @details
 * @param[in] 	timer_timerId_enum : Id of the timer (0 for Timer0, 1  for timer1, ...)
 * @param[in] 	wCounter : the counter value
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
__INLINE void timer_SetCounterVal (timer_timerId_enum timerId, uint32_t wCounter)
{
	switch (timerId)
	{
		case  TIMER_TIMER0ID:
			TIMER_CounterSet(TIMER0,wCounter);
		break;
		default:
		break;
	}
}

/***************************************************************************//**
 * @brief 		Initializes the TIMER 1
 * @param[in] 	presc : HFRC0 prescaler
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void TIMER1_Init()
{
	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
	{
		.enable	    = false,  				/* don't start after initialization*/
		.debugRun   = false,  		 		/* don't run during debug */
		.prescale   = timerPrescale1024,  	/* scale factor  = 1024  (divide by 1024)*/
		.clkSel	    = timerClkSelHFPerClk,  /* peripheral high frequency clock */
		.fallAction = timerInputActionNone, /* no action on the rising input*/
		.riseAction = timerInputActionNone, /* no action on the falling input*/
		.mode 	    = timerModeUp,		  	/* increase counter*/
		.dmaClrAct  = false,			 	/* no DMA feature */
		.quadModeX4 = false,				/* quadrature feature disable */
		.oneShot	= true,				    /* counting up one time*/
		.sync 	    = false,			    /* No start/stop/reload by other timers */
	};

	CMU_ClockEnable(cmuClock_TIMER1, true); /* Enable clock for TIMER0 module */
	TIMER_Init(TIMER1, &timerInit); /* Configure TIMER */
}

/***************************************************************************//**
 * @brief		Deinits the TIMER 1
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void TIMER1_DeInit(void)
{
	/* Disables TIMER1 interrupts */
	TIMER_IntEnable(TIMER1,_TIMER_IEN_RESETVALUE);
	TIMER_IntClear(TIMER1,_TIMER_IFC_MASK);
	NVIC_DisableIRQ(TIMER1_IRQn);
	/* Stops the TIMER 1 */
	TIMER_Enable(TIMER1, false);
	/* Disable clock for TIMER1 module */
	CMU_ClockEnable(cmuClock_TIMER1, false);
}

/***************************************************************************//**
 * @brief
 *    This synchronous function waits for a delay in ms
 *    HFRC0 = 13 MHz, presc = 1024
 *    (tempo min = 0.073 ms ; tempo max = 4793.41 ms)
 *
 * @param[in] 	ui8DelayMs : The delay in ms
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void TIMER1_DelayAt13Mhz(uint16_t ui16DelayMs)
{
	/* HFRC0 = 13 MHz, presc = 1024 */
	uint16_t ui16Cycles = (13000 >> 10) * ui16DelayMs;

	/* adjustment for this function */
	ui16Cycles += 3;

	/* Reset Timer */
	TIMER_CounterSet(TIMER1,(uint32_t)0);

	/* Start TIMER0 */
	TIMER_Enable(TIMER1,true);

	/* Wait until counter value is over top */
	while(TIMER_CounterGet(TIMER1) < ui16Cycles);

	/* Stop TIMER0 */
	TIMER_Enable(TIMER1,false);
}


