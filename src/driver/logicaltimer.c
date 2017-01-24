/*******************************************************************************
 * @file logicaltimer.c
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "logicaltimer.h"

#if (USELOGICALTIMER==1)
static timer_logicalobject_struct asLogicalTimer [TIMER_LOGICALTIMER_NB]; /* contains the logical timer */
static timer_logicaltimer_struct	logtimer_config;
static uint8_t uNthLogicalTimerOngoing; 								  /* index of the array of the current logical timer on going*/

static void logtimer_ClearLETimer(LETIMER_TypeDef *letimer);
static bool logtimer_IsLETimerRunning(LETIMER_TypeDef *letimer);
static bool logtimer_IslogTimerReady( uint8_t *ui8NthTimer );
static void logtimer_SetCountLEtimer (const uint32_t wCount);

static void logtimer_lookingforMin (uint32_t *wMinCounter, uint8_t *uNthTimerWithMin );
static void logtimer_updateRemainingTime (const uint32_t wMinCounter, const uint8_t uNthTimerWithMin );

/***************************************************************************//**
 * @brief
 *  Typedef for the user supplied callback function which is called when
 *  a timer elapse.
 *
 * @note This callback is called from within an interrupt handler with
 *       interrupts disabled.
 *
 * @param[in] id
 *   The timer id.
 *
 * @param[in] user
 *   Extra parameter for user application.
 ******************************************************************************/
typedef void (*logtimer_Callback_t)( void);

/***************************************************************************//**
 * @brief   This function returs true when the LE timer is running
 * @details this function doesn't exist on the emlib.
 * @param[in] letimer Pointer to LETIMER peripheral register block.
 * @param[out] none
 * @return true when the LEtimer is running false otherwise
 ******************************************************************************/
static bool logtimer_IsLETimerRunning(LETIMER_TypeDef *letimer)
{
	return (letimer->STATUS & LETIMER_STATUS_RUNNING);
}

/***************************************************************************//**
 * @brief   This function returns true when a logical timer is ready to run
 * @param[in] none
 * @param[out] ui8NthTimer : id of the logical timer
 * @return true when the a logical timer is ready to run
 ******************************************************************************/
static bool logtimer_IslogTimerReady( uint8_t *ui8NthTimer )
{
	bool bstatus = false;
	uint8_t uNthTimer = 0;
	(*ui8NthTimer) = TIMER_LOGICALTIMER_NB;

	do
	{
		if ( ( false == asLogicalTimer[uNthTimer].bIsEnabled ) &&									/* on the one shoot one shoot timer on going*/
			 ( true ==  asLogicalTimer[uNthTimer].bIsReady  ))
		{
			(*ui8NthTimer) = uNthTimer;
			bstatus = true;
		}
	}while ( (uNthTimer++ < TIMER_LOGICALTIMER_NB) && ((*ui8NthTimer)== TIMER_LOGICALTIMER_NB));

	return bstatus;
}

/***************************************************************************//**
 * @brief   This function clears the LE timer
 * @details Make sure disabled first, before resetting other registers
 * @param[in] letimer Pointer to LETIMER peripheral register block.
 * @param[out] none
 * @return none
 ******************************************************************************/
static void logtimer_ClearLETimer(LETIMER_TypeDef *letimer)
{
  /* Make sure disabled first, before resetting other registers */
  letimer->CMD = LETIMER_CMD_CLEAR;

}
/***************************************************************************//**
 * @brief
 *   This function initializes the LEtimer to count one time
 *
 * @details
 *
 * @param[in] dwTimeus time to wait until the interuption in �s
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
static void logtimer_SetCountLEtimer (const uint32_t wCount)
{

	logtimer_ClearLETimer(LETIMER0);
	LETIMER_CompareSet (LETIMER0,0,wCount);
	LETIMER_RepeatSet (LETIMER0,0,1); /* one shoot mode*/

}




/***************************************************************************//**
 * @brief
 *   This function is looking for the minimum value of the reamining time between
 *   the logical timers
 *
 * @details
 *
 * @param[in] none
 * @param[out] wMinCounter : minimum of the time reamining
 * @param[out] uNthTimerWithMin : index of the logical timer which have the min value
 * @return none
 *
 ******************************************************************************/
static void logtimer_lookingforMin (uint32_t *wMinCounter, uint8_t *uNthTimerWithMin )
{

uint8_t uNthTimer = 0;

	(*uNthTimerWithMin) = TIMER_LOGICALTIMER_NB+1;
	(*wMinCounter) = 0xFFFFFFFF;
	for (uNthTimer = 0;uNthTimer < TIMER_LOGICALTIMER_NB ; uNthTimer ++ )
	{
																								/* looking for the min value of the timing*/
		if ( (asLogicalTimer[uNthTimer].bIsEnabled == true) &&									/* on the one shoot one shoot timer on going*/
			 ( asLogicalTimer[uNthTimer].ui32Tremaining < (*wMinCounter) ))
		{
			(*wMinCounter) = asLogicalTimer[uNthTimer].ui32Tremaining;
			(*uNthTimerWithMin) = uNthTimer;
		}
	}
}



/***************************************************************************//**
 * @brief
 *   This function updates the field remaining time of the logical timer with
 *   the min value of the reamining time
 *
 * @details
 *
 * @param[in] wMinCounter : minimum of the remaining time
 * @param[in] uNthTimerWithMin : index of the logical timer which have the min value
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
static void logtimer_updateRemainingTime (const uint32_t wMinCounter, const uint8_t uNthTimerWithMin )
{
uint8_t uNthTimer = 0;

	/* loop on all the logical timer */
	for (uNthTimer = 0;uNthTimer < TIMER_LOGICALTIMER_NB ; uNthTimer ++ )
	{
		if (( uNthTimer != uNthTimerWithMin) &&							/*when an another timer is ongoing*/
		(asLogicalTimer[uNthTimer].bIsEnabled  == true ))
		{
			asLogicalTimer[uNthTimer].ui32Tremaining -= wMinCounter;		/*decrease the remaining time*/
		}
	}
}


/***************************************************************************//**
 * @brief
 *   This function reset the logical timers
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void logtimer_ResetLogicalTimer ( void )
{
uint8_t nthtimer;

	logtimer_config.NbRunningTimer = 0;
	logtimer_config.bIsEnabled = false;
	logtimer_config.uNthLogicalTimerOngoing = 0;
	/* initiate the logical timer */
	for (nthtimer = 0 ; nthtimer<TIMER_LOGICALTIMER_NB  ;nthtimer++)
	{
		asLogicalTimer[nthtimer].bIsReady = false;
		asLogicalTimer[nthtimer].bIsEnabled = false;
		asLogicalTimer[nthtimer].ui32Tinit = 0;
		asLogicalTimer[nthtimer].ui32Tremaining = 0;
		asLogicalTimer[nthtimer].callback= NULL;
	}

}

/***************************************************************************//**
 * @brief
 *   This function initializes the LEtimer to count one time
 *
 * @details
 *
 * @param[in] dwTimeus time to wait until the interruption in �s
 * @param[in] eType type of the logical timer ( oneShoot or periodic )
 * @param[in] pFunctionCb function called when the logical timer elapsed
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void logtimer_CreateLogicalTimer (const uint8_t eTimerId , const uint32_t dwTimes, const uint8_t eType , void (*pFunctionCb) ( void ))
{
uint16_t hwCount = (uint16_t) (dwTimes  * LETIMER_LFRCO_FREQ / LETIMER_CLOCKDIVISION) ;

	/* initiate the logical timer */
	if (eTimerId < TIMER_LOGICALTIMER_NB)
	{
		asLogicalTimer[eTimerId].bIsReady = true;
		asLogicalTimer[eTimerId].bIsEnabled = false;
		asLogicalTimer[eTimerId].eTimerType = eType;
		asLogicalTimer[eTimerId].ui32Tinit = hwCount;
		asLogicalTimer[eTimerId].ui32Tremaining = hwCount;
		asLogicalTimer[eTimerId].callback = pFunctionCb;

	}
	else
	{
		/* outside of the array */
	}

}

/***************************************************************************//**
 * @brief
 *   This function enables the logical timer
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void logtimer_StartLogicalTimer (void)
{
	LETIMER_Enable(LETIMER0,true);
}

/***************************************************************************//**
 * @brief
 *   This function enable the logical timer
 *
 * @details
 *
 * @param[in] dwTimeus time to wait until the interruption in �s
 * @param[in] eType type of the logical timer ( oneShoot or periodic )
 * @param[in] pFunctionCb function called when the logical timer elapsed
 * @param[out] none
 * @return CROSSRFID_SUCCESSCODE : the timer is running
 * @return CROSSRFID_ERROR_UNSPECIFIED : the timer is not running
 ******************************************************************************/
uint8_t logtimer_EnableLogicalTimer ( const uint8_t eNthLogicalTimer )
{
uint32_t wCounterOngoing;
uint8_t ui8status = CROSSRFID_ERROR_UNSPECIFIED;

	/* initiate the logical timer */
	if ((asLogicalTimer[eNthLogicalTimer].bIsReady == true) && 								/* if the timer has been initialized*/
	    (asLogicalTimer[eNthLogicalTimer].bIsEnabled == false))								/* and not active*/
	{
		asLogicalTimer[eNthLogicalTimer].bIsEnabled = true;
		logtimer_config.NbRunningTimer++;

		if (logtimer_IsLETimerRunning (LETIMER0) == true) 									/* when the LEtimer is ongoing*/
		{
			LETIMER_Enable(LETIMER0, false);												/* stop the timer for avoiding its IRQ */
			wCounterOngoing = LETIMER_CounterGet (LETIMER0);
			if (wCounterOngoing < asLogicalTimer[eNthLogicalTimer].ui32Tremaining) 			/* when the next irq will occured before the time to wait*/
			{
				/* nothing to do. Wait the end of the timer*/
			}
			else /* the time of next irq is greater than the time to wait*/
			{
				logtimer_SetCountLEtimer (asLogicalTimer[eNthLogicalTimer].ui32Tremaining); 		/* reconfigure it */
				asLogicalTimer[uNthLogicalTimerOngoing].ui32Tremaining -= wCounterOngoing; 	/* update the time to wait for the current timer*/
				uNthLogicalTimerOngoing = eNthLogicalTimer	;								/* update the next logical timer taht'll elapsed */
			}

			LETIMER_Enable(LETIMER0, true); 												/* restart the timer */
		}
		else  														/* when the timer is not running*/
		{
			logtimer_SetCountLEtimer (asLogicalTimer[eNthLogicalTimer].ui32Tremaining); 			/* reconfigure it */
			uNthLogicalTimerOngoing = eNthLogicalTimer	;
			LETIMER_Enable(LETIMER0, true);
		}

		ui8status = CROSSRFID_SUCCESSCODE;
	}
	return ui8status;
}

/***************************************************************************//**
 * @brief
 *   This function disables the logical timer
 *
 * @details
 *
 * @param[in] eNthLogicalTimer logical timer to diable
 * @param[out] none
 * @return none
 *
 ******************************************************************************/
void logtimer_DisableLogicalTimer ( const uint8_t eNthLogicalTimer )
{
uint8_t ui8TimerId=0;

	if (eNthLogicalTimer < TIMER_LOGICALTIMER_NB)
	{
		logtimer_config.NbRunningTimer --;
		asLogicalTimer[eNthLogicalTimer].bIsEnabled = false;
		asLogicalTimer[eNthLogicalTimer].bIsReady = false;
		if ( false == logtimer_IslogTimerReady (&ui8TimerId))
		{
			LETIMER_Enable(LETIMER0, false);	/* no digital timer are ready to run*/
		}
		else
		{
			logtimer_EnableLogicalTimer ( ui8TimerId );
		}
	}
	else
	{
		/* outside of the array */
	}
}

/***************************************************************************//**
 * @brief
 *   This function manages the LEtimer interruption
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 *
 ******************************************************************************/

void LETIMER0_IRQHandler(void)
{
uint32_t dwInterupt;
uint32_t wMinCounter = 0xFFFFFFFF;
//uint8_t uNthTimer = 0;
uint8_t uNextTimer = TIMER_LOGICALTIMER_NB+1;

	dwInterupt =  LETIMER_IntGet(LETIMER0);
	/* Clear flag for TIMER0 overflow interrupt */

	LETIMER_IntClear(LETIMER0,dwInterupt);

	if ( (dwInterupt & _LETIMER_IF_UF_MASK) == LETIMER_IF_UF) /**< the time is elapsed*/
	{

		if (asLogicalTimer[uNthLogicalTimerOngoing].eTimerType == TIMER_LOGICALTIMERONESOOT)	/* only the one shoot timer should be disabled*/
		{
			asLogicalTimer[uNthLogicalTimerOngoing].bIsEnabled = false;
			logtimer_config.NbRunningTimer --;
		}
		else if (asLogicalTimer[uNthLogicalTimerOngoing].eTimerType == TIMER_LOGICALTIMERPERIODIC)	/* only the periodic timer should be reinitiate*/
		{
			asLogicalTimer[uNthLogicalTimerOngoing].ui32Tremaining = asLogicalTimer[uNthLogicalTimerOngoing].ui32Tinit;
		}

		if(NULL != asLogicalTimer [uNthLogicalTimerOngoing].callback)
		{
			asLogicalTimer [uNthLogicalTimerOngoing].callback (  ); /* call the callback function */
		}

		if (logtimer_config.NbRunningTimer>1 )
		{
			logtimer_lookingforMin(&wMinCounter,&uNextTimer); /* find the minimum value between all the timers*/
			logtimer_updateRemainingTime(wMinCounter,uNextTimer); /* update the remaining time of the others timers*/


			if (uNextTimer<TIMER_LOGICALTIMER_NB)		/* when an another round is required*/
			{
				uNthLogicalTimerOngoing = uNextTimer;
				logtimer_SetCountLEtimer (wMinCounter); 	/* configure LEtimer*/
				LETIMER_Enable(LETIMER0,true);
			}
			else
			{  /* nothing to do */		}
		}
	}
}

/***************************************************************************//**
 * @brief 		This function initializes the logical timer
 * @details 	the perdio of the RTC clock is 1 s
 * @param[in] 	sensorId : Id of the sensor (thermomether, accelerometer or magnetomter ...)
 * @param[in] 	ui32timerId : timerId allocated during the configuration  of the measurement
 * @param[in] 	ui8TimerType : periodic or one sheeo measurement
 * @param[in] 	ui16PeriodValue : period of the measurement. the unit is defined by ui16PeriodUnit
 * @param[in] 	ui16PeriodUnit : unit of the measuement period
 * @param[in]	pdata : is the pointer on the data measurement
 * @param[out]	none
 * @return 		CROSSRFID_ERROR_UNKNOWNPARAMETER : one parameter is erroneous
 * @return 		CROSSRFID_SUCCESSCODE :the function is successful. the timer is launched
 ******************************************************************************/
uint8_t logtimer_inittimer ( uint8_t sensorId , uint32_t ui32timerId, uint8_t ui8TimerType, uint16_t ui16PeriodValue , uint16_t ui16PeriodUnit  )
{
uint32_t timeout;
uint8_t status = CROSSRFID_SUCCESSCODE;
logtimer_Callback_t pCallback;


	switch (ui16PeriodUnit)
	{
		case KERNEL_SENSOR_ACQUNIT_US:
			if (ui16PeriodValue>10000000)			/* because the period of the RTC clock is 1s (ticks = 1s)*/
			{
				timeout = ui16PeriodValue /10000000;
			}
			else
			{
				timeout = 0; /* immediate call*/
			}
		break;
		case KERNEL_SENSOR_ACQUNIT_MS:
			if (ui16PeriodValue>1000)				/* because the period of the RTC clock is 1s (ticks = 1s) */
			{
				timeout = ui16PeriodValue /1000;
			}
			else
			{
				timeout = 0; /* immediate call*/
			}
		break;

		case KERNEL_SENSOR_ACQUNIT_S :
			timeout = ui16PeriodValue ;
		break;

		case KERNEL_SENSOR_ACQUNIT_MIN :
			timeout = ui16PeriodValue *60 ;
		break;

		case KERNEL_SENSOR_ACQUNIT_HOUR :
			timeout = ui16PeriodValue *60 *60 ;
		break;
		case KERNEL_SENSOR_ACQUNIT_DAY :
			timeout = ui16PeriodValue *60 *60 *24;
		break;
		default :
			status = CROSSRFID_ERROR_UNKNOWNPARAMETER;
		break;

	}

	/* select the callback*/
	switch (sensorId)
	{
		case KERNEL_SENSOR_ID_TEMPERATURE :

			pCallback = srvtemp_ThermoCB ;
		break;
		case KERNEL_SENSOR_ID_ACCELERO :

			pCallback = srvadxl363_AcceleroCB ;
		break;
		case KERNEL_SENSOR_ID_MAGNETO :

			pCallback =  srvlis3mdl_MagnetoCB ;
		break;
		default :
			status = CROSSRFID_ERROR_UNKNOWNPARAMETER;
		break;
	}

	if (CROSSRFID_SUCCESSCODE  == status )
	{
		 logtimer_CreateLogicalTimer (  ui32timerId ,
										 timeout,
										 ui8TimerType,
										 pCallback);


	}else {/*do noting*/}

	return status;

}







#endif
