/***************************************************************************//**
 * @file cryodrv.c
 * @brief CRYO TIMER API implementation.
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "cryodrv.h"

/*===========================================================================================================
						Private variables declarations
===========================================================================================================*/
static uint64_t cryodrv_tui64NbCycles[CRYODRV_NB_ENUM_PERIOD] =
{
	1 << cryotimerPeriod_1,
	1 << cryotimerPeriod_2,
	1 << cryotimerPeriod_4,
	1 << cryotimerPeriod_8,
	1 << cryotimerPeriod_16,
	1 << cryotimerPeriod_32,
	1 << cryotimerPeriod_64,
	1 << cryotimerPeriod_128,
	1 << cryotimerPeriod_256,
	1 << cryotimerPeriod_512,
	1 << cryotimerPeriod_1k,
	1 << cryotimerPeriod_2k,
	1 << cryotimerPeriod_4k,
	1 << cryotimerPeriod_8k,
	1 << cryotimerPeriod_16k,
	1 << cryotimerPeriod_32k,
	1 << cryotimerPeriod_64k,
	1 << cryotimerPeriod_128k,
	1 << cryotimerPeriod_256k,
	1 << cryotimerPeriod_512k,
	1 << cryotimerPeriod_1m,
	1 << cryotimerPeriod_2m,
	1 << cryotimerPeriod_4m,
	1 << cryotimerPeriod_8m,
	1 << cryotimerPeriod_16m,
	1 << cryotimerPeriod_32m,
	1 << cryotimerPeriod_64m,
	1 << cryotimerPeriod_128m,
	1 << cryotimerPeriod_256m,
	1 << cryotimerPeriod_512m,
	1 << cryotimerPeriod_1024m,
	1 << cryotimerPeriod_2048m,
	1 << cryotimerPeriod_4096m
};

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   Initializes the CRYOTIMER driver to wake up in EM4.
 *
 * @param[in] none
 *
 * @return none
 ******************************************************************************/
void cryodrv_Init(void)
{
	CRYOTIMER_Init_TypeDef init = CRYOTIMER_INIT_EM4;

	/* Enabling CRYOTIMER cmuClock */
	CMU_ClockEnable(cmuClock_CRYOTIMER, true);

	NVIC_ClearPendingIRQ(CRYOTIMER_IRQn);
	NVIC_SetPriority(CRYOTIMER_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> ((8 - __NVIC_PRIO_BITS)) & 0x7)); /* TODO: set the priority */
	NVIC_EnableIRQ(CRYOTIMER_IRQn);

	/* Enable "Period" interruption */
	CRYOTIMER_IntClear(CRYOTIMER_IEN_PERIOD);
	CRYOTIMER_IntEnable(CRYOTIMER_IEN_PERIOD);

	/* init PERIODSEL, CTRL and EM4WUEN registers */
	CRYOTIMER_Init(&init);
}

/***************************************************************************//**
 * @brief
 *   Sets the cryotimer prescaler
 *
 * @param[in] prescaler: the prescaler value
 *
 * @return none
 ******************************************************************************/
void cryodrv_SetPrescaler(const CRYOTIMER_Presc_TypeDef prescaler)
{
	/* Updates the prescaler */
	CRYOTIMER->CTRL = (CRYOTIMER->CTRL & ~_CRYOTIMER_CTRL_PRESC_MASK)
	                  | ((uint32_t)prescaler << _CRYOTIMER_CTRL_PRESC_SHIFT);
}

/***************************************************************************//**
 * @brief
 *   As the period of the cryotimer is configured by setting the number of
 *   Pre-scaled clock cycles, this function finds this number and the prescaler
 *   to configure according to the chosen perdiod.
 *
 * @param[in] ui16Period: the period value
 * @param[in] ui16unit: the period unit
 * @param[out] pNbCycle: the number of Pre-scaled clock
 * @param[out] ui16Period: the prescaler
 *
 * @return status: true if successful process else false
 ******************************************************************************/
bool cryodrv_ConvPeriodToNbCycleAndPresc(const uint16_t ui16Period, const uint16_t ui16unit, CRYOTIMER_Period_TypeDef * pNbCycle, CRYOTIMER_Presc_TypeDef * pPrescaler)
{
	bool bStatus = true;
	unsigned int nbCycleComputed, idxNbCycleTypdef, nbCycleComputedLimit;

	if(ui16Period > 0)
	{
		switch (ui16unit)
		{
			case KERNEL_SENSOR_ACQUNIT_US: /* (cryotimerPeriod_1)- (cryotimerPeriod_2k) */
				/* MIN : 30.5 �s */
				*pPrescaler = cryotimerPresc_1;
				nbCycleComputed = ui16Period * 32768 / 1000000;
				break;

			case KERNEL_SENSOR_ACQUNIT_MS: /* (cryotimerPeriod_32) - (cryotimerPeriod_2m) */
				*pPrescaler = cryotimerPresc_1;
				nbCycleComputed = ui16Period * 32768 / 1000;
				break;

			case KERNEL_SENSOR_ACQUNIT_S: /* (cryotimerPeriod_32k) - (cryotimerPeriod_2048m) */
				*pPrescaler = cryotimerPresc_1;
				nbCycleComputed = ui16Period * 32768;
				break;

			case KERNEL_SENSOR_ACQUNIT_MIN: /* (cryotimerPeriod_32k) - (cryotimerPeriod_2048m) */
				*pPrescaler = cryotimerPresc_64;
				nbCycleComputed = ui16Period * 60 * 32768 / 64;
				break;

			case KERNEL_SENSOR_ACQUNIT_HOUR: /* (cryotimerPeriod_1m) - (cryotimerPeriod_4096m) */
				/* avoids the overflow of nbCycleComputed */
				/* 0xFFFFFFFF = 4294967295 (32bit)*/
				/* Limit nbCycleComputed = (0xFFFFFFFF * 128) / (3600 * 32768) */
				nbCycleComputedLimit = (unsigned int)(0xFFFFFFFF / 921600);
				if(ui16Period < nbCycleComputedLimit)
				{
					*pPrescaler = cryotimerPresc_128;
					nbCycleComputed = ui16Period * 3600 * 32768 / 128;
				}
				else
				{
					bStatus = false;
				}
				break;

			case KERNEL_SENSOR_ACQUNIT_DAY:
				/* avoids the overflow of nbCycleComputed */
				/* 0xFFFFFFFF = 4294967295 (32bit)*/
				/* Limit nbCycleComputed = (0xFFFFFFFF * 128) / (3600 * 24 * 32768) */
				nbCycleComputedLimit = (unsigned int)(0xFFFFFFFF / 22118400);
				if(ui16Period < nbCycleComputedLimit)
				{
					*pPrescaler = cryotimerPresc_128;
					nbCycleComputed = ui16Period * 3600 * 24 * 32768 / 128;
				}
				else
				{
					bStatus = false;
				}
				*pPrescaler = cryotimerPresc_128;
				break;

			default:
				bStatus = false;
				break;
		}

		/* Wrong period & wrong unit */
		if(false ==	bStatus)
		{
			*pPrescaler = cryotimerPresc_1;
			*pNbCycle = 0;
		}
		else
		{
			/* Seeking the number of cycles "CRYOTIMER_Period_TypeDef" according to the computed number of cycles */
			/* the loop index is the enumerator */
			for(idxNbCycleTypdef = 0 ; idxNbCycleTypdef < CRYODRV_NB_ENUM_PERIOD; idxNbCycleTypdef++)
			{
				/* nbcy < nbcy in the CRYOTIMER_Period_TypeDef */
				if (nbCycleComputed < cryodrv_tui64NbCycles[idxNbCycleTypdef])
				{
					/* perform the test to find the higher or lower interval */
					if( idxNbCycleTypdef >= 2 )
					{
						/* x > (nbCy(n-1) + nbCy(n-1)/2) */
						if(nbCycleComputed > (cryodrv_tui64NbCycles[idxNbCycleTypdef-1] + cryodrv_tui64NbCycles[idxNbCycleTypdef-2]))
						{
							*pNbCycle = idxNbCycleTypdef;
						}
						else
						{
							*pNbCycle = idxNbCycleTypdef-1;
						}
					}
					/* takes by default the lower interval */
					else if( idxNbCycleTypdef >= 1 )
					{
						*pNbCycle = idxNbCycleTypdef-1;
					}
					else
					{
						*pNbCycle = idxNbCycleTypdef;
					}
					break;
				}
			}
		}
	}
	else
	{
		/* wrong period value */
		*pNbCycle = 0;
		*pPrescaler = cryotimerPresc_1;
		bStatus = false;
	}

	return bStatus;
}

/**************************************************************************//**
 * @brief 		CRYOTIMER IRQ Handler
 * @param[in]  	none
 * @param[out]  none
 * @return 		none
 *****************************************************************************/
void CRYOTIMER_IRQHandler ( void )
{
	uint32_t	ui32Flags;
#ifdef CRYODRV_USING_FREE_RTOS_TO_MONITOR
	Kernel_QueueItem_struct pCommBufferQueueItem = {KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
													KERNEL_MESSAGEID_SAMPLETOGET, 	/* sample to get */
													0,NULL};
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* We have not waken a task at the start of the ISR.*/
#endif
	ui32Flags = CRYOTIMER_IntGet();
	CRYOTIMER_IntClear (ui32Flags);
#ifdef CRYODRV_USING_FREE_RTOS_TO_MONITOR
	if (CRYOTIMER_IF_PERIOD == ui32Flags)
	{
		/* Sends a message to the Serial task */
		xQueueSendFromISR (sKernel_QueuePointer.pSensorQueue,&pCommBufferQueueItem, &xHigherPriorityTaskWoken);
	}
	else { /* do nothing*/}
#endif

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD ();
	}
}
