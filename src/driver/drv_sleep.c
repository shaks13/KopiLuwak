/***************************************************************************//**
 * @file sleep.c
 * @brief Energy Modes management driver.
 * @version 4.1.0
 * @details
 * This is a energy modes management module consisting of sleep.c and sleep.h
 * source files. The main purpose of the module is to ease energy
 * optimization with a simple API. The module allows the system to always sleep
 * in the lowest possible energy mode. Users could set up callbacks that are
 * being called before and after each and every sleep. A counting semaphore is
 * available for each low energy mode (EM1/EM2/EM3) to protect certain system
 * states from being corrupted. This semaphore has limit set to maximum 255 locks.
 *
 * The module provides the following public API to the users:
 * sleep_Init()
 * sleep_Sleep()
 * sleep_SleepBlockBegin()
 * sleep_SleepBlockEnd()
 * sleep_ForceSleepInEM4()
 *
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

/* Module header file(s). */
#include "drv_sleep.h"


/***************************************************************************//**
 * @addtogroup EM_Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup SLEEP
 * @brief Energy Modes management driver.
 * @details
 * This is a energy modes management module consisting of sleep.c and sleep.h
 * source files. The main purpose of the module is to ease energy
 * optimization with a simple API. The module allows the system to always sleep
 * in the lowest possible energy mode. Users could set up callbacks that are
 * being called before and after each and every sleep. A counting semaphore is
 * available for each low energy mode (EM1/EM2/EM3) to protect certain system
 * states from being corrupted. This semaphore has limit set to maximum 255 locks.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   MACROS   ************************************
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/* Number of low energy modes (EM1, EM2, EM3). Note: EM4 sleep/wakeup is handled
 * differently therefore it is not part of the list! */
#define sleep_NUMOF_LOW_ENERGY_MODES    3U



/*******************************************************************************
 ******************************   TYPEDEFS   ***********************************
 ******************************************************************************/


/*******************************************************************************
 ******************************   CONSTANTS   **********************************
 ******************************************************************************/


/*******************************************************************************
 *******************************   STATICS   ***********************************
 ******************************************************************************/

/* Callback functions to call before and after sleep. */
static sleep_CbFuncPtr_t sleepCallback  = NULL;
static sleep_CbFuncPtr_t wakeUpCallback = NULL;

/* Sleep block counter array representing the nested sleep blocks for the low
 * energy modes (EM1/EM2/EM3). Array index 0 corresponds to EM1, 1 to EM2 and 2
 * to EM3 accordingly.
 *
 * Note:
 * - EM4 sleep/wakeup is handled differently therefore it is not part of the
 *   list!
 * - Max. number of sleep block nesting is 255. */
static uint8_t sleepBlockCnt[sleep_NUMOF_LOW_ENERGY_MODES];

/*******************************************************************************
 ******************************   PROTOTYPES   *********************************
 ******************************************************************************/

static void sleep_EnterEMx(sleep_EnergyMode_t eMode);
static void sleep_ConfigureToEnteringSleepState( void );
static void sleep_ConfigureToExitingSleepState( void );
//static sleep_EnergyMode_t sleep_LowestEnergyModeGet(void);

/** @endcond */

/*******************************************************************************
 ***************************   GLOBAL FUNCTIONS   ******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Initialize the Sleep module.
 *
 * @details
 *   Use this function to initialize the Sleep module, should be called
 *   only once! Pointers to sleep and wake-up callback functions shall be
 *   provided when calling this function.
 *   If sleep_EM4_WAKEUP_CALLBACK_ENABLED is set to true, this function checks
 *   for the cause of the reset that implicitly called it and calls the wakeup
 *   callback if the reset was a wakeup from EM4 (does not work on Gecko MCU).
 *
 * @param[in] pSleepCb
 *   Pointer to the callback function that is being called before the device is
 *   going to sleep.
 *
 * @param[in] pWakeUpCb
 *   Pointer to the callback function that is being called after wake up.
 ******************************************************************************/
void sleep_Init(sleep_CbFuncPtr_t pSleepCb, sleep_CbFuncPtr_t pWakeUpCb)
{
  /* Initialize callback functions. */
  sleepCallback  = pSleepCb;
  wakeUpCallback = pWakeUpCb;

  /* Reset sleep block counters. Note: not using for() saves code! */
  sleepBlockCnt[0U] = 0U;
  sleepBlockCnt[1U] = 0U;
  sleepBlockCnt[2U] = 0U;

#if (sleep_EM4_WAKEUP_CALLBACK_ENABLED == true) && defined(RMU_RSTCAUSE_EM4WURST)
  /* Check if the Init() happened after an EM4 reset. */
  if (RMU_ResetCauseGet() & RMU_RSTCAUSE_EM4WURST)
  {
    /* Clear the cause of the reset. */
    RMU_ResetCauseClear();
    /* Call wakeup callback with EM4 parameter. */
    if (NULL != wakeUpCallback)
    {
      wakeUpCallback(sleepEM4);
    }
  }
#endif
}


/***************************************************************************//**
 * @brief
 *   Sets the system to sleep into the lowest possible energy mode.
 *
 * @details
 *   This function takes care of the system states protected by the sleep block
 *   provided by sleep_SleepBlockBegin() / sleep_SleepBlockEnd(). It allows
 *   the system to go into the lowest possible energy mode that the device can
 *   be set into at the time of the call of this function.
 *   This function will not go lower than EM3 because leaving EM4 requires
 *   resetting MCU. To enter into EM4 call sleep_ForceSleepInEM4().
 *
 * @return
 *   Energy Mode that was entered. Possible values:
 *   @li sleepEM0
 *   @li sleepEM1
 *   @li sleepEM2
 *   @li sleepEM3
 ******************************************************************************/
sleep_EnergyMode_t sleep_Sleep(void)
{
  sleep_EnergyMode_t allowedEM;

  INT_Disable();

  allowedEM = sleep_LowestEnergyModeGet();

  if ((allowedEM >= sleepEM1) && (allowedEM <= sleepEM4))
  {
    sleep_EnterEMx(allowedEM);
  }
  else
  {
    allowedEM = sleepEM0;
  }

  INT_Enable();

  return(allowedEM);
}


/***************************************************************************//**
 * @brief
 *   Force the device to go to EM4 without doing any checks.
 *
 * @details
 *   This function unblocks the low energy sleep block then goes to EM4.
 *
 * @note
 *   Regular RAM is not retained in EM4 and the wake up causes a reset.
 *   If the configuration option sleep_EM4_WAKEUP_CALLBACK_ENABLED is set to
 *   true, the sleep_Init() function checks for the reset cause and calls the
 *   EM4 wakeup callback.
 ******************************************************************************/
void sleep_ForceSleepInEM4(void)
{
#if (sleep_HW_LOW_ENERGY_BLOCK_ENABLED == true)
  /* Unblock the EM2/EM3/EM4 block in the EMU. */
  EMU_EM2UnBlock();
#endif

  /* Request entering to EM4. */
  sleep_EnterEMx(sleepEM4);
}

/***************************************************************************//**
 * @brief
 *   Begin sleep block in the requested energy mode.
 *
 * @details
 *   Blocking a critical system state from a certain energy mode makes sure that
 *   the system is not set to that energy mode while the block is not being
 *   released.
 *   Every sleep_SleepBlockBegin() increases the corresponding counter and
 *   every sleep_SleepBlockEnd() decreases it.
 *
 *   Example:\code
 *      sleep_SleepBlockBegin(sleepEM2);  // do not allow EM2 or higher
 *      // do some stuff that requires EM1 at least, like ADC sampling
 *      sleep_SleepBlockEnd(sleepEM2);    // remove restriction for EM2\endcode
 *
 * @note
 *   Be aware that there is limit of maximum blocks nesting to 255.
 *
 * @param[in] eMode
 *   Energy mode to begin to block. Possible values:
 *   @li sleepEM1 - Begin to block the system from being set to EM1 (and EM2..4).
 *   @li sleepEM2 - Begin to block the system from being set to EM2 (and EM3/EM4).
 *   @li sleepEM3 - Begin to block the system from being set to EM3 (and EM4).
 ******************************************************************************/
void sleep_SleepBlockBegin(sleep_EnergyMode_t eMode)
{
  EFM_ASSERT((eMode >= sleepEM1) && (eMode < sleepEM4));
  EFM_ASSERT((sleepBlockCnt[(uint8_t) eMode - 1U]) < 255U);

  /* Increase the sleep block counter of the selected energy mode. */
  sleepBlockCnt[(uint8_t) eMode - 1U]++;

#if (sleep_HW_LOW_ENERGY_BLOCK_ENABLED == true)
  /* Block EM2/EM3 sleep if the EM2 block begins. */
  if (eMode == sleepEM2)
  {
    EMU_EM2Block();
  }
#endif
}

/***************************************************************************//**
 * @brief
 *   End sleep block in the requested energy mode.
 *
 * @details
 *   Release restriction for entering certain energy mode. Every call of this
 *   function reduce blocking counter by 1. Once the counter for specific energy
 *   mode is 0 and all counters for lower energy modes are 0 as well, using
 *   particular energy mode is allowed.
 *   Every sleep_SleepBlockBegin() increases the corresponding counter and
 *   every sleep_SleepBlockEnd() decreases it.
 *
 *   Example:\code
 *      // at start all energy modes are allowed
 *      sleep_SleepBlockBegin(sleepEM2); // EM2, EM3, EM4 are blocked
 *      sleep_SleepBlockBegin(sleepEM1); // EM1, EM2, EM3, EM4 are blocked
 *      sleep_SleepBlockBegin(sleepEM1); // EM1, EM2, EM3, EM4 are blocked
 *      sleep_SleepBlockEnd(sleepEM2);   // still EM1, EM2, EM3, EM4 are blocked
 *      sleep_SleepBlockEnd(sleepEM1);   // still EM1, EM2, EM3, EM4 are blocked
 *      sleep_SleepBlockEnd(sleepEM1);   // all energy modes are allowed now\endcode
 *
 * @param[in] eMode
 *   Energy mode to end to block. Possible values:
 *   @li sleepEM1 - End to block the system from being set to EM1 (and EM2..4).
 *   @li sleepEM2 - End to block the system from being set to EM2 (and EM3/EM4).
 *   @li sleepEM3 - End to block the system from being set to EM3 (and EM4).
 ******************************************************************************/
void sleep_SleepBlockEnd(sleep_EnergyMode_t eMode)
{
  EFM_ASSERT((eMode >= sleepEM1) && (eMode < sleepEM4));

  /* Decrease the sleep block counter of the selected energy mode. */
  if (sleepBlockCnt[(uint8_t) eMode - 1U] > 0U)
  {
    sleepBlockCnt[(uint8_t) eMode - 1U]--;
  }

#if (sleep_HW_LOW_ENERGY_BLOCK_ENABLED == true)
  /* Check if the EM2/EM3 block should be unblocked in the EMU. */
  if (0U == sleepBlockCnt[(uint8_t) sleepEM2 - 1U])
  {
    EMU_EM2UnBlock();
  }
#endif
}

/***************************************************************************//**
 * @brief
 *   Gets the lowest energy mode that the system is allowed to be set to.
 *
 * @details
 *   This function uses the low energy mode block counters to determine the
 *   lowest possible that the system is allowed to be set to.
 *
 * @return
 *   Lowest energy mode that the system can be set to. Possible values:
 *   @li sleepEM0
 *   @li sleepEM1
 *   @li sleepEM2
 *   @li sleepEM3
 ******************************************************************************/
sleep_EnergyMode_t sleep_LowestEnergyModeGet(void)
{
  sleep_EnergyMode_t tmpLowestEM = sleepEM0;

  /* Check which is the lowest energy mode that the system can be set to. */
  if (0U == sleepBlockCnt[(uint8_t) sleepEM1 - 1U])
  {
    tmpLowestEM = sleepEM1;
    if (0U == sleepBlockCnt[(uint8_t) sleepEM2 - 1U])
    {
      tmpLowestEM = sleepEM2;
      if (0U == sleepBlockCnt[(uint8_t) sleepEM3 - 1U])
      {
        tmpLowestEM = sleepEM3;
      	if (0U == sleepBlockCnt[(uint8_t) sleepEM4 - 1U])
		{
		  tmpLowestEM = sleepEM4;
		}
      }
    }
  }

  /* Compare with the default lowest energy mode setting. */
  if (SLEEP_LOWEST_ENERGY_MODE_DEFAULT < tmpLowestEM)
  {
    tmpLowestEM = SLEEP_LOWEST_ENERGY_MODE_DEFAULT;
  }

  return tmpLowestEM;
}
/***************************************************************************//**
 * @brief this function is executed before going to an EM mode
 * @param[in] eMode sleep mode where the �c is going to enter.
 * @param[out] none
 * @details
 *
 * @return none
 ******************************************************************************/
void sleep_sleepEMxManagement(sleep_EnergyMode_t eMode)
{
	switch(eMode)
	{
		case 1:

		break;
		case 2:
		case 3:	/* EM1 or EM2 or EM3*/
			sleep_ConfigureToEnteringSleepState ();
		break;

		case 4:
			sleep_ConfigureToEnteringSleepState ();
			/* The pins used for EM4 wake-up must be configured as inputs with glitch filters using the GPIO_Px_MODEL/GPIO_Px_MODEH register.*/
			GPIO_PinModeSet(INTERFACE_MISO_PORT, INTERFACE_MISO_PIN, gpioModeInputPullFilter, 0);  	/* MISO */
			/*it is important to clear the wake-up logic by setting the GPIO_IFC bit, which clears the wake-up logic, including the GPIO_IF register*/
			GPIO_IntClear (GPIO_EXTILEVEL_EM4WU8);
		break;
		default:
			/* do nothing */
		break;
	}
}
/***************************************************************************//**
 * @brief this function is executed after leaving to an EM mode
 * @param[in] eMode sleep mode where the �c is waking up
 * @param[out] none
 * @details
 *
 * @return none
 ******************************************************************************/
void sleep_wakeUpEMxManagement(sleep_EnergyMode_t eMode)
{
	switch(eMode)
	{

		case 1:

		break;
		case 2:
		case 3:
			sleep_ConfigureToExitingSleepState ();
		break;

		case 4:

		break;
		default:
			/* do nothing */
		break;
	}
}

/***************************************************************************//**
 * @brief 		This function blocks the task in EM1 .
 * @detail 		because the taskdelay uses the systick as a clock and
 * the systick is clocked by the HF clock the uc should not do below EM1
 * @param[in] 	eCurrentSleepMode : the used current sleep mode this value
 * should be an member of the enum sleep_EnergyMode_t
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void sleep_DelayTask (uint32_t ui32Delay , sleep_EnergyMode_t eCurrentSleepMode)
{
	sleep_SleepBlockBegin(sleepEM1+1);	/* because the systick is clocked by the HF clock*/
	/* 5ms write cycle */
	vTaskDelay(ui32Delay);
	sleep_SleepBlockEnd(sleepEM1+1);	/* because the systick is clocked by the HF clock*/
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/***************************************************************************//**
 * @brief
 *   Call the callbacks and enter the requested energy mode.
 *
 * @details
 *   This function is not part of the API, therefore it shall not be called by
 *   the user directly as it doesn not have any checks if the system is ready
 *   for sleep!
 *
 * @note
 *   The EM4 wakeup callback is not being called from this function because
 *   waking up from EM4 causes a reset.
 *   If sleep_EM4_WAKEUP_CALLBACK_ENABLED is set to true, sleep_Init() function
 *   checks for the cause of the reset and calls the wakeup callback if the
 *   reset was a wakeup from EM4.
 ******************************************************************************/
static void sleep_EnterEMx(sleep_EnergyMode_t eMode)
{
  EFM_ASSERT((eMode > sleepEM0) && (eMode <= sleepEM4));

  /* Call sleepCallback() before going to sleep. */
  if (NULL != sleepCallback)
  {
    /* Call the callback before going to sleep. */
    sleepCallback(eMode);
  }

  /* Enter the requested energy mode. */
  switch (eMode)
  {
  case sleepEM1:
  {
    EMU_EnterEM1();
  } break;

  case sleepEM2:
  {
    EMU_EnterEM2(true);
  } break;

  case sleepEM3:
  {
    EMU_EnterEM3(true);
  } break;

  case sleepEM4:
  {
    EMU_EnterEM4();
  } break;

  default:
  {
    /* Don't do anything, stay in EM0. */
  } break;
  }

  /* Call the callback after waking up from sleep. */
  if (NULL != wakeUpCallback)
  {
    wakeUpCallback(eMode);
  }
}
/** @endcond */
/***************************************************************************//**
 * @brief
 *   this function configures the �c before entering in the low energy mode
 *
 * @details
 *
 * @note
 ******************************************************************************/
static void sleep_ConfigureToEnteringSleepState( void )
{
sleep_EnergyMode_t allowedEM;

	allowedEM = sleep_LowestEnergyModeGet();

	switch (allowedEM)
	{
	case sleepEM2 :
	case sleepEM3 :
		CMU_ClockEnable(cmuClock_USART1, false);
		CMU_ClockEnable(cmuClock_USART0, false);
		CMU_ClockEnable(cmuClock_DBG, false);
		CMU_ClockEnable(cmuClock_AUX, false);		/* 10.3.1.12 Debug Trace Clock
					Note:
					When using AUXHFRCO as the debug trace clock, it must be stopped before entering EM2 or EM3.*/
#if 0 /* should be activated for the release*/
		srvEM4325_sConfiguration.eMode = SRVEM4325_SIGNALING_COMM_BUFFER;
		interface_EnablePinInterrupt (INTERFACE_MISO_PORT,INTERFACE_MISO_PIN,INTERFACE_RISING_EDGE);
#endif
		interface_DisableSpiGPIOBeforeSleeping();
	break;

	case sleepEM4 :

	break;
	default:
	break;
	}

}
/***************************************************************************//**
 * @brief
 *   this function configures the �c after exiting from a low energy mode
 *
 * @details
 *
 * @note
 ******************************************************************************/
static void sleep_ConfigureToExitingSleepState( void )
{
sleep_EnergyMode_t allowedEM;

	allowedEM = sleep_LowestEnergyModeGet();
	if (allowedEM == sleepEM2)
	{
		//CMU_ClockEnable(cmuClock_ADC0, true);
		CMU_ClockEnable(cmuClock_USART0, true);
		CMU_ClockEnable(cmuClock_USART1, true);
		CMU_ClockEnable(cmuClock_DBG, true);
		CMU_ClockEnable(cmuClock_AUX, true);
		interface_EnableSpiGPIOAfterSleeping();
	}
	else if (allowedEM == sleepEM3)
	{
		//CMU_ClockEnable(cmuClock_ADC0, true);
		CMU_ClockEnable(cmuClock_USART0, true);
		CMU_ClockEnable(cmuClock_USART1, true);
		CMU_ClockEnable(cmuClock_DBG, true);
		CMU_ClockEnable(cmuClock_AUX, true);
		interface_EnableSpiGPIOAfterSleeping();
		interface_InitAuxPad ();
	}
}

/***************************************************************************//**
 * @brief		this function return true when the lowest sleep mode allowed
 * is the EM1.
 * @details		Some peripherals like the USART or Timer could only work in EM0
 * or EM1 state
 * @note 		this function is called in a IRQ context
 * @param[in] 	none
 * @param[out] 	none
 * @return 		true : the uc works in EM0 or EM1
 ******************************************************************************/
bool sleep_IsWakeup( void )
{
bool Iswakeup = false ;

	if ( sleep_LowestEnergyModeGet () <= sleepEM1)
	{
		Iswakeup = true;
	}
	else
	{
		/* do nothing*/
	}
	return Iswakeup;
}

/** @} (end addtogroup SLEEP */
/** @} (end addtogroup EM_Drivers) */
