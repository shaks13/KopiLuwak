
#ifndef SLEEP_H_
#define SLEEP_H_

#include "common_library.h"
/* Chip specific header file(s). */
#include "em_device.h"
#include "em_assert.h"
#include "em_int.h"
#include "em_rmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "srvEM4325.h"


#include "interface_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

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


/*******************************************************************************
 ****************************   CONFIGURATION   ********************************
 ******************************************************************************/

/** Enable/disable the HW block for protecting accidental setting of low energy
 *  modes (recommended to be set to true). */
#ifndef SLEEP_HW_LOW_ENERGY_BLOCK_ENABLED
#define SLEEP_HW_LOW_ENERGY_BLOCK_ENABLED    true
#endif

/** Enable/disable calling wakeup callback after EM4 reset. */
#ifndef SLEEP_EM4_WAKEUP_CALLBACK_ENABLED
#define SLEEP_EM4_WAKEUP_CALLBACK_ENABLED    true
#endif

/** Configure default lowest energy mode that the system can be set to.
 *  Possible values:
 *  @li sleepEM1 - EM1, the CPU core is turned off.
 *  @li sleepEM2 - EM2, like EM1 + all HF clocks are turned off, LF clocks are on.
 *  @li sleepEM3 - EM3, like EM2 + LF clocks are off, RAM retention, GPIO and ACMP
 *                   interrupt is on. */
#ifndef SLEEP_LOWEST_ENERGY_MODE_DEFAULT
#define SLEEP_LOWEST_ENERGY_MODE_DEFAULT    sleepEM3
#endif

/*******************************************************************************
 ******************************   TYPEDEFS   ***********************************
 ******************************************************************************/

/** Status value used for showing the Energy Mode the device is currently in. */
typedef enum
{
  /** Status value for EM0. */
  sleepEM0 = 0,

  /** Status value for EM1. */
  sleepEM1 = 1,

  /** Status value for EM2. */
  sleepEM2 = 2,

  /** Status value for EM3. */
  sleepEM3 = 3,

  /** Status value for EM4. */
  sleepEM4 = 4
} sleep_EnergyMode_t;

/** Callback function pointer type. */
typedef void (*sleep_CbFuncPtr_t)(sleep_EnergyMode_t);


/*******************************************************************************
 ******************************   PROTOTYPES   *********************************
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
void sleep_Init(sleep_CbFuncPtr_t pSleepCb, sleep_CbFuncPtr_t pWakeUpCb);

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
sleep_EnergyMode_t sleep_LowestEnergyModeGet(void);

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
sleep_EnergyMode_t sleep_Sleep(void);


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
void sleep_ForceSleepInEM4(void);


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
void sleep_SleepBlockBegin(sleep_EnergyMode_t eMode);


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
void 	sleep_SleepBlockEnd			( sleep_EnergyMode_t eMode);

/* Functions called by the sleepCallback and wakeUpCallback callback pointers */
void 	sleep_sleepEMxManagement	( sleep_EnergyMode_t eMode);
void 	sleep_wakeUpEMxManagement	( sleep_EnergyMode_t eMode);
void 	sleep_DelayTask 			( uint32_t ui32Delay , sleep_EnergyMode_t eCurrentSleepMode);
bool 	sleep_IsWakeup				( void );


/** @} (end addtogroup SLEEP) */
/** @} (end addtogroup EM_Drivers) */

#ifdef __cplusplus
}
#endif
#endif /* SLEEP_H_ */
