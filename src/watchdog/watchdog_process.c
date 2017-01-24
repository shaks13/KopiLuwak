/*******************************************************************************
 * @file watchdog_process.c
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "watchdog_process.h"

void watchdog_InitWatchdog (void)
{
WDOG_Init_TypeDef sWatchdog = {	false, 				/** Enable watch dog when init completed. */
								false,  			/** Counter shalln't keep running during debug halt. */
								false, 				/** Counter shall keep running when in EM2. */
								false, 				/** Counter shall keep running when in EM3. */
								true, 				/** Block EMU from entering EM4. */
								true, 				/** Block SW from disabling LFRCO/LFXO oscillators. */
								true, 				/** Block SW from modifying the configuration (a reset is needed to reconfigure). */
								wdogClkSelULFRCO, 	/**< ultra low frequency RC oscillator */
								wdogPeriod_257 		/**< 257 * clock source = 257 ms */
								};
	WDOG_Init(&sWatchdog);
}

/***************************************************************************//**
 * @brief
 *   reset the watch dog counter
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
 * @return none
 ******************************************************************************/
void watchdog_TickleWatchdog (void)
{
	WDOG_Feed();
	//TOGGLE_PE12(); /* debug purpose*/
}
