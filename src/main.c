#include "main.h"

#include "em_dbg.h"

#if (RF_MODEM_TEST==2)
#include "protocol_epctag.h"
#include "protocol_EM4325.h"
#endif
#include "protocol_htu21d.h"

/**
 * @brief this functions toggles a pad in case of hardfault.
 * the reset should be done by the watchdog
 *
 */
void HardFault_Handler(void)
{

	__disable_irq();
	do{
		__NOP();
	}while (1);

	// NVIC_SystemReset(); /* commented to detect the hardfault */
}

#ifdef TEST_PRS_DMA_TO_MONITOR /* TODO removed, code to test cryo + prs + dma */
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_ldma.h"
#include "cryodrv.h"

#include <stdint.h>
#include <stdbool.h>

#include "prsdrv.h"
#include "dmadrv.h"


/* TODO removed, code to test cryo + prs + dma */
static uint32_t destBuffer[256];
static uint32_t srcBuffer = 0xFFFFFFFF;

/***************************************************************************//**
 * @brief This function is the entry point of the application
 *
 * @details
 *
 * @param[in] none
 * @param[out] none
  ******************************************************************************/
int main(void)
{
bool bStatusInit = CROSSRFID_TASKNOTCREATED;

  /* Chip errata */
  CHIP_Init();


  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_CORELE, true);
    sleep_SleepBlockBegin(sleepEM1);			/* do not let to sleep deeper than define */

  prsdrv_SetCryoTimerCh();
    bStatusInit = rftask_init();
  TEMPDRV_Init();

  /* init the DMA driver */
  DMADRV_Init();
  /* Set Up dma channel to be triggered by the PRS SREQ 0 */
  DMADRV_SetUpChannelForPRS(uiDmaChn0,&srcBuffer, &destBuffer[0]);
/*
    float32_t a = 0.0;float32_t b = 123.456;float32_t c = 654.321;
    a=b*c;*/


    bStatusInit = CROSSRFID_SUCCESSCODE;
{
	CHIP_Init();
	SystemCoreClockUpdate();					/* Ensure core frequency has been updated */
	boot_InitializePeripheral(); 				/* initialize the different peripheral*/

	if (boot_CreateTasks () == CROSSRFID_SUCCESSCODE) /*Create the tasks of the OS */
	{

		/* Initializes the System Timer and its interrupt, and starts the System Tick Timer */
		init_systick();

		/* Start the task initialization */
		kernel_PostInitMessage();

		/* Starts the application */
		vTaskStartScheduler ();
	}
	else{ /* should not happen */ }

  	return 0;
}
#else



/***************************************************************************//**
 * @brief This function is the entry point of the application
 * @details
 * @param[in] none
 * @param[out] 0
  ******************************************************************************/
int main(void)
{
	CHIP_Init();
	SystemCoreClockUpdate();					/* Ensure core frequency has been updated */
	boot_InitializePeripheral(); 				/* initialize the different peripheral*/


#if (USESWO == 1)
	drvdbg_setupSWOForPrint ();
	printf ("\n");
	printf ("=== SWO alive ^-^===\n");
#endif

#if ((APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER) || (APP_CHOSEN_FLAG == APP_GLUEPOT) || (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL))

	if (boot_CreateTasks () == CROSSRFID_SUCCESSCODE) /*Create the tasks of the OS */
	{
		/* Initializes the System Timer and its interrupt, and starts the System Tick Timer */
		init_systick();
		/* Start the task initialization */
		kernel_PostInitMessage();
		/* Starts the application */
		vTaskStartScheduler ();
	}
	else{ /* should not happen */ }
#else
	/* Nothing */
#endif

  	return 0;
}

#endif /*TEST_PRS_DMA_TO_MONITOR */
