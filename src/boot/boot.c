/*******************************************************************************
 * @file boot.c
 * @brief the file contains the functions set to start up the device
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "boot.h"

static void boot_InitializeClock(void);
static void boot_InitializeGpio(void);
static void boot_InitDCDC	( void );
//static void boot_InitEM4 	( void );
static void boot_SystemInit		( void );
//extern volatile cross_rfid_flags_bits cross_rfid_flags;

#define SELECTLFRC

/***************************************************************************//**
 * @brief
 *   Create the tasks of the OS and save theirs handle in the global structure
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
uint8_t boot_CreateTasks (void )
{
TaskHandle_t pCreatedTask ;
QueueHandle_t pCreatedQueue ;
/*uint32_t lsize ;*/
uint8_t status = CROSSRFID_SUCCESSCODE;


	/* create the kernel task and its queue and save theirs handles*/
	if (kernel_CreateTask ( &pCreatedTask, &pCreatedQueue ) == CROSSRFID_SUCCESSCODE)
	{
		sKernel_TaskPointer.pKernelTask = (pCreatedTask);
		sKernel_QueuePointer.pKernelQueue = (pCreatedQueue);
	}
	else
	{
		status = CROSSRFID_TASKNOTCREATED;
	}

	/* create the kernel task and its queue and save theirs handles*/
	if (rftask_CreateTask ( &pCreatedTask, &pCreatedQueue ) == CROSSRFID_SUCCESSCODE)
	{
		sKernel_TaskPointer.pRFtask = (pCreatedTask);
		sKernel_QueuePointer.pRFtaskQueue = (pCreatedQueue);
	}
	else
	{
		status = CROSSRFID_TASKNOTCREATED;
	}

	/* create the sensor task and its queue and save theirs handles*/
	if (sensor_CreateTask ( &pCreatedTask, &pCreatedQueue) == CROSSRFID_SUCCESSCODE)
	{
		sKernel_TaskPointer.pSensorTask = (pCreatedTask);
		sKernel_QueuePointer.pSensorQueue = (pCreatedQueue);
	}
	else
	{
		status = CROSSRFID_TASKNOTCREATED;
	}

	/* create the sensor task and its queue and save theirs handles*/
	if (serial_CreateTask ( &pCreatedTask, &pCreatedQueue) == CROSSRFID_SUCCESSCODE)
	{
		sKernel_TaskPointer.pSerialTask = (pCreatedTask);
		sKernel_QueuePointer.pSerialQueue = (pCreatedQueue);
	}
	else
	{
		status = CROSSRFID_TASKNOTCREATED;
	}

	/*lsize = */xPortGetFreeHeapSize();

	return status;
}


/***************************************************************************//**
 * @brief
 *   this function initializes the clocks
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void boot_InitializeClock(void)
{
	/* Low Energy Peripheral Interface Clock Enable (LE) for the watchdog */
	/* TODO: sets CMU_HFBUSCLKEN0->LE but the referenceManual notifies CMU_HFCORECLKEN0->LE */
	CMU_ClockEnable( cmuClock_CORELE, true );
	/* Using HFRCO at 14MHz as high frequency clock, HFCLK */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
	CMU_HFRCOFreqSet(cmuHFRCOFreq_13M0Hz);
	/* Enable clock for ADC0 */
	//CMU_ClockEnable(cmuClock_ADC0, true);
	CMU_OscillatorEnable(cmuOsc_LFXO, true, false);		/* Enable LFRCO oscillator */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO); 		/* Select LFRCO as clock source for LFACLK */
	/* Enable GPCRC clock */
	CMU_ClockEnable( cmuClock_GPCRC, true );
}


/***************************************************************************//**
 * @brief
 *   this function initializes the peripherals
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void boot_InitializeGpio(void)
{

	/* Enable GPIO clock */
	CMU_ClockEnable(cmuClock_GPIO, true);
	interface_InitializeUnusedGpios ();
	/* Clears the GPIO IRQ in the NVIC */
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	/* because these interupt handlers 'll call a FreeRtos function*/
	NVIC_SetPriority(GPIO_ODD_IRQn, (223 >> (8 - __NVIC_PRIO_BITS)) & 0x7); /* previous priority configKERNEL_INTERRUPT_PRIORITY*/
	NVIC_SetPriority(GPIO_EVEN_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);

}

/***************************************************************************//**
 * @brief
 *   this function initializes the on-chip DCDC
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none*********************************************************/
static void boot_InitDCDC ( void )
{
EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;

	EMU_DCDCInit (&dcdcInit);
}
#if 0
/***************************************************************************//**
 * @brief
 *   this function initializes the EM4 configuration
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void boot_InitEM4 ( void )
{
EMU_EM4Init_TypeDef em4Init   = EMU_EM4INIT_DEFAULT;

	EMU_EM4Init(&em4Init);
}
#endif
/***************************************************************************//**
 * @brief 		this function initializes the peripherals
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void boot_InitializePeripheral (void)
{
	boot_InitializeClock ();
	boot_InitializeGpio();

	sleep_Init(&sleep_sleepEMxManagement,
			   &sleep_wakeUpEMxManagement); 	/* Initialize SLEEP driver */
    sleep_SleepBlockBegin(sleepEM0+1);			/* do not let to sleep deeper than define */

#if (USEDCDC == 1)
	boot_InitDCDC ();	/* initialize the on-chip  DCDC*/
#endif


#if (__FPU_PRESENT==1)
	boot_SystemInit();
#endif

}

#if (__FPU_PRESENT==1)
/***************************************************************************//**
 * @brief  		this function initializes the peripherals
 *
 * @details 	http://community.silabs.com/t5/32-bit-MCU-Knowledge-Base/How-to-Enable-Hardware-Floating-Point-Math-for-Cortex-M4-with/ta-p/119846
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void boot_SystemInit (void)
{
    /* Set floating point coprosessor access mode. */
    SCB->CPACR |= ((3UL << 10*2) | /* set CP10 Full Access */
                                   (3UL << 11*2) ); /* set CP11 Full Access */
}
#endif

