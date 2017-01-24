/**************************************************************************//**
 * @file
 * @brief helper functions for configuring SWO
 * @version 4.4.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#include "drv_debug.h"
/***************************************************************************//**
 * @brief 		This function allows to use the function printf to send some
 * data on SWO
 * @details 	http://blog.atollic.com/cortex-m-debugging-printf-redirection-to-a-debugger-console-using-swv/itm-part-1
 * 				http://community.silabs.com/t5/Simplicity-Studio-and-Software/how-to-enable-printf-output/td-p/133981
 *
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
******************************************************************************/
int RETARGET_WriteChar(char c)
{
  return ITM_SendChar(c);
}
/***************************************************************************//**
 * @brief 		This function allows to use the function printf to send some
 * data on SWO
 * @details 	http://blog.atollic.com/cortex-m-debugging-printf-redirection-to-a-debugger-console-using-swv/itm-part-1
 * 				http://community.silabs.com/t5/Simplicity-Studio-and-Software/how-to-enable-printf-output/td-p/133981
 *
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
******************************************************************************/
int RETARGET_ReadChar(void)
{
  return 0;
}

/***************************************************************************//**
 * @brief 		This function allows to use the function printf to send some
 * data on SWO
 * @details 	http://blog.atollic.com/cortex-m-debugging-printf-redirection-to-a-debugger-console-using-swv/itm-part-1
 * 				http://community.silabs.com/t5/Simplicity-Studio-and-Software/how-to-enable-printf-output/td-p/133981
 *
 * @param[in] 	file : not used
 * @param[in] 	*prt : pointer on the data to send
 * @param[in] 	len : number of characters to send
 * @param[out] 	none
 * @return 		len
******************************************************************************/
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

/***************************************************************************//**
 * @brief 		This function setup the SWO pad to display some debug information
 * on the console
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
******************************************************************************/
void drvdbg_setupSWOForPrint(void)
{
	/* Enable GPIO clock. */
	CMU_ClockEnable(cmuClock_GPIO, true);

	/* Enable Serial wire output pin */
	GPIO->ROUTEPEN |= GPIO_ROUTEPEN_SWVPEN;

	/* Set location 0 */
	GPIO->ROUTELOC0 = GPIO_ROUTELOC0_SWVLOC_LOC0;

	/* Enable output on pin - GPIO Port F, Pin 2 */
	GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
	GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

	/* Enable debug clock AUXHFRCO */
	CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
	CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

	/* Wait until clock is ready */
	while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

	/* Enable trace in core debug */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	ITM->LAR  = 0xC5ACCE55;
	ITM->TER  = 0x0;
	ITM->TCR  = 0x0;
	TPI->SPPR = 2;
	TPI->ACPR = 0x15;	// AUXHFRCC0 = 19 MHz / 22 = 866 kHz
	ITM->TPR  = 0x0;
	DWT->CTRL = 0x400003FE;
	ITM->TCR  = 0x0001000D;
	TPI->FFCR = 0x00000100;
	ITM->TER  = 0x1;
}
