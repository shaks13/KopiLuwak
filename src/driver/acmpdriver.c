/***************************************************************************//**
 * @file acmpdrv.c
 * @brief Analogcompare driver
 * @version 4.2.1
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
#include "acmpdrv.h"

static bool acmpdrv_IsAportConflict (ACMP_TypeDef *acmp);

/**************************************************************************//**
 * @brief this functions returns true is an Aport conflict is detected
 * @detail take care of the maximum input voltage
 * @param[in]  acmp Pointer to the ACMP peripheral register block.
 * @param[out] none
 * return CROSSRFID_SUCCESSCODE : there is no aPort conflict
 * return true: there is an aPort conflict else nothing
 *****************************************************************************/
static bool acmpdrv_IsAportConflict (ACMP_TypeDef *acmp)
{
	if (acmp->APORTCONFLICT == 0x0000)
	{
		return false;
	}
	else
	{
		return true;
	}
}
/**************************************************************************//**
 * @brief this functions initializes the voltage monitoring. the input is the
 * battery voltage and the reference is one bandgap of the chip
 * @detail take care of the maximum input voltage
 * @param[in] none
 * @param[out] none
 * return none
 *****************************************************************************/
void acmpdrv_Init (void)
{
const ACMP_Init_TypeDef acmp_init =	{
									false,                              /* Full bias current*/
									7,                                  /* Biasprog current configuration */
									false,                              /* Enable interrupt for falling edge */
									false,                              /* Enable interrupt for rising edge */
									acmpInputRangeFull,			/*ACMP_InputRange_TypeDef  Input can be from 0 to Vdd */
									acmpAccuracyLow,	/*ACMP_Accuracy_TypeDef*/
									acmpPowerSourceAvdd,
									acmpHysteresisLevel0,
									acmpHysteresisLevel0,
									acmpVLPInputVADIV,
									false,
									true};				 /* Enable ACMP */
ACMP_VAConfig_TypeDef vaconfig = { acmpVAInputAPORT1YCH11, /* PC11*/
									63,	/* VA divided = VA input * (div0 + 1) / 64*/
									63}; /* VA divided = VA input * (div1 + 1) / 64*/
ACMP_VBConfig_TypeDef vbconfig = { 	acmpVBInput2V5, /* Use 1.25 V as VB voltage input source */              \
									58,              /* No division of the VB source when ACMP output is 0 */ \
									58,              /* No division of the VB source when ACMP output is 1 */ \
								};

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(INTERFACE_ACMPIN_PORT,INTERFACE_ACMPIN_PIN, gpioModeInput, 1);		/* PC11 as input */
	GPIO_PinModeSet(INTERFACE_ACMPOUT_PORT,INTERFACE_ACMPOUT_PIN, gpioModePushPull, 1); /* PD15 as output */

	CMU_ClockEnable(cmuClock_ACMP0, true);
	/* Init and set ACMP channel */
	ACMP_Init(ACMP0, &acmp_init);
	ACMP_VASetup(ACMP0, &vaconfig);
	ACMP_VBSetup(ACMP0, &vbconfig);
	/* Configure 2.0V as upper boundary and channel0 on PC12 as input */
	ACMP_ChannelSet(ACMP0, acmpInputVBDIV, acmpInputVADIV);

	ACMP_GPIOSetup (ACMP0, _ACMP_ROUTELOC0_OUTLOC_LOC23, true, false); /* Gpio output loc 23 = PD15*/

	if (acmpdrv_IsAportConflict (ACMP0) == true)
	{
		CMU_ClockEnable(cmuClock_ACMP0, false);
	}

}


