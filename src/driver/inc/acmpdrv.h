/***************************************************************************//**
 * @file acmpdrv.h
 * @brief ACMPDRIVER API definition.
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

#ifndef __ACMP_DRIVER__
#define __ACMP_DRIVER__
#include <string.h>
#include "common_library.h"
#include "common_statuscode.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_acmp.h"
#include "em_gpio.h"
#include "em_common.h"
#include "em_int.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define INTERFACE_ACMPIN_PORT							gpioPortC
#define INTERFACE_ACMPIN_PIN							11
#define INTERFACE_ACMPOUT_PORT							gpioPortD
#define INTERFACE_ACMPOUT_PIN							15
/*===========================================================================================================
						Enum definition
===========================================================================================================*/



/*===========================================================================================================
						structure definition
===========================================================================================================*/


/*===========================================================================================================
						prototype
===========================================================================================================*/
void acmpdrv_Init (void);
#endif /* __SILICON_LABS_DMADRV_H__ */
