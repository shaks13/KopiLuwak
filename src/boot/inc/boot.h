/*******************************************************************************
 * @file boot.h
 * @brief the file contains the functions set to start up the device
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef BOOT_H
#define BOOT_H

#include "em_chip.h"
#include "drv_sleep.h"
#include "em_emu.h"
#include "srvRTCC.h"
#include "srvLIS2DH12.h"
/*===========================================================================================================
						constant
===========================================================================================================*/
#include "kernel_common.h"
#include "kernel_creation.h"
#include "rftask_creation.h"
#include "serial_creation.h"

#include "../../sensortask/inc/sensor_creation.h"

/*===========================================================================================================
						Public functions declarations
===========================================================================================================*/

uint8_t boot_CreateTasks (void );
void boot_InitializePeripheral (void);

#endif

