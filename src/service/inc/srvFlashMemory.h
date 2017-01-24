/*******************************************************************************
 * @file srvFlashMemory.h
 * @brief Non-Volatile Memory Wear-Leveling service API
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SRVFLASHMEMORY_H_
#define SRVFLASHMEMORY_H_

#include "kernel_common.h"
#include "nvm.h"

/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/
uint8_t srvFlashMemory_Init 				(void);
uint8_t srvFlashMemory_ReadAllTempObjects 	(void);
uint8_t srvFlashMemory_WriteAllTempObjects 	(void);

#endif /* SRVFLASHMEMORY_H_ */
