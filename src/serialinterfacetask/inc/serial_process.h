/*******************************************************************************
 * @file serial_process.h
 * @brief this function set manages the data exchange between the task and the
 * device.
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef SERIAL_PROCESS_H
#define SERIAL_PROCESS_H

#include "common_library.h"
#include "kernel_common.h"
#include "common_version.h"
#include "srvM2M.h"
#include "srvActivityRecorder.h"


/*===========================================================================================================
						enum definition
===========================================================================================================*/


/*===========================================================================================================
						enum definition
===========================================================================================================*/

/*===========================================================================================================
						structure definition
===========================================================================================================*/

/*===========================================================================================================
						Public variables definition
===========================================================================================================*/


/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
uint8_t serial_ProcessInit 						( Kernel_QueueItem_struct *pQueueItems );
void 	serial_processKernelM2Mack 				( void );
void 	serial_processEndOfWakeupMode 			( void );
uint8_t serial_ProcessM2MRxmessage 				( Kernel_QueueItem_struct *pQueueItems );
uint8_t serial_ProcessSensorResponse 			( Kernel_QueueItem_struct *pQueueItems );
uint8_t serial_ProcessMeasureReady 				( Kernel_QueueItem_struct *pQueueItems );
#endif

