/*******************************************************************************
 * @file kernel_process.h
 * @brief the file contain the command set for the kernel
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef KERNEL_PROCESS_H
#define KERNEL_PROCESS_H

#include "kernel_common.h"
#include "watchdog_process.h"
#include "srvCalendar.h"
#include "drv_sleep.h"
#include "cryodrv.h"
#include "drv_debug.h"
#include "srvFlashMemory.h"
#include "srvShelfLife.h"

/*===========================================================================================================
						enum definition
===========================================================================================================*/
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/**
 *  @enum this enum contains state of the life calculation.
 *
 */
typedef enum{
	KERNEL_LIFE_STOP = 0,	/*!< the life calculations is not in progress*/
	KERNEL_LIFE_START		/*!< the life calculations is in progress*/
}Kernel_ShelfLifeComputingState_enum;
#endif

/*===========================================================================================================
						struct definition
===========================================================================================================*/
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/**
 * @struct this structure contains the last shelf life computed and the state of the shelf life calculation:
 *  @var srvEM4325_GluePot_struct::ui32ShelfLife
 *  Member 'ui32ShelfLife' defines the last shelf life computed.
 *  @var srvEM4325_GluePot_struct::ui16State
 *  Member 'ui16State' defines the state of the shelf life calculation.
 */
typedef struct {
	double dShelfLife;
	uint16_t ui16State;
}Kernel_GluePot_struct;
#endif

/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/
uint8_t kernel_ProcessInit 				( Kernel_QueueItem_struct *pQueueItems  );
void 	kernel_PostInitMessage			( void );
uint8_t kernel_ProcessReset 			( Kernel_QueueItem_struct *psQueueItem );
void 	kernel_AllowSleepInEM1			( void );
void 	kernel_AllowSleepInDefault		( void );
void 	kernel_initSleepInDefault 		( void );
uint8_t kernel_processGetTime 			( Kernel_QueueItem_struct * psQueueItem );
uint8_t	kernel_processSetTime 			( Kernel_QueueItem_struct * psQueueItem );
uint8_t kernel_processGetDate 			( Kernel_QueueItem_struct * psQueueItem );
uint8_t	kernel_processSetDate 			( Kernel_QueueItem_struct * psQueueItem );
uint8_t kernel_processGetAge 			( Kernel_QueueItem_struct * psQueueItem );
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
void 	kernel_BackUpTempObjectsInFlash	( void );
uint8_t kernel_RestoreFlashToRam		( void );
uint8_t kernel_ProcessShelfLife 		( Kernel_QueueItem_struct * psQueueItems );
void 	kernel_EnablDisablLifeComputing ( const Kernel_QueueItem_struct sQueue );
uint8_t kernel_ProcessGiveMeLife 		( Kernel_QueueItem_struct *  psQueue );
#endif
uint8_t kernel_ProcessM2MRxmessage 		( Kernel_QueueItem_struct *pQueueItems );
uint8_t kernel_processEndOfWakeUpMode	( Kernel_QueueItem_struct *pQueueItems );
uint8_t kernel_processStartWakeUpMode	( Kernel_QueueItem_struct *pQueueItems );
#if (USESWO==1)
void kernel_processNotificationInText ( const uint8_t ui8recvsender, const Kernel_FreeRTOSMessageId_enum eFreeRTOSMessageId, uint16_t ui16NbBytes);
#endif
#endif

