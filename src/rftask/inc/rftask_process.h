/*******************************************************************************
 * @file rftask_process.h
 * @brief this function set process the received  command
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef RFTASK_PROCESS_H
#define RFTASK_PROCESS_H
#include "common_library.h"
#include "kernel_common.h"
#include "srvEM4325.h"
#include "srvRTCC.h"
#include "timerdrv.h"
/*===========================================================================================================
						const
===========================================================================================================*/
/* Size max of the free rtos message to serial (from rf task) */
#define RFTASK_NB_WORDS_MESSAGE_TOSERIAL (43)
/*===========================================================================================================
						enum definition
===========================================================================================================*/
/**
 *  @enum this enum contains the direction used by the serial interface task (slave or master)
 */
typedef enum{
	RFTASK_DIRECTION_SLAVE = 0,	/*!< the MCU is a slave  */
	RFTASK_DIRECTION_MASTER		/*!< the MCU is the master  */
}rftask_direction_enum;



/*===========================================================================================================
						structure definition
===========================================================================================================*/


/** @struct serial_configuration_struct
 *  @brief This structure contains the configuration of the serial interface
 *  @var serial_configuration_struct::serial_SlaveOrMaster
 *  Member 'serial_SlaveOrMaster' defines if the MCU acts as a master or a slave
 *  @var serial_configuration_struct::serial_Protocol
 *  Member 'serial_Protocol' defines the used protocol for the serial interface task
 */
typedef struct{
	rftask_direction_enum serial_SlaveOrMaster ;
}rftask_configuration_struct;

/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/
uint8_t 	rftask_ProcessInit 						( Kernel_QueueItem_struct *psQueueItems  );
void 		rftask_WriteSuccessInitFw				( void );
uint8_t  	rftask_ProcessReponseGetTemp 			( uint8_t const * pui8Data );
uint8_t		rftask_ProcessCommBufferCommand 		( Kernel_QueueItem_struct * psQueueItems );
uint8_t  	rftask_ProcessReponseDataIsAvailable	( const uint16_t  ui16Data );
uint8_t  	rftask_ProcessReponseGiveMeSamples		( uint8_t const * pui8Data );
uint8_t		rftask_ProcessResponseSysFile 			( uint8_t const * pui8Data );
uint8_t 	kernel_processSetDate 					( Kernel_QueueItem_struct *pQueueItems );
void	 	rftask_UpdateSysFileFromSerial 			( const srvEM4325_RegisterAddress_enum eSysFileAdrs, uint16_t const * pui16Data );
uint8_t 	rftask_ProcessSetAlarmFromSensor		( const Kernel_AlarmId_enum eAlarmId, Kernel_QueueItem_struct * psqueueItems );
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
void 		rftask_WriteSystemFileAccToQueueData 	( const uint8_t ui8SysFileAdrs,  const  uint8_t ui8NbByte  , uint8_t const * pui8Data );
#endif
void 		rftask_HookErrorCode 					( const CrossRfid_Status_enum ui8Error );
uint8_t		rftask_FinishInitFirmware 				( Kernel_QueueItem_struct * psqueueItems );
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
void 		rftask_WriteTempAverageInUserMem 		( const int16_t i16Average );
uint8_t 	rftask_ProcessReponseGiveMeLife 		( const Kernel_QueueItem_struct sQueue );
#endif

#endif

