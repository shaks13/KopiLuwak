/*******************************************************************************
 * @file logicaltimer.h
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#ifndef LOGICALTIMER_H
#define LOGICALTIMER_H

#if USELOGICALTIMER==1

#include <stdbool.h>
#include "letimerdrv.h"
#include "srvMeasTemperature.h"
#include "srvlis3mdl.h"
#include "srvadxl363.h"



/*===========================================================================================================
						constant definition
===========================================================================================================*/

#define TIMER_LOGICALTIMER_NB						3


/*===========================================================================================================
						enum definition
===========================================================================================================*/

/**
 *  @enum	timer_logicaltimertype_enum
 *  @brief 	this enum contains the type of the logical timer
 *
 */
typedef enum {
	TIMER_LOGICALTIMERONESOOT = 0,	/*!< Id of the first timer*/
	TIMER_LOGICALTIMERPERIODIC ,	/*!< Id of the 2nd timer*/
}timer_logicaltimertype_enum;

/**
 *  @enum timer_logicaltimerId_enum
 * @brief this enum contains the id of timers embedded
 */
typedef enum {
	TIMER_LOGICALTIMER0 = 0,	/*!< Id of the first timer*/
	TIMER_LOGICALTIMER1 ,		/*!< Id of the 2nd timer*/
	TIMER_LOGICALTIMER2 ,		/*!< Id of the 3th timer*/
}timer_logicaltimerId_enum;


/*===========================================================================================================
						structure definition
===========================================================================================================*/
typedef struct
{
	bool bIsReady;
	bool bIsEnabled;
	timer_logicaltimertype_enum eTimerType;
	uint32_t ui32Tinit;
	uint32_t ui32Tremaining;
	void (*callback) ( void );

}timer_logicalobject_struct;

typedef struct
{
	bool 		bIsEnabled;
	uint8_t 	NbRunningTimer ;
	uint8_t  	uNthLogicalTimerOngoing;
}timer_logicaltimer_struct;

/*===========================================================================================================
						prototype  definition
===========================================================================================================*/
uint8_t logtimer_inittimer 				( uint8_t sensorId , uint32_t ui32timerId, uint8_t ui8TimerType, uint16_t ui16PeriodValue , uint16_t ui16PeriodUnit);
void 	logtimer_ResetLogicalTimer 		( void );
void 	logtimer_CreateLogicalTimer 	(const uint8_t eTimerId , const uint32_t dwTimeus, const uint8_t eType , void (*pFunctionCb) ( void ) );
void 	logtimer_StartLogicalTimer 		( void );
uint8_t	logtimer_EnableLogicalTimer 	( const uint8_t eNthLogicalTimer );
void 	logtimer_DisableLogicalTimer 	( const uint8_t eNthLogicalTimer );



void 	logtimer_ThermoCB 			( int8_t *pi8data );
void 	logtimer_AcceleroCB 		( int8_t *pi8data );
void 	logtimer_MagnetoCB 			( int8_t *pi8data );
#endif
#endif
