/*******************************************************************************
 * @file timerdrv.h
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef INLUDETIMER_H
#define INLUDETIMER_H

#include "em_cmu.h"
#include "em_timer.h"
#include "kernel_common.h"


/*===========================================================================================================
						constant definition
===========================================================================================================*/

/* 13671 Hz -> 14Mhz (clock frequency) / 1024 (prescaler)
  Setting TOP to 27342 results in an overflow each 2 seconds */
#define TIMER_TOP_TIMER0 							(700)
#define TIMER_CPU_CLOCK_HZ                        	(( unsigned long ) 14000000)
//#define TIMER_LFRCO_FREQ  							(32768UL) /* EFM32_LFRCO_FREQ*/
#define TIMER_WAIT50US	                        	(( unsigned long ) 50)
#define TIMER_WAIT100US	                        	(( unsigned long ) 100)
#define TIMER_WAIT200US	                        	(( unsigned long ) 200)
#define TIMER_WAIT300US	                        	(( unsigned long ) 300)
#define TIMER_WAIT350US	                        	(( unsigned long ) 350)
#define TIMER_WAIT500US	                        	(( unsigned long ) 500)
#define TIMER_WAIT1MS	                        	(( unsigned long ) 1000)
#define TIMER_WAIT5MS	                        	(( unsigned long ) 5000)
#define TIMER_WAIT10MS	                        	(( unsigned long ) 10000)
#define TIMER_WAIT100MS	                        	(( unsigned long ) 100000)
#define TIMER_WAIT1S	                        	(( unsigned long ) 1000000) /* the 1s value could be coded on 16 bits   */

/**
 * @enum 	timer_logicaltimerId_enum
 * @brief
 */
typedef enum {
	TIMER_TIMER0ID = 0,
	TIMER_TIMER1ID ,
	TIMER_TIMER2ID ,
	TIMER_TIMERLEID
}timer_timerId_enum;

extern volatile uint8_t timer1_bElapsed;
extern volatile bool 	timer0_bElapsed;


/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/

void 	timer_inittimer0 			( uint32_t dwTimeus);
void 	timer_initCounter 			( void );
void 	timer_SetCounterValue 		( uint32_t dwCount);
void 	timer_ResetCounterTimer0 	( void );
void 	timer_inittimerToCountContinously (uint32_t dwTimeus);
void 	timer_SetTimetimer0			( uint32_t dwTimeus );
void 	timer_start					( timer_timerId_enum timerId );
void 	timer_stop 					( timer_timerId_enum timerId );
void 	TIMER0_IRQHandler			( void );
void 	timer_delay 				( uint32_t dwTimeus);
void 	timer_SetCounterVal 		( timer_timerId_enum timerId, uint32_t wCounter);
void 	timer_GetCounterVal			( timer_timerId_enum timerId, uint32_t *wCounter);
void 	TIMER1_Init					( void );
void 	TIMER1_DeInit				( void );
void 	TIMER1_DelayAt13Mhz			( uint16_t ui16DelayMs );

#endif
