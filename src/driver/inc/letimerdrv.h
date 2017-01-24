/*******************************************************************************
 * @file letimerdrv.h
 * @brief
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef INLUDELETIMER_H
#define INLUDELETIMER_H

#include "em_cmu.h"
#include "em_letimer.h"
#include "srvActivityRecorder.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/

#define LETIMER_LFRCO_FREQ  							(32768UL) /* EFM32_LFRCO_FREQ*/
#if (APP_CHOSEN_FLAG == APP_NANOLIKE)
#define LETIMER_CLOCKDIVISION 							(cmuClkDiv_512)
#define LETIMER_COMPARESET	 							(10)
#else
#define LETIMER_CLOCKDIVISION 							(cmuClkDiv_32768)
#define LETIMER_COMPARESET	 							(100)
#endif

#define LETIMER_INIT_ACTIVITYCOUNTER                                                  \
{                                                                             \
  false,               /* Enable timer when init complete. */                  \
  false,              /* Stop counter during debug halt. */                   \
  true,              /* Do not load COMP0 into CNT on underflow. */          \
  false,              /* Do not load COMP1 into COMP0 when REP0 reaches 0. */ \
  0,                  /* Idle value 0 for output 0. */                        \
  0,                  /* Idle value 0 for output 1. */                        \
  letimerUFOANone,    /* No action on underflow on output 0. */               \
  letimerUFOANone,    /* No action on underflow on output 1. */               \
  letimerRepeatFree   /* Count until stopped by SW. */                        \
}

/*===========================================================================================================
						Public constant declaration
===========================================================================================================*/
extern volatile bool bLeTimerElapsed ;
/*===========================================================================================================
						Public functions declaration
===========================================================================================================*/

void 	letimerdrv_init			( void );
void 	letimerdrv_SetTime 		( uint16_t dwTime );
void 	letimerdrv_enable 		( const bool enable );

#endif
