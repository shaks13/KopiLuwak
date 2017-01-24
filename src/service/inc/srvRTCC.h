/*******************************************************************************
 * @file srvRTCC.h
 * @brief this file defines the command set for the RTC and calendar
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVRTCC_H_
#define SRVRTCC_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "rtcdriver.h"
#include "em_rtcc.h"






/*===========================================================================================================
						constant definition
===========================================================================================================*/

/*===========================================================================================================
						Enum definition
===========================================================================================================*/

/**
 *  @enum srvrtcc_CalendarEventType_enum
 *  @brief this enum contains the index of the different versions of the board
 *  @details should be match to serial_CalendarEventType_enum
 *
 */
typedef enum{
	SRVRTCC_CALENDAREVENT_STOPMEAS		= 0x00	,	/*!< the type  of the calendar event is start measurement  */
	SRVRTCC_CALENDAREVENT_STARTMEAS				,	/*!< the type  of the calendar event is stop measurement  */
	SRVRTCC_CALENDAREVENT_HOUR_TICK				,	/*!< the type  of the calendar event is HOUR TICK interrupt */
}srvrtcc_CalendarEventType_enum;

/*===========================================================================================================
						structure definition
===========================================================================================================*/


/*===========================================================================================================
						prototype
===========================================================================================================*/

void 	srvrtcc_GetTime 			( uint32_t *ui32time );
void 	srvrtcc_GetDate 			( uint32_t *ui32date );

void 	srvrtcc_IncreaseDayCounter 	( void );
void 	srvrtcc_GetDayCounter 		( uint16_t *ui16LifeTime );



#endif /* SRVRTCC_H_ */
