/*******************************************************************************
 * @file srvMeasTemperature.h
 * @brief this file defines the commans set for the temperature mesurement
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVCALENDAR_H_
#define SRVCALENDAR_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "srvRTCC.h"



/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define SRVCALENDAR_NBYEAR_SHIFT							(12)
#define SRVCALENDAR_NBYEAR_MASK								(0xF000)
#define SRVCALENDAR_NBMONTHTENS_SHIFT						(10)
#define SRVCALENDAR_NBMONTHTENS_MASK						(0x0C00)
#define SRVCALENDAR_NBMONTHUNIT_SHIFT						(6)
#define SRVCALENDAR_NBMONTHUNIT_MASK						(0x03C0)
#define SRVCALENDAR_NBDAYTENS_SHIFT							(4)
#define SRVCALENDAR_NBDAYTENS_MASK							(0x0030)
#define SRVCALENDAR_NBDAYUNIT_SHIFT							(0)
#define SRVCALENDAR_NBDAYUNIT_MASK							(0x000F)


/* 9'41''00*/
#define SRVRTCC_INIT_HOUR							0x9
#define SRVRTCC_INIT_MIN							0x41
#define SRVRTCC_INIT_SECONDE 						0x00

/* june, 29 2007*/
#define SRVRTCC_INIT_DAY							0x29
#define SRVRTCC_INIT_MONTH							0x6
#define SRVRTCC_INIT_YEAR	 						0x7

#define SRVRTCC_GETUNIT(x)							(x%10)
#define SRVRTCC_GETTENS(x)							(x/10)

/*===========================================================================================================
						Enum definition
===========================================================================================================*/



/*===========================================================================================================
						structure definition
===========================================================================================================*/
/** @struct srvCalendar_CommandBitField_Type
 *  @var srvEM4325_CommandBitField_Type::ui16DayUnit
 *  Member 'ui16DayUnit' is the unit of the number of day (0 to 9)
 *  @var srvCalendar_CommandBitField_Type::ui16DayTens
 *  Member 'ui16DayTens' is the tens of the number of day (0 to 3)
 *  @var srvCalendar_CommandBitField_Type::ui16MonthUnit
 *  Member 'ui16MonthUnit' is the unit of the number of month (0 to 9)
 *  @var srvCalendar_CommandBitField_Type::ui16MonthTens
 *  Member 'ui16MonthTens' is the tens of the number of day (0 to 1)
 *  @var srvCalendar_CommandBitField_Type::ui16YearUnit
 *  Member 'ui16YearUnit'  is the number of year (0 to 15)
 */
typedef struct {
	uint16_t ui16DayUnit : 4;	/* LSB */
	uint16_t ui16DayTens: 2;
	uint16_t ui16MonthUnit: 4;
	uint16_t ui16MonthTens : 2;
	uint16_t ui16YearUnit : 4;				/* MSB */
}srvCalendar_CommandBitField_Type;

/** @union srvEM4325_CmdFirstWord_union
 *
 */
typedef union {
	srvCalendar_CommandBitField_Type sBitsField;
	uint16_t ui16Value;
}srvEM4325_HowOldAreYou_union;


/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
extern uint16_t 	ui16BoardAge ;
/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	srvCalendar_GetBoardAge 		( uint16_t **ui16LifeTimeConverted  );
void 	srvCalendar_DateDiff 			( const uint32_t ui32RecentDate , uint32_t ui32OldDate ,  uint32_t *ui32diffdate );
void 	srvCalendar_Init 				( void );
void 	srvCalendar_SetTime 			( const uint8_t ui8second,const uint8_t ui8minutes ,const uint8_t ui8hours );
void 	srvCalendar_SetDate 			( const uint8_t ui8day,const uint8_t ui8month ,const uint8_t ui8year );
bool 	srvCalendar_IsCoherentDate 		( const uint32_t ui32DateTocheck );
void 	srvCalendar_ConfigFuturCCxEvent	( const uint8_t ui8ChannelNumber, const uint32_t ui32Date, const uint32_t ui32Time );
uint8_t srvCalendar_CompareDateTime 	( const uint32_t ui32Date1, const uint32_t ui32Time1, const uint32_t ui32Date2, const uint32_t ui32Time2);
void 	srvCalendar_DisableMeasureCCxIrq ( void );
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
void 	srvCalendar_EnableMinutLifeComputing	( void );
void 	srvCalendar_DisableMinutLifeComputing	( void );
#endif
#endif /* SRVCALENDAR_H_ */
