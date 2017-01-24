/*******************************************************************************
 * @file srvCalendar
 * @brief this file defines the command setfor the calendar (RTCC)
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvCalendar.h"


/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/

/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
uint16_t 	ui16BoardAge = 0;		/* this variable is used to save the age of the board and its be shared between tasks*/

/*===========================================================================================================
						Private functions declarations
===========================================================================================================*/
static void srvCalendar_EnableDayIrq 	( void );
static void srvCalendar_EnableMinuteIrq	( void );
static void srvCalendar_DisableMinuteIrq( void );
static void srvCalendar_EnableCCxIrq 	( void );
static void srvrtcc_SetCCxTime 			( const int ch, const uint32_t ui32time );
static void srvrtcc_SetCCxDate 			( const int ch, const uint32_t ui32date );

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief This function write the CCx_TIME in the RTCC calendar
 *
 * @param[in] ui32time: CCxTIME value.
 * @param[out] none
 * @return none
 *
 ******************************************************************************/

static void srvrtcc_SetCCxTime (const int ch, const uint32_t ui32time )
{
	RTCC_ChannelTimeSet(ch,ui32time);
}

/***************************************************************************//**
 * @brief This function write the CCx_DATE in the RTCC calendar
 *
 * @param[in] ui32date: CCxDATE value.
 * @param[out] none
 * @return none
 *
 ******************************************************************************/

static void srvrtcc_SetCCxDate (const int ch, const uint32_t ui32date )
{

	RTCC_ChannelDateSet (ch,ui32date);
}

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief 		This function enables the irq of each day
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void srvCalendar_EnableDayIrq ( void )
{
	RTCC_IntEnable (RTCC_IEN_DAYTICK);

}

/***************************************************************************//**
 * @brief 		This function enables the irq of each minute
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void srvCalendar_EnableMinuteIrq ( void )
{
	RTCC_IntEnable (RTCC_IEN_MINTICK);
}

/***************************************************************************//**
 * @brief 		This function disables the minute irq
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void srvCalendar_DisableMinuteIrq ( void )
{
	RTCC_IntDisable (RTCC_IEN_MINTICK);
}

/***************************************************************************//**
 * @brief 		This function enables the irqs of rtcc CCx
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void srvCalendar_EnableCCxIrq ( void )
{
	RTCC_IntEnable (RTCC_IEN_CC1);
}

/***************************************************************************//**
 * @brief 		This function sets the time in the RTCC calendar
 *
 * @param[in]	ui8second: the seconds value
 * @param[in]	ui8minutes: the minutes value
 * @param[in] 	ui8hours: the hours value
 * @return 		none
 ******************************************************************************/
void srvCalendar_SetTime ( const uint8_t ui8second,const uint8_t ui8minutes ,const uint8_t ui8hours )
{
uint32_t ui32time = 0  ;
#if 0
	/* add the seconds */
	ui32time = (((SRVRTCC_GETUNIT (ui8second) <<_RTCC_TIME_SECU_SHIFT )& _RTCC_TIME_SECU_MASK )  |
				((SRVRTCC_GETTENS (ui8second) <<_RTCC_TIME_SECT_SHIFT)& _RTCC_TIME_SECT_MASK ) );
	/* add the minute */
	ui32time |= (((SRVRTCC_GETUNIT (ui8minutes)<<_RTCC_TIME_MINU_SHIFT ) & _RTCC_TIME_MINU_MASK )  |
				((SRVRTCC_GETTENS (ui8minutes) <<_RTCC_TIME_MINT_SHIFT ) & _RTCC_TIME_MINT_MASK ) );
	/* add the hours */
	ui32time |= (((SRVRTCC_GETUNIT (ui8hours)<<_RTCC_TIME_HOURU_SHIFT ) & _RTCC_TIME_HOURU_MASK ) |
				((SRVRTCC_GETTENS (ui8hours) <<_RTCC_TIME_HOURT_SHIFT) & _RTCC_TIME_HOURT_MASK )  );
#else
	ui32time = (ui8hours<<_RTCC_TIME_HOURU_SHIFT) |  (ui8minutes<<_RTCC_TIME_MINU_SHIFT) | (ui8second << _RTCC_TIME_SECU_SHIFT);
#endif
	RTCC_TimeSet (ui32time);
}

/***************************************************************************//**
 * @brief 		This function sets the time in the RTCC calendar
 *
 * @param[in] 	ui8day: the days value
 * @param[in] 	ui8month: the months value
 * @param[in] 	ui8year: the years value
 * @return 		none
 *
 ******************************************************************************/
void srvCalendar_SetDate ( const uint8_t ui8day,const uint8_t ui8month ,const uint8_t ui8year )
{
uint32_t ui32date = 0  ;

	/* add the day  */
#if 0
	ui32date = (((SRVRTCC_GETUNIT (ui8day) <<_RTCC_DATE_DAYOMU_SHIFT )& _RTCC_DATE_DAYOMU_MASK )|
				((SRVRTCC_GETTENS (ui8day) <<_RTCC_DATE_DAYOMT_SHIFT) & _RTCC_DATE_DAYOMT_MASK )  );
	/* add the month */
	ui32date |= (((SRVRTCC_GETUNIT (ui8month)<<_RTCC_DATE_MONTHU_SHIFT)& _RTCC_DATE_MONTHU_MASK ) |
				((SRVRTCC_GETTENS (ui8month) <<_RTCC_DATE_MONTHT_SHIFT) & _RTCC_DATE_MONTHT_MASK )  );
	/* add the year */
	ui32date |= (((SRVRTCC_GETUNIT (ui8year)<<_RTCC_DATE_YEARU_SHIFT)& _RTCC_DATE_YEARU_MASK )|
				((SRVRTCC_GETTENS (ui8year) <<_RTCC_DATE_YEART_SHIFT)& _RTCC_DATE_YEART_MASK ) );
#else
	ui32date = (ui8year<<_RTCC_DATE_YEARU_SHIFT) |  (ui8month<<_RTCC_DATE_MONTHU_SHIFT) | (ui8day << _RTCC_DATE_DAYOMU_SHIFT);
#endif
	RTCC_DateSet (ui32date);
}

/***************************************************************************//**
 * @brief 		This function initializes the RTC as a calendar
 *
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvCalendar_Init ( void )
{
	/* Initializes the rtc driver, CALENDAR mode */
	RTCDRV_Init ();

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	/* disable the rtcc calendar irq which trigs the life computing */
	srvCalendar_DisableMinutLifeComputing();
#endif

	/* Sets initial date and time */
	srvCalendar_SetTime ( SRVRTCC_INIT_SECONDE, SRVRTCC_INIT_MIN, SRVRTCC_INIT_HOUR);	/* write the time */
	srvCalendar_SetDate ( SRVRTCC_INIT_DAY, SRVRTCC_INIT_MONTH, SRVRTCC_INIT_YEAR );	/* write the date */

	/* Enable day IRQ */
	//srvCalendar_EnableDayIrq ();

	/* Enable CC1 IRQ */
	srvCalendar_EnableCCxIrq();

	NVIC_SetPriority(RTCC_IRQn, (configKERNEL_INTERRUPT_PRIORITY >> (8 - __NVIC_PRIO_BITS)) & 0x7);
}


/***************************************************************************//**
 * @brief 		This function converts the day counter to the folowing format
 * |------------|-----------|-----------|-----------|-----------|
 * |  4 bits	|   2bits   |  4 bits	|  2 bits	|  4 bits	|
 * |------------|-----------|-----------|-----------|-----------|
 * | Year Unit  | Month tens| month unit| day ten   | day unit 	|
 * |------------|-----------|-----------|-----------|-----------|
 * @param[in] 	none
 * @param[out] 	ui16LifeTimeConverted :
 * @return 		none
 ******************************************************************************/
void srvCalendar_GetBoardAge ( uint16_t **pui16LifeTimeConverted  )
{
uint16_t	ui16NbDay=0;
uint8_t ui8NbYear  	= 0;
uint8_t ui8NbMonth 	= 0 ;
uint8_t ui8NbDay 	= 0 ;

	(*pui16LifeTimeConverted) = &ui16BoardAge;
	srvrtcc_GetDayCounter (&ui16NbDay);
	/* compute the number if year month and day*/
	ui8NbYear = ui16NbDay/365;
	ui8NbMonth = (ui16NbDay - ui8NbYear*365)/30;
	ui8NbDay = ui16NbDay - ui8NbYear*365 - ui8NbMonth*30;
	ui16BoardAge = 	( ((ui8NbYear) 		<< SRVCALENDAR_NBYEAR_SHIFT) 		& SRVCALENDAR_NBYEAR_MASK) |
					( (SRVRTCC_GETTENS(ui8NbMonth ) << SRVCALENDAR_NBMONTHTENS_SHIFT) 	& SRVCALENDAR_NBMONTHTENS_MASK) |
					( (SRVRTCC_GETUNIT(ui8NbMonth ) << SRVCALENDAR_NBMONTHUNIT_SHIFT) 	& SRVCALENDAR_NBMONTHUNIT_MASK) |
					( (SRVRTCC_GETTENS(ui8NbDay) 	<< SRVCALENDAR_NBDAYTENS_SHIFT) 	& SRVCALENDAR_NBDAYTENS_MASK) |
					( (SRVRTCC_GETUNIT (ui8NbDay) 	<< SRVCALENDAR_NBDAYUNIT_SHIFT) 	& SRVCALENDAR_NBDAYUNIT_MASK) ;


}

/***************************************************************************//**
 * @brief 		This function makes the difference between two dates
 * @param[in] 	ui32RecentDate : the more recent date
 * @param[in] 	ui32OldDate : the older date
 * @param[out] 	ui32diffdate : is the current data of the board
 * @return 		none
 ******************************************************************************/
void srvCalendar_DateDiff ( const uint32_t ui32RecentDate , uint32_t ui32OldDate ,  uint32_t *ui32diffdate )
{
int8_t i8nbday = 0;
int8_t i8nbMonth = 0;
int8_t i8nbYear = 0;

 	(*ui32diffdate)= 0 ;

	i8nbYear = (ui32RecentDate & _RTCC_DATE_YEARU_MASK)  + 10* (ui32RecentDate & _RTCC_DATE_YEART_MASK) -
					  (ui32OldDate & _RTCC_DATE_YEARU_MASK)  + 10* (ui32RecentDate & _RTCC_DATE_YEART_MASK)	;

	i8nbMonth = (ui32RecentDate & _RTCC_DATE_MONTHU_MASK)  + 10* (ui32RecentDate & _RTCC_DATE_MONTHT_MASK) -
					  (ui32OldDate & _RTCC_DATE_MONTHU_MASK)  + 10* (ui32RecentDate & _RTCC_DATE_MONTHT_MASK)	;

	if (i8nbMonth<0)
	{
		i8nbYear ++ ;
		i8nbMonth =+12;
	}
	else {/* do nothing*/}

	i8nbday = (ui32RecentDate & _RTCC_DATE_DAYOMU_MASK)  + 10* (ui32RecentDate & _RTCC_DATE_DAYOMT_MASK) -
				  (ui32OldDate & _RTCC_DATE_DAYOMU_MASK)  + 10* (ui32RecentDate & _RTCC_DATE_DAYOMT_MASK)	;
	if (i8nbday<0)
	{
		i8nbMonth ++ ;
		i8nbday =+30;
	}
	else {/* do nothing*/}


	/* add the day  */
	(*ui32diffdate) = (((SRVRTCC_GETUNIT (i8nbday) <<_RTCC_DATE_DAYOMU_SHIFT )& _RTCC_DATE_DAYOMU_MASK )|
				((SRVRTCC_GETTENS (i8nbday) <<_RTCC_DATE_DAYOMT_SHIFT) & _RTCC_DATE_DAYOMT_MASK )  );
	/* add the month */
	(*ui32diffdate)  |= (((SRVRTCC_GETUNIT (i8nbMonth)<<_RTCC_DATE_MONTHU_SHIFT)& _RTCC_DATE_MONTHU_MASK ) |
				((SRVRTCC_GETTENS (i8nbMonth) <<_RTCC_DATE_MONTHT_SHIFT) & _RTCC_DATE_MONTHT_MASK )  );
	/* add the year */
	(*ui32diffdate)  |= ((i8nbYear <<_RTCC_DATE_YEARU_SHIFT)& _RTCC_DATE_YEARU_MASK ) ;

}

/***************************************************************************//**
 * @brief 		This function checks if the date could be coherent with the
 * date stored in the board. the date to check should not be anterior to the
 * board date
 * @param[in]	ui32DateTocheck : the more recent date
 * @param[out] 	none
 * @return 		true : the date is coherent with this one in the board
 * @return 		false : the date is not coherent with this one in the board
 ******************************************************************************/
bool srvCalendar_IsCoherentDate ( const uint32_t ui32DateTocheck )
{
uint32_t ui32BoardDate;
bool 		bresult = true;
uint8_t ui8temp;

	/* to ACQ (begin or end) infinite or disable a calendar event */
	if(KERNEL_ACQ_BEGIN_END_INFINITE_VALUE == ui32DateTocheck)
	{
		/* Do nothing */
	}
	else
	{
		/* test the unit */
		if ( ((ui32DateTocheck & (_RTCC_DATE_DAYOMU_MASK |_RTCC_DATE_DAYOMT_MASK)) == 0x00) ||	/*check the unit of month and day is different to  0*/
			 ((ui32DateTocheck & (_RTCC_DATE_MONTHU_MASK | _RTCC_DATE_MONTHT_MASK)) == 0x00) )
		{
			bresult = false;
		}

		/* test the day */
		ui8temp = ((ui32DateTocheck & _RTCC_DATE_DAYOMU_MASK) >>_RTCC_DATE_DAYOMU_SHIFT) +
					((ui32DateTocheck & _RTCC_DATE_DAYOMT_MASK) >>_RTCC_DATE_DAYOMT_SHIFT) * 10;
		if(ui8temp > 31)
		{
			bresult = false;
		}

		/* test the month */
		ui8temp = ((ui32DateTocheck & _RTCC_DATE_MONTHU_MASK) >>_RTCC_DATE_MONTHU_SHIFT) +
					((ui32DateTocheck & _RTCC_DATE_MONTHT_MASK) >>_RTCC_DATE_MONTHT_SHIFT) * 10;
		if(ui8temp > 12)
		{
			bresult = false;
		}

		/* test the year */
		ui8temp = ((ui32DateTocheck & _RTCC_DATE_YEARU_MASK) >>_RTCC_DATE_YEARU_SHIFT) +
					((ui32DateTocheck & _RTCC_DATE_YEART_MASK) >>_RTCC_DATE_YEART_SHIFT) * 10;
		if(ui8temp == 0)
		{
			bresult = false;
		}

		if (true == bresult)
		{
			/*check the coherence with the board date*/
			srvrtcc_GetDate ( &ui32BoardDate );
			if ((ui32DateTocheck - ui32BoardDate) < 0)
			{
				bresult = false;
			}
			else
			{
				bresult = true;
			}
		}
	}

	return bresult;
}

/***************************************************************************//**
 * @brief 		This function programs the calendar to trigger an event at a
 * 				date/time in the future.
 * @detail		Sets the CCxDate and the CCxTime to configure the interruption.
 * @param[in]	ui8ChannelNumber: the channel number of the RTCC between 0 and 2
 * @param[in]	ui32Date: the date when the RTCC is going to trigger the interruption.
 * @param[in] 	ui32Time: the time when the RTCC is going to trigger the interruption.
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvCalendar_ConfigFuturCCxEvent (const uint8_t ui8ChannelNumber, const uint32_t ui32Date, const uint32_t ui32Time)
{

	srvrtcc_SetCCxTime (ui8ChannelNumber, ui32Time);
	srvrtcc_SetCCxDate (ui8ChannelNumber, ui32Date);
	/* enable the IRQ*/
	switch (ui8ChannelNumber)
	{

		case 0 : //SERIAL_RTCCCHANNEL_0
			RTCC_IntClear (RTCC_IFC_CC0);
			RTCC_IntClear (RTCC_IFC_CC0);
			RTCC_IntEnable (RTCC_IEN_CC0);
		break;

		case 2 ://SERIAL_RTCCCHANNEL_2
			RTCC_IntClear (RTCC_IFC_CC2);
			RTCC_IntEnable (RTCC_IEN_CC2);
		break;

		default :
		break;
	}

}

/***************************************************************************//**
 * @brief 		This function compares date/time.
 * @param[in]	ui32Date1: the date 1
 * @param[in] 	ui32Time1: the time 1
 * @param[in]	ui32Date2: the date 2
 * @param[in] 	ui32Time2: the time 2
 * @param[out] 	none
 * @return 		the index of the date/time the most recent.
 * @return 		0 is it the same date/time.
 ******************************************************************************/
uint8_t srvCalendar_CompareDateTime (const uint32_t ui32Date1, const uint32_t ui32Time1, const uint32_t ui32Date2, const uint32_t ui32Time2)
{
	uint8_t ui8IsMostRecent;

	if(ui32Date1 < ui32Date2)
	{
		ui8IsMostRecent = 1;
	}
	else if(ui32Date1 == ui32Date2)
	{
		if(ui32Time1 < ui32Time2)
		{
			ui8IsMostRecent = 1;
		}
		else if(ui32Time1 == ui32Time2)
		{
			ui8IsMostRecent = 0;
		}
		else
		{
			ui8IsMostRecent = 2;
		}
	}
	else
	{
		ui8IsMostRecent = 2;
	}

	return ui8IsMostRecent;
}

/***************************************************************************//**
 * @brief 		This function disables the irqs of rtcc CCx
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvCalendar_DisableMeasureCCxIrq ( void )
{
	RTCC_IntDisable (RTCC_IEN_CC0);
	RTCC_IntDisable (RTCC_IEN_CC2);
	RTCC_IntClear (RTCC_IFC_CC0);
	RTCC_IntClear (RTCC_IFC_CC2);
}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/***************************************************************************//**
 * @brief 		This function clears and enables the minute irq
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvCalendar_EnableMinutLifeComputing ( void )
{
	RTCC_IntClear (KERNEL_RTCC_IFC_LIFE_CALCULATION_TICK);
	RTCC_IntEnable (KERNEL_RTCC_IEN_LIFE_CALCULATION_TICK);
}

/***************************************************************************//**
 * @brief 		This function clears and disables the minute irq
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvCalendar_DisableMinutLifeComputing ( void )
{
	RTCC_IntDisable (KERNEL_RTCC_IEN_LIFE_CALCULATION_TICK);
	RTCC_IntClear (KERNEL_RTCC_IFC_LIFE_CALCULATION_TICK);
}
#endif
