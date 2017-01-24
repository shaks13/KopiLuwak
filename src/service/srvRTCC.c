/*******************************************************************************
 * @file srvRTCC.c
 * @brief this file defines the command set for the RTC and calendar
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvRTCC.h"


/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
static uint16_t sui16LifeTime = 52; /* 52 is for debug purpose*/
/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/


/*===========================================================================================================
						Private functions definition
===========================================================================================================*/

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/



/***************************************************************************//**
 * @brief 		This function increase the lifetime of the board
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvrtcc_IncreaseDayCounter ( void )
{
	sui16LifeTime ++;
}


/***************************************************************************//**
 * @brief 		This function returns the number of day
 * @param[in] 	none
 * @param[out] 	ui16LifeTime :number of day since the last reset
 * @return 		none
 ******************************************************************************/
void srvrtcc_GetDayCounter ( uint16_t *ui16LifeTime )
{
	(*ui16LifeTime) = sui16LifeTime ;
}


/***************************************************************************//**
 * @brief 		This function gets the time in the RTCC calendar and format as defined
 * by the crossRFID datasheet
 * @param[in] 	none
 * @param[out] 	ui32time : time
 * @return	 	none
 ******************************************************************************/
void srvrtcc_GetTime ( uint32_t *ui32time )
{
uint32_t ui32registerTime = 0  ;

	ui32registerTime = RTCC_TimeGet ();
	(*ui32time) = ui32registerTime & ( _RTCC_TIME_SECU_MASK | _RTCC_TIME_SECT_MASK | _RTCC_TIME_MINU_MASK | _RTCC_TIME_MINT_MASK | _RTCC_TIME_HOURU_MASK | _RTCC_TIME_HOURT_MASK);

}

/***************************************************************************//**
 * @brief 		This function gets the date in the RTCC calendar
 * @param[in] 	none
 * @param[out]	ui32date : is the current data of the board
 * @return 		none
 ******************************************************************************/
void srvrtcc_GetDate ( uint32_t *ui32date )
{
uint32_t ui32registerdate = 0  ;

	ui32registerdate = RTCC_DateGet ();
	(*ui32date) = ui32registerdate & (_RTCC_DATE_DAYOMU_MASK | _RTCC_DATE_DAYOMT_MASK | _RTCC_DATE_MONTHU_MASK  |_RTCC_DATE_MONTHT_MASK |  _RTCC_DATE_YEARU_MASK | _RTCC_DATE_YEART_MASK);
}

