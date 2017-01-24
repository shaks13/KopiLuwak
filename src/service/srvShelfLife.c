/*******************************************************************************
 * @file srvShelfLife.c
 * @brief ShelfLife service API implementation
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)

#include "srvShelfLife.h"

/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
/* The loss factor allows the service to compute the loss shelf life according to the temperature, the previous
 * shelf life computed and the time which has elapsed since the last calculation:
 * 		loss factor = derivative [Avg(temp)][OldShelfLife] */
static float srvShelfLife_faaLossFactor[SRV_SHELFLIFE_NB_TEMPERATURE_IN_2D_ARRAY][SRV_SHELFLIFE_NB_SHELFLIFER_IN_2D_ARRAY] = {
/* 					Shelf life:			0(last)	0.1		0.2		0.3		0.4		0.5		0.6		0.7		0.8		0.9		1(init)	*/
/* for -50 deg ; Shelf life(t) */	{	6, 		6,		6,		6,		6,		6,		6,		6,		6,		6,		6},
/* for -40 deg ; Shelf life(t) */	{	5, 		5,		5,		5,		5,		5,		5,		5,		5,		5,		5},
/* for -30 deg ; Shelf life(t) */	{	4, 		4,		4,		4,		4,		4,		4,		4,		4,		4,		4},
/* for -20 deg ; Shelf life(t) */	{	3, 		3,		3,		3,		3,		3,		3,		3,		3,		3,		3},
/* for -10 deg ; Shelf life(t) */	{	2, 		2,		2,		2,		2,		2,		2,		2,		2,		2,		2},
/* for   0 deg ; Shelf life(t) */	{	1.5, 	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5},
/* for  10 deg ; Shelf life(t) */	{	1.2, 	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2},
/* for  20 deg ; Shelf life(t) */	{	1, 		1,		1,		1,		1,		1,		1,		1,		1,		1,		1},
/* for  30 deg ; Shelf life(t) */	{	1.2, 	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2,	1.2},
/* for  40 deg ; Shelf life(t) */	{	1.5, 	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5,	1.5},
/* for  50 deg ; Shelf life(t) */	{	2, 		2,		2,		2,		2,		2,		2,		2,		2,		2,		2},
/* for  60 deg ; Shelf life(t) */	{	3, 		3,		3,		3,		3,		3,		3,		3,		3,		3,		3}
};

/* The absolute shelf life of a glue pot: starts with 1, ends with 0:
 * e.g. if the initial life provided fot the product is 260 days, the absolute initial shelf life is 1/260 */
static double srvShelfLife_dShelfLife = 1.0f;

/* the factor to convert the loss coefficients to an absolute value according to the "srvShelfLife_dShelfLife" variable */
static const float srvShelfLife_fAbsoluteConv = (float)((float)(1.0f)/SRV_SHELFLIFE_INITIAL_LIFE_IN_DAY); /* e.g. with 260 days: 1/260 */

/* These variables help to segment the temperature buffer in order to compute the average on temperature between two shelf life calculation */
static uint16_t srvShelfLife_ui16NbTempSamples;
static uint16_t  srvShelfLife_ui16IdxTempSamples;
/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/
static uint8_t 	srvShelfLife_GetIdxTempIn2DArray 		(const int16_t i16Temperature, uint8_t * pui8Idx);
static void 	srvShelfLife_GetIdxShelfLifeIn2DArray 	(uint8_t * pui8Idx);

/*===========================================================================================================
						Private functions definitions
===========================================================================================================*/

/***************************************************************************//**
 * @brief
 * 		Finds the temperature index in the loss factor 2D array.
 *
 * @details
 * 		if the temperature is -50: tenth = -5, idx 	= -5 + offset
 * 													= -5 + 5
 * 													= 0
 * 		if the temperature is 20: tenth = 2,   idx 	= 2 + offset
 * 													= 2 + 5
 * 													= 7 *
 * @param[in] i16Temperature: the temperature
 * @param[out] pui8Idx: the pointer to the index
 *
 * @return 		CROSSRFID_SUCCESSCODE:
 * @return 		CROSSRFID_ERROR_UNSPECIFIED:
 ******************************************************************************/
static uint8_t srvShelfLife_GetIdxTempIn2DArray (const int16_t i16Temperature, uint8_t * pui8Idx)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	int8_t	i8Tenth;
	int8_t	i8Unit;
	*pui8Idx = 0;

	/* the temperature is coherent */
	if(i16Temperature > PROTOCOL_EM4325_TEMERATURE_0_KELVIN)
	{
		/* computes the tenth */
		i8Tenth = i16Temperature / 10;

		/* temperature in range */
		if((SRV_SHELFLIFE_TENTH_MIN_TEMPERATURE <= i8Tenth) && (SRV_SHELFLIFE_TENTH_MAX_TEMPERATURE >= i8Tenth))
		{
			/* computes the unit */
			i8Unit = i16Temperature % 10;

			/* rounds the tenth according to the units */
			if((i8Unit > 5) && ((i8Tenth + SRV_SHELFLIFE_OFFSET_IDX_TEMPERATURE) < (SRV_SHELFLIFE_NB_TEMPERATURE_IN_2D_ARRAY-1)))
			{
				i8Tenth += 1;
			}
			else if((i8Unit < -5) && ((i8Tenth + SRV_SHELFLIFE_OFFSET_IDX_TEMPERATURE) > 0))
			{
				i8Tenth -= 1;
			}
			else { /* do nothing */ }

			*pui8Idx = i8Tenth + SRV_SHELFLIFE_OFFSET_IDX_TEMPERATURE;
		}
		/* temperature not in range: positive case */
		else if(i8Tenth > SRV_SHELFLIFE_TENTH_MAX_TEMPERATURE)
		{
			/* index = max = size of array - 1 */
			*pui8Idx = SRV_SHELFLIFE_NB_TEMPERATURE_IN_2D_ARRAY - 1;
		}
		/* temperature not in range: negative case */
		else if (i8Tenth < SRV_SHELFLIFE_TENTH_MIN_TEMPERATURE)
		{
			/* index = min = 0 */
			*pui8Idx = 0;
		}
		else
		{
			/* do nothing */
		}
	}
	else
	{
		/* unexpected temperature */
		ui8Status = CROSSRFID_ERROR_UNSPECIFIED;
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 * 		Finds the shelf life index in the loss factor 2D array.
 *
 * @param[out] pui8Idx: the pointer to the index
 *
 ******************************************************************************/
static void srvShelfLife_GetIdxShelfLifeIn2DArray (uint8_t * pui8Idx)
{
	/* computes the tenth */
	*pui8Idx = (uint8_t)(srvShelfLife_dShelfLife * 10);
}

/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 * 		Computes the shelf life according to the (d shelfLife / dt) derivative
 *		and the temperature average during the 'dt'.
 *
 * @details
 * 		shelf life[t] = shelf life[t-(Delta t)] - derivative[temp][shelf life[t-(Delta t)]] * (Delta t)
 *
 * @return 		CROSSRFID_SUCCESSCODE: shelf life successfully computed
 * @return 		CROSSRFID_GP_ERROR_LIFE_COMPUT: error during the life calculation due to the temperature acquisitions
 * @return 		CROSSRFID_GP_END_LIFE: reach the expiration date
 ******************************************************************************/
uint8_t srvShelfLife_ComputeShelfLife (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint8_t uiIdxShelfLife, uiIdxTemp; /* index used to point to a loss factor in the srvShelfLife_faaLossFactor array */
	int16_t i16TemperatureAvg;
	uint16_t ui16Nbtemp = 0;

	/* reads the number of sample since the last average calculation */
	srvtemp_GetNbMeas(&ui16Nbtemp);
	/* no temperature acquisition or buffer overflow */
	if(ui16Nbtemp <= srvShelfLife_ui16IdxTempSamples)
	{
		/* ERROR, should not happen */
		ui8Status = CROSSRFID_GP_ERROR_LIFE_COMPUT;
	}
	else
	{
		/* Computes temperature average on a section of the sample buffer */
		ui8Status = srvtemp_ComputeAvgOnSectionOfTempBuffer (srvShelfLife_ui16IdxTempSamples,(ui16Nbtemp-srvShelfLife_ui16IdxTempSamples),&i16TemperatureAvg);

		/* computing the average successful */
		if (CROSSRFID_SUCCESSCODE == ui8Status)
		{
			/* saves the number of sample since the last average calculation */
			/* saves the index for the next average calculation */
			srvShelfLife_ui16NbTempSamples 	= ui16Nbtemp;
			srvShelfLife_ui16IdxTempSamples	= srvShelfLife_ui16NbTempSamples;

			/* gets the index of the temperature in the "Loss Factor" 2D array */
			ui8Status = srvShelfLife_GetIdxTempIn2DArray(i16TemperatureAvg, &uiIdxTemp);

			/* gets the index of the shelf life in the "Loss Factor" 2D array */
			srvShelfLife_GetIdxShelfLifeIn2DArray(&uiIdxShelfLife);

			if(CROSSRFID_SUCCESSCODE == ui8Status)
			{
				srvShelfLife_dShelfLife = (srvShelfLife_dShelfLife
						- (((double)srvShelfLife_faaLossFactor[uiIdxTemp][uiIdxShelfLife] * srvShelfLife_fAbsoluteConv) * SRV_SHELFLIFE_TIME_DELTA_BETWEEN_CALCULATION));

				if(srvShelfLife_dShelfLife <= 0)
				{
					ui8Status = CROSSRFID_GP_END_LIFE;
					srvShelfLife_dShelfLife = 0.0f;
				}
			}
			else
			{
				/* no loss factor found, unexpected index in the srvShelfLife_faaLossFactor array */
				ui8Status = CROSSRFID_GP_ERROR_LIFE_COMPUT;
			}
		}
	}


	return ui8Status;
}

/***************************************************************************//**
 * @brief
 * 		Gets the last computed shelf life.
 *
 * @return The absolute value of the shelf life
 ******************************************************************************/
double srvShelfLife_GetShelfLife (void)
{
	return srvShelfLife_dShelfLife;
}

/***************************************************************************//**
 * @brief
 * 		Resets the last computed shelf life.
 *
 * @return The absolute value of the shelf life
 ******************************************************************************/
void srvShelfLife_ResetShelfLife (void)
{
	srvShelfLife_dShelfLife = 1.0f;

	/* reset the temperature buffer segmentation variables which select the index and the number
	 * of temperature samples to compute the average*/
	srvShelfLife_ui16NbTempSamples = 0;
	srvShelfLife_ui16IdxTempSamples = 0;
}

#endif /* APP_GLUEPOT */
