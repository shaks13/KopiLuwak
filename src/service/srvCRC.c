/*******************************************************************************
 * @file srvCRC.c
 * @brief this file defines the CRC service
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvCRC.h"

/*===========================================================================================================
						Public functions definitions
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   Computes the crc ccitt and put in it at the last address
 * @param[in] 	ui16BufLength
 *   Length of the buffer ( word unit)
 * @param[in] 	pu16Buffer
 *   Pointer to the buffer
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvCRC_InsertCrc16CCITT ( const uint16_t ui16BufLenght, uint16_t * pu16Buffer)
{
	const GPCRC_Init_TypeDef 	init = GPCRC_INIT_CRC16;
	uint16_t 					crc16;
	uint16_t					ui16Idx;

	GPCRC_Init(GPCRC, &init);
	GPCRC_Start(GPCRC);

	for(ui16Idx = 0; ui16Idx < ui16BufLenght-1; ui16Idx++)
	{
		GPCRC_InputU16(GPCRC, pu16Buffer[ui16Idx]);
	}
	crc16 = (uint16_t)GPCRC_DataReadBitReversed( GPCRC );

	pu16Buffer[ui16BufLenght-1] = crc16^0xffff;

	GPCRC_Reset(GPCRC);
}

/***************************************************************************//**
 * @brief
 *   Check if the residu of the crc ccitt in the buffer is right
 *
 * @param[in] ui16BufLenght
 *   Length of the buffer
 *
 * @param[in] pu16Buffer
 *   Pointer to the buffer
 *
 * @return
 * 	true: right residue, else wrong
 ******************************************************************************/
bool srvCRC_IsRightResidue ( const uint16_t ui16BufLength, uint16_t const * pu16Buffer)
{
	const GPCRC_Init_TypeDef 	init = GPCRC_INIT_CRC16;
	uint16_t					ui16Idx;
	bool 						bResult;

	GPCRC_Init(GPCRC, &init);
	GPCRC_Start(GPCRC);

	for(ui16Idx = 0; ui16Idx < ui16BufLength; ui16Idx++)
	{
		GPCRC_InputU16(GPCRC, pu16Buffer[ui16Idx]);
	}

	if ( SRVCRC_RESIDUE_CRC16CCITT == (uint16_t)GPCRC_DataReadBitReversed( GPCRC ))
	{
		bResult = true;
	}
	else
	{
		bResult = false;
	}

	GPCRC_Reset(GPCRC);

	return bResult;
}
