/*******************************************************************************
 * @file srvCRC.h
 * @brief this file defines the API of the CRC service
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVCRC_H_
#define SRVCRC_H_
#include "common_library.h"
#include "em_cmu.h"
#include "em_gpcrc.h"

/*===========================================================================================================
						Constants
===========================================================================================================*/
#define SRVCRC_RESIDUE_CRC16CCITT	(0x1D0F)


/** CRC16 configuration for GPCRC_Init_TypeDef structure. */
#define GPCRC_INIT_CRC16                                               \
{                                                                      \
  0x00001021UL,          /* CRC16 Polynomial value. */                 \
  0x0000FFFFUL,          /* Initialization value. */                   \
  true,                  /* When set to true the input bytes are reversed before entering the CRC calculation */ \
  true,                  /* When byte-level bit reversal is enabled then each byte of input data will be reversed before entering the CRC calculation.*/  \
  false,                 /* Disable byte mode. */                      \
  false,                 /* Disable automatic init on data read. */    \
  true,                  /* Enable GPCRC. */                           \
}
/*===========================================================================================================
						Public functions declarations
===========================================================================================================*/
void srvCRC_InsertCrc16CCITT		( const uint16_t ui16BufLenght, uint16_t * pu16Buffer);
bool srvCRC_IsRightResidue			( const uint16_t ui16BufLenght, uint16_t const * pu16Buffer);

#endif /* SRVCRC_H_ */
