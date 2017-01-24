/*******************************************************************************
 * @file crc16.h
 * @brief this file contains the function set to compute and check the CRC16
 * according to the epc standard
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef CRC16_H
#define CRC16_H

//#include "utils.h"
#include "common_library.h"


#define CRC_POLYNOMIAL	(0xA001)
#define CRC_INIT_VALUE	(0xFFFF)
#define CRC_POLY16 		(0x1021)
#define CRC16_RESIDUE 	(0x1D0F)

/**
 * @brief Performs CRC computation on "bit_data" fake bits array of a given "n" length.
 */
uint16_t crc16_ccitt_bit(const uint8_t* const bit_data, uint16_t n);

/**
 * @brief Performs CRC computation on "data" array of a given "n" length.
 */
uint16_t crc16_ccitt_byte(const uint8_t* const data, uint16_t n);

/**
 * @brief Performs CRC-CCITT computation on "data" array of a given "bit_length".
 */
uint16_t crc16_ccitt_bit2(const uint8_t buffer[], uint16_t bit_length);

void crc16_ccitt_bit2_part(uint8_t *buf,unsigned short length,unsigned short *crcReg);


/**
 * @brief Adds the final not "~" to the CRC-CCITT.
 */
uint16_t crc16_ccitt_bit_epc(const uint8_t buffer[], uint16_t bit_length);

/**
 * @brief Performs CRC computation on "data" array of a given "bit_length".
 */
uint16_t crc16_ccitt_bit3( const uint8_t buf[], uint16_t bit_length);

uint16_t crc_16(uint8_t* bytes, uint32_t size);

void CRC16_init_crc16_computation(void);
void CRC16_update_crc16_computation(unsigned char bit);
uint16_t CRC16_IsRightResidue (void);




#endif // #ifndef CRC16_H
