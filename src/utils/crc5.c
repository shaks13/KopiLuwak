/*******************************************************************************
 * @file crc5.c
 * @brief this file contains the function set to compute and check the CRC5
 * according to the epc standard
 * @version 1.00.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "crc5.h"

uint8_t crc5(const uint8_t *buf, uint16_t length)
{
	uint8_t shift;
	uint8_t frame_data;
	uint8_t val;
	uint16_t i;
	/*
	 this version of the 5 bit crc closely models the hardware shift register description.
	 */
	shift = POLY5;
	frame_data = 0;

	for (i = 0; i < length; i++)
	{
		if ((i % 8) == 0)
			frame_data = *buf++;
		val = shift ^ frame_data;
		shift = shift << 1;
		frame_data = (frame_data << 1);
		if (val & 0x80)
			shift = shift ^ POLY5;
	}
	shift = shift >> 3;
	return (uint8_t) (shift);
}
