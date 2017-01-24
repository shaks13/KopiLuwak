/*******************************************************************************
 * @file crc5.h
 * @brief this file contains the function set to compute and check the CRC5
 * according to the epc standard
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef CRC5_H
#define CRC5_H
#include "common_library.h"

#define POLY5 0x48

uint8_t crc5(const uint8_t *buf, uint16_t length);

#endif // #ifndef CRC5_H
