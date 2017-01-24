/*******************************************************************************
 * @file interface_i2c.h
 * @brief this files contains the function definitions for the management of the I2C
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef INTERFACE_I2C_H
#define INTERFACE_I2C_H

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "common_library.h"
#include "common_statuscode.h"

/*===========================================================================================================
						constant definition
===========================================================================================================*/
/* DEFAULT ADDRESS */
#define INTERFACEI2C_DEFAULT_ADDRESS					(0xA0)
#if USESTARTERKIT ==1
/* GPIO PIN/PORT */
#define INTERFACE_I2CSCL_PORT							(gpioPortC)
#define INTERFACE_I2CSCL_PIN							(11)
#define INTERFACE_I2CSDA_PORT							(gpioPortC)
#define INTERFACE_I2CSDA_PIN							(10)
#elif (USECROSSTAG == 1)
#define INTERFACE_I2CPWR_PORT							(gpioPortA)
#define INTERFACE_I2CPWR_PIN							(2)
#define INTERFACE_I2CSCL_PORT							(gpioPortC)
#define INTERFACE_I2CSCL_PIN							(11)
#define INTERFACE_I2CSDA_PORT							(gpioPortC)
#define INTERFACE_I2CSDA_PIN							(10)

#define INTERFACE_MAGNETODATAREADY_PORT					(gpioPortD)
#define INTERFACE_MAGNETODATAREADY_PIN					(10)
#define INTERFACE_MAGNETOIRQ_PORT						(gpioPortD)
#define INTERFACE_MAGNETOIRQ_PIN						(11)
#else

#endif
/* GET LSB AND MSB OF WORD */
#define INTERFACEI2C_GETLSB(ui16data)					(ui16data & 0x00FF)
#define INTERFACEI2C_GETMSB(ui16data)					((ui16data & 0xFF00)>>8)

/*===========================================================================================================
						prototype
===========================================================================================================*/
void 						I2C_init 						( void );
uint8_t 					I2C_IsAvailable 				( const uint8_t ui8address );
I2C_TransferReturn_TypeDef 	I2C_Readwith2AddressBytes		( const uint8_t i2c_ui16deviceAddress, const uint16_t ui16MemoryAdr, uint8_t ui8NbByteToRead, uint8_t *pui8ReadData );
void 						I2C_WriteByteWith2AddressBytes 	( const uint8_t i2c_ui16deviceAddress, const uint16_t ui16MemoryAdr, const uint8_t ui8ByteToWrite );
I2C_TransferReturn_TypeDef 	I2C_Write 						( const uint8_t i2c_ui16deviceAddress, const uint8_t ui8NbBytePerAddress, const uint16_t ui16NbByteToWrite, uint8_t const * pui8DataToWrite);
I2C_TransferReturn_TypeDef 	I2C_Read 						(const uint8_t i2c_ui16deviceAddress, const uint8_t ui8MemoryAdr, uint8_t ui8NbByteToRead, uint8_t *pui8ReadData);
I2C_TransferReturn_TypeDef 	I2C_CurrentRead 				( const uint8_t i2c_ui16deviceAddress , uint8_t ui8NbByteToRead, uint8_t *pui8ReadData);
void 						I2C_EnablePowerSuppply 			( bool bOnOrOff );
#endif /* INTERFACE_I2C_H */

