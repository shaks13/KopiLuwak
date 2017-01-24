/*******************************************************************************
 * @file bsp.h
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef BSP_H
#define BSP_H

#include "em_gpio.h"

//PA0
#define TOGGLE_PA0()		/* {GPIO_PinOutToggle(gpioPortA,0);}*/ GPIO->P[gpioPortA].DOUTTGL = 1 << 0;
#define GET_PA0(x)			{x = GPIO_PinInGet(gpioPortA,0);}
//PA1
#define TOGGLE_PA1()		{GPIO_PinOutToggle(gpioPortA,1);}
#define GET_PA1(x)			{x = GPIO_PinInGet(gpioPortA,1);}

//PB11
#define TOGGLE_PB11()		GPIO->P[gpioPortB].DOUTTGL = 1 << 11;

#define TOGGLE_PD6()		{GPIO_PinOutToggle(gpioPortD,6);}

// GPIO & LEDS
// Before Pearl migration:  #define CLEAR_PC0()			{ GPIO->P[gpioPortC].DOUTCLR = 1 << 0;}
#define CLEAR_PC0()			{ GPIO->P[gpioPortC].DOUT = ((GPIO->P[gpioPortC].DOUT & ~((uint32_t)(0x00000001 << 0))) | (uint32_t)(0x00000000) << 0);}
#define GET_PC0()			( GPIO->P[gpioPortC].DIN & 0x1)
//#define SET_PC0()			{ GPIO->P[gpioPortC].DOUTSET = 1 << 0;}
#define SET_PC0()			{ GPIO->P[gpioPortC].DOUT = ((GPIO->P[gpioPortC].DOUT & ~((uint32_t)(0x00000001 << 0))) | (uint32_t)(0x00000001) << 0);}
#define SET_CLK()			{ GPIO->P[gpioPortB].DOUTSET = 1 << 7;}		/*PB7*/
#define TOGGLE_PC0()		{ GPIO->P[gpioPortC].DOUTTGL = 1 << 0;}
#define GETDATA_PA1()		((GPIO->P[gpioPortA].DIN >> 1) & 0x1)		/* PA1*/

/* EM4325 I/O: */
/* SPI EM4325: CS*/
#define GET_EM4325_CS()			((GPIO->P[INTERFACE_CS_PORT].DIN >> INTERFACE_CS_PIN) & 0x1)
#define SET_EM4325_CS()			{ GPIO->P[INTERFACE_CS_PORT].DOUT = ((GPIO->P[gpioPortC].DOUT & ~((uint32_t)(0x00000001 << INTERFACE_CS_PIN))) | (uint32_t)(0x00000001) << INTERFACE_CS_PIN);}
#define CLEAR_EM4325_CS()		{ GPIO->P[INTERFACE_CS_PORT].DOUT = ((GPIO->P[gpioPortC].DOUT & ~((uint32_t)(0x00000001 << INTERFACE_CS_PIN))) | (uint32_t)(0x00000000) << INTERFACE_CS_PIN);}
/* SPI EM4325: MISO */
#define GET_EM4325_MISO()		((GPIO->P[INTERFACE_MISO_PORT].DIN >> INTERFACE_MISO_PIN) & 0x1)
/* SPI EM4325: AUX */
#define GET_EM4325_AUXSTATE()	((GPIO->P[INTERFACE_AUX_PORT].DIN>>INTERFACE_AUX_PIN) & 0x1)

#endif
