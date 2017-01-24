/*******************************************************************************
 * @file interface_gpio.h
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef INTERFACE_GPIO_H
#define INTERFACE_GPIO_H


#include "common_library.h"
#include "em_gpio.h"
#include "em_bitband.h"
#include "interface_spi.h"
#include "interface_uart.h"
#include "interface_leuart.h"

/*===========================================================================================================
						constant
===========================================================================================================*/
#if USESTARTERKIT ==1
#define INTERFACE_ADXL363_IRQ1_PORT					(gpioPortD)
#define INTERFACE_ADXL363_IRQ1_PIN					(14)

#elif (USECROSSTAG==1)
#define INTERFACE_ADXL363_IRQ1_PORT					(gpioPortD)
#define INTERFACE_ADXL363_IRQ1_PIN					(14)

#else
/* I2C bus */
#define INTERFACE_I2CSDA_PORT						gpioPortA
#define INTERFACE_I2CSDA_PIN						0
#define INTERFACE_I2CSCL_PORT						gpioPortA
#define INTERFACE_I2CSCL_PIN						1

#define INTERFACE_RSTFT232_PORT						gpioPortB
#define INTERFACE_RSTFT232_PIN						11
#define INTERFACE_LIS2DHINT_PORT					gpioPortB
#define INTERFACE_LIS2DHINT_PIN						12
#define INTERFACE_ADXL363INT_PORT					gpioPortB
#define INTERFACE_ADXL363INT_PIN					13

/* EEPROM*/
#define INTERFACE_EEPROMCS_PORT						gpioPortC
#define INTERFACE_EEPROMCS_PIN						6
#define INTERFACE_EEPROMHOLD_PORT					gpioPortC
#define INTERFACE_EEPROMHOLD_PIN					7

#define INTERFACE_MICROSDPWR_PORT					gpioPortC
#define INTERFACE_MICROSDPWR_PIN					8

#define INTERFACE_MAGNETODATAIN_PORT				gpioPortC
#define INTERFACE_MAGNETODATAIN_PIN					9

#define INTERFACE_ADC_PORT							gpioPortC
#define INTERFACE_ADC_PIN							11

#define INTERFACE_SPI1TX_PORT						gpioPortD
#define INTERFACE_SPI1TX_PIN						9
#define INTERFACE_SPI1RX_PORT						gpioPortD
#define INTERFACE_SPI1RX_PIN						10
#define INTERFACE_SPI1CLK_PORT						gpioPortD
#define INTERFACE_SPI1CLK_PIN						11
#define INTERFACE_SPI1CS_PORT						gpioPortD
#define INTERFACE_SPI1CS_PIN						12
#define INTERFACE_SPI1CTS_PORT						gpioPortD
#define INTERFACE_SPI1CTS_PIN						13
#define INTERFACE_SPI1RTS_PORT						gpioPortD
#define INTERFACE_SPI1RTS_PIN						14
#define INTERFACE_SPI1PWR_PORT						gpioPortD
#define INTERFACE_SPI1PWR_PIN						15


#define INTERFACE_CMUCLK_PORT						gpioPortF
#define INTERFACE_CMUCLK_PIN						3
#define INTERFACE_SPOPWR_PORT						gpioPortF
#define INTERFACE_SPOPWR_PIN						4
#define INTERFACE_EXTCS_PORT						gpioPortF
#define INTERFACE_EXTCS_PIN							5
#define INTERFACE_MICROSDAVB_PORT					gpioPortF
#define INTERFACE_MICROSDAVB_PIN					6
#define INTERFACE_GREENLED_PORT						gpioPortF
#define INTERFACE_GREENLED_PIN						5
#define INTERFACE_REDLED_PORT						gpioPortF
#define INTERFACE_REDLED_PIN						6
#define INTERFACE_PULSECOUNTER_PORT					gpioPortF
#define INTERFACE_PULSECOUNTER_PIN					7

#endif
/*===========================================================================================================
						constant
===========================================================================================================*/
#if (USESTARTERKIT==1)


#elif (USECROSSTAG==1)


#define INTERFACE_VCOMENABLE_PORT					gpioPortA
#define INTERFACE_VCOMENABLE_PIN					5

#define INTERFACE_BUTTON2CAPA_PORT					gpioPortB
#define INTERFACE_BUTTON2CAPA_PIN					11
#define INTERFACE_BUTTON1CAPA_PORT					gpioPortB
#define INTERFACE_BUTTON1CAPA_PIN					12



#define INTERFACE_USART1TX_PORT						gpioPortC
#define INTERFACE_USART1TX_PIN						6
#define INTERFACE_USART1RX_PORT						gpioPortC
#define INTERFACE_USART1RX_PIN						7
#define INTERFACE_USART1CLK_PORT					gpioPortC
#define INTERFACE_USART1CLK_PIN						8
#define INTERFACE_USART1CS_PORT						gpioPortC
#define INTERFACE_USART1CS_PIN						9
#define INTERFACE_GREENLED_PORT						gpioPortD
#define INTERFACE_GREENLED_PIN						12
#define INTERFACE_USART1EN_PORT						gpioPortD
#define INTERFACE_USART1EN_PIN						13
#define INTERFACE_USART1IRQ_PORT					gpioPortD
#define INTERFACE_USART1IRQ_PIN						14

#define INTERFACE_SENSORI2C_PORT					gpioPortD
#define INTERFACE_SENSORI2C_PIN						9
#define INTERFACE_EFMDISPLAYEN_PORT					gpioPortD
#define INTERFACE_EFMDISPLAYEN_PIN					15


#define INTERFACE_ADCINPUT_PORT						gpioPortF
#define INTERFACE_ADCINPUT_PIN						3
#define INTERFACE_MAGNETICPWR_PORT					gpioPortF
#define INTERFACE_MAGNETICPWR_PIN					4
#define INTERFACE_MAGNETICIN_PORT					gpioPortF
#define INTERFACE_MAGNETICIN_PIN					5
#else
/* I2C bus */
#define INTERFACE_I2CSDA_PORT						gpioPortA
#define INTERFACE_I2CSDA_PIN						0
#define INTERFACE_I2CSCL_PORT						gpioPortA
#define INTERFACE_I2CSCL_PIN						1

#define INTERFACE_RSTFT232_PORT						gpioPortB
#define INTERFACE_RSTFT232_PIN						11
#define INTERFACE_LIS2DHINT_PORT					gpioPortB
#define INTERFACE_LIS2DHINT_PIN						12
#define INTERFACE_ADXL363INT_PORT					gpioPortB
#define INTERFACE_ADXL363INT_PIN					13

/* EEPROM*/
#define INTERFACE_EEPROMCS_PORT						gpioPortC
#define INTERFACE_EEPROMCS_PIN						6
#define INTERFACE_EEPROMHOLD_PORT					gpioPortC
#define INTERFACE_EEPROMHOLD_PIN					7

#define INTERFACE_MICROSDPWR_PORT					gpioPortC
#define INTERFACE_MICROSDPWR_PIN					8

#define INTERFACE_MAGNETODATAIN_PORT				gpioPortC
#define INTERFACE_MAGNETODATAIN_PIN					9

#define INTERFACE_ADC_PORT							gpioPortC
#define INTERFACE_ADC_PIN							11

#define INTERFACE_SPI1TX_PORT						gpioPortD
#define INTERFACE_SPI1TX_PIN						9
#define INTERFACE_SPI1RX_PORT						gpioPortD
#define INTERFACE_SPI1RX_PIN						10
#define INTERFACE_SPI1CLK_PORT						gpioPortD
#define INTERFACE_SPI1CLK_PIN						11
#define INTERFACE_SPI1CS_PORT						gpioPortD
#define INTERFACE_SPI1CS_PIN						12
#define INTERFACE_SPI1CTS_PORT						gpioPortD
#define INTERFACE_SPI1CTS_PIN						13
#define INTERFACE_SPI1RTS_PORT						gpioPortD
#define INTERFACE_SPI1RTS_PIN						14
#define INTERFACE_SPI1PWR_PORT						gpioPortD
#define INTERFACE_SPI1PWR_PIN						15


#define INTERFACE_CMUCLK_PORT						gpioPortF
#define INTERFACE_CMUCLK_PIN						3
#define INTERFACE_SPOPWR_PORT						gpioPortF
#define INTERFACE_SPOPWR_PIN						4
#define INTERFACE_EXTCS_PORT						gpioPortF
#define INTERFACE_EXTCS_PIN							5
#define INTERFACE_MICROSDAVB_PORT					gpioPortF
#define INTERFACE_MICROSDAVB_PIN					6
#define INTERFACE_GREENLED_PORT						gpioPortF
#define INTERFACE_GREENLED_PIN						5
#define INTERFACE_REDLED_PORT						gpioPortF
#define INTERFACE_REDLED_PIN						6
#define INTERFACE_PULSECOUNTER_PORT					gpioPortF
#define INTERFACE_PULSECOUNTER_PIN					7

#endif

/*===========================================================================================================
						enumerators
===========================================================================================================*/
/**
 *  @enum nterface_irq_enum
 *  @brief this enum contains the different state of the decoding state machien
 */
typedef enum {
	INTERFACE_IRQOFF,
	INTERFACE_IRQON,
} interface_irq_enum;

/**
 *  @enum interface_edgeGPIOinterrupt_enum
 *  @brief
 */
typedef enum{
	INTERFACE_FALLING_EDGE = 0x00,
	INTERFACE_RISING_EDGE,
	INTERFACE_FALLINGANDRISING_EDGE,
} interface_edgeGPIOinterrupt_enum;

/*===========================================================================================================
						structures
===========================================================================================================*/
typedef struct{
	bool interface_EM4325Miso_state;
	bool interface_EM4325Aux_state;
	bool interface_AccelerometerIRQ_state;
	bool interface_m2mIRQ_state;
	bool interface_MagnetoIRQ_state;
	bool interface_MagnetoDataReady_state;
} interface_irqstate_struct;

volatile bool gpio_IsFirstEdge;

/*===========================================================================================================
						global constant declarations
===========================================================================================================*/
extern volatile int testCPT;
/*===========================================================================================================
						Public functions declarations
===========================================================================================================*/
extern void interface_InitSpiGPIO					( const bool bSpiMaster );
extern void interface_EnableSpiGPIOAfterSleeping 	( void );
extern void interface_InitAccelerometerIrq 			( const bool bOnorOff ,  interface_edgeGPIOinterrupt_enum etype);
extern void interface_DisableSpiGPIOBeforeSleeping 	( void );
extern void interface_DisableGpioUsart1 			( void );
extern void interface_InitAuxPad 					( void );
extern void interface_EnableInputPad 				( const GPIO_Port_TypeDef port, const unsigned int pin, const bool bEnableOrDisable );
extern void interface_InitializeUnusedGpios 		( void );
extern void interface_DisableSpiGPIO 				( void );
extern void interface_DisableAuxPad 				( void );
extern void interface_ActivatePinInterrupt			( const uint8_t ui8Epin, const bool bOrOrOff);
extern void interface_EnablePinInterrupt 			( const uint8_t ui8Eport, const uint8_t ui8Epin, const uint8_t ui8Edge);
extern void interface_DisablePinInterrupt			( const uint8_t ui8Eport, const uint8_t ui8Epin);
extern void interface_InitLIS2DHIntGPIO 			( void );

void	 	GPIO_ODD_IRQHandler						( void );
void	 	GPIO_EVEN_IRQHandler					( void );



#endif

