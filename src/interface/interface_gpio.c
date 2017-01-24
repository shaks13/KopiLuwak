/*******************************************************************************
 * @file interface_gpio.c
 * @brief
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "interface_gpio.h"
#include "interface_i2c.h"


#pragma GCC optimize ("O0")

/*===========================================================================================================
						defines
===========================================================================================================*/
//#define GET_STATE()			((GPIO->P[INTERFACE_RX_PORT].DIN >> INTERFACE_RX_PIN) & 0x1)

/*===========================================================================================================
						Public variables declarations
===========================================================================================================*/

/*===========================================================================================================
						Private variables declarations
===========================================================================================================*/
static interface_irqstate_struct interface_irqstate; /* this structure contains the state of irq */



/*===========================================================================================================
						Public functions definition
===========================================================================================================*/

/**************************************************************************//**
 * @brief This function initializes the unsused GPIO pads
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void interface_InitializeUnusedGpios ( void )
{

#if (USESTARTERKIT==1)

#elif (USECROSSTAG==1)


	GPIO_PinModeSet(INTERFACE_LIS2DHINT1_PORT,INTERFACE_LIS2DHINT1_PIN, gpioModeInputPull, 0);/*Interrupt of the LIS2DH sensor*/
	GPIO_PinModeSet(INTERFACE_LIS2DHINT2_PORT,INTERFACE_LIS2DHINT2_PIN, gpioModeInputPull, 0);/*Interrupt of the LIS2DH sensor*/   /* gpioModeInputPullFilter*/
	GPIO_PinModeSet(INTERFACE_VCOMENABLE_PORT,INTERFACE_VCOMENABLE_PIN, gpioModePushPull, 0);

	GPIO_PinModeSet(INTERFACE_BUTTON2CAPA_PORT,INTERFACE_BUTTON2CAPA_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(INTERFACE_BUTTON1CAPA_PORT,INTERFACE_BUTTON1CAPA_PIN, gpioModePushPull, 0);


	GPIO_PinModeSet(INTERFACE_USART1TX_PORT,INTERFACE_USART1TX_PIN,  gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_USART1RX_PORT,INTERFACE_USART1RX_PIN,  gpioModeInputPullFilter, 0);
	GPIO_PinModeSet(INTERFACE_USART1CLK_PORT,INTERFACE_USART1CLK_PIN,  gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_USART1CS_PORT,INTERFACE_USART1CS_PIN,  gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_USART1IRQ_PORT,INTERFACE_USART1IRQ_PIN,  gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(INTERFACE_USART1EN_PORT, INTERFACE_USART1EN_PIN, 	gpioModePushPull, 0);

	GPIO_PinModeSet(INTERFACE_ADXL363_IRQ1_PORT,INTERFACE_ADXL363_IRQ1_PIN, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(INTERFACE_SENSORI2C_PORT,INTERFACE_SENSORI2C_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(INTERFACE_MAGNETODATAREADY_PORT,INTERFACE_MAGNETODATAREADY_PIN, gpioModeInputPullFilter, 0);
	GPIO_PinModeSet(INTERFACE_MAGNETOIRQ_PORT,INTERFACE_MAGNETOIRQ_PIN, gpioModeInputPullFilter, 0);
	GPIO_PinModeSet(INTERFACE_LIS2DHEN_PORT,INTERFACE_LIS2DHEN_PIN, gpioModePushPull, 0);

	GPIO_PinModeSet(INTERFACE_EFMDISPLAYEN_PORT,INTERFACE_EFMDISPLAYEN_PIN, gpioModePushPull, 0);

	GPIO_PinModeSet(INTERFACE_ADCINPUT_PORT,INTERFACE_ADCINPUT_PIN, gpioModeInputPull,0);
	GPIO_PinModeSet(INTERFACE_MAGNETICPWR_PORT,INTERFACE_MAGNETICPWR_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(INTERFACE_MAGNETICIN_PORT,INTERFACE_MAGNETICIN_PIN, gpioModeInputPull, 0);


#else
	/* I2c Bus*/
	GPIO_PinModeSet(INTERFACE_I2CSDA_PORT,INTERFACE_I2CSDA_PIN, gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_I2CSCL_PORT,INTERFACE_I2CSCL_PIN, gpioModePushPull, 1);

	GPIO_PinModeSet(INTERFACE_RSTFT232_PORT,INTERFACE_RSTFT232_PIN, gpioModePushPull, 1);/* reset of the FT232 */
	GPIO_PinModeSet(INTERFACE_LIS2DHINT_PORT,INTERFACE_LIS2DHINT_PIN, gpioModeInputPullFilter, 1);/*Interrupt of the LIS2DH sensor*/
	GPIO_PinModeSet(INTERFACE_ADXL363INT_PORT,INTERFACE_ADXL363INT_PIN, gpioModeInputPullFilter, 1);/*Interrupt of the ADXL363 sensor*/

	GPIO_PinModeSet(INTERFACE_EEPROMCS_PORT,INTERFACE_EEPROMCS_PIN, gpioModePushPull, 1);/* chip select of the EEPROM */
	GPIO_PinModeSet(INTERFACE_EEPROMHOLD_PORT,INTERFACE_EEPROMHOLD_PIN, gpioModePushPull, 1);/*hold pad of the EEPROM*/
	GPIO_PinModeSet(INTERFACE_MICROSDPWR_PORT,INTERFACE_MICROSDPWR_PIN, gpioModePushPull, 0);/*power of the �sd*/
	GPIO_PinModeSet(INTERFACE_MAGNETODATAIN_PORT,INTERFACE_MAGNETODATAIN_PIN, gpioModeInputPullFilter, 1);/*magnetometer data in*/
	GPIO_PinModeSet(INTERFACE_ADC_PORT,INTERFACE_ADC_PIN, gpioModeDisabled, 1);/*ADC input*/

	GPIO_PinModeSet(INTERFACE_SPI1TX_PORT,INTERFACE_SPI1TX_PIN, gpioModePushPull, 1);			/*TX pad of the 2nd SPI */
	GPIO_PinModeSet(INTERFACE_SPI1RX_PORT,INTERFACE_SPI1RX_PIN, gpioModeInputPullFilter, 1);	/*RX pad of the 2nd SPI */
	GPIO_PinModeSet(INTERFACE_SPI1CLK_PORT,INTERFACE_SPI1CLK_PIN, gpioModePushPull, 1);			/*CLK pad of the 2nd SPI */
	GPIO_PinModeSet(INTERFACE_SPI1CS_PORT,INTERFACE_SPI1CS_PIN, gpioModePushPull, 1);			/*CS pad of the 2nd SPI */
	GPIO_PinModeSet(INTERFACE_SPI1CTS_PORT,INTERFACE_SPI1CTS_PIN, gpioModeInputPullFilter, 1);	/*CTS pad of the 2nd SPI */
	GPIO_PinModeSet(INTERFACE_SPI1RTS_PORT,INTERFACE_SPI1RTS_PIN, gpioModePushPull, 1);			/*RTS pad of the 2nd SPI */
	GPIO_PinModeSet(INTERFACE_SPI1PWR_PORT,INTERFACE_SPI1PWR_PIN, gpioModePushPull, 0);			/*PWR pad of the 2nd SPI */

	GPIO_PinModeSet(INTERFACE_CMUCLK_PORT,INTERFACE_CMUCLK_PIN, gpioModePushPull, 0);			/*CMU clock */
	GPIO_PinModeSet(INTERFACE_SPOPWR_PORT,INTERFACE_SPOPWR_PIN, gpioModePushPull, 0);			/*  */
	GPIO_PinModeSet(INTERFACE_EXTCS_PORT,INTERFACE_EXTCS_PIN, gpioModePushPull, 1);				/* external CS  */
	GPIO_PinModeSet(INTERFACE_MICROSDAVB_PORT,INTERFACE_MICROSDAVB_PIN, gpioModePushPull, 0);	/* presence of the �sd  */
	GPIO_PinModeSet(INTERFACE_PULSECOUNTER_PORT,INTERFACE_PULSECOUNTER_PIN, gpioModeInputPullFilter, 1);	/* Pulse counter data In  */

#endif

}
/***************************************************************************//**
 * @brief Initializes the GPIO module for the serial communication.
 *
 * @param[in] 	bSpiMaster - bIsMaster true when the mcu is the master, false for the slave
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
extern void interface_InitSpiGPIO (const bool bSpiMaster)
{
	CMU_ClockEnable(cmuClock_GPIO, true);

	if (bSpiMaster == true)
	{
		/* GPIO configuration for the master config*/
		GPIO_PinModeSet(INTERFACE_MOSI_PORT,INTERFACE_MOSI_PIN, gpioModePushPull, 0);  	/* MOSI */
		GPIO_PinModeSet(INTERFACE_MISO_PORT,INTERFACE_MISO_PIN, gpioModeInput, 	  1);  	/* MISO + filter*/
		GPIO_PinModeSet(INTERFACE_CS_PORT, 	INTERFACE_CS_PIN, 	gpioModePushPull, 1);  	/* CS */
		GPIO_PinModeSet(INTERFACE_CLK_PORT, INTERFACE_CLK_PIN, 	gpioModePushPull, 0);  	/* Clock */

		GPIO_PinModeSet(INTERFACE_AUX_PORT, INTERFACE_AUX_PIN, 	gpioModeInputPull, 0);  	/* aux pad */

		/* Enable GPIO interrupts in the NVIC interrupt controller */
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		/* because this interrupt handler 'll call a FreeRtos function*/
		NVIC_EnableIRQ(GPIO_ODD_IRQn);
	}
	else
	{
		/* GPIO configuration for the slave config*/
		GPIO_PinModeSet(INTERFACE_MOSI_PORT, INTERFACE_MOSI_PIN, gpioModeInput, 0);  		/* MOSI */
		GPIO_PinModeSet(INTERFACE_MISO_PORT, INTERFACE_MISO_PIN, gpioModePushPull, 0);  	/* MISO */
		GPIO_PinModeSet(INTERFACE_CS_PORT, INTERFACE_CS_PIN, gpioModeInput,   0);  			/* CS */
		GPIO_PinModeSet(INTERFACE_CLK_PORT, INTERFACE_CLK_PIN, gpioModeInput,  0);  		/* Clock */
	}

	interface_irqstate.interface_EM4325Miso_state = INTERFACE_IRQOFF;
	interface_irqstate.interface_EM4325Aux_state = INTERFACE_IRQOFF;
}

/***************************************************************************//**
 * @brief Initializes the interuption GPIO used by the 2nd SPI bus
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
extern void interface_InitLIS2DHIntGPIO ( void )
{
	/* GPIO configuration for the master config*/
	GPIO_PinModeSet(INTERFACE_LIS2DHINT1_PORT,INTERFACE_LIS2DHINT1_PIN, gpioModeInputPullFilter, 0);
	interface_EnablePinInterrupt (INTERFACE_LIS2DHINT1_PORT, INTERFACE_LIS2DHINT1_PIN , INTERFACE_FALLING_EDGE); /* enable the IRQ */
}

/***************************************************************************//**
 * @brief 		Initializes the interruption GPIO used by the accelerometer
 * @param[in] 	bOnorOff : true to enable the irq, false to disable
 * @param[in] 	etype : slect the rising, falling or both edges
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_InitAccelerometerIrq ( const bool bOnorOff , interface_edgeGPIOinterrupt_enum etype )
{
	if (true == bOnorOff )
	{
		/* GPIO configuration for the master config*/
		GPIO_PinModeSet(INTERFACE_ADXL363_IRQ1_PORT,INTERFACE_ADXL363_IRQ1_PIN, gpioModeInput, 0);
		interface_EnablePinInterrupt (INTERFACE_ADXL363_IRQ1_PORT, INTERFACE_ADXL363_IRQ1_PIN , etype); /* enable the IRQ */
	}
	else
	{
		interface_DisablePinInterrupt (INTERFACE_ADXL363_IRQ1_PORT, INTERFACE_ADXL363_IRQ1_PIN) ;
	}
}


/***************************************************************************//**
 * @brief
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_EnableSpiGPIOAfterSleeping ( void )
{
	/* GPIO configuration for EM4325 SPI */
	//GPIO_PinModeSet(INTERFACE_MOSI_PORT,INTERFACE_MOSI_PIN, gpioModePushPull, 0);  	/* MOSI */
	//GPIO_PinModeSet(INTERFACE_CS_PORT, 	INTERFACE_CS_PIN, 	gpioModePushPull, 1);  	/* CS */
	GPIO_PinModeSet(INTERFACE_MISO_PORT, INTERFACE_MISO_PIN, gpioModeInput, 0);  	/* MISO */
	GPIO_PinModeSet(INTERFACE_AUX_PORT,INTERFACE_AUX_PIN, gpioModeInput, 1);
	//GPIO_PinModeSet(INTERFACE_CLK_PORT, INTERFACE_CLK_PIN, 	gpioModePushPull, 1);  	/* Clock */
}
/***************************************************************************//**
 * @brief this function disables the GPIO of the SPI bus except the MISO pin
 * that is used as wake-up IRQ
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_DisableSpiGPIOBeforeSleeping ( void )
{
	/* GPIO configuration for EM4325 SPI */
	//GPIO_PinModeSet(INTERFACE_MOSI_PORT,INTERFACE_MOSI_PIN, gpioModeDisabled, 0);  	/* MOSI */
	//GPIO_PinModeSet(INTERFACE_CS_PORT, 	INTERFACE_CS_PIN, 	gpioModePushPull, 1);  	/* CS */
	GPIO_PinModeSet(INTERFACE_MISO_PORT, INTERFACE_MISO_PIN, gpioModeInputPullFilter, 0);  	/* MISO */
	GPIO_PinModeSet(INTERFACE_AUX_PORT, INTERFACE_AUX_PIN, gpioModeInputPullFilter, 0);
	//GPIO_PinModeSet(INTERFACE_CLK_PORT, INTERFACE_CLK_PIN, 	gpioModePushPull, 1);  	/* Clock */

}

/***************************************************************************//**
 * @brief this function disables the GPIO of the SPI bus except the MISO pin
 * that is used as wake-up IRQ
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_DisableGpioUsart1 (void)
{
	GPIO_PinModeSet(INTERFACE_USART1TX_PORT,INTERFACE_USART1TX_PIN, 	gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_USART1RX_PORT, 	INTERFACE_USART1RX_PIN, gpioModeInputPullFilter, 1); /* when disabled add a pull donw on RX*/
	GPIO_PinModeSet(INTERFACE_USART1CLK_PORT, INTERFACE_USART1CLK_PIN, gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_USART1CS_PORT, INTERFACE_USART1CS_PIN, 	gpioModePushPull, 1);
	GPIO_PinModeSet(INTERFACE_USART1EN_PORT, INTERFACE_USART1EN_PIN, 	gpioModePushPull, 0);
	GPIO_PinModeSet(INTERFACE_USART1IRQ_PORT, INTERFACE_USART1IRQ_PIN,	gpioModeInputPullFilter, 1);
}

/***************************************************************************//**
 * @brief this function disables  the Aux pad
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/

void interface_DisableAuxPad ( void )
{
#ifdef USESTARTERKIT
	GPIO_PinModeSet(INTERFACE_AUX_PORT, INTERFACE_AUX_PIN, 	gpioModeDisabled, 0);  	/* AUX */
#else
	GPIO_PinModeSet(INTERFACE_AUX_PORT, INTERFACE_AUX_PIN, 	  gpioModeInputPullFilter, 0);  	/* AUX */
#endif
}


/***************************************************************************//**
 * @brief this function enables or disables an pad as an input
 * @param[in] 	port The GPIO port to access.
 * @param[in] 	pin The pin number in the port.
 * @param[in] 	pin The pin number in the port.
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_EnableInputPad ( const GPIO_Port_TypeDef port, const unsigned int pin, const bool bEnableOrDisable )
{
	if (true == bEnableOrDisable)
	{
		GPIO_PinModeSet(port, pin, 	gpioModeInputPullFilter, 1);
	}
	else
	{
		GPIO_PinModeSet(port, pin, 	gpioModeInputPullFilter, 0);
	}

}

/***************************************************************************//**
 * @brief 		this function initializes the Aux pad as input
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_InitAuxPad ( void )
{
	GPIO_PinModeSet(INTERFACE_AUX_PORT,INTERFACE_AUX_PIN, gpioModeInput, 1);
}


/***************************************************************************//**
 * @brief this function initializes the Irq of the GPIO interface
 *
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void interface_InitIrq ( void )
{
	memset (&interface_irqstate,0x00, sizeof (interface_irqstate_struct));
}
/**************************************************************************//**
 * @brief This function enables one GPIO irq on a falling/rising edge
 * event
 * @param[in] ui8Eport The GPIO port
 * @param[in] ui8Epin The GPIO pin
 * @param[in] ui8Edge 2: rising/falling edge, 1: rising edge, 0: falling edge
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
extern void interface_EnablePinInterrupt (const uint8_t ui8Eport, const uint8_t ui8Epin, const uint8_t ui8Edge)
{
	/* Configure interrupt on rising edge */
	if(ui8Edge == INTERFACE_FALLING_EDGE)
	{
		GPIO_IntConfig(ui8Eport, ui8Epin, false, true, true);
	}
	/* Configure interrupt on falling edge */
	else if (ui8Edge == INTERFACE_RISING_EDGE)
	{
		GPIO_IntConfig(ui8Eport, ui8Epin, true, false, true);
	}
	else if (ui8Edge == INTERFACE_FALLINGANDRISING_EDGE)
	{
		GPIO_IntConfig(ui8Eport, ui8Epin, true, true, true);
	}
	else
	{
		/* should not happen */
	}


	if ((ui8Eport == INTERFACE_AUX_PORT) && (ui8Epin == INTERFACE_AUX_PIN))
	{
		interface_irqstate.interface_EM4325Aux_state = INTERFACE_IRQON;
	}
	else if ((ui8Eport == INTERFACE_MISO_PORT) && (ui8Epin == INTERFACE_MISO_PIN))
	{
		interface_irqstate.interface_EM4325Miso_state = INTERFACE_IRQON;
	}
	else if ((ui8Eport == INTERFACE_ADXL363_IRQ1_PORT) && (ui8Epin == INTERFACE_ADXL363_IRQ1_PIN))
	{
		interface_irqstate.interface_AccelerometerIRQ_state = INTERFACE_IRQON;
	}
	else if ((ui8Eport == INTERFACELEUART_RX_PORT) && (ui8Epin == INTERFACELEUART_RX_PIN))
	{
		interface_irqstate.interface_m2mIRQ_state = INTERFACE_IRQON;
	}
	else if ((ui8Eport == INTERFACE_MAGNETODATAREADY_PORT) && (ui8Epin == INTERFACE_MAGNETODATAREADY_PIN))
	{
		interface_irqstate.interface_MagnetoDataReady_state = INTERFACE_IRQON;
	}
	else if ((ui8Eport == INTERFACE_MAGNETOIRQ_PORT) && (ui8Epin == INTERFACE_MAGNETOIRQ_PIN))
	{
		interface_irqstate.interface_MagnetoIRQ_state = INTERFACE_IRQON;
	}
	else { /* do nothing*/}

	if (0 != (ui8Epin %2 ))
	{
		NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		NVIC_EnableIRQ(GPIO_ODD_IRQn);
	}
	else
	{
		NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
		NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	}
}
/**************************************************************************//**
 * @brief		This function disables the one GPIO irq
 * @param[in] 	ui8Eport The GPIO port
 * @param[in] 	ui8Epin The GPIO pin
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
extern void interface_ActivatePinInterrupt (const uint8_t ui8Epin, const bool bOrOrOff)
{
	if ( true == bOrOrOff)
	{
		GPIO_IntClear (1<<ui8Epin);
		GPIO_IntEnable (1<<ui8Epin);
	}
	else
	{
		GPIO_IntDisable (1<<ui8Epin);
	}

}

/**************************************************************************//**
 * @brief		This function disables the one GPIO irq
 * @param[in] 	ui8Eport The GPIO port
 * @param[in] 	ui8Epin The GPIO pin
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
extern void interface_DisablePinInterrupt (const uint8_t ui8Eport, const uint8_t ui8Epin)
{
	GPIO_IntConfig(ui8Eport, ui8Epin, false, false, false);

	if ((ui8Eport == INTERFACE_AUX_PORT) && (ui8Epin == INTERFACE_AUX_PIN))
	{
		interface_irqstate.interface_EM4325Aux_state = INTERFACE_IRQOFF;
	}
	else if ((ui8Eport == INTERFACE_MISO_PORT) && (ui8Epin == INTERFACE_MISO_PIN))
	{
		interface_irqstate.interface_EM4325Miso_state = INTERFACE_IRQOFF;
	}
	else if ((ui8Eport == INTERFACE_ADXL363_IRQ1_PORT) && (ui8Epin == INTERFACE_ADXL363_IRQ1_PIN))
	{
		interface_irqstate.interface_AccelerometerIRQ_state = INTERFACE_IRQOFF;
	}
	else if ((ui8Eport == INTERFACELEUART_RX_PORT) && (ui8Epin == INTERFACELEUART_RX_PIN))
	{
		interface_irqstate.interface_m2mIRQ_state = INTERFACE_IRQOFF;
	}
	else if ((ui8Eport == INTERFACE_MAGNETODATAREADY_PORT) && (ui8Epin == INTERFACE_MAGNETODATAREADY_PIN))
	{
		interface_irqstate.interface_MagnetoDataReady_state = INTERFACE_IRQOFF;
	}
	else if ((ui8Eport == INTERFACE_MAGNETOIRQ_PORT) && (ui8Epin == INTERFACE_MAGNETOIRQ_PIN))
	{
		interface_irqstate.interface_MagnetoIRQ_state = INTERFACE_IRQOFF;
	}
	else { /* do nothing*/}
}

/*===========================================================================================================
						Interruptions definition
===========================================================================================================*/

/**************************************************************************//**
 * @brief 		GPIO_EVEN_IRQHandler
 * 				Interrupt Service Routine Even GPIO Interrupt Line
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
uint32_t wIntflags = 0;

BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* We have not woken a task at the start of the ISR.*/
Kernel_QueueItem_struct pAccelleroQueueItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
												KERNEL_MESSAGEID_ACCELEROMETERIRQ, 	/* the message is that an RF event has been triggered */
												0,NULL};
Kernel_QueueItem_struct pQueueMagnetoItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
												KERNEL_MESSAGEID_DATAREADYIRQ,
											0,NULL};
Kernel_QueueItem_struct pQueueM2MItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_KERNELTASKID,KERNEL_KERNELTASKID),
											KERNEL_STAYWAKEUP,
											0,NULL};

	wIntflags = GPIO_IntGet() ;
	GPIO_IntClear(wIntflags);
	if (0x0000 != (wIntflags & (1 << INTERFACE_ADXL363_IRQ1_PIN) ) ) 				/* Interruption from the accelerometer  */
	{
		GPIO_IntClear(1 << INTERFACE_ADXL363_IRQ1_PIN);
		if (INTERFACE_IRQON == interface_irqstate.interface_AccelerometerIRQ_state ) /* when the IRQ of the accelerometer has been enabled*/
		{
			xQueueSendFromISR (sKernel_QueuePointer.pSensorQueue,&pAccelleroQueueItem, &xHigherPriorityTaskWoken);

			GPIO_PinOutToggle(INTERFACE_USART1TX_PORT,INTERFACE_USART1TX_PIN);
		}
	}
	else if (0x0000 != (wIntflags & (1 << INTERFACE_MAGNETODATAREADY_PIN) ) ) 				/* Interruption from the magnetometer  */
	{
		GPIO_IntClear(1 << INTERFACE_MAGNETODATAREADY_PIN);
		xQueueSendFromISR (sKernel_QueuePointer.pSensorQueue,&pQueueMagnetoItem, &xHigherPriorityTaskWoken);
	}
	else if(0x0000 != (wIntflags & (1 << INTERFACELEUART_RX_PIN))  ) /* Interruption from USART RX */
	{
		GPIO_IntClear(1 << INTERFACELEUART_RX_PIN);
		interface_DisablePinInterrupt (INTERFACELEUART_RX_PORT,INTERFACELEUART_RX_PIN);
		xQueueSendFromISR (sKernel_QueuePointer.pKernelQueue,&pQueueM2MItem, &xHigherPriorityTaskWoken);
	}
	else {/* do nothing */}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD ();
	}

}

/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler
 * Interrupt Service Routine Odd GPIO Interrupt Line
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 *****************************************************************************/
void __attribute__((optimize("O0")))  GPIO_ODD_IRQHandler(void)
{
uint32_t wIntflags = 0;
Kernel_QueueItem_struct pCommBufferQueueItem = {KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_RFFRONTENDTASKID),
												KERNEL_MESSAGEID_COMMBUFFERSEMAPHORE, 	/* the message is that an RF event has been triggered */
												0,NULL};
Kernel_QueueItem_struct pQueueMagnetoItem = {	KERNEL_CREATE_RECANDSEND(KERNEL_SENSORTASKID,KERNEL_SENSORTASKID),
											KERNEL_MESSAGEID_MAGNETOIRQ,
											0,NULL};
#if 0 /* not used anymore*/
Kernel_QueueItem_struct pStartQueueItem = 		{(((KERNEL_SENSORTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | /* the receiver is the Serial task */
												((KERNEL_SENSORTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,	/* the sender is the Serial task */
												KERNEL_MESSAGEID_RFFIELDDETECTED, 	/* the message is that an RF event has been triggered */
												0,NULL};
#endif
BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* We have not woken a task at the start of the ISR.*/

	wIntflags = GPIO_IntGet() ;

	if(0 != (wIntflags & (1 << INTERFACE_MISO_PIN) ))									/* on EM4325's MISO  pad */
	{
		if (INTERFACE_IRQON == interface_irqstate.interface_EM4325Miso_state )
		{
			//interface_DisablePinInterrupt (INTERFACE_MISO_PORT, INTERFACE_MISO_PIN); /* avoid the IRQ during the SPI com*/
			GPIO_IntClear(1 << INTERFACE_MISO_PIN);
			interface_ActivatePinInterrupt (INTERFACE_MISO_PIN , false );
			if ( SRVEM4325_SPIMASTER == srvEM4325_sConfiguration.eMode )
			{
				interface_bEm4325SpiDataReady = true; /* the EM4325 sent the signal that the response is ready*/
			}
			else if ( SRVEM4325_SIGNALING_COMM_BUFFER == srvEM4325_sConfiguration.eMode )
			{
				/* Sends a message to the RF task */
				xQueueSendFromISR (sKernel_QueuePointer.pRFtaskQueue,&pCommBufferQueueItem, &xHigherPriorityTaskWoken);
			}
			else{/* do nothing*/}
		}
	}
	else if(0 != (wIntflags & (1 << INTERFACE_AUX_PIN)))
	{

#if 0 /* not used yet*/
		/* ---------- the EM4325 behaves as a mix between AFE and tag ------------------------------------------------------ */
		else if ( SRVEM4325_RF_MODEM_SHARED_LIMITED == srvEM4325_sConfiguration.eMode )
		{
			/* Interruption on MISO PC1 */
			if (interface_irqstate.interface_EM4325Miso_state == INTERFACE_IRQON)
			{
				/* saves the BLF counter concerning the previous symbol */
				if (false == gpio_IsFirstEdge)
				{
					/* save the counter */
					auClockCounterPerSymbol[uNbSymbol++] = (uint8_t) TIMER1->CNT;
					/* and reset the counters*/
					TIMER0->CNT = 0;
					TIMER1->CNT = 0;
				}
				else				/* BLF counter clear to count the number of BLF for the delimiter */
				{
					/* reset the counters on the falling edge*/
					TIMER0->CNT = 0;
					TIMER1->CNT = 0;
					/* restarts the timers */
					//TIMER0->CMD = TIMER_CMD_START;
					TIMER1->CMD = TIMER_CMD_START;

					/* Todo: optimization decreasing the IRQ processing time to obtain the delimiter (beginning) */
					GPIO->EXTIPSELH = (GPIO->EXTIPSELH & ~(0xF << (4 * INTERFACE_RX_PIN))) | (INTERFACE_RX_PORT << (4 * INTERFACE_RX_PIN));
					//BITBAND_Peripheral(&(GPIO->EXTIRISE), INTERFACE_RX_PIN, (unsigned int)true); /* activate the rising edge IRQ on the RX */
					BITBAND_Peripheral(&(GPIO->EXTIFALL), INTERFACE_RX_PIN, (unsigned int)false);  /* disable the falling edge IRQ on the RX */
					//GPIO->IFC = 1 << INTERFACE_RX_PIN;
					//BITBAND_Peripheral(&(GPIO->IEN), INTERFACE_RX_PIN, (unsigned int)true);
					interface_irqstate.interface_EM4325Miso_state = INTERFACE_IRQON;
					/* Todo: optimization (end) */

					gpio_IsFirstEdge = false;
				}
			}
		}
#endif
	}
#if 0
	else if(0x0000 != (wIntflags & (1 << INTERFACE_LIS2DHINT1_PIN) ) ) /* Interruption from the LIS2DH */
	{
		if (INTERFACE_IRQON == interface_irqstate.interface_AccelerometerIRQ_state ) /* when the IRQ of the accelerometer has been enabled*/
		{
			#if 0
						xQueueSendFromISR (sKernel_QueuePointer.pSensorQueue,&pQueueItem, &xHigherPriorityTaskWoken);
			#elif 0

						srvLIS2DH12_GetGz ();
			#elif 0
						xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
			#else

			#endif
		}
	}
#endif
	else if (0x0000 != (wIntflags & (1 << INTERFACE_MAGNETOIRQ_PIN) ) ) 				/* Interruption from the magnetometer  */
	{
		GPIO_IntClear(1 << INTERFACE_MAGNETOIRQ_PIN);
		xQueueSendFromISR (sKernel_QueuePointer.pSensorQueue,&pQueueMagnetoItem, &xHigherPriorityTaskWoken);
	}

	else
	{		/* Do nothing */	}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD ();
	}

}

