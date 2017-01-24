# Overview
this code is dedicated to the pearl starter kit (Silicon labs) and a daughter card which have a EPC gen2 front end (EM4325), a magnetic sensor (LIS3), an EEPROM and a accelorometer.
the embedded OS is freertos and 4 tasks are uses :
	- the RF task is the interface with the RF front end EM4325
	- the serial task is the interface with the UART
	- the sensor task is the interface with the sensors ( accelerometer and magnetometer)
	- the kernel task is the interface with the others tasks

# Hardware

the uc platform is a pearl starter kit from Silicon lab connectedto a daughter baord.

the peal uc have 2 USART peripherals and should be shared with these 3 interfaces UART for the seria communication, the SPI of the EM4325 and the SPI of the accelerometer (ADXl)

| Pearl starter kit	| daughter	  	| Direction |
|:-----------------:|:-------------:|:---------:|
| 		PA0    		| US0 CLK(loc30)|    OUT    |
| 		PA1    		| US0 CS (loc30)|    OUT    |
| 		PA2    		| I2C PWR     	|    OUT    |
| 		PA3    		| US0 TX      	|    OUT    |
| 		PA4    		| US0 RX      	|    IN     |
| 		PC6    		| US1 TX(loc11) |    OUT    |
| 		PC7    		| US1 RX(loc11) |    IN     |
| 		PC8    		| US1 CLK(loc11)|    OUT    |
| 		PC9    		| US1 CS(loc11) |    OUT    |
| 		PC10   		| US1 SDA(loc15)|    IN/OUT |
| 		PC11   		| US1 SCL(loc15)|    OUT    |
| 		PD10   		| LIS3 dataready|    IN	    |
| 		PD11   		| LIS3 irq		|    IN	    |
| 		PD12   		| greeen led	|    OUT	|
| 		PD13   		| orange led	|    OUT	|
| 		PD14   		| adxl IRQ		|    IN		|
| 		PF6   		| US0 TX (loc30)|    IN		|
| 		FF7   		| adxlIRQ(loc30)|    IN		|

#OS

for the application that mesures periodically the temperature of the EM4325 it may have at the same time two access to the EM435, one for the temperature measurement triggered by a internal timer and and an other one for the RF front end access triggered by an IRQ. Thus a semaphore protects the access to the EM4325 (xBinarySemphrEm4325)


#configuration

