 /****************************************************************************************
 * @file LIS2DH12
 * @brief Header file of LIS2DH12 Driver
 * @autor
*****************************************************************************************
*****************************************************************************************
 * THIS SOFWARE IS NOT CREATED BY STMICROELECTRONICS
 * THE WARRANTY IS NOT INCLUDED WITH THIS SOFTWARE
 *
******************************************************************************************
 *
 * */

#ifndef PROTOCOL_LIS2DH12_H_
#define PROTOCOL_LIS2DH12_H_


/* Librairies */

#include "common_library.h"
#include "arm_math.h"
//#include "utils.h"
#include "interface_spi.h"
#include "interface_i2c.h"
#include "interface_gpio.h"

/* buffer size */
#define PROTOCOL_LIS2DH12_BUFFERSIZE       					16
#define PROTOCOL_LIS2DH12_NBCMDBYTE      					1

/* Pin configuration */
#define INTERFACE_LIS2DH12INT2_PORT							INTERFACE_SPI2INT2_PORT
#define INTERFACE_LIS2DH12INT2_PIN							INTERFACE_SPI2INT2_PIN
#define INTERFACE_LIS2DH12MISO_PORT							INTERFACE_SPI2MISO_PORT
#define INTERFACE_LIS2DH12MISO_PIN							INTERFACE_SPI2MISO_PIN
#define INTERFACE_LIS2DH12MOSI_PORT							INTERFACE_SPI2MOSI_PORT
#define INTERFACE_LIS2DH12MOSI_PIN							INTERFACE_SPI2MOSI_PIN
#define INTERFACE_LIS2DH12CLK_PORT							INTERFACE_SPI2CLK_PORT
#define INTERFACE_LIS2DH12CLK_PIN							INTERFACE_SPI2CLK_PIN
#define INTERFACE_LIS2DH12CS_PORT							INTERFACE_SPI2CS_PORT
#define INTERFACE_LIS2DH12CS_PIN							INTERFACE_SPI2CS_PIN

/* Options for communicating with the device. */
#define LIS2DH12_SPI_COMM       							1
#define LIS2DH12_I2C_COMM       							0

/* SAO Configuration */
#define SAO_CONFIG											0

/* SPI slave device ID */
#define LIS2DH12_SLAVE_ID      							 	1
#define LIS2DH12_ID											0x33
#define LIS2DH12_ADDRESS									(0x30)

#define PROTOCOL_LIS2DH12_NBBYTE_READ						1
#define PROTOCOL_LIS2DH12_NBBYTE_WRITE						1

/* LIS2DH12 I2C Communication */

#define LIS2DH12_SA00_I2C_READ								0x31
#define LIS2DH12_SA00_I2C_WRITE								0x32
#define LIS2DH12_SA01_I2C_READ								0x33
#define LIS2DH12_SA01_I2C_WRITE								0x34

/* LIS2DH12 SPI communication */

#define LIS2DH12_SPI_READ        							(1 << 7)
#define LIS2DH12_SPI_WRITE       							(0 << 7)
#define LIS2DH12_SPI_MB          							(1 << 6)		/* multiple bytes to read or write */
#define LIS2DH12_SPI_SINGLEBYTE     						(0 << 6)			/* single byte to read or write */


#define LIS2DH12_SCALE_FACTOR			1000/(1024*32)

/* #####################################################
 * LIS2DH12 Configuration
 * VALUES JUST FOR ACTIVITY COUNTER PROJECT
 * SEE STMICROELECTRONICS DATASHEET FOR OTHER SOLUTIONS
 * #####################################################*/

/* For TEMP_CFG_REG */
#define LIS2DH12_TEMPERATURE_ENABLE 		0xC0
#define LIS2DH12_TEMPERATURE_DISABLE		0x00
/* For CTRL_REG1 */
#define LIS2DH12_REG1_1HZ					0x14
#define LIS2DH12_REG1_10HZ					0x24
#define LIS2DH12_REG1_25HZ					0x34
#define LIS2DH12_REG1_50HZ					0x44
#define LIS2DH12_REG1_100HZ					0x54
#define LIS2DH12_REG1_200HZ					0x64
#define LIS2DH12_REG1_400HZ					0x74
#define LIS2DH12_REG1_1620HZ				0x84
#define LIS2DH12_REG1_5376HZ				0x94
/* For CTRL_REG2 */
#define LIS2DH12_REG2_ENABLEHF				0x38
/* For CTRL_REG3 */
#define LIS2DH12_REG3_AOI1					0x30
#define LIS2DH12_REG3_DATAREADY				0x10
/* For CTRL_REG4 */
#define LIS2DH12_REG4_16G					0xB0
#define LIS2DH12_REG4_8G					0xA0
#define LIS2DH12_REG4_4G					0x90
#define LIS2DH12_REG4_2G					0x80
/* For CTRL_REG5 */
#define LIS2DH12_REG5_ENABLEFIFO			0x40


/* ###############################################################
 * LIS2DH12 Register Map
 * SEE STMICROELECTRONICS DATASHEET
 * ###############################################################*/

#define LIS2DH12_STATUS_REG_AUX 		0x07 //
#define LIS2DH12_OUT_TEMP_L 			0x0C
#define LIS2DH12_OUT_TEMP_H				0x0D
#define LIS2DH12_INT_COUNTER_REG		0x0E
#define LIS2DH12_WHO_AM_I				0x0F
#define LIS2DH12_TEMP_CFG_REG			0x1F
#define LIS2DH12_CTRL_REG1				0x20
#define LIS2DH12_CTRL_REG2				0x21
#define LIS2DH12_CTRL_REG3				0x22
#define LIS2DH12_CTRL_REG4				0x23
#define LIS2DH12_CTRL_REG5				0x24
#define LIS2DH12_CTRL_REG6				0x25
#define LIS2DH12_REFERENCE_DATACAPTURE 	0x26
#define LIS2DH12_STATUS_REG				0x27
#define LIS2DH12_OUT_X_L				0x28
#define LIS2DH12_OUT_X_H				0x29
#define LIS2DH12_OUT_Y_L				0x2A
#define LIS2DH12_OUT_Y_H				0x2B
#define LIS2DH12_OUT_Z_L				0x2C
#define LIS2DH12_OUT_Z_H				0x2D
#define LIS2DH12_FIFO_CTRL_REG			0x2E
#define LIS2DH12_FIFO_SRC_REG			0x2F
#define LIS2DH12_INT1_CFG				0x30
#define LIS2DH12_INT1_SRC				0x31
#define LIS2DH12_INT1_THS				0x32
#define LIS2DH12_INT1_DURATION			0x33
#define LIS2DH12_INT2_CFG				0x34
#define LIS2DH12_INT2_SRC				0x35
#define LIS2DH12_INT2_THS				0x36
#define LIS2DH12_INT2_DURATION			0x37
#define LIS2DH12_CLICK_CFG				0x38
#define LIS2DH12_CLICK_SRC				0x39
#define LIS2DH12_CLICK_THS				0x3A
#define LIS2DH12_TIME_LIMIT				0x3B
#define LIS2DH12_TIME_LATENCY			0x3C
#define LIS2DH12_TIME_WINDOW			0x3D
#define LIS2DH12_Act_THS				0x3E
#define LIS2DH12_Act_DUR				0x3F

#define LIS2DH12_SIZE_DATA_BUFFER 	3

/*****************************************************************************
 ********************* Structure & Union Declaration *************************
 ******************************************************************************/

typedef struct
{
	short int x : 16;
	short int y : 16;
	short int z : 16;
}LIS2DH12_AxisBitField;

typedef union
{
	uint8_t aui8Axis[LIS2DH12_SIZE_DATA_BUFFER*2];
	uint16_t aui16Axis[LIS2DH12_SIZE_DATA_BUFFER];
	LIS2DH12_AxisBitField axis;
}LIS2DH12_AxisData_Union;

/******************************************************************************/
/*************************** Global variables *********************************/
/******************************************************************************/

/* Axis data buffer */
extern LIS2DH12_AxisData_Union 	LIS2DH12_uAxisData;


/*****************************************************************************
 *************************** Function Declaration ****************************
 ******************************************************************************/
uint8_t	prtLIS2DH_Init 					( void );
void 	prtLIS2DH_Deinit 				( void );
/* Read a value of register */
uint8_t prtLIS2DH_GetRegisterValue		( const uint8_t ui8registerAddress,const uint8_t ui8NbByteToRead, uint8_t *ui8registerValue);

/* Send value of register */
void 	prtLIS2DH_SetRegisterValue		( unsigned char registerAddress,unsigned char registerValue );

/* Init of the sensor */
uint8_t LIS2DH12_Configure 				( void );


/* Obtain axis */;
void 	prtLIS2DH_GetGxyz				( float32_t* x,float32_t* y,float32_t* z );
void 	prtLIS2DH_GetGz					( float* z );
void 	prtLIS2DH_GetGzloop 			( uint16_t ui16nbloop);

/* Obtain temperature of sensor */
void 	prtLIS2DH_GetTemp 				( short* temp );

/*! Saves the raw output data of each axis into the Axis Data buffer.*/
uint8_t prtLIS2DH_SaveAxisData 			( void );

/*! Releases the Axis Data buffer.*/
void 	prtLIS2DH_ReleaseAxixDataBuffer	( void );
void 	prtLIS2DH_Plugbuffer 			( void );
void 	prtLIS2DH_DisableGPIOInt 		( void );
void 	prtLIS2DH_EnableGPIOInt 		( void );
void 	prtLIS2DH_CleanBuffer 			( void );


#endif /* PROTOCOL_LIS2DH12_H_ */


