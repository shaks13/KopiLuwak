/*******************************************************************************
 * @file srvM2M.c
 * @brief this file is the service layer for the M2M communication
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvM2M.h"

#if USEM2MINTERFACE == 1 /* these IRQ handlers is dedicated to the M2M communication*/
/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
//static uint8_t aui8serialcommmand [SRVM2M_COMMANDLENGTH];
/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/

/*===========================================================================================================
						Private function declaration
===========================================================================================================*/

//static void 	srvm2m_ProcessGetAccelerometerState ( void );
/*===========================================================================================================
						Private functions definition
===========================================================================================================*/


#if 0
/***************************************************************************//**
 * @brief 		This function process to the reception of the data
 * @param[in] 	pui8Rxdata : pointer of the RX message
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void srvm2m_ProcessGetAccelerometerState ( void )
{

	if (true == srvadxl363_IsEnable ())
	{
		prtm2m_SendString("on\n");
	}
	else
	{
		prtm2m_SendString("off\n");
	}
}
#endif
/*===========================================================================================================
						Public functions definition
===========================================================================================================*/

/***************************************************************************//**
 * @brief 		This function initializes the M2M interface
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvm2m_init ( void )
{
	prtm2m_init ();
}

#if 0
/***************************************************************************//**
 * @brief 		This function process to the reception of the data
 * @detail 		the exchanged data between the tasks follow this protocol.
 * 	the first element of pData is the operation Get or Set (Read or write)
 * 	the second element is about the feature, continuous measurement mode or activity
 * 	counter ...
 * 	the third element is the action on or off or ...
 * 	the notification and the 2nd element of pData is the same information
 * @param[out] 	prtM2M_SerialCommand : pointer of the ID of the different field
 * of the serial command
 * @param[out] 	pQueueItems : pointer on the queue object.
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted to an
 * another task
 ******************************************************************************/
uint8_t srvm2m_ProcessAccelerometerOp ( prtM2M_SerialCommand_Type *prtM2M_SerialCommand , Kernel_QueueItem_struct *pQueueItems  )
{
	uint8_t ui8status = CROSSRFID_SUCCESSCODE;


	pQueueItems->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SENSORTASKID,KERNEL_SERIALTASKID);
	pQueueItems->ui16NbByte = 3;
	pQueueItems->ui16notification = KERNEL_MESSAGEID_MEASURE;	/* the notification and the feature are the same information */

	switch (prtM2M_SerialCommand->ui8CommandId)
	{
		case PRTM2M_COMMANDCODE_GET: 	/* the M2M host request a get operation*/

			switch (prtM2M_SerialCommand->ui8ActionId)
			{
				case PRTM2M_ACTIONCODE_STATE:		/* the M2M host request to known the state of the accelerometer*/

					pQueueItems->pData[0] = SRVM2M_COMMANDCODE_GET;
					pQueueItems->pData[1] = SRVM2M_FEATURE_CONTINUOUS;
					pQueueItems->pData[2] = SRVM2M_ACTIONID_STATE;
					ui8status = CROSSRFID_MESSAGETOBEPOSTED;
					prtm2m_SendString("msg sent\n");
				break;
				default :
					prtm2m_SendString("execution error\n");
				break;
			}

		break;

		case PRTM2M_COMMANDCODE_SET : /*request an action of the accelerometer*/
			switch (prtM2M_SerialCommand->ui8ActionId)
			{
				case PRTM2M_ACTIONCODE_ON:
					pQueueItems->pData[0] = SRVM2M_COMMANDCODE_SET;
					pQueueItems->pData[1] = SRVM2M_FEATURE_CONTINUOUS;
					pQueueItems->pData[2] = SRVM2M_ACTIONID_ON;
					ui8status = CROSSRFID_MESSAGETOBEPOSTED;
					prtm2m_SendString("msg sent\n");
				break;

				case PRTM2M_ACTIONCODE_OFF:
					pQueueItems->pData[0] = SRVM2M_COMMANDCODE_SET;
					pQueueItems->pData[1] = SRVM2M_FEATURE_CONTINUOUS;
					pQueueItems->pData[2] = SRVM2M_ACTIONID_OFF;
					ui8status = CROSSRFID_MESSAGETOBEPOSTED;
					prtm2m_SendString("msg sent\n");
				break;

				default :
					prtm2m_SendString("execution error\n");
				break;
			}
		break;

		default :
			/*should not happen*/
		break;

	}

	return ui8status;
}

#endif


/***************************************************************************//**
 * @brief 		This function process to the reception of the data
 * @note		the frame format is the next one
 * 	|---------------|---------------|-----------------------|
 * 	| Command code	|  feature		| action 				|
 * 	|				|				|						|
 * 	|---------------|---------------|-----------------------|
 * 	| set			| 				|						|
 * 	| get			| 				| 						|
 * 	|---------------|---------------|-----------------------|
 * @param[in] 	pui8Rxdata : pointer of the RX message
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
uint8_t srvm2m_ProcessRxMessage ( uint8_t * const pui8Rxdata, kernel_DataExchange_Type *psdataobject)
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* identify the different fields of the serial command*/
	ui8status = prtm2m_ProcessRxMessage (  pui8Rxdata ,psdataobject );

	return ui8status;

}

/***************************************************************************//**
 * @brief 		this function request a shunt request to the kernel to receive
 * the information requested by the serial host.
 * @details  	some OS message can be sent to different task and the
 * kernel sent the message to the right task.
 * @param[in] 	sdataobject
 * @param[out] 	pQueueItem  :the queue message too be posted
 * @return 		CROSSRFID_MESSAGETOBEPOSTED
 ******************************************************************************/
uint8_t srvm2m_ShuntKernelMessage ( const kernel_DataExchange_Type sdataobject, Kernel_QueueItem_struct *pQueueItem)
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	(*pQueueItem) = (Kernel_QueueItem_struct) {   KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_SERIALTASKID),
													KERNEL_STAYWAKEUP,1,KERNEL_SHUNTID_MEASUREREADY	};

	if ( (KERNEL_COMMANDCODE_SET == sdataobject.ui8CommandId) &&
		 (KERNEL_OBJECTCODE_ACCELEROMETER == sdataobject.ui8ObjectId) &&
		 (KERNEL_ACTIONCODE_ON == sdataobject.ui8ActionId) )
	{
		ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else
	{
		/*do nothing*/
	}

	return ui8status ;
}

/***************************************************************************//**
 * @brief 		this function displays the the accelerometer measurement
 * @details  	the minimum ODR of the accelerometer is around 12 Hz and
 * in order to be more readable the measurement is displayed every s.
 * @param[in] 	bIsActive : true when the activity counter is enabled
 * @param[in] 	i16ActivityTime :number of hours of the activity
 * @param[in] 	i16NbStart : number of start of the equipement
 * @param[out] 	pQueueItem  :the queue message too be posted
 * @return 		CROSSRFID_MESSAGETOBEPOSTED
 ******************************************************************************/
void srvM2M_PrintActivityCounterStatus (const bool bIsActive , const uint16_t i16ActivityTime , const uint16_t i16NbStart)
{
char ui8stringtodisplay [] = "status : Active     Activity time : aaaaa NbStart : nnnnn\n";
uint8_t saccel[6];

	if ( false == bIsActive)
	{
		memcpy( &(ui8stringtodisplay [strlen ("status : ")]), "Not active" , strlen ("Not active"));
	} else {/* donothing */}

	prtm2m_i16ToStr(i16ActivityTime , 4 , saccel);
	memcpy( &(ui8stringtodisplay [strlen ("status : Active     Activity time : ")]), saccel , 5);
	prtm2m_i16ToStr(i16NbStart , 4 , saccel);
	memcpy( &(ui8stringtodisplay [strlen ("status : Active     Activity time : aaaa NbStart : ")]), saccel , 5);
}

/***************************************************************************//**
 * @brief 		this function displays the the accelerometer measurement
 * @details  	the minimum ODR of the accelerometer is around 12 Hz and
 * in order to be more readable the measurement is displayed every s.
 * @param[in] 	i16xaxis : accelerometer measurement on the Xaxis
 * @param[in] 	i16xaxis : accelerometer measurement on the Xaxis
 * @param[in] 	i16xaxis : accelerometer measurement on the Xaxis
 * @param[out] 	pQueueItem  :the queue message too be posted
 * @return 		CROSSRFID_MESSAGETOBEPOSTED
 ******************************************************************************/
void srvM2M_PrintAccelmeasurement (int16_t i16xaxis, int16_t i16yaxis , int16_t i16zaxis)
{
char ui8stringtodisplay [] = "Xaxis : x.xxx Yaxis : y.yyy Z Axis : z.zzz\n";
uint8_t saccel[6];
static uint8_t ui8NbLoop = 0;


	if (ui8NbLoop++ >11) /* because the accel measurement is made at a frequency of 12Hz*/
	{

		/* convert the integer in printable character and display them*/
		prtm2m_i16ToStr(i16xaxis , 4 , saccel);
		memcpy( &(ui8stringtodisplay [strlen ("Xaxis : ")]), saccel , 5);
		prtm2m_i16ToStr(i16yaxis , 4 , saccel);
		memcpy( &(ui8stringtodisplay [strlen ("Xaxis : x.xxx Yaxis : ")]), saccel , 5);
		prtm2m_i16ToStr(i16zaxis , 4 , saccel);
		memcpy( &(ui8stringtodisplay [strlen ("Xaxis : x.xxx Yaxis : y.yyy Z Axis : ")]), saccel , 5);
		srvm2m_SendString (ui8stringtodisplay);

		ui8NbLoop = 0;
	} else {/* do nothing */}


}

/***************************************************************************//**
 * @brief		helper wrapper that performs the strlen before calling uartPutData()
 * @details		this USART  peripheral is might be used for the external sensor
 * or the M2M communication. The choice is make thanks the compilation variable
 * USEM2MINTERFACE
 * @param[in] 	str, null terminated string
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void srvm2m_SendString(const char* const str)
{
	prtm2m_SendString( str );
}


/******** THE REST OF THE FILE IS DOCUMENTATION ONLY !**********************//**
 * @{

@page serial_interface Serial communication

	Overview:

	@li @ref Introduction
	@li @ref Data integrity
	@li @ref Access to the memory

@n @section serial_intro Introduction

the serial peripheral uses the UART interface and runs at 9600 bauds with 8 bits data and 1 stop bit.

@n @section serial_command Command format

the command format is the UTF8. a white space should be inserted between the different fields.

  	|---------------|---------------|-----------------------|
  	| Command code	|  feature		| action 				|
  	|				|				|						|
  	|---------------|---------------|-----------------------|
  	| set			| 	accel		|			on			|
  	| set			| 	accel		|			off			|
  	| get			| 	accel		|			state		|
  	| get			| 	magneto		| 						|
  	|---------------|---------------|-----------------------|

@n @section exchangedata_task format of the exchanged data between the task

the structure prtM2M_SerialCommand_Type save the fields of the serial command.
this structure is shared to the other task.

typedef struct {
	uint8_t ui8CommandId ;
	uint8_t ui8ObjectId ;
	uint8_t ui8ActionId ;
	uint8_t *ui8pdata;
}prtM2M_SerialCommand_Type;

@n @section serial_accelerometercommand Enable the accelerometer

	this command enables or disables the accelerometer

	|---------------|-------------------|-----------------------|-------------------------------|
  	| Command code	|  operation		| 		data			|	comments					|
  	|---------------|-------------------|-----------------------|-------------------------------|
  	| set			| 	accelerometer	|		on				| to enable the accelerometer	|
  	| set			| 	accelerometer	|		off				| to disable the accelerometer	|
  	|---------------|-------------------|-----------------------|-------------------------------|

  	example : set accelerometer	on

@n @section tempdrv_access Access to the memory



 * @}**************************************************************************/


#endif
