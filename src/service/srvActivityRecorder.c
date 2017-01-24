/*******************************************************************************
 * @file srvActivityRecorder.c
 * @brief this file defines the command set for the activity recorder
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvActivityRecorder.h"

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)


#define SRVACT_COMPUTEFFT 	 1		/* compute the FFT of the logs of the magnetometer and the accellerometer*/
/*===========================================================================================================
						Private variables definition
===========================================================================================================*/
static srvActivityRecorder_status_struct 		srvActRec_status = SRVACTIVITYREC_DEFAULTSTATUS;
static uint16_t 								ui16MagneticThreshold = SRVACTIVITYREC_MAGNETICFIELDTHRESHOLD;		/* measurement floor measured during the calibration*/
static srvActivityRecorder_AxisBuffer_struct 	saAccelBuffer ;
static int16_t 									*pai16MagnetoBuffer  ;													/* pointer to the buffer of the magnetometer measurement*/
static uint16_t									*paui16FFTMagnetoBuffer  ;													/* pointer to the buffer of the magnetometer measurement*/
static uint16_t									ui16QueueBufferMsg[7];	/* bufffer for queue message*/
/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
static void 	srvActRec_ResetActivityTime 		( void );
static void 	srvActRec_ResetActivityCounter 		( void );
static uint8_t 	srvActRec_CalibrateMagneticDetector ( void );
static uint8_t 	srvActRec_InitLogMagneticField 		( void );
static uint8_t 	srvActRec_ComputeFFT 				( int16_t * pi16sourcedata , uint16_t * pui16FFTdata ,  uint16_t *ui16Nbelemt);
/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief 		this function measures the magnetic field and updtaed the
 * 				threshold
 * @details		the measurement is carried out 10 times in a loop. Between
 * 				each measurement the power supply of the I2C bus is powered
 * 				down.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_INITSENSOR_ERROR : the function is not successful.
 * the measurement floor is updated to its default value
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful.
*******************************************************************************/
static uint8_t srvActRec_CalibrateMagneticDetector ( void )
{
uint8_t 	ui8status = CROSSRFID_INITSENSOR_ERROR;
int8_t 		i8NthMeas = SRVACTIVITYREC_NBMEASUREFORTHECALIBRATION;
uint16_t 	ui16MagneticField;
uint32_t 	ui32MagneticFloor = 0;
uint16_t 	ui16NbMeas=0;

	srvlis3mdl_SetNbMeas (0);
	//srvlis3mdl_GetMeasBuffer (&saMagnetoBuffer);
	do	/* launch the loop of measurement */
	{
		prtlis3mdh_EnablePowerSuppply (true);
		sleep_DelayTask(10,configSLEEP_MODE);
		ui8status = srvlis3mdl_GetAbsMagneticFieldOn3Axis (&ui16MagneticField);	/* measure the magnetic field */
		pai16MagnetoBuffer[ui16NbMeas++]= ui16MagneticField;
		ui32MagneticFloor += ui16MagneticField;
		prtlis3mdh_EnablePowerSuppply (false);
		sleep_DelayTask(10,configSLEEP_MODE);
	}while (i8NthMeas-->0);
	srvlis3mdl_SetNbMeas (ui16NbMeas);

	if ( CROSSRFID_SUCCESSCODE == ui8status)
	{
		ui16MagneticThreshold = (uint16_t) ((SRVACTIVITYREC_FACTORFORTHECALIBRATION/SRVACTIVITYREC_NBMEASUREFORTHECALIBRATION)* ui32MagneticFloor);
	}
	else
	{
		ui16MagneticThreshold = SRVACTIVITYREC_MAGNETICFIELDTHRESHOLD; /* update with the default value*/
	}
	return ui8status;
}


/***************************************************************************//**
 * @brief 		this function initializes the continuous measurement of the
 * magnetic field.
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_INITSENSOR_ERROR : the function is not successful.
 * the measurement floor is updated to its default value
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful.
*******************************************************************************/
static uint8_t srvActRec_InitLogMagneticField ( void )
{
uint8_t 	ui8status = CROSSRFID_INITSENSOR_ERROR;

	prtlis3mdh_EnablePowerSuppply (true);
	sleep_DelayTask(10,configSLEEP_MODE);
	ui8status = prtlis3mdh_CheckDeviceId ();
	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		prtlis3mdh_initDataReadypad (true);
		prtlis3mdh_InitiateMagneticFieldLog (true);
	}
	else
	{
		ui8status = CROSSRFID_INITSENSOR_ERROR;
	}

	return ui8status;
}

/***************************************************************************//**
 * @brief 		this function resets the activity recorder
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
static void srvActRec_ResetActivityTime ( void )
{
	srvActRec_status.ui16Activitytime = 0;
}


/***************************************************************************//**
 * @brief 		this function resets the activity recorder
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
static void srvActRec_ResetActivityCounter ( void )
{
	srvActRec_status.ui16NbActivity = 0;
}

/***************************************************************************//**
 * @brief 			this function compute the FFT stored tin the dedicated buffer.
 * the data well be copied to the dedicated buffer
 * @param[in] 		pi16sourcedata : raw data measurement
 * @param[out] 		pui16FFTdata : FFT data
 * @param[in,out] 	ui16Nbelemt : the number of element to be computed. this value
 * may be updated to be equal to 2 power number (16,32,64...).
 * @return CROSSRFID_SUCCESSCODE : the function is successful
 * @return CROSSRFID_ERROR_WRONGPARAMETER : one parameter is erroneous
*******************************************************************************/
static uint8_t srvActRec_ComputeFFT ( int16_t * pi16sourcedata , uint16_t * pui16FFTdata , uint16_t *ui16Nbelemt)
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	srvfft_UpdateNbMeas(ui16Nbelemt);
	ui8status = srvfft_CopyRawData (pi16sourcedata,(*ui16Nbelemt));
	if (CROSSRFID_SUCCESSCODE == ui8status )
	{
		srvfft_ComputeFFT ();
		srvfft_CopyFFTData (pui16FFTdata,*ui16Nbelemt);
	}
	else
	{
		*ui16Nbelemt = 0;
	}
	return ui8status;
}


/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief 		this function increase the Activity recorder
 * @details 	this function is called by the LEtimer IRQ which is fired every
 * hour of activity ( see letimerdrv_init)
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvActRec_IncreaseActivityTime ( void )
{
	srvActRec_status.ui16Activitytime++;
}

/***************************************************************************//**
 * @brief 		this function increase the Activity recorder
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvActRec_IncreaseActivityCounter ( void )
{
	srvActRec_status.ui16NbActivity++;
}

/***************************************************************************//**
 * @brief 		this function increase the Activity recorder
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvActRec_GetActivityTime ( uint16_t  *ui16ActivityRecorder)
{
	(*ui16ActivityRecorder) = srvActRec_status.ui16Activitytime;
}




/***************************************************************************//**
 * @brief 		this function initializes the activity recorder
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvActRec_InitActivityRecorder ( void )
{
	srvActRec_status = (srvActivityRecorder_status_struct)  SRVACTIVITYREC_DEFAULTSTATUS;
	/* Initialize the pointer of the buffer */
	srvadxl363_GetAxisMeasBuffer ( &(saAccelBuffer.Xaxis),&(saAccelBuffer.Yaxis),&(saAccelBuffer.Zaxis));
	srvlis3mdl_GetMeasBuffer (&pai16MagnetoBuffer );
	srvlis3mdl_GetFFTBuffer ( &paui16FFTMagnetoBuffer);


}

#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)
/***************************************************************************//**
 * @brief 		this function process the message KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN
 * coming from the RF host and according to the parameters of the queue object
 * read or write the time counter
 * @param[in] 	ui8ReadOrWriteAccess : 0 for a Read Access 1 for a Write
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvActRec_ProcessHowMuchTimeYouRun ( uint8_t ui8ReadOrWriteAccess )
{
Kernel_QueueItem_struct sQueueItem = {	(((KERNEL_RFFRONTENDTASKID << KERNEL_RECEIVER_SHIFT)& KERNEL_RECEIVER_MASK) | 	/* the receiver is the RF task */
										 ((KERNEL_SENSORTASKID << KERNEL_SENDER_SHIFT) & KERNEL_SENDER_MASK)) ,			/* the sender is the serial task */
											KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN,
											0,NULL};


	if ( SRVACTREC_SYSTEMFILE_WRITEACCESSBITFIELD == ui8ReadOrWriteAccess )
	{
		srvActRec_ResetActivityTime ();
		sQueueItem.ui16NbByte = 0x00; 	/* no data are sent only acknowledge*/
	}
	else /*SRVACTREC_SYSTEMFILE_READACCESSBITFIELD*/
	{
		sQueueItem.ui16NbByte = 0x02; 			/* 2 bytes no data are sent only acknowledge*/
		sQueueItem.pData = (uint8_t *) (&srvActRec_status.ui16Activitytime); /* copy the pointer address*/
	}

	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
}

/***************************************************************************//**
 * @brief 		this function process the message KERNEL_MESSAGEID_HOWMUCHTIMEYOURUN
 * coming from the RF host and according to the parameters of the queue object
 * read or write the time counter
 * @param[in] 	ui8ReadOrWriteAccess : 0 for a Read Access 1 for a Write
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
void srvActRec_ProcessHowManyTimeYouRun ( uint8_t ui8ReadOrWriteAccess )
{
Kernel_QueueItem_struct sQueueItem = {	KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID),
										KERNEL_MESSAGEID_HOWMANYTIMEYOURUN,
										0, NULL};


	if ( SRVACTREC_SYSTEMFILE_WRITEACCESSBITFIELD == ui8ReadOrWriteAccess )
	{
		srvActRec_ResetActivityCounter ();
		sQueueItem.ui16NbByte = 0x00; 	/* no data are sent only acknowledge*/
	}
	else /*SRVACTREC_SYSTEMFILE_READACCESSBITFIELD*/
	{
		sQueueItem.ui16NbByte = 0x02; 			/* 2 bytes no data are sent only acknowledge*/
		sQueueItem.pData = (uint8_t *)  (&srvActRec_status.ui16NbActivity); /* copy the pointer address*/
	}

	xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
}

/***************************************************************************//**
 * @brief 		this function process the calibration message coming from the
 * RF task. The RF host can request a get operation to know the status
 * of the calibration of one sensor or a set operation to start the
 * caibration of a sensor.
 *
 * @param[in] 	psQueueItem : the incoming and outgoing queue message
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be posted
*******************************************************************************/
uint8_t srvActRec_ProcessCalibration ( Kernel_QueueItem_struct * psQueueItem )
{
uint16_t ui16SensorId = (uint16_t ) *( (uint16_t *)(psQueueItem->pData) );
uint16_t IsGetOrSetRequest = (uint16_t ) *( (uint16_t *)(psQueueItem->pData+2) );
uint8_t ui8status  = CROSSRFID_MESSAGETOBEPOSTED;

	if (( SRVACTREC_SYSTEMFILE_WRITEACCESSBITFIELD == IsGetOrSetRequest) &&  /* the calibration should be launched*/
		  srvActRec_status.aeSensorCalStatus[ui16SensorId] != SRVACTREC_STATUS_ONGOING)	/* and the calibration is not on going*/
	{
		switch (ui16SensorId)
		{
			case KERNEL_SENSOR_ID_ACCELERO:

			break;

			case KERNEL_SENSOR_ID_MAGNETO:
				srvActRec_status.aeSensorCalStatus[ui16SensorId] = SRVACTREC_STATUS_ONGOING;
				ui8status = srvActRec_CalibrateMagneticDetector ();
				if (CROSSRFID_SUCCESSCODE == ui8status)					/* now the calibration is over*/
				{
					srvActRec_status.aeSensorCalStatus[ui16SensorId] = SRVACTREC_STATUS_DONE;
				}
				else
				{
					srvActRec_status.aeSensorCalStatus[ui16SensorId] = SRVACTREC_STATUS_ERROR;
				}
			break;

			default :
				/*should not happen*/
			break;
		}

	}
	else {/*do nothing*/}

	psQueueItem->ui16NbByte = 4;
	ui16QueueBufferMsg[0] = ui16SensorId;
	ui16QueueBufferMsg[1] = (uint16_t)srvActRec_status.aeSensorCalStatus[ui16SensorId];
	psQueueItem->pData = (uint8_t*) ui16QueueBufferMsg;
	/* fill the message */
	psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);

	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	return ui8status;
}


/***************************************************************************//**
 * @brief 		this function process the continuous measurement for one
 * sensor. The sensor is configured to run autonomously and send a IRQ when
 * a measurement is available.
 * @param[in/out] 	ui8pdata : the incoming and outgoing queue message
 * @param[out] 	none
 * @return 		CROSSRFID_ERROR : the function is not successful
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
*******************************************************************************/
uint8_t srvActRec_ProcessSerialRequest ( uint8_t * ui8pdata )
{
kernel_DataExchange_Type *psRequest = (kernel_DataExchange_Type*) ui8pdata ;
uint8_t ui8status  = CROSSRFID_ERROR;


	switch (psRequest->ui8ObjectId)
	{
		case KERNEL_OBJECTCODE_ACCELEROMETER:

			switch (psRequest->ui8CommandId)
			{
				case KERNEL_COMMANDCODE_SET:

				break;
				case KERNEL_COMMANDCODE_GET:
					psRequest->ui8pdata[0] = srvActRec_status.aeSensorLogStatus[KERNEL_SENSOR_ID_ACCELERO];
					ui8status  = CROSSRFID_SUCCESSCODE;
				break;
				default :
					/*should not happen*/
				break;
			}
		break;

		case KERNEL_OBJECTCODE_MAGNETOMETER:

		break;

		default:
			/*should not happen*/
		break;
	}


	return ui8status;
}
/***************************************************************************//**
 * @brief 		this function process the continuous measurement for one
 * sensor. The sensor is configured to run autonomously and send a IRQ when
 * a measurement is available.
 *
 * @param[in] 	psQueueItem : the incoming and outgoing queue message
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be posted
*******************************************************************************/
uint8_t srvActRec_ProcessLog ( Kernel_QueueItem_struct * psQueueItem )
{
uint16_t IsGetOrSetRequest = (uint16_t ) *( (uint16_t *)(psQueueItem->pData) );
uint16_t ui16SensorId = (uint16_t ) *( (uint16_t *)(psQueueItem->pData+2) );
uint8_t ui8status  = CROSSRFID_MESSAGETOBEPOSTED;
uint8_t *pui8XYZaxis;

	if (( SRVACTREC_SYSTEMFILE_WRITEACCESSBITFIELD == IsGetOrSetRequest) &&  /* the calibration should be launched*/
		  srvActRec_status.aeSensorLogStatus[ui16SensorId] != SRVACTREC_STATUS_ONGOING)	/* and the calibration is not on going*/
	{
		switch (ui16SensorId)
		{
			case KERNEL_SENSOR_ID_ACCELERO:

				srvadxl363_EnableDataReadyIrq (false);					/* to avoid the irq */
				ui8status = srvadxl363_InitLogMeasurement (true);		/* configure the sensor and activate the IRQ*/
				if (CROSSRFID_SUCCESSCODE == ui8status)
				{
					srvActRec_status.aeSensorLogStatus[ui16SensorId] = SRVACTREC_STATUS_ONGOING;
					srvadxl363_EnableDataReadyIrq (true);
					//prtadxl363_ReadStatus ( &(pui8XYZaxis) ); 		/* clean the IRQ from the point of view of the Adxl363*/
					srvadxl363_ReadAcceloMeasure (&pui8XYZaxis);		/* clean the IRQ from the point of view of the Adxl363*/
				}
				else
				{
					srvActRec_status.aeSensorLogStatus[ui16SensorId] = SRVACTREC_STATUS_ERROR;
				}

			break;

			case KERNEL_SENSOR_ID_MAGNETO:

				ui8status = srvActRec_InitLogMagneticField ();	/* configure the sensor and activate the IRQ*/
				if (CROSSRFID_SUCCESSCODE == ui8status)
				{
					srvActRec_status.aeSensorLogStatus[ui16SensorId] = SRVACTREC_STATUS_ONGOING;
				}
				else
				{
					srvActRec_status.aeSensorLogStatus[ui16SensorId] = SRVACTREC_STATUS_ERROR;
				}
			break;

			default :
				/*should not happen*/
			break;
		}

	}
	else {/*do nothing*/}


	ui16QueueBufferMsg[0] = ui16SensorId;
	ui16QueueBufferMsg[1] = (uint16_t)srvActRec_status.aeSensorLogStatus[ui16SensorId];
	/* fill the message */
	psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte = 4;
	psQueueItem->ui16notification = KERNEL_MESSAGEID_LOG;
	psQueueItem->pData = (uint8_t*) ui16QueueBufferMsg;

	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	return ui8status;
}

/***************************************************************************//**
 * @brief 		this function process the continuous measurement for one
 * sensor. the sensor is configured to run autonomously and send a IRQ when
 * a measurement is available.
 * @param[in] 	psQueueItem : the incoming and outgoing queue message
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be posted
*******************************************************************************/
uint8_t srvActRec_ProcessComputeFFT ( Kernel_QueueItem_struct * psQueueItem )
{
uint16_t IsGetOrSetRequest = (uint16_t ) *( (uint16_t *)(psQueueItem->pData) );
uint16_t ui16SensorId = (uint16_t ) *( (uint16_t *)(psQueueItem->pData+2) );
uint16_t ui16MeasurmentId = (uint16_t ) *( (uint16_t *)(psQueueItem->pData+4) );
uint16_t ui16NbMeasurement = 0;
uint8_t ui8status  = CROSSRFID_MESSAGETOBEPOSTED;
int8_t *ipdata;
int8_t *i8pfftdata;



	if (( SRVACTREC_SYSTEMFILE_WRITEACCESSBITFIELD == IsGetOrSetRequest) &&  /* the calibration should be launched*/
		  srvActRec_status.aeSensorLogStatus[ui16SensorId] == SRVACTREC_STATUS_DONE)	/* and the log is now over*/
	{
		srvActRec_status.eFFTcomputationStatus  = SRVACTREC_STATUS_NOTDONE;
		switch (ui16SensorId)				/* select the right sensor*/
		{
			case KERNEL_SENSOR_ID_ACCELERO:
				switch (ui16MeasurmentId)/* select the right buffer pointer*/
				{
					case KERNEL_MEASID_XRAWAXIS:
					case KERNEL_MEASID_YRAWAXIS:
					case KERNEL_MEASID_ZRAWAXIS:
						srvadxl363_GetMeasBuffer (ui16MeasurmentId, &ipdata);
						srvadxl363_GetNbMeas (ui16MeasurmentId, &ui16NbMeasurement);
						srvadxl363_GetMeasBuffer (KERNEL_MEASID_FFT, &i8pfftdata);
					break;
					default :
						srvActRec_status.eFFTcomputationStatus = SRVACTREC_STATUS_ERROR;
					break;
				}

			break;

			default :
				srvActRec_status.eFFTcomputationStatus = SRVACTREC_STATUS_ERROR;
			break;

		}

		if (SRVACTREC_STATUS_NOTDONE == srvActRec_status.eFFTcomputationStatus )
		{
#if (SRVACT_COMPUTEFFT == 1)
			/* compute the FFT*/
			ui8status = srvActRec_ComputeFFT ( (int16_t *) ipdata,(uint16_t *) i8pfftdata,&ui16NbMeasurement);
			if ( CROSSRFID_SUCCESSCODE == ui8status)
			{
				srvActRec_status.eFFTcomputationStatus = SRVACTREC_STATUS_DONE;
				srvadxl363_SetNbMeas ( KERNEL_MEASID_FFT , ui16NbMeasurement/2);
			}
			else
			{
				srvActRec_status.eFFTcomputationStatus = SRVACTREC_STATUS_ERROR;
				srvadxl363_SetNbMeas ( KERNEL_MEASID_FFT , 0);
			}
#endif
		}

	}
	else {/*do nothing*/}


	/* fill the message */
	(*psQueueItem) = (Kernel_QueueItem_struct) {   KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID),
													KERNEL_MESSAGEID_COMPUTEFFT,4, (uint8_t *)ui16QueueBufferMsg};
	ui16QueueBufferMsg[0] = ui16SensorId;
	ui16QueueBufferMsg[1] = srvActRec_status.eFFTcomputationStatus;

	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	return ui8status;
}


#endif

/***************************************************************************//**
 * @brief 		this function process the IRQ coming from the accelerometer
 * when it is configured as motion detection
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is succesful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED
*******************************************************************************/
uint8_t srvActRec_ProcessAcceleromterIRQ ( Kernel_QueueItem_struct *sQueueItem )
{
prtadxl363_status_union pustatus;
uint16_t ui16MagneticField=0;
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
uint8_t *pui8XYZaxis;
uint16_t ui16NthPoint=0;


	prtadxl363_ReadStatus ( &(pustatus.ui8status) ); 		/* clean the IRQ from the point of view of the Adxl363*/

#if 0

	if (pustatus.sstatus.bawake == 1 )						/* an activity is detected */
	{
		if (true == srvActRec_status.IsAdxlLowpowerMode )
		{
			prtadxl363_DisableLowPowerMotionDetection (); 	/* disable the low power mode of the ADXL363 to ensure the activity */
			timer_inittimer0(4*TIMER_WAIT1S) ;			  	/* and lauch a timer for 4 s to deteect the false activity detection*/
			kernel_AllowSleepInEM1 ( );					  	/* because the timer 0 works on the HFXO the lowest sleep mode should be EM1*/ /* todo : should be replaced by a queue message*/
			srvActRec_status.IsAdxlLowpowerMode = false;
		}
		else
		{													/* when the activity is confirmed*/
			timer_stop (TIMER_TIMER0ID);					/* stop the timer 0 */
			kernel_AllowSleepInDefault ( );					/* allow the mcu to goes below EM1*/											/* todo : should be replaced by a queue message*/
			letimerdrv_enable (true);						/* start the letimer to count the time activity */
			srvActRec_IncreaseActivityCounter ();
		}

	}
	else if (pustatus.sstatus.bawake == 0 )					/* an inactivity is detected */
	{
		letimerdrv_enable (false);
		srvActRec_status.IsAdxlLowpowerMode = false;
		prtadxl363_EnableLowPowerMotionDetection ();
	}
#else
		if ( SRVACTREC_STATUS_ONGOING == srvActRec_status.aeSensorLogStatus[KERNEL_SENSOR_ID_ACCELERO] )	/* the log is on going */
		{

			srvadxl363_ReadAcceloMeasure (&pui8XYZaxis);										/* get the measurements*/
			srvadxl363_GetNbMeas ( KERNEL_MEASID_XRAWAXIS, &ui16NthPoint );
			saAccelBuffer.Xaxis[ui16NthPoint] = (int16_t) ( ((int16_t*)pui8XYZaxis)[0]);	/* save the measurement*/
			saAccelBuffer.Yaxis[ui16NthPoint] = (int16_t) ( ((int16_t*)pui8XYZaxis)[1]);
			saAccelBuffer.Zaxis[ui16NthPoint] = (int16_t) ( ((int16_t*)pui8XYZaxis)[2]);
			srvadxl363_SetNbMeas (KERNEL_MEASID_XYZRAWAXIS , ui16NthPoint+1 );
			if ( ui16NthPoint >= SRVACTIVITYREC_ACCELEROMETERBUFFERNBELMEMENT )
			{
				srvadxl363_EnableMotionIrq (false);
				srvadxl363_InitLogMeasurement (false);
				srvActRec_status.aeSensorLogStatus[KERNEL_SENSOR_ID_ACCELERO] = SRVACTREC_STATUS_DONE;
				(*sQueueItem) = (Kernel_QueueItem_struct) {   KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID),
																KERNEL_MESSAGEID_LOG,4, (uint8_t *)ui16QueueBufferMsg};
				ui16QueueBufferMsg[0] = KERNEL_SENSOR_ID_ACCELERO;
				ui16QueueBufferMsg[1] = (uint16_t)(srvActRec_status.aeSensorLogStatus[KERNEL_SENSOR_ID_ACCELERO]);

				ui8status = CROSSRFID_MESSAGETOBEPOSTED;
			}else {/* do nothing*/}

		}
		else if (SRVACTREC_STATUS_ONGOING == srvActRec_status.aeSensorContinuousStatus[KERNEL_SENSOR_ID_ACCELERO] )
		{

		}
		else
		{
			if (pustatus.sstatus.bawake == 0 )					/* an inactivity is detected */
			{
				letimerdrv_enable (false);
			}
			else
			{

				//srvlis3mdl_Init ();
				//prtlis3mdh_InitiateMagneticFieldDetection ();
				prtlis3mdh_EnablePowerSuppply (true);
				vTaskDelay(10);
				ui8status = srvlis3mdl_GetAbsMagneticFieldOn3Axis (&ui16MagneticField);	/* measure the Magnetic field */
				printf ("magnetic field %d\n",ui16MagneticField );
				prtlis3mdh_EnablePowerSuppply (false);
	#if 0
				/* get the log of the acceleration */

	#endif
				if ((CROSSRFID_SUCCESSCODE == ui8status ) && (ui16MagneticField>ui16MagneticThreshold))
				{
					letimerdrv_enable (true);						/* start the letimer to count the time activity */
					srvActRec_IncreaseActivityCounter ();
				} else {/* the magnetic measurement doesn't confirm the activity*/}
			}
		}



#endif
		return ui8status;
}

/***************************************************************************//**
 * @brief 		this function process the IRQ coming from the magnetometer
 * when it is configured as a continuous measurement
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
*******************************************************************************/
uint8_t srvActRec_ProcessMagnetoIRQ ( Kernel_QueueItem_struct *psQueueItem )
{
uint8_t ui8status = CROSSRFID_SUCCESSCODE;
uint16_t ui16MagneticField;
static uint16_t ui16NbMeas=0;

	ui8status = srvlis3mdl_GetAbsMagneticFieldOn3Axis (&ui16MagneticField);	/* measure the magnetic field */
	pai16MagnetoBuffer[ui16NbMeas++]= ui16MagneticField;

	if (ui16NbMeas>=SRVLIS3MDH_BUFFER_NBELEMENT)
	{

		srvlis3mdl_SetNbMeas (ui16NbMeas);
		/* now the buffer is full */
		prtlis3mdh_initDataReadypad (false);
		prtlis3mdh_InitiateMagneticFieldLog (false);
		prtlis3mdh_EnablePowerSuppply (false);

		srvActRec_status.aeSensorLogStatus[KERNEL_SENSOR_ID_MAGNETO] = SRVACTREC_STATUS_DONE;

#if (SRVACT_COMPUTEFFT == 1)
		/* compute the FFT*/
		ui16NbMeas = SRVLIS3MDH_BUFFER_NBELEMENT;
		srvActRec_ComputeFFT (pai16MagnetoBuffer,paui16FFTMagnetoBuffer,&ui16NbMeas);
		srvlis3mdl_SetNbFFTMeas (ui16NbMeas/2);
#endif
		ui16NbMeas=0;

		psQueueItem->ui16NbByte = 4;
		ui16QueueBufferMsg[0] = KERNEL_SENSOR_ID_MAGNETO;
		ui16QueueBufferMsg[1] = (uint16_t)srvActRec_status.aeSensorLogStatus[KERNEL_SENSOR_ID_MAGNETO];
		psQueueItem->pData = (uint8_t*) ui16QueueBufferMsg;
		psQueueItem->ui16notification = KERNEL_MESSAGEID_LOG;
		/* fill the message */
		psQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
		ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else {/* do nothing*/}

	return ui8status;

}


/***************************************************************************//**
 * @brief 		this function configures the ADXL363 to act as a motion detector
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
*******************************************************************************/
uint8_t srvActRec_EnableMotionDetection ( bool bOnOrOff )
{
prtadxl363_status_union pustatus;
uint8_t status = CROSSRFID_INITSENSOR_ERROR;

	if (true == bOnOrOff)
	{
		prtadxl363_Init ();
		status = prtADXL363_CheckDeviceId ();
		if (CROSSRFID_SUCCESSCODE == status)
		{
			letimerdrv_init ();								/* initialize the letimer to measure the activity */
			prtadxl363_InitiateMotionDetection ();			/* configure the Adxl */
			prtadxl363_EnableMotionIrq (true);					/* enable the IRQ*/
			prtadxl363_ReadStatus (&(pustatus.ui8status));	/* read the status to clear the IRQ of the adxl363*/
		}
		else
		{
			prtadxl363_Deinit ();
		}
	}
	else
	{
		interface_DisableGpioUsart1 ();
	}

	return status;
}


/***************************************************************************//**
 * @brief 		this function launches the calibration of the accelerometer
 * @note		the accelerometer is configured as continuous measurement
 * 				the
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_INITSENSOR_ERROR : the function is not successful.
 * the measurement floor is updated to its default value
 * @return 		CROSSRFID_INITSENSOR_ERROR : the function is not successful.
 * the noise floor is updated to the average of the measurement
*******************************************************************************/
uint8_t srvActRec_CalibrateAcceleroDetector ( void )
{
uint8_t 	ui8status = CROSSRFID_INITSENSOR_ERROR;

	ui8status = srvadxl363_InitLogMeasurement (true);
	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		srvadxl363_EnableMotionIrq (true);	/* enable the Irq*/
		srvActRec_status.aeSensorCalStatus[KERNEL_SENSOR_ID_ACCELERO] = SRVACTREC_STATUS_ONGOING;
	} else {/* do nothing*/}

	return ui8status;
}
#endif


