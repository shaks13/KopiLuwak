/*******************************************************************************
 * @file sensor_process.c
 * @brief this function set manages the data exchange between the task and the
 * device.
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/

#include "sensor_process.h"

/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/
/* The structure containing the data sampled and other information about a sample buffer */
#if 0 /* not used anymore*/
static sensor_SampleData_struct sensor_sSample;
#endif
/* Data to send to rf task, could be:
 * -	the one shot sample value
 * - 	the number of samples in the buffer
 * Used as an address pointed by the "pui8Data" free RTOS notification pointer */
static uint16_t sensor_ui16DataToSendToRfTask;

/* parameters of acquisitions for each external sensor */
static sensor_SensorConfig_struct sensor_sSensorConfig[KERNEL_SENSOR_ID_NBSENSOR];

/* bit field to know the enable/disable alarms */
//static sensor_EnableAlarm_union sensor_uEnableAlarm;
/* bit field to know the alarm status */
//static sensor_StateAlarm_union sensor_uStateAlarm;
static sensor_Alarm_struct sensor_sAlarm;
static bool sensor_bProcessIsSuccess;

static sensor_Alarm_struct sensor_Alarm;

/* allows serial to send a sensor configuration to the RF task
 * in order to update the acq system files, see ACQmode / ACQperiod / ACQbegin / ACQend system files
 * format with parameters on 16-bit:
 * [0] 				= sensor id
 * [1]  			= first system file
 * [2]  			= second system file
 * format with parameters on 32-bit:
 * [0] 				= sensor id
 * [1]or[2]<<16 	= first system file
 * [3]or[4]<<16 	= second system file */
static uint16_t sensor_tui16SensorAcqConf[KERNEL_MSG_IDX_ACQ_TIME_MSW+1];

/* Data in the external eeprom about the alarm status and the dates/times when they occurred, the mapping is:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Bit field of the states alarm 			|
 * 	|	0x0001		|		2			|	Date of the "below temp" One shot alarm	|
 * 	|	0x0003		|		2			|	Time of the "below temp" One shot alarm	|
 * 	|	0x0005		|		2			|	Date of the "upper temp" One shot alarm	|
 * 	|	0x0007		|		2			|	Time of the "upper temp" One shot alarm	|
 * 	|	0x0009		|		2			|	Date of the "low bat" One shot alarm	|
 * 	|	0x000B		|		2			|	Time of the "low bat" One shot alarm	|
 * 	|	0x000D		|		1			|	CRC16 CCITT 							|
 *  --------------------------------------------------------------------------------- */
static uint16_t sensor_tui16AlarmDataInEeprom[SENSOR_SIZE_16BITBUFFER];

/* This structure allows serial to program the starts/stops operations of the monitoring
 * according to the AcqBegin and the AcqEnd configuration sent by the RF HOST.
 * It is used when:
 * 1- Serial fills this structure when the RF host sends a SetCommand of AcqBegin or AcqEnd.
 * 2- Serial will launch or stop a sensor monitoring after receiving a message from kernel
 * notifying that a calendar event occurred */
static sensor_CalendarEvent_struct sCalendarEvent[2];
static sensor_CalendarEvent_struct * psCalendarBeginMeasEvent;
static sensor_CalendarEvent_struct * psCalendarEndMeasEvent;

/* This structure allows the serial to store the CCx calendar event linked to AcqBegin/AcqEnd
 * that are not plugged in the CCxDate/CCxTime RTCC channel because all the CCx RTCC channel are busied
 * by a date/time most recent.*/
//static sensor_CalendarEvent_struct sCalendarEventToPlug[SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG];

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/* The temperature monitoring should be launched just one time */
static bool sensor_bTempIsAlreadyStarted = false;
#endif
/*===========================================================================================================
						Private functions declarations
===========================================================================================================*/
static void  	sensor_UpdateSensorCfgFromRF			( uint16_t * pui16Data);
static void  	sensor_UpdateAlarmCfgFromRF				( uint16_t * pui16Data);
static void  	sensor_init 							( uint16_t * pui16Data );
static void  	sensor_EnableDisableAlarms				( const uint16_t ui16NewConfig );
static uint8_t 	sensor_ResetAlarms						( const uint16_t ui16NewConfig );
static uint8_t 	sensor_InitAlarms 						( void );
static uint8_t 	sensor_ReadAlarmBitFieldInExtMemory 	( void );
//static uint8_t 	sensor_BeginMonitoring 					( sensor_CalendarEvent_struct sCalendarEvent,Kernel_QueueItem_struct * psQueueItempsQueueItem);
//static uint8_t 	sensor_EndMonitoring 					( sensor_CalendarEvent_struct sCalendarEvent, Kernel_QueueItem_struct * psQueueItem);
//static bool 	sensor_IsCalendarChannelFree 			( bool bIsBeginEvent  );
static bool 	sensor_IsCalendarEventToPlug 			( uint8_t ui8EventType , int8_t * pi8IdxEvent );
//static void 	sensor_FindFreeIndexInRtccEventToPlug 	( uint8_t * pui8FreeIdxRtccEvent );
static void		sensor_ManageNewAcqBeginAcqEnd 			( 	const Kernel_Sensor_Id_enum eSensorId,
															const uint8_t ui8type,
															const uint32_t ui32Date,
															const uint32_t ui32Time );
static void  	sensor_GetMeasBuffer					( const Kernel_Sensor_Id_enum ui8SensorId , const Kernel_MeasurementId_enum eMeasurementId , int8_t **pi8MeasBuffer );
static void  	sensor_GetNbMeas						( const uint16_t ui16SensorId , const Kernel_MeasurementId_enum eMeasId , uint16_t * pui16NbMeasurement );
static uint8_t 	sensor_SendMeasureStatus 				( const uint8_t SensorId,   Kernel_QueueItem_struct *pQueueItem );
static uint8_t 	sensor_ManageTempAlarm 					( int16_t i16Measure);
static uint8_t	sensor_ConnectaNewCalendarEvent			( const uint8_t CalendarEventId);

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief This function pasts the sensor configuration of RF task
 * into the serial sensors configuration
 * @detail the buffer map is the following
 * 	|		1 word		|		type (0x01 one shot or 0x01 periodic)			|
 * 	|		1 word		|		state (0x01 free to use or in progress)			|
 * 	|		1 word		|		period value 									|
 * 	|		1 word		|		period unit 									|
 * 	|		2 words		|		start date  									|
 * 	|		2 words		|		start time 										|
 * 	|		2 words		|		end date  										|
 * 	|		2 words		|		end time  										|
 * 	|		1 word		|		Low Threshold									|
 * 	|		1 word		|		High Threshold 									|
 * @param[in] 	pui16Data: pointer on the configuration of the sensors
 * @return 		none
 ******************************************************************************/
static void sensor_UpdateSensorCfgFromRF ( uint16_t * pui16Data )
{
	uint8_t ui8Idx;
	uint8_t offsetPerSensor = KERNEL_NB_WORDS_IN_SENSOR_CONFIG;
	uint8_t offsetInConfig = SENSOR_IDX_SFILE_ACQ;

	for (ui8Idx = 0; ui8Idx < KERNEL_SENSOR_ID_NBSENSOR; ui8Idx++)
	{
		sensor_sSensorConfig[ui8Idx].ui8Type 			= pui16Data[(ui8Idx*offsetPerSensor)+offsetInConfig];
		sensor_sSensorConfig[ui8Idx].IsRunning 			= pui16Data[(ui8Idx*offsetPerSensor)+1+offsetInConfig];
		sensor_sSensorConfig[ui8Idx].ui16PeriodValue 	= pui16Data[(ui8Idx*offsetPerSensor)+2+offsetInConfig];
		sensor_sSensorConfig[ui8Idx].ui16PeriodUnit 	= pui16Data[(ui8Idx*offsetPerSensor)+3+offsetInConfig];
		sensor_sSensorConfig[ui8Idx].ui32BeginDateValue =
				(uint32_t)	(pui16Data[(ui8Idx*offsetPerSensor)+4+offsetInConfig]  | (((uint32_t)pui16Data[(ui8Idx*offsetPerSensor)+5+offsetInConfig] & 0x0000FFFF) << 16));
		sensor_sSensorConfig[ui8Idx].ui32BeginTimeValue =
				(uint32_t)	(pui16Data[(ui8Idx*offsetPerSensor)+6+offsetInConfig]  | (((uint32_t)pui16Data[(ui8Idx*offsetPerSensor)+7+offsetInConfig] & 0x0000FFFF) << 16));
		sensor_sSensorConfig[ui8Idx].ui32EndDateValue 	=
				(uint32_t)	(pui16Data[(ui8Idx*offsetPerSensor)+8+offsetInConfig]  | (((uint32_t)pui16Data[(ui8Idx*offsetPerSensor)+9+offsetInConfig] & 0x0000FFFF) << 16));
		sensor_sSensorConfig[ui8Idx].ui32EndTimeValue 	=
				(uint32_t)	(pui16Data[(ui8Idx*offsetPerSensor)+10+offsetInConfig] | (((uint32_t)pui16Data[(ui8Idx*offsetPerSensor)+11+offsetInConfig] & 0x0000FFFF) << 16));

		sensor_sSensorConfig[ui8Idx].i16LowThreshold 	= pui16Data[(ui8Idx*offsetPerSensor)+12+offsetInConfig];
		sensor_sSensorConfig[ui8Idx].i16HighThreshold 	= pui16Data[(ui8Idx*offsetPerSensor)+13+offsetInConfig];
		sensor_sSensorConfig[ui8Idx].bEndDateTimeIsPlugged 	= false;
		sensor_sSensorConfig[ui8Idx].bStartDateTimeIsPlugged 	= false;
		sensor_sSensorConfig[ui8Idx].IsConfigured.ui8field = KERNEL_SENSOR_ACQSTATE_FREE;
	}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	sensor_sSensorConfig[0].IsConfigured.bitfield.bIsEndDefined = 1; /* in infinite mode */
	sensor_sSensorConfig[0].IsConfigured.bitfield.bIsPeriodDefined = 1;
	sensor_sSensorConfig[0].IsConfigured.bitfield.bIsModeDefined = 1;
#endif
}

/***************************************************************************//**
 * @brief This function pasts the alarm configuration of the RF task
 * into the serial alarm configuration
 * @detail
 * 	The message sent from RF task:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	... 		|		...			|	....									|
 * 	|	0x0009		|		1			|	enable alarm system file				|
 * 	|	... 		|		...			|	....									|
 *  ---------------------------------------------------------------------------------
 * @param[in] 	pui16Data: pointer on the configuration
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void sensor_UpdateAlarmCfgFromRF ( uint16_t * pui16Data)
{
	sensor_Alarm.EnableAlarm.ui16Value = pui16Data[SENSOR_IDX_SFILE_ALARM_ENABLE];

}

/***************************************************************************//**
 * @brief  		Initializes the serial task
 * @param[in] 	pui16Data: the configuration sent by RF task
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void  sensor_init ( uint16_t * pui16Data )
{
	sensor_ui16DataToSendToRfTask = 0;

	/* The buffer of the temperature is empty*/
#if 0
	sensor_sSample.bOverFlow = false;
	sensor_sSample.ui16Count = 0;
	memset (sensor_sSample.ti16Data, 0 , sizeof (sensor_sSample.ti16Data));
#endif
	/* reset the configuration */
	memset (sensor_tui16SensorAcqConf, 0 , sizeof (sensor_tui16SensorAcqConf));

	sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].ui8TimerId = (uint8_t) KERNEL_SENSOR_ID_TEMPERATURE;
	sensor_sSensorConfig[KERNEL_SENSOR_ID_ACCELERO].ui8TimerId = (uint8_t) KERNEL_SENSOR_ID_ACCELERO;
	sensor_sSensorConfig[KERNEL_SENSOR_ID_MAGNETO].ui8TimerId = (uint8_t) KERNEL_SENSOR_ID_MAGNETO;

	sensor_Alarm.EnableAlarm.ui16Value = 0;
	sensor_Alarm.StateAlarm.ui16Value = 0;
	sensor_bProcessIsSuccess = false;

	/* reset the sensor sCalendarEvent config */
	memset (sCalendarEvent, 0 , sizeof (sCalendarEvent));

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	sensor_bTempIsAlreadyStarted = false;
#endif
	psCalendarBeginMeasEvent = &sCalendarEvent[0];
	psCalendarEndMeasEvent = &sCalendarEvent[1];
	/* free the next calendar event (used to manage the future start/stop monitoring with ACqBegin and AcqEnd) */
	psCalendarBeginMeasEvent->i8CCxChNumber = SENSOR_RTCCCHANNEL_0;
	psCalendarEndMeasEvent->i8CCxChNumber = SENSOR_RTCCCHANNEL_2;

	/* initializes the sensor configuration of the serial task */
	sensor_UpdateSensorCfgFromRF (pui16Data);
	/* initializes the alarm configuration of the serial task */
	sensor_UpdateAlarmCfgFromRF (pui16Data);

//#if	USERTHECRYOTIMER == 1
	/* The cryotimer triggers new temperature acquisitions, it is started by default by the Kernel task */
	/* Update the Acquisition period: */
	if (	(KERNEL_SENSOR_ACQTYPE_PERIODIC == 	sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].ui8Type) &&
			(KERNEL_SENSOR_ACQSTATE_BUSY == 	sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning)	)
	{
		srvtemp_SetPeriodOnchipTemp(	sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].ui16PeriodValue,
										sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].ui16PeriodUnit);
	}
//#endif

	srvtemp_init ((bool)sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning);

	/* initializes the service layer of the 24LC64 external memory */
	srv24LC64_init();


}

/***************************************************************************//**
 * @brief  Enables or disables the alarms by making a distinction between the
 * previous and the new enableAlarm configuration
 * @param[in] ui16NewConfig: the configuration sent by RF task
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
static void sensor_EnableDisableAlarms ( const uint16_t ui16NewConfig )
{
	/* ------------------------------ LOW TEMPERATURE ALARM ------------------------------ */
	/* the configuration has changed */
	if(sensor_Alarm.EnableAlarm.sBitField.ui16BelowTemp != ((ui16NewConfig & SENSOR_ALARM_MASK_BELOWTEMP) >> SENSOR_ALARM_SHIFT_BELOWTEMP))
	{
		/* enables the alarm */
		if(KERNEL_ALARM_ID_ENABLE == ((ui16NewConfig & SENSOR_ALARM_MASK_BELOWTEMP) >> SENSOR_ALARM_SHIFT_BELOWTEMP))
		{
			srvtemp_SetTempAlarmCB(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16LowThreshold ,false);
		}
		/* disables the alarm */
		else
		{
			srvtemp_RemoveTempAlarmCB(false);
		}
	}
	/* ------------------------------ HIGH TEMPERATURE ALARM ------------------------------ */
	/* the configuration has changed */
	if(sensor_Alarm.EnableAlarm.sBitField.ui16UpperTemp != ((ui16NewConfig & SENSOR_ALARM_MASK_UPPERTEMP) >> SENSOR_ALARM_SHIFT_UPPERTEMP))
	{
		/* enables the alarm */
		if(KERNEL_ALARM_ID_ENABLE == ((ui16NewConfig & SENSOR_ALARM_MASK_UPPERTEMP) >> SENSOR_ALARM_SHIFT_UPPERTEMP))
		{
			srvtemp_SetTempAlarmCB(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16HighThreshold ,true);
		}
		/* disables the alarm */
		else
		{
			srvtemp_RemoveTempAlarmCB(true);
		}
	}
	/* ------------------------------ LOW BATTERY ALARM ------------------------------ */
	/* the configuration has changed */
	if(sensor_Alarm.EnableAlarm.sBitField.ui16LowBattery != ((ui16NewConfig & SENSOR_ALARM_MASK_LOWBAT) >> SENSOR_ALARM_SHIFT_LOWBAT))
	{
	}
}

/***************************************************************************//**
 * @brief  		this function resets the alarms by making a distinction between the
 * previous and the new StateAlarm configuration
 * @param[in] 	ui16NewConfig: the configuration sent by RF task
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE the alarm has been reset
 * @return 		CROSSRFID_ERROR_I2C_WRITE - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_24LC64_INVALID_ADRS - invalid 24lc64 address
 ******************************************************************************/
static uint8_t sensor_ResetAlarms ( const uint16_t ui16NewConfig )
{
uint8_t 	ui8WriteStatus = CROSSRFID_SUCCESSCODE;
uint16_t 	ui16PrevioustateAlarm= sensor_sAlarm.StateAlarm.ui16Value;
uint16_t 	tui16PreviousAlarmDataInEeprom[SENSOR_SIZE_16BITBUFFER];
bool 		bMustWrite = false;

	/* make a copy of the configuration in case of unsuccessful write operation in the eeprom */
	ui16PrevioustateAlarm = sensor_sAlarm.StateAlarm.ui16Value;
	memcpy (tui16PreviousAlarmDataInEeprom, sensor_tui16AlarmDataInEeprom , sizeof (sensor_tui16AlarmDataInEeprom));

	/* ------------------------------ LOW TEMPERATURE ALARM ------------------------------ */
	/* the alarm must be reset */
	if((1 == ((ui16NewConfig & SENSOR_ALARM_MASK_BELOWTEMP) >> SENSOR_ALARM_SHIFT_BELOWTEMP)) /*&& (1 == sensor_uStateAlarm.sBitField.ui16BelowTemp)*/)
	{
		sensor_sAlarm.StateAlarm.sBitField.ui16BelowTemp = 0;
		sensor_sAlarm.AckStateAlarm.sBitField.ui16BelowTemp = 0;
		memset (&sensor_tui16AlarmDataInEeprom[SENSOR_ALARMLOWTEMP], 0x00 , 8);
		bMustWrite = true;
	}
	/* ------------------------------ HIGH TEMPERATURE ALARM ------------------------------ */
	/* the alarm must be reset */
	if(((ui16NewConfig & SENSOR_ALARM_MASK_UPPERTEMP) >> SENSOR_ALARM_SHIFT_UPPERTEMP) /*&& (1 == sensor_uStateAlarm.sBitField.ui16UpperTemp)*/)
	{
		sensor_sAlarm.StateAlarm.sBitField.ui16UpperTemp = 0;
		sensor_sAlarm.AckStateAlarm.sBitField.ui16UpperTemp = 0;
		memset (&sensor_tui16AlarmDataInEeprom[SENSOR_ALARMHIGHTEMP], 0x00 , 8);
		bMustWrite = true;
	}
	/* ------------------------------ LOW BATTERY ALARM ------------------------------ */
	/* the alarm must be reset */
	if(((ui16NewConfig & SENSOR_ALARM_MASK_LOWBAT) >> SENSOR_ALARM_SHIFT_LOWBAT) /*&& (1 == sensor_uStateAlarm.sBitField.ui16LowBattery)*/)
	{
		sensor_sAlarm.StateAlarm.sBitField.ui16LowBattery = 0;
		sensor_sAlarm.AckStateAlarm.sBitField.ui16LowBattery = 0;
		memset (&sensor_tui16AlarmDataInEeprom[SENSOR_ALARMLOWBAT], 0x00 , 8);
		bMustWrite = true;
	}

	if(true == bMustWrite)
	{
		/* updates the AlarmState */
		sensor_tui16AlarmDataInEeprom[SENSOR_ALARMBITFIELD] = sensor_sAlarm.StateAlarm.ui16Value;

		/* Computes and inserts the crc */
		srvCRC_InsertCrc16CCITT(SENSOR_SIZE_16BITBUFFER,&sensor_tui16AlarmDataInEeprom[0]);

		/* Writes the new alarm buffer in the eeprom */
		//ui8WriteStatus = srv24lc64_Write16bitData(SENSOR_ALARMBITFIELD,(SENSOR_SIZE_16BITBUFFER*2),(uint8_t*)(&sensor_tui16AlarmDataInEeprom[0]));
		ui8WriteStatus = srv24LC_WriteAlarmFields (&sensor_tui16AlarmDataInEeprom[0]);
	}
	else
	{
		/* no data to write in external memory */
	}

	if(CROSSRFID_SUCCESSCODE != ui8WriteStatus)
	{
		/* restores the previous values */
		sensor_sAlarm.StateAlarm.ui16Value = ui16PrevioustateAlarm;
		memcpy (&sensor_tui16AlarmDataInEeprom[0], &tui16PreviousAlarmDataInEeprom[0] , sizeof (sensor_tui16AlarmDataInEeprom));
		/* Writes the previous alarm buffer in the eeprom */
		ui8WriteStatus = srv24LC_WriteAlarmFields (&sensor_tui16AlarmDataInEeprom[0]);
	}
	else
	{
		/* success */
	}

	return ui8WriteStatus;
}


/***************************************************************************//**
 * @brief  		this function resets the alarms by making a distinction between the
 * previous and the new StateAlarm configuration
 * @param[in] 	ui16NewConfig: the configuration sent by RF task
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE the alarm has been reset
 * @return 		CROSSRFID_ERROR_I2C_WRITE - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_24LC64_INVALID_ADRS - invalid 24lc64 address
 ******************************************************************************/
static uint8_t sensor_InitAlarms ( void )
{
uint8_t 	ui8WriteStatus = CROSSRFID_SUCCESSCODE;
//uint16_t 	ui16PrevioustateAlarm= sensor_uStateAlarm.ui16Value;
uint16_t 	tui16TmpAlarmData[SENSOR_SIZE_16BITBUFFER];
//bool 		bMustWrite = false;


	memset (tui16TmpAlarmData, 0x00,SENSOR_SIZE_16BITBUFFER*2);
	/* Computes and inserts the crc */
	srvCRC_InsertCrc16CCITT(SENSOR_SIZE_16BITBUFFER,tui16TmpAlarmData);

	/* Writes the new alarm buffer in the eeprom */
	//ui8WriteStatus = srv24lc64_Write16bitData(SENSOR_ALARMBITFIELD,(SENSOR_SIZE_16BITBUFFER*2),(uint8_t*)(&sensor_tui16AlarmDataInEeprom[0]));
	ui8WriteStatus = srv24LC_WriteAlarmFields (tui16TmpAlarmData);
	if (CROSSRFID_SUCCESSCODE == ui8WriteStatus)
	{
		memcpy(sensor_tui16AlarmDataInEeprom, tui16TmpAlarmData,SENSOR_SIZE_16BITBUFFER*2);/* updates the AlarmState */
	}
	else {/*do  nothing*/}
	return ui8WriteStatus;
}

/***************************************************************************//**
 * @brief 		This function reads the stored data in external EEPROM about alarms
 * @param[in]	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE - transfer completed successfully.
 * @return 		CROSSRFID_ERROR_I2C_READ - an error has occurred during the i2c transfer.
 * @return 		CROSSRFID_ERROR_EXTEEPROM_CRC - the crc is wrong
 ******************************************************************************/
static uint8_t sensor_ReadAlarmBitFieldInExtMemory( void )
{
uint8_t	ui8Status;
bool 				bIsRightResidue = false;
uint16_t *			pui16;

	ui8Status = srv24LC_ReadAlarmFields ( &pui16 );

	if(CROSSRFID_SUCCESSCODE == ui8Status)
	{
		bIsRightResidue = srvCRC_IsRightResidue (SENSOR_SIZE_16BITBUFFER, pui16);

		if(true == bIsRightResidue)
		{
			/* return success code */
			ui8Status = CROSSRFID_SUCCESSCODE ;
			memcpy(sensor_tui16AlarmDataInEeprom,pui16,sizeof (sensor_tui16AlarmDataInEeprom));
		}
		else
		{
			ui8Status = CROSSRFID_ERROR_EXTEEPROM_CRC;
		}
	}
	else
	{
		/* error during i2c read: returns a bad status */
	}

	return ui8Status ;
}

/***************************************************************************//**
 * @brief 		This function reads the stored data in external EEPROM about alarms
 * @param[in]	ui8SensorId : Id of the sensor (see Kernel_Sensor_Id_enum)
 * @param[in]	eMeasurementId : Id of the measurement (Kernel_MeasurementId_enum) (optional)
 * @param[out] 	pi8MeasBuffer : pointer of the buffer measurement
 * @return 		none
 ******************************************************************************/
static void  sensor_GetMeasBuffer( const Kernel_Sensor_Id_enum ui8SensorId , const Kernel_MeasurementId_enum eMeasurementId , int8_t **pi8MeasBuffer )
{

	switch (ui8SensorId)
	{
		case KERNEL_SENSOR_ID_TEMPERATURE:
			srvtemp_GetBufferTemp (pi8MeasBuffer);
		break;
		case KERNEL_SENSOR_ID_ACCELERO:
			srvadxl363_GetMeasBuffer ( eMeasurementId ,(int8_t**)pi8MeasBuffer);

		break;
		case KERNEL_SENSOR_ID_MAGNETO:
			if (KERNEL_MEASID_FFT == eMeasurementId)
			{
				srvlis3mdl_GetFFTBuffer((uint16_t**)pi8MeasBuffer);
			}
			else
			{
				srvlis3mdl_GetMeasBuffer((int16_t**)pi8MeasBuffer);
			}
		break;
		default:
		break;
	}
}

/***************************************************************************//**
 * @brief 		This function reads the stored data in external EEPROM about alarms
 * @param[in]	none
 * @param[out] 	pui16NbMeasurement  : number of measurement available
 * @return 		none
 ******************************************************************************/
static void  sensor_GetNbMeas( const uint16_t ui16SensorId , const Kernel_MeasurementId_enum eMeasId , uint16_t * pui16NbMeasurement )
{
	switch (ui16SensorId)
	{
		case KERNEL_SENSOR_ID_TEMPERATURE:
			srvtemp_GetNbMeas (pui16NbMeasurement);
		break;
		case KERNEL_SENSOR_ID_ACCELERO:
			srvadxl363_GetNbMeas (eMeasId,pui16NbMeasurement);
		break;
		case KERNEL_SENSOR_ID_MAGNETO:
			if ( KERNEL_MEASID_RAW == eMeasId )
			{
				srvlis3mdl_GetNbMeas(pui16NbMeasurement);
			}
			else if ( KERNEL_MEASID_FFT == eMeasId)
			{
				srvlis3mdl_GetNbFFTpoint(pui16NbMeasurement);
			}
			else {/*do nothing*/}

		break;
		default:
			*pui16NbMeasurement = 0;
		break;
	}
}


/***************************************************************************//**
 * @brief 		This function sends to the RF task the status of a sensor
 *
 * @param[in]	ui8CalendarMeasEvent
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
static uint8_t sensor_SendMeasureStatus ( const uint8_t SensorId,  Kernel_QueueItem_struct *pQueueItem )
{
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	/* builds the message */
	sensor_tui16SensorAcqConf[0] = SensorId; /* the sensor id */
	sensor_tui16SensorAcqConf[1] = sensor_sSensorConfig[SensorId].ui8Type;
	sensor_tui16SensorAcqConf[2] = sensor_sSensorConfig[SensorId].IsRunning;
	pQueueItem->pData 			= (uint8_t*)&sensor_tui16SensorAcqConf[0];
	/* sends a message to RF task in order to modify the ACQ mode system file */
	pQueueItem->ui16NbByte 		= 3;
	/* the receiver is the Rf Task // the sender is the SERIAL task */
	pQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	pQueueItem->ui16notification 	= KERNEL_MESSAGEID_CHANGESTATESENSOR;

	return ui8status ;
}
#if 0
/***************************************************************************//**
 * @brief 		This function begins the monitoring of the sensor after
 * 				receiving a calendar event.
 * 				Sends a message to Rf Task to change the system files configuration.
 * @param[in]	sCalendarEvent: the calendar event structure to manage the
 * 				monitoring.
 * @param[out] 	psQueueItem : the incoming queue object. it might be updated
 * and may sent back
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be posted
 ******************************************************************************/
static uint8_t sensor_BeginMonitoring (sensor_CalendarEvent_struct sCalendarEvent,Kernel_QueueItem_struct * psQueueItem)
{
//	Kernel_QueueItem_struct sQueueItem;
uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED ;

	if ( SENSOR_MEASUREMENT_ISREADY == sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsConfigured.ui8field )
	{
		/* Starts  the acquisitions (stop performs a reset counter) */
		sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsRunning = KERNEL_SENSOR_ACQSTATE_BUSY;
		/* TODO: (Several sensors purpose) do not stop the cryo */
		srvtemp_StartOnchipTempAcq((bool)sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsRunning);
		/* Free the the capture compare channel */
		psCalendarBeginMeasEvent->i8CCxChNumber = SENSOR_FREE_CALENDAR_EVENT_CHANNEL;

		/* builds the free rtos message */
		sensor_tui16SensorAcqConf[0] = psCalendarBeginMeasEvent->ui8SensorId; /* the sensor id */
		sensor_tui16SensorAcqConf[1] = sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].ui8Type;
		sensor_tui16SensorAcqConf[2] = sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsRunning;

		/* sends a message to RF task in order to modify the ACQ mode system file */
		psQueueItem->ui16NbByte 		= 3;
		psQueueItem->pData 			= (uint8_t*)&sensor_tui16SensorAcqConf[0];
		psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID, KERNEL_SENSORTASKID);
		psQueueItem->ui16notification 	= KERNEL_MESSAGEID_SETBUSYSENSOR;
		//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief 		This function closes the monitoring of the sensor after
 * 				receiving a calendar event.
 * 				Sends a message to Rf Task to change the system files configuration.
 * @param[in]	sCalendarEvent: the calendar event structure to manage the
 * 				monitoring.
 * @param[out] 	psQueueItem : the incoming queue object. it might be updated
 * and may sent back
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
static uint8_t sensor_EndMonitoring (sensor_CalendarEvent_struct sCalendarEvent, Kernel_QueueItem_struct * psQueueItem)
{
	//	Kernel_QueueItem_struct sQueueItem;
	uint8_t ui8Status = CROSSRFID_MESSAGETOBEPOSTED ;

	/* Stops the acquisitions (stop performs a reset counter) */
	sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsRunning = KERNEL_SENSOR_ACQSTATE_FREE;
	/* TODO: (Several sensors purpose) do not stop the cryo */
	srvtemp_StartOnchipTempAcq((bool)sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsRunning);
	/* Free the the capture compare channel */
	psCalendarBeginMeasEvent->i8CCxChNumber = SENSOR_FREE_CALENDAR_EVENT_CHANNEL;

	/* builds the free rtos message */
	sensor_tui16SensorAcqConf[0] = psCalendarBeginMeasEvent->ui8SensorId; /* the sensor id */
	sensor_tui16SensorAcqConf[1] = sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].ui8Type;
	sensor_tui16SensorAcqConf[2] = sensor_sSensorConfig[psCalendarBeginMeasEvent->ui8SensorId].IsRunning;

	/* sends a message to RF task in order to modify the ACQ mode system file */
	psQueueItem->ui16NbByte 		= 3;
	psQueueItem->pData 			= (uint8_t*)&sensor_tui16SensorAcqConf[0];
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID, KERNEL_SENSORTASKID);
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_SETFREESENSOR;
    //xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);

	return ui8Status;
}

#endif

#if 0
/***************************************************************************//**
 * @brief 		This function notifies if the RTCC channel is free or not.
 * @param[in] 	bIsBeginEvent : true when the channel to check is this one dedicated
 * to the start date & time measurement false for the end
 * @param[out]	none
 * @return 		true if a CCx channel used to manages the monitoring (Begin/End)
 * 				is free to use, else false.
 ******************************************************************************/
static bool sensor_IsCalendarChannelFree ( bool bIsBeginEvent )
{
	bool bChannelIsFree = false;
	if ( (( true == bIsBeginEvent ) && (SENSOR_FREE_CALENDAR_EVENT_CHANNEL == psCalendarBeginMeasEvent->i8CCxChNumber)) ||
		 (( false == bIsBeginEvent ) && (SENSOR_FREE_CALENDAR_EVENT_CHANNEL == psCalendarEndMeasEvent->i8CCxChNumber))  )
	{
		bChannelIsFree = true;

	}
	else {/* do nothing */ 	}

	return bChannelIsFree;
}
#endif

/***************************************************************************//**
 * @brief 		This function returns the next AcqBeginAcqEnd calendar event
 * 				to plug on the capture compare channel.
 * @param[in] 	ui8EventType : type of the Event ( start measurement ot stop measurement)
 * @param[out]	ui8IdChannel: the Idx of the event to plug in the events array.
 * @return 		true if one event must be plugged in the rtcc capture compare,
 * 				else false.
 ******************************************************************************/
static bool sensor_IsCalendarEventToPlug ( uint8_t ui8EventType ,int8_t * pi8IdxEvent )
{
	uint8_t ui8Idx;
	uint8_t ui8Result;
	bool bOneChannelToPlug = false;

	*pi8IdxEvent = SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG; /* no channel free: SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG channel does not exist */

	/* tries to find a used RTCC capture compare channel */
	for( ui8Idx = 0; ui8Idx < SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG; ui8Idx ++ )
	{
		/* loop on the sensor which are configured and not running*/
		if (  (false == sensor_sSensorConfig[ui8Idx].IsRunning) &&
			 (((true  == sensor_sSensorConfig[ui8Idx].IsConfigured.bitfield.bIsEndDefined) && (SENSOR_CALENDAREVENT_STOPMEAS == ui8EventType) ) ||
			  ((true  == sensor_sSensorConfig[ui8Idx].IsConfigured.bitfield.bIsStartDefined) && (SENSOR_CALENDAREVENT_STARTMEAS == ui8EventType) ) )
			)
		{
			bOneChannelToPlug = true;
			if(SENSOR_CALENDAREVENT_STARTMEAS == ui8EventType)
			{
				/* compares the date/time with the previous event found */
				ui8Result = srvCalendar_CompareDateTime(	sensor_sSensorConfig[ui8Idx].ui32BeginDateValue,
															sensor_sSensorConfig[ui8Idx].ui32BeginTimeValue,
															sensor_sSensorConfig[*pi8IdxEvent].ui32BeginDateValue,
															sensor_sSensorConfig[*pi8IdxEvent].ui32BeginTimeValue);
			}
			else
			{
				/* compares the date/time with the previous event found */
				ui8Result = srvCalendar_CompareDateTime(	sensor_sSensorConfig[ui8Idx].ui32EndDateValue,
															sensor_sSensorConfig[ui8Idx].ui32EndTimeValue,
															sensor_sSensorConfig[*pi8IdxEvent].ui32EndDateValue,
															sensor_sSensorConfig[*pi8IdxEvent].ui32EndTimeValue);
			}

			/* the current date is most recent than the previous */
			if(1 == ui8Result)
			{
				/* event found */
				*pi8IdxEvent = (int8_t)ui8Idx;
			}
			else
			{  	/* Do nothing */ 	}


		}
		else
		{
			/* Do nothing */
		}
	}

	return bOneChannelToPlug;
}
#if 0
/***************************************************************************//**
 * @brief 		This function returns the first index found in the array of the
 * 				event to plug where no event is stored.
 * @param[in] 	none
 * @param[out]	pui8FreeIdxRtccEvent: the Idx where one can be stored.
 * @return 		none
 ******************************************************************************/
static void sensor_FindFreeIndexInRtccEventToPlug (uint8_t * pui8FreeIdxRtccEvent)
{
	uint8_t ui8Idx;
	bool bOneChannelFree = false;

	*pui8FreeIdxRtccEvent = SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG; /* no channel free: SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG channel does not exist */

	/* tries to find a used rtcc capture compare channel */
	for( ui8Idx = 0; ui8Idx < SENSOR_NB_MAX_RTCC_EVENT_TO_PLUG; ui8Idx ++ )
	{
		/* this index is free? */
		if( ( SENSOR_FREE_CALENDAR_EVENT_CHANNEL == sCalendarEventToPlug[ui8Idx].i8CCxChNumber ) &&
		    ( false == bOneChannelFree) )
		{
			bOneChannelFree = true;
			*pui8FreeIdxRtccEvent = ui8Idx;
		}
		else
		{
			/* Do nothing */
		}
	}
}
#endif
/***************************************************************************//**
 * @brief 		This function configures the RTCC capture compare channel
 * 				or stored the RTCC event according to its AcqBegin or AcqEnd
 * 				parameters.
 * @param[in] 	eSensorId: the sensor id of the AcqBegin or AcqEnd event to manage
 * @param[in] 	bIsBeginEvent: if it is a AcqBegin or AcqEnd event
 * @param[in] 	ui32Date: the RTCC date of the AcqBegin or AcqEnd event to manage
 * @param[in] 	ui32Time: the RTCC time of the AcqBegin or AcqEnd event to manage
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
static void sensor_ManageNewAcqBeginAcqEnd (const Kernel_Sensor_Id_enum eSensorId,
											const uint8_t ui8type,
											const uint32_t ui32Date,
											const uint32_t ui32Time)
{
uint8_t ui8IdxEventToPlug = eSensorId ;
uint8_t ui8MostRecentDate;
uint8_t uit8status = CROSSRFID_SUCCESSCODE;
uint32_t ui32DatetoCompare =0;
uint32_t ui32TimetoCompare =0;
sensor_CalendarEvent_struct *psCalendarEvent;
bool bIsRTCCchannelToUpdate = false;


	if ( SENSOR_CALENDAREVENT_STARTMEAS == ui8type )	/* when it is about the start date/time */
	{
		psCalendarEvent = psCalendarBeginMeasEvent;
		ui32DatetoCompare =sensor_sSensorConfig[ui8IdxEventToPlug].ui32BeginDateValue;
		ui32TimetoCompare =sensor_sSensorConfig[ui8IdxEventToPlug].ui32BeginTimeValue;
	}
	else if ( SENSOR_CALENDAREVENT_STOPMEAS == ui8type ) /* when it is about the end date/time */
	{
		psCalendarEvent = psCalendarEndMeasEvent;
		ui32DatetoCompare =sensor_sSensorConfig[ui8IdxEventToPlug].ui32EndDateValue;
		ui32TimetoCompare =sensor_sSensorConfig[ui8IdxEventToPlug].ui32EndTimeValue;
	}
	else
	{
		uit8status = CROSSRFID_ERROR_UNKNOWNPARAMETER;
	}

	if (CROSSRFID_SUCCESSCODE == uit8status )
	{
		/* when the RTCC channel is free */
		if ( false == psCalendarEvent->bIsEnable )
		{
			/* the RTCC channel is no longer free */
			psCalendarEvent->bIsEnable = true;

			if ( SENSOR_CALENDAREVENT_STARTMEAS== ui8type)
			{
				sensor_sSensorConfig[ui8IdxEventToPlug].bStartDateTimeIsPlugged = true;
			}
			else
			{
				sensor_sSensorConfig[ui8IdxEventToPlug].bEndDateTimeIsPlugged = true;
			}
			bIsRTCCchannelToUpdate = true;
		}
		else if ( true == psCalendarEvent->bIsEnable ) /* when an event is already plugged*/
		{
			/* compares the date/time of the event stored in the queue of event to plug
			 * with the one already plugged to the capture compare channel
			 * in order to know if this it must replace by the stored event */
			ui8MostRecentDate = srvCalendar_CompareDateTime(	psCalendarEvent->ui32Date,
																psCalendarEvent->ui32Time,
																ui32DatetoCompare,
																ui32TimetoCompare);

			if(1 == ui8MostRecentDate)
			{
				/* do  nothing */
				/* the event already plugged to the capture compare channel should not be substituted */
			}
			else
			{
				/* the event already plugged to the capture compare channel must be replaced by the event stored in the queue of event to plug */
				if ( SENSOR_CALENDAREVENT_STARTMEAS== ui8type)
				{
					sensor_sSensorConfig[psCalendarEvent->ui8SensorId ].bStartDateTimeIsPlugged = false;
					sensor_sSensorConfig[ui8IdxEventToPlug].bStartDateTimeIsPlugged = true;
				}
				else
				{
					sensor_sSensorConfig[psCalendarEvent->ui8SensorId ].bEndDateTimeIsPlugged = false;
					sensor_sSensorConfig[ui8IdxEventToPlug].bEndDateTimeIsPlugged = true;
				}
				bIsRTCCchannelToUpdate = true;
			}
		}
	}

	if (true == bIsRTCCchannelToUpdate)
	{
		/* capture compare channel <--- stored event "to plug" */
		psCalendarEvent->ui8SensorId = eSensorId;
		psCalendarEvent->ui8type = ui8type;
		psCalendarEvent->ui32Date = ui32Date;
		psCalendarEvent->ui32Time = ui32Time;
		/* Configures the capture compare channel with the new event parameters */
		srvCalendar_ConfigFuturCCxEvent(	psCalendarEvent->i8CCxChNumber,
											ui32DatetoCompare,
											ui32TimetoCompare);
	} else {/*do nothing*/}

}

/***************************************************************************//**
 * @brief 		This function manages the alarm for the thermometer
 * @param[in] 	i16Measure : the last measured temperature
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
static uint8_t sensor_ManageTempAlarm ( int16_t i16Measure)
{
	uint8_t ui8status;

	/* check the alarm state if it is enabled*/
	ui8status =  srvEM4325_IsAlarmOn (SRVTEMP_ALARMTYPE_LOW ,
										sensor_Alarm.EnableAlarm.sBitField.ui16BelowTemp,
										i16Measure ,
										sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16LowThreshold );
	if ((true == ui8status) &&
		(false == sensor_sAlarm.AckStateAlarm.sBitField.ui16BelowTemp ))  /* the RF task doesn't acknowledge this state yet*/
	{
		sensor_sAlarm.StateAlarm.sBitField.ui16BelowTemp =true; /* update the state alarm */
		srvtemp_SendLowTempAlarmState (i16Measure);
		sensor_sAlarm.AckStateAlarm.sBitField.ui16BelowTemp = true;

	}

	ui8status =  srvEM4325_IsAlarmOn ( SRVTEMP_ALARMTYPE_HIGH ,
										sensor_Alarm.EnableAlarm.sBitField.ui16UpperTemp,
										i16Measure ,
										sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16HighThreshold );
	if ((true == ui8status) &&
		(false == sensor_sAlarm.AckStateAlarm.sBitField.ui16UpperTemp ))  /* the RF task doesn't acknowledge this state yet*/
	{
		sensor_sAlarm.StateAlarm.sBitField.ui16UpperTemp =true; /* update the state alarm */
		srvtemp_SendHighTempAlarmState (i16Measure);
		sensor_sAlarm.AckStateAlarm.sBitField.ui16UpperTemp = true;
	}

	return ui8status;
}

/***************************************************************************//**
 * @brief 		This function connects a new calendar event when the RF host
 * requests a new AcqBegin or AcqEnd. The new calendar envent may replace
 * the current one when it 'll elapsed earlier
 *
 * @param[in]	ui8CalendarMeasEvent: start or stop measurement
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
static uint8_t sensor_ConnectaNewCalendarEvent ( const uint8_t ui8CalendarMeasEvent )
{
	int8_t i8IdxEventToPlug;
	uint8_t ui8status = CROSSRFID_SUCCESSCODE;
	sensor_CalendarEvent_struct *psCalendarMeasEvent ;

	/* It is a AcqBegin event? */
	if (SENSOR_CALENDAREVENT_STARTMEAS == ui8CalendarMeasEvent)
	{
		psCalendarMeasEvent = psCalendarBeginMeasEvent;
	}
	/* It is a AcqEnd event? */
	else
	{
		psCalendarMeasEvent = psCalendarEndMeasEvent;
	}

	/* a new capture compare calendar event must be plugged? */
	if(true == sensor_IsCalendarEventToPlug(psCalendarMeasEvent->ui8type, &i8IdxEventToPlug) )
	{
		/* fills the new configuration of the event (capture compare channel, sensor id, date, time, etc.) */
		psCalendarMeasEvent->bIsEnable 		= true;
		psCalendarMeasEvent->ui8SensorId 	= i8IdxEventToPlug;
		psCalendarMeasEvent->ui8type 		= sensor_sSensorConfig[i8IdxEventToPlug].ui8Type;
		if (SENSOR_CALENDAREVENT_STARTMEAS == sensor_sSensorConfig[i8IdxEventToPlug].ui8Type)
		{
			psCalendarMeasEvent->ui32Date = sensor_sSensorConfig[i8IdxEventToPlug].ui32BeginDateValue;
			psCalendarMeasEvent->ui32Time = sensor_sSensorConfig[i8IdxEventToPlug].ui32BeginTimeValue;
		}
		else
		{
			psCalendarMeasEvent->ui32Date = sensor_sSensorConfig[i8IdxEventToPlug].ui32EndDateValue;
			psCalendarMeasEvent->ui32Time = sensor_sSensorConfig[i8IdxEventToPlug].ui32EndTimeValue;

		}
		/* Configures the capture compare according to the date and the time of the event */
		srvCalendar_ConfigFuturCCxEvent(	psCalendarBeginMeasEvent->i8CCxChNumber,
											psCalendarBeginMeasEvent->ui32Date,
											psCalendarBeginMeasEvent->ui32Time);

	}
	else
	{
		/* no event to plug to a RTCC capture compare channel: the channel is not free */
		psCalendarMeasEvent->bIsEnable = false;
	}

	return ui8status;
}

/*===========================================================================================================
						Public functions definition
===========================================================================================================*/

/***************************************************************************//**
 * @brief  		This function processes the initialization of serial task
 * @param[in] 	pui16Data: the data sent by the RF task (sensor configuration)
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void sensor_ProcessInit ( uint16_t * pui16Data )
{
uint8_t status;
uint16_t *pui16BoardVersion;
Kernel_QueueItem_struct sQueueItem = { 	KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID) ,
										KERNEL_MESSAGEID_UPDATEALARMSTATE, 	/* sample to get */
										0,NULL};

	sensor_init(pui16Data);			/* Initializes the serial task */
	srvhtu21d_init (false);			/* initialize the interface for the Si7021*/
	letimerdrv_init ();
	logtimer_ResetLogicalTimer ();
    /* when the 2nd spi interface is available for the communication to the external sensor*/

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	//srvlis3mdl_EnableMagneticDetection (false);
	//srvActRec_EnableMotionDetection (false); 	 	/* initialize the Adxl363 sensor */
#elif (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)
	srvlis3mdl_EnableMagneticDetection (false);
	srvBubble_EnableMotionDetection (true); 	 	/* initialize the Adxl363 sensor */
#else
	srvlis3mdl_EnableMagneticDetection (false);
	srvActRec_EnableMotionDetection (false); 	 	/* initialize the Adxl363 sensor */
	srvActRec_InitActivityRecorder ();
#endif

	memset (&sensor_sAlarm,0,sizeof (sensor_Alarm_struct));					/* init the alarm */
	status = srv24LC_ReadBoardVersion(  &pui16BoardVersion  );				/* read the board versions in the external memory*/
	if ((CROSSRFID_ERROR_EXTEEPROM_CRC == status) &&						/* when the version of the firmware is not read in the external memory*/
		(COMMON_VERSION_FIRMWARE != pui16BoardVersion[SENSOR_FIRMAWAREVERSION]))
	{
		srv24LC_WriteBoardVersion ();	/* Write the board version*/
		sleep_DelayTask (PRT24LC64_WRITECYCLEMS, configSLEEP_MODE);
		sensor_InitAlarms ( );			/* Initialize the Alarm field of the EEPROM*/

		/* stay in EM1 during the write cycle time of the memory */
		sleep_DelayTask (PRT24LC64_WRITECYCLEMS, configSLEEP_MODE);

		xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
	}
	else if ((CROSSRFID_SUCCESSCODE == status) &&
			(COMMON_VERSION_FIRMWARE != pui16BoardVersion[SENSOR_FIRMAWAREVERSION]) )/* when the version of the firmware is not read in the external memory*/
	{
		srv24LC_WriteBoardVersion ();	/* Write the board version*/
		sleep_DelayTask (PRT24LC64_WRITECYCLEMS, configSLEEP_MODE); /* stay in EM1 during the write cycle time of the memory */
	}
	else
	{
		status = sensor_ReadAlarmBitFieldInExtMemory ();			/* reads the alarm states in the external memory */
		if(CROSSRFID_SUCCESSCODE == status)
		{
			sensor_sAlarm.StateAlarm.ui16Value = sensor_tui16AlarmDataInEeprom[SENSOR_ALARMBITFIELD];
			/* sends message to RF task in order to change the Alarm system files */
			sQueueItem.ui16NbByte 		= (SENSOR_SIZE_16BITBUFFER*2);
			//sQueueItem.pData 			= (uint8_t*)&sensor_tui16AlarmDataInEeprom[0];	 /* sends the read values in the external eeprom (about alarms) */
			sQueueItem.pData 			= (uint8_t*)&(sensor_sAlarm.StateAlarm.ui16Value);	 /* sends the read values in the alarm status  */
			sQueueItem.ui16notification = KERNEL_MESSAGEID_UPDATEALARMSTATE;
		}
		else
		{
			/* unexpected behavior : Stop the firmware and debug*/
			/* sends a message to kernel to sleep the uC */
			sQueueItem.ui16NbByte 		= 0;
			sQueueItem.pData 			= NULL;
			sQueueItem.urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID, KERNEL_SENSORTASKID);
			sQueueItem.ui16notification 	= KERNEL_MESSAGEID_STOPFIRMWARE;

		}
		xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
	}


}

#if (USE_TEMPERATURE_UC == 1)
/***************************************************************************//**
 * @brief
 *   This function start the management of the alarms. It is called after the
 *   firmware initialization.
 * @param[in]	none
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
void sensor_StartAlarmManagement (void)
{
	/* Update the EnableAlarm configuration */
	if (KERNEL_ALARM_ID_ENABLE == sensor_Alarm.EnableAlarm.sBitField.ui16BelowTemp)
	{
		/* Enables the below alarm of the OnChip temperature sensor according to the threshold in serial configuration */
		srvtemp_SetTempAlarmCB(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16LowThreshold ,false);
	}

	if (KERNEL_ALARM_ID_ENABLE == sensor_Alarm.EnableAlarm.sBitField.ui16UpperTemp)
	{
		/* Enables the upper alarm of the OnChip temperature sensor according to the threshold in serial configuration */
		srvtemp_SetTempAlarmCB(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16HighThreshold ,true);
	}

	if (KERNEL_ALARM_ID_ENABLE == sensor_Alarm.EnableAlarm.sBitField.ui16LowBattery)
	{

	}
}
#endif

/***************************************************************************//**
 * @brief	   This function gets the temperature and sends its value to rf task
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessGetTemperature ( 	Kernel_QueueItem_struct  * psQueueItem  )
{
	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	int16_t i16Temperature = 0;

#if USE_TEMPERATURE_EM4325 ==1

	/* Stores the temperature in the data to send to rf task */
	ui8status =  srvEM4325_MeasureTemperature (&i16Temperature);
	sensor_ManageTempAlarm ( i16Temperature );

#else
	//ui8status = srvhtu21d_MeasureTemperature (&sensor_ui16DataToSendToRfTask);
	srvtemp_MeasureOnchipTemp((uint8_t*)&sensor_ui16DataToSendToRfTask);
#endif
	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		psQueueItem->ui16NbByte 		= 2;
		/* Send the temperature sample */
		sensor_ui16DataToSendToRfTask 	= (uint16_t)i16Temperature;
		psQueueItem->pData 				= (uint8_t *) (&sensor_ui16DataToSendToRfTask);
	}
	else
	{
		psQueueItem->ui16NbByte 		= 0;
		psQueueItem->pData 				= NULL;
	}
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_GETTEMP;
	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	/* sends the message to the RF task */
	//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);

	return ui8status;
}

/***************************************************************************//**
 * @brief
 *   This function gets the number of data available and sends its value to the
 *   RF task
 * @details
 * |____________________________________________________________________________|
 * | 									word 1									|
 * |____________________________________________________________________________|
 * | 	sensor ID																|
 * | 		0x0000 � temperature												|
 * | 		0x0001 - accelerometer												|
 * |		0x0002 - magnetometer												|
 * |		Others - RFU														|
 * |____________________________________________________________________________|
 * @param[in] 	none
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_MESSAGETOBEPOSTED
 ******************************************************************************/
uint8_t sensor_ProcessIsDataAvailable ( Kernel_QueueItem_struct *sQueueItem )
{
uint8_t ui8status  = CROSSRFID_MESSAGETOBEPOSTED;
uint16_t ui16SensorId = (uint16_t ) (*sQueueItem->pData);
uint16_t ui16MeasId = (uint16_t ) (*(sQueueItem->pData+2));

	sQueueItem->urecvsender 		=  KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	sQueueItem->ui16NbByte 			= 4;
	sQueueItem->ui16notification 	= KERNEL_MESSAGEID_ISDATAAVAILABLE;

	/* Gets the number of data in the buffer */
	sensor_GetNbMeas (ui16SensorId , ui16MeasId, (uint16_t*)(&sensor_ui16DataToSendToRfTask));
	sQueueItem->pData = (uint8_t*)&sensor_ui16DataToSendToRfTask;

	return ui8status;
}

/***************************************************************************//**
 * @brief 		This function sends the sample buffer to the RF task
 * @details
 * @details
 * |____________________________________________________________________________|
 * | 									word 1									|
 * |____________________________________________________________________________|
 * | 	sensor ID																|
 * | 		0x0000 � temperature												|
 * | 		0x0001 - accelerometer												|
 * |		0x0002 - magnetometer												|
 * |		Others - RFU														|
 * |____________________________________________________________________________|
 * | 									word 2									|
 * |____________________________________________________________________________|
 * | 	measurement ID (optional)												|
 * | 		0x0000 � raw														|
 * | 		0x0001 - FFT														|
 * |		Others - RFU														|
 * |____________________________________________________________________________|
 * @param[in] 	none
 * @param[out] 	sQueueItem : the incoming and outgoing queue message
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a queue message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessGiveMeSamples ( Kernel_QueueItem_struct *sQueueItem )
{
uint8_t ui8status  			= CROSSRFID_MESSAGETOBEPOSTED;
uint16_t ui16SensorId 		= (uint16_t ) (*sQueueItem->pData);
uint16_t ui16MeasurementId 	= (uint16_t ) (*(sQueueItem->pData+2));
int8_t *pi8MeasBuffer;

	sQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	sQueueItem->ui16NbByte 			= 1;
	sQueueItem->ui16notification 	= KERNEL_MESSAGEID_GIVESAMPLES;

	sensor_GetMeasBuffer (ui16SensorId, ui16MeasurementId, &pi8MeasBuffer );
	sQueueItem->pData 				= (uint8_t *) pi8MeasBuffer;
	return ui8status;
}

/***************************************************************************//**
 * @brief 		This function enables or disables the alarms according to the rf task message
 *
 * @detail
 * 	The message sent from RF task:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	enable alarm system file				|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in] 	pui16Data: information about alarm to enable/disable
 * @param[out]	none
 * @return 		none
 ******************************************************************************/
uint8_t sensor_ProcessEnableDisableAlarm ( Kernel_QueueItem_struct * psQueueItem )
{
	uint8_t ui8status  = CROSSRFID_MESSAGETOBEPOSTED;
	uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;
	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte 		= 3;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ENABLEALARM;
	sensor_bProcessIsSuccess 	= true;
	psQueueItem->pData  			= (uint8_t*)&sensor_bProcessIsSuccess;

	/* Enable or Disable the alarms by making a distinction between the previous and the new enableAlarm word sent by the RF task */
	sensor_EnableDisableAlarms (pui16Data[0]);

	/* Updates the enable-alarm configuration value */
	sensor_Alarm.EnableAlarm.ui16Value = pui16Data[0];

	/* sends the acknowledge message to the RF task */
	//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
	return ui8status;
}

/***************************************************************************//**
 * @brief 		This function resets the alarms according to the RF task message
 *
 * @detail
 * 	The message sent from RF task:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	reset alarm system file					|
 *  ---------------------------------------------------------------------------------
 *
 * @param[in] 	pui16Data: information about the alarms to reset
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
uint8_t sensor_ProcessResetAlarm ( Kernel_QueueItem_struct * psQueueItem )
{
	uint8_t ui8status;
	uint16_t ui16alarmId = psQueueItem->pData[0];

	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND(KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_RESETALARM;

	/* reset the alarms by making a distinction between the previous and the new enableAlarm word sent by RF task */
	/* update the content of the 24LC64 external EEPROM */
	ui8status = sensor_ResetAlarms (ui16alarmId);

	if(CROSSRFID_SUCCESSCODE == ui8status)
	{
		psQueueItem->ui16NbByte = SENSOR_SIZE_16BITBUFFER; /* tested at the reception of the message by rf task because pData already used (to acknowledge) */
		psQueueItem->pData = (uint8_t*)(sensor_tui16AlarmDataInEeprom);
	}
	else
	{
		psQueueItem->ui16NbByte = 0;
		psQueueItem->pData = NULL;
	}

	/* stay in EM1 during the write cycle time of the memory */
	sleep_DelayTask (PRT24LC64_WRITECYCLEMS,configSLEEP_MODE);
#if 0
	srv24LC_Enable24LC64(false);
#endif
	ui8status  = CROSSRFID_MESSAGETOBEPOSTED;
	return ui8status;
}

/***************************************************************************//**
 * @brief		This function sends its process status to RF task and updates the
 *   acquisition configuration according to the parameters sent by RF task
 * @details
 * |____________________________________________________________________________|
 * | 									word 1									|
 * |____________________________________________________________________________|
 * | 	sensor ID																|
 * | 		0x0000 � temperature												|
 * | 		0x0001 - accelerometer												|
 * |		0x0002 - magnetometer												|
 * |		Others - RFU														|
 * |____________________________________________________________________________|
 * | 									word 2									|
 * |____________________________________________________________________________|
 * | 	0x0000 : one shot 														|
 * | 	0x0001 : periodic 														|
 * |____________________________________________________________________________|
 * | 									word 3									|
 * |____________________________________________________________________________|
 * | 	When the RF host reads it												|
 * | 		0x0000 � Free to use												|
 * | 		0x0001 � In Progress												|
 * | 		Others � RFU														|
 * | 	When the RF host writes it												|
 * | 		0x0000 � wait calendar event										|
 * | 		0x0001 � start														|
 * | 		0x0002 � stop				 										|
 * | 		Others � RFU														|
 * |____________________________________________________________________________|
 * @param[in] 	psQueueItem : the data field of the structure contains the parameters
 * of the host request
 * @param[out] 	psQueueItem : the queue object might be updated and should be
 * sent back to an another task
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessAcqMode ( Kernel_QueueItem_struct * psQueueItem )
{
	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;	/* sends the acknowledge message to the RF task */
	uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;
	uint8_t ui8SensorId = (uint8_t )psQueueItem->pData[0];

	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte 		= 1;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ACQMODE;

	if (ui8SensorId < KERNEL_SENSOR_ID_NBSENSOR)
	{
		sensor_bProcessIsSuccess 		= true;
		/* Updates the acquisition configuration */
		switch(ui8SensorId) /* = sensor id */
		{
			case KERNEL_SENSOR_ID_TEMPERATURE:
				sensor_sSensorConfig[ui8SensorId].ui8Type = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
				//sensor_sSensorConfig[ui8SensorId].IsRunning = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
				sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsModeDefined = true;

				if (( KERNEL_SENSOR_ACQMODE_STOP == pui16Data[KERNEL_MSG_IDX_ACQ_MODE] ) &&
					( KERNEL_SENSOR_ACQTYPE_PERIODIC == sensor_sSensorConfig[ui8SensorId].ui8Type ))
				{
					logtimer_DisableLogicalTimer (ui8SensorId);
					sensor_sSensorConfig[ui8SensorId].IsRunning = false ;
				}
#if 0
				else if (( KERNEL_SENSOR_ACQMODE_START == pui16Data[KERNEL_MSG_IDX_ACQ_MODE] ) &&
						( KERNEL_SENSOR_ACQTYPE_PERIODIC == sensor_sSensorConfig[ui8SensorId].ui8Type ))
				{
					ui8status  = logtimer_EnableLogicalTimer (ui8SensorId);
					if (CROSSRFID_SUCCESSCODE == ui8status)
					{
						sensor_sSensorConfig[ui8SensorId].IsRunning = true ;
					}
				} else {/*do nothing*/}
#endif

#if USERTHECRYOTIMER == 1
				/* Starts or Stops the onchip acquisitions (stop performs a reset counter) */
				srvtemp_StartOnchipTempAcq((bool) sensor_sSensorConfig[ui8SensorId].IsRunning);
#else /* by using the RTCCC driver */
				/* nothing to do because we are waiting the calendar event */
				/* todo : add the immediate start */
				/* todo : add the immediate stop */
#endif

				break;

			case KERNEL_SENSOR_ID_ACCELERO:
			case KERNEL_SENSOR_ID_MAGNETO:
				sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsModeDefined = true;
				sensor_sSensorConfig[ui8SensorId].ui8Type = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
				//sensor_sSensorConfig[ui8SensorId].IsRunning = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
			break;

			default:
				break;
		}
	}
	else
	{
		sensor_bProcessIsSuccess = false;
	}
	ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	psQueueItem->pData = (uint8_t*)&sensor_bProcessIsSuccess;

	return ui8status ;
}

/***************************************************************************//**
 * @brief   	This function sends  its process status to RF task and updates the
 *   acquisition configuration according to the parameters sent by rf task
 * @details 	the format of the acqperiod
 * |____________________________________________________________________________|
 * | 									word 1									|
 * |____________________________________________________________________________|
 * | 	sensor ID																|
 * | 		0x0000 � temperature												|
 * | 		0x0001 - accelerometer												|
 * |		0x0002 - magnetometer												|
 * |		Others - RFU														|
 * |____________________________________________________________________________|
 * | 									word 2									|
 * |____________________________________________________________________________|
 * | 	value 																	|
 * |____________________________________________________________________________|
 * | 									word 3									|
 * |____________________________________________________________________________|
 * | 	When the measurement is configured as periodic (ACQMode register word 2)|
 * | 		0x0000 - us															|
 * | 		0x0001 - ms															|
 * | 		0x0002 - s															|
 * | 		0x0003 - min														|
 * | 		0x0004 - hour														|
 * | 		0x0005 � day														|
 * | 		Otherwise don�t care												|
 * |____________________________________________________________________________|
 * @param[in] 	psQueueItem : the data field of the structure contains the parameters
 * of the host request
 * @param[out] 	psQueueItem : the queue object might be updated and should be
 * sent back to an another task
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessAcqPeriod (Kernel_QueueItem_struct * psQueueItem)
{
	//Kernel_QueueItem_struct sQueueItem;
	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;  /* sends the acknowledge message to the RF task */
	uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;
	uint8_t ui8SensorId = (uint8_t )psQueueItem->pData[0];
	//uint32_t ui32timerId =0;

	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte 			= 1;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ACQPERIOD;
	psQueueItem->pData  			= (uint8_t*)&sensor_bProcessIsSuccess;


	if (ui8SensorId < KERNEL_SENSOR_ID_NBSENSOR)
	{
		sensor_bProcessIsSuccess 		= true;
		/* Updates the acquisition configuration */
		switch(ui8SensorId) /* = sensor id */
		{
			case KERNEL_SENSOR_ID_TEMPERATURE:
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
				if(false == sensor_bTempIsAlreadyStarted)
				{
#endif
				//sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsPeriodDefined = true;
				sensor_sSensorConfig[ui8SensorId].ui16PeriodValue = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
				sensor_sSensorConfig[ui8SensorId].ui16PeriodUnit = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
				/* Sets the new period of the onchip acquisitions */
#if USERTHECRYOTIMER == 1
				sensor_bProcessIsSuccess = srvtemp_SetPeriodOnchipTemp(	sensor_sSensorConfig[ui8SensorId].ui16PeriodValue,
																		sensor_sSensorConfig[ui8SensorId].ui16PeriodUnit);
#else /* use the logical timer */

				sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsPeriodDefined = true;

				if (sensor_sSensorConfig[ui8SensorId].IsRunning == true) /* when the timer is running*/
				{
					logtimer_DisableLogicalTimer (sensor_sSensorConfig[ui8SensorId].ui8TimerId); 	/* disable it*/
					logtimer_inittimer ( 	ui8SensorId ,
											sensor_sSensorConfig[ui8SensorId].ui8TimerId,			/* reinitiate it with the new period*/
											sensor_sSensorConfig[ui8SensorId].ui8Type,
											sensor_sSensorConfig[ui8SensorId].ui16PeriodValue,
											sensor_sSensorConfig[ui8SensorId].ui16PeriodUnit);
					logtimer_EnableLogicalTimer (sensor_sSensorConfig[ui8SensorId].ui8TimerId); 	/* enable it */
				} else {/* do nothing wait the calendar event*/}

#endif
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
				}
				else
				{
					/* impossible to change the period, the temperature monitoring has been already started */
					sensor_bProcessIsSuccess = false;
				}
#endif
				break;

			case KERNEL_SENSOR_ID_ACCELERO:
			case KERNEL_SENSOR_ID_MAGNETO:
				sensor_sSensorConfig[ui8SensorId].ui16PeriodValue = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
				sensor_sSensorConfig[ui8SensorId].ui16PeriodUnit = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];

				sensor_sSensorConfig[ui8SensorId].ui8TimerId = (uint8_t) ui8SensorId;
				sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsPeriodDefined = true;

				break;

			default:
				break;
		}
	}
	return ui8status ;
}

/***************************************************************************//**
 * @brief 		This function sends its process status to RF task and updates the
 *   acquisition configuration according to the parameters sent by the RF task
 * @param[in] 	psQueueItem : the data field of the structure is the data
 * to update the date and time when the acquisitions will begin / has begun.
 * @param[out] 	psQueueItem : the queue object might be updated and should be
 * sent back to an another task
 * @details 	the format of the acqbegin date & time
 * |________________________________________|___________________________________|
 * | 			byte 0 						| 				byte 1				|
 * |________________________________________|___________________________________|
 * | 	month ten	|		month unit		| 		day ten		|	day unit	|
 * |________________|_______________________|___________________________________|
 * | 			byte 2		 				| 				byte 3 				|
 * |________________________________________|___________________________________|
 * | 	RFU			|		RFU				| 		year ten	|	year unit	|
 * |________________|_______________________|___________________________________|
 * | 			byte 4		 				| 				byte 5				|
 * |________________________________________|___________________________________|
 * | 	min ten		|		min unit 		| 		second ten	|	second unit	|
 * |________________|_______________________|___________________________________|
 * | 			byte 6		 				| 				byte 7				|
 * |________________________________________|___________________________________|
 * | 	RFU			|		RFU				| 		hour ten	|	hour unit	|
 * |________________|_______________________|___________________________________|
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessAcqBegin (Kernel_QueueItem_struct * psQueueItem)
{
#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;  /* sends the acknowledge message to the RF task */
uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;
uint8_t ui8SensorId = (uint8_t )psQueueItem->pData[0];

	bool bDataCoherence= false ;

	if(KERNEL_ACQ_BEGIN_END_INFINITE_VALUE == (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16)))
	{
		bDataCoherence = false; /* not implemented yet for the ACQ BEGIN, just for the ACQ END*/
	}
	else
	{
		bDataCoherence = srvCalendar_IsCoherentDate ((uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16)));
	}

	if (( true == bDataCoherence ) && (ui8SensorId < KERNEL_SENSOR_ID_NBSENSOR))
	{
		/* Updates the acquisition configuration */
		switch(ui8SensorId) /* = sensor id */
		{
			case KERNEL_SENSOR_ID_MAGNETO:
			case KERNEL_SENSOR_ID_TEMPERATURE:
			case KERNEL_SENSOR_ID_ACCELERO:
				/* if it is the temperature sensor id, test if the monitoring has been already started */
				if((false == sensor_bTempIsAlreadyStarted) ||
				   ((true == sensor_bTempIsAlreadyStarted) && (KERNEL_SENSOR_ID_TEMPERATURE != ui8SensorId)))
				{
					/* ACQ BEGIN is defined now */
					sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsStartDefined = true;
					sensor_bProcessIsSuccess = true ;

					/* Stores the new date, time */
					sensor_sSensorConfig[ui8SensorId].ui32BeginDateValue
								= (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16));
					sensor_sSensorConfig[ui8SensorId].ui32BeginTimeValue
								= (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_TIME_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_TIME_MSW] & 0x0000FFFF) << 16));

					/* This event is not plugged yet to the BEGIN channel (Rtcc) */
					sensor_sSensorConfig[ui8SensorId].bStartDateTimeIsPlugged = false;

					/* configure the RTCC capture compare channel or stored this RTCC event */
					sensor_ManageNewAcqBeginAcqEnd (ui8SensorId,
													SENSOR_CALENDAREVENT_STARTMEAS,
													sensor_sSensorConfig[ui8SensorId].ui32BeginDateValue,
													sensor_sSensorConfig[ui8SensorId].ui32BeginTimeValue);
				}
				/* the temperature monitoring has been already launched */
				else
				{
					/* temperature acquisitions still in progress, should not be stopped */
					sensor_bProcessIsSuccess = false;
				}
				break;

			default:
				break;
		}
	}
	else
	{
		sensor_bProcessIsSuccess = false;
	}

	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte 		= 1;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ACQBEGIN;
	psQueueItem->pData  			= (uint8_t*)&sensor_bProcessIsSuccess;
#else
uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;  /* sends the acknowledge message to the RF task */
uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;
uint8_t ui8SensorId = (uint8_t )psQueueItem->pData[0];

	bool bDataCoherence= false ;

	if(KERNEL_ACQ_BEGIN_END_INFINITE_VALUE == (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16)))
	{
		bDataCoherence = false; /* not implemented yet for the ACQ BEGIN, just for the ACQ END*/
	}
	else
	{
		bDataCoherence = srvCalendar_IsCoherentDate ((uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16)));
	}

	if (( true == bDataCoherence ) && (ui8SensorId < KERNEL_SENSOR_ID_NBSENSOR))
	{
		/* Updates the acquisition configuration */
		switch(ui8SensorId) /* = sensor id */
		{
			case KERNEL_SENSOR_ID_MAGNETO:
			case KERNEL_SENSOR_ID_TEMPERATURE:
			case KERNEL_SENSOR_ID_ACCELERO:
				/* ACQ BEGIN is defined now */
				sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsStartDefined = true;
				sensor_bProcessIsSuccess = true ;

				/* Stores the new date, time */
				sensor_sSensorConfig[ui8SensorId].ui32BeginDateValue
							= (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16));
				sensor_sSensorConfig[ui8SensorId].ui32BeginTimeValue
							= (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_TIME_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_TIME_MSW] & 0x0000FFFF) << 16));

				/* This event is not plugged yet to the BEGIN channel (Rtcc) */
				sensor_sSensorConfig[ui8SensorId].bStartDateTimeIsPlugged = false;

				/* configure the RTCC capture compare channel or stored this RTCC event */
				sensor_ManageNewAcqBeginAcqEnd (ui8SensorId,
												SENSOR_CALENDAREVENT_STARTMEAS,
												sensor_sSensorConfig[ui8SensorId].ui32BeginDateValue,
												sensor_sSensorConfig[ui8SensorId].ui32BeginTimeValue);
				//else
				{ /* do nothing*/	}

				break;

			default:
				break;
		}
	}
	else
	{
		sensor_bProcessIsSuccess 	= false;
	}

	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte 		= 1;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ACQBEGIN;
	psQueueItem->pData  			= (uint8_t*)&sensor_bProcessIsSuccess;
#endif
	return ui8status ;
}

/***************************************************************************//**
 * @brief
 *   This function sends its process status to rf task and updates the
 *   acquisition configuration according to the parameters sent by rf task
  * @details 	the format of the acqend date & time
 * |________________________________________|___________________________________|
 * | 			byte 0 						| 				byte 1				|
 * |________________________________________|___________________________________|
 * | 	month ten	|		month unit		| 		day ten		|	day unit	|
 * |________________|_______________________|___________________________________|
 * | 			byte 2		 				| 				byte 3 				|
 * |________________________________________|___________________________________|
 * | 	RFU			|		RFU				| 		year ten	|	year unit	|
 * |________________|_______________________|___________________________________|
 * | 			byte 4		 				| 				byte 5				|
 * |________________________________________|___________________________________|
 * | 	min ten		|		min unit 		| 		second ten	|	second unit	|
 * |________________|_______________________|___________________________________|
 * | 			byte 6		 				| 				byte 7				|
 * |________________________________________|___________________________________|
 * | 	RFU			|		RFU				| 		hour ten	|	hour unit	|
 * |________________|_______________________|___________________________________|
 * @param[in] 	psQueueItem the data field to update the date and time when the
 * acquisitions will finish / has finished
 * @param[out] 	psQueueItem : the queue object might be updated and should be
 * sent back to an another task
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessAcqEnd (Kernel_QueueItem_struct * psQueueItem)
{
	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;  /* sends the acknowledge message to the RF task */
	uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;
	uint8_t ui8SensorId = (uint8_t )psQueueItem->pData[0];

	bool bDataCoherence= false ;
	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);

	bDataCoherence = srvCalendar_IsCoherentDate ((uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16)));
	if (( true == bDataCoherence ) && (ui8SensorId < KERNEL_SENSOR_ID_NBSENSOR))
	{
		sensor_bProcessIsSuccess 	= true;
		/* Updates the acquisition configuration */
		switch(ui8SensorId) /* = sensor id */
		{
			case KERNEL_SENSOR_ID_ACCELERO:
			case KERNEL_SENSOR_ID_TEMPERATURE:
			case KERNEL_SENSOR_ID_MAGNETO:
				/* ACQ END is defined now */
				sensor_sSensorConfig[ui8SensorId].IsConfigured.bitfield.bIsEndDefined = true;

				/* Stores the new date, time */
				sensor_sSensorConfig[ui8SensorId].ui32EndDateValue
				= (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_DATE_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_DATE_MSW] & 0x0000FFFF) << 16));
				sensor_sSensorConfig[ui8SensorId].ui32EndTimeValue
				= (uint32_t)(pui16Data[KERNEL_MSG_IDX_ACQ_TIME_LSW] | (((uint32_t)pui16Data[KERNEL_MSG_IDX_ACQ_TIME_MSW] & 0x0000FFFF) << 16));

				/* ----------- if the DATE & TIME == 0: infinite monitoring ----------- */
				if((KERNEL_ACQ_BEGIN_END_INFINITE_VALUE == sensor_sSensorConfig[ui8SensorId].ui32EndDateValue)
					&& (KERNEL_ACQ_BEGIN_END_INFINITE_VALUE == sensor_sSensorConfig[ui8SensorId].ui32EndTimeValue))
				{
					/* if this ACQEND event is already plugged, we should unplug it */
					if(true == sensor_sSensorConfig[ui8SensorId].bEndDateTimeIsPlugged)
					{
						/* change the software variable to notify that this event is not plugged */
						sensor_sSensorConfig[ui8SensorId].bEndDateTimeIsPlugged = false;

						psCalendarEndMeasEvent->bIsEnable = false;

						/* unplug this ACQEND event by writing 0 in the capture compare values */
						srvCalendar_ConfigFuturCCxEvent(	psCalendarEndMeasEvent->i8CCxChNumber,
															sensor_sSensorConfig[ui8SensorId].ui32EndDateValue,
															sensor_sSensorConfig[ui8SensorId].ui32EndTimeValue);
					}
					/* if this ACQEND event is not already plugged */
					else
					{
						/* Do nothing */
					}

					/* This event is not plugged yet to the END channel (Rtcc) */
					sensor_sSensorConfig[ui8SensorId].bEndDateTimeIsPlugged = false;

					/* Connect a new ACQEND calendar event if one is pending */
					sensor_ConnectaNewCalendarEvent(SENSOR_CALENDAREVENT_STOPMEAS);
				}
				/* ----------- ACQ END event to store (and may be plug to the ACQ END rtcc channel ----------- */
				else
				{
					/* configure the RTCC capture compare channel or stored this rtcc event */
					sensor_ManageNewAcqBeginAcqEnd (ui8SensorId,
													SENSOR_CALENDAREVENT_STOPMEAS,
													sensor_sSensorConfig[ui8SensorId].ui32EndDateValue,
													sensor_sSensorConfig[ui8SensorId].ui32EndTimeValue);
				}

				break;

			default:
				break;
		}
	}
	else
	{
		sensor_bProcessIsSuccess 	= false;
	}

	psQueueItem->ui16NbByte 		= 1;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ACQEND;
	psQueueItem->pData  			= (uint8_t*)&sensor_bProcessIsSuccess;
	/* sends the acknowledge message to the RF task */
	//xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
	return ui8status ;
}

/***************************************************************************//**
 * @brief
 *   This function sends its process status to RF task and updates the
 *   acquisition configuration according to the parameters sent by rf task
 *
 * @param[in] 	psQueueItem: data to update the threshold
 * @param[out] 	psQueueItem : the queue object might be updated and should be
 * sent back to an another task
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessAcqThresholds (Kernel_QueueItem_struct * psQueueItem)
{
	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;  /* sends the acknowledge message to the RF task */
	uint16_t * pui16Data = (uint16_t *)psQueueItem->pData;

	sensor_bProcessIsSuccess 	= true;
	/* initializes the queue */
	psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
	psQueueItem->ui16NbByte 		= 1;
	psQueueItem->ui16notification 	= KERNEL_MESSAGEID_ACQTHRESHOLD;
	psQueueItem->pData  			= (uint8_t*)&sensor_bProcessIsSuccess;

	/* Updates the acquisition configuration */
	switch(pui16Data[0]) /* = sensor id */
	{
		case KERNEL_SENSOR_ID_TEMPERATURE:
			/* if the alarm is enable */
			if(KERNEL_ALARM_ID_ENABLE == sensor_Alarm.EnableAlarm.sBitField.ui16BelowTemp)
			{
				/* if the alarm threshold value changes */
				if(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16LowThreshold != pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD])
				{
					/* updates the threshold value and changes the threshold of the alarm callback */
					sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16LowThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
#if USE_TEMPERATURE_UC==1
					srvtemp_RemoveTempAlarmCB(false);
					srvtemp_SetTempAlarmCB(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16LowThreshold,false);
#endif
				}
			}
			/* if the alarm is enable */
			if(KERNEL_ALARM_ID_ENABLE == sensor_Alarm.EnableAlarm.sBitField.ui16UpperTemp)
			{
				/* if the alarm threshold value changes */
				if(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16HighThreshold != pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD])
				{
					/* updates the threshold value and changes the threshold of the alarm callback */
					sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16HighThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
#if USE_TEMPERATURE_UC==1
					srvtemp_RemoveTempAlarmCB(true);
					srvtemp_SetTempAlarmCB(sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].i16HighThreshold,true);
#endif
				}
			}
			break;

		case KERNEL_SENSOR_ID_ACCELERO:
			sensor_sSensorConfig[KERNEL_SENSOR_ID_ACCELERO].i16LowThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
			sensor_sSensorConfig[KERNEL_SENSOR_ID_ACCELERO].i16HighThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
			break;

		case KERNEL_SENSOR_ID_MAGNETO:
			sensor_sSensorConfig[KERNEL_SENSOR_ID_MAGNETO].i16LowThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_TYPE_OR_PERIOD];
			sensor_sSensorConfig[KERNEL_SENSOR_ID_MAGNETO].i16HighThreshold = pui16Data[KERNEL_MSG_IDX_ACQ_STATE_OR_UNITPRIOD];
			break;

		default:
			break;
	}

	/* sends the acknowledge message to the RF task */
	// xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem,0);
	return ui8status;
}

/***************************************************************************//**
 * @brief
 *   This function gets and stores a new temperature sample in a circular buffer
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void sensor_StoreNewSample (void)
{
#if 0 /* not used anymore*/
	Kernel_QueueItem_struct sQueueItem;

	/* No Overflow set ? and temperature monitoring in progress ?*/
	if((false == sensor_sSample.bOverFlow) && (KERNEL_SENSOR_ACQSTATE_BUSY == sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning))
	{
#if USE_TEMPERATURE_EM4325
		srvEM4325_MeasureTemperature (&(sensor_sSample.ti16Data[sensor_sSample.ui16Count]));
#else
		/* gets the on chip temperature */
		srvtemp_MeasureOnchipTemp((int8_t*)&(sensor_sSample.ti16Data[sensor_sSample.ui16Count]));
#endif
		/* still increasing the number of samples in the buffer */
		sensor_sSample.ui16Count++;

		/* samples Overflow ? */
		if (sensor_sSample.ui16Count >= KERNEL_COMMBUFFER_LENGTH)
		{
			sensor_sSample.bOverFlow = true;

			/* monitoring state = stopped in the serial task */
			sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning = KERNEL_SENSOR_ACQSTATE_FREE;

			/* TODO: (Several sensors purpose) Do not stop the cryo but test the AcqMode.state */
			/* Stops the timer triggering the acquisitions */
			srvtemp_StartOnchipTempAcq((bool)sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning);

			/* builds the message */
			sensor_tui16SensorAcqConf[0] 	= KERNEL_SENSOR_ID_TEMPERATURE; /* the sensor id */
			sensor_tui16SensorAcqConf[1] 	= sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].ui8Type;
			sensor_tui16SensorAcqConf[2] 	= sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning;
			sQueueItem.pData 				= (uint8_t*)&sensor_tui16SensorAcqConf[0];
			/* sends a message to RF task in order to modify the ACQ mode system file */
			sQueueItem.ui16NbByte 			= 6;
			/* the receiver is the Rf Task // the sender is the SERIAL task */
			sQueueItem.urecvsender 			= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
			sQueueItem.ui16notification 	= KERNEL_MESSAGEID_CHANGESTATESENSOR;
			xQueueSend (sKernel_QueuePointer.pKernelQueue,&sQueueItem, pdFALSE);
		}
		else
		{
			/* no overflow, monitoring in progress */
		}
	}
	else
	{
		/* stop monitoring */
	}
#endif
}

/***************************************************************************//**
 * @brief
 *   This function updates the information from rf task in the external eeprom
 *   memory about alarm status
 *
 * @detail the external memory map is the following:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of words	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		1			|	Bit field of the states alarm 			|
 * 	|	0x0001		|		2			|	Date of the "below temp" One shot alarm	|
 * 	|	0x0003		|		2			|	Time of the "below temp" One shot alarm	|
 * 	|	0x0005		|		2			|	Date of the "upper temp" One shot alarm	|
 * 	|	0x0007		|		2			|	Time of the "upper temp" One shot alarm	|
 * 	|	0x0009		|		2			|	Date of the "low bat" One shot alarm	|
 * 	|	0x000B		|		2			|	Time of the "low bat" One shot alarm	|
 * 	|	0x000D		|		1			|	CRC16 CCITT 							|
 *  ---------------------------------------------------------------------------------
 * The message sent from RF task:
 *  ---------------------------------------------------------------------------------
 *  |	address		|	number of bytes	|	definition								|
 *  ---------------------------------------------------------------------------------
 * 	|	0x0000		|		2			|	the alarm id	 						|
 * 	|	0x0002		|		4			|	Date of the One shot alarm				|
 * 	|	0x0006		|		4			|	Time of the One shot alarm				|
 *  ---------------------------------------------------------------------------------
 * @param[in] 	pui16Data: Data and other information to write in the external memory
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void sensor_StoreSetAlarmInExtMemory ( const Kernel_QueueItem_struct queueItems )
{
	uint16_t ui16Address;

	/* 1) updates the alarm bit filed according to the alarm id */
	switch (queueItems.pData[0])
	{
		case KERNEL_ALARM_ID_LOW_TEMP:
			/* updates the address to write in the external eeprom */
			ui16Address = SENSOR_ALARMLOWTEMP;
			/* updates the state alarm of the serial task */
			sensor_sAlarm.StateAlarm.sBitField.ui16BelowTemp = 1;
			break;

		case KERNEL_ALARM_ID_HIGH_TEMP:
			/* updates the address to write in the external eeprom */
			ui16Address = SENSOR_ALARMHIGHTEMP;
			/* updates the state alarm of the serial task */
			sensor_sAlarm.StateAlarm.sBitField.ui16UpperTemp = 1;
			break;

		case KERNEL_ALARM_ID_LOW_BATTERY:
			/* updates the address to write in the external eeprom */
			ui16Address = SENSOR_ALARMLOWBAT;
			/* updates the state alarm of the serial task */
			sensor_sAlarm.StateAlarm.sBitField.ui16LowBattery = 1;
			break;

		default:
			break;
	}

	sensor_tui16AlarmDataInEeprom[SENSOR_ALARMBITFIELD] = sensor_sAlarm.StateAlarm.ui16Value;

	/* 2) updates the date/time when the alarm has been triggered */
	sensor_tui16AlarmDataInEeprom[ui16Address] 	= (uint16_t)(queueItems.pData[2] | ((uint16_t)(queueItems.pData[3])<<8));/* ex: pData[2]: LSW, pData[3]: MSW */
	sensor_tui16AlarmDataInEeprom[ui16Address+1]= (uint16_t)(queueItems.pData[4] | ((uint16_t)(queueItems.pData[5])<<8));
	sensor_tui16AlarmDataInEeprom[ui16Address+2]= (uint16_t)(queueItems.pData[6] | ((uint16_t)(queueItems.pData[7])<<8));
	sensor_tui16AlarmDataInEeprom[ui16Address+3]= (uint16_t)(queueItems.pData[8] | ((uint16_t)(queueItems.pData[9])<<8));

	/* 3) computes and inserts the crc */
	srvCRC_InsertCrc16CCITT(SENSOR_SIZE_16BITBUFFER,&sensor_tui16AlarmDataInEeprom[0]);

	/* 4) write the new alarm buffer in the eeprom */
	//srv24lc64_Write16bitData(SENSOR_ALARMBITFIELD,(SENSOR_SIZE_16BITBUFFER*2),(uint8_t*)(&sensor_tui16AlarmDataInEeprom[0]));
	srv24LC_WriteAlarmFields ( sensor_tui16AlarmDataInEeprom);

	/* stay in EM1 during the write cycle time of the memory */
	sleep_DelayTask (PRT24LC64_WRITECYCLEMS, configSLEEP_MODE);
#if 0
	srv24LC_Enable24LC64(false);
#endif

}

/***************************************************************************//**
 * @brief
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		none
 ******************************************************************************/
void sensor_processDataAcquisition ( void )
{
uint8_t status = CROSSRFID_SUCCESSCODE;

	do { /* loop during the measurement*/

		if (xSemaphoreTake(xSemaphore,100)== pdTRUE)
		{
			status = srvLIS2DH12_GetGz (); /* read the measurement */
		}else {/* do nothing*/}

	}while (status != CROSSRFID_SUCCESSCODE);
}

/***************************************************************************//**
 * @brief		this function process the interrupt coming from the
 * 				accelerometer
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE
 ******************************************************************************/
uint8_t sensor_ProcessAcceleromterIRQ ( Kernel_QueueItem_struct *sQueueItem  )
{
uint8_t ui8status =CROSSRFID_SUCCESSCODE;
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)


	/* acc to the ADXL363 data sheet p 41
	 * Clear interrupts in one of several ways, as follows:
	 * - Read the status register (Address 0x0B) clears activity and inactivity interrupts.
	 * - Read from the data registers. Address 0x08 to Address 0x0A or Address 0x0E to Address 0x15 clears the data ready interrupt.
	 * - Read enough data from the FIFO buffer so that interrupt conditions are no longer met clears the FIFO ready, FIFO watermark, and FIFO overrun interrupts.
	*/
	ui8status = srvActRec_ProcessAcceleromterIRQ ( sQueueItem );

#elif (APP_CHOSEN_FLAG == APP_DEMOTAG_BUBBLELEVEL)
	ui8status = srvBubble_ProcessAcceleromterIRQ ( sQueueItem);
#endif
return ui8status;
}


/***************************************************************************//**
 * @brief		this function process the interrupt coming from the
 * 				accelerometer
 * @details
 * @param[in] 	none
 * @param[out] 	none
 * @return 		CROSSRFID_SUCCESSCODE
 ******************************************************************************/
uint8_t sensor_ProcessMagnetoIRQ ( Kernel_QueueItem_struct *sQueueItem  )
{
uint8_t ui8status =CROSSRFID_SUCCESSCODE;
#if (APP_CHOSEN_FLAG == APP_ACTIVITYCOUNTER)

	ui8status= srvActRec_ProcessMagnetoIRQ ( sQueueItem );

#endif
return ui8status;
}


/***************************************************************************//**
 * @brief		this function process the queue message coming from the
 * serial interface about the measurement mode.
 * @param[in/out] 	sQueueItem
 * @param[out] 		none
 * @return 			CROSSRFID_SUCCESSCODE
 ******************************************************************************/
uint8_t sensor_ProcessSerialRequest ( Kernel_QueueItem_struct *sQueueItem  )
{
uint8_t ui8status = CROSSRFID_ERROR;

	ui8status = srvActRec_ProcessSerialRequest(sQueueItem->pData);
	if (CROSSRFID_SUCCESSCODE == ui8status)
	{
		sQueueItem->urecvsender = KERNEL_CREATE_RECANDSEND (KERNEL_SERIALTASKID,KERNEL_SENSORTASKID); /* send the response to the task*/
		sQueueItem->ui16NbByte = 1;
		sQueueItem->pData = CROSSRFID_SUCCESSCODE;
		ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else
	{
		/* do nothing*/
	}
	return ui8status;

}


/***************************************************************************//**
 * @brief		this function process the calendar event. it could be a start or
 * a stop measurement
 * @details
 * @param[in] 	CalendarEventId : Id of the calendar event (see sensor_CalendarEventType_enum )
 * @param[out] 	pQueueItems : the incoming and outgoing queue message
 * @return 		CROSSRFID_SUCCESSCODE : the function is successful
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_ProcessCalendarEvent ( const uint8_t CalendarEventId ,  Kernel_QueueItem_struct *pQueueItems)
{
uint32_t 						ui32timerId;
uint8_t 						ui8status = CROSSRFID_SUCCESSCODE;
sensor_CalendarEvent_struct 	*psCalendarMeasEvent;

	switch (CalendarEventId)
	{
		/* ****************************************************************************************************** */
		case SENSOR_CALENDAREVENT_STOPMEAS:
		/* ****************************************************************************************************** */
			psCalendarMeasEvent = psCalendarEndMeasEvent;
			/* get the timer id */
			ui32timerId = (uint32_t ) sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].ui8TimerId;
			logtimer_DisableLogicalTimer ((uint8_t)ui32timerId);

			sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].IsRunning = false ;
			sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].bEndDateTimeIsPlugged = false ;
			sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].IsConfigured.ui8field = SENSOR_MEASUREMENT_RESET;

			/* if a ACQ END calendar event is pending, plugs it */
			sensor_ConnectaNewCalendarEvent (CalendarEventId);
		break;

		/* ****************************************************************************************************** */
		case SENSOR_CALENDAREVENT_STARTMEAS:
		/* ****************************************************************************************************** */
			psCalendarMeasEvent = psCalendarBeginMeasEvent;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
			/* if it is the temperature sensor id, test if the monitoring has been already started */
			if((false == sensor_bTempIsAlreadyStarted) ||
			   ((true == sensor_bTempIsAlreadyStarted) && (KERNEL_SENSOR_ID_TEMPERATURE != psCalendarMeasEvent->ui8SensorId)))
			{
#endif
				/* get the timer id */
				ui32timerId = (uint32_t ) sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].ui8TimerId;

				/* unplugged the calendar event */
				sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].bStartDateTimeIsPlugged = false ;
				/* enable the RTCC timer when the measurement has been full defined*/
				if ( SENSOR_MEASUREMENT_ISREADY == sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].IsConfigured.ui8field  )
				{
					sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].IsRunning = true ;
					/* init and start the LEtimer logical timer */
					logtimer_inittimer ( 	psCalendarMeasEvent->ui8SensorId ,
											ui32timerId,
											sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].ui8Type,
											sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].ui16PeriodValue,
											sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].ui16PeriodUnit);
					logtimer_EnableLogicalTimer (ui32timerId);

				}

				/* if a ACQ BEGIN calendar event is pending, plugs it */
				sensor_ConnectaNewCalendarEvent (CalendarEventId);

				sensor_sSensorConfig[psCalendarMeasEvent->ui8SensorId].IsConfigured.bitfield.bIsStartDefined = false ;

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
			}
			/* the temperature monitoring has been already launched */
			else
			{
				/* temperature acquisitions still in progress, should not be stopped */
			}
#endif
		break;

		default :
			/* do nothing this event is unknown*/
		break;
	}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
	    /* if the temperature monitoring was never launched */
	if(((false == sensor_bTempIsAlreadyStarted) && (SENSOR_CALENDAREVENT_STARTMEAS == CalendarEventId)) ||
	    /* or if the start event does not concern the temperature */
	   ((KERNEL_SENSOR_ID_TEMPERATURE != psCalendarMeasEvent->ui8SensorId) && (SENSOR_CALENDAREVENT_STARTMEAS == CalendarEventId)) ||
	    /* or if it is not a start operation */
	   (SENSOR_CALENDAREVENT_STOPMEAS == CalendarEventId))
	{
		ui8status =  sensor_SendMeasureStatus (  psCalendarMeasEvent->ui8SensorId , pQueueItems );

		/* if it is the start event of the temperature acquisition */
		if((KERNEL_SENSOR_ID_TEMPERATURE == psCalendarMeasEvent->ui8SensorId) && (SENSOR_CALENDAREVENT_STARTMEAS == CalendarEventId))
		{
			/* The monitoring should no longer be stopped */
			sensor_bTempIsAlreadyStarted = true;
		}
	}
	else
	{
		/* no message to send, no ACQ MODE modification */
	}
#else
	ui8status =  sensor_SendMeasureStatus (  psCalendarMeasEvent->ui8SensorId , pQueueItems );
#endif


	return ui8status ;

}

/***************************************************************************//**
 * @brief 		This function gets and stores a temperature sample.
 *
 * @param[in] 	psQueueItem : the data field of the structure contains the parameters
 * 				of the host request
 * @param[out] 	psQueueItem : the queue object might be updated and should be
 * 				sent back to an another task
 *
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 * @return 		CROSSRFID_READSENSOR_ERROR : the temperature is invalid
 * @return 		CROSSRFID_SUCCESSCODE : the temperature is valid
 ******************************************************************************/
uint8_t sensor_ProcessMeasureTemperature ( Kernel_QueueItem_struct*  psQueueItem )
{
	uint8_t ui8status = CROSSRFID_SUCCESSCODE;

	/* Gets and stores a new temperature sample */
	ui8status = srvtemp_MeasureTemperature();

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
#if 0
/* code used if we want to not save the temperature objects (in ram, nvm_config.c)
 * directly in the service measurement called by the serial task after a logical timer irq
 * (see srvtemp_MeasureTemperature)
 * --- note: sensor_ProcessMeasureTemperature no need argument if this code is not used.*/
	if(CROSSRFID_SUCCESSCODE == ui8status)
	{
		/* SERIAL -> KERNEL
		 * kernel should update the buffer in flash memory */
		psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_KERNELTASKID,KERNEL_SENSORTASKID);
		psQueueItem->ui16notification 	= KERNEL_MESSAGEID_UPDATEFLASH_TEMPERATURE;
		psQueueItem->pData 				= NULL;
		psQueueItem->ui16NbByte 		= 0;

		ui8status = CROSSRFID_MESSAGETOBEPOSTED;
	}
	else
	{
		/* do nothing */
	}
#endif

	if(CROSSRFID_MESSAGETOBEPOSTED == ui8status)
	{
		sensor_ui16DataToSendToRfTask = (uint16_t)srvtemp_GetLastTempAverage();

		/* SERIAL -> RF
		 * RF should write the temperature average in the em4325 user memory */
		psQueueItem->urecvsender 		= KERNEL_CREATE_RECANDSEND (KERNEL_RFFRONTENDTASKID,KERNEL_SENSORTASKID);
		psQueueItem->ui16notification 	= KERNEL_MESSAGEID_WRITE_TEMP_AVERAGE;
		psQueueItem->pData 				= (uint8_t*)&sensor_ui16DataToSendToRfTask;
		psQueueItem->ui16NbByte 		= 2;
	}
	else
	{
		/* do nothing */
	}
#endif

	return ui8status;
}

#if (APP_CHOSEN_FLAG == APP_GLUEPOT)
/***************************************************************************//**
 * @brief 		This function stops the temperature acquisitions, then the associated
 * 				logical timer.
 * @param[out] 	psQueueItems : the queue object to send to rf task
 *
 * @return 		CROSSRFID_MESSAGETOBEPOSTED : a message should be posted
 ******************************************************************************/
uint8_t sensor_StopTempAcquisitions (Kernel_QueueItem_struct  * psQueueItems)
{
	uint8_t ui8status = CROSSRFID_MESSAGETOBEPOSTED;

	sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsRunning = false ;
	sensor_sSensorConfig[KERNEL_SENSOR_ID_TEMPERATURE].IsConfigured.bitfield.bIsStartDefined = false;

	logtimer_DisableLogicalTimer(KERNEL_SENSOR_ID_TEMPERATURE);

	/* fill message to rf task to update the ACQ mode */
	ui8status = sensor_SendMeasureStatus(KERNEL_SENSOR_ID_TEMPERATURE, psQueueItems);

	return ui8status;
}
#endif


/******** THE REST OF THE FILE IS DOCUMENTATION ONLY !**********************//**
 * @{

@page sensor_exteeprom 24LC64 external EEPROM management

	Overview:

	@li @ref Introduction
	@li @ref Mapping
	@li @ref Data integrity
	@li @ref Access to the memory

@n @section sensor_intro Introduction

This external memory allows the firmware to stored information about the triggered alarms.<br>
At the initialization, these data are read by serial and sent to RF task in order to update the system files.
As soon as an one shot alarm is triggered, serial task writes this new alarm in the 24LC64 if it has not already been triggered.<br>

@n @section sensor_mapping Mapping

The 24LC64 serial EEPROM is organized as 8 blocks of 8K bit (64K bit).

<table>
<caption id="multi_row">24LC64 Mapping</caption>
<tr><th>  <th> Block 0
<tr><th> Byte address <th> Word description
<tr><td> 0x0000 <td> Bit field of the alarm status
<tr><td> 0x0001 <td> RTCC Date - Low temperature alarm  (LSW)
<tr><td> 0x0002 <td> RTCC Date - Low temperature alarm  (LSW)
<tr><td> 0x0003 <td> RTCC Time - Low temperature alarm  (MSW)
<tr><td> 0x0004 <td> RTCC Time - Low temperature alarm  (MSW)
<tr><td> 0x0005 <td> RTCC Date - High temperature alarm (LSW)
<tr><td> 0x0006 <td> RTCC Date - High temperature alarm (LSW)
<tr><td> 0x0007 <td> RTCC Time - High temperature alarm (MSW)
<tr><td> 0x0008 <td> RTCC Time - High temperature alarm (MSW)
<tr><td> 0x0009 <td> RTCC Date - Low battery alarm      (LSW)
<tr><td> 0x000A <td> RTCC Date - Low battery alarm      (LSW)
<tr><td> 0x000B <td> RTCC Time - Low battery alarm      (MSW)
<tr><td> 0x000C <td> RTCC Time - Low battery alarm      (MSW)
<tr><td> 0x000D <td> CRC CCITT 16bit
<tr><td> ... <td> ...
<tr><th>  <th> Block 1
<tr><td> ... <td> ...
<tr><th>  <th> Block 2
<tr><td> ... <td> ...
<tr><td> ... <td> ...
<tr><th>  <th> Block 7
<tr><th> Byte address <th> Word description
<tr><td> 0x1C00 <td> Firmware version
<tr><td> 0x1C01 <td> Hardware Version
<tr><td> 0x1C02 <td> Test Version
<tr><td> 0x1C03 <td>
<tr><td> 0x1C04 <td>
<tr><td> 0x1C05 <td>
<tr><td> 0x1C06 <td>
<tr><td> 0x1C07 <td>
<tr><td> 0x1C08 <td>
<tr><td> 0x1C09 <td>
<tr><td> 0x1C0A <td>
<tr><td> 0x1C0B <td>
<tr><td> 0x1C0C <td>
<tr><td> 0x1C0E <td> CRC CCITT 16bit (LSB)
<tr><td> 0x1C0F <td> CRC CCITT 16bit (MSB)
<tr><td> ... <td> ...

</table>

@n @section tempdrv_crc Data integrity

The stored data in the memory are protected by a CRC CCITT16bit.<br>
It is computed by the GPCRC module of the micro controller.<br>
The polynomial is 0x1021.<br>

@n @section tempdrv_access Access to the memory

Serial is the task that interacts with the 24LC64 via the I2C 0.<br>
The firmware always checks the CRC after reading the memory.<br>
Similarly, the crc is computed to replace the old crc when data must be written.<br>
Write cycle time (byte or page): 2ms (typical), 5ms (max)<br>


@page calendar event

	Overview:

	@li @ref Introduction
	@li @ref sensor_CalendarEvent_struct
	@li @ref period

@n @section calendarEvent Introduction
	the user can configure the start and the end date and time for each measurement.
	the RTCC is configured as the calendar and 3 compare channels are available. The channel CC1 is used for the calendar feature.
	the two others are using for the start & end date & time for the measurement.


@n @section structure sensor_CalendarEvent_struct
	the structure sensor_CalendarEvent_struct is used to manage the calendar events (CCxdate/CCxTime).
	2 are available, one for the start date/time, the other for the end date/time

	When a user defines a start and end time the most recent event stored in the sCalendarEventToPlug will
	be enabled

@n @section pediod of the measurement
	When the user chooses a periodic measurement one RTCC timer will be used


 * @}**************************************************************************/
