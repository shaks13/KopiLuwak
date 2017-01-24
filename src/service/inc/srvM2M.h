/*******************************************************************************
 * @file srvM2M.h
 * @brief this file is the service layer for the M2M communication
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#ifndef SRVM2M_H_
#define SRVM2M_H_

#include "common_library.h"
#include "common_statuscode.h"
#include "protocol_M2M.h"




/*===========================================================================================================
						constant definition
===========================================================================================================*/
#define SRVM2M_COMMANDLENGTH 					(64)
/*===========================================================================================================
						Enum definition
===========================================================================================================*/
#if 0
/**
 *  @enum srvm2m_commandcode_enum
 *  @brief this enum defines the command code used by the queue object to exchange with the other tasks.
 *  The equivalent in the Sensor task is srvActivityRecorder_ReadWriteAccess_enum
 */
typedef enum {
	SRVM2M_COMMANDCODE_GET = 0x00,			/*!< the get command code allow to retrieve some information about one of the feature */
	SRVM2M_COMMANDCODE_SET ,				/*!< the set command code allow to configure some information about one of the feature */
}srvm2m_commandcode_enum;
#endif

/**
 *  @enum srvm2m_featureId_enum
 *  @brief this enum defines the command code used by the queue object to exchange with the other tasks.
 *  The equivalent in the Sensor task is srvActivityRecorder_ReadWriteAccess_enum
 */
typedef enum {
	SRVM2M_FEATURE_CONTINUOUS = 0x00,			/*!< the continuous mode configure a continuous measurement and send regularly the results to the host */
	SRVM2M_FEATURE_ACTIVITY ,					/*!< the activity counts the number of time that a movement is detected and the number of hours of an activity  */
	SRVM2M_FEATURE_LOG ,						/*!< the log configure a measure during a given period   */
}srvm2m_featureId_enum;


/**
 *  @enum srvm2m_featureId_enum
 *  @brief this enum defines the command code used by the queue object to exchange with the other tasks.
 *  The equivalent in the Sensor task is srvActivityRecorder_ReadWriteAccess_enum
 */
typedef enum {
	SRVM2M_ACTIONID_ON = 0x00,					/*!< the continuous mode configure a continuous measurement and send regularly the results to the host */
	SRVM2M_ACTIONID_OFF ,						/*!< the activity counts the number of time that a movement is detected and the number of hours of an activity  */
	SRVM2M_ACTIONID_STATE							/*!< the activity counts the number of time that a movement is detected and the number of hours of an activity  */
}srvm2m_actionId_enum;


/*===========================================================================================================
						structure definition
===========================================================================================================*/


/*===========================================================================================================
						prototype
===========================================================================================================*/
void 	srvm2m_init 					( void );
uint8_t srvm2m_ProcessRxMessage 		( uint8_t * const pui8Rxdata, uint8_t ui8nbCarac );
void 	srvm2m_SendString				( const char* const str);
//uint8_t srvm2m_ProcessAccelerometerOp 	( kernel_DataExchange_Type *prtM2M_SerialCommand , Kernel_QueueItem_struct *pQueueItems  );

#endif /* SRVM2M_H_ */
