/*******************************************************************************
 * @file srvFlashMemory.c
 * @brief Non-Volatile Memory Wear-Leveling service API implementation
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015  , </b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "srvFlashMemory.h"

/*===========================================================================================================
						Private variables declaration
===========================================================================================================*/

static bool srvFlashMemory_bIsErase = false;
/*===========================================================================================================
						Public variables declaration
===========================================================================================================*/
/*===========================================================================================================
						Private function declaration
===========================================================================================================*/

/*===========================================================================================================
						Private functions definition
===========================================================================================================*/


/*===========================================================================================================
						Public functions definition
===========================================================================================================*/
/***************************************************************************//**
 * @brief
 *   Initializes the NVM service.
 *
 * @return CROSSRFID_SUCCESSCODE: successful initialization
 * @return CROSSRFID_ERROR_NVM_INIT: error during the initialization of the NVM module
 *
 ******************************************************************************/
uint8_t srvFlashMemory_Init (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	/* initialize NVM module */
	if (ECODE_EMDRV_NVM_NO_PAGES_AVAILABLE == NVM_Init(NVM_ConfigGet()))
	{
		/* no valid data in flash! we have to erase NVM */
		ui32NvmStatus = NVM_Erase(0);

		/* is empty */
		srvFlashMemory_bIsErase = true;

		if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
		{
			ui8Status = CROSSRFID_ERROR_NVM_INIT;
		}
	}

	return ui8Status;
}

#if 0 /* function not used */
/***************************************************************************//**
 * @brief
 *   Reads the temperature buffer in flash.
 *
 * @details
 *   The user should access to the samples via the buffer in nvm_config.c.
 *
 * @return CROSSRFID_SUCCESSCODE: successful read operation
 * @return CROSSRFID_ERROR_NVM_READ: read operation failed
 *
 ******************************************************************************/
uint8_t srvFlashMemory_ReadTempBuffer (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	/* Read the temperature objects */
	ui32NvmStatus = NVM_Read(NVM_PAGE_TEMPERATURE, NVM_TABLE_ID_TEMPERATURE);

	if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
	{
		/* Read error */
		ui8Status = CROSSRFID_ERROR_NVM_READ;
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   Reads the number of temperature samples in flash.
 *
 * @details
 *   The user should access to this value via the variable in ram in nvm_config.c.
 *
 * @return CROSSRFID_SUCCESSCODE: successful read operation
 * @return CROSSRFID_ERROR_NVM_READ: read operation failed
 *
 ******************************************************************************/
uint8_t srvFlashMemory_ReadNbTempSample (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	/* Read the temperature objects */
	ui32NvmStatus = NVM_Read(NVM_PAGE_TEMPERATURE, NVM_TABLE_ID_NB_TEMPERATURE);

	if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
	{
		/* Read error */
		ui8Status = CROSSRFID_ERROR_NVM_READ;
	}

	return ui8Status;
}
#endif

/***************************************************************************//**
 * @brief
 *   Read the temperature objects (values in buffer + number of samples values)
 *   in flash.
 *
 * @details
 *   The user should access to the objects via the variables (ram) in nvm_config.c.
 *
 * @return CROSSRFID_SUCCESSCODE: successful read operation
 * @return CROSSRFID_ERROR_NVM_READ: read operation failed
 * @return CROSSRFID_ERROR_NVM_ISERASE: no read operation, the flash is empty
 ******************************************************************************/
uint8_t srvFlashMemory_ReadAllTempObjects (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	if (false == srvFlashMemory_bIsErase)
	{
		/* Read the temperature objects */
		ui32NvmStatus = NVM_Read(NVM_PAGE_TEMPERATURE, NVM_READ_ALL_CMD);

		if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
		{
			/* Read error */
			ui8Status = CROSSRFID_ERROR_NVM_READ;
		}
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_NVM_ISERASE;
	}

	return ui8Status;
}

#if 0 /* function not used */
/***************************************************************************//**
 * @brief
 *   Writes the temperature buffer in flash.
 *
 * @details
 *   The user should fill the buffer in nvm_config.c before calling this function.
 *
 * @return CROSSRFID_SUCCESSCODE: successful write operation
 * @return CROSSRFID_ERROR_NVM_WRITE: write operation failed
 *
 ******************************************************************************/
uint8_t srvFlashMemory_WriteTempBuffer (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	/* Write the temperature objects */
	ui32NvmStatus = NVM_Write(NVM_PAGE_TEMPERATURE, NVM_TABLE_ID_TEMPERATURE);

	if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
	{
		/* Write error */
		ui8Status = CROSSRFID_ERROR_NVM_WRITE;
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   Writes the number of temperature sample.
 *
 * @details
 *   The user should fill the number of samples in nvm_config.c before calling
 *   this function.
 *
 * @return CROSSRFID_SUCCESSCODE: successful write operation
 * @return CROSSRFID_ERROR_NVM_WRITE: write operation failed
 *
 ******************************************************************************/
uint8_t srvFlashMemory_WriteNbTempInBuffer (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	/* Write the objects */
	ui32NvmStatus = NVM_Write(NVM_PAGE_TEMPERATURE, NVM_TABLE_ID_NB_TEMPERATURE);

	if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
	{
		/* Write error */
		ui8Status = CROSSRFID_ERROR_NVM_WRITE;
	}

	return ui8Status;
}
#endif

/***************************************************************************//**
 * @brief
 *   Write the temperature buffer in flash and the number of temperature samples.
 *
 * @details
 *   The user should fill the buffer in nvm_config.c before calling this function.
 *
 * @return CROSSRFID_SUCCESSCODE: successful write operation
 * @return CROSSRFID_ERROR_NVM_WRITE: write operation failed
 *
 ******************************************************************************/
uint8_t srvFlashMemory_WriteAllTempObjects (void)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;
	uint32_t ui32NvmStatus;

	/* Write the temperature objects */
	ui32NvmStatus = NVM_Write(NVM_PAGE_TEMPERATURE, NVM_WRITE_ALL_CMD);

	if (ECODE_EMDRV_NVM_OK != ui32NvmStatus)
	{
		/* Write error */
		ui8Status = CROSSRFID_ERROR_NVM_WRITE;
	}
	else
	{
		/* no longer empty */
		srvFlashMemory_bIsErase = false;
	}

	return ui8Status;
}

#if 0 /* function not used */
/***************************************************************************//**
 * @brief
 *   Saves one temperature sample in the buffer in ram which is the image of
 *   the data in flash.
 *
 * @details
 *   After calling this function, the flash is not yet updated.
 *
 * @param[in]	ui8Idx: the index of the value to write in the buffer
 * @param[out] 	i16Value: the value to write in the buffer
 *
 * @return CROSSRFID_SUCCESSCODE: successful save operation
 * @return CROSSRFID_ERROR_NVM_OVERFLOW: invalid index
 *
 ******************************************************************************/
uint8_t srvFlashMemory_SaveOneTempSample (const uint8_t ui8Idx, const int16_t i16Value)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;

	if(NVM_SIZE_SAMPLE_BUFFER_IN_WORD >= ui8Idx)
	{
		nvm_ui16TempBuffer[ui8Idx] = (uint16_t)i16Value;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_NVM_OVERFLOW;
	}

	return ui8Status;
}

/***************************************************************************//**
 * @brief
 *   Sets the number of temperature samples in the Flash buffer.
 *
 * @details
 *   After calling this function, the flash is not yet updated.
 *
 * @param[in]
 *   ui16Nb: the number of samples
 *
 * @return CROSSRFID_SUCCESSCODE: successful save operation
 * @return CROSSRFID_ERROR_NVM_OVERFLOW: invalid index
 *
 ******************************************************************************/
uint8_t srvFlashMemory_SetNbTempSamples (const uint16_t ui16Nb)
{
	uint8_t ui8Status = CROSSRFID_SUCCESSCODE;

	if(NVM_SIZE_SAMPLE_BUFFER_IN_WORD > ui16Nb)
	{
		nvm_ui16NbTempSamples = (uint16_t)ui16Nb;
	}
	else
	{
		ui8Status = CROSSRFID_ERROR_NVM_OVERFLOW;
	}

	return ui8Status;
}
#endif
