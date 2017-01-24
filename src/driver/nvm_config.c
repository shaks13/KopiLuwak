/***************************************************************************//**
 * @file nvm_config.c
 * @brief NVM driver configuration
 * @version 4.3.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stddef.h>
#include "em_common.h"
#include "nvm.h"
#include "nvm_config.h"

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/*******************************************************************************
 ***********************   DATA SPECIFICATION START   **************************
 ******************************************************************************/

#if 0 /* Old code provided by Silabs for its samples */
/* Object IDs */
typedef enum
{
  COLOR_TABLE_ID,
  COEFFICIENT_TABLE_ID,
  PRIME_NUMBER_TABLE_ID,
  BONUS_TABLE_ID,
  PRIVATE_KEY_TABLE_ID,
  TRANSFORM_TABLE_ID,
  SINGLE_VARIABLE_ID,
  SAFETY_TABLE_ID,
  BIG_EMPTY_TABLE_ID,
  SMALL_NEGATIVE_TABLE_ID,
  SHORT_POSITIVE_TABLE_ID
} NVM_Object_Ids;


/* Page IDs */
typedef enum
{
  MY_PAGE_1,
  MY_PAGE_2,
  MY_PAGE_3,
  MY_PAGE_4,
  MY_PAGE_5,
  MY_PAGE_6
} NVM_Page_Ids;
#endif

#if 1

/* Flash buffers containing the sensor samples already stored in RAM
 * These buffer are used for the backup process. */
uint16_t nvm_ui16TempBuffer[NVM_SIZE_SAMPLE_BUFFER_IN_WORD]; /* #define DATAACQ_MEASBUFFER_NBELEMENT in measurement service  */
uint16_t nvm_ui16AcceleroBuffer[NVM_SIZE_SAMPLE_BUFFER_IN_WORD];
uint16_t nvm_ui16MagnetoBuffer[NVM_SIZE_SAMPLE_BUFFER_IN_WORD];

uint16_t nvm_ui16NbTempSamples;
uint16_t nvm_ui16NbAcceleroSamples;
uint16_t nvm_ui16NbMagnetoSamples;

/* Page definition 0.
 * Combine objects with their ID, and put them in a page. */
NVM_Page_t const myPage0 =
{
/*{ Pointer to object,            			Size of object,           			Object ID}, */
  { (uint8_t *) nvm_ui16TempBuffer, 		sizeof(nvm_ui16TempBuffer),			NVM_TABLE_ID_TEMPERATURE },
  { (uint8_t *) &nvm_ui16NbTempSamples, 	sizeof(nvm_ui16NbTempSamples),		NVM_TABLE_ID_NB_TEMPERATURE },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 1.
 * Combine objects with their ID, and put them in a page. */
NVM_Page_t const myPage1 =
{
/*{ Pointer to object,            			Size of object,           			Object ID}, */
  { (uint8_t *) nvm_ui16AcceleroBuffer, 	sizeof(nvm_ui16AcceleroBuffer),		NVM_TABLE_ID_ACCELERO },
  { (uint8_t *) &nvm_ui16NbAcceleroSamples, sizeof(nvm_ui16NbAcceleroSamples),	NVM_TABLE_ID_NB_ACCELERO },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 2.
 * Combine objects with their ID, and put them in a page. */
NVM_Page_t const myPage2 =
{
/*{ Pointer to object,            			Size of object,           			Object ID}, */
  { (uint8_t *) nvm_ui16MagnetoBuffer, 		sizeof(nvm_ui16MagnetoBuffer),		NVM_TABLE_ID_MAGNETO },
  { (uint8_t *) &nvm_ui16NbMagnetoSamples, 	sizeof(nvm_ui16NbMagnetoSamples),	NVM_TABLE_ID_NB_MAGNETO },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Register all pages into the page table.
 * Assosiate each page to the page ID, and define the type of page. */
NVM_Page_Table_t const nvmPages =
{
/*{ Page ID,   				Page pointer, 	Page type}, */
  { NVM_PAGE_TEMPERATURE, 	&myPage0, 		nvmPageTypeNormal },
  { NVM_PAGE_ACCELERO, 		&myPage1, 		nvmPageTypeNormal },
  { NVM_PAGE_MAGNETO, 		&myPage2, 		nvmPageTypeNormal }
};

#else  /* Old code provided by Silabs for its samples */

/**< Example data objects */
uint32_t colorTable[]           = { 0x000000, 0xff0000, 0x00ff00, 0x0000ff, 0xffff00,
                                    0x00ffff, 0xff00ff, 0xc0c0c0, 0xffffff };

uint8_t coefficientTable[]      = { 11, 12, 13, 14, 15 };

uint8_t primeNumberTable[]      = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41,
                                    43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97,
                                    101, 103, 107, 109, 113, 127, 131, 137, 139, 149,
                                    151, 157, 163, 167, 173, 179, 181, 191, 193, 197,
                                    199, 211, 223, 227, 229, 233, 239, 241, 251 };

uint16_t bonusTable[]           = { 39, 38, 37, 36 };

uint8_t privateKeyTable[]       = { 49, 48, 47, 46 };

uint16_t transformTable[]       = { 59, 58, 57, 56 };

int32_t safetyTable[]           = { -71, 72, -73, 74, -75, -76, -77, -78, 79, -70 };

uint8_t bigEmptyTable[450];

uint32_t singleVariable         = 68;

int8_t smallNegativeTable[]     = { -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11,
                                    -12, -13, -14, -15, -16, -17, -18, -19, -20 };

uint16_t shortPositiveTable[]   = {0xFFF0, 0xFFF1, 0xFFF2};

/* Page definition 1.
 * Combine objects with their ID, and put them in a page. */
NVM_Page_t const myPage1 =
{
/*{ Pointer to object,            Size of object,           Object ID}, */
  { (uint8_t *) colorTable,       sizeof(colorTable),       COLOR_TABLE_ID },
  { (uint8_t *) bonusTable,       sizeof(bonusTable),       BONUS_TABLE_ID },
  { (uint8_t *) &singleVariable,  sizeof(singleVariable),   SINGLE_VARIABLE_ID },
  { (uint8_t *) bigEmptyTable,    sizeof(bigEmptyTable),    BIG_EMPTY_TABLE_ID },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 2.
 * Combine objects with their ID, and put them in a page.
 * This page contains only one object, since it is going to be
 * used as a wear page. */
NVM_Page_t const myPage2 =
{
/*{ Pointer to object,            Size of object,           Object ID}, */
  { (uint8_t *) safetyTable,      sizeof(safetyTable),      SAFETY_TABLE_ID },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 3.
 * Combine objects with their ID, and put them in a page. */
NVM_Page_t const myPage3 =
{
/*{ Pointer to object,            Size of object,           Object ID}, */
  { (uint8_t *) coefficientTable, sizeof(coefficientTable), COEFFICIENT_TABLE_ID },
  { (uint8_t *) privateKeyTable,  sizeof(privateKeyTable),  PRIVATE_KEY_TABLE_ID },
  { (uint8_t *) &singleVariable,  sizeof(singleVariable),   SINGLE_VARIABLE_ID },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 4.
 * Combine objects with their ID, and put them in a page. */
NVM_Page_t const myPage4 =
{
/*{ Pointer to object,            Size of object,           Object ID}, */
  { (uint8_t *) primeNumberTable, sizeof(primeNumberTable), PRIME_NUMBER_TABLE_ID },
  { (uint8_t *) transformTable,   sizeof(transformTable),   TRANSFORM_TABLE_ID },
  { (uint8_t *) &singleVariable,  sizeof(singleVariable),   SINGLE_VARIABLE_ID },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 5.
 * Combine objects with their ID, and put them in a page.
 * This page contains only one object, since it is going to be
 * used as a wear page. */
NVM_Page_t const myPage5 =
{
/*{ Pointer to object,            Size of object,           Object ID}, */
  { (uint8_t *) smallNegativeTable,      sizeof(smallNegativeTable),      SMALL_NEGATIVE_TABLE_ID },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Page definition 6.
 * Combine objects with their ID, and put them in a page.
 * This page contains only one object, since it is going to be
 * used as a wear page. */
NVM_Page_t const myPage6 =
{
/*{ Pointer to object,            Size of object,           Object ID}, */
  { (uint8_t *) shortPositiveTable,      sizeof(shortPositiveTable),      SHORT_POSITIVE_TABLE_ID },
  NVM_PAGE_TERMINATION /* Null termination of table. This is required. */
};

/* Register all pages into the page table.
 * Assosiate each page to the page ID, and define the type of page. */
NVM_Page_Table_t const nvmPages =
{
/*{ Page ID,   Page pointer, Page type}, */
  { MY_PAGE_1, &myPage1, nvmPageTypeNormal },
  { MY_PAGE_2, &myPage2, nvmPageTypeWear },
  { MY_PAGE_3, &myPage3, nvmPageTypeNormal },
  { MY_PAGE_4, &myPage4, nvmPageTypeNormal },
  { MY_PAGE_5, &myPage5, nvmPageTypeWear },
  { MY_PAGE_6, &myPage6, nvmPageTypeWear }
};
#endif
/*******************************************************************************
 ************************   DATA SPECIFICATION END   ***************************
 ******************************************************************************/

/** The code below should not be changed.
 *
 *  Set the storage location in memory. This area should be reserved
 *  using the linker and needs to be aligned with the physical page
 *  grouping of the device.
 *
 *  For the internal flash in the Gecko and Tiny Gecko MCUs, the flash pages are
 *  512 bytes long. This means that the start location must be a multiple of
 *  512 bytes, and that an area equal to 512 bytes * the number of pages and
 *  scratch page must be reserved here.
 *
 *  This can be allocated in the IAR linker file using:
 *
 *    place at address mem:0x1f000 { readonly section NVM_PAGES };
 *
 *  and specifying the pages in the C code using:
 *
 *    const uint8_t nvmData[NVM_PAGE_SIZE * NUMBER_OF_PAGES] @ "NVM_PAGES";
 *
 */
#define NUMBER_OF_USER_PAGES  (sizeof(nvmPages) / sizeof(NVM_Page_Descriptor_t))
#define NUMBER_OF_PAGES       (NVM_PAGES_SCRATCH + NUMBER_OF_USER_PAGES)

/// @endcond

SL_ALIGN(NVM_PAGE_SIZE)
#ifdef __ICCARM__
/**< Set storage size and location */
static const uint8_t nvmData[NVM_PAGE_SIZE * NUMBER_OF_PAGES] @ ".text";
#else
/**< Set storage size and location */
static const uint8_t nvmData[NVM_PAGE_SIZE * NUMBER_OF_PAGES] SL_ATTRIBUTE_ALIGN(NVM_PAGE_SIZE) = { 0xFF };
#endif


static NVM_Config_t const nvmConfig =
{
  &nvmPages,            /**< Page table */
  NUMBER_OF_PAGES,      /**< Total number of pages */
  NUMBER_OF_USER_PAGES, /**< Wear leveling pages */
  nvmData               /**< NVM data */
};                      /**< Top-level configuration data */

/***************************************************************************//**
 * @brief
 *   Return a pointer to the config data.
 *
 * @return
 *   A pointer to the configuration
 ******************************************************************************/
NVM_Config_t const *NVM_ConfigGet(void)
{
  return( &nvmConfig );
}

#if 0
/***************************************************************************//**
 * @brief
 *   Return a pointer to the sample buffer according to the Object ID.
 *
 * @param[in] ObjectID: the object ID
 * @param[out] pui16BufferAddress: the pointer to the sample buffer
 *
 * @return
 *   Returns the result of the operation.
 ******************************************************************************/
Ecode_t NVM_GetBufferAddress (const NVM_Object_Ids ObjectID, uint16_t** ppui16BufferAddress)
{
	Ecode_t ui8Status = ECODE_EMDRV_NVM_OK;
	*ppui16BufferAddress = NULL;

	switch(ObjectID)
	{
		case NVM_TABLE_ID_TEMPERATURE:
			*ppui16BufferAddress = &nvm_ui16TempBuffer[0];
			break;

		case NVM_TABLE_ID_ACCELERO:
			*ppui16BufferAddress = &nvm_ui16AcceleroBuffer[0];
			break;

		case NVM_TABLE_ID_MAGNETO:
			*ppui16BufferAddress = &nvm_ui16MagnetoBuffer[0];
			break;

		default:
			ui8Status = ECODE_EMDRV_NVM_ADDR_INVALID;
			break;
	}

	return ui8Status;
}
#endif
