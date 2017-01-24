/*******************************************************************************
 * @file crc16.c
 * @brief this file contains the function set to compute and check the CRC16
 * according to the epc standard
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 /b>
 *******************************************************************************
 *
 *
 *******************************************************************************/
#include "crc16.h"



static uint16_t ui16crc16;

/**
 * @brief this function initializes the CRC value
 * param[in] none
 * param[out] none
 * return none
 */
void CRC16_init_crc16_computation(void)
{
  ui16crc16 = 0xFFFF; // Equivalent Preset to 0x1D0F
}

/**
 * @brief this function updates the CRC 16 by one bit
 * param[in] bit : new bit to be computed
 * param[out] none
 * return none
 */
void CRC16_update_crc16_computation(unsigned char bit)
{
  ui16crc16 ^= (bit << 15);
  
  if (ui16crc16&0x8000)
  {
	ui16crc16 <<= 1;
	ui16crc16 ^= CRC_POLY16; // (CCITT) x16 + x12 + x5 + 1
  }
  else
  {
    ui16crc16 <<= 1;
  }
}

/**
 * @brief this function returns the final CRC16
 * CRC16 = 2 complement CRC16
 * param[in] none
 * param[out] none
 * return true if the residue is the good one
 * return false when the residue is not the good one
 */
unsigned short int CRC16_get_crc16_computation()
{
  // final xor
  return(ui16crc16^0xffff);
}


/**
 * @brief this function returns the current CRC16
 * param[in] none
 * param[out] none
 * return true if the residue is the good one
 * return false when the residue is not the good one
 */
uint16_t CRC16_IsRightResidue (void)
{
  if (ui16crc16 == CRC16_RESIDUE)
  {
	  return true;
  }
  else
  {
	  return false;
  }
}


/**
 * @brief this function returns the current CRC16
 * param[in] none
 * param[out] none
 * return CRC16
 */
uint16_t get_raw_crc16_without_xor(void)
{
  return(ui16crc16);
}
#if 0
/**
 * @brief Performs CRC computation on "data" array of a given "n" length.
 */
uint16_t crc16_ccitt_byte(const uint8_t* const data, uint16_t n)
{
uint8_t bit_data[8];
uint16_t i,j;

  CRC16_init_crc16_computation();
  
  for (i=0; i < n; i++)
  {
    scatter_8bits(data[i], bit_data);
    for (j=0; j<8; j++)
    {
      CRC16_update_crc16_computation(bit_data[j]);
    }
  }
  
  return CRC16_get_crc16_computation();
}
#endif
/**
 * @brief Performs CRC computation on "bit_data" fake bits array of a given "n" length.
 */
uint16_t crc16_ccitt_bit(const uint8_t* const bit_data, uint16_t n)
{
uint16_t i;

	CRC16_init_crc16_computation();
  
  for (i=0; i < n; i++)
  {
    CRC16_update_crc16_computation(bit_data[i]);
  }
  
  return CRC16_get_crc16_computation();
}






/**
 * @brief Performs CRC-CCITT computation on "data" array of a given "bit_length".
 * @param[in] buffer the bit array of data
 * @param[in] bit_length the number of bit of  buffer
 */
uint16_t __attribute__((optimize("O2"))) crc16_ccitt_bit2(const uint8_t buffer[], uint16_t bit_length)
{
uint16_t ui16val;
uint16_t ui16data = 0;
uint16_t ui16shift = CRC_INIT_VALUE;
size_t i;

	for (i = 0; i < bit_length; i++)
	{
		if ((i % 8) == 0)
		{
			ui16data = (*buffer++) << 8;
		}
		ui16val = ui16shift ^ ui16data;
		ui16shift = ui16shift << 1;
		ui16data = ui16data << 1;
		if (ui16val & 0x8000)
		{
			ui16shift = ui16shift ^ CRC_POLY16;
		}else {/*do nothing*/}
	}
	return ui16shift;
}


void crc16_ccitt_bit2_part(uint8_t *buf,unsigned short length,unsigned short *crcReg)
{
    uint16_t data=0,val;
    uint32_t i=0;


    for (i=0;i<length;i++)
    {
        if ((i % 8) == 0)
        {
            data = (*buf++)<<8;
        }
        val = *crcReg ^ data;
        *crcReg = *crcReg<<1;
        data = data <<1;
        if (val&0x8000)
        {
            *crcReg = *crcReg ^ CRC_POLY16;
        }
    }
}



/**
 * @brief Adds the final not "~" to the CRC-CCITT.
 */
uint16_t crc16_ccitt_bit_epc(const uint8_t buffer[], uint16_t bit_length)
{
	return ~crc16_ccitt_bit2(buffer, bit_length);
}

uint16_t crc_16(uint8_t* bytes, uint32_t size)
{
	uint16_t crc = CRC_INIT_VALUE;
	uint16_t done;
	uint8_t i,todo;
	i = 0;
	done = 0;
	if(size != 0)
	{
		do
		{
			//to match with data alignement MSB first
			if(done & 0x0001)
			{
				todo = bytes[done-1];
			}
			else
			{
				todo = bytes[done+1];
			}
			crc ^= todo;
			for(i=0;i<8;i++)
			{
				if(crc & 0x01)
				{
					crc = (crc >> 1) ^ CRC_POLYNOMIAL;
				}
				else
				{
					crc = crc >> 1;
				}
			}
			done++;
		}while (done < size);
	}
	return crc;
}

