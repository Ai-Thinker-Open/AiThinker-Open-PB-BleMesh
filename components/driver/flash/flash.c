/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/*******************************************************************************
* @file		flash.c
* @brief	Contains all functions support for flash driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "flash.h"
#include "log.h"
#include "common.h"
#include "error.h"
#include "hal_mcu.h"

//#include ""


// spif config register


//typedef unsigned char  uint8_t;
//typedef unsigned long   uint32_t;



uint8_t flash_write_ucds(uint32_t addr,uint32_t value)
{
	if(addr>0x3ffc || (addr&0x03)>0)
	{
        return PPlus_ERR_INVALID_DATA;
	}
	else
	{
        uint32_t temp = value;
        uint32_t offset = (addr + FLASH_UCDS_ADDR_BASE) & 0xffffff;
        if(ProgramWord (offset, (uint8_t *) &temp, 4)==0)  return PPlus_ERR_FATAL;
	}
    
	return PPlus_SUCCESS;
    
}

uint8_t flash_write_ucds_block(uint32_t addr,uint32_t length,uint32_t *buf)
{
  
	if(addr>0x3ffc||((addr+(length<<2))>0x3ffc || (addr&0x03)>0))
	{
		return PPlus_ERR_INVALID_DATA;
	}
	else
	{  
		for(uint32_t i=0;i<length;i++)
		{
			flash_write_ucds(addr+(i<<2),buf[i]);
		}
	}
	  
	  return PPlus_SUCCESS;
  
}

uint8_t flash_read_ucds_block(uint32_t addr,uint32_t length,uint32_t *tobuf)
{
	if(addr>0x3ffc||((addr+(length<<2))>0x3ffc) || (addr&0x03)>0 )
	{ 
		return PPlus_ERR_INVALID_DATA;
	}
	else
	{  
		for(uint32_t i=0;i<length;i++)
		{    
			tobuf[i]=flash_read_ucds(addr+(i<<2));       
		}
	}
  
	return PPlus_SUCCESS;
}

uint8_t flash_write_ucds_block_byte(uint32_t addr,uint32_t length,uint8_t *buf)
{
  
    if(addr>0x3ffc||((addr+(length))>0x3ffc)|| (addr&0x03)>0)
    {
        return PPlus_ERR_INVALID_DATA;
    }
    else
  	{  
	    uint32_t i;
        for(i=0;i<(length>>2);i++)
		{

            flash_write_ucds(addr+(i<<2), (buf[4*i+3]<<24 ) | (buf[4*i+2]<<16)| (buf[4*i+1]<<8) | buf[4*i] );
        }

        if(length-(i<<2)==3)
        {
            flash_write_ucds(addr+(i<<2), (buf[4*i+2]<<16)| (buf[4*i+1]<<8) | buf[4*i] );
        }
        else if(length-(i<<2)==2)
        {
            flash_write_ucds(addr+(i<<2), (buf[4*i+1]<<8) | buf[4*i] );
        }
        else if(length-(i<<2)==1)
        {
            flash_write_ucds(addr+(i<<2), buf[4*i] );
        }
    }
  
    return PPlus_SUCCESS;
  
}

uint8_t flash_read_ucds_block_byte(uint32_t addr,uint32_t length,uint8_t *tobuf)
{
	if(addr>0x3ffc||((addr+(length))>0x3ffc)|| (addr&0x03)>0)
	{
		return PPlus_ERR_INVALID_DATA;
	}
	else
	{  
		uint32_t i,wbuf;
		for(i=0;i<(length>>2);i++)
		{      
			wbuf = flash_read_ucds(addr+(i<<2)); 
			tobuf[(i<<2)  ] =  0x000000ff & wbuf;
			tobuf[(i<<2)+1] = (0x0000ff00 & wbuf)>>8;
			tobuf[(i<<2)+2] = (0x00ff0000 & wbuf)>>16;
			tobuf[(i<<2)+3] = (0xff000000 & wbuf)>>24;      
		}

		if(length-(i<<2)==3)
		{
			wbuf = flash_read_ucds(addr+(i<<2)); 
			tobuf[(i<<2)  ] =  0x000000ff & wbuf;
			tobuf[(i<<2)+1] = (0x0000ff00 & wbuf)>>8;
			tobuf[(i<<2)+2] = (0x00ff0000 & wbuf)>>16;
		}
		else if(length-(i<<2)==2)
		{
			wbuf = flash_read_ucds(addr+(i<<2)); 
			tobuf[(i<<2)  ] =  0x000000ff & wbuf;
			tobuf[(i<<2)+1] = (0x0000ff00 & wbuf)>>8;
		}
		else if(length-(i<<2)==1)
		{      
			wbuf = flash_read_ucds(addr+(i<<2)); 
			tobuf[(i<<2)  ] =  0x000000ff & wbuf;
		}
	}
  
	return PPlus_SUCCESS;
}


uint32_t flash_read_ucds(uint32_t addr)
{
    if(addr>0x3ffc)
	{
        return 0xffffffff;
	}
	else
	{
        return read_reg(addr+FLASH_UCDS_ADDR_BASE);
	}    
}

void flash_erase_ucds_all(void)
{
	flash_sector_erase(0x11005000);
    flash_sector_erase(0x11006000);
    flash_sector_erase(0x11007000);
    flash_sector_erase(0x11008000);       
}

uint8_t flash_erase_ucds(uint32_t addr)
{
	if(addr>0x3ffc)
	{
        return PPlus_ERR_INVALID_DATA;
	}
    
    if(addr<0x0fff)
	{
        flash_sector_erase(0x11005000);
	}
	else if(addr<0x1fff)
	{
        flash_sector_erase(0x11006000);
	}
	else if(addr<0x2fff)
	{
        flash_sector_erase(0x11007000);
	}
	else if(addr<0x3fff)
	{
        flash_sector_erase(0x11008000);
	} 

	return PPlus_SUCCESS;
}



