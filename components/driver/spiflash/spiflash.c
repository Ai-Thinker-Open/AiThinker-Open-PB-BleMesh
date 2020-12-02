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

#include "spi.h"
#include "error.h"
#include "spiflash.h"
#include "hal_mcu.h"
#include "log.h"
/*
hal_spi_t spi ={
.spi_index = SPI0,
};
*/
extern hal_spi_t spi;

#define spiflash_cmd_tx_and_rx(tx_buf,rx_buf,len)   hal_spi_transmit(&spi,tx_buf,rx_buf,len)

//gd25q16 driver
uint32_t spiflash_read_identification(void)//check
{
	uint8_t buf_send[4] = {FLASH_RDID,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,4))
	{
		return (buf_rece[1] << 16)|(buf_rece[2] << 8) | (buf_rece[3]);
	}
	else
	{
		return 0xFFFFFF;
	}
}

uint16_t spiflash_read_status_register(uint8_t bitsSel)//0~low other~high
{
	uint8_t buf_send[4] = {0x00,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	if(bitsSel == 0)
	{
		buf_send[0] = FLASH_RDSR_LOW;
	}
	else
	{
		buf_send[0] = FLASH_RDSR_HIGH;
	}
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,3))
	{
		return (buf_rece[1] << 8) | (buf_rece[2]);
	}
	else
	{
		return 0xFFFF;
	}
}

bool spiflash_bus_busy(void)
{
	return (spiflash_read_status_register(0) & 0x01);
}


void spiflash_program_erase_suspend(void)
{
	uint8_t buf_send[1] = {FLASH_PES};
	uint8_t buf_rece[1] = {0x00};
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_program_erase_resume(void)
{
	uint8_t buf_send[1] = {FLASH_PER};
	uint8_t buf_rece[1] = {0x00};
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_deep_powerdown(void)
{
	uint8_t buf_send[1] = {FLASH_DP};
	uint8_t buf_rece[1] = {0x00};
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_release_from_powerdown(void)
{
	uint8_t buf_send[1] = {FLASH_RDI};
	uint8_t buf_rece[1] = {0x00};
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_write_enable(void)
{
	uint8_t buf_send[1] = {FLASH_WREN};
	uint8_t buf_rece[1] = {0x00};
	
	while(spiflash_bus_busy());
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_write_disable(void)
{
	uint8_t buf_send[1] = {FLASH_WRDIS};
	uint8_t buf_rece[1] = {0x00};
	
	//while(spiflash_bus_busy());
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_chip_erase(void)
{	
	uint8_t buf_send[1] = {FLASH_CE};
	uint8_t buf_rece[1] = {0x00};
	
	buf_send[0] = FLASH_CE;
	spiflash_write_enable();
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,1))
	{
		;
	}
}

void spiflash_sector_erase(uint32_t addr)
{	
	uint8_t buf_send[4] = {FLASH_SE,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	buf_send[1] = (addr>>16) & 0xff;
	buf_send[2] = (addr>>8) & 0xff;
	buf_send[3] = addr & 0xff;

	spiflash_write_enable();
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,4))
	{
		;
	}
}

void spiflash_block_erase_32KB(uint32_t addr)
{
	uint8_t buf_send[4] = {FLASH_BE_32KB,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	buf_send[1] = (addr>>16) & 0xff;
	buf_send[2] = (addr>>8) & 0xff;
	buf_send[3] = addr & 0xff;

	spiflash_write_enable();
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,4))
	{
		;
	}
}

void spiflash_block_erase_64KB(uint32_t addr)
{
	uint8_t buf_send[4] = {FLASH_BE_64KB,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	buf_send[1] = (addr>>16) & 0xff;
	buf_send[2] = (addr>>8) & 0xff;
	buf_send[3] = addr & 0xff;

	spiflash_write_enable();
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,4))
	{
		;
	}
}


void spiflash_write_status_register(uint8_t data)
{	
	uint8_t buf_send[2] = {FLASH_WRSR,0x00};
	uint8_t buf_rece[2] = {0x00,0x00};
	
	buf_send[1] = data;
	while(spiflash_bus_busy());
	spiflash_write_enable();	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,2))
	{
		;
	}
}

static void spiflash_write_unit(uint32_t addr,uint8_t* tx_buf,uint8_t tx_len)//tx_len in [1,4]
{
	uint8_t buf_send[8] = {FLASH_PP,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	uint8_t buf_rece[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	buf_send[1] = (addr>>16)&0xff;
	buf_send[2] = (addr>>8)&0xff;
	buf_send[3] = addr & 0xff;
	switch(tx_len)
	{
			case 1:
			buf_send[4] = *tx_buf;
			break;
			case 2:
			buf_send[4] = *tx_buf;
			buf_send[5] = *(tx_buf+1);
			break;
			case 3:
			buf_send[4] = *tx_buf;
			buf_send[5] = *(tx_buf+1);
			buf_send[6] = *(tx_buf+2);
			break;
			case 4:
			buf_send[4] = *tx_buf;
			buf_send[5] = *(tx_buf+1);
			buf_send[6] = *(tx_buf+2);
			buf_send[7] = *(tx_buf+3);
			break;
			default:
			break;
	}
	spiflash_write_enable();
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,(tx_len + 4)))
	{
		;
	}
}

void spiflash_write(uint32_t addr,uint8_t* tx_buf,uint16_t tx_len)
{
//polling mode=polling
//force_cs=0
	uint16_t offset = 0,ret16;
	
	ret16 = spiflash_read_status_register(0);
	if(ret16 != 0)
	{
		spiflash_write_status_register(0x00);
		while(spiflash_bus_busy() == TRUE);
	}
					
	while(tx_len > 0)
	{
		if(tx_len >= 4)
		{
			spiflash_write_unit((addr + offset),(tx_buf + offset),4);
			offset += 4;
			tx_len -= 4;
		}
		else
		{
			spiflash_write_unit((addr + offset),(tx_buf + offset),tx_len);
			tx_len = 0;
		}
	}

//you can process the protect with your requirenment	
//	if(ret16 != 0)
//	{
//		spiflash_write_status_register(ret16);
//	}
}


static void spiflash_read_unit(uint32_t addr,uint8_t* rx_buf,uint8_t rx_len)//rx_len in [1,4]
{
	uint8_t buf_send[8] = {FLASH_READ,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	uint8_t buf_rece[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	buf_send[1] = (addr>>16)&0xff;
	buf_send[2] = (addr>>8)&0xff;
	buf_send[3] = addr & 0xff;
	
	if(PPlus_SUCCESS == spiflash_cmd_tx_and_rx(buf_send,buf_rece,(rx_len + 4)))
	{
		switch(rx_len)
		{
			case 1:
			*rx_buf = buf_rece[4];
			break;
			case 2:
			*rx_buf = buf_rece[4];
			*(rx_buf+1) = buf_rece[5];
			break;
			case 3:
			*rx_buf = buf_rece[4];
			*(rx_buf+1) = buf_rece[5];
			*(rx_buf+2) = buf_rece[6];
			break;
			case 4:
			*rx_buf = buf_rece[4];
			*(rx_buf+1) = buf_rece[5];
			*(rx_buf+2) = buf_rece[6];
			*(rx_buf+3) = buf_rece[7];
			break;
			default:
			break;
		}
	}
}

void spiflash_read(uint32_t addr,uint8_t* rx_buf,uint16_t rx_len)
{
//polling mode=polling
//force_cs=0
	uint16_t offset = 0;
	
	while(rx_len > 0)
	{
		if(rx_len >= 4)
		{
			spiflash_read_unit((addr + offset),(rx_buf + offset),4);
			offset += 4;
			rx_len -= 4;
		}
		else
		{
			spiflash_read_unit((addr + offset),(rx_buf + offset),rx_len);
			rx_len = 0;
		}
	}
}

//gd25q16
int GD25_init(void)
{
	//if(hal_spi_bus_init(&spi,cfg) == PPlus_SUCCESS)//config and init spi first
	//	LOG("spi init success!\n");
	
	if(0xC84015 != spiflash_read_identification()){
		//LOG("read flash id error\n");
		return false;
	}
	return true;
}

int GD25_read(uint32_t addr,uint8_t *data,uint8_t len)
{
	if((addr < 0x200000) && (data != NULL) && (len > 0)){
		spiflash_read(addr,data,len);
		return true;
	}
	return false;
}

int GD25_erase(uint32_t addr,uint32_t len)
{
	uint8_t lockinfo = 0;
	uint32_t remainder = 0;

	if((addr >= 0x200000) || (len == 0))
		return false;
	
	lockinfo = spiflash_read_status_register(0);
	spiflash_write_status_register(0x00);
	
	if((addr == 0) && (len == 0x200000))
	{
		spiflash_chip_erase();
	}
	else
	{
		remainder = addr%0x1000;//4KB
		if(remainder){
			addr -= remainder;
			len += remainder;
		}
		remainder = len%0x1000;//4KB
		if(remainder){
			len = len + 0x1000 - remainder;
		}
		
		addr = addr/0x1000;
		len = len/0x1000;
		
		while(len > 0){
			//LOG("addr:%d len:%d\n",addr,len);
			if(((addr %16) == 0) && (len >= 16)){
					while(spiflash_bus_busy());
					spiflash_block_erase_64KB(addr*0x1000);
					addr += 16;
					len -= 16;
					continue;
				}
			
			if(((addr %8) == 0) && (len >= 8)){
					while(spiflash_bus_busy());
					spiflash_block_erase_32KB(addr*0x1000);
					addr += 8;
					len -= 8;
					continue;
			}
				
			if(len >= 1){
					while(spiflash_bus_busy());
					spiflash_sector_erase(addr*0x1000);
					addr += 1;
					len -= 1;
					continue;
			}
		}
	}
	
	spiflash_write_status_register(lockinfo);
	while(spiflash_bus_busy());

	return true;
}

int GD25_write(uint32_t addr,const uint8_t *data,uint8_t len)
{
	if((addr < 0x200000) && (data != NULL) && (len > 0)){
		spiflash_write(addr,(uint8_t *)data,len);
		return true;
	}
	return false;
}
