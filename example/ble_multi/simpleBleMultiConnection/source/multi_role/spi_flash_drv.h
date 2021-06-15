
#ifndef __SPIFLASH_DRV_H__
#define __SPIFLASH_DRV_H__
#include "bcomdef.h"

/*gd25q16 cmd define*/
#define FLASH_WREN						0x06
#define FLASH_WRDIS						0x04	
#define FLASH_CE							0x60
//#define FLASH_CE						0xC7

#define FLASH_DP							0xB9
#define FLASH_RDI							0xAB
#define FLASH_SE							0x20
#define FLASH_BE_32KB					0x52
#define FLASH_BE_64KB					0xD8
#define FLASH_WRSR						0x01
#define FLASH_RDID						0x9F
#define FLASH_RDSR_LOW				0x05
#define FLASH_RDSR_HIGH				0x35
#define FLASH_PP							0x02
#define FLASH_READ						0x03	

int spiflash_write(uint32_t addr, uint8* data, uint32_t len);
int spiflash_read(uint32_t addr, uint8* data, uint32_t len);
int spiflash_erase(uint32_t addr,uint32_t len);
int spiflash_erase_all(void);
int spiflash_init(void);


#endif


