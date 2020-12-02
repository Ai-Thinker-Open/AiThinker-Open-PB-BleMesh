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


#include "flash.h"
#include "ota_flash.h"
#include "ota_app_service.h"
#include "ota_protocol.h"
#include "error.h"
#include "log.h"
#include "hal_mcu.h"

bool is_crypto_app(void);
void flash_load_parition(unsigned char* pflash, int size, unsigned char* run_addr);

static uint16_t __attribute__((section("ota_app_loader_area"))) crc16_byte(uint16_t crc, uint8_t byte)
{
    static const uint16_t crc16_table[16] =
    {
        0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
        0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
    };

    uint16_t temp;

    // Compute checksum of lower four bits of a byte.
    temp         = crc16_table[crc & 0xF];
    crc  = (crc >> 4u) & 0x0FFFu;
    crc  = crc ^ temp ^ crc16_table[byte & 0xF];

    // Now compute checksum of upper four bits of a byte.
    temp         = crc16_table[crc & 0xF];
    crc  = (crc >> 4u) & 0x0FFFu;
    crc  = crc ^ temp ^ crc16_table[(byte >> 4u) & 0xF];

    return crc;
}


static uint16_t __attribute__((section("ota_app_loader_area"))) crc16(uint16_t seed, const volatile void * p_data, uint32_t size)
{
    uint8_t * p_block = (uint8_t *)p_data;

    while (size != 0)
    {
        seed = crc16_byte(seed, *p_block);
        p_block++;
        size--;
    }

   return seed;
}


int __attribute__((section("ota_app_loader_area"))) ota_flash_read(uint32_t* dest, uint32_t addr, uint32_t size)
{
  int i;
  if((((uint32_t)dest)%4) || (addr %4)|| (size%4))
    return PPlus_ERR_DATA_ALIGN;

  if(addr < OTAF_START_ADDR || addr >= OTAF_END_ADDR)
    return PPlus_ERR_INVALID_ADDR;

  for(i = 0; i < size ; i += 4)
    *dest++ = read_reg(addr + i);

  return PPlus_SUCCESS;
}



int __attribute__((section("ota_app_loader_area"))) ota_flash_load_app(void)
{
  int i;
  uint32_t partition_num = 0;
  uint32_t bank_info = 0;
  uint32_t bank_addr = 0;
  bool is_encrypt = FALSE;
  
  ota_flash_read(&partition_num, OTAF_2nd_BOOTINFO_ADDR, 4);
  if(partition_num == 0xffffffff)
    return PPlus_ERR_OTA_NO_APP;

  if(partition_num > OTAF_PARTITION_NUM_MAX || partition_num == 0)
    return PPlus_ERR_OTA_BAD_DATA;

  ota_flash_read(&bank_info, OTAF_2nd_BOOTINFO_ADDR + 4, 4);
  if(bank_info == OTAF_DUAL_BANK_1){
    bank_addr = OTAF_APP_BANK_1_ADDR;
  }
  else if(bank_info == OTAF_DUAL_BANK_0 || bank_info == OTAF_SINGLE_BANK){
    bank_addr = OTAF_APP_BANK_0_ADDR;
  }
  else{
    return PPlus_ERR_OTA_BAD_DATA;
  }
  
	is_encrypt = is_crypto_app();
  
  for(i = 1; i< partition_num+1; i++){

    uint32_t flash_addr;
    uint32_t run_addr;
    uint32_t size;
    uint32_t checksum;
    uint16_t crc;
    ota_flash_read(&flash_addr, OTAF_2nd_BOOTINFO_ADDR + i*4*4, 4);
    ota_flash_read(&run_addr,   OTAF_2nd_BOOTINFO_ADDR + i*4*4 + 4,  4);
    ota_flash_read(&size,       OTAF_2nd_BOOTINFO_ADDR + i*4*4 + 8,  4);
    ota_flash_read(&checksum,   OTAF_2nd_BOOTINFO_ADDR + i*4*4 + 12, 4);

    //case XIP mode, shoud be in single bank and no fct
    if(run_addr == flash_addr)
    {
      if(USE_FCT==0 && CFG_OTA_BANK_MODE==OTA_SINGLE_BANK)
        continue;
      return PPlus_ERR_INVALID_DATA;
    }

    //load binary
		if(is_encrypt){
      crc = crc16(0, (const volatile void * )(flash_addr + bank_addr), size);
      if(crc != (uint16)checksum){
        //if crc incorrect, reboot to OTA mode
        write_reg(OTA_MODE_SELECT_REG, OTA_MODE_OTA);
        NVIC_SystemReset();
      }

      flash_load_parition((uint8_t*)(flash_addr + bank_addr), (int)size, (uint8_t*)run_addr);
      //aes_ccm_phyplus_dec((const unsigned char*)0x200127e0, (uint8_t*)(flash_addr + bank_addr), (int)size, NULL, (uint8_t*)run_addr);
    }
    else
    {
      ota_flash_read((uint32_t*)run_addr, flash_addr + bank_addr, size);
      crc = crc16(0, (const volatile void * )run_addr, size);
      if(crc != (uint16)checksum){
        //if crc incorrect, reboot to OTA mode
        write_reg(OTA_MODE_SELECT_REG, OTA_MODE_OTA);
        NVIC_SystemReset();
      }
    }
  }

  return PPlus_SUCCESS;
  
}

#if(CFG_FLASH >= 512)
int ota_flash_load_fct(void)
{
  

  return PPlus_SUCCESS;
  
}
#endif




int ota_flash_read_bootsector(uint32_t* bank_addr)
{
  uint32_t partition_num = 0;
  uint32_t bank_info = 0;
	*bank_addr = OTAF_APP_BANK_0_ADDR;
  ota_flash_read(&partition_num, OTAF_2nd_BOOTINFO_ADDR, 4);
  if(partition_num == 0xffffffff)
    return PPlus_ERR_OTA_NO_APP;

  if(partition_num > OTAF_PARTITION_NUM_MAX)
    return PPlus_ERR_OTA_BAD_DATA;

  ota_flash_read(&bank_info, OTAF_2nd_BOOTINFO_ADDR + 4, 4);
  if(bank_info == OTAF_DUAL_BANK_0){
    *bank_addr = OTAF_APP_BANK_1_ADDR;
  }
  else if(bank_info == OTAF_DUAL_BANK_1 || bank_info == OTAF_SINGLE_BANK){
    *bank_addr = OTAF_APP_BANK_0_ADDR;
  }
  else{
    return PPlus_ERR_OTA_BAD_DATA;
  }
  return PPlus_SUCCESS;
}

int ota_flash_write_partition(uint32 addr, uint32_t* p_sect, uint32_t size)
{
  uint32_t i;
  int ret = 0;
  if(addr % 4)
    return PPlus_ERR_DATA_ALIGN;
  size = (size + 3) & 0xfffffffc;
  
  for(i = 0; i < size /4; i++){
    ret = WriteFlash(addr + i*4, p_sect[i]);
    if(ret == 1)
			continue;
    ret = WriteFlash(addr + i*4, p_sect[i]);

    if(ret == 1)
			continue;
    ret = WriteFlash(addr + i*4, p_sect[i]);

    if(ret == 1)
			continue;
    ret = WriteFlash(addr + i*4, p_sect[i]);
    if(ret == 0)
      return PPlus_ERR_SPI_FLASH;
  }
  return PPlus_SUCCESS;
}

int ota_flash_write_boot_sector(uint32_t* p_sect, uint32_t size, uint32_t offset)
{
  uint32_t i;
  int ret = 0;
  if(size % 4)
    return PPlus_ERR_DATA_ALIGN;

  for(i = 0; i < size /4; i++){
    ret = WriteFlash(OTAF_2nd_BOOTINFO_ADDR + i*4 + offset, p_sect[i]);
    if(ret == 0)
      ret = WriteFlash(OTAF_2nd_BOOTINFO_ADDR + i*4 + offset, p_sect[i]);
    if(ret == 0)
      ret = WriteFlash(OTAF_2nd_BOOTINFO_ADDR + i*4 + offset, p_sect[i]);
    if(ret == 0)
      ret = WriteFlash(OTAF_2nd_BOOTINFO_ADDR + i*4 + offset, p_sect[i]);
    if(ret == 0)
      return PPlus_ERR_SPI_FLASH;
  }
  return PPlus_SUCCESS;

}

int ota_flash_erase(uint32_t bank_addr)
{
  int i;
  if(CFG_OTA_BANK_MODE == OTA_SINGLE_BANK){
    if(bank_addr != OTAF_APP_BANK_0_ADDR)
      return PPlus_ERR_INVALID_PARAM;
    //erase boot sector
    flash_sector_erase(OTAF_2nd_BOOTINFO_ADDR);
    //erase application bank
    if(bank_addr%(64*1024) == 0){
      flash_block64_erase(bank_addr);
      flash_block64_erase(bank_addr + 64*1024);
    }
    else{
      for(i = 0; i< OTAF_APP_BANK_SIZE; i+= OTAF_SECTOR_SIZE)
        flash_sector_erase(OTAF_APP_BANK_0_ADDR + i);
    }
    return PPlus_SUCCESS;
  }
  else
  {
    if(bank_addr == OTAF_APP_BANK_0_ADDR || bank_addr == OTAF_APP_BANK_1_ADDR)
    {
      //erase application bank
      if(bank_addr%(64*1024) == 0){
        flash_block64_erase(bank_addr);
        flash_block64_erase(bank_addr + 64*1024);
      }
      else{
        for(i = 0; i< OTAF_APP_BANK_SIZE; i+= OTAF_SECTOR_SIZE)
          flash_sector_erase(bank_addr + i);
      }
      return PPlus_SUCCESS;
    }
  }
  return PPlus_ERR_INVALID_PARAM;
}


int ota_flash_erase_area(uint32_t flash_addr, uint32_t size)
{
    int ret = PPlus_ERR_INVALID_ADDR;
    int offset;
    flash_addr = flash_addr | 0x11000000;
    
    if(flash_addr >=OTAF_1st_BOOTINFO_ADDR + OTAF_1st_BOOTINFO_SIZE && (flash_addr + size) <= OTAF_2nd_BOOTINFO_ADDR ){
      ret = PPlus_SUCCESS;
    }
    if(flash_addr >=OTAF_2nd_BOOTINFO_ADDR + 4*1024 && (flash_addr + size) <= OTAF_APP_BANK_0_ADDR ){
      ret = PPlus_SUCCESS;
    }
    if(flash_addr >= OTAF_NVM_ADDR && (flash_addr + size) <= OTAF_END_ADDR +1){
      ret = PPlus_SUCCESS;
    }

    if(flash_addr & 0xfff || size & 0xfff){
      ret = PPlus_ERR_DATA_ALIGN;
    }

    if(ret)
      return ret;

    if((flash_addr & 0xffff  == 0 ) && (size & 0xffff == 0)){ //case 64K align, erase block
        for(offset= 0; offset < size; offset += 64*1024)
          flash_block64_erase(flash_addr + offset);
    }
    else{ //erase sector
        for(offset = 0; offset< size; offset+= OTAF_SECTOR_SIZE)
          flash_sector_erase(flash_addr + offset);

    }

    return PPlus_SUCCESS;

}



