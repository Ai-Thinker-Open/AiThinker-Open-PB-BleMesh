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


#ifndef _OTA_FLASH_H
#define _OTA_FLASH_H

#include "types.h"

#define OTAF_PARTITION_NUM_MAX  16

enum{
  OTAF_SINGLE_BANK = 0,
  OTAF_DUAL_BANK_0  = 1,
  OTAF_DUAL_BANK_1  = 2
};



#define OTA_MAGIC_CODE    0x32af08cb  //random data

#define OTAF_SECTOR_SIZE  (1024*4)  //4k size

#define OTA_DUMMY_BANK  0 //undefined mode, will report error
#define OTA_SINGLE_BANK 1
#define OTA_DUAL_BANK   2

#ifndef CFG_OTA_BANK_MODE
  #define CFG_OTA_BANK_MODE OTA_DUMMY_BANK
#endif

#ifndef USE_FCT
#define USE_FCT 0
#endif

#ifndef CFG_FLASH
#define CFG_FLASH 0
#endif

#if(CFG_FLASH == 128 && USE_FCT==0 && CFG_OTA_BANK_MODE==OTA_SINGLE_BANK)

  #define OTAF_BASE_ADDR          0x11000000

  #define OTAF_1st_BOOTINFO_ADDR  0x11002000  //4k bytes
  #define OTAF_1st_BOOTINFO_SIZE  0x1000  //4k bytes

  #define OTAF_2nd_BOOTINFO_ADDR  0x11003000  //4k bytes
  #define OTAF_APP_BANK_0_ADDR    0x1100c000  //72K bytes
  #define OTAF_APP_BANK_1_ADDR    OTA_MAGIC_CODE  //Dummy

  #define OTAF_APP_BANK_SIZE      (1024*64)
  
  #define OTAF_NVM_ADDR           0x1100c000  //8K bytes


  #define OTAF_APPLICATION_RUN    0x1fff4000  //??? need confirm

#elif(CFG_FLASH >= 512 && USE_FCT==1 && CFG_OTA_BANK_MODE==OTA_DUAL_BANK)

  #define OTAF_BASE_ADDR          0x11000000

  #define OTAF_1st_BOOTINFO_ADDR  0x11002000  //4k bytes
  #define OTAF_1st_BOOTINFO_SIZE  0x1000  //4k bytes
  
  #define OTAF_2nd_BOOTINFO_ADDR  0x11009000  //4k bytes
  #define OTAF_APP_FCT_ADDR       0x11012000  //120K bytes, first 4k is FCT boot info
  #define OTAF_APP_BANK_0_ADDR    0x11030000  //128K bytes
  #define OTAF_APP_BANK_1_ADDR    0x11050000  //128K bytes

  #define OTAF_APP_BANK_SIZE      (1024*128)

  #define OTAF_NVM_ADDR           0x11070000  //64K bytes


  #define OTAF_APPLICATION_RUN    0x1fff4000  //??? need confirm

#elif(CFG_FLASH >= 512 && USE_FCT==0 && CFG_OTA_BANK_MODE==OTA_DUAL_BANK)

  #define OTAF_BASE_ADDR          0x11000000

  #define OTAF_1st_BOOTINFO_ADDR  0x11002000  //4k bytes
  #define OTAF_1st_BOOTINFO_SIZE  0x1000  //4k bytes
  
  #define OTAF_2nd_BOOTINFO_ADDR  0x11009000  //4k bytes
  #define OTAF_APP_BANK_0_ADDR    0x11012000  //128K bytes
  #define OTAF_APP_BANK_1_ADDR    0x11032000  //128K bytes

  #define OTAF_APP_BANK_SIZE      (1024*128)

  #define OTAF_NVM_ADDR           0x11052000  //64K bytes


  #define OTAF_APPLICATION_RUN    0x1fff4000  //??? need confirm

#elif(CFG_FLASH >= 512 && USE_FCT==1 && CFG_OTA_BANK_MODE==OTA_SINGLE_BANK)

  #define OTAF_BASE_ADDR          0x11000000

  #define OTAF_1st_BOOTINFO_ADDR  0x11002000  //4k bytes
  #define OTAF_1st_BOOTINFO_SIZE  0x1000  //4k bytes
  
  #define OTAF_2nd_BOOTINFO_ADDR  0x11009000  //4k bytes
  #define OTAF_APP_FCT_ADDR       0x11012000  //120K bytes, first 4k is FCT boot info
  #define OTAF_APP_BANK_0_ADDR    0x11030000  //128K bytes
  #define OTAF_APP_BANK_1_ADDR    OTA_MAGIC_CODE  //Dummy

  #define OTAF_APP_BANK_SIZE      (1024*128)

  #define OTAF_NVM_ADDR           0x11050000  //64K bytes


  #define OTAF_APPLICATION_RUN    0x1fff4000  //??? need confirm

#elif(CFG_FLASH >= 512 && USE_FCT==0 && CFG_OTA_BANK_MODE==OTA_SINGLE_BANK)

  #define OTAF_BASE_ADDR          0x11000000

  #define OTAF_1st_BOOTINFO_ADDR  0x11002000  //4k bytes
  #define OTAF_1st_BOOTINFO_SIZE  0x1000  //4k bytes

  #define OTAF_2nd_BOOTINFO_ADDR  0x11009000  //4k bytes
  #define OTAF_APP_BANK_0_ADDR    0x11012000  //128K bytes
  #define OTAF_APP_BANK_1_ADDR    OTA_MAGIC_CODE  //Dummy

  #define OTAF_APP_BANK_SIZE      (1024*128)

  #define OTAF_NVM_ADDR           0x11032000  //64K bytes


  #define OTAF_APPLICATION_RUN    0x1fff4000  //??? need confirm

#else
  #error "unsupported OTA config, please check these micro:CFG_FLASH, USE_FCT,CFG_OTA_BANK_MODE!"
#endif


#if(CFG_FLASH == 128)
  #define OTAF_START_ADDR         0x11003000
  #define OTAF_END_ADDR           0x1101ffff
#elif(CFG_FLASH == 512)
  #define OTAF_START_ADDR         0x11009000
  #define OTAF_END_ADDR           0x1107ffff
#elif(CFG_FLASH == 4096)
  #define OTAF_START_ADDR         0x11009000
  #define OTAF_END_ADDR           0x111fffff
#else
  #error "unsupported OTA config, please check these micro:CFG_FLASH!"
#endif

#define MAX_SECT_SUPPORT  16


typedef struct{
  uint32_t  flash_addr;
  uint32_t  run_addr;
  uint32_t  size;
  uint16_t  checksum;
}ota_fw_part_t;

typedef struct{
  uint8_t           part_num;
  uint8_t           part_current;
  uint32_t          total_size;
  uint32_t          total_offset;
  uint32_t          offset;
  ota_fw_part_t     part[MAX_SECT_SUPPORT];
}ota_fw_t;


#if(CFG_FLASH >= 512)
int ota_flash_load_fct(void);
#endif
int ota_flash_load_app(void);
int ota_flash_write_partition(uint32 addr, uint32_t* p_sect, uint32_t size);
int ota_flash_write_boot_sector(uint32_t* p_sect, uint32_t size, uint32_t offset);
int ota_flash_erase(uint32_t addr);
int ota_flash_erase_area(uint32_t flash_addr, uint32_t size);
int ota_flash_read_bootsector(uint32_t* bank_addr);

#endif

