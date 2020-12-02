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
* @file		flash.h
* @brief	Contains all functions support for flash driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef _FLASH_H_
#define _FLASH_H_

#include "types.h"
#include "gpio.h"



//define flash ucds
#define FLASH_UCDS_ADDR_BASE    0x11005000

#define SPIF_CONFIG_BASE        0x11080000
#define SPIF_RLEN               (SPIF_CONFIG_BASE+0x88)  
#define RFIFO_DEPTH             (SPIF_CONFIG_BASE+0x84)

// commands defines
#define PAGE_PROGRAM_CMD        0x2
#define FAST_PROGRAM_CMD        0xF2

#define WRITE_ENABLE_CMD        0x6
#define WRITE_DISABLE_CMD       0x4

#define READ_STATUS_CMD         0x5
#define WRITE_STATUS_CMD        0x1

#define SECT_ERASE_CMD          0x20
#define BLOCK32_ERASE_CMD       0x52
#define BLOCK64_ERASE_CMD       0xD8
#define CHIP_ERASE_CMD          0xC7

#define DEEP_DOWN_CMD           0xB9
#define RELEASE_DOWN_CMD        0xAB
#define MANUF_ID_CMD            0x90
#define READ_ID_CMD             0x9F

#define FAST_READ_CMD           0x0B
#define READ_DATA_CMD           0x3

#define WAIT_FOR_SPIF()         {while(!( (*(volatile uint32_t *)(SPIF_CONFIG_BASE+0x90)) & 0x4)) ;}

#define CHIP_ID_LENGTH          64
#define CHIP_ID_PID_LEN         16
#define CHIP_ID_LID_LEN         10
#define CHIP_ID_MID_LEN         16
#define CHIP_ID_TID_LEN         14
#define CHIP_ID_SID_LEN         8

#define CHIP_MADDR_LEN          6

typedef enum{
    CHIP_ID_UNCHECK,
    CHIP_ID_EMPTY,
    CHIP_ID_VALID,
    CHIP_ID_INVALID,
}CHIP_ID_STATUS_e;

typedef struct{
  CHIP_ID_STATUS_e chipIdStatus;
  uint8_t pid[CHIP_ID_PID_LEN];
  uint8_t lid[CHIP_ID_LID_LEN];
  uint8_t mid[CHIP_ID_MID_LEN];
  uint8_t tid[CHIP_ID_TID_LEN];
  uint8_t sid[CHIP_ID_SID_LEN];
}chipId_t;

typedef struct{
  CHIP_ID_STATUS_e chipMAddrStatus;
  uint8_t mAddr[CHIP_MADDR_LEN];
}chipMAddr_t;

  
extern chipId_t g_chipId;
extern chipMAddr_t g_chipMAddr;

void init_spif(void);
void close_spif(void);

void  read_deviceID(void);

int  read_status_register(void);
void write_status_register(int value);


void flash_block64_erase(unsigned int addr);
void flash_sector_erase(unsigned int addr);

int ProgramPage4 (unsigned long offset, uint32 data);
int ProgramPage64 (unsigned long offset, const unsigned char *buf,  int size);

int ProgramWord (unsigned long offset, const unsigned char *buf,uint8_t size) ;

int WriteFlash(unsigned int offset, uint32_t  value);
int WriteFlashShort(unsigned int offset, uint16_t  value);

uint8_t ReadFlash (unsigned int addr);
uint32_t ReadFlashWord (unsigned int addr);

uint8_t flash_write_ucds(uint32_t addr,uint32_t value);
uint32_t flash_read_ucds(uint32_t addr);
void flash_erase_ucds_all(void);
uint8_t flash_erase_ucds(uint32_t addr);
uint8_t flash_write_ucds_block(uint32_t addr,uint32_t length,uint32_t *buf);
uint8_t flash_read_ucds_block(uint32_t addr,uint32_t length,uint32_t *tobuf);
uint8_t flash_write_ucds_block_byte(uint32_t addr,uint32_t length,uint8_t *buf);
uint8_t flash_read_ucds_block_byte(uint32_t addr,uint32_t length,uint8_t *tobuf);


void check_chip_id(void);
void check_chip_mAddr(void);
void LOG_CHIP_ID(void);
void LOG_CHIP_MADDR(void);

/*******************************************************************************
 * @fn          pplus_enter_programming_mode
 *
 * @brief       force deive enter to programing mode.
 *
 * input parameters
 *
 * @param       none.
 *
 * output parameters
 *
 * @param       none.
 *
 * @return      none.
 */
void pplus_enter_programming_mode(void);


/*******************************************************************************
 * @fn          pplus_LoadMACFromChipMAddr
 *
 * @brief       Used to load MAC Address from chip Maddr
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      CHIP_ID_STATUS_e.
 */
extern CHIP_ID_STATUS_e pplus_LoadMACFromChipMAddr(void);

#endif








