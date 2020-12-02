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

/**************************************************************************************************
  Filename:       otaupgrade.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"

#include "otaboot_hex.c"
#include "otaupgrade.h"
#include "ota_flash.h"
#include "flash.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"

const uint32 s_magiccode[2] = {0x221342d3, 0xf2452345}; 

uint8 OTAUG_TaskID;
 

void OTAUG_Init(uint8 task_id)
{
	unsigned int i;
	int flash_size = CFG_FLASH;
	unsigned int* p_app = (unsigned int *)ota_apptable;
	unsigned int addr_1st_boot, addr_2nd_boot, addr_ota_fw;
	OTAUG_TaskID = task_id;
	LOG("OTA Boot loader upgrade...\n");
	LOG("0x%x\n", sizeof(ota_boottable));
	LOG("0x%x\n", sizeof(ota_apptable));
  if(flash_size == 512)//case 512k
  {
    addr_1st_boot = 0x2000;
    addr_2nd_boot = OTAF_2nd_BOOTINFO_ADDR;
    addr_ota_fw   = 0xa000;
	}
	else//case 128k
	{
    addr_1st_boot = 0x2000;
    addr_2nd_boot = OTAF_2nd_BOOTINFO_ADDR;
    addr_ota_fw   = 0x4000;
	}
  flash_sector_erase(addr_1st_boot);
  flash_sector_erase(addr_2nd_boot);
	for(i = 0; i< 8; i++)
		flash_sector_erase(addr_ota_fw+i*4096);
	
	for(i = 0; i< sizeof(ota_apptable)/4; i++){
    WriteFlash(addr_ota_fw + i*4, p_app[i]);
	}

	for(i = 0; i< 2; i++){
    WriteFlash(addr_ota_fw + sizeof(ota_apptable)+i*4, s_magiccode[i]);
	}
		
	for(i = 0; i< sizeof(ota_boottable)/4; i++)
    WriteFlash(addr_1st_boot+0x100 + i*4, ota_boottable[i]);

  WaitMs(100);

  NVIC_SystemReset();
  
}

uint16 OTAUG_ProcessEvent( uint8 task_id, uint16 events )
{
	

  // Discard unknown events
  return 0;
}

/*********************************************************************
*********************************************************************/
