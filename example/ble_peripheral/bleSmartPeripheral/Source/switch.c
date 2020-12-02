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


/************************************************************** 
 * COPYRIGHT(c)2018, Phy+.
 * All rights reserved. 
 *
 *

****************************************************************/
#include "types.h"
#include "bleSmartPeripheral.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "string.h"
#include "gpio.h"
#include "error.h"
#include "log.h"
#include "switch.h"
#include "pwrmgr.h"

uint32 switch_init(switch_cfg_t *cfg){
  
  uint8 length=(sizeof(cfg)/sizeof(cfg[0]));
//  LOG("length%d",length);
  
  for(uint8 i=0;i<length;i++){
    if(cfg[i].type==LOW){
    hal_gpio_write(cfg[i].led_pin,1);
  }else if(cfg[i].type==HIGH){
    hal_gpio_write(cfg[i].led_pin,0);
  }else{
    return PPlus_ERR_INVALID_PARAM;
  }
  }
  
  
  hal_pwrmgr_register(MOD_USR3,NULL,NULL);
  
  return PPlus_SUCCESS;
}

uint32 switch_on(switch_cfg_t cfg){
  hal_pwrmgr_lock(MOD_USR3);
  if(cfg.type==LOW){
    hal_gpio_write(cfg.led_pin,0);
  }else if(cfg.type==HIGH){
    hal_gpio_write(cfg.led_pin,1);
  }else{
    return PPlus_ERR_INVALID_PARAM;
  }
  return PPlus_SUCCESS;
}

uint32 switch_off(switch_cfg_t cfg){
  hal_pwrmgr_unlock(MOD_USR3);
  if(cfg.type==LOW){
    hal_gpio_write(cfg.led_pin,1);
  }else if(cfg.type==HIGH){
    hal_gpio_write(cfg.led_pin,0);
  }else{
    return PPlus_ERR_INVALID_PARAM;
  }
  return PPlus_SUCCESS;
}



