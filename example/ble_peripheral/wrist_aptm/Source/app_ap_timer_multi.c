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


#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_clock.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "wristservice.h"
#include "ota_app_service.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "app_wrist.h"
#include "pwrmgr.h"
#include "ui_page.h"
#include "ui_task.h"
#include "ui_display.h"
#include "ui_dispTFT.h"
#include "touch_key.h"
#include "em70xx.h"
#include "KX023.h"
#include "ap_timer.h"
#include "battery.h"
#include "led_light.h"
#include "app_ap_timer_multi.h"
#include "string.h"
#include "hal_mcu.h"
#include "clock.h"
#include "error.h"
#include "log.h"


typedef struct{
  bool            enable;
  uint32_t        systick_snap;
  uint16_t        osal_event;
  uint32_t        timeout;
  app_aptm_hdl_t  callback;
}app_aptm_ch_t;

typedef struct{
  uint8_t         taskID;
  app_aptm_ch_t   channel[4];
}app_aptm_ctx_t;

app_aptm_ctx_t s_aptm_ctx;


void app_aptimer_osal_timer_handler_m(uint8_t id)
{
  ap_timer_set(id, 2);
  hal_pwrmgr_lock(MOD_TIMER);
}


void on_ap_timer_m(uint8_t event)
{
  int diff;
  int i;
  uint8_t taskID = s_aptm_ctx.taskID;
  app_aptm_ch_t* pchn = NULL;
  //osal_stop_timerEx(AppWrist_TaskID, TIMER_AP_TM_EVT);
  switch(event){
  case APTM_EVT_TIMER_1:
  case APTM_EVT_TIMER_2:
  case APTM_EVT_TIMER_3:
  case APTM_EVT_TIMER_4:
    pchn = &(s_aptm_ctx.channel[event -1]);
    if(pchn->enable == false)
      break;
      
  	hal_pwrmgr_unlock(MOD_TIMER);
  	ap_timer_clear(event -1);
  	osalTimeUpdate();
    pchn->systick_snap = hal_systick();
  	ap_timer_set(event -1, pchn->timeout*1000);
  	osal_clear_event(taskID, pchn->osal_event);
    osal_stop_timerEx(taskID, pchn->osal_event);
    osal_start_timerEx(taskID, pchn->osal_event, pchn->timeout);
    if(event == APTM_EVT_TIMER_1){
      hal_gpio_write(P20, 1);
      hal_gpio_write(P20, 0);
    }
    else if(event == APTM_EVT_TIMER_2){
      hal_gpio_write(P19, 1);
      hal_gpio_write(P19, 0);
    }
    else if(event == APTM_EVT_TIMER_3){
      hal_gpio_write(P18, 1);
      hal_gpio_write(P18, 0);
    }
    else if(event == APTM_EVT_TIMER_4){
      //hal_gpio_write(P20, 1);
      //hal_gpio_write(P20, 0);
    }
    if(pchn->callback)
      pchn->callback();
    break;
  case APTM_EVT_WAKEUP:
    //hal_gpio_write(P19, 1);
    //hal_gpio_write(P19, 0);
    for(i = 0; i< 4; i++){
      pchn = &(s_aptm_ctx.channel[i]);
      if(pchn->enable == false)
        continue;
      diff = hal_ms_intv(pchn->systick_snap);
      if(diff >= pchn->timeout-1){
        pchn->systick_snap = hal_systick();
      	ap_timer_set(i, 2);
      	hal_pwrmgr_lock(MOD_TIMER);
      	osal_clear_event(taskID, pchn->osal_event);
        osal_stop_timerEx(taskID, pchn->osal_event);
        osal_start_timerEx(taskID, pchn->osal_event, pchn->timeout);
      }
      else
      {
      	ap_timer_set(i, (pchn->timeout-diff)*1000);
        osal_start_timerEx(taskID, pchn->osal_event, (pchn->timeout-diff));
      	osal_clear_event(taskID, pchn->osal_event);
      }
    }
    break;
  case APTM_EVT_SLEEP:
    //hal_gpio_write(P18, 1);
    //hal_gpio_write(P18, 0);
    break;

  default:
    break;
  }

}


int app_ap_timer_start_m(uint8_t tmID, uint32_t ms, app_aptm_hdl_t callback)
{
  int ret;
  int i;
  app_aptm_ch_t* pchn = &(s_aptm_ctx.channel[tmID]);
  bool is_enable= false;

  if(s_aptm_ctx.taskID == 0)
    return PPlus_ERR_NOT_REGISTED;
  
  if(pchn->enable)
    return PPlus_ERR_BUSY;
  if(pchn->osal_event == 0)
    return PPlus_ERR_NOT_REGISTED;

  for(i = 0; i< 4; i++){
    if(s_aptm_ctx.channel[i].enable){
      is_enable = true;
      break;
    }
  }

  if(!is_enable){
    ret = ap_timer_init(on_ap_timer_m);
  	if(ret != PPlus_SUCCESS)
  	  return ret;
  }

  
  pchn->systick_snap = hal_systick();
  
 	ret = ap_timer_set(tmID, ms*1000);
	if(ret != PPlus_SUCCESS)
	  return ret;
 	
  if(osal_start_timerEx(s_aptm_ctx.taskID, pchn->osal_event, ms) != SUCCESS)
    return PPlus_ERR_INVALID_PARAM;

  pchn->enable = true;
  pchn->timeout = ms;
  pchn->callback = callback;
  
	return PPlus_SUCCESS;

}


int app_ap_timer_stop_m(uint8_t tmID){
  int ret;
  int i;
  app_aptm_ch_t* pchn = &(s_aptm_ctx.channel[tmID]);
  bool is_enable= false;
  bool enable= pchn->enable;

  pchn->enable = false;

  for(i = 0; i< 4; i++){
    if(s_aptm_ctx.channel[i].enable){
      is_enable = true;
      break;
    }
  }

  if(is_enable == false && enable){
    ret = ap_timer_deinit();
  }

  if(pchn->osal_event){
   	osal_clear_event(s_aptm_ctx.taskID, pchn->osal_event);
    osal_stop_timerEx(s_aptm_ctx.taskID, pchn->osal_event);
  }
  
  pchn->systick_snap = 0;;

  pchn->timeout = 0;
  pchn->callback = NULL;

  return ret;
}


int app_ap_timer_init_m(uint8_t taskID, uint16_t osal_evt_1, uint16_t osal_evt_2, uint16_t osal_evt_3, uint16_t osal_evt_4)
{
  memset(&s_aptm_ctx, 0, sizeof(s_aptm_ctx));
  s_aptm_ctx.taskID =taskID;
  s_aptm_ctx.channel[AP_TM_ID_1].osal_event = osal_evt_1;
  s_aptm_ctx.channel[AP_TM_ID_2].osal_event = osal_evt_2;
  s_aptm_ctx.channel[AP_TM_ID_3].osal_event = osal_evt_3;
  s_aptm_ctx.channel[AP_TM_ID_4].osal_event = osal_evt_4;
  return PPlus_SUCCESS;
}


