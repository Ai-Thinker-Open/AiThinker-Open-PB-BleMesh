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
#include "app_ap_timer.h"
#include "hal_mcu.h"
#include "clock.h"
#include "error.h"
#include "log.h"

static uint32_t s_systick_snap = 0;;

static uint8_t s_taskID = 0;
static uint16_t s_osal_event = 0;
static uint32_t s_timeout = 0;
static app_aptm_hdl_t s_callback;

void app_aptimer_osal_timer_handler(void)
{
  ap_timer_set(0, 2);
  hal_pwrmgr_lock(MOD_TIMER);
}


void on_ap_timer(uint8_t event)
{
  int diff;
  //osal_stop_timerEx(AppWrist_TaskID, TIMER_AP_TM_EVT);
  switch(event){
  case APTM_EVT_TIMER_1:
    
  	hal_pwrmgr_unlock(MOD_TIMER);
  	ap_timer_clear(0);
  	osalTimeUpdate();
    s_systick_snap = hal_systick();
  	ap_timer_set(0, s_timeout*1000);
  	osal_clear_event(s_taskID, s_osal_event);
    osal_stop_timerEx(s_taskID, s_osal_event);
    osal_start_timerEx(s_taskID, s_osal_event, s_timeout);
    hal_gpio_write(P20, 1);
    hal_gpio_write(P20, 0);
    if(s_callback)
      s_callback();
    break;
  case APTM_EVT_WAKEUP:
    diff = hal_ms_intv(s_systick_snap);
    //LOG("!%d\n",diff);
    hal_gpio_write(P19, 1);
    hal_gpio_write(P19, 0);
    if(diff >= s_timeout-1){
      s_systick_snap = hal_systick();
    	//ap_timer_clear(0);
    	ap_timer_set(0, 2);
    	hal_pwrmgr_lock(MOD_TIMER);
    	osal_clear_event(s_taskID, s_osal_event);
      osal_stop_timerEx(s_taskID, s_osal_event);
      osal_start_timerEx(s_taskID, s_osal_event, s_timeout);
      //hal_gpio_write(P20, 1);
      //hal_gpio_write(P20, 0);
    }
    else
    {
    	ap_timer_set(0, (s_timeout-diff)*1000);
    	//hal_pwrmgr_lock(MOD_USR2);
      //osal_stop_timerEx(AppWrist_TaskID, s_osal_event);
      osal_start_timerEx(s_taskID, s_osal_event, (s_timeout-diff));
    }
    break;
  case APTM_EVT_SLEEP:
    hal_gpio_write(P18, 1);
    hal_gpio_write(P18, 0);
    break;

  default:
    break;
  }

}


int app_ap_timer_start(uint8_t taskID, uint16_t osal_evt, uint32_t ms, app_aptm_hdl_t callback)
{
  int ret;
  if(s_timeout)
    return PPlus_ERR_BUSY;
  s_systick_snap = hal_systick();
	ret = ap_timer_init(on_ap_timer);
	if(ret != PPlus_SUCCESS)
	  return ret;
 	ret = ap_timer_set(0, ms*1000);
	if(ret != PPlus_SUCCESS)
	  return ret;
 	
  if(osal_start_timerEx(taskID, osal_evt, ms) != SUCCESS)
    return PPlus_ERR_INVALID_PARAM;

  s_taskID = taskID;
  s_osal_event = osal_evt;
  s_timeout = ms;
  s_callback = callback;
  
	return PPlus_SUCCESS;

}


int app_ap_timer_stop(void){
  if(s_timeout){
    ap_timer_deinit();
    
  	osal_clear_event(s_taskID, s_osal_event);
    osal_stop_timerEx(s_taskID, s_osal_event);
    s_systick_snap = 0;;

    s_taskID = 0;
    s_osal_event = 0;
    s_timeout = 0;
    s_callback = NULL;

  }
  return PPlus_SUCCESS;
}

