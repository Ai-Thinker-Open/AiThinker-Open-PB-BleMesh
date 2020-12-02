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


#include "gpio.h"
#include "touch_key.h"
#include "log.h"
#include "OSAL.h"
#include "Sensor_Broadcast.h"
#include "pwrmgr.h"
#include "common.h"


extern uint8 Sensor_Broadcast_TaskID;
static key_contex_t key_state;

extern uint32 getMcuPrecisionCount(void);

static int key_timer_start(uint32 intval_ms)
{
    osal_start_timerEx(Sensor_Broadcast_TaskID, TIMER_KEY_EVT, intval_ms);
    return 0;
}

static int key_timer_long_press(uint32 intval_ms)
{
    osal_start_timerEx(Sensor_Broadcast_TaskID, KEY_LONG_PRESS_EVT, intval_ms);
    return 0;
}

static int key_timer_long_press_stop(void)
{
    osal_stop_timerEx(Sensor_Broadcast_TaskID, KEY_LONG_PRESS_EVT);
    return 0;
}

//static int key_timer_stop(void)
//{
//  osal_stop_timerEx(Sensor_Broadcast_TaskID, TIMER_KEY_EVT);
//  return 0;
//}


static void key_idle_handler(IO_Wakeup_Pol_e type){
  if(type == POSEDGE){
    hal_pwrmgr_lock(MOD_USR1);
    key_state.state=STATE_KEY_PRESS_DEBONCE;
    key_timer_start(20);
  }
}


static void key_press_debonce_handler(IO_Wakeup_Pol_e type){
  if(type == POSEDGE){
    key_timer_start(20);
  } 
}

static void key_press_handler(IO_Wakeup_Pol_e type){
  if(type == NEGEDGE){
//    hal_pwrmgr_lock(MOD_USR1);
    key_state.state=STATE_KEY_RELEASE_DEBONCE;
    gpio_key_timer_handler();
//    key_timer_start(20);
  } 
}

//static void key_release_debonce_handler(IO_Wakeup_Pol_e type){
//  if(type == NEGEDGE){
//    key_timer_start(20);
//  } 
//}

static void pin_event_handler(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
//  LOG("key_state");
  switch(key_state.state){
    case STATE_KEY_IDLE:
      key_idle_handler(type);
      break;
    case STATE_KEY_PRESS_DEBONCE:
      key_press_debonce_handler(type);
      break;
    case STATE_KEY_PRESS:
      key_press_handler(type);
      break;
//    case STATE_KEY_RELEASE_DEBONCE:
//      key_release_debonce_handler(type);
//      break;
  }
}

int touch_init(key_callbank_hdl_t hdl)
{
  
//  LOG("touch_init");
	uint32_t ret;
  
  key_state.key_callbank=hdl;
  key_state.state=STATE_KEY_IDLE;
  key_state.timer_tick=0;

	
	ret = hal_gpioin_register(P14, pin_event_handler, pin_event_handler);//pin_event_handler);
  hal_pwrmgr_register(MOD_USR1, NULL, NULL);

//	LOG("touch pin input enable %d\n", ret);
  return ret;
}

static void key_press_debonce_timer_handler(void){
    if(hal_gpio_read(P14)==1){
        osal_stop_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV);
//        LOG("stop");
//        osal_start_timerEx(Sensor_Broadcast_TaskID, KEY_LONG_PRESS_EVT, 3*1000);
        key_timer_long_press(3000);
//        hal_pwrmgr_unlock(MOD_USR1);
        key_state.state=STATE_KEY_PRESS;
        key_state.timer_tick=getMcuPrecisionCount();
        if(key_state.key_callbank!=NULL){ 
          key_state.key_callbank(TOUCH_EVT_PRESS);
//          LOG("press");
        }
     }else if(hal_gpio_read(P14)==0){
        hal_pwrmgr_unlock(MOD_USR1);
        key_state.state=STATE_KEY_IDLE;
       } 
}

static void key_release_debonce_timer_handler(void){
    if(hal_gpio_read(P14)==0){
        uint32_t hold_tick = (getMcuPrecisionCount()-key_state.timer_tick)*625;
//      LOG("hold_tick:%d",hold_tick);
        hal_pwrmgr_unlock(MOD_USR1);
        key_state.state=STATE_KEY_IDLE;
        if(key_state.key_callbank!=NULL){
          key_timer_long_press_stop();
          if(hold_tick >= 3*1000*1000){
//            LOG("long");
            key_state.key_callbank(TOUCH_EVT_LONG_PRESS);
          }else if(hold_tick <= 800*1000){
//            key_timer_long_press_stop();
//            LOG("short");
            key_state.key_callbank(TOUCH_EVT_SHORT_PRESS);
          }else{
//            key_timer_long_press_stop();
//            LOG("release");
            key_state.key_callbank(TOUCH_EVT_RELEASE);
          }
        }
     }else if(hal_gpio_read(P14)==1){
        key_state.state=STATE_KEY_PRESS;
       } 
}

void gpio_key_timer_handler(void){
  switch(key_state.state){
    case STATE_KEY_PRESS_DEBONCE:
      key_press_debonce_timer_handler();
      break;
    case STATE_KEY_RELEASE_DEBONCE:
      key_release_debonce_timer_handler();
      break;
  }
          
}


