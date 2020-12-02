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
 *
 * Module Name: UI page management
 * File name: ui_page.c 
 * Brief description:
 *    UI page management 
 * Author:  Eagle.Lao
 * Revision:V0.01
****************************************************************/

#include "ui_task.h"
#include "ui_page.h"
#include "ui_display.h"
#include "ui_dispTFT.h"
#include "ui_disp_substance.h"
#include "app_datetime.h"
#include "OSAL_timers.h"
#include "app_wrist.h"
#include <string.h>
#include "battery.h"
#include "ui_font.h"
#include "em70xx.h"
#include "kx023.h"
#include "log.h"

enum{
  UI_STYLE_SIMPLE,
  UI_STYLE_FULL
};

enum{
  UI_ST_BLANK = 0,
  UI_ST_NORMAL,
  UI_ST_POP
};


enum{
  UI_STYLE_DEFAULT = 0,
  UI_STYLE_1,
  UI_STYLE_2
};

enum{
  PAGE_ID_HOME = 0,
  PAGE_ID_PEDOMETER,
  PAGE_ID_CALORIE,
  PAGE_ID_MILE,
  PAGE_ID_SLEEP,
  PAGE_ID_HEARTRATE,
  PAGE_ID_TEMPERATUE,
  PAGE_NUM_NORMAL,
};
typedef int (* ui_evhdl_t)(const ui_ev_t*);

typedef struct{
  uint16_t  ev_type;
  ui_evhdl_t  ev_handler;
}ui_page_t;


typedef struct{
  uint8_t state;
  uint8_t index;
  uint8_t subst;
  ui_ev_t ev;
  uint8_t style;
  bool page_mask[PAGE_NUM_NORMAL];
  bool acc_dbg;
  uint32_t value;
  char ancs_attr[64];
  uint16_t msg_ev;
  char* msg_title;
  char* msg_info;
  uint8_t* data;
}ui_page_cfg_t;

static ui_page_cfg_t s_pagecfg = {
            .state = UI_ST_BLANK,
            .index = 0,
            .acc_dbg = FALSE};




static int ui_timer_start(uint32 intval_ms)
{
    osal_start_timerEx(AppWrist_TaskID, TIMER_UI_EVT, intval_ms);
    return 0;
}
static int ui_timer_stop(void)
{
  osal_stop_timerEx(AppWrist_TaskID, TIMER_UI_EVT);
  return 0;
}



static int ui_page_next(void);
static int ui_page_blank(void);

static void ui_fsm_destruct(void)
{
  ui_ev_t ev_destruct = {
          .ev = UI_EV_PAGE_DESTRUCT,
          .param = 0,
          .data = 0,
        };

  ui_fsm_run(&ev_destruct);
}


static int uipg_key_press_long(const ui_ev_t* pev)
{
  return 0;
}



static int uipg_call_in(const ui_ev_t* pev)
{
  ui_ev_t ev_construct = {
          .ev = UI_EV_PAGE_CONSTRUCT,
          .param = 0,
          .data = 0,
        };

  LOG("uipg_call_in\n");
  ui_fsm_destruct();
  s_pagecfg.state = UI_ST_POP;
  s_pagecfg.index = POP_INDEX_CALL;
  s_pagecfg.data = 0;
  

  return ui_fsm_run(&ev_construct);
}


static int uipg_push_msg(const ui_ev_t* pev)
{
  ui_ev_t ev_construct = {
          .ev = UI_EV_PAGE_CONSTRUCT,
          .param = pev->ev,
          .data = pev->data,
        };

  LOG("uipg_push_msg\n");
  ui_fsm_destruct();
  s_pagecfg.state = UI_ST_POP;
  s_pagecfg.index = POP_INDEX_MSG;
  s_pagecfg.data = 0;
  

  return ui_fsm_run(&ev_construct);
}




static int uipg_batt_charge(const ui_ev_t* pev)
{
  ui_ev_t ev_construct = {
          .ev = UI_EV_PAGE_CONSTRUCT,
          .param = 0,
          .data = 0,
        };

  LOG("uipg_charge_auth\n");
  
  ui_fsm_destruct();
  
  s_pagecfg.state = UI_ST_POP;
  s_pagecfg.index = POP_INDEX_BATT;
  s_pagecfg.data = 0;
  
  ui_timer_stop();

  return ui_fsm_run(&ev_construct);
}


static int uipg_common_destruct(const ui_ev_t* pev)
{
  s_pagecfg.subst = 0;
  ui_timer_stop();
  return APP_SUCCESS;
  
}

static int uipg_blank_construct(const ui_ev_t* pev)
{
  disp_rect_t rect;

  lcd_bus_init();

  disp_off();
  
  //clear frame buffer
  DISP_RECT_FULL(rect);
  disp_clrscn(rect);
  disp_reflash_all();
  return APP_SUCCESS;
}

static int uipg_blank_key_press(const ui_ev_t* pev)
{
  if(0){//batt_charge_detect()){
    uipg_batt_charge(pev);
    return 0;
  }
  s_pagecfg.ev = *pev;
  ui_page_next();
  
  return 0;
}

static int uipg_blank_hand_up(const ui_ev_t* pev)
{
  if(0){//batt_charge_detect()){
    uipg_batt_charge(pev);
    return 0;
  }
  s_pagecfg.ev = *pev;
  ui_page_next();
  
  return 0;
}


static int uipg_blank_timer(const ui_ev_t* pev)
{
  return 0;
}

const ui_page_t s_page_blank[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_blank_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_HAND_UP,     uipg_blank_hand_up},
  {UI_EV_KEY_PRESS,   uipg_blank_key_press},
  {UI_EV_KEY_PRESS_LONG,  uipg_blank_key_press},
  {UI_EV_TIMER,     uipg_blank_timer},
  {UI_EV_BLE_CALL,    uipg_call_in},
  {UI_EV_BATT_CHARGE,   uipg_batt_charge},
  {UI_EV_BLE_SMS,     uipg_push_msg},
  {UI_EV_BLE_WECHAT,    uipg_push_msg},
  {UI_EV_BLE_QQ,      uipg_push_msg},
  {UI_EV_BLE_MSG_NOTIFY,  uipg_push_msg},
  {0,0}
};

static int uipg_home_construct(const ui_ev_t* pev)
{
  datetime_t dtm = {0};
  app_datetime(&dtm);
  ui_normal_blank_on_style_simple(&dtm, NULL);
  ui_timer_start(3000);
  return APP_SUCCESS;
}

static int uipg_home_key_press(const ui_ev_t* pev)
{
  ui_page_next();
  return 0;
}

static int uipg_home_hand_down(const ui_ev_t* pev)
{
  if(s_pagecfg.ev.ev == UI_EV_HAND_UP)
    return ui_page_blank();
  return APP_SUCCESS;
}


static int uipg_home_timer(const ui_ev_t* pev)
{
  return ui_page_blank();
}

const ui_page_t s_page_home[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_home_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_home_key_press},
  {UI_EV_HAND_DOWN,   uipg_home_hand_down},
  {UI_EV_KEY_PRESS_LONG,  uipg_key_press_long},
  {UI_EV_TIMER,     uipg_home_timer},
  {UI_EV_BLE_CALL,    uipg_call_in},
  {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_CALL,    uipg_call_in},
    {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_SMS,     uipg_push_msg},
    {UI_EV_BLE_WECHAT,    uipg_push_msg},
    {UI_EV_BLE_QQ,      uipg_push_msg},
    {UI_EV_BLE_MSG_NOTIFY,  uipg_push_msg},
  {0,0}
};


static int uipg_pedometer_construct(const ui_ev_t* pev)
{
  ui_normal_acc_init();
  ui_timer_stop();
  //kx023_enable();
  return APP_SUCCESS;
}
static int uipg_pedometer_destruct(const ui_ev_t* pev)
{
  //kx023_disable();
  s_pagecfg.subst = 0;
  ui_timer_stop();
  return APP_SUCCESS;
}

static int uipg_pedometer_acc_data(const ui_ev_t* pev)
{
  int* pacc = (int*)(pev->data);
  ui_normal_acc(pacc[0],pacc[1],pacc[2]);
  return APP_SUCCESS;
}

static int uipg_pedometer_key_press(const ui_ev_t* pev)
{
  return ui_page_next();
}


static int uipg_pedometer_key_press_long(const ui_ev_t* pev)
{
  //s_pagecfg.acc_dbg = s_pagecfg.acc_dbg? FALSE: TRUE;
  //mp_debug(s_pagecfg.acc_dbg);

  ui_normal_pedometer_key_press_long();
  return APP_SUCCESS;
}

static int uipg_pedometer_timer(const ui_ev_t* pev)
{
  ui_page_blank();
  return APP_SUCCESS;
}

const ui_page_t s_page_pedometer[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_pedometer_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_pedometer_destruct},
  {UI_EV_KEY_PRESS,   uipg_pedometer_key_press},
  {UI_EV_KEY_PRESS_LONG,  uipg_pedometer_key_press_long},
  {UI_EV_TIMER,     uipg_pedometer_timer},
  {UI_EV_ACCELERATOR, uipg_pedometer_acc_data},
  {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_CALL,    uipg_call_in},
    {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_SMS,     uipg_push_msg},
    {UI_EV_BLE_WECHAT,    uipg_push_msg},
    {UI_EV_BLE_QQ,      uipg_push_msg},
    {UI_EV_BLE_MSG_NOTIFY,  uipg_push_msg},
  {0,0}
};


static int uipg_heartrate_construct(const ui_ev_t* pev)
{
  em70xx_start();
  
  s_pagecfg.subst = 0;
  ui_normal_hr_blink(s_pagecfg.subst);
  s_pagecfg.subst++;
  
  ui_timer_start(500);
  
  return APP_SUCCESS;
}

static int uipg_heartrate_destruct(const ui_ev_t* pev)
{
  uipg_common_destruct(pev);
  em70xx_stop();
  return 0;
  
}
static int uipg_heartrate_key_press(const ui_ev_t* pev)
{
  return ui_page_next();
}

static int uipg_heartrate_timer(const ui_ev_t* pev)
{
  LOG("uipg_hr_timer %d\n", s_pagecfg.subst);

  //blink for measurement waiting
  if(s_pagecfg.subst < 80){
    //blink the heart rate icon
    ui_timer_start(400);
    ui_normal_hr_blink(s_pagecfg.subst);
    s_pagecfg.subst++;
  }else if(s_pagecfg.subst == 80){
    //subst should be 0xff
    em70xx_stop();
    ui_timer_stop();
    ui_normal_hr_failure();
    ui_timer_start(2000);
    s_pagecfg.subst=0xff;
  }
  else if(s_pagecfg.subst == 0xff){
    return ui_page_blank();
  }
  return APP_SUCCESS;
  
}

static int uipg_heartrate_hr_value(const ui_ev_t* pev)
{
  LOG("uipg_heartrate_hr_value %d\n", s_pagecfg.subst);


  if(s_pagecfg.subst!=0xff){
    ui_timer_stop();
    ui_timer_start(20000);
  }
  //subst should be 0xff
  ui_normal_hr_heartrate((uint8_t)pev->param);
  s_pagecfg.subst=0xff;
  
  return APP_SUCCESS;
  
}

static int uipg_heartrate_hr_failed(const ui_ev_t* pev)
{
  LOG("uipg_heartrate_hr_value %d\n", s_pagecfg.subst);
  em70xx_stop();

  if(s_pagecfg.subst!=0xff){
    ui_timer_stop();
    ui_timer_start(4000);
  }
  ui_normal_hr_failure();
  s_pagecfg.subst=0xff;

  return APP_SUCCESS;
  
}

const ui_page_t s_page_heart_rate[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_heartrate_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_heartrate_destruct},
  {UI_EV_KEY_PRESS,   uipg_heartrate_key_press},
  {UI_EV_HEARTRATE,   uipg_heartrate_hr_value},
  {UI_EV_HEARTRATE_FAILED,  uipg_heartrate_hr_failed},
  {UI_EV_TIMER,     uipg_heartrate_timer},
  {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_CALL,    uipg_call_in},
    {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_SMS,     uipg_push_msg},
    {UI_EV_BLE_WECHAT,    uipg_push_msg},
    {UI_EV_BLE_QQ,      uipg_push_msg},
    {UI_EV_BLE_MSG_NOTIFY,  uipg_push_msg},
  {0,0}
};


static int uipg_humiture_construct(const ui_ev_t* pev)
{
  //hr_start(0, 1);
  s_pagecfg.subst = 1;
  ui_timer_start(200);
  return APP_SUCCESS;
}

static int uipg_humiture_key_press(const ui_ev_t* pev)
{
  //hr_stop();
  return ui_page_next();
}

static int uipg_humiture_timer(const ui_ev_t* pev)
{
  LOG("uipg_hr_timer %d\n", s_pagecfg.subst);

  if(s_pagecfg.subst < 5){
    ui_timer_stop();
    ui_normal_humiture_icon(s_pagecfg.subst);
    ui_timer_start(50);
    s_pagecfg.subst++;
  }
  else{
    //hr_stop();
    return ui_page_blank();
  }
  return APP_SUCCESS;
  
}

static int uipg_humiture_value(const ui_ev_t* pev)
{
  LOG("uipg_heartrate_hr_value %d\n", s_pagecfg.subst);


  if(s_pagecfg.subst!=0xff){
    ui_timer_stop();
    ui_timer_start(4000);
  }
  //subst should be 0xff
  ui_normal_hr_heartrate((uint8_t)pev->param);
  s_pagecfg.subst=0xff;
  
  return APP_SUCCESS;
  
}



const ui_page_t s_page_humiture[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_humiture_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_humiture_key_press},
  {UI_EV_HUMITURE,    uipg_humiture_value},
  {UI_EV_TIMER,     uipg_humiture_timer},
  {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_CALL,    uipg_call_in},
    {UI_EV_BATT_CHARGE,   uipg_batt_charge},
    {UI_EV_BLE_SMS,     uipg_push_msg},
    {UI_EV_BLE_WECHAT,    uipg_push_msg},
    {UI_EV_BLE_QQ,      uipg_push_msg},
    {UI_EV_BLE_MSG_NOTIFY,  uipg_push_msg},
  {0,0}
};

static int uipg_call_construct(const ui_ev_t* pev)
{
  LOG("uipg_call_construct %d\n", s_pagecfg.subst);
  memset(s_pagecfg.ancs_attr, 0, 64);
  lcd_bus_init();
  disp_on();
  ui_normal_call_icon();
  s_pagecfg.subst = 0xff;
  ui_timer_start(20000);
  return APP_SUCCESS;
}

static int uipg_call_key_press(const ui_ev_t* pev)
{
  ui_page_next();
  return 0;
}

static int uipg_call_info(const ui_ev_t* pev)
{
  uint8_t len = strlen((const char*)pev->data);
  len = (len > 32)? 32 : len;
  memset(s_pagecfg.ancs_attr, 0, 64);
  memcpy(s_pagecfg.ancs_attr, pev->data, len);

  
  ui_timer_stop();
  ui_normal_call_info(0, s_pagecfg.ancs_attr);
  s_pagecfg.subst = 1;
  ui_timer_start(200);
  
  return 0;
}

static int uipg_call_off(const ui_ev_t* pev)
{
  ui_page_blank();
  return 0;
}

static int uipg_call_timer(const ui_ev_t* pev)
{
  LOG("uipg_call_timer %d\n", s_pagecfg.subst);
  if(s_pagecfg.subst < 15){
    ui_normal_call_info(s_pagecfg.subst, s_pagecfg.ancs_attr);
    s_pagecfg.subst++;
  }
  else if(s_pagecfg.subst == 15){
    ui_timer_stop();
    s_pagecfg.subst = 0xff;
    ui_timer_start(20000);
  }else if(s_pagecfg.subst == 0xff){
    ui_page_blank();
  }
  return APP_SUCCESS;
}

const ui_page_t s_page_pop_call[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_call_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_call_key_press},
  {UI_EV_BLE_CALL_OFF,  uipg_call_off},
  {UI_EV_BLE_CALL_INFO, uipg_call_info},
  {UI_EV_TIMER,     uipg_call_timer},
  {0,0}
};


static int uipg_msg_construct(const ui_ev_t* pev)
{
  uint8_t len = strlen((const char*)pev->data);
  len = (len > 32)? 32 : len;
  LOG("uipg_msg_construct\n");

  s_pagecfg.msg_ev = pev->param;
  s_pagecfg.msg_title = (char*)(pev->data);
  s_pagecfg.msg_info = (char*)(pev->data + len + 1);
  
  ui_timer_stop();
  ui_normal_msg_title(0, s_pagecfg.msg_ev, s_pagecfg.msg_title);
  s_pagecfg.subst = 1;
  ui_timer_start(600);
  
  return 0;
}


static bool uipg_get_msg(uint16_t scn_width, uint16_t* u_str, bool* pend)
{
  uint16_t offset, cnt;
  uint16_t unic_ch;
  char* utf8_str = s_pagecfg.msg_info;
  uint16_t w = 0;
  *pend = FALSE;
  cnt = 0;

  if(s_pagecfg.msg_info[0] == 0)
    return 0;
  
  while(1){
    offset = utf8_to_unicode(utf8_str, &unic_ch);
    if(offset == 0){
      *pend = TRUE;
      break;
    }
    u_str[cnt] = unic_ch;
    utf8_str += offset;
    cnt ++;
    w += (unic_ch > 127)? 16 : 9;
    if(scn_width < w + 9)
      break;
  }
  s_pagecfg.msg_info = utf8_str;

  if(cnt > 0)
    return TRUE;
  return FALSE;
  
}


static int uipg_msg_key_press(const ui_ev_t* pev)
{
  bool end,ret;
  int lines = 0;
  uint16_t u_str1[12];
  uint16_t u_str2[12];
  uint16_t u_str3[12];
  uint16_t u_str4[12];
  uint16_t* u_str_table[4] = {NULL,NULL,NULL,NULL};
  LOG("uipg_msg_key_press\n");

  ui_timer_stop();

  memset(u_str1, 0, 12*2);
  memset(u_str2, 0, 12*2);
  memset(u_str3, 0, 12*2);
  memset(u_str4, 0, 12*2);
  while(1){
    if(CFG_DISP==DISP_OLED)
      ret = uipg_get_msg(60, u_str1, &end);
    else
      ret = uipg_get_msg(88, u_str1, &end);
    if(ret == FALSE)
      break;
    lines++;
    u_str_table[0] = u_str1;
    
    if(end == TRUE)
      break;
    ret = uipg_get_msg(60, u_str2, &end);
    if(ret == FALSE)
      break;
    lines++;
    u_str_table[1] = u_str2;
    
    if(end == TRUE)
      break;
    ret = uipg_get_msg(60, u_str3, &end);
    if(ret == FALSE)
      break;
    lines++;
    u_str_table[2] = u_str3;
    
    if(end == TRUE)
      break;
    ret = uipg_get_msg(60, u_str4, &end);
    if(ret == FALSE)
      break;
    lines++;
    u_str_table[3] = u_str4;
    break;
  }

  if(lines == 0)
    return ui_page_next();

  ui_normal_msg_info(s_pagecfg.msg_ev, u_str_table, end);
  return 0;
}

static int uipg_msg_timer(const ui_ev_t* pev)
{
  LOG("uipg_msg_timer %d\n", s_pagecfg.subst);
  if(s_pagecfg.subst < 10){
    ui_normal_msg_title(s_pagecfg.subst, s_pagecfg.msg_ev, s_pagecfg.msg_title);
    if(s_pagecfg.subst%2)
      ui_timer_start(600);
    else
      ui_timer_start(300);
    s_pagecfg.subst++;
  }
  else if(s_pagecfg.subst == 10){
    //ui_timer_stop();
    s_pagecfg.subst = 0xff;
    ui_timer_start(30*1000);
  }else if(s_pagecfg.subst == 0xff){
    ui_page_blank();
  }
  return APP_SUCCESS;
}

const ui_page_t s_page_pop_msg[] = {
  {UI_EV_BLE_SMS,     uipg_push_msg},
  {UI_EV_BLE_WECHAT,    uipg_push_msg},
  {UI_EV_BLE_QQ,      uipg_push_msg},
  {UI_EV_BLE_MSG_NOTIFY,  uipg_push_msg},
  {UI_EV_PAGE_CONSTRUCT,  uipg_msg_construct},
  {UI_EV_BLE_CALL,    uipg_call_in},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_msg_key_press},
  {UI_EV_TIMER,     uipg_msg_timer},
  {0,0}
};



static int uipg_sysinfo_construct(const ui_ev_t* pev)
{
  return 0;
}


static int uipg_sysinfo_key_press(const ui_ev_t* pev)
{
  return 0;
}

static int uipg_sysinfo_timer(const ui_ev_t* pev)
{
  return 0;
}

const ui_page_t s_page_pop_sysinfo[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_sysinfo_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_sysinfo_key_press},
  {UI_EV_TIMER,     uipg_sysinfo_timer},
  {0,0}
};



static int uipg_batt_construct(const ui_ev_t* pev)
{
  LOG("uipg_batt_construct %d\n", s_pagecfg.subst);
  
  lcd_bus_init();
  disp_on();
  s_pagecfg.subst = 0;
  s_pagecfg.value = 0;//batt_percent();
  ui_timer_start(400);
  return APP_SUCCESS;
}


static int uipg_batt_key_press(const ui_ev_t* pev)
{
  return ui_page_next();
}

static int uipg_batt_timer(const ui_ev_t* pev)
{
  LOG("uipg_batt_timer %d\n", s_pagecfg.subst);

  if(s_pagecfg.subst < 25){

    if(s_pagecfg.value >= 100){
      s_pagecfg.value = 0;//batt_percent();
    }
    else
    {
      s_pagecfg.value += 10;
      if(s_pagecfg.value>100)
        s_pagecfg.value = 100;
    }
    ui_normal_batt_reflash(s_pagecfg.subst, s_pagecfg.value);
    s_pagecfg.subst++;
    ui_timer_start(400);
  }
  else{
    return ui_page_blank();
  }
  return APP_SUCCESS;
}


const ui_page_t s_page_pop_batt[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_batt_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_batt_key_press},
  {UI_EV_TIMER,     uipg_batt_timer},
  {0,0}
};


static int uipg_alarm_clk_construct(const ui_ev_t* pev)
{
  LOG("uipg_alarm_clk_construct %d\n", s_pagecfg.subst);
  ui_normal_alarm_clk_icon();
  //vibra_on(3000);
  s_pagecfg.subst = 0xff;
  ui_timer_start(3000);
  return APP_SUCCESS;
}

static int uipg_alarm_clk_key_press(const ui_ev_t* pev)
{
  //vibra_off();
  ui_page_next();
  return 0;
}


static int uipg_alarm_clk_timer(const ui_ev_t* pev)
{
  LOG("uipg_alarm_clk_timer %d\n", s_pagecfg.subst);
  ui_page_blank();
  return APP_SUCCESS;
}

const ui_page_t s_page_pop_alarm_clk[] = {
  {UI_EV_PAGE_CONSTRUCT,  uipg_alarm_clk_construct},
  {UI_EV_PAGE_DESTRUCT, uipg_common_destruct},
  {UI_EV_KEY_PRESS,   uipg_alarm_clk_key_press},
  {UI_EV_TIMER,     uipg_alarm_clk_timer},
  {0,0}
};


static const ui_page_t* s_list_normal[PAGE_NUM_NORMAL] = {
  s_page_home,
  s_page_pedometer,
  s_page_heart_rate,
  s_page_humiture
};


static const ui_page_t* s_list_pop[] = {
  s_page_pop_call,
  s_page_pop_msg,
  s_page_pop_batt,
};

static int ui_page_next(void)
{
  ui_ev_t ev_construct = {
          .ev = UI_EV_PAGE_CONSTRUCT,
          .param = 0,
          .data = 0,
        };

  
  ui_fsm_destruct();

  switch(s_pagecfg.state){
  case UI_ST_BLANK:
    s_pagecfg.state = UI_ST_NORMAL;
    s_pagecfg.index = 0;
    break;
  case UI_ST_NORMAL:
    while(1){
      s_pagecfg.index = (s_pagecfg.index + 1) % 4;
      if(s_pagecfg.page_mask[s_pagecfg.index])
        break;
    }
    break;

  case UI_ST_POP:
    s_pagecfg.state = UI_ST_NORMAL;
    s_pagecfg.index = 0;
    break;
  default:
    //error protect
    s_pagecfg.state = UI_ST_BLANK;
    s_pagecfg.index = 0;
    
  }
  
  return ui_fsm_run(&ev_construct);
  
}

//page to blank
static int ui_page_blank(void)
{
  ui_ev_t ev = {
          .ev = UI_EV_PAGE_CONSTRUCT,
          .param = 0,
          .data = 0,
        };

  ui_fsm_destruct();

  s_pagecfg.state = UI_ST_BLANK;
  s_pagecfg.index = 0;
  ui_timer_stop();
  return ui_fsm_run(&ev);
}


int ui_fsm_dispatch_event(const ui_page_t* ppage, const ui_ev_t* pev){
  int i =0;
  if(ppage == NULL)
    return APP_ERR_INVALID_PAGE;
  for(i = 0; ; i++){
    if(ppage[i].ev_type == 0)
      break;
    if(ppage[i].ev_type == pev->ev)
      return ppage[i].ev_handler(pev);
  }
  return 0;
}


void ui_page_timer_evt(void* pctx)
{
  ui_ev_t ev = {
          .ev = UI_EV_TIMER,
          .param = 0,
          .data = 0,
        };

  LOG("ui_page_timer_evt\n");
  ui_fsm_run(&ev);
  
}
void ui_accelerator_event(void* pacc)
{
  ui_ev_t ev = {
      .ev = UI_EV_ACCELERATOR,
      .param = 0,
      .data = pacc,
  };
  ui_fsm_run(&ev);

}

void ui_batt_event(batt_evt_t evt)
{
  ui_ev_t uiev;
  switch(evt){
  case BATT_CHARGE_PLUG:
    uiev.ev = UI_EV_BATT_CHARGE;
    ui_fsm_run(&uiev);
    break;
  case BATT_CHARGE_UNPLUG:
    uiev.ev = UI_EV_BATT_CHARGE_OFF;
    ui_fsm_run(&uiev);
    break;
  case BATT_VOLTAGE:
    uiev.ev = UI_EV_BATT_VOLTAGE;
    uiev.param = (uint16_t)(batt_voltage()*1000);
    ui_fsm_run(&uiev);
    break;
  default:
    break;

  }
}

void ui_key_evt(void* pctx)
{
  ui_ev_t ev = {
      .ev = UI_EV_KEY_PRESS,
      .param = 0,
      .data = NULL
  };
  ui_fsm_run(&ev);
  LOG("Key\n");
}

int ui_page_config(void)
{
  s_pagecfg.page_mask[0] = TRUE;
  for(int i = 0; i< PAGE_NUM_NORMAL; i++){
    s_pagecfg.page_mask[i] = TRUE;
  }
  return APP_SUCCESS;
}

void ui_page_init(void)
{
  s_pagecfg.state = UI_ST_BLANK;
  s_pagecfg.index = 0;
  s_pagecfg.style = UI_STYLE_SIMPLE;

  ui_page_config();


}


int ui_fsm_run(ui_ev_t* pev)
{
  int ret = APP_ERR_INVALID_PAGE;
  const ui_page_t* ppage = NULL;
  //LOG("ui_fsm_run: ev[0x%x], state[%d]\n", pev->ev, s_pagecfg.state);
  switch(s_pagecfg.state){
  case UI_ST_BLANK:
    ret = ui_fsm_dispatch_event(s_page_blank, pev);
    break;
  case UI_ST_NORMAL:
    {
      if(s_pagecfg.index > sizeof(s_list_normal)/sizeof(ui_page_t))
        break;
      ppage = s_list_normal[s_pagecfg.index];
      ret = ui_fsm_dispatch_event(ppage, pev);
      break;
    }
  case UI_ST_POP:
    {
      //if(s_pagecfg.index >= sizeof(s_list_pop)/sizeof(ui_page_t))
      //  break;
      ppage = s_list_pop[s_pagecfg.index];
      ret = ui_fsm_dispatch_event(ppage, pev);
      break;
    }
  default:
    //error protect
    s_pagecfg.state = UI_ST_BLANK;
    s_pagecfg.index = 0;
    ret = ui_fsm_dispatch_event(s_page_blank, pev);
    break;
  }
  return ret;
}

