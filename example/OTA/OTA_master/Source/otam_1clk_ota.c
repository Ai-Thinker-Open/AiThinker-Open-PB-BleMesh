

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "ll_common.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "gap_internal.h"
#include "simpleGATTprofile_ota.h"
#include "ota_master.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib.h"
#include "otam_cmd.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ota_master.h"
#include "otam_protocol.h"
#include "otam_1clk_ota.h"
#include "ui_disp_substance.h"
#include "error.h"

enum{
  ONECLK_ST_APP_IDLE = 0,
  ONECLK_ST_APP_CONNECTING,
  ONECLK_ST_APP_INFO,
  ONECLK_ST_APP_MODE,
  ONECLK_ST_OTA_IDLE,
  ONECLK_ST_OTA_CONNECTING,
  ONECLK_ST_OTAING,
};


typedef struct{
  uint8_t state;
  uint16_t cnt;

  uint8_t retry;
  

}oneclk_ota_ctx_t;

static oneclk_ota_ctx_t s_1clk_ctx;


static const uint8_t s_mac_app[6] = {0x55, 0x44, 0x36, 0x36, 0x45, 0x45};
static const uint8_t s_mac_ota[6] = {0x56, 0x44, 0x36, 0x36, 0x45, 0x45};



static void show_error(int errorcode, uint8_t state)
{
	LOG("show_error: %d, %d",errorcode, state);
}

static void show_info(const char* infostr)
{
	LOG("show_info: %s",infostr);

}


static int otam_oneclick_app_conn(void){
  bool bret;
  char macstr[20];
  if(s_1clk_ctx.state != ONECLK_ST_APP_IDLE && s_1clk_ctx.state != ONECLK_ST_APP_CONNECTING)
    return PPlus_ERR_INVALID_STATE;
  bret = otaMaster_EstablishLink(ADDRTYPE_PUBLIC, (uint8_t*)s_mac_app);
  if(bret == FALSE)
    return PPlus_ERR_BLE_FAIL;
  sprintf(macstr, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", s_mac_app[5],s_mac_app[4],s_mac_app[3],s_mac_app[2],s_mac_app[1],s_mac_app[0]);
  ui_normal_start_ota(macstr);

  return PPlus_SUCCESS;
}

static int otam_oneclick_ota_conn(void){
  bStatus_t bret;
  char macstr[20];
  if(s_1clk_ctx.state != ONECLK_ST_OTA_CONNECTING && s_1clk_ctx.state != ONECLK_ST_APP_MODE)
    return PPlus_ERR_INVALID_STATE;
  bret = otaMaster_EstablishLink(ADDRTYPE_PUBLIC, (uint8_t*)s_mac_ota);
  LOG("otaMaster_EstablishLink %d\n", bret);
  //if(bret)
  //  return PPlus_ERR_BLE_FAIL;
  sprintf(macstr, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", s_mac_ota[5],s_mac_ota[4],s_mac_ota[3],s_mac_ota[2],s_mac_ota[1],s_mac_ota[0]);
  ui_normal_ota_running(macstr);
  return PPlus_SUCCESS;
}


void otam_oneclick_evt(oneclk_evt_t* pev)
{
  switch(pev->ev){
  case ONECLK_EVT_CONNECTED:
    if(s_1clk_ctx.state == ONECLK_ST_APP_CONNECTING){
      s_1clk_ctx.retry = 0;
      otamProtocol_app_start_ota(2);
      osal_start_timerEx(otaMasterTaskId, OTA_APP_DISCONN_DELAY_EVT, 200);
      s_1clk_ctx.state = ONECLK_ST_APP_MODE;
    }
    else if(s_1clk_ctx.state == ONECLK_ST_OTA_CONNECTING)
    {
      s_1clk_ctx.retry = 0;
      otamProtocol_start_ota((s_1clk_ctx.cnt) % 2);
      hal_gpio_write(P32, 0);

      s_1clk_ctx.state = ONECLK_ST_OTAING;
    }
    break;

    
  case ONECLK_EVT_TERMINATED:
    LOG("Terminated\n");
    hal_gpio_write(P32, 1);

    if(s_1clk_ctx.state == ONECLK_ST_APP_CONNECTING){
      s_1clk_ctx.retry++;
      if(s_1clk_ctx.retry >= 2){
        show_error(PPlus_ERR_BLE_FAIL, s_1clk_ctx.state);
        return;
      }
      otam_oneclick_app_conn();
      osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 5000);
      s_1clk_ctx.state = ONECLK_ST_APP_CONNECTING;
    }
    else if(s_1clk_ctx.state == ONECLK_ST_OTA_CONNECTING)
    {
      s_1clk_ctx.retry++;
      if(s_1clk_ctx.retry >= 2){
        show_error(PPlus_ERR_BLE_FAIL, s_1clk_ctx.state);
        return;
      }
      otam_oneclick_ota_conn();
      osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 5000);
      s_1clk_ctx.state = ONECLK_ST_OTA_CONNECTING;
    }
    else if(s_1clk_ctx.state == ONECLK_ST_APP_MODE)
    {
      s_1clk_ctx.retry = 0;
      otam_oneclick_ota_conn();
      osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 2000);
      s_1clk_ctx.state = ONECLK_ST_OTA_CONNECTING;
    }
    else
    {
      show_error(PPlus_ERR_BLE_FAIL, s_1clk_ctx.state);
      s_1clk_ctx.state = 0;
      s_1clk_ctx.retry = 0;
    }
    break;

  case ONECLK_EVT_TIMER:
    if(s_1clk_ctx.state == ONECLK_ST_APP_CONNECTING)
    {
      //case did not find the app device, try to connect ota device
      gapCancelLinkReq(otaMasterTaskId, GAP_CONNHANDLE_ALL);
      s_1clk_ctx.retry = 0;
      s_1clk_ctx.state = ONECLK_ST_OTA_IDLE;
      osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 200);
    }
    else if(s_1clk_ctx.state == ONECLK_ST_OTA_IDLE)
    {
      s_1clk_ctx.retry = 0;
      s_1clk_ctx.state = ONECLK_ST_OTA_CONNECTING;
      otam_oneclick_ota_conn();
      osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 2000);
    }
    else if(s_1clk_ctx.state == ONECLK_ST_OTA_CONNECTING)
    {
      //gapCancelLinkReq(otaMasterTaskId, GAP_CONNHANDLE_ALL);
      show_error(PPlus_ERR_BLE_FAIL, s_1clk_ctx.state);
      s_1clk_ctx.state = 0;
      s_1clk_ctx.retry = 0;
    }
    break;
  case ONECLK_EVT_OTA_FINISHED:
    show_info("OTA completed\n");
    s_1clk_ctx.state = 0;
    s_1clk_ctx.retry = 0;
    s_1clk_ctx.cnt++;
    break;
  }

}


void otam_oneclick_ota(void)
{
  int ret = otam_oneclick_app_conn();
  if(ret){
    show_error(ret, s_1clk_ctx.state);
  }
  else
  {
    osal_start_timerEx(otaMasterTaskId, ONECLK_OP_TIMEOUT_EVT, 2000);
    s_1clk_ctx.state = ONECLK_ST_APP_CONNECTING;
    show_info("App Mode connecting");
  }
}

