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
#include "phy_console.h"
#include "osal.h"
#include "uart.h"
#include "otam_cmd.h"
#include "stdlib.h"
#include "string.h"
#include "error.h"


static uint8_t s_app_task_id = 0;


static const cons_cmd_t s_cmd_list[] = {
	{OTAM_CMD_HELP, "help"},
	{OTAM_CMD_SCAN, "scan"},
	{OTAM_CMD_DISCONNECT, "disconn"},
	{OTAM_CMD_CONNECT, "conn"},
	{OTAM_CMD_OTA_MODE, "otamode"},
	{OTAM_CMD_OTA_START, "ota"},
	{OTAM_CMD_OTA_STOP, "otastop"},
	{OTAM_CMD_OTA_REBOOT, "reboot"},
	{0, NULL},
};


static void on_cmd_help(uint8_t argc, char** argv)
{
  uint8_t i;
  LOG("\"AT\" Command List:\n");
  for(i = 0; ; i++){
    if(s_cmd_list[i].cmd_id == 0)
      break;
    LOG("at%s\n",s_cmd_list[i].cmd_name);
  }
  LOG("\n\n");
}

static bool is_help(uint8_t argc, char** argv)
{
  if(argc == 1){
    if(strcmp(argv[0], "-h") == 0){
      LOG("USAGE:\n");
      return true;
    }
    if(strcmp(argv[0], "-help") == 0){
      LOG("USAGE:\n");
      return true;
    }
    if(strcmp(argv[0], "-?") == 0){
      LOG("USAGE:\n");
      return true;
    }
  }
  return false;
}

bool parse_mac(uint8_t* id, uint8_t *mac, char* param)
{
  char tmp[4];
  *id = 0xff;
  if(strlen(param) == 12){
    for(int i = 0; i< 6; i++){
      memcpy(tmp, param+i*2, 2);
      tmp[2] = 0;
      mac[5-i] = strtol(tmp, NULL, 16);
    }
    return true;
  }
  else if(strlen(param) <= 2 && strlen(param)>0){
    *id = atoi(param);
    return true;
  }
  return false;
}


static void on_cmd_scan(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 0){
    LOG("scan connectable device\n");
    LOG("CMD:[atscan]\n");
    return;
  }
  osal_event_hdr_t* pev = (osal_event_hdr_t*)osal_msg_allocate( (uint16)(sizeof (osal_event_hdr_t)) );
  if(pev){
    pev->event = OTAM_MSG_EVENT;
    pev->status = OTAM_CMD_SCAN;
    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;
  }
  LOG("scan command failed\n");
}


static void on_cmd_connect(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 1){
    LOG("connect device, parameter can be mac address, or device in scan list\n");
    LOG("CMD:[atconn MAC]\n");
    LOG("or\n");
    LOG("CMD:[atconn ID]\n");
    LOG("   ID: index in scan list\n");
    return;
  }
  uint8_t mac[6];
  uint8_t id = 0xff;
  otam_cmd_conn_t* pev;
  if(parse_mac(&id, mac, argv[0]) == false){
    LOG("conn command failed\n");
    LOG("MAC format or ID is incorrect!\n");
    return;
  }
  pev = (otam_cmd_conn_t*)osal_msg_allocate( (uint16)(sizeof (otam_cmd_conn_t)) );

  if(pev){
    pev->hdr.event = OTAM_MSG_EVENT;
    pev->hdr.status = OTAM_CMD_CONNECT;
    pev->id = id;
    memcpy(pev->mac, mac, 6);
    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;  
  }
  LOG("scan command failed\n");
}

static void on_cmd_disconnect(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 0){
    LOG("terminate connection\n");
    LOG("CMD:[atdisconn]\n");
    return;
  }
  osal_event_hdr_t* pev = (osal_event_hdr_t*)osal_msg_allocate( (uint16)(sizeof (osal_event_hdr_t)) );
  if(pev){
    pev->event = OTAM_MSG_EVENT;
    pev->status = OTAM_CMD_DISCONNECT;
    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;
  }
  LOG("disconnect failed\n");
}
static void on_cmd_ota_choose_mode(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 1){
    LOG("If device is in application mode, run this command to set OTA or OTA resource mode!\n");
    LOG("when this command issued, host will terminate connection\n");
    LOG("CMD:[atotamode mode]\n");
    LOG("   mode: 0 is application mode, 2 is OTA mode, 3 is OTA resource mode\n");
    return;
  }
  otam_cmd_mode_t* pev = (otam_cmd_mode_t*)osal_msg_allocate( (uint16)(sizeof (otam_cmd_mode_t)) );
  if(pev){
    pev->hdr.event = OTAM_MSG_EVENT;
    pev->hdr.status = OTAM_CMD_OTA_MODE;
    pev->mode = atoi(argv[0]);

    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;
  }
  LOG("mode command failed\n");
}


static void on_cmd_ota_start(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 0){
    LOG("if device is in ota mode or resource mode, run this command to start OTA procedure\n");
    LOG("CMD:[atota]\n");
    return;
  }
  osal_event_hdr_t* pev = (osal_event_hdr_t*)osal_msg_allocate( (uint16)(sizeof (osal_event_hdr_t)) );
  if(pev){
    pev->event = OTAM_MSG_EVENT;
    pev->status = OTAM_CMD_OTA_START;
    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;
  }
  LOG("OTA command failed\n");
}


static void on_cmd_ota_stop(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 0){
    LOG("\n");
    LOG("CMD:[atotastop]\n");
    LOG("   :\n");
    return;
  }

  osal_event_hdr_t* pev = (osal_event_hdr_t*)osal_msg_allocate( (uint16)(sizeof (osal_event_hdr_t)) );
  if(pev){
    pev->event = OTAM_MSG_EVENT;
    pev->status = OTAM_CMD_OTA_STOP;
    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;
  }
  LOG("OTA stop command failed\n");

}


static void on_cmd_ota_reboot(uint8_t argc, char** argv)
{
  if(is_help(argc, argv) || argc != 0){
    LOG("\n");
    LOG("CMD:[atreboot]\n");
    LOG("   :\n");
    return;
  }
  osal_event_hdr_t* pev = (osal_event_hdr_t*)osal_msg_allocate( (uint16)(sizeof (osal_event_hdr_t)) );
  if(pev){
    pev->event = OTAM_MSG_EVENT;
    pev->status = OTAM_CMD_OTA_REBOOT;
    osal_msg_send( s_app_task_id, (uint8 *)pev);
    return;
  }

}




void cons_callback(uint16_t cmd_id, uint8_t argc, char** argv)
{
	LOG("cmd id is 0x%x, parameter num is %d\n", cmd_id, argc);
  switch(cmd_id){
  case OTAM_CMD_HELP:
    on_cmd_help(argc, argv);
    break;
  case OTAM_CMD_SCAN:
    on_cmd_scan(argc, argv);
    break;
  case OTAM_CMD_CONNECT:
    on_cmd_connect(argc, argv);
    break;
  case OTAM_CMD_DISCONNECT:
    on_cmd_disconnect(argc, argv);
    break;
  case OTAM_CMD_OTA_MODE:
    on_cmd_ota_choose_mode(argc, argv);
    break;
	case OTAM_CMD_OTA_START:
    on_cmd_ota_start(argc, argv);
    break;
	case OTAM_CMD_OTA_STOP:
    on_cmd_ota_stop(argc, argv);
    break;
	case OTAM_CMD_OTA_REBOOT:
    on_cmd_ota_reboot(argc, argv);
    break;
  default:
    LOG("Unknow command!\n");
    break;
	}
}




int otam_cmd_init(uint8_t task_id)
{
	console_init(s_cmd_list, cons_callback);
	LOG("Console registed!\n");
  s_app_task_id = task_id;    
  return PPlus_SUCCESS;
}


