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


#include <string.h>
#include "bcomdef.h"
#include "common.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "peripheral.h"
#include "gattservapp.h"

#include "cam_rx.h"
#include "cam_service.h"
#include "error.h"
#include "log.h"


CONST uint8 cam_ServiceUUID[ATT_UUID_SIZE] =
{ 0xdd, 0x4f, 0x7c, 0x8b, 0x73, 0x49, 0x4d, 0xb9, 0xe5, 0x0f, 0x46, 0x7b, 0x01, 0xff, 0x04, 0x86};

//command characteristic
CONST uint8 cam_CommandUUID[ATT_UUID_SIZE] =
{ 0xdd, 0x4f, 0x7c, 0x8b, 0x73, 0x49, 0x4d, 0xb9, 0xe5, 0x0f, 0x46, 0x7b, 0x02, 0xff, 0x04, 0x86};

// Sensor location characteristic
CONST uint8 cam_ControlUUID[ATT_UUID_SIZE] =
{ 0xdd, 0x4f, 0x7c, 0x8b, 0x73, 0x49, 0x4d, 0xb9, 0xe5, 0x0f, 0x46, 0x7b, 0x03, 0xff, 0x04, 0x86};

// Command characteristic
CONST uint8 cam_DataUUID[ATT_UUID_SIZE] =
{ 0xdd, 0x4f, 0x7c, 0x8b, 0x73, 0x49, 0x4d, 0xb9, 0xe5, 0x0f, 0x46, 0x7b, 0x04, 0xff, 0x04, 0x86};


static CONST gattAttrType_t cam_Service = { ATT_UUID_SIZE, cam_ServiceUUID };

static uint8 cam_CommandProps = GATT_PROP_WRITE_NO_RSP;
static uint8 cam_CommandValue = 0;

// CAM response Characteristic
static uint8 cam_ControlProps = GATT_PROP_NOTIFY;
static uint8 cam_ControlValue = 0;
static gattCharCfg_t cam_ControlCCCD[GATT_MAX_NUM_CONN];

// CAM Data Characteristic
static uint8 cam_DataProps = GATT_PROP_NOTIFY;
static uint8 cam_DataValue = 0;
static gattCharCfg_t cam_DataCCCD[GATT_MAX_NUM_CONN];

#define CAM_COMMAND_HANDLE 2
#define CAM_CTRL_HANDLE 4
#define CAM_DATA_HANDLE 7
#define CAM_CTRL_CCCD_HANDLE 5
#define CAM_DATA_CCCD_HANDLE 8

static gattAttribute_t cam_AttrTbl[] = 
{
  //CAM Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&cam_Service                     /* pValue */
  },

      //CAM Command Declaration
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &cam_CommandProps 
      },
    
            //CAM Command Value
            { 
              { ATT_UUID_SIZE, cam_CommandUUID },
              GATT_PERMIT_WRITE, 
              0, 
              &cam_CommandValue
            },
        

      // CAM response Declaration
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &cam_ControlProps
      },

            //response Value
            { 
              { ATT_UUID_SIZE, cam_ControlUUID },
              GATT_PERMIT_READ, 
              0, 
              &cam_ControlValue
            },
            
            // CAM response Client Characteristic Configuration
            { 
              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
              GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
              0, 
              (uint8 *) cam_ControlCCCD
            },      

      //Data Declaration
      { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &cam_DataProps 
      },

            // data Value
            { 
              { ATT_UUID_SIZE, cam_DataUUID },
              GATT_PERMIT_WRITE, 
              0, 
              &cam_DataValue 
            },
            // CAM data Client Characteristic Configuration
            { 
              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
              GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
              0, 
              (uint8 *) cam_DataCCCD
            },      
};



static void cam_handleConnStatusCB( uint16 connHandle, uint8 changeType );

static uint8 cam_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t cam_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

CONST gattServiceCBs_t cam_ProfileCBs =
{
  cam_ReadAttrCB,  // Read callback function pointer
  cam_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};


uint16_t s_mtu_size = 20;

static uint8_t s_notif_buf[BLECAM_MAX_MTU];

typedef struct{
  uint8_t         state;
  uint8_t         taskID;
  uint16_t        osal_tm_evt;
  //uint16_t        osal_tm_gu_evt;
  uint16_t        cccd_ctrl;
  uint16_t        cccd_data;
  
  uint16_t        index;
  camrx_buf_t     framebuf;

  //uint16_t        miss_num;
  //uint16_t        miss_curr;  //current miss index
  //uint16_t        miss[FRAME_SIZE_MAX/19];
  
  camsv_cb_t      cb;
}camsv_ctx_t;


static camsv_ctx_t s_camsv_ctx = {
    .state = 0,
    .taskID = 0,
    .osal_tm_evt = 0,
    .cccd_ctrl = 0,
    .cccd_data = 0,
    .index = 0,
    .framebuf = {0},
    .cb = NULL};


enum{
  CAMSV_ST_UNINIT = 0,
  CAMSV_ST_DISCONNECTED,
  CAMSV_ST_CONNECTED,
  CAMSV_ST_DATA,
};

static void print_hex (const uint8 *data, uint16 len)
{
  //return;
#if(DEBUG_INFO > 1)
  uint16 i;
  char strdata[5];
  for (i = 0; i < len - 1; i++)
  {
    //if(i %16 ==  0 && i >0)
    //  LOG("\n");
    sprintf(strdata, "%.2x", data[i]);
    AT_LOG("%s ",strdata);
  }
  sprintf(strdata, "%.2x", data[i]);
  AT_LOG("%s\n",strdata);
#endif
}


static uint8 cam_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = ATT_ERR_READ_NOT_PERMITTED;
  LOG("ReadAttrCB\n");
  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  return ( status );
}





static void start_timer(uint32_t timeout)
{
  osal_start_timerEx(s_camsv_ctx.taskID, s_camsv_ctx.osal_tm_evt, (uint32)timeout);
}
static void stop_timer(void)
{
  osal_stop_timerEx(s_camsv_ctx.taskID, s_camsv_ctx.osal_tm_evt);
}


static void update_cccd(void)
{
  camrx_buf_t* pfbuf = &(s_camsv_ctx.framebuf);
  stop_timer();
  pfbuf->size = 0;
  pfbuf->used = 0;
  if(s_camsv_ctx.cccd_ctrl == 1 && s_camsv_ctx.cccd_data == 1){
    s_camsv_ctx.state = CAMSV_ST_CONNECTED;
    s_camsv_ctx.index = 0;
  }
  else
  {
    s_camsv_ctx.state = CAMSV_ST_DISCONNECTED;
    s_camsv_ctx.index = 0;
  }
}

static int start_send_frame_data(void)
{
  camrx_buf_t* pfbuf = &(s_camsv_ctx.framebuf);
  s_camsv_ctx.state = CAMSV_ST_DATA;
  s_camsv_ctx.index = 0;

  pfbuf->size = 0;
  pfbuf->used = 0;

  

  start_timer(1);
  //cam_ble_send_data();
  return PPlus_SUCCESS;
}

static int stop_send_frame_data(void)
{
  camrx_buf_t* pfbuf = &(s_camsv_ctx.framebuf);
  s_camsv_ctx.state = CAMSV_ST_CONNECTED;
  s_camsv_ctx.index = 0;

  pfbuf->size = 0;
  pfbuf->used = 0;

  stop_timer();
  //cam_ble_send_data();
  return PPlus_SUCCESS;
}


static bStatus_t cam_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    if(uuid == GATT_CLIENT_CHAR_CFG_UUID  && pAttr->handle == cam_AttrTbl[CAM_CTRL_CCCD_HANDLE].handle)
    {
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
      if ( status == SUCCESS)
      {
        uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
      
        LOG("CCCD control: [%d]\n", charCfg);
        s_camsv_ctx.cccd_ctrl = charCfg;
        update_cccd();
      }
    }
    if(uuid == GATT_CLIENT_CHAR_CFG_UUID  && pAttr->handle == cam_AttrTbl[CAM_DATA_CCCD_HANDLE].handle)
    {
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
      if ( status == SUCCESS)
      {
        uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
        LOG("CCCD data: [%d]\n", charCfg);
        s_camsv_ctx.cccd_data = charCfg;
        update_cccd();
      }
    }

  }
  else if( pAttr->type.len == ATT_UUID_SIZE && pAttr->handle == cam_AttrTbl[CAM_COMMAND_HANDLE].handle )
  {
    switch( s_camsv_ctx.state ){
    case CAMSV_ST_DISCONNECTED:
      s_camsv_ctx.state = CAMSV_ST_DISCONNECTED;
      s_camsv_ctx.cccd_ctrl = 0;
      s_camsv_ctx.cccd_data = 0;
      s_camsv_ctx.index = 0;
      stop_timer();
      break;
    case CAMSV_ST_CONNECTED:
    case CAMSV_ST_DATA:
      if(pValue[0] == CAMSV_CMD_START_SEND)
        start_send_frame_data();
      else if(pValue[0] == CAMSV_CMD_STOP_SEND)
        stop_send_frame_data();
      break;
    }
  }
  return ( status );
}


bStatus_t cam_notify_ctrl(const uint8_t* data, uint16_t len)
{
  uint16 connHandle;
  uint16 value;
  attHandleValueNoti_t Noti;

  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
  value = GATTServApp_ReadCharCfg( connHandle, cam_ControlCCCD);
  
  if(connHandle == INVALID_CONNHANDLE)
    return bleIncorrectMode;
  
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    LOG("Notif\n");
    // Set the handle
    Noti.handle = cam_AttrTbl[CAM_CTRL_HANDLE].handle;
    Noti.len = len;
    memcpy(Noti.value, data, len);
  
    // Send the Indication
    return GATT_Notification( connHandle, &Noti, FALSE);
   
  }
  return bleIncorrectMode;
  
}


bStatus_t cam_notify_data(const uint8_t* data, uint16_t len)
{
  uint16 connHandle;
  uint16 value;
  attHandleValueNoti_t Noti;

  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
  value = GATTServApp_ReadCharCfg( connHandle, cam_DataCCCD);
  
  if(connHandle == INVALID_CONNHANDLE)
    return bleIncorrectMode;

  
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    bStatus_t ret;
    // Set the handle
    Noti.handle = cam_AttrTbl[CAM_DATA_HANDLE].handle;
    Noti.len = len;
    memcpy(Noti.value, data, len);
  
    // Send the Indication
    print_hex(data, len);
    ret = GATT_Notification( connHandle, &Noti, FALSE);
    //AT_LOG("N %x %d\n", data[0], ret);
    return ret;
   
  }
  return bleIncorrectMode;
}


static void cam_handleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, cam_ControlCCCD);
      GATTServApp_InitCharCfg( connHandle, cam_DataCCCD);
      s_camsv_ctx.state = CAMSV_ST_DISCONNECTED;
      s_camsv_ctx.cccd_ctrl = 0;
      s_camsv_ctx.cccd_data = 0;
      s_camsv_ctx.index = 0;
      s_camsv_ctx.framebuf.size = 0;
    }
  }
}


static int cam_ble_send_data(void)
{
	bStatus_t bret;
	int ret;
	uint16_t offset = 0;

  camrx_buf_t* pfbuf = &(s_camsv_ctx.framebuf);
  switch(s_camsv_ctx.state)
  {
  case CAMSV_ST_DATA:
  {
    while(1){
      uint8_t size;
      if(pfbuf->size == 0){
        ret = camrx_get_frame(pfbuf);
        if(ret == false){
          //schedual timer to restart request frame;
          //LOG("s1\n");
          start_timer(1);
          break;
        }
        //LOG("s2\n");
        s_camsv_ctx.index = 0;
      }

      if(s_camsv_ctx.index == 0){
        s_notif_buf[0] = 0;
        s_notif_buf[1] = 0;
        s_notif_buf[2] = (uint8_t)((pfbuf->size)&0xff);
        s_notif_buf[3] = (uint8_t)(((pfbuf->size)>>8)&0xff);
        size = 2;
      }
      else
      {
        offset = (s_mtu_size - 2) * (s_camsv_ctx.index -1);
        size = (pfbuf->size - offset) > s_mtu_size - 2 ? s_mtu_size - 2 : (pfbuf->size - offset);
        s_notif_buf[0] = (uint8_t)((s_camsv_ctx.index)&0xff);
        s_notif_buf[1] = (uint8_t)(((s_camsv_ctx.index)>>8)&0xff);
        memcpy(s_notif_buf + 2, pfbuf->buf + offset, size);

      }

      bret = cam_notify_data(s_notif_buf, size + 2);
      
      if(bret == SUCCESS){
				//LOG("~");
        s_camsv_ctx.index ++;
      }
      else
      {
        if(bret == MSG_BUFFER_NOT_AVAIL){
          start_timer(1);
          break;
        }
        else
        {
          LOG("busy!\n");
          return PPlus_ERR_BUSY;
        }
      }
      if(pfbuf->size <= offset + size && pfbuf->size > 0){
        pfbuf->size = 0;
        LOG(">");
				stop_timer();
      }

    }
    break;
  }
  case CAMSV_ST_DISCONNECTED:
  case CAMSV_ST_CONNECTED:
  default:
    break;
  }
  return PPlus_SUCCESS;
}

static camrx_buf_t* s_uart_framebuf = NULL;
void uart_sendbuf(void){
  if(s_uart_framebuf){
    hal_uart_send_buff(s_uart_framebuf->buf, s_uart_framebuf->size);
		//WaitMs(1000);
    //framebuf_unlock(s_uart_framebuf);
    LOG("~~~%x\n", (uint32_t)(s_uart_framebuf->buf));
    s_uart_framebuf = NULL;
  }
}
//#define UARTCAM
void cam_ble_tmevt(void)
{
#ifdef UARTCAM
   //if(s_camsv_ctx.state == CAMSV_ST_DISCONNECTED)
   {
     uart_sendbuf();
		 return ;
   }
#endif
   cam_ble_send_data();
}

/*void camrx_evt_handle(camrx_evt_t* pev)
{
  bStatus_t bret = SUCCESS;
  camrx_buf_t* pfbuf = NULL;
  
#ifdef UARTCAM

  {
    if(s_uart_framebuf == NULL){
      s_uart_framebuf = (camrx_buf_t*)pev->data;
      framebuf_lock(s_uart_framebuf);
      LOG(">>>>%x, %d\n",(uint32_t)(s_uart_framebuf->buf), s_uart_framebuf->busy);
      osal_set_event(s_camsv_ctx.taskID, s_camsv_ctx.osal_tm_evt);
    }
    return;
  }
#endif
	//LOG("ST:%d\n",s_camsv_ctx.state);
  switch(s_camsv_ctx.state)
  {
  case CAMSV_ST_DATA:
  case CAMSV_ST_CONNECTED:
    if(pev->type == CAMRX_EVT_FRAMEDATA)
    {
      pfbuf = (camrx_buf_t*)pev->data;
      cam_set_MTU(ATT_GetCurrentMTUSize());
      s_camsv_ctx.pframebuf = pfbuf;
      uint16_t ctrl_data[3];
      ctrl_data[0] = CAMSV_CTRL_FRAME_SIZE;
			pfbuf->size = pev->size;
      ctrl_data[1] = pfbuf->size;
      ctrl_data[2] = s_mtu_size;
      framebuf_lock(pfbuf);
      s_camsv_ctx.state = CAMSV_ST_CONTROL;
      bret = cam_notify_ctrl((const uint8_t*)ctrl_data, 6);
      if(bret){
        framebuf_unlock(pfbuf);
        s_camsv_ctx.state = CAMSV_ST_CONNECTED;
      }
      start_guard_timer(10000);
    }
    break;
  default:
    break;
  }
}
*/

int cam_set_MTU(uint8_t mtu)
{
	LOG("MTU:%d\n", mtu);
  s_mtu_size = mtu - 3;
	return s_mtu_size;
}

int cam_addservice(camsv_cb_t cb, uint8_t taskID, uint16_t osal_tm_evt)
{
  uint8 status = SUCCESS;
  memset(&s_camsv_ctx, 0, sizeof(s_camsv_ctx));
  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( cam_handleConnStatusCB);
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cam_ControlCCCD );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, cam_DataCCCD );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( cam_AttrTbl, 
                                        GATT_NUM_ATTRS( cam_AttrTbl ),
                                        &cam_ProfileCBs );
  if(status!=SUCCESS)
    LOG("Add camera service failed!\n");

  s_camsv_ctx.taskID = taskID;
  s_camsv_ctx.osal_tm_evt = osal_tm_evt;
  s_camsv_ctx.state = CAMSV_ST_DISCONNECTED;
  s_camsv_ctx.cb = cb;


  camrx_init();
	
	return PPlus_SUCCESS;
  
}


