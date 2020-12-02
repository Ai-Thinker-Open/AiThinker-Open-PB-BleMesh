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



/*********************************************************************
 * INCLUDES
 */

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
#include "simpleGATTprofile_ota.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "ota_flash_mesh.h"
#include "rflib.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ota_mesh_master.h"
//#include "ota_app_service.h"
#include "otam_protocol.h"
#include "error.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_NUM                  30

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 5000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      10

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      300

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           400

// Default passcode
#define DEFAULT_PASSCODE                      0//19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_NO_PAIRING//GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  FALSE //TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           200

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE


// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};


#define OTAC_CHAR_MAX_LEN 5


#define DEFAULT_OTAM_MTU_SIZE 203

typedef enum{
  OTAM_FSM_UNINIT = 0,
  OTAM_FSM_IDLE,
  OTAM_FSM_SCANING,
  OTAM_FSM_CONNECTING,
  OTAM_FSM_CONNECTED,
}otam_fsm_t;

typedef enum{
  DISC_ST_INIT = 0,            //Idle
  DISC_ST_SVC,                 //Service discovery
  DISC_ST_CHAR,                //Characteristic discovery
  DISC_ST_COMPLETED,            //discovery completed 
  DISC_ST_FAILED = 0xff
} disc_st_t;


typedef struct
{
  uint8_t       addr_type;
  uint8_t       addr[B_ADDR_LEN];
  int8          rssi;
  uint8_t       flags;
  char          name[32];
}otac_scan_dev_t;


typedef struct
{
  uint16_t char_cmd_hdl;
  uint16_t char_rsp_hdl;
  uint16_t char_rsp_cccd_hdl;
  uint16_t char_data_hdl;
}otac_service_t;

typedef struct
{
  uint16_t        conn_hdl;
  disc_st_t       disc_state;
  uint8_t         run_mode; //application mode or ota mode or unknow mode
  uint16_t        mtu;
  uint16_t        attr_start_hdl;
  uint16_t        attr_end_hdl;

  uint16_t        op_handle;  //save the characteristic handle of the operation
  
  otac_service_t  ota_service;
}otac_dev_t;


// Task ID for internal task/event processing
uint8 otaMM_TaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "PHY OTA Server";



#define OTAM_FSM_SET(new_state) {m_otam_ctx.fsm_state = new_state;}

typedef struct{
  otam_fsm_t          fsm_state;

  otafmesh_dev_t      dev;
  
  //scan
  uint8_t             scan_dev_num;
  otac_scan_dev_t     scan_dev_list[DEFAULT_MAX_SCAN_NUM];

  //connected device
  uint8_t             mode_choose;
  otac_dev_t          otac_dev;
  
  
  
}otam_ctx_t;


static otam_ctx_t m_otam_ctx;


static void otaMM_RssiCB( uint16 connHandle, int8 rssi );
static void otaMM_EventCB( gapCentralRoleEvent_t *pEvent );
static void otaMM_PasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void otaMM_PairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void otac_dle_phy_mtu(void);


// GAP Role Callbacks
static const gapCentralRoleCB_t otaMM_RoleCB =
{
  otaMM_RssiCB,       // RSSI callback
  otaMM_EventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t otaMM_BondCB =
{
  otaMM_PasscodeCB,
  otaMM_PairStateCB
};

static const uint8_t s_mac_zero[B_ADDR_LEN] = {0,0,0,0,0,0};
//static const uint8 ota_ServiceUUID[ATT_UUID_SIZE] =
//{ 0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x01, 0xff, 0x33, 0x58};

//command characteristic
//static const uint8 ota_CommandUUID[ATT_UUID_SIZE] =
//{ 0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x02, 0xff, 0x33, 0x58};

// Sensor location characteristic
//static const uint8 ota_ResponseUUID[ATT_UUID_SIZE] =
//{ 0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x03, 0xff, 0x33, 0x58};

// Command characteristic
//static const uint8 ota_DataUUID[ATT_UUID_SIZE] =
//{ 0x23, 0xf1, 0x6e, 0x53, 0xa4, 0x22, 0x42, 0x61, 0x91, 0x51, 0x8b, 0x9b, 0x04, 0xff, 0x33, 0x58};

static const uint8 ota_ServiceUUID[ATT_UUID_SIZE] =
{ 0xde, 0x61, 0x49, 0x67, 0xea, 0x50, 0x57, 0x2c, 0xbb, 0xba, 0x52, 0x9b, 0x01, 0xff, 0x33, 0x58};

//command characteristic
static const uint8 ota_CommandUUID[ATT_UUID_SIZE] =
{ 0xde, 0x61, 0x49, 0x67, 0xea, 0x50, 0x57, 0x2c, 0xbb, 0xba, 0x52, 0x9b, 0x02, 0xff, 0x33, 0x58};

// Sensor location characteristic
static const uint8 ota_ResponseUUID[ATT_UUID_SIZE] =
{ 0xde, 0x61, 0x49, 0x67, 0xea, 0x50, 0x57, 0x2c, 0xbb, 0xba, 0x52, 0x9b, 0x03, 0xff, 0x33, 0x58};

// Command characteristic
static const uint8 ota_DataUUID[ATT_UUID_SIZE] =
{ 0xde, 0x61, 0x49, 0x67, 0xea, 0x50, 0x57, 0x2c, 0xbb, 0xba, 0x52, 0x9b, 0x04, 0xff, 0x33, 0x58};

static void print_hex (const uint8 *data, uint16 len)
{
		//return;
#if(DEBUG_INFO>1)
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



static void otac_analyze_advdata(otac_scan_dev_t* pdev,gapDeviceInfoEvent_t *pData)
{
  int8 DataLength;
  int8 New_ADStructIndex = 0;
  int8 AD_Length = 0;
  uint8_t AD_Type;
  
  DataLength = pData->dataLen;
  pdev->rssi = pData->rssi;
  while(DataLength)
  {
    New_ADStructIndex += AD_Length;
    // DATA FORMAT : Length + AD Type + AD Data
    AD_Length = pData->pEvtData[pData->dataLen - DataLength];
    AD_Type = pData->pEvtData[New_ADStructIndex+1];

    if(AD_Length<2 || AD_Length>0x1f)
    {
        LOG("[AD_TYPE] ERR %02x %02x\n",AD_Type,AD_Length);
        break;
        }
    switch(AD_Type)
    {
      case GAP_ADTYPE_FLAGS:
        pdev->flags = pData->pEvtData[New_ADStructIndex+2];
        break;
      case GAP_ADTYPE_LOCAL_NAME_SHORT:
      case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
        memset(pdev->name, 0, 32);
        memcpy(pdev->name,&(pData->pEvtData[New_ADStructIndex+2]), AD_Length-1);
        LOG("%s\n",pdev->name);
        break;
      default:
        break;
    }
    AD_Length++;
    DataLength -= AD_Length;
  }
}


static void otac_scan_update_devinfo( gapCentralRoleEvent_t *pEvent )
{
  otam_ctx_t* pctx = &m_otam_ctx;
  otac_scan_dev_t* p_scanlist = pctx->scan_dev_list;
  gapDeviceInfoEvent_t* p_devinfo = (gapDeviceInfoEvent_t*)pEvent;
  uint8_t i;
  
  if( pEvent->deviceInfo.opcode != GAP_DEVICE_INFO_EVENT )
    return;

  if(pctx->fsm_state != OTAM_FSM_SCANING)
    return;

  if (p_devinfo->eventType != GAP_ADRPT_ADV_IND && p_devinfo->eventType != GAP_ADRPT_ADV_DIRECT_IND
      && p_devinfo->eventType != GAP_ADRPT_SCAN_RSP)
    return;

  //check if the mac address is in list
  //if true, update device info
  //if false, add new device
  //if device number is runout, just drop!
  for(i=0; i< DEFAULT_MAX_SCAN_NUM;i++)
  {
    //case new device
    if(memcmp( p_scanlist[i].addr, s_mac_zero , B_ADDR_LEN ) == 0)
    {
      p_scanlist[i].addr_type = p_devinfo->addrType;
      memcpy(p_scanlist[i].addr, p_devinfo->addr, 6);
      break;
    }
    else if(memcmp(p_scanlist[i].addr , p_devinfo->addr, B_ADDR_LEN ) == 0)
    {
      break;
    }
  }
  if(i == DEFAULT_MAX_SCAN_NUM){
    return;
  }
  otac_analyze_advdata(&(p_scanlist[i]), &(pEvent->deviceInfo));
}

static int otac_cccd(bool enable)
{
  otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
  attWriteReq_t wreq;
  bStatus_t bret;

  if(pdev->conn_hdl != 0 || pdev->disc_state != DISC_ST_COMPLETED)
    return PPlus_ERR_INVALID_STATE;
  if(pdev->run_mode < OTAC_RUNMODE_APP)
    return PPlus_ERR_INVALID_STATE;
    
  memset(&wreq, 0, sizeof(attWriteReq_t));
  wreq.cmd = 0;
  wreq.sig = 0;
  wreq.handle = pdev->ota_service.char_rsp_cccd_hdl;
  wreq.len = 2;
  if(enable){
    wreq.value[0] = 1;
  }

  pdev->op_handle = wreq.handle;
  
  bret = GATT_WriteCharValue(pdev->conn_hdl, &wreq, otaMM_TaskId);
  if(bret!= SUCCESS)
    return PPlus_ERR_BLE_BUSY;
  return PPlus_SUCCESS;
  
}


bool otaMM_EstablishLink(uint8_t addr_type, uint8_t* addr)
{
  bStatus_t ret;
  char str_mac[20];
  sprintf(str_mac, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
  LOG("Start EstablishLink: %s\n", str_mac);
  ret = GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                  DEFAULT_LINK_WHITE_LIST, addr_type, addr);
  if(ret == SUCCESS)
    return true;

  return false;
}

static void otaMM_StartDiscoveryService( void )
{
  otam_ctx_t* pctx = &m_otam_ctx;
  otac_dev_t* pdev = &(pctx->otac_dev);
  LOG("otaMM_StartDiscoveryService\r\n");
  
  //osal_memset(&(pctx->otac_dev), 0, sizeof(otac_dev_t));
  
  pdev->disc_state = DISC_ST_SVC;
  
  // Discovery simple BLE service
  GATT_DiscAllPrimaryServices(pdev->conn_hdl ,otaMM_TaskId);
}



//static void otaMM_ProcessCMDMsg(osal_event_hdr_t * pMsg)
//{
//}



static void on_gatt_read_rsp(attReadRsp_t* prsp)
{

}

static void on_gatt_write_rsp(void)
{
  otam_ctx_t* pctx = &m_otam_ctx;
  otac_dev_t* pdev = &(pctx->otac_dev);
  if(pdev->disc_state == DISC_ST_COMPLETED && pdev->op_handle == pdev->ota_service.char_rsp_cccd_hdl){
    int ret;
    otap_evt_t otap_ev;
    otam_proto_conn_param_t conn_param;
    ota_fw_t ffw;
    otap_ev.ev = OTAP_EVT_CONNECTED;
    otap_ev.len = sizeof(conn_param);
    conn_param.mtu = pdev->mtu;
    conn_param.run_mode = pdev->run_mode;
    otap_ev.data = (void*)(&conn_param);
    LOG("CCCD enabled\n");
    otamProtocol_event(&otap_ev);

    ret = otafm_fw_load(&ffw);
    if(ret == PPlus_SUCCESS)
      ret = otamProtocol_start_ota(&ffw);

    if(ret != PPlus_SUCCESS){
      //error handler
    }
    
  }
}

static void on_gatt_notify(attHandleValueNoti_t* prsp)
{
  otam_ctx_t* pctx = &m_otam_ctx;
  otac_dev_t* pdev = &(pctx->otac_dev);
  if(pdev->disc_state == DISC_ST_COMPLETED && prsp->handle == pdev->ota_service.char_rsp_hdl){

    osal_stop_timerEx(otaMM_TaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
    osal_clear_event(otaMM_TaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
    
    otap_evt_t otap_ev;
    otap_ev.ev = OTAP_EVT_NOTIFY;
    otap_ev.len = (uint16_t)(prsp->len);
    otap_ev.data = (void*)(prsp->value);
    otamProtocol_event(&otap_ev);
  }
}

static void on_gatt_indicate(attHandleValueInd_t* prsp)
{
  //do nothing
}

static void on_gatt_discovery(gattMsgEvent_t* pMsg)
{
  otam_ctx_t* pctx = &m_otam_ctx;
  otac_dev_t* pdev = &(pctx->otac_dev);
  uint8_t i;
  switch(pdev->disc_state){
  case DISC_ST_INIT:
    //not started, just drop message
    break;
  case DISC_ST_SVC:
  {
    if((pMsg->method==ATT_READ_BY_GRP_TYPE_RSP)&&(pMsg->msg.readByGrpTypeRsp.numGrps>0))
    {
      uint16_t len, numgrp;
      uint8_t* pdata;

      len = (uint16_t)(pMsg->msg.readByGrpTypeRsp.len);
      numgrp = (uint16_t)(pMsg->msg.readByGrpTypeRsp.numGrps);
      pdata = pMsg->msg.readByGrpTypeRsp.dataList;
      LOG("ATT_READ_BY_GRP_TYPE_RSP: numGrps %d\n", numgrp);
      print_hex(pdata, len * numgrp);
      if(len == 20){
        for(i=0; i<numgrp; i++)
        {
          //find ota service
          if(memcmp(pdata + 4, ota_ServiceUUID, ATT_UUID_SIZE) == 0){
            pdev->attr_start_hdl = BUILD_UINT16( pdata[0], pdata[1]);
            pdev->attr_end_hdl = BUILD_UINT16( pdata[2], pdata[3]);
            break;
          }
        }
        pdata += len;
      }
    }
    else if((pMsg->method==ATT_READ_BY_GRP_TYPE_RSP) &&(pMsg->hdr.status == bleProcedureComplete))
    {
      bStatus_t bret = FAILURE;
      // Primary Service Discover OK , Prepare Discover Characteristic
      pdev->disc_state = DISC_ST_CHAR;
      memset(&(pdev->ota_service), 0, sizeof(otac_service_t));
      
      if(pdev->attr_start_hdl != 0 &&  pdev->attr_end_hdl != 0)
      {
        bret = GATT_DiscAllChars(pdev->conn_hdl,pdev->attr_start_hdl,pdev->attr_end_hdl,otaMM_TaskId);
      }
      if(bret != SUCCESS){
        LOG("Discovery failed, or can't find OTA service!\n");
        pdev->disc_state = DISC_ST_FAILED;
      }
    }

    break;
  }
  case DISC_ST_CHAR:
  {
    // Characteristic found, store handle
    if (pMsg->method==ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs>0)
    {
      uint16_t len, numpairs;
      uint8_t* pdata;
      len = (uint16_t)(pMsg->msg.readByTypeRsp.len);
      numpairs = (uint16_t)(pMsg->msg.readByTypeRsp.numPairs);
      pdata = pMsg->msg.readByTypeRsp.dataList;
      LOG("ATT_READ_BY_TYPE_RSP: numpairs %d\n", numpairs);
      print_hex(pdata, len * numpairs);
      for(i = 0; i < numpairs ; i++)
      {
        // Extract the starting handle, ending handle, and UUID of the current characteristic.
        // characteristic Handle
        if(memcmp(pdata + 5, ota_CommandUUID, ATT_UUID_SIZE) == 0)
        {
          pdev->ota_service.char_cmd_hdl = BUILD_UINT16( pdata[3],pdata[4]);
        }
        else if(memcmp(pdata + 5, ota_ResponseUUID, ATT_UUID_SIZE) == 0)
        {
          pdev->ota_service.char_rsp_hdl = BUILD_UINT16( pdata[3],pdata[4]);
          pdev->ota_service.char_rsp_cccd_hdl = pdev->ota_service.char_rsp_hdl + 1;
        }
        else if(memcmp(pdata + 5, ota_DataUUID, ATT_UUID_SIZE) == 0)
        {
          pdev->ota_service.char_data_hdl = BUILD_UINT16( pdata[3],pdata[4]);
        }
        pdata += len;
      }
    }
    else if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->hdr.status == bleProcedureComplete )
    {
      //discovery completed
      LOG("discovery completed");
      LOG("Command handle: 0x%x\n", pdev->ota_service.char_cmd_hdl);
      LOG("Response handle: 0x%x\n", pdev->ota_service.char_rsp_hdl);
      LOG("Rsp CCCD handle: 0x%x\n", pdev->ota_service.char_rsp_cccd_hdl);
      LOG("Data handle: 0x%x\n", pdev->ota_service.char_data_hdl);
      pdev->disc_state = DISC_ST_COMPLETED;
      if(pdev->ota_service.char_data_hdl == 0){
        pctx->mode_choose = 0;
        pdev->run_mode = OTAC_RUNMODE_APP;
      }
      else
      {
        pdev->run_mode = OTAC_RUNMODE_OTA;
      }

      otac_cccd(true);

    }
    break;
  }
  case DISC_ST_COMPLETED:
  default:
    break;

  }

}

static void on_gatt_err_rsp(attErrorRsp_t* prsp)
{
  switch(prsp->reqOpcode)
  {
  case ATT_READ_REQ:
  case ATT_WRITE_REQ:
  case ATT_HANDLE_VALUE_NOTI:
  case ATT_HANDLE_VALUE_IND:
    break;
  }
}

static void on_gatt_mtu_rsp(attExchangeMTURsp_t* prsp)
{
  otam_ctx_t* pctx = & m_otam_ctx;
  if(pctx->fsm_state == OTAM_FSM_CONNECTED){
    pctx->otac_dev.mtu = (DEFAULT_OTAM_MTU_SIZE > prsp->serverRxMTU) ?
                            prsp->serverRxMTU : DEFAULT_OTAM_MTU_SIZE;
    LOG("MTU size rsp: %d | set: %d\n", prsp->serverRxMTU, pctx->otac_dev.mtu);
  }
}


static int otaMM_Connect(void)
{
  int ret;
  otam_ctx_t* pctx = & m_otam_ctx;
  otafmesh_dev_t* pdev = &(pctx->dev);
 
  //load mac address from flash
  ret = otafm_dev_pull(pdev);
  if(ret)
    return ret;

  //establish link
  otaMM_EstablishLink(ADDRTYPE_PUBLIC, pdev->dev_addr);
  return PPlus_SUCCESS;
}

static void otaMM_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  otam_ctx_t* pctx = & m_otam_ctx;
  //LOG("[GATT] %x\n",pMsg->method);
  
  if ( pctx->fsm_state != OTAM_FSM_CONNECTED)
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }

  if(pMsg->hdr.status==bleTimeout)
  {
    LOG("[GATT TO] %x\n",pMsg->method);
    return;
  }


  switch(pMsg->method){
  case ATT_READ_RSP:
    on_gatt_read_rsp(&(pMsg->msg.readRsp));
    break;
  case ATT_WRITE_RSP:
    on_gatt_write_rsp();
    break;
  case ATT_HANDLE_VALUE_NOTI:
    on_gatt_notify(&(pMsg->msg.handleValueNoti));
    break;
  case ATT_HANDLE_VALUE_IND:
    on_gatt_indicate(&(pMsg->msg.handleValueInd));
    break;
  case ATT_EXCHANGE_MTU_RSP:
    on_gatt_mtu_rsp(&(pMsg->msg.exchangeMTURsp));
    break;  
  case ATT_ERROR_RSP:
    on_gatt_err_rsp(&(pMsg->msg.errorRsp));
    break;

  //below case is response for gatt discovery
  case ATT_READ_BY_GRP_TYPE_RSP:
  case ATT_READ_BY_TYPE_RSP:
    on_gatt_discovery(pMsg);
  
  default:
    break;
  }

}



static void otaMM_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
      otaMM_ProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
//    case OTAM_MSG_EVENT:
//      otaMM_ProcessCMDMsg(pMsg);
//      break;
  }
}


uint16 otaMM_ProcessEvent( uint8 task_id, uint16 events )
{
  
    VOID task_id; // OSAL required parameter that isn't used in this function

        if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( otaMM_TaskId )) != NULL )
        {
            otaMM_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &otaMM_RoleCB );

        // Register with bond manager after starting device
        GAPBondMgr_Register( (gapBondCBs_t *) &otaMM_BondCB );

        return ( events ^ START_DEVICE_EVT );
    }

  
    if ( events & START_DISCOVERY_SERVICE_EVT )
    {
        otaMM_StartDiscoveryService( );
        return ( events ^ START_DISCOVERY_SERVICE_EVT );
    }
  
    if ( events & OTA_DATA_DELAY_WRITE_EVT )
    {
      otap_evt_t ev;
      ev.ev = OTAP_EVT_DATA_WR_DELAY;
      ev.len = 0;
      ev.data = NULL;
      otamProtocol_event(&ev);
      return ( events ^ OTA_DATA_DELAY_WRITE_EVT );
    }
  
    if ( events & BLE_CMD_WRITE_OP_TIMEOUT_EVT )
    {
      otap_evt_t ev;
      ev.ev = OTAP_EVT_BLE_TIMEOUT;
      ev.len = 0;
      ev.data = NULL;
      otamProtocol_event(&ev);
      return ( events ^ BLE_CMD_WRITE_OP_TIMEOUT_EVT );
    }
  
    if ( events & OTA_APP_DISCONN_DELAY_EVT)
    {
      LOG("GAPCentralRole_TerminateLink\n");
      GAPCentralRole_TerminateLink(m_otam_ctx.otac_dev.conn_hdl);
      return ( events ^ OTA_APP_DISCONN_DELAY_EVT);
    }

    if ( events & START_PHY_DEL_MTU_EVT)
    {
      otac_dle_phy_mtu();
      return ( events ^ START_PHY_DEL_MTU_EVT);
    }

    if ( events & OTAM_CONN_EVT)
    {
      otaMM_Connect();
      return ( events ^ OTAM_CONN_EVT);
    }
    if ( events & OTAM_OTA_EVT)
    {
      otamProtocol_start_ota(0);
      return ( events ^ OTAM_CONN_EVT);
    }
    // Discard unknown events
    return 0;
}


static void otaMM_RssiCB( uint16 connHandle, int8 rssi )
{
  LOG( "RSSI -dB: %d\r\n", (uint8) (-rssi) );
}

static void otac_scan_completed(void)
{
  char str_mac[30];
  otam_ctx_t* pctx = & m_otam_ctx;
  otac_scan_dev_t* p_scandev = pctx->scan_dev_list;
  
  //1: reorder device list, remove devices that no name
  int i;
  pctx->scan_dev_num = 0;
  if((strlen(p_scandev->name) > 0) && (memcmp(p_scandev->addr, s_mac_zero, 6) != 0))
  {
    pctx->scan_dev_num = 1;
  }
  
  for(i = 1; i<DEFAULT_MAX_SCAN_NUM ; i ++ )
  {
    if(memcmp(p_scandev->addr, s_mac_zero, 6) == 0)
      break;
    if((strlen(p_scandev[i].name) > 0) && (memcmp(p_scandev[i].addr, s_mac_zero, 6) != 0))
    {
      if(pctx->scan_dev_num < i)
        memcpy(&(p_scandev[pctx->scan_dev_num]),&(p_scandev[i]), sizeof(otac_scan_dev_t));
      pctx->scan_dev_num ++;
    }
  }

  //2: print list
  LOG( "Devices Found: %d\n", pctx->scan_dev_num);
  for(i = 0; i < pctx->scan_dev_num; i++)
  {
    sprintf(str_mac, "[%.2x:%.2x:%.2x:%.2x:%.2x:%.2x]", 
                                      p_scandev[i].addr[5],
                                      p_scandev[i].addr[4],
                                      p_scandev[i].addr[3],
                                      p_scandev[i].addr[2],
                                      p_scandev[i].addr[1],
                                      p_scandev[i].addr[0]);
    LOG("%d  %s rssi:%d name:\"%s\"\n", i, str_mac, p_scandev[i].rssi, p_scandev[i].name);
  }
    
}

static void otac_dle_phy_mtu(void)
{
  otam_ctx_t* pctx = & m_otam_ctx;
  otac_dev_t* pdev = &(pctx->otac_dev);
  //-------------------------------------------------------------------------------------
  // DLE
  
  LL_SetDataLengh(0, 251, (251+10+4)*8);
  
  //phy update
  HCI_LE_SetPhyMode(0,0x00,0x02,0x02, 0);

  //MTU
  ATT_SetMTUSizeMax(DEFAULT_OTAM_MTU_SIZE);
  {
    attExchangeMTUReq_t pReq;
    pReq.clientRxMTU = DEFAULT_OTAM_MTU_SIZE;
    uint8 status =GATT_ExchangeMTU(pdev->conn_hdl, &pReq, otaMM_TaskId);
    LOG( "[MTU Req]%d %d\n",status,pReq.clientRxMTU);
  }

  osal_start_timerEx(otaMM_TaskId, START_DISCOVERY_SERVICE_EVT, DEFAULT_SVC_DISCOVERY_DELAY );

}

static void otac_dev_connected(gapEstLinkReqEvent_t* pEvent)
{
  otam_ctx_t* pctx = & m_otam_ctx;
  otac_dev_t* pdev = &(pctx->otac_dev);

  memset(pdev, 0, sizeof(otac_dev_t));
  
  pdev->conn_hdl = pEvent->connectionHandle;
  pdev->mtu = 23;
  pdev->disc_state = DISC_ST_SVC;
  
  LOG("otac_dev_connected Handle: 0x%x\n", pdev->conn_hdl);
  
  llInitFeatureSetDLE(TRUE);
  llInitFeatureSet2MPHY(TRUE);

  //HCI_ReadRemoteVersionInfoCmd(pdev->conn_hdl);
  HCI_LE_ReadRemoteUsedFeaturesCmd(pdev->conn_hdl);
  gapUpdateLinkParamReq_t Params = {
  .connectionHandle = 0, //!< Connection handle of the update
  .intervalMin = 12,      //!< Minimum Connection Interval
  .intervalMax = 12,      //!< Maximum Connection Interval
  .connLatency = 0,      //!< Connection Latency
  .connTimeout = 2000,      //!< Connection Timeout
    
  };
  
  GAP_UpdateLinkParamReq(&Params);
  
  osal_start_timerEx(otaMM_TaskId, START_PHY_DEL_MTU_EVT, 20);
  
}


static void otaMM_EventCB( gapCentralRoleEvent_t *pEvent )
{
  otam_ctx_t* pctx = &m_otam_ctx;
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INFO_EVENT:
    {
      // only save connectable adv
      otac_scan_update_devinfo( pEvent );
    }
    break;
    
    case GAP_DEVICE_DISCOVERY_EVENT:
    {
      // discovery complete
      // initialize scan index to last device
      
      otac_scan_completed();
      OTAM_FSM_SET(OTAM_FSM_IDLE);
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      LOG("\n== GAP_LINK_ESTABLISHED_EVENT ==\r\n");
      if ( pEvent->gap.hdr.status == SUCCESS )
      {
        otac_dev_connected(&(pEvent->linkCmpl));
        OTAM_FSM_SET(OTAM_FSM_CONNECTED);
      }
      else
      {
        LOG( "Connect Failed\n" );
				LOG( "Reason: 0x%02x\r\n", pEvent->gap.hdr.status);
        OTAM_FSM_SET(OTAM_FSM_IDLE);
        
      }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
      OTAM_FSM_SET(OTAM_FSM_IDLE);
      LOG( "Disconnected. " );
      LOG( "Reason: 0x%02x\r\n", pEvent->linkTerminate.reason);
      memset(&pctx->otac_dev, 0, sizeof(otac_dev_t));
      pctx->otac_dev.conn_hdl = 0xffff;
      otap_evt_t ev;
      ev.ev = OTAP_EVT_DISCONNECTED;
      ev.data = NULL;
      ev.len = 0;
      otamProtocol_event(&ev);
    }
    break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      LOG( "Server Request for Param Update\r\n");
    }
    break;
      
    default:
    {
      LOG(" otaMM_EventCB --> pEvent->gap.opcode: 0x%02X\r\n", pEvent->gap.opcode);
    }
    break;
  }
}


static void otaMM_PairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  LOG("otaMM_PairStateCB in param state 0x%02X,status 0x%02X\r\n",state,status);
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LOG( "Pairing started\n" );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LOG( "Pairing success\n" );
    }
    else
    {
      LOG( "Pairing fail\n" );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LOG( "Bonding success\n" );
    }
  }
}


static void otaMM_PasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
    LOG("otaMM_PasscodeCB\r\n");
}



/********* method  ******/
static int otaMM_method_clear(void)
{
  //otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
  osal_stop_timerEx(otaMM_TaskId, OTA_DATA_DELAY_WRITE_EVT);
  osal_clear_event(otaMM_TaskId, OTA_DATA_DELAY_WRITE_EVT);
  osal_stop_timerEx(otaMM_TaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
  osal_clear_event(otaMM_TaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT);
  return PPlus_SUCCESS;
}

static int otaMM_write_data_delay(uint32_t ms_delay)
{
  osal_start_timerEx(otaMM_TaskId, OTA_DATA_DELAY_WRITE_EVT, ms_delay);
  return PPlus_SUCCESS;
}

static int otaMM_write_data(uint8_t* data, uint16_t len)
{
  otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
  attWriteReq_t wreq;
  bStatus_t bret;

  if(pdev->conn_hdl != 0 || pdev->disc_state != DISC_ST_COMPLETED)
    return PPlus_ERR_INVALID_STATE;
  if(pdev->run_mode < OTAC_RUNMODE_OTA)
    return PPlus_ERR_INVALID_STATE;
  if(len > pdev->mtu -3)
    return PPlus_ERR_INVALID_LENGTH;
    
  memset(&wreq, 0, sizeof(attWriteReq_t));
  wreq.cmd = TRUE;
  wreq.sig = 0;
  wreq.handle = pdev->ota_service.char_data_hdl;
  wreq.len = len;
  memcpy(wreq.value, data, len);
  
  AT_LOG("TXDATA:\n");
  print_hex(data, len);
  bret = GATT_WriteNoRsp(pdev->conn_hdl, &wreq);
  //LOG("TXDATA RSP%d:\n",bret);
  if(bret!= SUCCESS)
    return PPlus_ERR_BLE_BUSY;
  return PPlus_SUCCESS;
}

static int otaMM_write_command(uint8_t* data, uint16_t len, uint32_t timeout)
{
  otac_dev_t* pdev = &(m_otam_ctx.otac_dev);
  attWriteReq_t wreq;
  bStatus_t bret;

  if(pdev->conn_hdl != 0 || pdev->disc_state != DISC_ST_COMPLETED)
    return PPlus_ERR_INVALID_STATE;
  if(pdev->run_mode < OTAC_RUNMODE_APP)
    return PPlus_ERR_INVALID_STATE;
  if(len > pdev->mtu -3)
    return PPlus_ERR_INVALID_LENGTH;
    
  memset(&wreq, 0, sizeof(attWriteReq_t));
  wreq.cmd = 0;
  wreq.sig = 0;
  wreq.handle = pdev->ota_service.char_cmd_hdl;
  wreq.len = len;
  memcpy(wreq.value, data, len);

  pdev->op_handle = wreq.handle;
  
  LOG("TXCMD:\n");
  print_hex(data, len);
  bret = GATT_WriteCharValue(pdev->conn_hdl, &wreq, otaMM_TaskId);
  if(bret!= SUCCESS)
    return PPlus_ERR_BLE_BUSY;
    
  if(timeout)
    osal_start_timerEx(otaMM_TaskId, BLE_CMD_WRITE_OP_TIMEOUT_EVT, timeout);
    
  return PPlus_SUCCESS;
  
}


/********end method******/


void otaMM_Init( uint8 task_id )
{
  otaMM_TaskId = task_id;

  memset(&m_otam_ctx, 0 , sizeof(m_otam_ctx));

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_NUM;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
	GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT    ,40);
	GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND   ,40);
  
  GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, 30 );      //  * 1.25ms      // 30
  GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, 40 );
  
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;//GAPBOND_PAIRING_MODE_INITIATE;//DEFAULT_PAIRING_MODE;    // GAPBOND_PAIRING_MODE_NO_PAIRING
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( otaMM_TaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  {
    otam_proto_meth_t op = {
      .clear            = otaMM_method_clear,
      .write_cmd        = otaMM_write_command,
      .write_data       = otaMM_write_data,
      .write_data_delay = otaMM_write_data_delay,
    };
    otamProtocol_init(&op);
  }

  OTAM_FSM_SET(OTAM_FSM_IDLE);

  hal_gpio_write(P23, 1);
  hal_gpio_write(P31, 1);
  hal_gpio_write(P32, 1);
	
  // Setup a delayed profile startup
  osal_set_event( otaMM_TaskId, START_DEVICE_EVT );
  osal_start_timerEx( otaMM_TaskId, OTAM_CONN_EVT, OTAM_CONN_TIMEOUT_PERIOD);

}





/*********************************************************************
*********************************************************************/

