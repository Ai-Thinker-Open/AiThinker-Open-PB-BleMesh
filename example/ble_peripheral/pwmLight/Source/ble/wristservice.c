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
    Filename:       wristservice.c
    Revised:        $Date: 2018-02-06 13:33:47 -0700 (Mon, 06 May 2013) $
    Revision:       $Revision: 34153 $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include <string.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
//#include "ui_page.h"
#include "led_light.h"
//#include "em70xx.h"
#include "wristservice.h"
#include "log.h"
#include "app_err.h"
#include "error.h"

/*********************************************************************
    MACROS
*/
#define SERVAPP_NUM_ATTR_SUPPORTED 8

/*********************************************************************
    CONSTANTS
*/


/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
// Wrist GATT Profile Service UUID: 0xFF01
CONST uint8 wristServUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WRIST_SERV_UUID), HI_UINT16(WRIST_SERV_UUID)
};

// Characteristic 1 UUID: 0xFF02
CONST uint8 wristProfilecharCommandUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WRIST_CHAR_COMMAND_UUID), HI_UINT16(WRIST_CHAR_COMMAND_UUID)
};

// Characteristic 2 UUID: 0xFF10
CONST uint8 wristProfilecharReadonlyUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WRIST_CHAR_READONLY_UUID), HI_UINT16(WRIST_CHAR_READONLY_UUID)
};

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

/*********************************************************************
    Profile Attributes - variables
*/

// Wrist Profile Service attribute
static CONST gattAttrType_t wristProfileService = { ATT_BT_UUID_SIZE, wristServUUID };


// Wrist Profile Characteristic 1 Properties
static uint8 wristProfileCharCommandProps = GATT_PROP_READ | GATT_PROP_WRITE|GATT_PROP_NOTIFY;

// Characteristic 1 Value
static uint8 wristProfileCharCommand[20];

// Wrist Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t wristProfileCmdCCC[GATT_MAX_NUM_CONN];

// Wrist Profile Characteristic 1 User Description
static uint8 wristProfileCharCommandUserDesp[14] = "CharCommand\0";


// Wrist Profile Characteristic 2 Properties
static uint8 wristProfileCharReadonlyProps = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 wristProfileCharReadonly[20];

// Wrist Profile Characteristic 2 User Description
static uint8 wristProfileCharReadonlyUserDesp[14] = "CharReadonly\0";



/*********************************************************************
    Profile Attributes - Table
*/

static gattAttribute_t wristProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
    // Wrist Profile Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)& wristProfileService            /* pValue */
    },

    // Command Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &wristProfileCharCommandProps
    },

    // Characteristic Value 1
    {
        { ATT_BT_UUID_SIZE, wristProfilecharCommandUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &wristProfileCharCommand[0]
    },

    // Characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8*)wristProfileCmdCCC
    },

    // Characteristic command User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        wristProfileCharCommandUserDesp
    },

    // Characteristic readonly Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &wristProfileCharReadonlyProps
    },

    // Characteristic Value 2
    {
        { ATT_BT_UUID_SIZE, wristProfilecharReadonlyUUID },
        GATT_PERMIT_READ,
        0,
        &wristProfileCharReadonly[0]
    },

    // Characteristic 2 User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        wristProfileCharReadonlyUserDesp
    }

};


static wristServiceCB_t wristServiceCB = NULL;


//static wristService_t sWristService;

static bool sNotifyAccelerationDataFlag = FALSE;
//static char   s_msg_notif_data[MSG_NOTIF_SIZE];


/*********************************************************************
    LOCAL FUNCTIONS
*/
static uint8 wristProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                      uint8* pValue, uint16* pLen, uint16 offset, uint8 maxLen );
static bStatus_t wristProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                           uint8* pValue, uint16 len, uint16 offset );

static int wristProfile_Notify(attHandleValueNoti_t* pNoti );

static int cmd_response_err(const uint8* data, uint16 len, uint8 err_code);
static int cmd_response(const uint8* data, uint16 len);


static uint8 checksum(const uint8* data, uint8 len)
{
    uint8 i = 0;
    uint8 chksum = 0;

    for(i = 0; i < len; i ++)
        chksum = chksum^data[i];

    return chksum;
}

static void print_hex (const uint8* data, uint16 len)
{
    uint16 i;

    for (i = 0; i < len - 1; i++)
    {
        LOG("%x",data[i]);
        LOG(" ");
    }

    LOG("%x\n",data[i]);
}


static int cmd_query_version(const uint8* data)
{
    wristRspVersion_t version;
    version.cmd = data[0];  //cmd
    version.csn = data[1];  //csn
    version.ver_fw[0] = 1;//FW_VER_MAJOR;
    version.ver_fw[1] = 0;//FW_VER_MINOR;
    version.ver_fw[2] = 2;//FW_VER_REVISION;
    version.ver_stack[0] = 1;
    version.ver_stack[1] = 0;
    version.ver_hw[0] = 1;
    version.ver_hw[1] = 0;
    GAPRole_GetParameter(GAPROLE_BD_ADDR,version.mac);
    //version.capability = (CAPABILITY_LCD|
    //          CAPABILITY_SE|
    //          CAPABILITY_HRATE|
    //          CAPABILITY_GSENSOR|
    //          CAPABILITY_LONG_SIT|
    //          CAPABILITY_ANCS|
    //          CAPABILITY_WECHAT|
    //          CAPABILITY_HAND_UP|
    //          CAPABILITY_LOSE_REMIND);
    version.chksum = checksum((uint8*)(&version), sizeof(version)-1);
    return cmd_response((uint8*)(&version), sizeof(version));
}

static int cmd_set_time(const uint8* data, uint16 len)
{
    int ret = APP_SUCCESS;
    datetime_t  dtm, dtm_new;
    wristCmdTM_t* psettm = (wristCmdTM_t*) data;
    //apply datetime
    app_datetime(&dtm);
    BCDTM2DTM(&dtm_new, psettm->datetime);
    LOG("cmd_set_time: %d,%d,%d,%d,%,%d\n", dtm_new.year,dtm_new.month,dtm_new.day,dtm_new.hour,dtm_new.minutes,dtm_new.seconds);
    ret = app_datetime_set(dtm_new);
    return cmd_response_err(data, len, ret);
}


static int cmd_get_time(const uint8* data, uint16 len)
{
    datetime_t  dtm;
    wristCmdTM_t rsp;
    //apply datetime
    app_datetime(&dtm);
    rsp.cmd = data[0];
    rsp.csn = data[1];
    DTM2BCDTM(rsp.datetime, &dtm);
    rsp.chksum = checksum((uint8*)(&rsp), sizeof(rsp)-1);
    return cmd_response((uint8*)(&rsp), sizeof(rsp));
}

//static int cmd_HR_get_status(const uint8_t* data, uint16_t len)
//{

//  return cmd_response_err(data, len, 0);
//}
//static int cmd_HR_start(const uint8_t* data, uint16_t len)
//{
//  em70xx_start();
//  return cmd_response_err(data, len, APP_ERR_NOT_IMPLEMENTED);
//}
//static int cmd_HR_stop(const uint8_t* data, uint16_t len)
//{
//  em70xx_stop();
//  return cmd_response_err(data, len, APP_SUCCESS);
//}

//static int cmd_acc_notif_start(const uint8* data, uint16 len)
//{
//  sNotifyAccelerationDataFlag = TRUE;
//  return cmd_response_err(data, len, APP_SUCCESS);
//}

//static int cmd_acc_notif_stop(const uint8* data, uint16 len)
//{
//  sNotifyAccelerationDataFlag = FALSE;
//  return cmd_response_err(data, len, APP_SUCCESS);
//}

static int cmd_light_ctrl(const uint8_t* data, uint16_t len)
{
    int ret;
    wristCmdLight_t* plight = (wristCmdLight_t*)data;
    ret = light_ctrl(plight->ch, plight->value);
    ret = (ret == PPlus_SUCCESS) ? APP_SUCCESS: APP_ERR_PARAM;
    return cmd_response_err(data, len, ret);
}

static int cmd_lookup_bracelet(const uint8* data, uint16 len)
{
    //add code for vibrating the bracelet
    //......
    //response code
    return cmd_response_err(data, len, APP_SUCCESS);
}

//static int cmd_read_batt_volt(const uint8* data, uint16 len)
//{
//  wristRspBatt_t battrsp;
//  uint16_t volt = (uint16_t)batt_voltage_int();
//  LOG("cmd_read_batt_volt: %x\n",volt);
//  battrsp.cmd = data[0];
//  battrsp.csn = data[1];
//  battrsp.batt[0] = (uint8_t)(volt&0xff);
//  battrsp.batt[1] = (uint8_t)((volt>>8)&0xff);
//  battrsp.chksum = checksum((uint8*)(&battrsp), sizeof(battrsp)-1);
//  return cmd_response((uint8_t*)(&battrsp), sizeof(battrsp));
//}

//void msg_notif_dispatch(void)
//{
//  notifInfo_t* p_notifInfo = &(sWristService.notifInfo);
//  ui_ev_t ev;
//  switch(p_notifInfo->type){
//  case MSG_NOTIF_T_UNDEF:// 0
//    break;
//  case MSG_NOTIF_T_CALL://  1
//    ev.ev = UI_EV_BLE_CALL;
//    ui_fsm_run(&ev);
//    ev.ev = UI_EV_BLE_CALL_INFO;
//    ev.data = (uint8*)p_notifInfo->data;
//    ui_fsm_run(&ev);
//    break;
//  case MSG_NOTIF_T_CALL_OFF://  2
//    ev.ev = UI_EV_BLE_CALL_OFF;
//    ui_fsm_run(&ev);
//    break;
//  case MSG_NOTIF_T_SMS://   3
//    ev.ev = UI_EV_BLE_SMS;
//    ev.data = (uint8*)p_notifInfo->data;
//    ui_fsm_run(&ev);
//    break;
//  case MSG_NOTIF_T_MAIL://  4
//    ev.ev = UI_EV_BLE_MAIL;
//    ev.data = (uint8*)p_notifInfo->data;
//    ui_fsm_run(&ev);
//    break;
//  case MSG_NOTIF_T_WECHAT://  5
//    ev.ev = UI_EV_BLE_WECHAT;
//    ev.data = (uint8*)p_notifInfo->data;
//    ui_fsm_run(&ev);
//    break;
//  case MSG_NOTIF_T_QQ://    6
//    ev.ev = UI_EV_BLE_QQ;
//    ev.data = (uint8*)p_notifInfo->data;
//    ui_fsm_run(&ev);
//    break;
//  case MSG_NOTIF_T_APP://   7
//    ev.ev = UI_EV_BLE_MSG_NOTIFY;
//    ev.data = (uint8*)p_notifInfo->data;
//    ui_fsm_run(&ev);
//    break;
//  default:
//    break;
//  }
//}

//static int cmd_msg_notification(const uint8* data, uint16 len)
//{
//  wristCmdNotif_t* p_msg = (wristCmdNotif_t*)data;
//  notifInfo_t* p_notif_info = &(sWristService.notifInfo);
//  char* p_data = p_notif_info->data;
//  uint8 msg_len = len - 4;

//  uint8 msg_type = p_msg->msg_type;
//  uint8 pkt_type = p_msg->pkt_type;
//
//  bool flg = p_notif_info->flg;
//
//    //if just brief info, discard exist message, replaced new
//  if(pkt_type == MSG_NOTIF_BRIEF || pkt_type == MSG_NOTIF_BRIEF_E){
//    memset(p_data, 0, MSG_NOTIF_SIZE);
//    memcpy(p_data, p_msg->msg_data, msg_len);
//    p_notif_info->type = msg_type;
//    p_notif_info->offset = msg_len;
//    p_notif_info->flg = false;
//    if(pkt_type == MSG_NOTIF_BRIEF){
//      p_notif_info->flg = false;
//      msg_notif_dispatch();
//      return cmd_response_err(data, len, APP_SUCCESS);
//    }
//    else
//    {
//      p_notif_info->flg = true;
//      return cmd_response_err(data, len, MSG_NOTIF_MORE_DATA);
//    }
//  }
//  else if(flg){
//    if(msg_type != p_notif_info->type){
//      p_notif_info->type = 0;
//      return cmd_response_err(data, len, APP_ERR_PARAM);
//    }
//    memcpy(p_data + p_notif_info->offset, p_msg->msg_data, msg_len);
//    p_notif_info->offset += msg_len;
//    if(pkt_type == MSG_NOTIF_DATA){
//      p_notif_info->flg = false;
//      msg_notif_dispatch();
//      return cmd_response_err(data, len, APP_SUCCESS);
//    }
//    else
//    {
//      p_notif_info->flg = true;
//      return cmd_response_err(data, len, MSG_NOTIF_MORE_DATA);
//    }

//  }

//  return APP_SUCCESS;
//}


int on_recieved_cmd_packet(const uint8* data, uint16 len)
{
    uint32 ret = APP_SUCCESS;
    uint8 chksum = data[len-1];
    uint8 err_data = 0;
    LOG("RX Cmd:");
    print_hex(data, len);

    if(chksum != checksum(data, len-1))
    {
        err_data = WRIST_CMD_UNKNOW;
        return cmd_response_err(&err_data, sizeof(err_data), APP_ERR_CRC);
    }

    switch(data[0])
    {
    case  WRIST_CMD_QUERY_VERSION:
        ret = cmd_query_version(data);
        break;

    case  WRIST_CMD_SET_TIME:
        ret = cmd_set_time(data, len);
        break;

    case  WRIST_CMD_GET_TIME:
        ret = cmd_get_time(data, len);
        break;

    case  WRIST_CMD_LOOKUP_BRACELET:
        ret = cmd_lookup_bracelet(data, len);
        break;

//  case  WRIST_CMD_READ_BATT_VOLT:
//    ret = cmd_read_batt_volt(data, len);
//    break;
//  case  WRIST_CMD_HR_GET_STATUS:
//    ret = cmd_HR_get_status(data, len);
//    break;
//  case  WRIST_CMD_HR_START:
//    ret = cmd_HR_start(data, len);
//    break;
//  case  WRIST_CMD_HR_STOP:
//    ret = cmd_HR_stop(data, len);
//    break;

//  case  WRIST_CMD_ACC_NOTIF_START:
//    ret = cmd_acc_notif_start(data, len);
//    break;
//  case  WRIST_CMD_ACC_NOTIF_STOP:
//    ret = cmd_acc_notif_stop(data, len);
//    break;
//
    case  WRIST_CMD_LIGHT_CTRL:
        ret = cmd_light_ctrl(data, len);
        break;

//  case  WRIST_CMD_MSG_NOTIF:
//    ret = cmd_msg_notification(data, len);
//    break;
    default:
        err_data = WRIST_CMD_UNKNOW;
        return cmd_response_err(&err_data, sizeof(err_data), APP_ERR_UNKNOW_CMD);
        //unknow command
    }

    return ret;
}

static int cmd_response_err(const uint8* data, uint16 len, uint8 err_code)
{
    attHandleValueNoti_t notif;
    wristRsp_t data_rsp;
    data_rsp.cmd = data[0];
    data_rsp.csn = (len >1) ? data[1] : 0;
    data_rsp.err_code = err_code;
    data_rsp.chksum = checksum((uint8*)&data_rsp, sizeof(wristRsp_t)-1);
    memset(&notif, 0, sizeof(notif));
    notif.len = sizeof(data_rsp);
    notif.value[0] = data_rsp.cmd;
    notif.value[1] = data_rsp.csn;
    notif.value[2] = data_rsp.err_code;
    notif.value[3] = data_rsp.chksum;
    return wristProfile_Notify(&notif);
}

static int cmd_response(const uint8* data, uint16 len)
{
    int i;
    attHandleValueNoti_t notif;
    memset(&notif, 0, sizeof(notif));
    notif.len = len;

    for(i = 0; i< len; i++)
        notif.value[i] = data[i];

    return wristProfile_Notify(&notif);
}


/*********************************************************************
    PROFILE CALLBACKS
*/
// Wrist Profile Service Callbacks
CONST gattServiceCBs_t wristProfileCBs =
{
    wristProfile_ReadAttrCB,  // Read callback function pointer
    wristProfile_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/



int wristProfileResponseHRValue(uint8_t HR_Value)
{
    wristRspHrValue_t value;
    value.cmd = WRIST_NOTIFY_HR;
    value.csn = 0;
    value.value = HR_Value;
    value.chksum = checksum((uint8*)(&value), sizeof(value)-1);
    return cmd_response((uint8*)(&value), sizeof(value));
}

int wristProfileResponseHRRawData(uint8_t cnt, uint16_t* pdata)
{
    wristRspHrRaw_t rawdata;
    rawdata.cmd = WRIST_NOTIFY_HR_RAW;
    rawdata.csn = 0;
    memcpy(rawdata.raw, pdata, cnt*2);
    rawdata.chksum = checksum((uint8*)(&rawdata), sizeof(rawdata)-1);
    return cmd_response((uint8*)(&rawdata), sizeof(rawdata));
}

/*acc_value: 1000 == 1G*/
int wristProfileResponseAccelerationData(int gx, int gy, int gz)
{
    if(sNotifyAccelerationDataFlag)
    {
        int acc[3];
        wristRspAcc_t accdata;
        acc[0] = gx;
        acc[1] = gy;
        acc[2] = gz;
        accdata.cmd = WRIST_NOTIFY_ACC;
        accdata.csn = 0;
        memcpy(accdata.acc, (void*)acc, 4*3);
        accdata.chksum = checksum((uint8*)(&accdata), sizeof(accdata)-1);
        return cmd_response((uint8*)(&accdata), sizeof(accdata));
    }

    return APP_SUCCESS;
}

/*response key scan data*/
int wristProfileResponseKScan(uint8_t num, uint8_t* key)
{
    wristRspKScan_t kscandata;
    memset(&kscandata, 0, sizeof(kscandata));
    kscandata.cmd = WRIST_NOTIFY_KSCAN;
    kscandata.csn = 0;
    kscandata.num = num;
    memcpy(kscandata.key, (void*)key, num);
    kscandata.chksum = checksum((uint8*)(&kscandata), sizeof(kscandata)-1);
    return cmd_response((uint8*)(&kscandata), sizeof(kscandata));
}


/*********************************************************************
    @fn      SimpleProfile_AddService

    @brief   Initializes the Wrist Profile service by registering
            GATT attributes with the GATT server.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.

    @return  Success or Failure
*/
bStatus_t wristProfile_AddService(wristServiceCB_t cb)
{
    uint8 status = SUCCESS;
    //sWristService.notifInfo.data = s_msg_notif_data;
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, wristProfileCmdCCC );
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( wristProfileAttrTbl,
                                          GATT_NUM_ATTRS( wristProfileAttrTbl ),
                                          &wristProfileCBs );
    wristServiceCB = cb;
    return ( status );
}



/*********************************************************************
    @fn          wristProfile_ReadAttrCB

    @brief       Read an attribute.

    @param       connHandle - connection message was received on
    @param       pAttr - pointer to attribute
    @param       pValue - pointer to data to be read
    @param       pLen - length of data to be read
    @param       offset - offset of the first octet to be read
    @param       maxLen - maximum length of data to be read

    @return      Success or Failure
*/
static uint8 wristProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                      uint8* pValue, uint16* pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;

    // If attribute permissions require authorization to read, return error
    if ( gattPermitAuthorRead( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if ( offset > 0 )
    {
        return ( ATT_ERR_ATTR_NOT_LONG );
    }

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

        switch ( uuid )
        {
        case WRIST_CHAR_COMMAND_UUID:
        case WRIST_CHAR_READONLY_UUID:
            *pLen = 10;
            pValue[0] = 1;
            pValue[1] = 2;
            pValue[2] = 3;
            pValue[3] = 4;
            pValue[4] = 5;
            pValue[5] = 6;
            pValue[6] = 7;
            break;

        default:
            // Should never get here! (characteristics 3 and 4 do not have read permissions)
            *pLen = 0;
            status = ATT_ERR_ATTR_NOT_FOUND;
            break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    return ( status );
}

/*********************************************************************
    @fn      wristProfile_WriteAttrCB

    @brief   Validate attribute data prior to a write operation

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/
static bStatus_t wristProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                           uint8* pValue, uint16 len, uint16 offset )
{
    bStatus_t status = SUCCESS;

    // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

        switch ( uuid )
        {
        case WRIST_CHAR_COMMAND_UUID:
            on_recieved_cmd_packet(pValue, len);
            break;

        case GATT_CLIENT_CHAR_CFG_UUID:
            status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                     offset, GATT_CLIENT_CFG_NOTIFY );

            if ( status == SUCCESS && wristServiceCB)
            {
                uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
                uint8 evt = (charCfg == GATT_CFG_NO_OPERATION) ? WRIST_NOTI_DISABLED :WRIST_NOTI_ENABLED;
                (*wristServiceCB)(evt, 0, NULL);
            }

            break;

        default:
            // Should never get here! (characteristics 2 and 4 do not have write permissions)
            status = ATT_ERR_ATTR_NOT_FOUND;
            break;
        }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    return ( status );
}

/*********************************************************************
    @fn          wristProfile_HandleConnStatusCB

    @brief       Wrist Profile link status change handler function.

    @param       connHandle - connection handle
    @param       changeType - type of change

    @return      none
*/
void wristProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
                ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
                  ( !linkDB_Up( connHandle ) ) ) )
        {
            GATTServApp_InitCharCfg( connHandle, wristProfileCmdCCC );
        }
    }
}

static int wristProfile_Notify(attHandleValueNoti_t* pNoti )
{
    uint16 connHandle;
    uint16 value;
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
    value = GATTServApp_ReadCharCfg( connHandle, wristProfileCmdCCC);
    LOG("GATT_Notification: %x\n", value);

    // If notifications enabled
    if ( value & GATT_CLIENT_CFG_NOTIFY )
    {
        bStatus_t st;
        // Set the handle
        pNoti->handle = wristProfileAttrTbl[2].handle;
        // Send the notification
        st = GATT_Notification( connHandle, pNoti, FALSE);
        LOG("st: %x\n", st);

        if(st != SUCCESS)
            return APP_ERR_BLE_SEND_FAIL;

        return APP_SUCCESS;
    }

    return APP_ERR_INVALID_STATE;
}

/*********************************************************************
*********************************************************************/
