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
    Filename:       wristservice.h
    Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
    Revision:       $Revision: 23333 $

    Description:    This file contains the wrist GATT profile definitions and
                  prototypes.

**************************************************************************************************/

#ifndef WRIST_SERVICE_H
#define WRIST_SERVICE_H

#include "peripheral.h"

#include "types.h"
#include "app_datetime.h"
/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/

#define WRIST_NOTI_ENABLED      1
#define WRIST_NOTI_DISABLED     2
#define WRIST_CMD_REQ           3



// Wrist Profile Service UUID
#define WRIST_SERV_UUID               0xFF01
// Key Pressed UUID
#define WRIST_CHAR_COMMAND_UUID       0xFF02
#define WRIST_CHAR_READONLY_UUID      0xFF10

// Simple Keys Profile Services bit fields
#define WRIST_SERVICE               0x00000001




/***************************************/
/*   private message protocol: 0x38    */
/***************************************/
#define MSG_NOTIF_MORE_DATA     1


#define MSG_NOTIF_BRIEF         0   //brief info, no extend data
#define MSG_NOTIF_BRIEF_E       1   //brief info, has more data
#define MSG_NOTIF_DATA          2   //message info, has no extend data
#define MSG_NOTIF_DATA_E        3   //message info, has more data


#define MSG_NOTIF_T_UNDEF   0
#define MSG_NOTIF_T_CALL    1
#define MSG_NOTIF_T_CALL_OFF    2
#define MSG_NOTIF_T_SMS     3
#define MSG_NOTIF_T_MAIL    4
#define MSG_NOTIF_T_WECHAT  5
#define MSG_NOTIF_T_QQ      6
#define MSG_NOTIF_T_APP     7

#define CAPABILITY_LCD      (1ul << 0)
#define CAPABILITY_DUAL_BT  (1ul << 1)  //dual mode Bluetooth
#define CAPABILITY_USB      (1ul << 2)  //support usb?
#define CAPABILITY_SE       (1ul << 3)  //support SE
#define CAPABILITY_HRATE    (1ul << 4)  //support heart rate measure
#define CAPABILITY_GSENSOR  (1ul << 5)  //support acceleration sensor
#define CAPABILITY_GYRO     (1ul << 6)  //support gyroscope
#define CAPABILITY_LONG_SIT (1ul << 7)  //support long sit remind
#define CAPABILITY_ANCS     (1ul << 8)  //support apple messge center service
#define CAPABILITY_WECHAT   (1ul << 9)  //support we chat sports
#define CAPABILITY_HAND_UP      (1ul << 10) //support led light when hand up 
#define CAPABILITY_LOSE_REMIND  (1ul << 11) //support lose remind


#define MSG_NOTIF_SIZE  128


enum
{
    WRIST_CMD_QUERY_VERSION     = 0x01,
    WRIST_CMD_SET_TIME          = 0x02,
    WRIST_CMD_GET_TIME          = 0x03,
    WRIST_CMD_LOOKUP_BRACELET   = 0x0e,
    WRIST_CMD_READ_BATT_VOLT  = 0x0f,

    WRIST_CMD_HR_GET_STATUS       = 0x20,
    WRIST_CMD_HR_START              = 0x21,
    WRIST_CMD_HR_STOP             = 0x22,
    WRIST_CMD_ACC_NOTIF_START = 0x23,
    WRIST_CMD_ACC_NOTIF_STOP  = 0x24,

    WRIST_CMD_LIGHT_CTRL            = 0x30,

    WRIST_CMD_MSG_NOTIF             = 0x38,

    //WRIST_NOTIFY_FIT          = 0x80,
    WRIST_NOTIFY_HR               = 0x81,
    //WRIST_NOTIFY_ATH          = 0x82,
    //WRIST_NOTIFY_UV               = 0x83,
    //WRIST_NOTIFY_ENV          = 0x84,
    WRIST_NOTIFY_HR_RAW         = 0x85,
    WRIST_NOTIFY_ACC        = 0x86,
    WRIST_NOTIFY_KSCAN      = 0x87,

    WRIST_RO_DATA               = 0xa0,

    WRIST_CMD_UNKNOW = 0xff
};


/*************************************
    command response data structure

*************************************/
typedef struct _wristRspVersion_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t     ver_fw[3];
    uint8_t     ver_stack[2];
    uint8_t     ver_hw[2];
    uint8_t     mac[6];
    uint8_t   capability[4];
    uint8_t     chksum;
} wristRspVersion_t;

typedef struct _wristRsp_t
{
    uint8       cmd;
    uint8       csn;
    uint8       err_code;
    uint8       chksum;
} wristRsp_t;


typedef struct _wristCmdNotif_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t     msg_type:6;
    uint8_t     pkt_type:2;
    uint8_t     msg_data[1];
} wristCmdNotif_t;

typedef struct _wristRspHrRaw_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint16_t  raw[8];
    uint8_t     chksum;
} wristRspHrRaw_t;

typedef struct _wristRspAcc_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t       acc[3*4];
    uint8_t     chksum;
} wristRspAcc_t;

typedef struct _wristRspKScan_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t   num;
    uint8_t       key[16];
    uint8_t     chksum;
} wristRspKScan_t;
typedef struct _wristRspHrValue_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t   value;
    uint8_t     chksum;
} wristRspHrValue_t;

typedef struct _wristRspBatt_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t     batt[2];
    uint8_t     chksum;
} wristRspBatt_t;


/*************************************
    command request data structure

*************************************/
typedef struct _wristCmdTM_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t     datetime[6];
    uint8_t     chksum;
} wristCmdTM_t;

typedef struct _wristCmdLight_t
{
    uint8_t     cmd;
    uint8_t     csn;
    uint8_t   value;
    uint8_t     ch;
    uint8_t     chksum;
} wristCmdLight_t;

enum
{
    SERV_TASK_IDLE = 0,
    SERV_TASK_FIT_RECORD = 1,
    SERV_TASK_SLEEP_RECORD,
    SERV_TASK_REQ_BINDING,
};

typedef struct _service_task_t
{
    uint8       id;
    uint8       cmd;
    uint8       csn;
    uint8       day_index;
    datetime_t  tm_start;
    uint16  record_num;
    uint16  record_index;
} service_task_t;

typedef struct _notifInfo_t
{
    uint8       type;
    uint8       offset;
    char*       data;
    bool        expired;
    bool        flg;
} notifInfo_t;


typedef struct _wristService_t
{
    service_task_t              service_task;
    notifInfo_t                 notifInfo;
} wristService_t;


/*********************************************************************
    MACROS
*/

/*********************************************************************
    Profile Callbacks
*/

typedef void (*wristServiceCB_t)(uint8 event, uint8 param_len, uint8* param);



extern int wristProfileResponseHRValue(uint8_t HR_Value);
extern int wristProfileResponseHRRawData(uint8_t cnt, uint16_t* pdata);
extern int wristProfileResponseAccelerationData(int gx, int gy, int gz);
extern int wristProfileResponseKScan(uint8_t num, uint8_t* key);

extern bStatus_t wristProfile_AddService(wristServiceCB_t cb);


extern void wristProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );



#endif /* WRIST_SERVICE_H */
