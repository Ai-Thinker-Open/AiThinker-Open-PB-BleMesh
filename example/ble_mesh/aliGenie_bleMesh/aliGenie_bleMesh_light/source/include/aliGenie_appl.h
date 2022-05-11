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


/**
    \file appl_sample_example_1.c

    Source File for Generic OnOff Server Standalone application without CLI or
    menu based console input interface.
*/

/*
    Copyright (C) 2018. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _aliGenie_appl_h_
#define _aliGenie_appl_h_
/* ----------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_config_api.h"
#include "MS_net_api.h"
#include "prov_pl.h"

#include "MS_health_server_api.h"

#include "access_extern.h"
#include "EXT_cbtimer.h"

#include "MS_scene_api.h"

#include "blebrr.h"
#include "nvsto.h"
#include "model_state_handler_pl.h"

#include "aliGenie_appl_node_mng.h"

#include "ali_genie_profile.h"
#include "aliGenie_led_light.h"

#include "aliGenie_appl_Prov.h"
#include "aliGenie_appl_Proxy.h"
#include "aliGenie_appl_Generic.h"

//printf("%s %s %s %s %d:"fmt, __FILE__, __FUNCTION__, __DATE__, __TIME__, __LINE__, ##__VA_ARGS__)
//#define printf_my(fmt, ...) printf("%s %s %s %s %d:"fmt, __FILE__, __FUNCTION__, __DATE__, __TIME__, __LINE__, ##__VA_ARGS__)


/* Console Input/Output */
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)
#define WHAT_THE(fmt,...)   printf("\n\n[E]!!!???WHAT???!!! @%s #%d\t\n"fmt,__FUNCTION__, __LINE__, ##__VA_ARGS__)

#define ERROR_PRINT(...)    {printf("[[E]]%s #%d\t",__FUNCTION__, __LINE__); printf(__VA_ARGS__)}
#define WARN_PRINT(...)     {printf("[W]"); printf(__VA_ARGS__)}
#define INFO_PRINT(...)     //{printf("[I]"); printf(__VA_ARGS__)}
#if 1
    #define DEBUG_PRINT(...)    {printf("[D]%s #%d\t",__FUNCTION__, __LINE__); printf(__VA_ARGS__)}
    #define DEBUG_ONLYTHIS(...)  //{printf("[D]%s #%d\t",__FUNCTION__, __LINE__); printf(__VA_ARGS__)}
#else
    #define DEBUG_PRINT(...)
    #define DEBUG_ONLYTHIS(...) {printf("[D]%s #%d\t",__FUNCTION__, __LINE__); printf(__VA_ARGS__)}
#endif
#define TRACELOG_PRINT(fmt,...) printf("[T]%s #%d\t\n"fmt,__FUNCTION__, __LINE__, ##__VA_ARGS__)

#undef     USE_HSL             // disable Light HSL server model
#undef     USE_CTL             // enable Light CTL server model
#undef     USE_LIGHTNESS       // enable Light lightness server model
#undef     USE_MAINLIGHT       // enable Light mainlight server model
#undef     USE_BACKLIGHT       // enable Light backlight server model
#define     USE_SCENE           // enable Light scene server model


#define     SCENE_READING_MODE          3
#define     SCENE_CINEMATIC_MODE        4
#define     SCENE_WARM_MODE             5
#define     SCENE_NIGHTLIGHT_MODE       6
#define     SCENE_GETUP_MODE            8
#define     SCENE_SLEEP_MODE            14
#define     SCENE_EYESHIELD_MODE        57

extern void appl_show_model_handle(MS_ACCESS_MODEL_HANDLE* model_handle);
extern void appl_dump_bytes(UCHAR* buffer, UINT16 length);
extern void appl_mesh(void);



void UI_register_prov(void);

void UI_reinit(void);
void UI_gatt_iface_event_pl_cb
(
    UCHAR  ev_name,
    UCHAR  ev_param
);
API_RESULT UI_check_app_key(void);
API_RESULT UI_set_brr_scan_rsp_data (void);


/* --------------------------------------------- Global Definitions */
#define MS_MODEL_ID_VENDOR_ALIGENIE_SERVER                         0x01A80000

#define MS_ACCESS_VENDOR_ALIGENIE_GET_OPCODE                         0x00D0A801
#define MS_ACCESS_VENDOR_ALIGENIE_SET_OPCODE                         0x00D1A801
#define MS_ACCESS_VENDOR_ALIGENIE_SET_UNACKNOWLEDGED_OPCODE          0x00D2A801
#define MS_ACCESS_VENDOR_ALIGENIE_STATUS                             0x00D3A801
#define MS_ACCESS_VENDOR_ALIGENIE_INDICATION                         0x00D4A801
#define MS_ACCESS_VENDOR_ALIGENIE_CONFIRMATION                       0x00D5A801
#define MS_ACCESS_VENDOR_ALIGENIE_TRANSPARENT_MSG                    0x00CFA801


#define MS_STATE_VENDOR_HSL_T                                   0x0123
#define MS_STATE_VENDOR_ONOFF_T                                 0x0100
#define MS_STATE_VENDOR_LIGHTNESS_T                             0x0121
#define MS_STATE_VENDOR_CTL_T                                   0x0122
#define MS_STATE_VENDOR_MAINLIGHTONOFF_T                        0x0534
#define MS_STATE_VENDOR_BACKLIGHTONOFF_T                        0x0533
#define MS_STATE_VENDOR_MODENUMBER_T                            0xF004
#define MS_STATE_VENDOR_EVENT_INDICATE_T                        0xF009

#define MS_STATE_VENDOR_END_TIME_ONOFF_T                         0xF010//change by johhn
#define MS_STATE_VENDOR_NOW_TIME_ONOFF_T                         0xF01F//change by johhn
/** Vendor Model specific state parameters in a request or response message */
typedef struct _MS_access_vendor_model_state_params
{
    /** State Type */
    UINT16 state_type;

    /** State pointer */
    void* state;

} MS_ACCESS_VENDOR_MODEL_STATE_PARAMS;


typedef struct MS_state_vendor_model_end_time_onoff_struct

{

    UINT8 index;//第几条信息
    UINT8 second1;//时间戳
    UINT8 second2;//时间戳
    UINT8 second3;//时间戳
    UINT8 second4;//时间戳
    UINT8 index1;//
    UINT8 index2;//
    UINT8 onoff;//开关

} MS_STATE_VENDOR_MODEL_END_TIME_ONOFF_STRUCT;

typedef struct MS_state_vendor_model_now_time_onoff_struct
{


    UINT8 second1;//时间戳
    UINT8 second2;//时间戳
    UINT8 second3;//时间戳
    UINT8 second4;//时间戳


} MS_STATE_VENDOR_MODEL_NOW_TIME_ONOFF_STRUCT;





/* ----------------------------------------- External Global Variables */
extern UCHAR blebrr_state;
extern uint8 g_dev_reset_flg;

/* ----------------------------------------- External Global Function */
extern void WaitMs(uint32_t msecond);
/* ----------------------------------------- Exported Global Variables */


/* ----------------------------------------- Static Global Variables */



extern EM_timer_handle thandle;
extern uint8 vendor_msg_tid;
extern UINT16 aligenie_addr;

extern void timeout_cb (void* args, UINT16 size);
extern API_RESULT ms_scene_store(MS_ACCESS_MODEL_HANDLE* handle, UINT16 scene_number);




extern MS_ACCESS_MODEL_HANDLE    UI_config_server_model_handle;
extern MS_ACCESS_MODEL_HANDLE    UI_health_server_model_handle;

/* Vendor Defined Model Server */
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    VENDOR_ALIGENIE_1 server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
extern void vm_subscription_add ( MS_ACCESS_ELEMENT_HANDLE element_handle,MS_NET_ADDR addr);
extern API_RESULT UI_get_device_key(void );
extern API_RESULT UI_binding_app_key(MS_ACCESS_MODEL_HANDLE model_handle);
extern void UI_vendor_defined_model_states_initialization(void);
extern API_RESULT UI_register_vendor_defined_model_server(MS_ACCESS_ELEMENT_HANDLE element_handle);
//extern API_RESULT UI_vendor_aliGenie_model_server_create(MS_ACCESS_NODE_ID    node_id,MS_ACCESS_ELEMENT_HANDLE element_handle,UI_DATA_GENERIC_ONOFF_MODEL_T * generic_onoff_me);


#endif
