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
    \file appl_sample_switch.c

    Source File for Mesh SIG Model Client(OnOff/HSL/CTL/...) as SWITCH Standalone application

*/

/*
    Copyright (C) 2018. Phyplus Ltd.
    All rights reserved.
*/



/* ----------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_config_api.h"
#include "MS_health_server_api.h"
#include "MS_generic_onoff_api.h"
#include "MS_light_hsl_api.h"
#include "MS_light_ctl_api.h"
#include "MS_scene_api.h"

#include "MS_net_api.h"
#include "blebrr.h"
#include "nvsto.h"
#include "model_state_handler_pl.h"

#include "flash.h"


#include "pwrmgr.h"
#include "led_light.h"
#include "bleMesh.h"
#include "access_extern.h"
#include "EXT_cbtimer.h"

#define USE_HSL                 // enable Light HSL server model
#undef USE_LIGHTNESS            // enable Light Lightness server model
#undef  USE_CTL                 // disable Light CTL server model
#define USE_SCENE               // enable Light Scene server model
#define USE_VENDORMODEL         // enable Light vendormodel server model
#define  EASY_BOUNDING


//#ifdef USE_VENDORMODEL
//    #define  EASY_BOUNDING
//#endif


/* Console Input/Output */
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)

void appl_dump_bytes(UCHAR* buffer, UINT16 length);
void appl_mesh_sample (void);

#define VENDOR_PRODUCT_MAC_ADDR         0x4000
#define PROCFG_COMPLETE_TIMEOUT         30


#define UI_FRND_CRITERIA                0x4B
#define UI_FRND_RECEIVE_DELAY_MS        100
#define UI_FRND_POLLTIMEOUT_100MS       100
#define UI_FRND_SETUPTIMEOUT            10000

#define MS_MODEL_ID_VENDOR_EXAMPLE_CLIENT                         0x00010504

#define MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE                         0x00D00405
#define MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE                         0x00D10405
#define MS_ACCESS_VENDOR_EXAMPLE_STATUS_OPCODE                      0x00D30405
#define MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE                    0x00D40405
#define MS_ACCESS_VENDOR_EXAMPLE_INDICATION_OPCODE                  0x00D50405

#define MS_STATE_VENDOR_CTRL_T                                      0x0122

extern uint32            osal_sys_tick;

uint16  src_uaddr;

EM_timer_handle thandle;


extern UCHAR blebrr_prov_started;

/* Provsion timeout handle */
EM_timer_handle procfg_timer_handle;

/** Vendor Model specific state parameters in a request or response message */
typedef struct _MS_access_vendor_model_state_params
{
    /** State Type */
    UINT16 state_type;

    /** State pointer */
    void* state;

    UINT8 state_coun;

} MS_ACCESS_VENDOR_MODEL_STATE_PARAMS;

/* ----------------------------------------- Static Global Variables */
static DECL_CONST UINT32 vendor_example_client_opcode_list[] =
{
    // >>>, ADD, PANDA, GET/SET opcode for binding group key
    MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE,
    // <<<,

    MS_ACCESS_VENDOR_EXAMPLE_STATUS_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_INDICATION_OPCODE,
};

typedef struct MS_state_vendor_example_struct
{
    UINT16  maun_opcode;
    UCHAR  value;

} MS_STATE_VENDOR_EXAMPLE_STRUCT;

typedef struct MS_state_vendor_example_test_struct
{
    uint16  message_index;
    uint32  osal_tick;
    uint16  src_addr;
    uint8   data_len;

} MS_STATE_VENDOR_EXAMPLE_TEST_STRUCT;



/**
    Vendor Example Server application Asynchronous Notification Callback.

    Vendor Example Server calls the registered callback to indicate events occurred to the
    application.

    \param [in] ctx           Context of the message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
typedef API_RESULT (* MS_VENDOR_EXAMPLE_CLIENT_CB)
(
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT*      ctx,
    MS_ACCESS_MODEL_REQ_MSG_RAW*          msg_raw,
    MS_ACCESS_MODEL_REQ_MSG_T*            req_type,
    MS_ACCESS_VENDOR_MODEL_STATE_PARAMS* state_params,
    MS_ACCESS_MODEL_EXT_PARAMS*           ext_params

) DECL_REENTRANT;

static MS_VENDOR_EXAMPLE_CLIENT_CB       vendor_example_client_UI_cb;



/* ----------------------------------------- External Global Variables */


/* ----------------------------------------- Exported Global Variables */
void UI_prov_bind(UCHAR brr, UCHAR index);
API_RESULT UI_prov_callback
(
    PROV_HANDLE* phandle,
    UCHAR         event_type,
    API_RESULT    event_result,
    void*         event_data,
    UINT16        event_datalen
);
void UI_proxy_callback
(
    NETIF_HANDLE*        handle,
    UCHAR                p_evt,
    UCHAR*               data_param,
    UINT16               data_len
);
void UI_proxy_start_adv(MS_SUBNET_HANDLE subnet_handle, UCHAR proxy_adv_mode);
API_RESULT UI_register_foundation_model_servers
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
);

void UI_register_prov(void);
void UI_register_proxy(void);
void UI_sample_reinit(void);
void UI_gatt_iface_event_pl_cb
(
    UCHAR  ev_name,
    UCHAR  ev_param
);
API_RESULT UI_sample_check_app_key(void);
API_RESULT UI_sample_get_device_key(void );
API_RESULT UI_set_brr_scan_rsp_data (void);
void UI_setup_prov(UCHAR role, UCHAR brr);
void UI_lpn_seek_friend(void);
API_RESULT UI_vendormodel_client_send_reliable_pdu
(
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param
);
void timeout_cb (void* args, UINT16 size);




/* -----------------------------------------  Global Variables */

uint8 enable_sleep_flag = 0;

/* ----------------------------------------- Functions */
/* Model Server - Foundation Models */

/* Health Server - Test Routines */
static void UI_health_self_test_00(UINT8 test_id, UINT16 company_id)
{
}

static void UI_health_self_test_01(UINT8 test_id, UINT16 company_id)
{
}

static void UI_health_self_test_FF(UINT8 test_id, UINT16 company_id)
{
}

/* List of Self Tests */
static MS_HEALTH_SERVER_SELF_TEST UI_health_server_self_tests[] =
{
    {
        0x00, /* Test ID: 0x00 */
        UI_health_self_test_00
    },
    {
        0x01, /* Test ID: 0x01 */
        UI_health_self_test_01
    },
    {
        0xFF, /* Test ID: 0xFF */
        UI_health_self_test_FF
    }
};


MS_ACCESS_MODEL_HANDLE   UI_generic_onoff_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_generic_lightness_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_hsl_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_ctl_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_scene_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_server_model_handle;


/**
    \brief Health Server application Asynchronous Notification Callback.

    \par Description
    Health Server calls the registered callback to indicate events occurred to the
    application.

    \param handle        Model Handle.
    \param event_type    Health Server Event type.
    \param event_param   Parameter associated with the event if any or NULL.
    \param param_len     Size of the event parameter data. 0 if event param is NULL.
*/
static API_RESULT UI_health_server_cb
(
    MS_ACCESS_MODEL_HANDLE* handle,
    UINT8                    event_type,
    UINT8*                   event_param,
    UINT16                   param_len
)
{
    CONSOLE_OUT(
        "Health Server Callback. Not handled. Returning\n");
    return API_SUCCESS;
}

API_RESULT UI_register_foundation_model_servers
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Configuration Server */
    MS_ACCESS_MODEL_HANDLE   UI_config_server_model_handle;
    MS_ACCESS_MODEL_HANDLE   UI_health_server_model_handle;
    API_RESULT retval;
    /* Health Server */
    UINT16                       company_id;
    MS_HEALTH_SERVER_SELF_TEST* self_tests;
    UINT32                       num_self_tests;
    CONSOLE_OUT("In Model Server - Foundation Models\n");
    retval = MS_config_server_init(element_handle, &UI_config_server_model_handle);
    CONSOLE_OUT("Config Model Server Registration Status: 0x%04X\n", retval);
    /* Health Server */
    company_id = MS_DEFAULT_COMPANY_ID;
    self_tests = &UI_health_server_self_tests[0];
    num_self_tests = sizeof(UI_health_server_self_tests)/sizeof(MS_HEALTH_SERVER_SELF_TEST);
    retval = MS_health_server_init
             (
                 element_handle,
                 &UI_health_server_model_handle,
                 company_id,
                 self_tests,
                 num_self_tests,
                 UI_health_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Health Server Initialized. Model Handle: 0x%04X\n",
            UI_health_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Sensor Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

UCHAR UI_proxy_state_get(void)
{
    UCHAR  proxy_state;
    MS_proxy_fetch_state(&proxy_state);
    return proxy_state;
}

// ================= Vendor model Client model Functions  ========
void UI_vendor_model_set(UCHAR test_len,UINT16 test_index)
{
    API_RESULT retval;
    MS_NET_ADDR addr;
    UCHAR   is_prov_req;
    MS_STATE_VENDOR_EXAMPLE_TEST_STRUCT  param;
    MS_NET_ADDR    dst_addr;
    UINT8      ttl;
    is_prov_req = MS_TRUE;
    retval = MS_access_cm_get_primary_unicast_address(&addr);

    if (API_SUCCESS == retval)
    {
        if (MS_NET_ADDR_UNASSIGNED != addr)
        {
            /* Set Provisioning is not Required */
            is_prov_req = MS_FALSE;
        }
    }

    if (MS_FALSE == is_prov_req)
    {
//        CONSOLE_OUT
//        ("Send Vendor Model Set\n");
        /* Get Model Publication Address and check if valid */
        retval = MS_access_get_publish_addr
                 (
                     &UI_vendor_defined_client_model_handle,
                     &dst_addr
                 );
        ACCESS_CM_GET_DEFAULT_TTL(ttl);
        param.message_index = test_index;
        param.osal_tick = osal_sys_tick;
        param.src_addr = src_uaddr;
        param.data_len = test_len;
        CONSOLE_OUT("[PDU_Tx] Pkt.INDEX:0x%04X,SRC:0x%04X,DST:0x%04X,TTL:0x%02X\r\n",
                    param.message_index,param.src_addr,dst_addr,ttl);
        retval=UI_vendormodel_client_send_reliable_pdu(MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE,&param);
//        CONSOLE_OUT
//        ("Retval - 0x%04X\n", retval);
    }
    else
    {
        osal_stop_timerEx(bleMesh_TaskID, BLEMESH_PDU_TX_OVERRUN);
        CONSOLE_OUT
        ("An Unprovisioned Device\n");
    }
}

API_RESULT UI_access_publish
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ UINT32                    opcode,
    /* IN */ MS_NET_ADDR                 pub_addr,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len,
    /* IN */ UINT8                     reliable
)
{
    API_RESULT     retval;
    MS_NET_ADDR    dst_addr;
    printf(
        "[ACCESS] Publish. Opcode: 0x%08X. Param Len: 0x%04X. Reliable: %s\n",
        opcode, data_len, ((MS_TRUE == reliable)?"True":"False"));
    /* TODO: Validate parameters */
    dst_addr = pub_addr;
    retval = MS_access_publish_ex
             (
                 handle,
                 opcode,
                 dst_addr,
                 data_param,
                 data_len,
                 MS_FALSE
             );
    return retval;
}


API_RESULT UI_generic_onoff_client_send_reliable_pdu
(
    /* IN */ MS_NET_ADDR pub_addr,
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    retval = API_FAILURE;
    marker = 0;

    switch(req_opcode)
    {
    case MS_ACCESS_GENERIC_ONOFF_SET_OPCODE:
    {
        MS_GENERIC_ONOFF_SET_STRUCT* param_p;
        param_p = (MS_GENERIC_ONOFF_SET_STRUCT*) param;
        buffer[marker] = param_p->onoff;
        marker += 1;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_GENERIC_ONOFF_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_GENERIC_ONOFF_SET_STRUCT* param_p;
        param_p = (MS_GENERIC_ONOFF_SET_STRUCT*) param;
        buffer[marker] = param_p->onoff;
        marker += 1;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = UI_access_publish
             (
                 &UI_generic_onoff_client_model_handle,
                 req_opcode,
                 pub_addr,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );
    return retval;
}

// ================= Generic OnOff Client model Functions  ========
void UI_generic_onoff_client_set(MS_NET_ADDR pub_addr,UCHAR state)
{
    API_RESULT retval;
    MS_GENERIC_ONOFF_SET_STRUCT  param;
    CONSOLE_OUT
    ("Send Generic Onoff Set %d\n",state);
    param.onoff = state;
    param.tid = 0;
    param.optional_fields_present = 0x00;
    retval = UI_generic_onoff_client_send_reliable_pdu(pub_addr,
                                                       MS_ACCESS_GENERIC_ONOFF_SET_UNACKNOWLEDGED_OPCODE, &param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}




// ================= Generic OnOff Client model Functions  ========
void UI_generic_onoff_set(UCHAR state)
{
    API_RESULT retval;
    MS_GENERIC_ONOFF_SET_STRUCT  param;
    CONSOLE_OUT
    ("Send Generic Onoff Set %d\n",state);
    param.onoff = state;
    param.tid = 0;
    param.optional_fields_present = 0x00;
    //retval = MS_generic_onoff_set(&param);
    retval = MS_generic_onoff_set_unacknowledged(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

/**
    \brief Client Application Asynchronous Notification Callback.

    \par Description
    Generic_Onoff client calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT UI_generic_onoff_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    CONSOLE_OUT (
        "[GENERIC_ONOFF_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_GENERIC_ONOFF_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_GENERIC_ONOFF_STATUS_OPCODE\n");
    }
    break;
    }

    return retval;
}


API_RESULT UI_register_generic_onoff_model_client
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Generic OnOff Server */
    API_RESULT retval;
    CONSOLE_OUT("In Generic OnOff Model Client\n");
    retval = MS_generic_onoff_client_init
             (
                 element_handle,
                 &UI_generic_onoff_client_model_handle,
                 UI_generic_onoff_client_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Generic Onoff Client Initialized. Model Handle: 0x%04X\n",
            UI_generic_onoff_client_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Generic Onoff Client Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

API_RESULT UI_light_hsl_client_send_reliable_pdu
(
    /* IN */ MS_NET_ADDR pub_addr,
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param,
    /* IN */ UINT32    rsp_opcode
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    retval = API_FAILURE;
    marker = 0;
    MS_IGNORE_UNUSED_PARAM(rsp_opcode);

    switch(req_opcode)
    {
    case MS_ACCESS_LIGHT_HSL_DEFAULT_GET_OPCODE:
    {
    }
    break;

    case MS_ACCESS_LIGHT_HSL_DEFAULT_SET_OPCODE:
    {
        MS_LIGHT_HSL_DEFAULT_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_DEFAULT_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->lightness);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation);
        marker += 2;
    }
    break;

    case MS_ACCESS_LIGHT_HSL_DEFAULT_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_LIGHT_HSL_DEFAULT_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_DEFAULT_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->lightness);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation);
        marker += 2;
    }
    break;

    case MS_ACCESS_LIGHT_HSL_GET_OPCODE:
    {
    }
    break;

    case MS_ACCESS_LIGHT_HSL_HUE_GET_OPCODE:
    {
    }
    break;

    case MS_ACCESS_LIGHT_HSL_HUE_SET_OPCODE:
    {
        MS_LIGHT_HSL_HUE_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_HUE_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue);
        marker += 2;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_LIGHT_HSL_HUE_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_LIGHT_HSL_HUE_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_HUE_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue);
        marker += 2;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_LIGHT_HSL_RANGE_GET_OPCODE:
    {
    }
    break;

    case MS_ACCESS_LIGHT_HSL_RANGE_SET_OPCODE:
    {
        MS_LIGHT_HSL_RANGE_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_RANGE_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue_range_min);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue_range_max);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation_range_min);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation_range_max);
        marker += 2;
    }
    break;

    case MS_ACCESS_LIGHT_HSL_RANGE_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_LIGHT_HSL_RANGE_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_RANGE_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue_range_min);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hue_range_max);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation_range_min);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation_range_max);
        marker += 2;
    }
    break;

    case MS_ACCESS_LIGHT_HSL_SATURATION_GET_OPCODE:
    {
    }
    break;

    case MS_ACCESS_LIGHT_HSL_SATURATION_SET_OPCODE:
    {
        MS_LIGHT_HSL_SATURATION_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_SATURATION_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation);
        marker += 2;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_LIGHT_HSL_SATURATION_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_LIGHT_HSL_SATURATION_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_SATURATION_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->saturation);
        marker += 2;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_LIGHT_HSL_SET_OPCODE:
    {
        MS_LIGHT_HSL_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hsl_lightness);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hsl_hue);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hsl_saturation);
        marker += 2;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_LIGHT_HSL_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_LIGHT_HSL_SET_STRUCT* param_p;
        param_p = (MS_LIGHT_HSL_SET_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hsl_lightness);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hsl_hue);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->hsl_saturation);
        marker += 2;
        buffer[marker] = param_p->tid;
        marker += 1;

        if (0x00 != param_p->optional_fields_present)
        {
            buffer[marker] = param_p->transition_time;
            marker += 1;
            buffer[marker] = param_p->delay;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_LIGHT_HSL_TARGET_GET_OPCODE:
    {
    }
    break;
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = UI_access_publish
             (
                 &UI_light_hsl_client_model_handle,
                 req_opcode,
                 pub_addr,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );
    return retval;
}

// ================= Light Hsl Client model Functions  ========
void UI_light_hsl_client_set(MS_NET_ADDR pub_addr,UINT16 lightness, UINT16 hue, UINT16 saturation)
{
    API_RESULT retval;
    MS_LIGHT_HSL_SET_STRUCT  param;
    CONSOLE_OUT
    ("Send Light HSL Set\n");
    param.hsl_lightness = lightness;
    param.hsl_hue       = hue;
    param.hsl_saturation = saturation;
    param.tid = 0;
    param.optional_fields_present = 0x00;
    retval = UI_light_hsl_client_send_reliable_pdu
             (
                 pub_addr,
                 MS_ACCESS_LIGHT_HSL_SET_OPCODE,
                 &param,
                 MS_ACCESS_LIGHT_HSL_STATUS_OPCODE
             );
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}



// ================= Light Hsl Client model Functions  ========
void UI_light_hsl_set(UINT16 lightness, UINT16 hue, UINT16 saturation)
{
    API_RESULT retval;
    MS_LIGHT_HSL_SET_STRUCT  param;
    CONSOLE_OUT
    ("Send Light HSL Set\n");
    param.hsl_lightness = lightness;
    param.hsl_hue       = hue;
    param.hsl_saturation = saturation;
    param.tid = 0;
    param.optional_fields_present = 0x00;
    retval = MS_light_hsl_set(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

/**
    Light Hsl Client application Asynchronous Notification Callback.

    Light Hsl Client calls the registered callback to indicate events occurred to the
    application.

    \param handle        Model Handle.
    \param opcode        Opcode.
    \param data_param    Data associated with the event if any or NULL.
    \param data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT UI_light_hsl_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    CONSOLE_OUT (
        "[LIGHT_HSL_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);
//    switch(opcode)
//    {
//        case MS_ACCESS_LIGHT_HSL_DEFAULT_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_LIGHT_HSL_DEFAULT_STATUS_OPCODE\n");
//        }
//        break;
//        case MS_ACCESS_LIGHT_HSL_HUE_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_LIGHT_HSL_HUE_STATUS_OPCODE\n");
//        }
//        break;
//        case MS_ACCESS_LIGHT_HSL_RANGE_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_LIGHT_HSL_RANGE_STATUS_OPCODE\n");
//        }
//        break;
//        case MS_ACCESS_LIGHT_HSL_SATURATION_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_LIGHT_HSL_SATURATION_STATUS_OPCODE\n");
//        }
//        break;
//        case MS_ACCESS_LIGHT_HSL_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_LIGHT_HSL_STATUS_OPCODE\n");
//        }
//        break;
//        case MS_ACCESS_LIGHT_HSL_TARGET_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_LIGHT_HSL_TARGET_STATUS_OPCODE\n");
//        }
//        break;
//    }
    return retval;
}


API_RESULT UI_register_light_hsl_model_client
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Generic OnOff Server */
    API_RESULT retval;
    CONSOLE_OUT("In Light HSL Model Client\n");
    retval = MS_light_hsl_client_init
             (
                 element_handle,
                 &UI_light_hsl_client_model_handle,
                 UI_light_hsl_client_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Light HSL Client Initialized. Model Handle: 0x%04X\n",
            UI_light_hsl_client_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Light HSL Client Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

// ================= Light Ctl Client model Functions  ========
void UI_light_ctl_set(UINT16 lightness, UINT16 temperature, UINT16 deltaUv)
{
    API_RESULT retval;
    MS_LIGHT_CTL_SET_STRUCT  param;
    CONSOLE_OUT
    ("Send Light CTL Set\n");
    param.ctl_lightness = lightness;
    param.ctl_temperature = temperature;
    param.ctl_delta_uv = deltaUv;
    param.tid = 0;
    param.optional_fields_present = 0x00;
    retval = MS_light_ctl_set(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

/**
    Light Ctl Client application Asynchronous Notification Callback.

    Light Ctl Client calls the registered callback to indicate events occurred to the
    application.

    \param handle        Model Handle.
    \param opcode        Opcode.
    \param data_param    Data associated with the event if any or NULL.
    \param data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT UI_light_ctl_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    CONSOLE_OUT (
        "[LIGHT_CTL_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);
    return retval;
}


API_RESULT UI_register_light_ctl_model_client
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Generic OnOff Server */
    API_RESULT retval;
    CONSOLE_OUT("In Light CTL Model Client\n");
    retval = MS_light_ctl_client_init
             (
                 element_handle,
                 &UI_light_ctl_client_model_handle,
                 UI_light_ctl_client_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Light CTL Client Initialized. Model Handle: 0x%04X\n",
            UI_light_ctl_client_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Light CTL Client Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

// ================= Scene Client model Functions  ========
void UI_scene_store(UINT16 scene_number)
{
    API_RESULT retval;
    MS_SCENE_STRUCT  param;
    CONSOLE_OUT
    ("Send Scene Store\n");
    param.scene_number       = scene_number;
    retval = MS_scene_store(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

void UI_scene_delete(UINT16 scene_number)
{
    API_RESULT retval;
    MS_SCENE_STRUCT  param;
    CONSOLE_OUT
    ("Send Scene Delete\n");
    param.scene_number       = scene_number;
    retval = MS_scene_delete(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

void UI_scene_recall(UINT16 scene_number)
{
    API_RESULT retval;
    MS_SCENE_RECALL_STRUCT  param;
    CONSOLE_OUT
    ("Send Scene Recall\n");
    param.scene_number       = scene_number;
    param.tid                = 0;                      // temp set
    param.optional_fields_present = 0x00;
    retval = MS_scene_recall(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

/**
    \brief Client Application Asynchronous Notification Callback.

    \par Description
    Scene client calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT UI_scene_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    CONSOLE_OUT (
        "[SCENE_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);
//    switch(opcode)
//    {
//        case MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE\n");
//        }
//        break;
//        case MS_ACCESS_SCENE_STATUS_OPCODE:
//        {
//            CONSOLE_OUT(
//            "MS_ACCESS_SCENE_STATUS_OPCODE\n");
//        }
//        break;
//    }
    return retval;
}


API_RESULT UI_register_scene_model_client
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    API_RESULT retval;
    CONSOLE_OUT("In Scene Model Client\n");
    retval = MS_scene_client_init
             (
                 element_handle,
                 &UI_scene_client_model_handle,
                 UI_scene_client_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Scene Model Client Initialized. Model Handle: 0x%04X\n",
            UI_scene_client_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Scene Model Client Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

/**
    \brief API to send acknowledged commands

    \par Description
    This is to initialize sending acknowledged commands.

    \param [in] req_opcode    Request Opcode.
    \param [in] param         Parameter associated with Request Opcode.
    \param [in] rsp_opcode    Response Opcode.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT UI_vendormodel_client_send_reliable_pdu
(
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    retval = API_FAILURE;
    marker = 0;
    printf(
        "[VENDOR_MODEL_CLIENT] Send Reliable PDU. Req Opcode 0x%08X\n",
        req_opcode);

    switch(req_opcode)
    {
    case MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE:
    {
        MS_STATE_VENDOR_EXAMPLE_STRUCT* param_p;
        printf(
            "MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE\n");
        param_p = (MS_STATE_VENDOR_EXAMPLE_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->maun_opcode);
        marker += 2;
        buffer[marker] = param_p->value;
        marker += 1;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE:
    {
        MS_STATE_VENDOR_EXAMPLE_TEST_STRUCT* param_p;
//            printf(
//            "MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE\n");
        param_p = (MS_STATE_VENDOR_EXAMPLE_TEST_STRUCT*) param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->message_index);
        marker += 2;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->osal_tick);
        marker += 4;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->src_addr);
        marker += 2;

        if(param_p->data_len)
        {
            EM_mem_set(&buffer[marker], 0, param_p->data_len);
            marker += param_p->data_len;
        }
    }
    break;

    default:
        break;
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = MS_access_publish
             (
                 &UI_vendor_defined_client_model_handle,
                 req_opcode,
                 pdu_ptr,
                 marker,
                 MS_FALSE
             );
    return retval;
}
/* Vendor Defined Model Get Handler */
extern MS_NET_ADDR bleMesh_gkey_get(uint8 indx);
extern void bleMesh_gkey_set(uint8 indx, MS_NET_ADDR  addr);
void UI_vendor_example_model_state_get(UINT16 state_t, UINT16 state_inst, void* param, UINT8 coun)
{
    switch ( state_t )
    {
    case MS_STATE_VENDOR_CTRL_T:
    {
        UINT16 rslt = bleMesh_gkey_get(*(UINT8*)((UINT8*)param+0));
        memcpy(((UINT8*)param+1), &rslt, sizeof(UINT16));
    }
    break;
    }
}

/* Vendor Defined Model Set Handler */
void UI_vendor_example_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 coun)
{
    switch( state_t )
    {
    case MS_STATE_VENDOR_CTRL_T:
    {
        if ( 3 == coun && NULL != param)
        {
            bleMesh_gkey_set(*(UINT8*)((UINT8*)param+0), *(UINT16*)((UINT8*)param+1));
        }

        // switch( param ) {
        //     case 0x00: {    // group key ->
        //     }
        //     break;
        //     case 0x01:  // group key ->
        //         break;
        //     case 0x02:  // group key ->
        //         break;
        //     case 0x03:  // group key ->
        //         break;
        //     default:
        //         break;
        // }
    }
    break;

    default:
        break;
    }
}

/**
    \brief Access Layer Application Asynchronous Notification Callback.

    \par Description
    Access Layer calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] appkey_handle AppKey Handle.
    \param [in] subnet_handle Subnet Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT vendor_example_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT         req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW             req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T               req_type;
    MS_ACCESS_MODEL_EXT_PARAMS*               ext_params_p;
    MS_ACCESS_VENDOR_MODEL_STATE_PARAMS     state_params;
    API_RESULT    retval;
    retval = API_SUCCESS;
    ext_params_p = NULL;
    /* Request Context */
    req_context.handle = *handle;
    req_context.saddr  = saddr;
    req_context.daddr  = daddr;
    req_context.subnet_handle = subnet_handle;
    req_context.appkey_handle = appkey_handle;
    /* Request Raw */
    req_raw.opcode = opcode;
    req_raw.data_param = data_param;
    req_raw.data_len = data_len;
    CONSOLE_OUT(
        "[VENDOR_EXAMPLE_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(vendor_example_get_handler);
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = *data_param|(*(data_param+1)<<8);
        state_params.state = data_param+2;
        state_params.state_coun = data_len-2;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(vendor_example_set_handler);
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x00;
        /* Assign reqeusted state type to the application */
        state_params.state_type = *data_param|(*(data_param+1)<<8);
        state_params.state = data_param+2;
        state_params.state_coun = data_len-2;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_TRANSPARENT_MSG\n");
        MODEL_OPCODE_HANDLER_CALL(vendor_example_transparent_msg_handler);
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_OTHERS;
        req_type.to_be_acked = 0x00;
    }
    break;

    default:
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_NONE_OPCODE\n");
        break;
    }

    /* Application callback */
    if (NULL != vendor_example_client_UI_cb)
    {
        vendor_example_client_UI_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
    }

    return retval;
}


/**
    \brief API to initialize Vendor_Example_1 Server model

    \par Description
    This is to initialize Vendor_Example_1 Server model and to register with Acess layer.

    \param [in] element_handle
    Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] UI_cb    Application Callback to be used by the Vendor_Example_1 Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_vendor_example_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_VENDOR_EXAMPLE_CLIENT_CB UI_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    CONSOLE_OUT(
        "[VENDOR_EXAMPLE] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_VENDOR_EXAMPLE_CLIENT;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
    model.elem_handle = element_handle;
    /* Register Callback */
    model.cb = vendor_example_client_cb;
    /* List of Opcodes */
    model.opcodes = vendor_example_client_opcode_list;
    model.num_opcodes = sizeof(vendor_example_client_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    vendor_example_client_UI_cb = UI_cb;
    //    /* TODO: Remove */
    //    vendor_example_server_model_handle = *model_handle;
    return retval;
}

/* Vendor Defined Model Server */
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Vendor_Example_1 server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
API_RESULT UI_vendor_example_client_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*          ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*              msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*                req_type,
    /* IN */ MS_ACCESS_VENDOR_MODEL_STATE_PARAMS*      state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*               ext_params
)
{
    UINT8 para[32];
    // MS_STATE_VENDOR_EXAMPLE_STRUCT      param;
    MS_ACCESS_VENDOR_MODEL_STATE_PARAMS current_state_params;
    API_RESULT retval;
    retval = API_SUCCESS;
    CONSOLE_OUT("[VENDOR_EXAMPLE] %04x (d%04x s%04x).\n",
                ctx->handle,ctx->daddr,ctx->saddr);

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT("[VENDOR_EXAMPLE] GET Request.\n");
        memcpy(para, msg_raw->data_param, msg_raw->data_len);
        UI_vendor_example_model_state_get(
            state_params->state_type,
            0, para+2, sizeof(para)-2);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = para;
        current_state_params.state_coun = msg_raw->data_len + 2;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] SET Request.\n");
        UI_vendor_example_model_state_set(state_params->state_type, 0, state_params->state, 3);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = msg_raw->data_param;
        current_state_params.state_coun = msg_raw->data_len;
    }

    /* See if to be acknowledged */
    // if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        // retval = MS_vendor_example_client_state_update(ctx, &current_state_params, NULL, 0, NULL);
        retval = MS_access_reply
                 (
                     &ctx->handle,
                     ctx->daddr,
                     ctx->saddr,
                     ctx->subnet_handle,
                     ctx->appkey_handle,
                     ACCESS_INVALID_DEFAULT_TTL,
                     msg_raw->opcode,
                     current_state_params.state,
                     current_state_params.state_coun
                 );
    }
    return retval;
}



API_RESULT UI_register_vendor_defined_model_client
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Vendor Defined Server */
    API_RESULT retval;
    retval = MS_vendor_example_client_init
             (
                 element_handle,
                 &UI_vendor_defined_client_model_handle,
                 UI_vendor_example_client_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Vendor Defined Server Initialized. Model Handle: 0x%04X\n",
            UI_vendor_defined_client_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Vendor Defined Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

#ifdef MS_FRIEND_SUPPORT
// ==============

void UI_frndsetup_cb(MS_SUBNET_HANDLE subnet, UCHAR event_type, UINT16 status)
{
    API_RESULT retval;
    UINT16 num_subaddr;
    UINT16 subaddr[5];
    CONSOLE_OUT("\nFriendship Event 0x%02X on Subnet 0x%04X - 0x%04X\n",
                event_type, subnet, status);

    switch (event_type)
    {
    case MS_TRN_FRIEND_SETUP_CNF:
        CONSOLE_OUT("Recvd MS_TRN_FRIEND_SETUP_CNF - 0x%04X\n", status);

        if (API_SUCCESS == status)
        {
            /* Get the subscription list */
            num_subaddr = sizeof(subaddr) / sizeof(UINT16);
            MS_access_cm_get_all_model_subscription_list(&num_subaddr, subaddr);

            if (0 < num_subaddr)
            {
                CONSOLE_OUT("Initiating FriendSubscriptionListAdd - %d addr\n", num_subaddr);
                retval = MS_trn_lpn_subscrn_list_add(subaddr, num_subaddr);
                CONSOLE_OUT("Retval - 0x%04X\n", retval);
            }

            if (enable_sleep_flag != 1)
            {
                printf("======= Friendship established, enable sleep\r\n");
                hal_pwrmgr_unlock(MOD_USR1);
                enable_sleep_flag = 1;
            }
        }
        else
        {
            CONSOLE_OUT("Friendship Setup Failure%04X\n", status);
            UI_lpn_seek_friend();
        }

        break;

    case MS_TRN_FRIEND_SUBSCRNLIST_CNF:
        CONSOLE_OUT("Recvd MS_TRN_FRIEND_SUBSCRNLIST_CNF - 0x%04X\n", status);
        break;

    case MS_TRN_FRIEND_CLEAR_CNF:
        CONSOLE_OUT("Recvd MS_TRN_FRIEND_CLEAR_CNF - 0x%04X\n", status);
        hal_pwrmgr_lock(MOD_USR1);
//        disableSleep();
        enable_sleep_flag = 0;
        break;

    case MS_TRN_FRIEND_TERMINATE_IND:
        CONSOLE_OUT("Recvd MS_TRN_FRIEND_TERMINATE_IND - 0x%04X\n", status);
        printf("======= Friendship terminate, disable sleep\r\n");
        hal_pwrmgr_lock(MOD_USR1);
//            disableSleep();
        enable_sleep_flag = 0;
        /* Enable Friend feature */
//            MS_ENABLE_FRIEND_FEATURE();
        break;

    default:
        break;
    }
}

void UI_lpn_seek_friend (void)
{
    API_RESULT retval;
    blebrr_scan_pl(FALSE);
    /* Disable Friend feature */
//    MS_DISABLE_FRIEND_FEATURE();
    /* Enable LPN feature */
//    MS_ENABLE_LPN_FEATURE();
    CONSOLE_OUT ("Requesting for friendship...\n");
    retval = MS_trn_lpn_setup_friendship
             (
                 0x00,
                 UI_FRND_CRITERIA,
                 UI_FRND_RECEIVE_DELAY_MS,
                 UI_FRND_POLLTIMEOUT_100MS,
                 UI_FRND_SETUPTIMEOUT,
                 UI_frndsetup_cb
             );
    CONSOLE_OUT ("Retval - 0x%04X\n", retval);
    return;
}
#endif
/* Provisionee */
#define UI_PROV_OUTPUT_OOB_ACTIONS            0x00
/** Public Key OOB Flag */
#define UI_PROV_PUBKEY_OOBINFO                0x00

/** Static OOB Flag */
#define UI_PROV_STATIC_OOBINFO                0x00



/** Output OOB Maximum size supported */
#define UI_PROV_OUTPUT_OOB_SIZE               0x00

/** Input OOB Actions supported */
#define UI_PROV_INPUT_OOB_ACTIONS             0x00

/** Input OOB Maximum size supported */
#define UI_PROV_INPUT_OOB_SIZE                0x00

/** Beacon setup timeout in seconds */
#define UI_PROV_SETUP_TIMEOUT_SECS            30

/** Beacon interleave in mseconds */
#define UI_PROV_SETUP_GATT_MSECS                200
#define UI_PROV_SETUP_ADV_MSECS                 200


/** Attention timeout for device in seconds */
#define UI_PROV_DEVICE_ATTENTION_TIMEOUT      30

#define PROV_AUTHVAL_SIZE_PL                  16

/** Authentication values for OOB Display - To be made random */
#define UI_DISPLAY_AUTH_DIGIT                 3
#define UI_DISPLAY_AUTH_NUMERIC               35007
#define UI_DISPLAY_AUTH_STRING                "F00L"

/** Provisioning capabilities of local device */
DECL_STATIC PROV_CAPABILITIES_S UI_prov_capab =
{
    /** Number of Elements */
    0x01,

    /** Supported algorithms */
    PROV_MASK_ALGO_EC_FIPS_P256,

    /** Public key type */
    UI_PROV_PUBKEY_OOBINFO,

    /** Static OOB type */
    UI_PROV_STATIC_OOBINFO,

    /** Output OOB information */
    { UI_PROV_OUTPUT_OOB_ACTIONS, UI_PROV_OUTPUT_OOB_SIZE },

    /** Input OOB information */
    { UI_PROV_INPUT_OOB_ACTIONS, UI_PROV_INPUT_OOB_SIZE },
};

/** Unprovisioned device identifier */
PROV_DEVICE_S UI_lprov_device =
{
    /** UUID */
    {0x05, 0x04, 0x62, 0x12, 0x01, 0x00, 0x00, 0x01, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00},

    /** OOB Flag */
    0x00,

    /**
        Encoded URI Information
        For example, to give a web address, "https://www.abc.com"
        the URI encoded data would be -
        0x17 0x2F 0x2F 0x77 0x77 0x77 0x2E 0x61 0x62 0x63 0x2E 0x63 0x6F 0x6D
        where 0x17 is the URI encoding for https:
    */
    NULL
};

/** Current role of application - Provisioner/Device */
DECL_STATIC UCHAR UI_prov_role;

/** Current brr of provision - Provisioner/Device */
DECL_STATIC UCHAR UI_prov_brr_handle;


void UI_provcfg_complete_timeout_handler(void* args, UINT16 size)
{
    procfg_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    MS_common_reset();
    EM_start_timer (&thandle, 1, timeout_cb, NULL, 0);
    printf("Provisining and config Complete Timeout\n");
}
static API_RESULT UI_prov_callback
(
    PROV_HANDLE* phandle,
    UCHAR         event_type,
    API_RESULT    event_result,
    void*         event_data,
    UINT16        event_datalen
)
{
//    PROV_DEVICE_S * rdev;
//    PROV_CAPABILITIES_S * rcap;
    PROV_DATA_S* rdata;
    PROV_OOB_TYPE_S* oob_info;
    API_RESULT retval;
    UCHAR authstr[PROV_AUTHVAL_SIZE_PL << 1];
    UINT32 authnum;
    UCHAR authtype;
    UCHAR* pauth;
    UINT16 authsize;
//    UCHAR  pdata[(MS_DEVICE_UUID_SIZE * 2) + 1];
//    UCHAR  * t_data;

    switch (event_type)
    {
    case PROV_EVT_PROVISIONING_SETUP:
        CONSOLE_OUT("Recvd PROV_EVT_PROVISIONING_SETUP\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);
        /* Display the attention timeout */
        CONSOLE_OUT("Attention TImeout - %d\n", *((UCHAR*)event_data));
        LIGHT_ONLY_BLUE_ON;
        blebrr_prov_started = MS_TRUE;
        break;

    case PROV_EVT_OOB_DISPLAY:
        CONSOLE_OUT("Recvd PROV_EVT_OOB_DISPLAY\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);
        /* Reference the Authenticatio Type information */
        oob_info = (PROV_OOB_TYPE_S*)event_data;
        CONSOLE_OUT("Authenticaion Action - 0x%02X\n", oob_info->action);
        CONSOLE_OUT("Authenticaion Size - 0x%02X\n", oob_info->size);

        /* If role is Device, the action is of Output OOB, else Input OOB */
        if (PROV_ROLE_DEVICE == UI_prov_role)
        {
            if (PROV_OOOB_ACTION_ALPHANUMERIC == oob_info->action)
            {
                authtype = 1;
            }
            else if (PROV_OOOB_ACTION_NUMERIC == oob_info->action)
            {
                authtype = 2;
            }
            else
            {
                authtype = 0;
            }
        }
        else
        {
            if (PROV_IOOB_ACTION_ALPHANUMERIC == oob_info->action)
            {
                authtype = 1;
            }
            else if (PROV_IOOB_ACTION_NUMERIC == oob_info->action)
            {
                authtype = 2;
            }
            else
            {
                authtype = 0;
            }
        }

        if (1 == authtype)
        {
            EM_str_copy (authstr, UI_DISPLAY_AUTH_STRING);
            CONSOLE_OUT("\n\n>>> AuthVal - %s <<<\n\n", authstr);
            pauth = authstr;
            authsize = (UINT16)EM_str_len(authstr);
        }
        else if (2 == authtype)
        {
            authnum = (UINT32)UI_DISPLAY_AUTH_NUMERIC;
            CONSOLE_OUT("\n\n>>> AuthVal - %d <<<\n\n", authnum);
            pauth = (UCHAR*)&authnum;
            authsize = sizeof(UINT32);
        }
        else
        {
            authnum = (UINT32)UI_DISPLAY_AUTH_DIGIT;
            CONSOLE_OUT("\n\n>>> AuthVal - %d <<<\n\n", authnum);
            pauth = (UCHAR*)&authnum;
            authsize = sizeof(UINT32);
        }

        /* Call to input the oob */
        CONSOLE_OUT("Setting the Authval...\n");
        retval = MS_prov_set_authval(&UI_prov_brr_handle, pauth, authsize);
        CONSOLE_OUT("Retval - 0x%04X\n", retval);
        break;

    case PROV_EVT_OOB_ENTRY:
        CONSOLE_OUT("Recvd PROV_EVT_OOB_ENTRY\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);
        /* Reference the Authenticatio Type information */
        oob_info = (PROV_OOB_TYPE_S*)event_data;
        CONSOLE_OUT("Authenticaion Action - 0x%02X\n", oob_info->action);
        CONSOLE_OUT("Authenticaion Size - 0x%02X\n", oob_info->size);
        break;

    case PROV_EVT_DEVINPUT_COMPLETE:
        CONSOLE_OUT("Recvd PROV_EVT_DEVINPUT_COMPLETE\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);
        break;

    case PROV_EVT_PROVDATA_INFO:
        CONSOLE_OUT("Recvd PROV_EVT_PROVDATA_INFO\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);
        /* Reference the Provisioning Data */
        rdata = (PROV_DATA_S*)event_data;
        CONSOLE_OUT("NetKey  : ");
        appl_dump_bytes(rdata->netkey, PROV_KEY_NETKEY_SIZE);
        CONSOLE_OUT("Key ID  : 0x%04X\n", rdata->keyid);
        CONSOLE_OUT("Flags   : 0x%02X\n", rdata->flags);
        CONSOLE_OUT("IVIndex : 0x%08X\n", rdata->ivindex);
        CONSOLE_OUT("UAddr   : 0x%04X\n", rdata->uaddr);
        /* Provide Provisioning Data to Access Layer */
        MS_access_cm_set_prov_data
        (
            rdata
        );
        break;

    case PROV_EVT_PROVISIONING_COMPLETE:
        CONSOLE_OUT("Recvd PROV_EVT_PROVISIONING_COMPLETE\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);

        if (API_SUCCESS == event_result)
        {
            /* Already Set while handling PROV_EVT_PROVDATA_INFO */
            light_blink_set(LIGHT_GREEN, LIGHT_BLINK_SLOW,3);
            UI_sample_get_device_key();

            if(*phandle == PROV_BRR_GATT)
            {
                MS_ENABLE_PROXY_FEATURE();
                osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_TERMINATE, 3000); //add gap terminate evt by hq
            }

            #ifdef EASY_BOUNDING
            procfg_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
            CONSOLE_OUT("Start timer\n");
            retval = EM_start_timer
                     (
                         &procfg_timer_handle,
                         PROCFG_COMPLETE_TIMEOUT,
                         UI_provcfg_complete_timeout_handler,
                         NULL,
                         0
                     );
            #endif
        }
        else
        {
            light_blink_set(LIGHT_RED, LIGHT_BLINK_SLOW,3);
            MS_common_reset();
            EM_start_timer (&thandle, 1, timeout_cb, NULL, 0);
        }

        break;

    default:
        CONSOLE_OUT("Unknown Event - 0x%02X\n", event_type);
    }

    return API_SUCCESS;
}

static void UI_register_prov(void)
{
    API_RESULT retval;
    CONSOLE_OUT("Registering with Provisioning layer...\n");
    retval = MS_prov_register(&UI_prov_capab, UI_prov_callback);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}

static void UI_prov_bind(UCHAR brr, UCHAR index)
{
    API_RESULT retval;
    /* Call to bind with the selected device */
    CONSOLE_OUT("Binding with the selected device...\n");
    retval = MS_prov_bind(brr, &UI_lprov_device, UI_PROV_DEVICE_ATTENTION_TIMEOUT, &UI_prov_brr_handle);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}



static void UI_setup_prov(UCHAR role, UCHAR brr)
{
    API_RESULT retval;

    if (PROV_BRR_GATT & brr)
    {
        blebrr_gatt_mode_set(BLEBRR_GATT_PROV_MODE);
    }

    if (PROV_ROLE_PROVISIONER != role)
    {
        CONSOLE_OUT("Setting up Device for Provisioning ...\n");
        retval = MS_prov_setup
                 (
                     brr,
                     role,
                     &UI_lprov_device,
                     UI_PROV_SETUP_GATT_MSECS,
                     UI_PROV_SETUP_ADV_MSECS
                 );
        UI_prov_role = PROV_ROLE_DEVICE;
    }
    else
    {
        CONSOLE_OUT("Setting up Provisioner for Provisioning ...\n");
        retval = MS_prov_setup
                 (
                     brr,
                     role,
                     NULL,
                     UI_PROV_SETUP_TIMEOUT_SECS,
                     UI_PROV_SETUP_TIMEOUT_SECS
                 );
        UI_prov_role = PROV_ROLE_PROVISIONER;
    }

    if (PROV_BRR_GATT != brr)
    {
        UI_prov_brr_handle = PROV_BRR_ADV;
        UI_prov_bind(brr & PROV_BRR_ADV, 0x00);
    }

    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}




void UI_proxy_start_adv(MS_SUBNET_HANDLE subnet_handle, UCHAR proxy_adv_mode)
{
    #ifdef MS_PROXY_SERVER
    API_RESULT retval;
    DECL_STATIC UINT8 first_time = 0;

    if (0 == first_time)
    {
        /**
            Register with Proxy Module as Device is going to be a Proxy.
            This is typically a one-time-event, and hence registering the
            PROXY when Proxy ADV is being initiated!
        */
        UI_register_proxy();
        first_time = 1;
    }

    /* Set the role to Proxy with bearer */
    blebrr_gatt_mode_set(BLEBRR_GATT_PROXY_MODE);
    CONSOLE_OUT("Start Proxy Advertisements with %s for Subnet 0x%04X\n",
                (proxy_adv_mode == MS_PROXY_NET_ID_ADV_MODE) ? "Network ID" : "Node Identity",
                subnet_handle);
    retval = MS_proxy_server_adv_start
             (
                 subnet_handle,
                 proxy_adv_mode
             );
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
    #else /* MS_PROXY_SERVER */
    CONSOLE_OUT("\n [** ERR **] MS_PROXY_SERVER feature is DISABLED!\n");
    return;
    #endif /* MS_PROXY_SERVER */
}

void UI_proxy_callback
(
    NETIF_HANDLE*        handle,
    UCHAR                p_evt,
    UCHAR*               data_param,
    UINT16               data_len
)
{
    UCHAR             role;
    MS_IGNORE_UNUSED_PARAM(data_len);

    switch(p_evt)
    {
    case MS_PROXY_UP_EVENT:
        CONSOLE_OUT(
            "\n\n[PROXY APPL]: MS_PROXY_UP_EVENT Received for NETIF Handle 0x%02X\n\n", *handle);

        if (NULL != data_param)
        {
            /* Catch the current role into a local */
            role = data_param[0];

            if (BRR_SERVER_ROLE == role)
            {
                /* Send Secure Network Beacons */
                /* MS_net_broadcast_secure_beacon(0x0000); */
            }
        }

        break;

    case MS_PROXY_DOWN_EVENT:
        CONSOLE_OUT(
            "\n\n[PROXY APPL]: MS_PROXY_DOWN_EVENT Received for NETIF Handle 0x%02X\n\n", *handle);
        break;

    default:
        CONSOLE_OUT(
            "\n\n[PROXY APPL ERR]: Unknown Event Received for NETIF Handle 0x%02X!!\n\n", *handle);
        break;
    }
}

void UI_register_proxy(void)
{
    API_RESULT retval;
    CONSOLE_OUT("Registering with Proxy layer...\n");
    retval =  MS_proxy_register(UI_proxy_callback);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}

API_RESULT UI_set_brr_scan_rsp_data (void)
{
    /**
        Currently setting MT-MESH-SAMPLE-10a as Complete Device Name!
        This can be updated to each individual devices as per requirement.
    */
    UCHAR UI_brr_scanrsp_data[] =
    {
        /**
            Shortened Device Name: MT-MESH-SAMPLE-10a
        */
        0x0b, 0x09, 'P', 'H', 'Y', '-', 'S', 'W', 'I', 'T', 'C', 'H'
    };
    CONSOLE_OUT("\n Setting PHY-SWITCH as Complete Device Name!\n");
    /* Set the Scan Response Data at the Bearer Layer */
    blebrr_set_adv_scanrsp_data_pl
    (
        UI_brr_scanrsp_data,
        sizeof(UI_brr_scanrsp_data)
    );
    return API_SUCCESS;
}

void timeout_cb (void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    UI_sample_reinit();
}

void UI_gatt_iface_event_pl_cb
(
    UCHAR  ev_name,
    UCHAR  ev_param
)
{
    switch(ev_name)
    {
    /* GATT Bearer BLE Link Layer Disconnected */
    case BLEBRR_GATT_IFACE_DOWN:
        CONSOLE_OUT("\r\n >> GATT Bearer BLE Link Layer Disconnection Event Received!\r\n");
        osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_TERMINATE);

        //UI_sample_reinit();
        if(thandle == EM_TIMER_HANDLE_INIT_VAL)
        {
            EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
        }

        break;

    /* GATT Bearer BLE Link Layer Connected */
    case BLEBRR_GATT_IFACE_UP:
        CONSOLE_OUT("\r\n >> GATT Bearer BLE Link Layer Connection Event Received!\r\n");

        /* Do Nothing! */
        if (BLEBRR_GATT_PROV_MODE == blebrr_gatt_mode_get())
        {
            MS_prov_stop_interleave_timer();
            MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);
            MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_PASSIVE);
        }
        else if (BLEBRR_GATT_PROXY_MODE == blebrr_gatt_mode_get())
        {
            MS_proxy_server_adv_stop();
        }

        break;

    case BLEBRR_GATT_IFACE_ENABLE:
        CONSOLE_OUT("\r\n >> GATT Bearer Active Event Received!\r\n");
        {
            if (BLEBRR_GATT_PROV_MODE == ev_param)
            {
                /* Call to bind with the selected device */
                UI_prov_brr_handle = PROV_BRR_GATT;
                UI_prov_bind(PROV_BRR_GATT, 0);
            }
        }
        break;

    case BLEBRR_GATT_IFACE_DISABLE:
        CONSOLE_OUT("\r\n >> GATT Bearer Inactive Event Received!\r\n");
        break;

    /* Unknown Event! */
    default:
        CONSOLE_OUT("\r\n >> GATT Bearer BLE Link Layer Unknown Event 0x%02X Received!\r\n", ev_name);
        /* Do Nothing! */
        break;
    }
}

API_RESULT ms_access_get_publish_addr
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ MS_NET_ADDR*              publish_addr
);

//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
API_RESULT UI_sample_binding_app_key(void)
{
    MS_APPKEY_HANDLE  handle;
    UINT8*              key;
    UINT8             aid;
    DECL_CONST UINT8  t_key[16] = {0};
    API_RESULT retval;
    CONSOLE_OUT("Fetching App Key for Handle 0x0000\n");
    handle = 0x0000;
    retval = MS_access_cm_get_app_key
             (
                 handle,
                 &key,
                 &aid
             );

    /* Check Retval. Print App Key */
    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT("App Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    handle, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                    key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

        if (0 == EM_mem_cmp(key, t_key, 16))
        {
            /* NO AppKey Bound */
            retval = API_FAILURE;
        }
        else
        {
            /* Found a Valid App Key */
            /* Keeping the retval as API_SUCCESS */
            retval=MS_access_bind_model_app(UI_generic_onoff_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_generic_onoff_client_model_handle,handle);
            #ifdef  USE_LIGHTNESS
            retval=MS_access_bind_model_app(UI_generic_lightness_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_generic_lightness_client_model_handle,handle);
            #endif
            #ifdef  USE_CTL
            retval=MS_access_bind_model_app(UI_light_ctl_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_light_ctl_client_model_handle,handle);
            #endif
            #ifdef  USE_HSL
            retval=MS_access_bind_model_app(UI_light_hsl_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_light_hsl_client_model_handle,handle);
            #endif
            #ifdef  USE_SCENE
            retval=MS_access_bind_model_app(UI_scene_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_scene_client_model_handle,handle);
            #endif
            #ifdef  USE_VENDORMODEL
            retval=MS_access_bind_model_app(UI_vendor_defined_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_vendor_defined_client_model_handle,handle);
            #endif
        }
    }

    //Provision ok,stop provision/config timeout handler  by hq
    CONSOLE_OUT("Stop timer\n");
    EM_stop_timer(&procfg_timer_handle);
    blebrr_prov_started = MS_FALSE;
    return retval;
}

void vm_subscriptiong_binding_cb (void)
{
//    UCHAR   proxy_state;
    UINT8 relay;
    MS_access_ps_store_disable(MS_TRUE);
    CONSOLE_OUT("vm_subscriptiong_binding_cb\n");
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    UI_sample_binding_app_key();
    MS_DISABLE_RELAY_FEATURE();
    MS_ENABLE_PROXY_FEATURE();
    MS_DISABLE_FRIEND_FEATURE();
    relay = MS_access_cm_get_features_field(&relay, MS_FEATURE_RELAY);

    if(relay == MS_TRUE)
    {
        MS_ENABLE_SNB_FEATURE();
        MS_net_start_snb_timer(0);
    }
    else
    {
        MS_DISABLE_SNB_FEATURE();
        MS_net_stop_snb_timer(0);
    }

    MS_access_cm_set_transmit_state(MS_RELAY_TX_STATE, (0<<3)|1);
    MS_access_ps_store_disable(MS_FALSE);
    MS_access_cm_set_transmit_state(MS_NETWORK_TX_STATE, (0<<3)|0);
//    if(UI_prov_brr_handle == PROV_BRR_ADV)
//    {
//        EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
//    }
//    else if(UI_prov_brr_handle == PROV_BRR_GATT)
//    {
//        MS_proxy_fetch_state(&proxy_state);
//        if(proxy_state == MS_PROXY_CONNECTED)
//        {
//            blebrr_disconnect_pl();
//        }
//    }
}

void vm_publication_add (UCHAR*             data_parm)
{
    MS_ACCESS_PUBLISH_INFO    publish_info;
    UINT8                     marker;
    //MS_ACCESS_MODEL_ID      model_id;
    MS_NET_ADDR uaddr;
    marker = 0;
    /* ElementAddress */
    MS_UNPACK_LE_2_BYTE(&uaddr, data_parm + marker);
    marker += 2;
    publish_info.addr.use_label = 0;
    MS_UNPACK_LE_2_BYTE(&publish_info.addr.addr, data_parm + marker);
    marker += 2;
    /* AppKeyIndex */
    MS_UNPACK_LE_2_BYTE(&publish_info.appkey_index, data_parm + marker);
    marker += 2;
    /* CredentialFlag */
    publish_info.crden_flag = (UCHAR)((publish_info.appkey_index >> 12) & 0x01);
    publish_info.appkey_index &= 0x0FFF;
    /* PublishTTL */
    publish_info.ttl = data_parm[marker];
    marker++;
    /* PublishPeriod */
    publish_info.period = data_parm[marker];
    marker++;
    /* PublishRetransmitCount, PublishRetransmitIntervalSteps */
    publish_info.rtx_count = (UCHAR)(data_parm[marker] & 0x07);
    publish_info.rtx_interval_steps = (UCHAR)(data_parm[marker] >> 3);
    marker++;
    /* Get Model ID type, based on the length */
    //MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_parm + marker));
    //model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
    //printf(
    //"[CONFIG] Model Publication Set. ElementAddress 0x%04X\n", uaddr);
    //if (0 == publish_info.addr.use_label)
    //{
    //    printf(
    //    "[CONFIG] PublishAddress 0x%04X\n", publish_info.addr.addr);
    //}
    //printf(
    //"[CONFIG] AppKeyIndex 0x%04X\n", publish_info.appkey_index);
    //printf(
    //"[CONFIG] CredentialFlag 0x%02X\n", publish_info.crden_flag);
    //printf(
    //"[CONFIG] PublishTTL 0x%02X\n", publish_info.ttl);
    //printf(
    //"[CONFIG] PublishPeriod 0x%02X\n", publish_info.period);
    //printf(
    //"[CONFIG] PublishRetransmitCount 0x%02X\n", publish_info.rtx_count);
    //printf(
    //"[CONFIG] PublishRetransmitIntervalSteps 0x%02X\n", publish_info.rtx_interval_steps);
    //printf(
    //"[CONFIG] Model Type: %s. Model ID 0x%08X\n",
    //((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
    MS_access_cm_set_model_publication(UI_generic_onoff_client_model_handle,&publish_info);
    #ifdef  USE_LIGHTNESS
    MS_access_cm_set_model_publication(UI_generic_lightness_client_model_handle,&publish_info);
    #endif
    #ifdef  USE_CTL
    MS_access_cm_set_model_publication(UI_light_ctl_client_model_handle,&publish_info);
    #endif
    #ifdef  USE_HSL
    MS_access_cm_set_model_publication(UI_light_hsl_client_model_handle,&publish_info);
    #endif
    #ifdef  USE_SCENE
    MS_access_cm_set_model_publication(UI_scene_client_model_handle,&publish_info);
    #endif
//#ifdef  USE_VENDORMODEL
//    MS_access_cm_set_model_publication(UI_vendor_defined_client_model_handle,&publish_info);
//#endif
}


API_RESULT UI_app_config_server_callback (
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR               saddr,
    /* IN */ MS_NET_ADDR               daddr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_APPKEY_HANDLE          appkey_handle,
    /* IN */ UINT32                    opcode,
    /* IN */ UCHAR*                    data_parm,
    /* IN */ UINT16                    data_len,
    /* IN */ API_RESULT                retval,
    /* IN */ UINT32                    response_opcode,
    /* IN */ UCHAR*                    response_buffer,
    /* IN */ UINT16                    response_buffer_len)
{
    uint8_t tx_state;
    UCHAR  proxy_state;
    #ifdef EASY_BOUNDING
    MS_ACCESS_ADDRESS         addr;
    #endif
    CONSOLE_OUT("[APP_CFG_SERV_CB] %04x \n", opcode);

    switch (opcode)
    {
    case MS_ACCESS_CONFIG_NODE_RESET_OPCODE:
        CONSOLE_OUT("[ST TimeOut CB]\n");
        proxy_state = UI_proxy_state_get();
        nvs_reset(NVS_BANK_PERSISTENT);

//        MS_access_cm_reset(PROV_ROLE_DEVICE);

        if(MS_PROXY_CONNECTED != proxy_state)
        {
            EM_start_timer (&thandle, 5, timeout_cb, NULL, 0);
        }
        else
        {
            blebrr_disconnect_pl();
        }

        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE:
        #ifdef EASY_BOUNDING
        MS_UNPACK_LE_2_BYTE(&addr.addr, data_parm + 2);
        CONSOLE_OUT("[CONFIG] Subscription Address 0x%04X\n",addr.addr);
        CONSOLE_OUT("[MODEL SUBS ADD]\n");
        //vm_subscriptiong_add(addr.addr);
        #endif
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE:
        #ifdef EASY_BOUNDING
        MS_UNPACK_LE_2_BYTE(&addr.addr, data_parm + 2);
        CONSOLE_OUT("[CONFIG] Subscription Address 0x%04X\n",addr.addr);
        CONSOLE_OUT("[MODEL SUBS DELETE]\n");
        //vm_subscriptiong_delete(addr.addr);
        #endif
        break;

    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_SET_OPCODE:
        MS_access_cm_get_transmit_state(MS_NETWORK_TX_STATE, &tx_state);
        CONSOLE_OUT("[NET TRX] 0x%02X \n",tx_state);
        break;

    case MS_ACCESS_CONFIG_RELAY_SET_OPCODE:
        MS_access_cm_get_transmit_state(MS_RELAY_TX_STATE, &tx_state);
        CONSOLE_OUT("[RLY TRX] 0x%02X \n",tx_state);
        break;

    case MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE:
        #ifdef EASY_BOUNDING
        blebrr_scan_pl(FALSE);
        vm_subscriptiong_binding_cb();
        ms_provisioner_addr = saddr;
        #if (CFG_HEARTBEAT_MODE)
        UI_trn_set_heartbeat_subscription(saddr);
        #endif
        #else
//        CONSOLE_OUT("Stop timer\n");
        ms_provisioner_addr = saddr;
//        EM_stop_timer(&procfg_timer_handle);
        blebrr_prov_started = MS_FALSE;
        #endif
        break;

    case MS_ACCESS_CONFIG_MODEL_APP_BIND_OPCODE:
        break;

    default:
        break;
    }

    return API_SUCCESS;
}



void appl_mesh_sample (void)
{
    MS_ACCESS_NODE_ID node_id;
    MS_ACCESS_ELEMENT_DESC   element;
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    API_RESULT retval;
    MS_CONFIG* config_ptr;
    #ifdef MS_HAVE_DYNAMIC_CONFIG
    MS_CONFIG  config;
    /* Initialize dynamic configuration */
    MS_INIT_CONFIG(config);
    config_ptr = &config;
    #else
    config_ptr = NULL;
    #endif /* MS_HAVE_DYNAMIC_CONFIG */
    /* Initialize OSAL */
    EM_os_init();
    /* Initialize Debug Module */
    EM_debug_init();
    /* Initialize Timer Module */
    EM_timer_init();
    timer_em_init();
    #if defined ( EM_USE_EXT_TIMER )
    EXT_cbtimer_init();
    ext_cbtimer_em_init();
    #endif
    /* Initialize utilities */
    nvsto_init(NVS_FLASH_BASE1,NVS_FLASH_BASE2);
    /* Initialize Mesh Stack */
    MS_init(config_ptr);
    /* Register with underlying BLE stack */
    blebrr_register();
    /* Register GATT Bearer Connection/Disconnection Event Hook */
    blebrr_register_gatt_iface_event_pl(UI_gatt_iface_event_pl_cb);
    /* Create Node */
    retval = MS_access_create_node(&node_id);
    /* Register Element */
    /**
        TBD: Define GATT Namespace Descriptions from
        https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors

        Using 'main' (0x0106) as Location temporarily.
    */
    element.loc = 0x0106;
    retval = MS_access_register_element
             (
                 node_id,
                 &element,
                 &element_handle
             );

    if (API_SUCCESS == retval)
    {
        /* Register foundation model servers */
        retval = UI_register_foundation_model_servers(element_handle);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model client */
        retval = UI_register_generic_onoff_model_client(element_handle);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Light HSL model client */
        retval = UI_register_light_hsl_model_client(element_handle);
    }

    #ifdef  USE_CTL

    if (API_SUCCESS == retval)
    {
        /* Register Light CTL model client */
        retval = UI_register_light_ctl_model_client(element_handle);
    }

    #endif

    if (API_SUCCESS == retval)
    {
        /* Register Scene model client */
        retval = UI_register_scene_model_client(element_handle);
    }

    #ifdef  USE_VENDORMODEL

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_register_vendor_defined_model_client(element_handle);
    }

    #endif
    /* Configure as provisionee/device */
    UI_register_prov();
    /**
        Set Scan Response Data Before Starting Provisioning.
        This is optional/additional set of Data that the device can
        set to enhance the User Experience.
        For Example, set a specific device name or URL as part of the
        Scan Response Data when awaiting connections over GATT bearer.
    */
    UI_set_brr_scan_rsp_data();
    APP_config_server_CB_init(UI_app_config_server_callback);
    uint32 address = VENDOR_PRODUCT_MAC_ADDR;
    hal_flash_read(address ++,&UI_lprov_device.uuid[10],1);
    hal_flash_read(address ++,&UI_lprov_device.uuid[11],1);
    hal_flash_read(address ++,&UI_lprov_device.uuid[12],1);
    hal_flash_read(address ++,&UI_lprov_device.uuid[13],1);
    hal_flash_read(address ++,&UI_lprov_device.uuid[8],1);
    hal_flash_read(address ++,&UI_lprov_device.uuid[9],1);
    EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
    return;
}

API_RESULT UI_sample_get_net_key(void )
{
    UINT8   index=0;
    UINT8   key[16];
    API_RESULT retval;
    CONSOLE_OUT("Fetching Net Key for indx 0x0000\n");
    retval = MS_access_cm_get_netkey_at_offset
             (
                 index,
                 0,
                 key
             );

    /* Check Retval. Print Net Key */
    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT("Network Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    index, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                    key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);
    }
    else
    {
        CONSOLE_OUT("FAILED. Reason: 0x%04X\n", retval);
    }

    return API_SUCCESS;
}

API_RESULT UI_sample_get_device_key(void )
{
    UINT8   index=0;
    UINT8* key;
    API_RESULT retval;
    CONSOLE_OUT("Fetching Dev Key for indx 0x0000\n");
    retval = MS_access_cm_get_device_key
             (
                 index,
                 &key
             );

    /* Check Retval. Print Device Key */
    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT("Device Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    index, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                    key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);
    }
    else
    {
        CONSOLE_OUT("FAILED. Reason: 0x%04X\n", retval);
    }

    return API_SUCCESS;
}

API_RESULT UI_sample_check_app_key(void)
{
    MS_APPKEY_HANDLE  handle;
    UINT8*              key;
    UINT8             aid;
    DECL_CONST UINT8  t_key[16] = {0};
    API_RESULT retval;
    CONSOLE_OUT("Fetching App Key for Handle 0x0000\n");
    handle = 0x0000;
    retval = MS_access_cm_get_app_key
             (
                 handle,
                 &key,
                 &aid
             );

    /* Check Retval. Print App Key */
    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT("App Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                    handle, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                    key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

        if (0 == EM_mem_cmp(key, t_key, 16))
        {
            /* NO AppKey Bound */
            retval = API_FAILURE;
        }
        else
        {
            /* Found a Valid App Key */
            /* Keeping the retval as API_SUCCESS */
        }
    }

    return retval;
}

/**
    On GATT Bearer Disconnection or on Startup:
    1. If device not provisioned, start unprovisioned beacons over GATT bearer
    2. If device is provisioned and App Key is bound i.e. Device is Configured
       - Check if Proxy Feature is enabled then Start ADV as Proxy,
         Else,
         Do nothing!
      Else,
      If device is provisioned and App Key is not bound i.e. Device is not Configured
         Start ADV as Proxy.
*/
void UI_sample_reinit(void)
{
    API_RESULT  retval;
    MS_NET_ADDR addr;
    UCHAR       is_prov_req;
    UCHAR       role, brr;
    UCHAR       state;
    retval      = API_SUCCESS;
    is_prov_req = MS_TRUE;
    retval = MS_access_cm_get_primary_unicast_address(&addr);

    if (API_SUCCESS == retval)
    {
        if (MS_NET_ADDR_UNASSIGNED != addr)
        {
            /* Set Provisioning is not Required */
            is_prov_req = MS_FALSE;
        }
    }

    if (MS_TRUE == is_prov_req)
    {
        /* Start Provisioning over GATT here */
        /**
            setup <role:[1 - Device, 2 - Provisioner]> <bearer:[1 - Adv, 2 - GATT]
        */
        role = PROV_ROLE_DEVICE;
        brr  = PROV_BRR_GATT;
        /**
            Setting up an Unprovisioned Device over GATT
        */
        LIGHT_ONLY_RED_ON;
        blebrr_prov_started = MS_FALSE;
        UI_setup_prov(role, brr);
        CONSOLE_OUT("\r\n Setting up as an Unprovisioned Device\r\n");
    }
    else
    {
        /* Fetch PROXY feature state */
        MS_access_cm_get_features_field(&state, MS_FEATURE_PROXY);

        /**
            Check if the Device is Configured.
            If not Configured, Start Proxy ADV.
            If it is Configured,
                Check if the Proxy Feature is Enabled.
                If not enabled, then Do Nothing!
                If it is, Start Proxy ADV.
        */
        if (API_SUCCESS == UI_sample_check_app_key())
        {
            UI_sample_get_device_key();

            if (MS_ENABLE == state)
            {
                light_blink_set(LIGHT_GREEN, LIGHT_BLINK_FAST,5);
//                CONSOLE_OUT("\r\n Provisioned Device - Starting Proxy with NetID on Subnet 0x0000!\r\n");
//
//                /* Start Proxy ADV with Network ID here */
//                UI_proxy_start_adv(0x0000, MS_PROXY_NET_ID_ADV_MODE);
                //for silab 2.0.0 app use NODE ID
                CONSOLE_OUT("\r\n Provisioned Device - Starting Proxy with NODE ID on Subnet 0x0000!\r\n");
                UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
                // >>>, PANDA,
                //osal_set_event(bleMesh_TaskID, BLEMESH_PROV_COMP_EVT);
            }
            else
            {
                light_blink_set(LIGHT_GREEN, LIGHT_BLINK_SLOW,3);
                MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NODEID, BRR_BCON_ACTIVE);
                CONSOLE_OUT("\r\n Provisioned Device\r\n");
//                CONSOLE_OUT("\r\n BLEBRR_GET_STATE()= %d\r\n",BLEBRR_GET_STATE());
                /**
                    Do Nothing!
                    Already Scaning is Enabled at Start Up
                */
                // blebrr_scan_enable();
                // >>>, PANDA,
                osal_set_event(bleMesh_TaskID, BLEMESH_PROV_COMP_EVT);
            }
        }
        else
        {
            light_blink_set(LIGHT_BLUE, LIGHT_BLINK_FAST,5);

            if (MS_ENABLE == state)
            {
                UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
            }

            /**
                Provisioned but not configured device.
                Still checking if the PROXY Feature is enabled or not.
                Depending on the state of Proxy Feature:
                 - If enabled, Start Proxy ADV with Network ID
                 - Else, Start Proxy ADV with Node Identity.
            */
//             (MS_ENABLE == state) ?
//             UI_proxy_start_adv(0x0000, MS_PROXY_NET_ID_ADV_MODE):
//             UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
        }
    }
}


