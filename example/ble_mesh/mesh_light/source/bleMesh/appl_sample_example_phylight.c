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
    \file appl_sample_example_phylight.c

    Source File for Mesh light Server, support SIG model: Generic OnOff, Light HSL, Light CTL, ...

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

#include "led_light.h"
#include "bleMesh.h"
/* For Persistent Storage */
#include "access_extern.h"
#include "bleMesh.h"
#include "vendormodel_server.h"
#include "EXT_cbtimer.h"
#include "net_internal.h"


#undef USE_HEALTH            // enable Light Lightness server model
#define USE_HSL                 // enable Light HSL server model
#define USE_LIGHTNESS            // enable Light Lightness server model
#define  USE_CTL                 // disable Light CTL server model
#undef USE_SCENE               // enable Light Scene server model
#define USE_VENDORMODEL         // enable Light vendormodel server model
#define  EASY_BOUNDING

/* Console Input/Output */
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)
extern uint32_t osal_sys_tick;

#define VENDOR_PRODUCT_MAC_ADDR         0x4000
#define PROCFG_COMPLETE_TIMEOUT         60



void appl_dump_bytes(UCHAR* buffer, UINT16 length);
void appl_mesh_sample (void);

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
void timeout_cb (void* args, UINT16 size);

/* --------------------------------------------- Global Definitions */


EM_timer_handle thandle;
EM_timer_handle proxy_dly_thandle;
#if (CFG_HEARTBEAT_MODE)
    EM_timer_handle heartbeat_reply_dly_thandle;
#endif


void proxy_dly_generic_onoff (void* args, UINT16 size);
//void proxy_dly_generic_hsl (void * args, UINT16 size);
extern UCHAR blebrr_prov_started;

/* Provsion timeout handle */
EM_timer_handle procfg_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

/* --------------------------------------------- Global Definitions */

typedef struct UI_state_scene_struct
{
    UINT32                          index;
    MS_STATE_LIGHT_HSL_STRUCT       light_hsl_state;
    MS_STATE_GENERIC_ONOFF_STRUCT   onoff_state;
} UI_STATE_SCENE_STRUCT;

typedef struct MS_state_vendor_model_hsl_struct
{
    /** The perceived lightness of a light emitted by the element */
    UINT16 hsl_lightness;

    /** The 16-bit value representing the hue */
    UINT16 hsl_hue;

    /** The saturation of a color light */
    UINT16 hsl_saturation;
} MS_STATE_VENDOR_MODEL_HSL_STRUCT;


#if (CFG_HEARTBEAT_MODE)
/** Vendor Model specific state parameters in a request or response message */
typedef struct _Heartbeat_rcv_params
{
    MS_NET_ADDR         heartbeat_addr;
    MS_NET_ADDR         heartbeat_sddr;
    MS_SUBNET_HANDLE    heartbeat_subnet_index;
    UINT8               heartbeat_countlog;

} HEARTBEAT_RCV_PARAMS;
#endif

typedef struct MS_state_vendor_example_struct
{
    UCHAR  value;

} MS_STATE_VENDOR_EXAMPLE_STRUCT;

/** -- Vendor Defined States */
static MS_STATE_VENDOR_EXAMPLE_STRUCT UI_vendor_example;


/* ----------------------------------------- External Global Variables */

/* ----------------------------------------- Exported Global Variables */


/* ----------------------------------------- Static Global Variables */

MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_generic_onoff_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_lightness_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_ctl_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_ctl_setup_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_hsl_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_hsl_setup_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_scene_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_scene_setup_server_model_handle;




/* Model Server - Foundation Models */

/* Health Server - Test Routines */
#ifdef USE_HEALTH
static void UI_health_self_test_00(UINT8 test_id, UINT16 company_id)
{
    MS_IGNORE_UNUSED_PARAM(test_id);
    MS_IGNORE_UNUSED_PARAM(company_id);
}

static void UI_health_self_test_01(UINT8 test_id, UINT16 company_id)
{
    MS_IGNORE_UNUSED_PARAM(test_id);
    MS_IGNORE_UNUSED_PARAM(company_id);
}

static void UI_health_self_test_FF(UINT8 test_id, UINT16 company_id)
{
    MS_IGNORE_UNUSED_PARAM(test_id);
    MS_IGNORE_UNUSED_PARAM(company_id);
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
#endif

API_RESULT UI_register_foundation_model_servers
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Configuration Server */
    MS_ACCESS_MODEL_HANDLE   UI_config_server_model_handle;
    API_RESULT retval;
    #ifdef USE_HEALTH
    /* Health Server */
    MS_ACCESS_MODEL_HANDLE   UI_health_server_model_handle;
    UINT16                       company_id;
    MS_HEALTH_SERVER_SELF_TEST* self_tests;
    UINT32                       num_self_tests;
    #endif
    CONSOLE_OUT("In Model Server - Foundation Models\n");
    retval = MS_config_server_init(element_handle, &UI_config_server_model_handle);
    CONSOLE_OUT("Config Model Server Registration Status: 0x%04X\n", retval);
    #ifdef USE_HEALTH
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

    #endif
    return retval;
}

UCHAR UI_proxy_state_get(void)
{
    UCHAR  proxy_state;
    MS_proxy_fetch_state(&proxy_state);
    return proxy_state;
}



// ===========================  Generic OnOff Server Model Functions =============
/* ---- Generic OnOff States */
static MS_STATE_GENERIC_ONOFF_STRUCT UI_generic_onoff;

/* Generic OnOff Model state Initialization */
static void UI_generic_onoff_model_states_initialization(void)
{
    EM_mem_set(&UI_generic_onoff, 0, sizeof(UI_generic_onoff));
}

/* Generic OnOff Model Get Handler */
static API_RESULT UI_generic_onoff_model_state_get(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    API_RESULT retval;
    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_GENERIC_ONOFF_T:
    {
        MS_STATE_GENERIC_ONOFF_STRUCT* param_p;
        param_p = (MS_STATE_GENERIC_ONOFF_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        *param_p = UI_generic_onoff;
    }
    break;

    default:
        break;
    }

    return retval;
}

/* Generic OnOff Model Set Handler */
static API_RESULT UI_generic_onoff_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    API_RESULT retval;
    UCHAR  proxy_state;
    retval = API_SUCCESS;

    switch (state_t)
    {
    case MS_STATE_GENERIC_ONOFF_T:
    {
        MS_STATE_GENERIC_ONOFF_STRUCT* param_p;
        param_p = (MS_STATE_GENERIC_ONOFF_STRUCT*)param;
        /* Instantaneous Change */
        UI_generic_onoff.onoff = param_p->onoff;
        *param_p = UI_generic_onoff;
        CONSOLE_OUT("[state] current: 0x%02X\n", UI_generic_onoff.onoff);
        CONSOLE_OUT("[state] target: 0x%02X\n", UI_generic_onoff.target_onoff);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", UI_generic_onoff.transition_time);
        proxy_state = UI_proxy_state_get();

        if(proxy_state==MS_PROXY_CONNECTED)
        {
            EM_start_timer (&proxy_dly_thandle,    (EM_TIMEOUT_MILLISEC | 100), proxy_dly_generic_onoff, NULL, 0);
        }
        else
        {
            generic_onoff_set_pl(param_p->onoff);
        }

        /* Ignoring Instance and direction right now */
    }
    break;

    default:
        break;
    }

    return retval;
}

/* Generic OnOff Model Server */
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Generic_Onoff server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
static API_RESULT UI_generic_onoff_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_STATE_GENERIC_ONOFF_STRUCT    param;
    MS_ACCESS_MODEL_STATE_PARAMS     current_state_params;
    API_RESULT                       retval;
    retval = API_SUCCESS;

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT("[GENERIC_ONOFF] GET Request.\n");
        UI_generic_onoff_model_state_get(state_params->state_type, 0, &param, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = &param;
        /* Using same as target state and remaining time as 0 */
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT("[GENERIC_ONOFF] SET Request.\n");
        retval = UI_generic_onoff_model_state_set(state_params->state_type, 0, (MS_STATE_GENERIC_ONOFF_STRUCT*)state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = (MS_STATE_GENERIC_ONOFF_STRUCT*)state_params->state;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT("[GENERIC_ONOFF] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_generic_onoff_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
    }

    return retval;
}
API_RESULT UI_register_generic_onoff_model_server
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Generic OnOff Server */
    API_RESULT retval;
    CONSOLE_OUT("In Generic OnOff Model Server\n");
    retval = MS_generic_onoff_server_init
             (
                 element_handle,
                 &UI_generic_onoff_server_model_handle,
                 UI_generic_onoff_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Generic Onoff Server Initialized. Model Handle: 0x%04X\n",
            UI_generic_onoff_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Generic Onoff Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}


// ===================== Light HSL Server Model Functions =====================
/** -- Light - HSL states*/
static MS_STATE_LIGHT_HSL_STRUCT UI_light_hsl;
static MS_STATE_LIGHT_HSL_RANGE_STRUCT UI_light_hsl_range;
static MS_STATE_LIGHT_HSL_DEFAULT_STRUCT UI_light_hsl_default;

void UI_light_hsl_model_states_initialization(void)
{
    /* Light HSL States */
    EM_mem_set (&UI_light_hsl, 0, sizeof(UI_light_hsl));
    EM_mem_set(&UI_light_hsl_range, 0, sizeof(UI_light_hsl_range));
    EM_mem_set(&UI_light_hsl_default, 0, sizeof(UI_light_hsl_default));
}



API_RESULT UI_light_hsl_model_state_get(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_LIGHT_HSL_T:
    {
        MS_STATE_LIGHT_HSL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_HSL_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        *param_p = UI_light_hsl;
    }
    break;

    default:
        break;
    }

    return API_SUCCESS;
}

void UI_light_hsl_set_actual(UINT16 state_inst, UINT16 lightness, UINT16 hue, UINT16 saturation, UCHAR forced_publish)
{
    light_hsl_set_pl(hue,saturation,lightness);
}

API_RESULT UI_light_hsl_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    API_RESULT retval;
    UCHAR  proxy_state;
    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_LIGHT_HSL_T:
    {
        MS_STATE_LIGHT_HSL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_HSL_STRUCT*)param;
        {
            UI_light_hsl.hsl_lightness = param_p->hsl_lightness;
            UI_light_hsl.hsl_hue = param_p->hsl_hue;
            UI_light_hsl.hsl_saturation = param_p->hsl_saturation;
            UI_light_hsl.tid = param_p->tid;
            /*
                appl_light_hsl[0].hsl_lightness = param_p->hsl_lightness;
                appl_light_hsl[0].hsl_lightness = param_p->hsl_lightness;
                appl_light_hsl[0].hsl_lightness = param_p->hsl_lightness;
            */
            //appl_light_lightness[state_inst].light_lightness_actual.lightness_actual = param_p->hsl_lightness;
        }
        *param_p = UI_light_hsl;
        CONSOLE_OUT("[state] current Hue: 0x%04X\n", UI_light_hsl.hsl_hue);
        CONSOLE_OUT("[state] current Saturation: 0x%04X\n", UI_light_hsl.hsl_saturation);
        CONSOLE_OUT("[state] current Lightness: 0x%04X\n", UI_light_hsl.hsl_lightness);
        CONSOLE_OUT("[state] target Hue: 0x%04X\n", UI_light_hsl.target_hsl_hue);
        CONSOLE_OUT("[state] target Saturation: 0x%04X\n", UI_light_hsl.target_hsl_saturation);
        CONSOLE_OUT("[state] target Lightness: 0x%04X\n", UI_light_hsl.target_hsl_lightness);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", UI_light_hsl.transition_time);
        proxy_state = UI_proxy_state_get();

        if(proxy_state==MS_PROXY_CONNECTED)
        {
            UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
        }
        else
        {
            UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
        }

        /* Ignoring Instance and direction right now */
    }
    break;

    default:
        break;
    }

    return retval;
}

/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Light_Hsl server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
API_RESULT UI_light_hsl_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_ACCESS_MODEL_STATE_PARAMS         current_state_params;
    MS_STATE_LIGHT_HSL_STRUCT            param;
    MS_STATE_LIGHT_HSL_RANGE_STRUCT      param_range;
    MS_STATE_LIGHT_HSL_DEFAULT_STRUCT    param_default;
    void*                                param_p;
    API_RESULT                           retval;
    MS_IGNORE_UNUSED_PARAM(msg_raw);
    MS_IGNORE_UNUSED_PARAM(ext_params);
    retval = API_SUCCESS;

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_HSL] GET Request.\n");

        switch (state_params->state_type)
        {
        case MS_STATE_LIGHT_HSL_T:
        case MS_STATE_LIGHT_HSL_HUE_T:
        case MS_STATE_LIGHT_HSL_SATURATION_T:
        case MS_STATE_LIGHT_HSL_TARGET_T:
        {
            param_p = &param;
        }
        break;

        case MS_STATE_LIGHT_HSL_DEFAULT_T:
        {
            param_p = &param_default;
        }
        break;

        case MS_STATE_LIGHT_HSL_RANGE_T:
        {
            param_p = &param_range;
        }
        break;

        default:
            break;
        }

        UI_light_hsl_model_state_get(state_params->state_type, 0, param_p, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = param_p;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_HSL] SET Request.\n");
        UI_light_hsl_model_state_set(state_params->state_type, 0, state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = state_params->state;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[LIGHT_HSL] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_light_hsl_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
    }

    return retval;
}


API_RESULT UI_register_light_hsl_model_server
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Generic OnOff Server */
    API_RESULT retval;
    retval = MS_light_hsl_server_init
             (
                 element_handle,
                 &UI_light_hsl_server_model_handle,
                 &UI_light_hsl_setup_server_model_handle,
                 UI_light_hsl_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Light Hsl Server Initialized. Model Handle: 0x%04X\n",
            UI_light_hsl_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Light Hsl Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

// ==================== Light CTL Server Model Functions ====================
/** -- Light - CTL states*/
static MS_STATE_LIGHT_CTL_STRUCT UI_light_ctl;
void UI_light_ctl_model_states_initialization(void)
{
    /* Light Lightness States */
    EM_mem_set (&UI_light_ctl, 0, sizeof(UI_light_ctl));
}



API_RESULT UI_light_ctl_model_state_get(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_CTL_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        *param_p = UI_light_ctl;
    }
    break;

    default:
        break;
    }

    return API_SUCCESS;
}

void UI_light_ctl_set_actual(UINT16 state_inst, UINT16 lightness, UINT16 temperature, UINT16 deltaUv)
{
    // TODO: convert CTL to RGB and set light
    light_lightness_set_pl (lightness);
    light_ctl_set_pl (temperature,deltaUv);
}

API_RESULT UI_light_ctl_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    API_RESULT retval;
    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_CTL_STRUCT*)param;
        {
            UI_light_ctl.ctl_lightness = param_p->ctl_lightness;
            UI_light_ctl.ctl_temperature = param_p->ctl_temperature;
            UI_light_ctl.ctl_delta_uv = param_p->ctl_delta_uv;
            UI_light_ctl.delay = param_p->delay;
            UI_light_ctl.transition_time = param_p->transition_time;
            UI_light_ctl.target_ctl_lightness = param_p->target_ctl_lightness;
            UI_light_ctl.target_ctl_temperature = param_p->target_ctl_temperature;
            //appl_light_lightness[state_inst].light_lightness_actual.lightness_actual = param_p->hsl_lightness;
        }
        UI_light_ctl_set_actual(0, UI_light_ctl.ctl_lightness, UI_light_ctl.ctl_temperature, UI_light_ctl.ctl_delta_uv);
        *param_p = UI_light_ctl;
        CONSOLE_OUT("[state] current Lightness: 0x%04X\n", UI_light_ctl.ctl_lightness);
        CONSOLE_OUT("[state] current Temperature: 0x%04X\n", UI_light_ctl.ctl_temperature);
        CONSOLE_OUT("[state] current Delta UV: 0x%04X\n",  UI_light_ctl.ctl_delta_uv);
        // below parameters are not set
        CONSOLE_OUT("[state] target Lightness: 0x%04X\n",  UI_light_ctl.target_ctl_lightness);
        CONSOLE_OUT("[state] target Temperature: 0x%04X\n", UI_light_ctl.target_ctl_temperature);
        CONSOLE_OUT("[state] delay time: 0x%04X\n", UI_light_ctl.delay);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", UI_light_hsl.transition_time);
        /* Ignoring Instance and direction right now */
    }
    break;

    default:
        break;
    }

    return retval;
}

/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Light_Hsl server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
API_RESULT UI_light_ctl_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_ACCESS_MODEL_STATE_PARAMS                   current_state_params;
    MS_STATE_LIGHT_CTL_STRUCT                      light_ctl_params;
    MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_STRUCT    light_ctl_temperature_range_params;
    MS_STATE_LIGHT_CTL_DEFAULT_STRUCT              light_ctl_default_params;
    MS_STATE_LIGHT_CTL_TEMPERATURE_STRUCT          light_ctl_temperature_params;
    void*                                          param_p;
    API_RESULT retval;
    retval = API_SUCCESS;

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_CTL] GET Request.\n");

        switch (state_params->state_type)
        {
        case MS_STATE_LIGHT_CTL_DEFAULT_T:
        {
            param_p = &light_ctl_default_params;
        }
        break;

        case MS_STATE_LIGHT_CTL_T:
        {
            param_p = &light_ctl_params;
        }
        break;

        case MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_T:
        {
            param_p = &light_ctl_temperature_range_params;
        }
        break;

        case MS_STATE_LIGHT_CTL_TEMPERATURE_T:
        {
            param_p = &light_ctl_temperature_params;
        }
        break;

        default:
            break;
        }

        UI_light_ctl_model_state_get(state_params->state_type, 0, param_p, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = param_p;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_CTL] SET Request.\n");
        UI_light_ctl_model_state_set(state_params->state_type, 0, state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = state_params->state;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[LIGHT_CTL] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_light_ctl_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
    }

    return retval;
}


API_RESULT UI_register_light_ctl_model_server
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Light CTL Server */
    API_RESULT retval;
    retval = MS_light_ctl_server_init
             (
                 element_handle,
                 &UI_light_ctl_server_model_handle,
                 &UI_light_ctl_setup_server_model_handle,
                 UI_light_ctl_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Light Ctl Server Initialized. Model Handle: 0x%04X\n",
            UI_light_ctl_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Light Ctl Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}

/* Vendor Defined Model state Initialization */
void UI_vendor_defined_model_states_initialization(void)
{
    /* Vendor Defined States */
    EM_mem_set (&UI_vendor_example, 0, sizeof(UI_vendor_example));
}

/* Vendor Defined Model Get Handler */
void UI_vendor_example_model_state_get(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_VENDORMODEL_ONOFF_T:
    {
        UINT8   onoff_status;
        MS_ACCESS_VENDORMODEL_STATE_PARAMS* param_p;
        param_p = (MS_ACCESS_VENDORMODEL_STATE_PARAMS*)param;
        /* Ignoring Instance and direction right now */
        param_p->vendormodel_type = MS_STATE_VENDORMODEL_ONOFF_T;
        onoff_status = UI_generic_onoff.onoff;
        param_p->vendormodel_param = &onoff_status;
    }
    break;

    default:
        break;
    }

//    param = param_p;
}
void UI_vendor_model_set_red(void)
{
    UI_light_hsl.hsl_lightness = 0x8000;
    UI_light_hsl.hsl_hue = 0x0000;
    UI_light_hsl.hsl_saturation = 0xFFFF;
    UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
    //*param_p = UI_light_hsl;
    CONSOLE_OUT("[state] current Hue: 0x%04X\n", UI_light_hsl.hsl_hue);
    CONSOLE_OUT("[state] current Saturation: 0x%04X\n", UI_light_hsl.hsl_saturation);
    CONSOLE_OUT("[state] current Lightness: 0x%04X\n", UI_light_hsl.hsl_lightness);
}

void UI_vendor_model_set_green(void)
{
    UI_light_hsl.hsl_lightness = 0x7FFF;
    UI_light_hsl.hsl_hue = 0xAAAA;
    UI_light_hsl.hsl_saturation = 0xFFFF;
    UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
    //*param_p = UI_light_hsl;
    CONSOLE_OUT("[state] current Hue: 0x%04X\n", UI_light_hsl.hsl_hue);
    CONSOLE_OUT("[state] current Saturation: 0x%04X\n", UI_light_hsl.hsl_saturation);
    CONSOLE_OUT("[state] current Lightness: 0x%04X\n", UI_light_hsl.hsl_lightness);
}

void UI_vendor_model_set_blue(void)
{
    UI_light_hsl.hsl_lightness = 0x7FFF;
    UI_light_hsl.hsl_hue = 0x5555;
    UI_light_hsl.hsl_saturation = 0xFFFF;
    UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
    //*param_p = UI_light_hsl;
    CONSOLE_OUT("[state] current Hue: 0x%04X\n", UI_light_hsl.hsl_hue);
    CONSOLE_OUT("[state] current Saturation: 0x%04X\n", UI_light_hsl.hsl_saturation);
    CONSOLE_OUT("[state] current Lightness: 0x%04X\n", UI_light_hsl.hsl_lightness);
}




/* Vendor Defined Model Set Handler */
void UI_vendor_example_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_VENDORMODEL_ONOFF_T:
    {
        UINT8*    param_p;
        param_p = (UCHAR*)param;
        generic_onoff_set_pl(*param_p);
    }
    break;

    case MS_STATE_VENDORMODEL_HSL_T:
    {
        MS_STATE_VENDOR_MODEL_HSL_STRUCT* param_p;
        param_p = (MS_STATE_VENDOR_MODEL_HSL_STRUCT*)param;
        UI_light_hsl.hsl_lightness = param_p->hsl_lightness;
        UI_light_hsl.hsl_hue = param_p->hsl_hue;
        UI_light_hsl.hsl_saturation = param_p->hsl_saturation;
        UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
        //*param_p = UI_light_hsl;
        CONSOLE_OUT("[state] current Hue: 0x%04X\n", UI_light_hsl.hsl_hue);
        CONSOLE_OUT("[state] current Saturation: 0x%04X\n", UI_light_hsl.hsl_saturation);
        CONSOLE_OUT("[state] current Lightness: 0x%04X\n", UI_light_hsl.hsl_lightness);
    }
    break;

    default:
        break;
    }
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
API_RESULT UI_phy_model_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*         ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*             msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*               req_type,
    /* IN */ MS_ACCESS_VENDORMODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*              ext_params
)
{
    MS_ACCESS_VENDORMODEL_STATE_PARAMS                    current_state_params;
    UINT32  opcode;
    UINT16  marker = 0;
    UINT8*   data_param;
    API_RESULT retval;
    retval = API_SUCCESS;
    opcode = msg_raw->opcode;

    switch(opcode)
    {
    case MS_ACCESS_VENDORMODEL_WRITECMD_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_PHY_MODEL_WRITECMD_OPCODE\n");

        switch(state_params->vendormodel_type)
        {
        case MS_STATE_VENDORMODEL_RESET_T:
        {
            printf("rcv MS_STATE_PHY_MODEL_RESET_T\n");
            MS_common_reset();
            EM_start_timer (&thandle, 1, timeout_cb, NULL, 0);
        }
        break;
        }
    }
    break;

    case MS_ACCESS_VENDORMODEL_NOTIFY_OPCODE:
    {
        uint16      message_index;
        uint32      osal_tick;
        UINT8       ack;
        UINT8       ttl;
        UINT8       len;
        ACCESS_CM_GET_RX_TTL(ttl);
        data_param = state_params->vendormodel_param;
        MS_UNPACK_LE_1_BYTE(&ack, data_param+marker);
        marker += 1;
        MS_UNPACK_LE_2_BYTE(&message_index, data_param+marker);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&osal_tick, data_param+marker);
        marker += 3;
        MS_UNPACK_LE_1_BYTE(&len, data_param+marker);
        marker += 1;
        marker = 0;

        if(ack)
        {
            MS_PACK_LE_1_BYTE_VAL(data_param+marker,++vendor_tid);
            marker++;
            MS_PACK_LE_1_BYTE_VAL(data_param+marker,0);
            marker++;
            MS_PACK_LE_2_BYTE_VAL(data_param+marker,message_index);
            marker += 2;
            MS_PACK_LE_2_BYTE_VAL(data_param+marker,osal_tick);
            marker += 2;
            MS_PACK_LE_1_BYTE_VAL(data_param+marker,ttl&0xff);
            marker++;
            MS_PACK_LE_1_BYTE_VAL(data_param+marker,len&0xff);
            marker++;

            if(len)
            {
                EM_mem_set(data_param+marker, 0, len);
                marker += len;
            }

            state_params->vendormodel_param = data_param;
            req_type->to_be_acked = 0x01;
        }

//            printf("[PDU_Rx] Pkt.INDEX:0x%04X\n",message_index);
    }
    break;

    default:
        break;
    }

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
//        CONSOLE_OUT(
//        "[VENDOR_EXAMPLE] GET Request.\n");
        UI_vendor_example_model_state_get(state_params->vendormodel_type, 0, &current_state_params, 0);
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
//        CONSOLE_OUT(
//            "[VENDOR_EXAMPLE] SET Request.\n");
        UI_vendor_example_model_state_set(state_params->vendormodel_type, 0, state_params->vendormodel_param, 0);
        current_state_params.vendormodel_type = state_params->vendormodel_type;
        current_state_params.vendormodel_param = state_params->vendormodel_param;
    }
    else
    {
//        CONSOLE_OUT(
//            "[VENDOR_EXAMPLE] Other Request.\n");
        current_state_params.vendormodel_type = state_params->vendormodel_type;
        current_state_params.vendormodel_param = state_params->vendormodel_param;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
//        CONSOLE_OUT(
//            "[VENDOR_EXAMPLE] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_vendormodel_server_state_update(ctx, &current_state_params, NULL, 0, NULL,marker);
    }

    return retval;
}




API_RESULT UI_register_vendor_defined_model_server
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Vendor Defined Server */
    API_RESULT retval;
    retval = MS_vendormodel_server_init
             (
                 element_handle,
                 &UI_vendor_defined_server_model_handle,
                 UI_phy_model_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Vendor Defined Server Initialized. Model Handle: 0x%04X\n",
            UI_vendor_defined_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Vendor Defined Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}


// ==================== Scene Server Model Functions ====================
void UI_scene_model_states_initialization(void)
{
    // nothing now
}

// TODO: update below comments
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Scene server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.

*/
void* UI_scene_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*              handle,
    /* IN */ UINT8                                event_type,
    /* IN */ void*                                event_param,
    /* IN */ UINT16                               event_length,
    /* IN */ void*                                context
)
{
    void*       param_p;
    UINT32     index;
    UI_STATE_SCENE_STRUCT*   state_p;
    MS_STATE_LIGHT_HSL_STRUCT* state_hsl;
    param_p = NULL;
    (void)handle;

    switch(event_type)
    {
    case MS_SCENE_EVENT_STORE:
    {
        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("store scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)EM_alloc_mem(sizeof(UI_STATE_SCENE_STRUCT));

        if (NULL == state_p)
        {
            printf("Allocate memory fail, size = %d\r\n", sizeof(UI_STATE_SCENE_STRUCT));
            return NULL;
        }

        // save current states
        state_p->index = index;
        EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
        EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        param_p = (void*)state_p;
    }
    break;

    case MS_SCENE_EVENT_DELETE:
    {
        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("delete scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            EM_free_mem(state_p);
            printf("scene %d deleted\r\n", index);
        }
    }
    break;

    case MS_SCENE_EVENT_RECALL_START:
    {
        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("start recalling scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            state_hsl = &(state_p->light_hsl_state);
            light_hsl_set_pl(state_hsl->hsl_hue, state_hsl->hsl_saturation, state_hsl->hsl_lightness);
        }
    }
    break;

    case MS_SCENE_EVENT_RECALL_COMPLETE:
    {
        printf("MS_SCENE_EVENT_RECALL_COMPLETE \r\n");

        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("complete recalling scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            // nothing to do here
        }
    }
    break;

    case MS_SCENE_EVENT_RECALL_IMMEDIATE:
    {
        printf("MS_SCENE_EVENT_RECALL_IMMEDIATE \r\n");

        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("complete recalling scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            state_hsl = &(state_p->light_hsl_state);
            light_hsl_set_pl(state_hsl->hsl_hue, state_hsl->hsl_saturation, state_hsl->hsl_lightness);
        }
    }
    break;
    }

    return param_p;
}


API_RESULT UI_register_scene_model_server
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Generic OnOff Server */
    API_RESULT retval;
    retval = MS_scene_server_init
             (
                 element_handle,
                 &UI_scene_server_model_handle,
                 &UI_scene_setup_server_model_handle,
                 UI_scene_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "Scene Server Initialized. Model Handle: 0x%04X\n",
            UI_scene_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] Scene Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}



/* Model state Initialization */
static void UI_model_states_initialization(void)
{
    /* Generic OnOff States */
    UI_generic_onoff_model_states_initialization();
    #ifdef USE_HSL
    /* Light HSL States */
    UI_light_hsl_model_states_initialization();
    #endif
    #ifdef USE_CTL
    /* Light CTL States */
    UI_light_ctl_model_states_initialization();
    #endif
    /* Scene states */
    UI_scene_model_states_initialization();
    #ifdef  USE_VENDORMODEL
    /* Vendor Defined States */
    UI_vendor_defined_model_states_initialization();
    #endif
}

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
//DECL_STATIC PROV_DEVICE_S UI_lprov_device =
PROV_DEVICE_S UI_lprov_device =
{
    /** UUID */
    {0x05, 0x04, 0x62, 0x12, 0x00, 0x01, 0x00, 0x01, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00},

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

#if (CFG_HEARTBEAT_MODE)
static void heartbeat_reply_delay_handler(void* args, UINT16 size)
{
    HEARTBEAT_RCV_PARAMS call_receive_para;
    MS_IGNORE_UNUSED_PARAM(size);
//    printf("__rcv 0x%04X 0x%04X 0x%02X\n",call_receive_para.heartbeat_addr,call_receive_para.heartbeat_subnet_index,call_receive_para.heartbeat_countlog);
    UCHAR      buffer[8];
    UCHAR*        pdu_ptr;
    UINT16      marker;
    UINT32      opcode;
    UINT16      appkey_handle;
    call_receive_para = (*((HEARTBEAT_RCV_PARAMS*)args));
    marker = 0;
    appkey_handle = 0x0000/* + MS_CONFIG_LIMITS(MS_MAX_APPS)*/;
    buffer[marker++] = ++vendor_tid;
    MS_PACK_LE_2_BYTE_VAL(&buffer[marker], MS_STATE_PHY_MODEL_HB_CALLBACK_T);
    marker += 2;
    buffer[marker] = call_receive_para.heartbeat_countlog&0xff;
    marker++;
//    for(UINT8 i=0;i<marker;i++)
//    {
//        printf("%02X\n",buffer[i]);
//    }
    opcode = MS_ACCESS_PHY_MODEL_WRITECMD_OPCODE;
    /* Publish - reliable */
    pdu_ptr = buffer;
//    if(cfg_retry_flag == 0)
//    {
//        cfg_retry_flag = 1;
//    }
    MS_access_reply
    (
        &UI_vendor_defined_server_model_handle,
        call_receive_para.heartbeat_sddr,
        call_receive_para.heartbeat_addr,
        call_receive_para.heartbeat_subnet_index,
        appkey_handle,
        ACCESS_INVALID_DEFAULT_TTL,
        opcode,
        pdu_ptr,
        marker
    );
}

static API_RESULT UI_heartbeat_rcv_callback
(
    MS_NET_ADDR         addr,
    MS_SUBNET_HANDLE    subnet_index,
    UINT8               countlog
)
{
    API_RESULT retval;
    HEARTBEAT_RCV_PARAMS receive_para;
    MS_NET_ADDR saddr;
    receive_para.heartbeat_addr = addr;
    receive_para.heartbeat_countlog = countlog;
    receive_para.heartbeat_subnet_index = subnet_index;
//    printf("rcv 0x%04X 0x%04X 0x%02X\n",addr,subnet_index,countlog);
    MS_access_cm_get_primary_unicast_address(&saddr);
    receive_para.heartbeat_sddr = saddr;
    heartbeat_reply_dly_thandle = EM_TIMER_HANDLE_INIT_VAL;
    retval = EM_start_timer
             (
                 &heartbeat_reply_dly_thandle,
                 (EM_TIMEOUT_MILLISEC | (50 * (saddr&0xff))),
                 heartbeat_reply_delay_handler,
                 (void*)&receive_para,
                 sizeof(receive_para)
             );
    return retval;
}

static API_RESULT UI_heartbeat_rcv_timeout_callback(void)
{
    printf("heartbeat_timeout_callback\n");
    MS_common_reset();
    EM_start_timer (&thandle, 1, timeout_cb, NULL, 0);
//    printf("heartbeat_timeout_callback\n");
    return API_SUCCESS;
}
#endif

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
            /* LED ON/OFF for Provisioning Indication Abstraction Call */
            mesh_model_device_provisioned_ind_pl();
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
//static void UI_prov_filter_add(void)
//{
//
//  NETIF_HANDLE*    handle ;
//  *handle = 0x01;
////    UCHAR      pdu[10] = {0xD0,0x04, 0x00,0x08, 0xC0,0x33, 0x22,0x44, 0xA0,0x00};
//  UCHAR      pdu[6] = {0xD0,0x04, 0xD0,0x04, 0xC0,0x33};
//   CONSOLE_OUT("Registering with prov_filter_add...\n");
//
//      /* Add the incoming Source Address to Filter */
//
//  net_proxy_server_filter_op
//   (
//           handle,
//              MS_PROXY_ADD_TO_FILTER_OPCODE,
//           pdu,
//           sizeof(pdu),
//               MS_FALSE
//      );
//}
#if (CFG_HEARTBEAT_MODE)
static void UI_register_heartbeat(void)
{
    API_RESULT retval;
    CONSOLE_OUT("Registering with heartbeat callback...\n");
    retval = MS_trn_heartbeat_register(UI_heartbeat_rcv_callback,UI_heartbeat_rcv_timeout_callback);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}
#endif

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
        Currently setting MT-MESH-SAMPLE-8 as Complete Device Name!
        This can be updated to each individual devices as per requirement.
    */
    UCHAR UI_brr_scanrsp_data[] =
    {
        0x0D, 0x09, 'P', 'H', 'Y', '-', 'M', 'S', 'H', 'L', 'I', 'G', 'H', 'T'
    };
    CONSOLE_OUT("\n Setting PHY MSH LIGHT as Complete Device Name!\n");
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

void proxy_dly_generic_onoff (void* args, UINT16 size)
{
    generic_onoff_set_pl(UI_generic_onoff.onoff);
}

//void proxy_dly_generic_hsl (void * args, UINT16 size)
//{
//    proxy_dly_thandle = EM_TIMER_HANDLE_INIT_VAL;
//    UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
//}


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
            retval=MS_access_bind_model_app(UI_generic_onoff_server_model_handle, handle);
            #ifdef  USE_LIGHTNESS
            retval=MS_access_bind_model_app(UI_light_lightness_server_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_light_lightness_server_model_handle,handle);
            #endif
            #ifdef  USE_CTL
            retval=MS_access_bind_model_app(UI_light_ctl_server_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_light_ctl_server_model_handle,handle);
            #endif
            #ifdef  USE_HSL
            retval=MS_access_bind_model_app(UI_light_hsl_server_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_light_hsl_server_model_handle,handle);
            #endif
            #ifdef  USE_SCENE
            retval=MS_access_bind_model_app(UI_scene_server_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_scene_server_model_handle,handle);
            #endif
            #ifdef  USE_VENDORMODEL
            retval=MS_access_bind_model_app(UI_vendor_defined_server_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_vendor_defined_server_model_handle,handle);
            #endif
        }
    }

    #ifdef EASY_BOUNDING
    //Provision ok,stop provision/config timeout handler  by hq
    CONSOLE_OUT("Stop timer\n");
    EM_stop_timer(&procfg_timer_handle);
    blebrr_prov_started = MS_FALSE;
    #endif
    return retval;
}

//---------------------------------------------------------------------------------
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

void vm_subscriptiong_add (MS_NET_ADDR addr)
{
    CONSOLE_OUT("vm_subscriptiong_add:%x\n",addr);
    MS_ACCESS_ADDRESS       sub_addr;
    sub_addr.use_label=0;
    sub_addr.addr=addr;
    MS_access_ps_store_disable(MS_TRUE);
    MS_access_cm_add_model_subscription(UI_generic_onoff_server_model_handle,&sub_addr);
    #ifdef  USE_LIGHTNESS
    MS_access_cm_add_model_subscription(UI_light_lightness_server_model_handle,&sub_addr);
    #endif
    #ifdef  USE_CTL
    MS_access_cm_add_model_subscription(UI_light_ctl_server_model_handle,&sub_addr);
    MS_access_cm_add_model_subscription(UI_light_ctl_setup_server_model_handle,&sub_addr);
    #endif
    #ifdef  USE_HSL
    MS_access_cm_add_model_subscription(UI_light_hsl_server_model_handle,&sub_addr);
    MS_access_cm_add_model_subscription(UI_light_hsl_setup_server_model_handle,&sub_addr);
    #endif
    #ifdef  USE_SCENE
    MS_access_cm_add_model_subscription(UI_scene_server_model_handle,&sub_addr);
    MS_access_cm_add_model_subscription(UI_scene_setup_server_model_handle,&sub_addr);
    #endif
    MS_access_ps_store_disable(MS_FALSE);
    MS_access_ps_store_all_record();
}

void vm_subscriptiong_delete (MS_NET_ADDR addr)
{
    MS_ACCESS_ADDRESS       sub_addr;
    sub_addr.use_label=0;
    sub_addr.addr=addr;
    MS_access_ps_store_disable(MS_TRUE);
    MS_access_cm_delete_model_subscription(UI_generic_onoff_server_model_handle,&sub_addr);
    #ifdef  USE_LIGHTNESS
    MS_access_cm_delete_model_subscription(UI_light_lightness_server_model_handle,&sub_addr);
    #endif
    #ifdef  USE_CTL
    MS_access_cm_delete_model_subscription(UI_light_ctl_server_model_handle,&sub_addr);
    MS_access_cm_delete_model_subscription(UI_light_ctl_setup_server_model_handle,&sub_addr);
    #endif
    #ifdef  USE_HSL
    MS_access_cm_delete_model_subscription(UI_light_hsl_server_model_handle,&sub_addr);
    MS_access_cm_delete_model_subscription(UI_light_hsl_setup_server_model_handle,&sub_addr);
    #endif
    #ifdef  USE_SCENE
    MS_access_cm_delete_model_subscription(UI_scene_server_model_handle,&sub_addr);
    MS_access_cm_delete_model_subscription(UI_scene_setup_server_model_handle,&sub_addr);
    #endif
    MS_access_ps_store_disable(MS_FALSE);
    MS_access_ps_store_all_record();
}

#if (CFG_HEARTBEAT_MODE)
API_RESULT UI_trn_set_heartbeat_subscription(MS_NET_ADDR saddr)
{
    API_RESULT retval;
    MS_TRN_HEARTBEAT_SUBSCRIPTION_INFO hb_sub_info;
    hb_sub_info.saddr = saddr;
    hb_sub_info.count_log = 0;
    hb_sub_info.daddr = 0xCFFF;
    hb_sub_info.max_hops = 0;
    hb_sub_info.min_hops = 0;
    hb_sub_info.period_log = 0x06;
    retval = MS_trn_set_heartbeat_subscription(&hb_sub_info);
    return retval;
}
#endif


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
        vm_subscriptiong_add(addr.addr);
        #endif
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE:
        #ifdef EASY_BOUNDING
        MS_UNPACK_LE_2_BYTE(&addr.addr, data_parm + 2);
        CONSOLE_OUT("[CONFIG] Subscription Address 0x%04X\n",addr.addr);
        CONSOLE_OUT("[MODEL SUBS DELETE]\n");
        vm_subscriptiong_delete(addr.addr);
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
        ms_provisioner_addr = saddr;
        vm_subscriptiong_binding_cb();
//        ms_provisioner_addr = saddr;
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
    /* Enable LED Port */
    /* Platform Abstraction Initializations of GPIOs/LEDs etc. */
    mesh_model_platform_init_pl();
    /* LED ON */
    /* LED ON/OFF for BOOT UP Indication Abstraction Call */
    mesh_model_device_bootup_ind_pl();
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
        /* Register Generic OnOff model server */
        retval = UI_register_generic_onoff_model_server(element_handle);
    }

    #ifdef  USE_HSL

    if (API_SUCCESS == retval)
    {
        /* Register Light Lightness model server */
        retval = UI_register_light_hsl_model_server(element_handle);
    }

    #endif
    #ifdef  USE_CTL

    if (API_SUCCESS == retval)
    {
        /* Register Light Lightness model server */
        retval = UI_register_light_ctl_model_server(element_handle);
    }

    #endif
    #ifdef  USE_SCENE

    if (API_SUCCESS == retval)
    {
        /* Register Light Scene model server */
        retval = UI_register_scene_model_server(element_handle);
    }

    #endif
    #ifdef  USE_VENDORMODEL

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_register_vendor_defined_model_server(element_handle);
    }

    #endif

    if (API_SUCCESS == retval)
    {
        /* Initialize model states */
        UI_model_states_initialization();
    }

    /* Configure as provisionee/device */
    UI_register_prov();
//      UI_prov_filter_add();
    #if (CFG_HEARTBEAT_MODE)
    UI_register_heartbeat();
    #endif
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

    // MS_access_cm_set_transmit_state(MS_RELAY_TX_STATE, (8<<3)|2);
    //MS_access_cm_set_transmit_state(MS_NETWORK_TX_STATE, (8<<3)|3);

    if (MS_TRUE == is_prov_req)
    {
        /* Start Provisioning over GATT here */
        /**
            setup <role:[1 - Device, 2 - Provisioner]> <bearer:[1 - Adv, 2 - GATT]
        */
        role = PROV_ROLE_DEVICE;
        brr  = PROV_BRR_GATT|PROV_BRR_ADV;  //PROV_BRR_ADV,PROV_BRR_GATT
        printf("Bearer type = 0x%02X(Bit0-adv, Bit1-GATT)\r\n", brr);
//        UI_prov_brr_handle = brr;
        /**
            Setting up an Unprovisioned Device over GATT
        */
        LIGHT_ONLY_RED_ON;
        blebrr_prov_started = MS_FALSE;
        UI_setup_prov(role, brr);
//        UI_prov_bind(brr, 0x00);
        //ms_access_ps_store(MS_PS_RECORD_SEQ_NUMBER);
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
                //for silab 2.0.0 app use NODE ID
                CONSOLE_OUT("\r\n Provisioned Device - Starting Proxy with NODE ID on Subnet 0x0000!\r\n");
                UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
                #if (CFG_HEARTBEAT_MODE)

                if(ms_provisioner_addr != 0)
                {
                    printf("sub ms_provisioner_addr 0x%04X\n",ms_provisioner_addr);
                    UI_trn_set_heartbeat_subscription(ms_provisioner_addr);
                }

                #endif
            }
            else
            {
                light_blink_set(LIGHT_GREEN, LIGHT_BLINK_SLOW,3);
                MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NODEID, BRR_BCON_ACTIVE);
                #if (CFG_HEARTBEAT_MODE)

                if(ms_provisioner_addr != 0)
                {
                    printf("sub ms_provisioner_addr 0x%04X\n",ms_provisioner_addr);
                    UI_trn_set_heartbeat_subscription(ms_provisioner_addr);
                }

                #endif
                CONSOLE_OUT("\r\n Provisioned Device!!!\r\n");
                /**
                    Do Nothing!
                    Already Scaning is Enabled at Start Up
                */
                blebrr_scan_enable();
            }
        }
        else
        {
            light_blink_set(LIGHT_BLUE, LIGHT_BLINK_FAST,5);

            //for silab 2.0.0 app use NODE ID
            if (MS_ENABLE == state)
            {
                UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
            }
        }
    }

    if((ms_iv_index.iv_expire_time!=0)&&(ms_iv_index.iv_expire_time!=0xffffffff))
    {
        MS_net_start_iv_update_timer(ms_iv_index.iv_update_state,MS_TRUE);
    }
}



