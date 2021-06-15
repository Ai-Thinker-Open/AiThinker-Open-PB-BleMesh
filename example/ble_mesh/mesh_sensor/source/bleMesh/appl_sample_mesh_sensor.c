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
 *  \file appl_sample_switch.c
 *
 *  Source File for Mesh SIG Model Client(OnOff/HSL/CTL/...) as SWITCH Standalone application 
 *  
 */

/*
 *  Copyright (C) 2018. Phyplus Ltd.
 *  All rights reserved.
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
/* For Persistent Storage */
#include "access_extern.h"
//#include "MS_sensor_api.h"
#include "sensor_server.h"
#include "sensorMonitor_task.h"
#include "EXT_cbtimer.h"

#undef USE_HSL                 // enable Light HSL server model
#undef USE_LIGHTNESS            // enable Light Lightness server model
#undef  USE_CTL                 // disable Light CTL server model
#undef USE_SCENE               // enable Light Scene server model

#define USE_SENSOR               // enable Sensor server model
#define USE_VENDORMODEL         // enable Light vendormodel server model

#ifdef USE_VENDORMODEL
#define  EASY_BOUNDING
#endif


/* Console Input/Output */
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)

#define VENDOR_PRODUCT_MAC_ADDR         0x4000


void appl_dump_bytes(UCHAR *buffer, UINT16 length);
void appl_mesh_sample (void);

API_RESULT UI_prov_callback
           (
               PROV_HANDLE * phandle,
               UCHAR         event_type,
               API_RESULT    event_result,
               void        * event_data,
               UINT16        event_datalen
           );
void UI_proxy_callback
     (
         NETIF_HANDLE       * handle,
         UCHAR                p_evt,
         UCHAR              * data_param,
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

/* --------------------------------------------- Global Definitions */
//vendor model server/client
#define MS_MODEL_ID_VENDOR_EXAMPLE_SERVER                         0x00000504


//vendor model opcode
#define MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE                         0x00D00405
#define MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE                         0x00D10405
#define MS_ACCESS_VENDOR_EXAMPLE_SET_UNACKNOWLEDGED_OPCODE          0x00D20405
#define MS_ACCESS_VENDOR_EXAMPLE_STATUS_OPCODE                      0x00D30405
#define MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE                    0x00D40405
#define MS_ACCESS_VENDOR_EXAMPLE_INDICATION_OPCODE                  0x00D50405
#define MS_ACCESS_VENDOR_EXAMPLE_CONFIRMATION_OPCODE                0x00D60405
#define MS_ACCESS_VENDOR_EXAMPLE_NOTIFY_OPCODE                      0x00D70405

//vendor model att
#define MS_STATE_VENDOR_HSL_T                                   0x0123
#define MS_STATE_VENDOR_ONOFF_T                                 0x0100
#define MS_STATE_VENDOR_LIGHTNESS_T                             0x0121
#define MS_STATE_VENDOR_CTL_T                                   0x0122
#define MS_STATE_VENDOR_MAINLIGHTONOFF_T                        0x0534
#define MS_STATE_VENDOR_BACKLIGHTONOFF_T                        0x0533
#define MS_STATE_VENDOR_MODENUMBER_T                            0xF004
#define MS_STATE_VENDOR_EVENT_INDICATE_T                        0xF009



EM_timer_handle thandle;
EM_timer_handle proxy_dly_thandle;
void proxy_dly_generic_onoff (void * args, UINT16 size);
void proxy_dly_generic_hsl (void * args, UINT16 size);
/* --------------------------------------------- Global Definitions */

typedef struct UI_state_scene_struct
{
    UINT32                          index;
    MS_STATE_LIGHT_HSL_STRUCT       light_hsl_state;
    MS_STATE_GENERIC_ONOFF_STRUCT   onoff_state;
} UI_STATE_SCENE_STRUCT;



/*****************************************************************************************/
typedef union
{
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

typedef union
{
    int16_t i16bit;
    uint8_t u8bit[2];
} axis1bit16_t;

typedef union
{
    int32_t i32bit[3];
    uint8_t u8bit[12];
} axis3bit32_t;

typedef union
{
    int32_t i32bit;
    uint8_t u8bit[4];
} axis1bit32_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} LIS2DH12_Axes_t;









/** Vendor Model specific state parameters in a request or response message */
typedef struct _MS_access_vendor_model_state_params
{
    /** State Type */
    UINT16 state_type;

    /** State pointer */
    void * state;

} MS_ACCESS_VENDOR_MODEL_STATE_PARAMS;


/**
 * Vendor Example Server application Asynchronous Notification Callback.
 *
 * Vendor Example Server calls the registered callback to indicate events occurred to the
 * application.
 *
 * \param [in] ctx           Context of the message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
typedef API_RESULT (* MS_VENDOR_EXAMPLE_SERVER_CB)
(
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT     * ctx,
    MS_ACCESS_MODEL_REQ_MSG_RAW         * msg_raw,
    MS_ACCESS_MODEL_REQ_MSG_T           * req_type,
    MS_ACCESS_VENDOR_MODEL_STATE_PARAMS * state_params,
    MS_ACCESS_MODEL_EXT_PARAMS          * ext_params

) DECL_REENTRANT;

typedef struct MS_state_vendor_example_struct
{
    UCHAR  value;

} MS_STATE_VENDOR_EXAMPLE_STRUCT;

/** -- Vendor Defined States */
static MS_STATE_VENDOR_EXAMPLE_STRUCT UI_vendor_example;


/* ----------------------------------------- External Global Variables */

/* ----------------------------------------- Exported Global Variables */


/* ----------------------------------------- Static Global Variables */
static DECL_CONST UINT32 vendor_example_server_opcode_list[] =
{
    MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_STATUS_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_INDICATION_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_CONFIRMATION_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_NOTIFY_OPCODE
};




static MS_VENDOR_EXAMPLE_SERVER_CB       vendor_example_server_UI_cb;





MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_generic_onoff_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_lightness_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_ctl_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_ctl_setup_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_hsl_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_light_hsl_setup_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_scene_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_scene_setup_server_model_handle;

MS_ACCESS_MODEL_HANDLE   UI_sensor_server_model_handle;
MS_ACCESS_MODEL_HANDLE	 UI_sensor_setup_server_model_handle;





/* ----------------------------------------- Functions */
/* Model Server - Foundation Models */

/* Health Server - Test Routines */
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
 * \brief Health Server application Asynchronous Notification Callback.
 *
 * \par Description
 * Health Server calls the registered callback to indicate events occurred to the
 * application.
 *
 * \param handle        Model Handle.
 * \param event_type    Health Server Event type.
 * \param event_param   Parameter associated with the event if any or NULL.
 * \param param_len     Size of the event parameter data. 0 if event param is NULL.
 */
static API_RESULT UI_health_server_cb
                  (
                      MS_ACCESS_MODEL_HANDLE * handle,
                      UINT8                    event_type,
                      UINT8                  * event_param,
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
    MS_HEALTH_SERVER_SELF_TEST * self_tests;
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



// ===========================  Generic OnOff Server Model Functions =============
/* ---- Generic OnOff States */
static MS_STATE_GENERIC_ONOFF_STRUCT UI_generic_onoff;

/* Generic OnOff Model state Initialization */
static void UI_generic_onoff_model_states_initialization(void)
{
    EM_mem_set(&UI_generic_onoff, 0, sizeof(UI_generic_onoff));
}

/* Generic OnOff Model Get Handler */
static API_RESULT UI_generic_onoff_model_state_get(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    API_RESULT retval;

    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_GENERIC_ONOFF_T:
    {
        MS_STATE_GENERIC_ONOFF_STRUCT * param_p;

        param_p = (MS_STATE_GENERIC_ONOFF_STRUCT *)param;

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
static API_RESULT UI_generic_onoff_model_state_set(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    API_RESULT retval;
    UCHAR  proxy_state;

    retval = API_SUCCESS;
    switch (state_t)
    {
    case MS_STATE_GENERIC_ONOFF_T:
    {
        MS_STATE_GENERIC_ONOFF_STRUCT * param_p;

        param_p = (MS_STATE_GENERIC_ONOFF_STRUCT *)param;

        /* Instantaneous Change */
        UI_generic_onoff.onoff = param_p->onoff;

        *param_p = UI_generic_onoff;
        CONSOLE_OUT("[state] current: 0x%02X\n", UI_generic_onoff.onoff);
        CONSOLE_OUT("[state] target: 0x%02X\n", UI_generic_onoff.target_onoff);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", UI_generic_onoff.transition_time);

        MS_proxy_fetch_state(&proxy_state);
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
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Generic_Onoff server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
static API_RESULT UI_generic_onoff_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT    * ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW        * msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T          * req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS       * state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS         * ext_params
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

        retval = UI_generic_onoff_model_state_set(state_params->state_type, 0, (MS_STATE_GENERIC_ONOFF_STRUCT *)state_params->state, 0);

        current_state_params.state_type = state_params->state_type;
        current_state_params.state = (MS_STATE_GENERIC_ONOFF_STRUCT *)state_params->state;
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



API_RESULT UI_light_hsl_model_state_get(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_LIGHT_HSL_T:
    {
        MS_STATE_LIGHT_HSL_STRUCT * param_p;

        param_p = (MS_STATE_LIGHT_HSL_STRUCT *)param;

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

API_RESULT UI_light_hsl_model_state_set(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    API_RESULT retval;
    UCHAR  proxy_state;

    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_LIGHT_HSL_T:
    {
        MS_STATE_LIGHT_HSL_STRUCT * param_p;

        param_p = (MS_STATE_LIGHT_HSL_STRUCT *)param;


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
        MS_proxy_fetch_state(&proxy_state);
        if(proxy_state==MS_PROXY_CONNECTED)
        {
            EM_start_timer (&proxy_dly_thandle,    (EM_TIMEOUT_MILLISEC | 350), proxy_dly_generic_hsl, NULL, 0);
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
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Light_Hsl server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
API_RESULT UI_light_hsl_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT    * ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW        * msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T          * req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS       * state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS         * ext_params
)
{
    MS_ACCESS_MODEL_STATE_PARAMS         current_state_params;
    MS_STATE_LIGHT_HSL_STRUCT            param;
    MS_STATE_LIGHT_HSL_RANGE_STRUCT      param_range;
    MS_STATE_LIGHT_HSL_DEFAULT_STRUCT    param_default;
    void                               * param_p;
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



API_RESULT UI_light_ctl_model_state_get(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT * param_p;

        param_p = (MS_STATE_LIGHT_CTL_STRUCT *)param;

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

API_RESULT UI_light_ctl_model_state_set(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    API_RESULT retval;

    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT * param_p;

        param_p = (MS_STATE_LIGHT_CTL_STRUCT *)param;


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
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Light_Hsl server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
API_RESULT UI_light_ctl_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT    * ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW        * msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T          * req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS       * state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS         * ext_params
)
{
    MS_ACCESS_MODEL_STATE_PARAMS                   current_state_params;

    MS_STATE_LIGHT_CTL_STRUCT                      light_ctl_params;
    MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_STRUCT    light_ctl_temperature_range_params;
    MS_STATE_LIGHT_CTL_DEFAULT_STRUCT              light_ctl_default_params;
    MS_STATE_LIGHT_CTL_TEMPERATURE_STRUCT          light_ctl_temperature_params;
    void                                         * param_p;

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
void UI_vendor_example_model_state_get(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    switch(state_t)
    {
    default:
        break;
    }
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
void UI_vendor_example_model_state_set(UINT16 state_t, UINT16 state_inst, void * param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_VENDOR_HSL_T:
    {
        MS_STATE_VENDOR_EXAMPLE_STRUCT * param_p;

        param_p = (MS_STATE_VENDOR_EXAMPLE_STRUCT *)param;

        switch(param_p->value)
        {
        case 0x00:
            UI_vendor_model_set_red();
            break;
        case 0x01:
            UI_vendor_model_set_green();
            break;
        case 0x02:
            UI_vendor_model_set_blue();
            break;
        case 0x03:
            generic_onoff_set_pl(0x00);
            break;
        default:
            break;
        }
    }
    break;
    default:
        break;
    }
}

/* ----------------------------------------- Functions */
/**
 *  \brief API to send reply or to update state change
 *
 *  \par Description
 *  This is to send reply for a request or to inform change in state.
 *
 * \param [in] ctx                     Context of the message.
 * \param [in] current_state_params    Model specific current state parameters.
 * \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
 * \param [in] remaining_time          Time from current state to target state (0: to be ignored).
 * \param [in] ext_params              Additional parameters (NULL: to be ignored).
 *
 *  \return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_vendor_example_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT    * ctx,
    /* IN */ MS_ACCESS_VENDOR_MODEL_STATE_PARAMS       * current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS       * target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS         * ext_params
)
{
    API_RESULT retval;

    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR    * pdu_ptr;
    UINT16     marker;
    UINT32     opcode;

    retval = API_FAILURE;
    marker = 0;

    CONSOLE_OUT(
        "[VENDOR_EXAMPLE_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
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

    retval = MS_access_reply
             (
                 &ctx->handle,
                 ctx->daddr,
                 ctx->saddr,
                 ctx->subnet_handle,
                 ctx->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 pdu_ptr,
                 marker
             );

    return retval;
}


/**
 * \brief Access Layer Application Asynchronous Notification Callback.
 *
 * \par Description
 * Access Layer calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] handle        Model Handle.
 * \param [in] saddr         16 bit Source Address.
 * \param [in] daddr         16 bit Destination Address.
 * \param [in] appkey_handle AppKey Handle.
 * \param [in] subnet_handle Subnet Handle.
 * \param [in] opcode        Opcode.
 * \param [in] data_param    Data associated with the event if any or NULL.
 * \param [in] data_len      Size of the event data. 0 if event data is NULL.
 */
API_RESULT vendor_example_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE * handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR                  * data_param,
    /* IN */ UINT16                   data_len
)
{
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT         req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW             req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T               req_type;

    MS_ACCESS_MODEL_EXT_PARAMS              * ext_params_p;

    MS_ACCESS_VENDOR_MODEL_STATE_PARAMS     state_params;
    uint16  marker = 0;

//    UINT16        marker;
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
        "[VENDOR_EXAMPLE_SERVER] Callback. Opcode 0x%06X\n", opcode);

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
        //state_params.state_type = MS_STATE_VENDOR_EXAMPLE_T;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(vendor_example_set_handler);

        state_params.state_type = *data_param|(*(data_param+1)<<8);
        state_params.state = data_param+2;

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x00;

    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_SET_UNACKNOWLEDGED_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_SET_UNACKNOWLEDGED_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(vendor_example_set_unacknowledged_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x00;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_STATUS\n");

        MODEL_OPCODE_HANDLER_CALL(vendor_example_status_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_OTHERS;
        req_type.to_be_acked = 0x00;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_WRITECMD_OPCODE:
    {
        uint16  message_index;
        uint32  osal_tick;
        uint16  src_addr;
//            CONSOLE_OUT(
//            "MS_ACCESS_VENDOR_EXAMPLE_INDICATION_WRITECMD\n");

        MS_UNPACK_LE_2_BYTE(&message_index, data_param+marker);
        marker += 2;
        MS_UNPACK_LE_4_BYTE(&osal_tick, data_param+marker);
        marker += 4;
        MS_UNPACK_LE_2_BYTE(&src_addr, data_param+marker);
        marker += 2;


        CONSOLE_OUT("[PDU_Rx] Pkt. INDEX:0x%04X, TICK:0x%08X, SRC:0x%04X\r\n",
                    message_index,(unsigned int)osal_tick,src_addr);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_OTHERS;
        req_type.to_be_acked = 0x00;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_INDICATION_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_INDICATION\n");

        MODEL_OPCODE_HANDLER_CALL(vendor_example_indication_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_OTHERS;
        req_type.to_be_acked = 0x00;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_CONFIRMATION_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_CONFIRMATION\n");

        MODEL_OPCODE_HANDLER_CALL(vendor_example_confirmation_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_OTHERS;
        req_type.to_be_acked = 0x00;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_NOTIFY_OPCODE:
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
    if (NULL != vendor_example_server_UI_cb)
    {
        vendor_example_server_UI_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
    }
           
    return retval;
}


/**
 *  \brief API to initialize Vendor_Example_1 Server model
 *
 *  \par Description
 *  This is to initialize Vendor_Example_1 Server model and to register with Acess layer.
 *
 *  \param [in] element_handle
 *  Element identifier to be associated with the model instance.
 *
 *  \param [in, out] model_handle
 *                   Model identifier associated with the model instance on successful initialization.
 *                   After power cycle of an already provisioned node, the model handle will have
 *                   valid value and the same will be reused for registration.
 *
 *  \param [in] UI_cb    Application Callback to be used by the Vendor_Example_1 Server.
 *
 *  \return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_vendor_example_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE    * model_handle,
    /* IN */    MS_VENDOR_EXAMPLE_SERVER_CB UI_cb
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
    model.model_id.id = MS_MODEL_ID_VENDOR_EXAMPLE_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
    model.elem_handle = element_handle;

    /* Register Callback */
    model.cb = vendor_example_server_cb;

    /* List of Opcodes */
    model.opcodes = vendor_example_server_opcode_list;
    model.num_opcodes = sizeof(vendor_example_server_opcode_list) / sizeof(UINT32);

    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );

    /* Save Application Callback */
    vendor_example_server_UI_cb = UI_cb;

    //    /* TODO: Remove */
    //    vendor_example_server_model_handle = *model_handle;
                      
    return retval;
}

/* Vendor Defined Model Server */
/**
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Vendor_Example_1 server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
API_RESULT UI_vendor_example_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT         * ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW             * msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T               * req_type,
    /* IN */ MS_ACCESS_VENDOR_MODEL_STATE_PARAMS     * state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS              * ext_params
)
{
    MS_STATE_VENDOR_EXAMPLE_STRUCT param;
    MS_ACCESS_VENDOR_MODEL_STATE_PARAMS                    current_state_params;

    API_RESULT retval;

    retval = API_SUCCESS;
    CONSOLE_OUT(
        "[VENDOR_EXAMPLE] %04x (d%04x s%04x).\n",ctx->handle,ctx->daddr,ctx->saddr);
    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] GET Request.\n");

        UI_vendor_example_model_state_get(state_params->state_type, 0, &param, 0);

        current_state_params.state_type = state_params->state_type;
        current_state_params.state = &param;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] SET Request.\n");

        UI_vendor_example_model_state_set(state_params->state_type, 0, (MS_STATE_VENDOR_EXAMPLE_STRUCT *)state_params->state, 0);

        current_state_params.state_type = state_params->state_type;
        current_state_params.state = (MS_STATE_VENDOR_EXAMPLE_STRUCT *)state_params->state;
        //vm_vendor_mode_indication(current_state_params);
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] Sending Response.\n");

        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_vendor_example_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
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

    retval = MS_vendor_example_server_init
             (
                 element_handle,
                 &UI_vendor_defined_server_model_handle,
                 UI_vendor_example_server_cb
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


/* Sensor server Model state Initialization */
static void UI_Sensor_Server_model_states_initialization(void)
{

  //  EM_mem_set(&UI_Sensor_Descrip_Infor, 0, sizeof(MS_STATE_SENSOR_DESCRIPTOR_STRUCT)*SENSOR_NUM);




}








/* Sensor servel Model Set Handler */
static API_RESULT UI_Sensor_servel_model_descriptor_get(UINT16 state_t, UINT16 id, uint8 * param, UINT16  *p_len)
{

    API_RESULT retval;

    UCHAR  sensor_index=0;
    uint16 tolerance;
	uint8 cnt=0;


    retval = API_SUCCESS;
    switch (state_t)
    {
    case MS_STATE_SENSOR_PROPERTY_ID_T:
    {
        if(id==0x0000) //prohibited
            break;

        for(sensor_index=0; sensor_index<SENSOR_NUM; sensor_index++)
        {
            if(UI_Sensor_Descrip_Infor[sensor_index].sensor_property_id==id)
            {

                tolerance=UI_Sensor_Descrip_Infor[sensor_index].sensor_negative_tolerance;
                tolerance=(tolerance<<12)|UI_Sensor_Descrip_Infor[sensor_index].sensor_positive_tolerance;


                *(param)   = UI_Sensor_Descrip_Infor[sensor_index].sensor_property_id;
                *(param+1) =  UI_Sensor_Descrip_Infor[sensor_index].sensor_property_id >> 8;
                *(param+2) = tolerance;
                *(param+3) = tolerance >> 8;
                *(param+4) = tolerance >> 16;
                *(param+5) = UI_Sensor_Descrip_Infor[sensor_index].sensor_sampling_function;
                *(param+6) = UI_Sensor_Descrip_Infor[sensor_index].sensor_measurement_period;
                *(param+7) = UI_Sensor_Descrip_Infor[sensor_index].sensor_update_interval;

                *p_len=8;
                break;
            }

        }

    }
    break;
    case MS_STATE_SENSOR_DESCRIPTOR_T:
    {
        cnt=0;
        // update all sensor description
        for(sensor_index=0; sensor_index<SENSOR_NUM; sensor_index++)
        {


            tolerance=UI_Sensor_Descrip_Infor[sensor_index].sensor_negative_tolerance;
            tolerance=(tolerance<<12)|UI_Sensor_Descrip_Infor[sensor_index].sensor_positive_tolerance;


            param[cnt]   = UI_Sensor_Descrip_Infor[sensor_index].sensor_property_id;
			cnt++;
            param[cnt]=  UI_Sensor_Descrip_Infor[sensor_index].sensor_property_id >> 8;
			cnt++;
            param[cnt] = tolerance;
			cnt++;
            param[cnt]= tolerance >> 8;
			cnt++;
            param[cnt] = tolerance >> 16;
			cnt++;
            param[cnt] = UI_Sensor_Descrip_Infor[sensor_index].sensor_sampling_function;
			cnt++;
            param[cnt] = UI_Sensor_Descrip_Infor[sensor_index].sensor_measurement_period;
			cnt++;
            param[cnt] = UI_Sensor_Descrip_Infor[sensor_index].sensor_update_interval;
			cnt++;

            *p_len+=8;

			//appl_dump_bytes((*char)(UI_Sensor_Descrip_Infor+sensor_index), 8);

        }

		CONSOLE_OUT("XXX %d=",*p_len);
				for(uint8 m=0;m<*p_len;m++)
				{
		            CONSOLE_OUT(
		        "%X,",param[m]);
				}
    }
    break;

    default:
        break;
    }

    return retval;

}



static API_RESULT UI_Sensor_servel_model_sensorStatus_get(UINT16 state_t, UINT16 id, uint8 * param, UINT16  *p_len)
{

    API_RESULT retval;

    UCHAR  sensor_index=0;
    uint8 data_len;
    retval = API_SUCCESS;

//there are tow format message:
// format A: length<=16,  property ID<2048(0X800)
// fromat B: length<=127, property ID<65526
// SIG spec: property values longer than 128 octets are not supporty by the sensor status message
    static LIS2DH12_Axes_t data_raw_Acceleration;
    uint16 humidity_data;
    uint16 Volt_data;

    //for test example
    data_raw_Acceleration.x=100+rand()%100+rand()%3*1000;;
    data_raw_Acceleration.y=200+rand()%100+rand()%3*1000;
    data_raw_Acceleration.z=300+rand()%100+rand()%3*1000;

    humidity_data=cht8305_humi;
    Volt_data=3500+rand()%200;


    switch (state_t)
    {
    case MS_STATE_SENSOR_DATA_PROPERTY_ID_T:
    {
        if(id==0x0000) //prohibited
            break;

        switch (id)
        {
        case VOLTAGE_PID:
        {
            //format A:
            data_len=0x02;
            *param      = ((VOLTAGE_PID & 0x07) << 5) | (data_len <<1) ;
            *(param+1)  = (VOLTAGE_PID >>3)&0XFF;
            osal_memcpy(&param[2], &Volt_data, 2);

            *p_len=4;
        }
        break;
        case ACCELERO_METER_PID:
        {
            //format B:
            data_len=6;
            *param      = ((data_len <<1) | 0x01);
            *(param+1)  = (uint8)ACCELERO_METER_PID;
            *(param+2)  = (uint8)(ACCELERO_METER_PID>>8);
            osal_memcpy(&param[3], &data_raw_Acceleration.x, 2);
            osal_memcpy(&param[5], &data_raw_Acceleration.y, 2);
            osal_memcpy(&param[7], &data_raw_Acceleration.z, 2);

            *p_len=9;

        }
        break;
        case HUMIDITY_PID:
        {
            //format B:
            data_len=0x02;
            *param      = ((data_len <<1) | 0x01);
            *(param+1)  = (uint8)HUMIDITY_PID;
            *(param+2)  = (uint8)(HUMIDITY_PID>>8);
            osal_memcpy(&param[3], &humidity_data, 2);

            *p_len=5;

        }
        break;


        default:
            break;

        }

    }
    break;
    case  MS_STATE_SENSOR_DATA_T:
    {




        data_len=0x02;
        sensor_index=0;
        *(param+sensor_index)     = ((VOLTAGE_PID & 0x07) << 5) | (data_len <<1) ;

        sensor_index++;
        *(param+sensor_index)  = (VOLTAGE_PID >>3)&0XFF;

        sensor_index++;
        osal_memcpy(&param[sensor_index], &Volt_data, 2);

        sensor_index=sensor_index+2;


		 data_len=6;
        *(param+sensor_index)    = ((data_len <<1) | 0x01);

        sensor_index++;
        *(param+sensor_index)  = (uint8)ACCELERO_METER_PID;
        sensor_index++;
        *(param+sensor_index)  = (uint8)(ACCELERO_METER_PID>>8);


        sensor_index++;
        osal_memcpy(&param[sensor_index], &data_raw_Acceleration.x, 2);
        sensor_index=sensor_index+2;
        osal_memcpy(&param[sensor_index], &data_raw_Acceleration.y, 2);
        sensor_index=sensor_index+2;
        osal_memcpy(&param[sensor_index], &data_raw_Acceleration.z, 2);

        sensor_index=sensor_index+2;

		data_len=2;
        *(param+sensor_index)      = ((data_len <<1) | 0x01);

        sensor_index++;
        *(param+sensor_index)  = (uint8)HUMIDITY_PID;

        sensor_index++;
        *(param+sensor_index)  = (uint8)(HUMIDITY_PID>>8);
        sensor_index++;
        osal_memcpy(&param[sensor_index], &humidity_data, 2);

        sensor_index=sensor_index+2;

        *p_len=sensor_index;
    }
    break;

    default:
        break;
    }

    return retval;


}

UINT32 UI_sensor_search_column_value_x(uint8 *value_x,uint8 len)
{
    UINT32 result=0;
    UINT8 i=0;
    for(i=0; i<len; i++)
    {
        result|=value_x[i]<<(i*8);
    }

    return result;

}


/* Sensor servel Model Set Handler */
static API_RESULT UI_Sensor_servel_model_column_get(MS_ACCESS_SENSOR_MODEL_STATE_PARAMS *state_param, UINT16 id, uint8 * param, UINT16  *p_len)
{

    API_RESULT retval;
    uint8 i=0;
    UCHAR index=0;
    uint8 column_index=0;

    uint8 *pValue=state_param->state;

    UINT32 Value_x_Identify=0;
    UINT32 Value_x_Base=0;

    for( i=0; i<(state_param->len-2); i++)
    {
        Value_x_Identify|=(pValue[i+2]<<(i*8));

    }


    retval = API_SUCCESS;
    switch (id)
    {
    case VOLTAGE_PID:
    {

        for(i=0; i<VOL_COLUMN_NUM; i++)
        {
            Value_x_Base=UI_sensor_search_column_value_x(UI_Vol_Sensor_series_column[i].sensor_raw_value_x,\
                         UI_Vol_Sensor_series_column[i].sensor_raw_value_x_len);
            if(Value_x_Base!=0&&Value_x_Base==Value_x_Identify)
            {
                column_index=i;
                break;
            }

        }

        if(Value_x_Base==0)
            break;
        index=0;
        *(param+index)   = (uint8)VOLTAGE_PID;
        index++;
        *(param+index) = (uint8)(VOLTAGE_PID>>8);
        index++;

        osal_memcpy((uint8*)param[index],(uint8*)UI_Vol_Sensor_series_column[column_index].sensor_raw_value_x,\
                    UI_Vol_Sensor_series_column[column_index].sensor_raw_value_x_len);

        index=index+UI_Vol_Sensor_series_column[column_index].sensor_raw_value_x_len;

        osal_memcpy((uint8*)param[index],(uint8*)UI_Vol_Sensor_series_column[column_index].sensor_column_width,\
                    UI_Vol_Sensor_series_column[column_index].sensor_column_width_len);

        index=index+UI_Vol_Sensor_series_column[column_index].sensor_column_width_len;

        osal_memcpy((uint8*)param[index],(uint8*)UI_Vol_Sensor_series_column[column_index].sensor_raw_value_y,\
                    UI_Vol_Sensor_series_column[column_index].sensor_raw_value_y_len);

        index=index+UI_Vol_Sensor_series_column[column_index].sensor_raw_value_y_len;

        *p_len=index;
    }
    break;
    case ACCELERO_METER_PID:
    {
        for(i=0; i<ACC_COLUMN_NUM; i++)
        {
            Value_x_Base=UI_sensor_search_column_value_x(UI_ACC_Sensor_series_column[i].sensor_raw_value_x,\
                         UI_ACC_Sensor_series_column[i].sensor_raw_value_x_len);
            if(Value_x_Base!=0&&Value_x_Base==Value_x_Identify)
            {
                column_index=i;
                break;
            }

        }

        if(Value_x_Base==0)
            break;
        index=0;
        *(param+index)   = (uint8)ACCELERO_METER_PID;
        index++;
        *(param+index) = (uint8)(ACCELERO_METER_PID>>8);
        index++;

        osal_memcpy((uint8*)param[index],(uint8*)UI_ACC_Sensor_series_column[column_index].sensor_raw_value_x,\
                    UI_ACC_Sensor_series_column[column_index].sensor_raw_value_x_len);

        index=index+UI_ACC_Sensor_series_column[column_index].sensor_raw_value_x_len;

        osal_memcpy((uint8*)param[index],(uint8*)UI_ACC_Sensor_series_column[column_index].sensor_column_width,\
                    UI_ACC_Sensor_series_column[column_index].sensor_column_width_len);

        index=index+UI_ACC_Sensor_series_column[column_index].sensor_column_width_len;

        osal_memcpy((uint8*)param[index],(uint8*)UI_ACC_Sensor_series_column[column_index].sensor_raw_value_y,\
                    UI_ACC_Sensor_series_column[column_index].sensor_raw_value_y_len);

        index=index+UI_ACC_Sensor_series_column[column_index].sensor_raw_value_y_len;

        *p_len=index;

    }
    break;
    case HUMIDITY_PID:
    {
        for(i=0; i<HUMITY_COLUMN_NUM; i++)
        {
            Value_x_Base=UI_sensor_search_column_value_x(UI_Humity_Sensor_series_column[i].sensor_raw_value_x,\
                         UI_Humity_Sensor_series_column[i].sensor_raw_value_x_len);
            if(Value_x_Base!=0&&Value_x_Base==Value_x_Identify)
            {
                column_index=i;
                break;
            }

        }

        if(Value_x_Base==0)
            break;
        index=0;
        *(param+index)   = (uint8)HUMIDITY_PID;
        index++;
        *(param+index) = (uint8)(HUMIDITY_PID>>8);
        index++;

        osal_memcpy((uint8*)param[index],(uint8*)UI_Humity_Sensor_series_column[column_index].sensor_raw_value_x,\
                    UI_Humity_Sensor_series_column[column_index].sensor_raw_value_x_len);

        index=index+UI_Humity_Sensor_series_column[column_index].sensor_raw_value_x_len;

        osal_memcpy((uint8*)param[index],(uint8*)UI_Humity_Sensor_series_column[column_index].sensor_column_width,\
                    UI_Humity_Sensor_series_column[column_index].sensor_column_width_len);

        index=index+UI_Humity_Sensor_series_column[column_index].sensor_column_width_len;

        osal_memcpy((uint8*)param[index],(uint8*)UI_Humity_Sensor_series_column[column_index].sensor_raw_value_y,\
                    UI_Humity_Sensor_series_column[column_index].sensor_raw_value_y_len);

        index=index+UI_Humity_Sensor_series_column[column_index].sensor_raw_value_y_len;

        *p_len=index;
    }
    break;

    default:
        break;
    }

    return retval;

}



static API_RESULT UI_Sensor_servel_model_Sensor_ColumSeries_get(MS_ACCESS_SENSOR_MODEL_STATE_PARAMS * state_param, UINT16 id, uint8 * param, UINT16  *p_len)
{

    API_RESULT retval;

    UCHAR  index=0;
    retval = API_SUCCESS;

    uint8 *pValue=state_param->state;

    UINT32 Value_x_Base=0;

    UINT32 Value_x_Start=0;
    UINT32 Value_x_End=0;

    uint8  start_index=0;
    uint8  end_index=0;

    uint8  section_flag=0;

    uint8 i=0;

    switch (id)
    {
    case VOLTAGE_PID:
    {
        if(state_param->len>2)
        {
            for(i=0; i<VOL_VALUE_LEN; i++)
            {
                Value_x_Start|=(pValue[i+2]<<(i*8));
                Value_x_End|=(pValue[i+2+VOL_VALUE_LEN]<<(i*8));
            }
            section_flag=1;
        }

        if(section_flag==0)
        {
            start_index=0;
            end_index=VOL_COLUMN_NUM;
        }
        else if(section_flag==1)
        {
            for(i=0; i<VOL_VALUE_LEN; i++)
            {
                Value_x_Base=UI_sensor_search_column_value_x(UI_Vol_Sensor_series_column[i].sensor_raw_value_x,\
                             UI_Vol_Sensor_series_column[i].sensor_raw_value_x_len);
                if(Value_x_Base!=0&&Value_x_Base==Value_x_Start)
                {
                    start_index=i;
                    break;
                }
            }

            for(i=0; i<VOL_VALUE_LEN; i++)
            {
                Value_x_Base=UI_sensor_search_column_value_x(UI_Vol_Sensor_series_column[i].sensor_raw_value_x,\
                             UI_Vol_Sensor_series_column[i].sensor_raw_value_x_len);
                if(Value_x_Base!=0&&Value_x_Base==Value_x_End)
                {
                    end_index=i+1;
                    break;
                }
            }

        }

        index=0;
        for(i=start_index; i<end_index; i++)
        {
            *(param+index)   = (uint8)VOLTAGE_PID;
            index++;
            *(param+index) = (uint8)(VOLTAGE_PID>>8);
            index++;

            osal_memcpy((uint8*)param[index],(uint8*)UI_Vol_Sensor_series_column[i].sensor_raw_value_x,\
                        UI_Vol_Sensor_series_column[i].sensor_raw_value_x_len);

            index=index+UI_Vol_Sensor_series_column[i].sensor_raw_value_x_len;

            osal_memcpy((uint8*)param[index],(uint8*)UI_Vol_Sensor_series_column[i].sensor_column_width,\
                        UI_Vol_Sensor_series_column[i].sensor_column_width_len);

            index=index+UI_Vol_Sensor_series_column[i].sensor_column_width_len;

            osal_memcpy((uint8*)param[index],(uint8*)UI_Vol_Sensor_series_column[i].sensor_raw_value_y,\
                        UI_Vol_Sensor_series_column[i].sensor_raw_value_y_len);

            index=index+UI_Vol_Sensor_series_column[i].sensor_raw_value_y_len;

            *p_len=index;


        }

    }
    break;

    case HUMIDITY_PID:
    {
        if(state_param->len>2)
        {
            for(i=0; i<HUMITY_VALUE_LEN; i++)
            {
                Value_x_Start|=(pValue[i+2]<<(i*8));
                Value_x_End|=(pValue[i+2+HUMITY_VALUE_LEN]<<(i*8));
            }
            section_flag=1;
        }

        if(section_flag==0)
        {
            start_index=0;
            end_index=HUMITY_COLUMN_NUM;
        }
        else if(section_flag==1)
        {
            for(i=0; i<HUMITY_VALUE_LEN; i++)
            {
                Value_x_Base=UI_sensor_search_column_value_x(UI_Humity_Sensor_series_column[i].sensor_raw_value_x,\
                             UI_Humity_Sensor_series_column[i].sensor_raw_value_x_len);
                if(Value_x_Base!=0&&Value_x_Base==Value_x_Start)
                {
                    start_index=i;
                    break;
                }
            }

            for(i=0; i<HUMITY_VALUE_LEN; i++)
            {
                Value_x_Base=UI_sensor_search_column_value_x(UI_Humity_Sensor_series_column[i].sensor_raw_value_x,\
                             UI_Humity_Sensor_series_column[i].sensor_raw_value_x_len);
                if(Value_x_Base!=0&&Value_x_Base==Value_x_End)
                {
                    end_index=i+1;
                    break;
                }
            }

        }

        index=0;
        for(i=start_index; i<end_index; i++)
        {
            *(param+index)   = (uint8)HUMIDITY_PID;
            index++;
            *(param+index) = (uint8)(HUMIDITY_PID>>8);
            index++;

            osal_memcpy((uint8*)param[index],(uint8*)UI_Humity_Sensor_series_column[i].sensor_raw_value_x,\
                        UI_Humity_Sensor_series_column[i].sensor_raw_value_x_len);

            index=index+UI_Humity_Sensor_series_column[i].sensor_raw_value_x_len;

            osal_memcpy((uint8*)param[index],(uint8*)UI_Humity_Sensor_series_column[i].sensor_column_width,\
                        UI_Humity_Sensor_series_column[i].sensor_column_width_len);

            index=index+UI_Humity_Sensor_series_column[i].sensor_column_width_len;

            osal_memcpy((uint8*)param[index],(uint8*)UI_Humity_Sensor_series_column[i].sensor_raw_value_y,\
                        UI_Humity_Sensor_series_column[i].sensor_raw_value_y_len);

            index=index+UI_Humity_Sensor_series_column[i].sensor_raw_value_y_len;

            *p_len=index;


        }
    }
    break;
    case ACCELERO_METER_PID:
    {
        if(state_param->len>2)
        {
            for(i=0; i<ACC_VALUE_LEN; i++)
            {
                Value_x_Start|=(pValue[i+2]<<(i*8));
                Value_x_End|=(pValue[i+2+ACC_VALUE_LEN]<<(i*8));
            }
            section_flag=1;
        }

        if(section_flag==0)
        {
            start_index=0;
            end_index=ACC_COLUMN_NUM;
        }
        else if(section_flag==1)
        {
            for(i=0; i<ACC_VALUE_LEN; i++)
            {
                Value_x_Base=UI_sensor_search_column_value_x(UI_ACC_Sensor_series_column[i].sensor_raw_value_x,\
                             UI_ACC_Sensor_series_column[i].sensor_raw_value_x_len);
                if(Value_x_Base!=0&&Value_x_Base==Value_x_Start)
                {
                    start_index=i;
                    break;
                }
            }

            for(i=0; i<ACC_VALUE_LEN; i++)
            {
                Value_x_Base=UI_sensor_search_column_value_x(UI_ACC_Sensor_series_column[i].sensor_raw_value_x,\
                             UI_ACC_Sensor_series_column[i].sensor_raw_value_x_len);
                if(Value_x_Base!=0&&Value_x_Base==Value_x_End)
                {
                    end_index=i+1;
                    break;
                }
            }

        }

        index=0;
        for(i=start_index; i<end_index; i++)
        {
            *(param+index)   = (uint8)ACCELERO_METER_PID;
            index++;
            *(param+index) = (uint8)(ACCELERO_METER_PID>>8);
            index++;

            osal_memcpy((uint8*)param[index],(uint8*)UI_ACC_Sensor_series_column[i].sensor_raw_value_x,\
                        UI_ACC_Sensor_series_column[i].sensor_raw_value_x_len);

            index=index+UI_ACC_Sensor_series_column[i].sensor_raw_value_x_len;

            osal_memcpy((uint8*)param[index],(uint8*)UI_ACC_Sensor_series_column[i].sensor_column_width,\
                        UI_ACC_Sensor_series_column[i].sensor_column_width_len);

            index=index+UI_ACC_Sensor_series_column[i].sensor_column_width_len;

            osal_memcpy((uint8*)param[index],(uint8*)UI_ACC_Sensor_series_column[i].sensor_raw_value_y,\
                        UI_ACC_Sensor_series_column[i].sensor_raw_value_y_len);

            index=index+UI_ACC_Sensor_series_column[i].sensor_raw_value_y_len;

            *p_len=index;


        }


    }
    break;

    default:
        break;

    }


    return retval;
}


static API_RESULT UI_Sensor_setup_servel_model_cadence_get(UINT16 state_t, UINT16 id, uint8 * param, UINT16  *p_len)
{

    API_RESULT retval;

    UCHAR  sensor_index=0;
    uint8  divisor_and_type;
    uint8  cnt=0;


    retval = API_SUCCESS;
    switch (state_t)
    {
    case MS_STATE_SENSOR_CADENCE_T:
    {
        if(id==0x0000) //prohibited
            break;

        for(sensor_index=0; sensor_index<SENSOR_CADENCE_NUMBER; sensor_index++)
        {
            if(Sensor_CadenceSetting[sensor_index].sensor_property_id==id)
            {
                cnt=0;

                divisor_and_type=(Sensor_CadenceSetting[sensor_index].fast_cadence_period_divisor<<1)&0xFE;
                divisor_and_type=divisor_and_type|(Sensor_CadenceSetting[sensor_index].status_trigger_type&0x01);


                *(param+cnt)   = Sensor_CadenceSetting[sensor_index].sensor_property_id;
                   cnt++;
                *(param+cnt) =  Sensor_CadenceSetting[sensor_index].sensor_property_id >> 8;
							     cnt++;
                *(param+cnt) = divisor_and_type;
                  cnt++;
                *(param+cnt) = *Sensor_CadenceSetting[sensor_index].status_trigger_delta_down;				
                   cnt++;
                *(param+cnt) = *Sensor_CadenceSetting[sensor_index].status_trigger_delta_up;
                   cnt++;
                *(param+cnt) = Sensor_CadenceSetting[sensor_index].status_min_interval;
                  cnt++;
                osal_memcpy((uint8*)param[cnt],(uint8*)Sensor_CadenceSetting[sensor_index].fast_cadence_low,\
                            Sensor_CadenceSetting[sensor_index].fast_cadence_low_len);
                  cnt+=Sensor_CadenceSetting[sensor_index].fast_cadence_low_len;

                osal_memcpy((uint8*)param[cnt],(uint8*)Sensor_CadenceSetting[sensor_index].fast_cadence_high,\
                            Sensor_CadenceSetting[sensor_index].fast_cadence_high_len);
                  cnt+=Sensor_CadenceSetting[sensor_index].fast_cadence_high_len;

                *p_len=cnt;
                break;
            }

        }

    }
    break;

    default:
        break;
    }

  return retval;

}



void UI_Sensor_setup_server_cadence_set(UINT16 state_t, UINT16 id, MS_ACCESS_MODEL_REQ_MSG_RAW * msg_rawData, UINT8 direction)
{
 uint8 divisor_and_type=0;
 uint8 index=2;
 
  if(id == VOLTAGE_PID)
  {
    divisor_and_type= msg_rawData->data_param[index];
    Sensor_CadenceSetting[0].fast_cadence_period_divisor = divisor_and_type>>1;
    Sensor_CadenceSetting[0].status_trigger_type = divisor_and_type&0x01 ;
	index++;
    *Sensor_CadenceSetting[0].status_trigger_delta_down = msg_rawData->data_param[index];
	index++;
    *Sensor_CadenceSetting[0].status_trigger_delta_up =msg_rawData->data_param[index];
	index++;
    Sensor_CadenceSetting[0].status_min_interval = msg_rawData->data_param[index];
	index++;
	osal_memcmp(Sensor_CadenceSetting[0].fast_cadence_low, (uint8*)msg_rawData->data_param[index], VOL_CADENCE_LEN);
	index+=VOL_CADENCE_LEN;

	osal_memcmp(Sensor_CadenceSetting[0].fast_cadence_high, (uint8*)msg_rawData->data_param[index], VOL_CADENCE_LEN);
	index+=VOL_CADENCE_LEN;

  }
  else if(id == HUMIDITY_PID)
  {
    divisor_and_type= msg_rawData->data_param[index];
    Sensor_CadenceSetting[1].fast_cadence_period_divisor = divisor_and_type>>1;
    Sensor_CadenceSetting[1].status_trigger_type = divisor_and_type&0x01 ;
	index++;
    *Sensor_CadenceSetting[1].status_trigger_delta_down = msg_rawData->data_param[index];
	index++;
    *Sensor_CadenceSetting[1].status_trigger_delta_up =msg_rawData->data_param[index];
	index++;
    Sensor_CadenceSetting[1].status_min_interval = msg_rawData->data_param[index];
	index++;
	osal_memcmp(Sensor_CadenceSetting[1].fast_cadence_low, (uint8*)msg_rawData->data_param[index], HUMITY_CADENCE_LEN);
	index+=HUMITY_CADENCE_LEN;

	osal_memcmp(Sensor_CadenceSetting[1].fast_cadence_high, (uint8*)msg_rawData->data_param[index], HUMITY_CADENCE_LEN);
	index+=HUMITY_CADENCE_LEN;
   
  }
  else
  {

  }




}



static API_RESULT UI_Sensor_setup_servel_model_settings_get(UINT16 state_t, UINT16 id, uint8 * param, UINT16  *p_len)
{

	API_RESULT retval;


	uint8  cnt=0;

   retval = API_SUCCESS;
	if(id==HUMIDITY_PID)
	{
		cnt=0;
		*(param+cnt)   = humity_sensor_settings.sensor_property_id;
		cnt++;
		*(param+cnt) =  humity_sensor_settings.sensor_property_id >> 8;
		cnt++;
		osal_memcmp((uint8*)param[cnt], (uint8*)humity_sensor_settings.setting_property_ids, sizeof(UINT16)*humity_sensor_settings.setting_property_ids_count);
		cnt+=sizeof(UINT16)*humity_sensor_settings.setting_property_ids_count;
		*p_len=cnt;
	}

	return retval;
}


static API_RESULT UI_Sensor_setup_servel_model_setting_get(MS_ACCESS_SENSOR_MODEL_STATE_PARAMS *state_param, UINT16 id, uint8 * param, UINT16  *p_len)
{

	API_RESULT retval;

	UCHAR  sensor_index=0;
	uint8  cnt=0;
	retval = API_SUCCESS;

	uint8 *pValue=state_param->state;
	uint16 setting_pid=0;
	setting_pid=pValue[2]|(pValue[3]<<8);

	if(id==HUMIDITY_PID)
	{
       for(sensor_index=0;sensor_index<SENSOR_SETTINGS_NUM;sensor_index++)
       {
        if(setting_pid==humity_sensor_setting[sensor_index].sensor_setting_property_id)
			break;
	   }
	   
	
		cnt=0;
		*(param+cnt)   =(uint8)HUMIDITY_PID ;
		cnt++;
		*(param+cnt) =  HUMIDITY_PID>> 8;
		cnt++;
		*(param+cnt)   =setting_pid ;
		cnt++;
		*(param+cnt) =  setting_pid>> 8;
		cnt++;
		*(param+cnt)= humity_sensor_setting[sensor_index].sensor_setting_access;
		
		osal_memcmp((uint8*)param[cnt], (uint8*)humity_sensor_setting[sensor_index].sensor_setting_raw, sizeof(uint8)*humity_sensor_setting[sensor_index].sensor_setting_raw_len);
		cnt+=sizeof(uint8)*humity_sensor_setting[sensor_index].sensor_setting_raw_len;
		
		*p_len=cnt;
	}

	return retval;
}


static API_RESULT UI_Sensor_setup_servel_model_setting_set(MS_ACCESS_SENSOR_MODEL_STATE_PARAMS *state_param, UINT16 id, uint8 * param, UINT8  direct)
{

	API_RESULT retval;

	UCHAR  sensor_index=0;

	retval = API_SUCCESS;

	uint8 *pValue=state_param->state;
	uint16 setting_pid=0;
	setting_pid=pValue[2]|(pValue[3]<<8);

	if(id==HUMIDITY_PID)
	{
       for(sensor_index=0;sensor_index<SENSOR_SETTINGS_NUM;sensor_index++)
       {
        if(setting_pid==humity_sensor_setting[sensor_index].sensor_setting_property_id)
			break;
	   }
    
	 osal_memcmp((uint8*)humity_sensor_setting[sensor_index].sensor_setting_raw,(uint8*)pValue[4],\
	               sizeof(uint8)*humity_sensor_setting[sensor_index].sensor_setting_raw_len);
		
	}

	return retval;
}



/* ----------------------------------------- Functions */
/**
 *  \brief API to send reply or to update state change
 *
 *  \par Description
 *  This is to send reply for a request or to inform change in state.
 *
 * \param [in] ctx                     Context of the message.
 * \param [in] current_state_params    Model specific current state parameters.
 * \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
 * \param [in] remaining_time          Time from current state to target state (0: to be ignored).
 * \param [in] ext_params              Additional parameters (NULL: to be ignored).
 *
 *  \return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_Sensor_server_model_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT    * ctx,
    /* IN */ MS_ACCESS_SENSOR_MODEL_STATE_PARAMS       * current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS       * target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS         * ext_params
)
{
    API_RESULT retval;

    /* TODO: Check what should be maximum length */
    UCHAR      buffer[256];
    UCHAR    * pdu_ptr;
    UINT16     marker;
    UINT32     opcode;

    retval = API_FAILURE;
    marker = 0;

  

    switch (current_state_params->state_type)
    {

    case MS_STATE_SENSOR_PROPERTY_ID_T:
    case MS_STATE_SENSOR_DESCRIPTOR_T:
    {

        opcode=MS_ACCESS_SENSOR_DESCRIPTOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_DATA_T:
    case MS_STATE_SENSOR_DATA_PROPERTY_ID_T:
    {
        opcode=MS_ACCESS_SENSOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_COLUMN_STATUS_T:
    {
        opcode=MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE;
    }
    break;


    case MS_STATE_SENSOR_SERIES_COLUMN_T:
    {
        opcode=MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE;
    }
    break;

    default:
        break;

    }


    if(current_state_params->len>0&&current_state_params->len<=256)
    {
        osal_memcpy(buffer, current_state_params->state, current_state_params->len);
        marker=current_state_params->len;
    }
    else
        marker=0;

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
		CONSOLE_OUT("send: %d=",marker);
				for(uint8 m=0;m<marker;m++)
				{
		            CONSOLE_OUT(
		        "%X,",buffer[m]);
				}
    }

    retval = MS_access_reply
             (
                 &ctx->handle,
                 ctx->daddr,
                 ctx->saddr,
                 ctx->subnet_handle,
                 ctx->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 pdu_ptr,
                 marker
             );

    return retval;
}

/* ----------------------------------------- Functions */
/**
 *  \brief API to send reply or to update state change
 *
 *  \par Description
 *  This is to send reply for a request or to inform change in state.
 *
 * \param [in] ctx                     Context of the message.
 * \param [in] current_state_params    Model specific current state parameters.
 * \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
 * \param [in] remaining_time          Time from current state to target state (0: to be ignored).
 * \param [in] ext_params              Additional parameters (NULL: to be ignored).
 *
 *  \return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_Sensor_server_model_state_publish
(
    /* IN */ MS_ACCESS_SENSOR_MODEL_STATE_PARAMS       * current_state_params
)
{
    API_RESULT retval;

    /* TODO: Check what should be maximum length */
    UCHAR      buffer[256];
    UCHAR    * pdu_ptr;
    UINT16     marker;
    UINT32     opcode;

    retval = API_FAILURE;
    marker = 0;

  
    switch (current_state_params->state_type)
    {

    

    case MS_STATE_SENSOR_DATA_T:
    case MS_STATE_SENSOR_DATA_PROPERTY_ID_T:
    {
        opcode=MS_ACCESS_SENSOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_COLUMN_STATUS_T:
    {
        opcode=MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE;
    }
    break;


    case MS_STATE_SENSOR_SERIES_COLUMN_T:
    {
        opcode=MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE;
    }
    break;

    default:
        break;

    }


    if(current_state_params->len>0&&current_state_params->len<=256)
    {
        osal_memcpy(buffer, current_state_params->state, current_state_params->len);
        marker=current_state_params->len;
    }
    else
        marker=0;

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
		CONSOLE_OUT("publish: %d=",marker);
				for(uint8 m=0;m<marker;m++)
				{
		            CONSOLE_OUT(
		        "%X,",buffer[m]);
				}
    }

    retval = MS_access_publish(  
               &UI_sensor_server_model_handle,
                 opcode,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );

    return retval;
}



/**
 *  \brief API to send reply or to update state change
 *
 *  \par Description
 *  This is to send reply for a request or to inform change in state.
 *
 * \param [in] ctx                     Context of the message.
 * \param [in] current_state_params    Model specific current state parameters.
 * \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
 * \param [in] remaining_time          Time from current state to target state (0: to be ignored).
 * \param [in] ext_params              Additional parameters (NULL: to be ignored).
 *
 *  \return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_Sensor_setup_server_model_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT    * ctx,
    /* IN */ MS_ACCESS_SENSOR_MODEL_STATE_PARAMS       * current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS       * target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS         * ext_params
)
{
    API_RESULT retval;

    /* TODO: Check what should be maximum length */
    UCHAR      buffer[256];
    UCHAR    * pdu_ptr;
    UINT16     marker;
    UINT32     opcode;

    retval = API_FAILURE;
    marker = 0;

    CONSOLE_OUT(
        "[VENDOR_EXAMPLE_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {

    case MS_STATE_SENSOR_PROPERTY_ID_T:
    case MS_STATE_SENSOR_DESCRIPTOR_T:
    {

        opcode=MS_ACCESS_SENSOR_DESCRIPTOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_DATA_T:
    case MS_STATE_SENSOR_DATA_PROPERTY_ID_T:
    {
        opcode=MS_ACCESS_SENSOR_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SENSOR_COLUMN_STATUS_T:
    {
        opcode=MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE;
    }
    break;


    case MS_STATE_SENSOR_SERIES_COLUMN_T:
    {
        opcode=MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE;
    }
    break;

    default:
        break;

    }


    if(current_state_params->len>0&&current_state_params->len<=256)
    {
        osal_memcpy(buffer, current_state_params->state, current_state_params->len);
        marker=current_state_params->len;
    }
    else
        marker=0;

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
		CONSOLE_OUT("Data=");
		for(uint8 m=0;m<marker;m++)
		{
            CONSOLE_OUT(
        "%X,",buffer[m]);
		}
        
    }

    retval = MS_access_reply
             (
                 &ctx->handle,
                 ctx->daddr,
                 ctx->saddr,
                 ctx->subnet_handle,
                 ctx->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 pdu_ptr,
                 marker
             );

    return retval;
}



/* Sensor Model Server */
/**
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Vendor_Example_1 server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
 UINT8 mesh_updata_buf0[256];
API_RESULT UI_Sensor_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT         * ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW             * msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T               * req_type,
    /* IN */ MS_ACCESS_SENSOR_MODEL_STATE_PARAMS     * state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS              * ext_params
)
{
    UINT16 mesh_update_len=256; //it depends on your application
    UINT8 *mesh_updata_buf=(UINT8 *)osal_mem_alloc(mesh_update_len);
    UINT16 get_len=0;
    UINT16 sensor_property_id=0x00;



    MS_ACCESS_SENSOR_MODEL_STATE_PARAMS     current_state_params;
    API_RESULT retval;

    retval = API_SUCCESS;
    CONSOLE_OUT(
        "[SENSOR SERVER] %04x (d%04x s%04x).\n",ctx->handle,ctx->daddr,ctx->saddr);
    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {


        CONSOLE_OUT(
            "[Sensor server] GET Request.\n\r");




        if(msg_raw->opcode==MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE)
        {
            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;

            UI_Sensor_servel_model_descriptor_get(state_params->state_type, sensor_property_id, mesh_updata_buf, &get_len);
        }
        else if(msg_raw->opcode==MS_ACCESS_SENSOR_GET_OPCODE)
        {
            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;
            UI_Sensor_servel_model_sensorStatus_get(state_params->state_type, sensor_property_id, mesh_updata_buf, &get_len);

				CONSOLE_OUT("Data %d=",get_len);
				for(uint8 m=0;m<get_len;m++)
				{
		            CONSOLE_OUT(
		        "%X,",mesh_updata_buf[m]);
				}
        }
        else if(msg_raw->opcode==MS_ACCESS_SENSOR_COLUMN_GET_OPCODE)
        {
            state_params->state_type=MS_STATE_SENSOR_COLUMN_STATUS_T;
            state_params->len   = msg_raw->data_len;
            state_params->state = msg_raw->data_param;

            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;

            UI_Sensor_servel_model_column_get(state_params, sensor_property_id, mesh_updata_buf, &get_len);
        }
        else if(msg_raw->opcode==MS_ACCESS_SENSOR_SERIES_GET_OPCODE)
        {
            state_params->state_type=MS_STATE_SENSOR_SERIES_COLUMN_T;
            state_params->len   = msg_raw->data_len;
            state_params->state = msg_raw->data_param;

            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;
            UI_Sensor_servel_model_Sensor_ColumSeries_get(state_params, sensor_property_id, &mesh_updata_buf[0], &get_len);
        }

        current_state_params.len=get_len;
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = mesh_updata_buf;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[Sensor server] SET Request.ERROR CMD\n\r");


    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] Sending Response.\n");

        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_Sensor_server_model_state_update(ctx, &current_state_params, NULL, 0, NULL);
    }

    osal_mem_free(mesh_updata_buf);
    return retval;
}




/* Sensor setup Model Server */
/**
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Vendor_Example_1 server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 */
API_RESULT UI_Sensor_setup_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT         * ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW             * msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T               * req_type,
    /* IN */ MS_ACCESS_SENSOR_MODEL_STATE_PARAMS     * state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS              * ext_params
)
{

    UINT16 mesh_update_len=256; //it depends on your application
    UINT8 *mesh_updata_buf=(UINT8 *)osal_mem_alloc(mesh_update_len);
    UINT16 get_len=0;
    UINT16 sensor_property_id=0x00;



    MS_ACCESS_SENSOR_MODEL_STATE_PARAMS     current_state_params;
    API_RESULT retval;

    retval = API_SUCCESS;

    CONSOLE_OUT(
        "[Sensor setup server] %04x (d%04x s%04x).\n",ctx->handle,ctx->daddr,ctx->saddr);
    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[Sensor setup server] GET Request.\n");

        if(msg_raw->opcode==MS_ACCESS_SENSOR_CADENCE_GET_OPCODE)
        {
            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;

            UI_Sensor_setup_servel_model_cadence_get(state_params->state_type, sensor_property_id, mesh_updata_buf, &get_len);
        }
        else if(msg_raw->opcode==MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE)
        {
            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;
            UI_Sensor_setup_servel_model_settings_get(state_params->state_type, sensor_property_id, mesh_updata_buf, &get_len);
        }
		else if(msg_raw->opcode==MS_ACCESS_SENSOR_SETTING_GET_OPCODE)
        {
            state_params->len   = msg_raw->data_len;
            state_params->state = msg_raw->data_param;

            if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;

            UI_Sensor_setup_servel_model_setting_get(state_params, sensor_property_id, mesh_updata_buf, &get_len);
        }


        current_state_params.len=get_len;
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = mesh_updata_buf;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[SENSOR_SETUP_SERVER] SET Request.\n");
		
		if(msg_raw->opcode==MS_ACCESS_SENSOR_CADENCE_SET_OPCODE||msg_raw->opcode==MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE)
		{
		   if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;
          UI_Sensor_setup_server_cadence_set(state_params->state_type, sensor_property_id, msg_raw, 0);
		}
		else if(msg_raw->opcode==MS_ACCESS_SENSOR_CADENCE_SET_OPCODE||msg_raw->opcode==MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE)
		{
		  state_params->len   = msg_raw->data_len;
          state_params->state = msg_raw->data_param;
		  
		  if(msg_raw->data_len>=2)
                MS_PACK_BE_2_BYTE(&sensor_property_id,&msg_raw->data_param[0]); //property_id;
                
           UI_Sensor_setup_servel_model_setting_set(state_params, sensor_property_id, mesh_updata_buf, 0);

		}

        get_len=0;
        current_state_params.len=get_len;
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = mesh_updata_buf;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[VENDOR_EXAMPLE] Sending Response.\n");

        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_Sensor_setup_server_model_state_update(ctx, &current_state_params, NULL, 0, NULL);
    }

	osal_mem_free(mesh_updata_buf);

    return retval;
}



API_RESULT UI_register_Sensor_server
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Vendor Defined Server */

    API_RESULT retval;

    retval = MS_Sensor_server_init
             (
                 element_handle,
                 &UI_sensor_server_model_handle,
                 UI_Sensor_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "MS sensor Server Initialized. Model Handle: 0x%04X\n",
            UI_sensor_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] MS sensor Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }


    retval = MS_Sensor_setup_server_init
             (
                 element_handle,
                 &UI_sensor_setup_server_model_handle,
                 UI_Sensor_setup_server_cb
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT(
            "MS sensor setup Server Initialized. Model Handle: 0x%04X\n",
            UI_sensor_setup_server_model_handle);
    }
    else
    {
        CONSOLE_OUT(
            "[ERR] MS sensor setup Server Initialization Failed. Result: 0x%04X\n",
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
 * \brief Server Application Asynchronous Notification Callback.
 *
 * \par Description
 * Scene server calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] ctx           Context of message received for a specific model instance.
 * \param [in] msg_raw       Uninterpreted/raw received message.
 * \param [in] req_type      Requested message type.
 * \param [in] state_params  Model specific state parameters.
 * \param [in] ext_params    Additional parameters.
 *
 */
void * UI_scene_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE             * handle,
    /* IN */ UINT8                                event_type,
    /* IN */ void                               * event_param,
    /* IN */ UINT16                               event_length,
    /* IN */ void                               * context
)
{
    void       *param_p;
    UINT32     index;
    UI_STATE_SCENE_STRUCT   *state_p;
    MS_STATE_LIGHT_HSL_STRUCT *state_hsl;

    param_p = NULL;
    (void)handle;

    switch(event_type)
    {
    case MS_SCENE_EVENT_STORE:
    {
        if (event_length != 4)           // UINT32
            return NULL;
        index = *(UINT32 *)event_param;
        printf("store scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT *)EM_alloc_mem(sizeof(UI_STATE_SCENE_STRUCT));
        if (NULL == state_p)
        {
            printf("Allocate memory fail, size = %d\r\n", sizeof(UI_STATE_SCENE_STRUCT));
            return NULL;
        }

        // save current states
        state_p->index = index;
        EM_mem_copy((void *)&(state_p->light_hsl_state), (void *)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
        EM_mem_copy((void *)&(state_p->onoff_state), (void *)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        param_p = (void *)state_p;

    }
    break;

    case MS_SCENE_EVENT_DELETE:
    {
        if (event_length != 4)           // UINT32
            return NULL;
        index = *(UINT32 *)event_param;
        printf("delete scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT *)context;

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
        index = *(UINT32 *)event_param;
        printf("start recalling scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT *)context;

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
        index = *(UINT32 *)event_param;
        printf("complete recalling scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT *)context;

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
        index = *(UINT32 *)event_param;
        printf("complete recalling scene, index = %d \r\n", index);
        state_p = (UI_STATE_SCENE_STRUCT *)context;

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

#ifdef USE_SENSOR
    UI_Sensor_Server_model_states_initialization();
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
     * Encoded URI Information
     * For example, to give a web address, "https://www.abc.com"
     * the URI encoded data would be -
     * 0x17 0x2F 0x2F 0x77 0x77 0x77 0x2E 0x61 0x62 0x63 0x2E 0x63 0x6F 0x6D
     * where 0x17 is the URI encoding for https:
     */
    NULL
};

/** Current role of application - Provisioner/Device */
DECL_STATIC UCHAR UI_prov_role;

/** Provisioning Handle */
DECL_STATIC PROV_HANDLE UI_prov_handle;

static API_RESULT UI_prov_callback
(
    PROV_HANDLE * phandle,
    UCHAR         event_type,
    API_RESULT    event_result,
    void        * event_data,
    UINT16        event_datalen
)
{
    PROV_DATA_S * rdata;
    PROV_OOB_TYPE_S * oob_info;
    API_RESULT retval;

    UCHAR authstr[PROV_AUTHVAL_SIZE_PL << 1];
    UINT32 authnum;
    UCHAR authtype;
    UCHAR * pauth;
    UINT16 authsize;

    switch (event_type)
    {
        case PROV_EVT_PROVISIONING_SETUP:
            CONSOLE_OUT("Recvd PROV_EVT_PROVISIONING_SETUP\n");
            CONSOLE_OUT("Status - 0x%04X\n", event_result);

            /* Display the attention timeout */
            CONSOLE_OUT("Attention TImeout - %d\n", *((UCHAR *)event_data));

            LIGHT_ONLY_BLUE_ON;
            break;

        case PROV_EVT_OOB_DISPLAY:
            CONSOLE_OUT("Recvd PROV_EVT_OOB_DISPLAY\n");
            CONSOLE_OUT("Status - 0x%04X\n", event_result);

            /* Reference the Authenticatio Type information */
            oob_info = (PROV_OOB_TYPE_S *)event_data;

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

                pauth = (UCHAR *)&authnum;
                authsize = sizeof(UINT32);
            }
            else
            {
                authnum = (UINT32)UI_DISPLAY_AUTH_DIGIT;

                CONSOLE_OUT("\n\n>>> AuthVal - %d <<<\n\n", authnum);

                pauth = (UCHAR *)&authnum;
                authsize = sizeof(UINT32);
            }

            /* Call to input the oob */
            CONSOLE_OUT("Setting the Authval...\n");
            retval = MS_prov_set_authval(&UI_prov_handle, pauth, authsize);
            CONSOLE_OUT("Retval - 0x%04X\n", retval);
            break;

        case PROV_EVT_OOB_ENTRY:
            CONSOLE_OUT("Recvd PROV_EVT_OOB_ENTRY\n");
            CONSOLE_OUT("Status - 0x%04X\n", event_result);

            /* Reference the Authenticatio Type information */
            oob_info = (PROV_OOB_TYPE_S *)event_data;

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
            rdata = (PROV_DATA_S *)event_data;

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
            mesh_model_device_provisioned_ind_pl();
                light_blink_set(LIGHT_GREEN, LIGHT_BLINK_SLOW,3);
                /* Already Set while handling PROV_EVT_PROVDATA_INFO */
                UI_sample_get_device_key();

            osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_TERMINATE, 3000); //add gap terminate evt by hq
        }
        else
        {
            light_blink_set(LIGHT_RED, LIGHT_BLINK_SLOW,3);
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


static void UI_setup_prov(UCHAR role, UCHAR brr)
{
    API_RESULT retval;

    if (PROV_BRR_GATT == brr)
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
                     UI_PROV_SETUP_TIMEOUT_SECS,
                     UI_PROV_SETUP_TIMEOUT_SECS
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

    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}

static void UI_prov_bind(UCHAR brr, UCHAR index)
{
    API_RESULT retval;

    /* Call to bind with the selected device */
    CONSOLE_OUT("Binding with the selected device...\n");
    retval = MS_prov_bind(brr, &UI_lprov_device, UI_PROV_DEVICE_ATTENTION_TIMEOUT, &UI_prov_handle);

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
         * Register with Proxy Module as Device is going to be a Proxy.
         * This is typically a one-time-event, and hence registering the
         * PROXY when Proxy ADV is being initiated!
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
         NETIF_HANDLE       * handle,
         UCHAR                p_evt,
         UCHAR              * data_param,
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
     * Currently setting MT-MESH-SAMPLE-10a as Complete Device Name!
     * This can be updated to each individual devices as per requirement.
     */
    UCHAR UI_brr_scanrsp_data[] =
          {
          /**
           *  Shortened Device Name: MT-MESH-SAMPLE-10a
           */
        0x0B, 0x09, 'P', 'H', 'Y', '-', 'S', 'E', 'N', 'S', 'O', 'R'
          };
    CONSOLE_OUT("\n Setting PHY MSH SENSOR as Complete Device Name!\n");

    /* Set the Scan Response Data at the Bearer Layer */
    blebrr_set_adv_scanrsp_data_pl
    (
        UI_brr_scanrsp_data,
        sizeof(UI_brr_scanrsp_data)
    );

    return API_SUCCESS;
}

void timeout_cb (void * args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    UI_sample_reinit();
}
void proxy_dly_generic_onoff (void * args, UINT16 size)
{
    proxy_dly_thandle = EM_TIMER_HANDLE_INIT_VAL;
    generic_onoff_set_pl(UI_generic_onoff.onoff);
}

void proxy_dly_generic_hsl (void * args, UINT16 size)
{
    proxy_dly_thandle = EM_TIMER_HANDLE_INIT_VAL;
    UI_light_hsl_set_actual(0,UI_light_hsl.hsl_lightness,UI_light_hsl.hsl_hue,UI_light_hsl.hsl_saturation,0);
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
            EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
            break;

        /* GATT Bearer BLE Link Layer Connected */
        case BLEBRR_GATT_IFACE_UP:
            CONSOLE_OUT("\r\n >> GATT Bearer BLE Link Layer Connection Event Received!\r\n");
            /* Do Nothing! */
            if (BLEBRR_GATT_PROV_MODE == blebrr_gatt_mode_get())
            {
                MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);
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
    UINT8             * key;
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
#ifdef  USE_SENSOR
					retval=MS_access_bind_model_app(UI_sensor_server_model_handle, handle);
					CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_sensor_server_model_handle,handle);
					retval=MS_access_bind_model_app(UI_sensor_setup_server_model_handle, handle);
					CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_sensor_setup_server_model_handle,handle);
#endif
        }
    }
     
    return retval;
}

void vm_subscriptiong_binding_cb (void)
{
    CONSOLE_OUT("vm_subscriptiong_binding_cb\n");
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    UI_sample_binding_app_key();
    MS_ENABLE_RELAY_FEATURE();
    MS_ENABLE_PROXY_FEATURE();
    MS_ENABLE_FRIEND_FEATURE();
    MS_access_cm_set_transmit_state(MS_RELAY_TX_STATE, (8<<3)|1);
    MS_access_cm_set_transmit_state(MS_NETWORK_TX_STATE, (8<<3)|3);
}

void vm_subscriptiong_add (MS_NET_ADDR addr)
{
    CONSOLE_OUT("vm_subscriptiong_add:%x\n",addr);
    MS_ACCESS_ADDRESS       sub_addr;
    sub_addr.use_label=0;
    sub_addr.addr=addr;

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

//#ifdef  USE_VENDORMODEL
//    MS_access_cm_add_model_subscription(UI_vendor_defined_server_model_handle,&sub_addr);
//#endif

#ifdef  USE_SENSOR
	MS_access_cm_add_model_subscription(UI_sensor_server_model_handle,&sub_addr);
	MS_access_cm_add_model_subscription(UI_sensor_setup_server_model_handle,&sub_addr);
#endif


}

void vm_subscriptiong_delete (MS_NET_ADDR addr)
{
    MS_ACCESS_ADDRESS       sub_addr;
    sub_addr.use_label=0;
    sub_addr.addr=addr;

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

#ifdef  USE_SENSOR
	MS_access_cm_delete_model_subscription(UI_sensor_server_model_handle,&sub_addr);
	MS_access_cm_delete_model_subscription(UI_sensor_setup_server_model_handle,&sub_addr);
#endif


//#ifdef  USE_VENDORMODEL
//    MS_access_cm_delete_model_subscription(UI_vendor_defined_server_model_handle,&sub_addr);
//#endif

}



API_RESULT UI_app_config_server_callback (
    /* IN */ MS_ACCESS_MODEL_HANDLE * handle,
    /* IN */ MS_NET_ADDR               saddr,
    /* IN */ MS_NET_ADDR               daddr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_APPKEY_HANDLE          appkey_handle,
    /* IN */ UINT32                    opcode,
    /* IN */ UCHAR                  *  data_parm,
    /* IN */ UINT16                    data_len,
    /* IN */ API_RESULT                retval,
    /* IN */ UINT32                    response_opcode,
    /* IN */ UCHAR                   * response_buffer,
    /* IN */ UINT16                    response_buffer_len)
{

    uint8_t tx_state;
#ifdef EASY_BOUNDING
    MS_ACCESS_ADDRESS         addr;
#endif

    CONSOLE_OUT("[APP_CFG_SERV_CB] %04x \n", opcode);

    switch (opcode)
    {
    case MS_ACCESS_CONFIG_NODE_RESET_OPCODE:
        CONSOLE_OUT("[ST TimeOut CB]\n");
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
        CONSOLE_OUT("[NET TRX] %d \n",tx_state);
        break;
    case MS_ACCESS_CONFIG_RELAY_SET_OPCODE:
        MS_access_cm_get_transmit_state(MS_RELAY_TX_STATE, &tx_state);
        CONSOLE_OUT("[RLY TRX] %d \n",tx_state);
        break;

    case MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE:
#ifdef EASY_BOUNDING
        vm_subscriptiong_binding_cb();
        //EM_start_timer (&thandle, 1, vm_subscriptiong_binding_cb, NULL, 0);
#endif
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

    MS_CONFIG * config_ptr;

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
    nvsto_init();

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
     * TBD: Define GATT Namespace Descriptions from
     * https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors
     *
     * Using 'main' (0x0106) as Location temporarily.
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

#ifdef  USE_SENSOR
    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_register_Sensor_server(element_handle);
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

    /**
     * Set Scan Response Data Before Starting Provisioning.
     * This is optional/additional set of Data that the device can
     * set to enhance the User Experience.
     * For Example, set a specific device name or URL as part of the
     * Scan Response Data when awaiting connections over GATT bearer.
     */
    UI_set_brr_scan_rsp_data();

    /* Enable Friend feature */
//    MS_ENABLE_FRIEND_FEATURE();

    /* Enable LPN feature */
    //MS_ENABLE_LPN_FEATURE();	

    APP_config_server_CB_init(UI_app_config_server_callback);
    
    uint32 address = VENDOR_PRODUCT_MAC_ADDR;
        
    UI_lprov_device.uuid[10] = (uint8_t)ReadFlash(address ++);
    UI_lprov_device.uuid[11] = (uint8_t)ReadFlash(address ++);
    UI_lprov_device.uuid[12] = (uint8_t)ReadFlash(address ++);
    UI_lprov_device.uuid[13] = (uint8_t)ReadFlash(address ++);
    UI_lprov_device.uuid[8]  = (uint8_t)ReadFlash(address ++);
    UI_lprov_device.uuid[9]  = (uint8_t)ReadFlash(address ++);
        
    //UI_sample_reinit();
    EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
    //EM_start_timer (&thandle, 5, timeout_cb, NULL, 0);

    return;
}

API_RESULT UI_sample_get_net_key(void )
{
    UINT8   index=0;
	   UINT8 offset=0;
    UINT8   key[16];

    API_RESULT retval;

    CONSOLE_OUT("Fetching Net Key for indx 0x0000\n");

	retval=MS_access_cm_get_netkey_at_offset
	(
	index,
	offset,
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
    UINT8 * key;

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
    UINT8             * key;
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
 * On GATT Bearer Disconnection or on Startup:
 * 1. If device not provisioned, start unprovisioned beacons over GATT bearer
 * 2. If device is provisioned and App Key is bound i.e. Device is Configured
 *     - Check if Proxy Feature is enabled then Start ADV as Proxy,
 *       Else,
 *       Do nothing!
 *    Else,
 *    If device is provisioned and App Key is not bound i.e. Device is not Configured
 *       Start ADV as Proxy.
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
         * setup <role:[1 - Device, 2 - Provisioner]> <bearer:[1 - Adv, 2 - GATT]
         */
        role = PROV_ROLE_DEVICE;
        brr  = PROV_BRR_GATT;

        /**
         * Setting up an Unprovisioned Device over GATT
         */
        UI_setup_prov(role, brr);

        CONSOLE_OUT("\r\n Setting up as an Unprovisioned Device\r\n");
    }
    else
    {
        /* Fetch PROXY feature state */
        MS_access_cm_get_features_field(&state, MS_FEATURE_PROXY);

        /**
         * Check if the Device is Configured.
         * If not Configured, Start Proxy ADV.
         * If it is Configured,
         *      Check if the Proxy Feature is Enabled.
         *      If not enabled, then Do Nothing!
         *      If it is, Start Proxy ADV.
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
            }
            else
            {
                light_blink_set(LIGHT_GREEN, LIGHT_BLINK_SLOW,3);
                MS_access_cm_set_features_field(MS_ENABLE,MS_FEATURE_PROXY);

                CONSOLE_OUT("\r\n Provisioned Device - Enable PROXY !!!\r\n");
                /**
                 * Do Nothing!
                 * Already Scaning is Enabled at Start Up
                 */
                //blebrr_scan_enable();
                EM_start_timer (&thandle, 5, timeout_cb, NULL, 0);
            }
        }
        else
        {
            light_blink_set(LIGHT_BLUE, LIGHT_BLINK_FAST,5);

            UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
            /**
             * Provisioned but not configured device.
             * Still checking if the PROXY Feature is enabled or not.
             * Depending on the state of Proxy Feature:
             *   - If enabled, Start Proxy ADV with Network ID
             *   - Else, Start Proxy ADV with Node Identity.
             */
//             (MS_ENABLE == state) ?
//             UI_proxy_start_adv(0x0000, MS_PROXY_NET_ID_ADV_MODE):
//             UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
        }
    }
}


