
/**
    \file appl_sample_example_6.c

    Source File for Generic OnOff Server and a vendor defined model Server
    Standalone application without CLI or menu based console input interface.
*/

/*
    Copyright (C) 2018. Mindtree Ltd.
    All rights reserved.
*/

#if (MESH_STANDALONE == 6)

/* ----------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_config_api.h"
#include "MS_health_server_api.h"
#include "MS_generic_onoff_api.h"
#include "blebrr.h"
#include "nvsto.h"
#include "model_state_handler_pl.h"

/* Console Input/Output */
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)

void appl_dump_bytes(UCHAR* buffer, UINT16 length);
void appl_mesh_sample (void);

/* --------------------------------------------- Global Definitions */
#define MS_MODEL_ID_VENDOR_EXAMPLE_SERVER                         0xA001A001

#define MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE                       0xA0010001
#define MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE                       0xA0010002
#define MS_ACCESS_VENDOR_EXAMPLE_SET_UNACKNOWLEDGED_OPCODE        0xA0010003

#define MS_STATE_VENDOR_EXAMPLE_T    0xA1

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
typedef API_RESULT (* MS_VENDOR_EXAMPLE_SERVER_CB)
(
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    MS_ACCESS_MODEL_EXT_PARAMS*          ext_params

) DECL_REENTRANT;

typedef struct MS_state_vendor_example_struct
{
    UCHAR  value;

} MS_STATE_VENDOR_EXAMPLE_STRUCT;


/* ----------------------------------------- External Global Variables */


/* ----------------------------------------- Exported Global Variables */


/* ----------------------------------------- Static Global Variables */
static DECL_CONST UINT32 vendor_example_server_opcode_list[] =
{
    MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE,
    MS_ACCESS_VENDOR_EXAMPLE_SET_UNACKNOWLEDGED_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   vendor_example_server_model_handle;
static MS_VENDOR_EXAMPLE_SERVER_CB       vendor_example_server_UI_cb;


/* ----------------------------------------- Functions */
/**
    \brief API to send reply or to update state change

    \par Description
    This is to send reply for a request or to inform change in state.

    \param [in] ctx                     Context of the message.
    \param [in] current_state_params    Model specific current state parameters.
    \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
    \param [in] remaining_time          Time from current state to target state (0: to be ignored).
    \param [in] ext_params              Additional parameters (NULL: to be ignored).

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_vendor_example_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
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

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(vendor_example_set_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(vendor_example_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(vendor_example_set_unacknowledged_handler)

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
API_RESULT vendor_example_server_cb
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
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT  req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW      req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T        req_type;
    MS_ACCESS_MODEL_EXT_PARAMS*      ext_params_p;
    MS_ACCESS_MODEL_STATE_PARAMS                     state_params;
    UINT16        marker;
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
        "[VENDOR_EXAMPLE_SERVER] Callback. Opcode 0x%04X\n", opcode);
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
        state_params.state_type = MS_STATE_VENDOR_EXAMPLE_T;
    }
    break;

    case MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_VENDOR_EXAMPLE_SET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(vendor_example_set_handler);
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;
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
    }

    /* Application callback */
    if (NULL != vendor_example_server_UI_cb)
    {
        vendor_example_server_UI_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
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
API_RESULT MS_vendor_example_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
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
    /* TODO: Remove */
    vendor_example_server_model_handle = *model_handle;
    return retval;
}


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
    company_id = 0x0000;
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


/* ---- Generic OnOff States */
static MS_STATE_GENERIC_ONOFF_STRUCT UI_generic_onoff;

/** -- Vendor Defined States */
static MS_STATE_VENDOR_EXAMPLE_STRUCT UI_vendor_example;

/* Get/Set State Handlers */

/* Generic OnOff Model state Initialization */
void UI_generic_onoff_model_states_initialization(void)
{
    EM_mem_set(&UI_generic_onoff, 0, sizeof(UI_generic_onoff));
}

/* Generic OnOff Model Get Handler */
API_RESULT UI_generic_onoff_model_state_get(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
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
API_RESULT UI_generic_onoff_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    API_RESULT retval;
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
        generic_onoff_set_pl(param_p->onoff);
        /* Ignoring Instance and direction right now */
    }
    break;

    default:
        break;
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
    case MS_STATE_VENDOR_EXAMPLE_T:
    {
        MS_STATE_VENDOR_EXAMPLE_STRUCT* param_p;
        param_p = (MS_STATE_VENDOR_EXAMPLE_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        *param_p = UI_vendor_example;
    }
    break;

    default:
        break;
    }
}

/* Vendor Defined Model Set Handler */
void UI_vendor_example_model_state_set(UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_VENDOR_EXAMPLE_T:
    {
        MS_STATE_VENDOR_EXAMPLE_STRUCT* param_p;
        param_p = (MS_STATE_VENDOR_EXAMPLE_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        UI_vendor_example = *param_p;
    }
    break;

    default:
        break;
    }
}


/* Model state Initialization */
void UI_model_states_initialization(void)
{
    /* Generic OnOff States */
    UI_generic_onoff_model_states_initialization();
    /* Vendor Defined States */
    UI_vendor_defined_model_states_initialization();
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
API_RESULT UI_generic_onoff_server_cb
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
    MS_ACCESS_MODEL_HANDLE   UI_generic_onoff_server_model_handle;
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
API_RESULT UI_vendor_example_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_STATE_VENDOR_EXAMPLE_STRUCT param;
    MS_ACCESS_MODEL_STATE_PARAMS                    current_state_params;
    API_RESULT retval;
    retval = API_SUCCESS;

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
        UI_vendor_example_model_state_set(state_params->state_type, 0, (MS_STATE_VENDOR_EXAMPLE_STRUCT*)state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = (MS_STATE_VENDOR_EXAMPLE_STRUCT*)state_params->state;
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
    MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_server_model_handle;
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

/* Provisionee */
#define UI_PROV_OUTPUT_OOB_ACTIONS \
    (PROV_MASK_OOOB_ACTION_BLINK | PROV_MASK_OOOB_ACTION_BEEP | \
     PROV_MASK_OOOB_ACTION_VIBRATE | PROV_MASK_OOOB_ACTION_NUMERIC | \
     PROV_MASK_OOOB_ACTION_ALPHANUMERIC)

/** Output OOB Maximum size supported */
#define UI_PROV_OUTPUT_OOB_SIZE               0x08

/** Input OOB Actions supported */
#define UI_PROV_INPUT_OOB_ACTIONS \
    (PROV_MASK_IOOB_ACTION_PUSH | PROV_MASK_IOOB_ACTION_TWIST | \
     PROV_MASK_IOOB_ACTION_NUMERIC | PROV_MASK_IOOB_ACTION_ALPHANUMERIC)

/** Input OOB Maximum size supported */
#define UI_PROV_INPUT_OOB_SIZE                0x08

/** Beacon setup timeout in seconds */
#define UI_PROV_SETUP_TIMEOUT_SECS            30

/** Attention timeout for device in seconds */
#define UI_PROV_DEVICE_ATTENTION_TIMEOUT      30

#define PROV_AUTHVAL_SIZE_PL                  16

/** Authentication values for OOB Display - To be made random */
#define UI_DISPLAY_AUTH_DIGIT                 3
#define UI_DISPLAY_AUTH_NUMERIC               35007
#define UI_DISPLAY_AUTH_STRING                "f00l"

/** Provisioning capabilities of local device */
DECL_STATIC PROV_CAPABILITIES_S UI_prov_capab =
{
    /** Number of Elements */
    0x01,

    /** Supported algorithms */
    PROV_MASK_ALGO_EC_FIPS_P256,

    /** Public key type */
    PROV_MASK_PUBKEY_OOBINFO,

    /** Static OOB type */
    PROV_MASK_STATIC_OOBINFO,

    /** Output OOB information */
    { UI_PROV_OUTPUT_OOB_ACTIONS, UI_PROV_OUTPUT_OOB_SIZE },

    /** Input OOB information */
    { UI_PROV_INPUT_OOB_ACTIONS, UI_PROV_INPUT_OOB_SIZE },
};

/** Unprovisioned device identifier */
PROV_DEVICE_S UI_lprov_device =
{
    /** UUID */
    {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},

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

/** Data exchanged during Provisiong procedure */
DECL_STATIC PROV_DATA_S UI_prov_data =
{
    /** NetKey */
    { 0x45, 0x74, 0x68, 0x65, 0x72, 0x4d, 0x69, 0x6e, 0x64, 0x4e, 0x65, 0x74, 0x4b, 0x65, 0x79, 0x00 },

    /** Index of the NetKey */
    0x0000,

    /** Flags bitmask */
    0x00,

    /** Current value of the IV index */
    0x00000001,

    /** Unicast address of the primary element */
    0x0002
};


/** Current role of application - Provisioner/Device */
DECL_STATIC UCHAR UI_prov_role;

/** Provisioning Handle */
DECL_STATIC PROV_HANDLE UI_prov_handle;

API_RESULT UI_prov_callback
(
    PROV_HANDLE* phandle,
    UCHAR         event_type,
    API_RESULT    event_result,
    void*         event_data,
    UINT16        event_datalen
)
{
    PROV_DEVICE_S* rdev;
    PROV_CAPABILITIES_S* rcap;
    PROV_DATA_S* rdata;
    PROV_OOB_TYPE_S* oob_info;
    API_RESULT retval;
    UCHAR i;
    UCHAR authstr[PROV_AUTHVAL_SIZE_PL << 1];
    UINT32 authnum;
    UCHAR authtype;
    UCHAR* pauth;
    UINT16 authsize;
    UCHAR  pdata[(MS_DEVICE_UUID_SIZE * 2) + 1];
    UCHAR*   t_data;

    switch (event_type)
    {
    case PROV_EVT_PROVISIONING_SETUP:
        CONSOLE_OUT("Recvd PROV_EVT_PROVISIONING_SETUP\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);
        /* Display the attention timeout */
        CONSOLE_OUT("Attention TImeout - %d\n", *((UCHAR*)event_data));
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
            authsize = EM_str_len(authstr);
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
        retval = MS_prov_set_authval(&UI_prov_handle, pauth, authsize);
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
        }

        break;

    default:
        CONSOLE_OUT("Unknown Event - 0x%02X\n", event_type);
    }

    return API_SUCCESS;
}

void UI_register_prov(void)
{
    API_RESULT retval;
    CONSOLE_OUT("Registering with Provisioning layer...\n");
    retval = MS_prov_register(&UI_prov_capab, UI_prov_callback);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}

void UI_setup_prov(UCHAR role, UCHAR brr)
{
    API_RESULT retval;

    if (PROV_ROLE_PROVISIONER != role)
    {
        CONSOLE_OUT("Setting up Device for Provisioning ...\n");
        retval = MS_prov_setup
                 (
                     brr,
                     role,
                     &UI_lprov_device,
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
                     UI_PROV_SETUP_TIMEOUT_SECS
                 );
        UI_prov_role = PROV_ROLE_PROVISIONER;
    }

    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}

void UI_prov_bind(UCHAR brr, UCHAR index)
{
    API_RESULT retval;
    /* Call to bind with the selected device */
    CONSOLE_OUT("Binding with the selected device...\n");
    retval = MS_prov_bind(brr, &UI_lprov_device, UI_PROV_DEVICE_ATTENTION_TIMEOUT, &UI_prov_handle);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}

void appl_mesh_sample (void)
{
    MS_ACCESS_NODE_ID node_id;
    MS_ACCESS_ELEMENT_DESC   element;
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    API_RESULT retval;
    UCHAR role, brr;
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
    /* Initialize utilities */
    nvsto_init();
    /* Initialize Mesh Stack */
    MS_init(config_ptr);
    /* Register with underlying BLE stack */
    blebrr_register();
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

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_register_vendor_defined_model_server(element_handle);
    }

    if (API_SUCCESS == retval)
    {
        /* Initialize model states */
        UI_model_states_initialization();
    }

    /* Configure as provisionee/device */
    UI_register_prov();
    /**
        setup <role:[1 - Device, 2 - Provisioner]> <bearer:[1 - Adv, 2 - GATT]
    */
    role = PROV_ROLE_DEVICE;
    brr = PROV_BRR_ADV;
    UI_setup_prov(role, brr);
    UI_prov_bind(brr, 0x00);
    return;
}

#endif /* (MESH_STANDALONE == 6) */

