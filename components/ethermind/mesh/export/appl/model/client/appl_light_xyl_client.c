/**
    \file appl_light_xyl_client.c

    \brief This file defines the Mesh Light Xyl Model Application Interface
    - includes Data Structures and Methods for Client.
*/

/*
    Copyright (C) 2018. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "appl_light_xyl_client.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Static Global Variables */
static const char main_light_xyl_client_options[] = "\n\
======== Light_Xyl Client Menu ========\n\
     0. Exit. \n\
     1. Refresh. \n\
 \n\
    10. Send Light Xyl Default Get. \n\
    11. Send Light Xyl Default Set. \n\
    12. Send Light Xyl Default Set Unacknowledged. \n\
    13. Send Light Xyl Get. \n\
    14. Send Light Xyl Range Get. \n\
    15. Send Light Xyl Range Set. \n\
    16. Send Light Xyl Range Set Unacknowledged. \n\
    17. Send Light Xyl Set. \n\
    18. Send Light Xyl Set Unacknowledged. \n\
    19. Send Light Xyl Target Get. \n\
 \n\
    20. Get Model Handle. \n\
    21. Set Publish Address. \n\
 \n\
Your Option ? \0";


/* --------------------------------------------- External Global Variables */
static MS_ACCESS_MODEL_HANDLE   appl_light_xyl_client_model_handle;


/* --------------------------------------------- Function */
/* light_xyl client application entry point */
void main_light_xyl_client_operations(void)
{
    int choice;
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    static UCHAR model_initialized = 0x00;

    /**
        Register with Access Layer.
    */
    if (0x00 == model_initialized)
    {
        API_RESULT retval;
        /* Use Default Element Handle. Index 0 */
        element_handle = MS_ACCESS_DEFAULT_ELEMENT_HANDLE;
        retval = MS_light_xyl_client_init
                 (
                     element_handle,
                     &appl_light_xyl_client_model_handle,
                     appl_light_xyl_client_cb
                 );

        if (API_SUCCESS == retval)
        {
            CONSOLE_OUT(
                "Light Xyl Client Initialized. Model Handle: 0x%04X\n",
                appl_light_xyl_client_model_handle);
        }
        else
        {
            CONSOLE_OUT(
                "[ERR] Light Xyl Client Initialization Failed. Result: 0x%04X\n",
                retval);
        }

        model_initialized = 0x01;
    }

    MS_LOOP_FOREVER()
    {
        CONSOLE_OUT
        ("%s", main_light_xyl_client_options);
        CONSOLE_IN
        ("%d", &choice);

        if (choice < 0)
        {
            CONSOLE_OUT
            ("*** Invalid Choice. Try Again.\n");
            continue;
        }

        switch (choice)
        {
        case 0:
            return;

        case 1:
            break;

        case 10: /* Send Light Xyl Default Get */
            appl_send_light_xyl_default_get();
            break;

        case 11: /* Send Light Xyl Default Set */
            appl_send_light_xyl_default_set();
            break;

        case 12: /* Send Light Xyl Default Set Unacknowledged */
            appl_send_light_xyl_default_set_unacknowledged();
            break;

        case 13: /* Send Light Xyl Get */
            appl_send_light_xyl_get();
            break;

        case 14: /* Send Light Xyl Range Get */
            appl_send_light_xyl_range_get();
            break;

        case 15: /* Send Light Xyl Range Set */
            appl_send_light_xyl_range_set();
            break;

        case 16: /* Send Light Xyl Range Set Unacknowledged */
            appl_send_light_xyl_range_set_unacknowledged();
            break;

        case 17: /* Send Light Xyl Set */
            appl_send_light_xyl_set();
            break;

        case 18: /* Send Light Xyl Set Unacknowledged */
            appl_send_light_xyl_set_unacknowledged();
            break;

        case 19: /* Send Light Xyl Target Get */
            appl_send_light_xyl_target_get();
            break;

        case 20: /* Get Model Handle */
            appl_light_xyl_client_get_model_handle();
            break;

        case 21: /* Set Publish Address */
            appl_light_xyl_client_set_publish_address();
            break;
        }
    }
}

/* Send Light Xyl Default Get */
void appl_send_light_xyl_default_get(void)
{
    API_RESULT retval;
    CONSOLE_OUT
    (">> Send Light Xyl Default Get\n");
    retval = MS_light_xyl_default_get();
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Default Set */
void appl_send_light_xyl_default_set(void)
{
    API_RESULT retval;
    int  choice;
    MS_LIGHT_XYL_DEFAULT_SET_STRUCT  param;
    CONSOLE_OUT
    (">> Send Light Xyl Default Set\n");
    CONSOLE_OUT
    ("Enter Lightness (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.lightness = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL x (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y = (UINT16)choice;
    retval = MS_light_xyl_default_set(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Default Set Unacknowledged */
void appl_send_light_xyl_default_set_unacknowledged(void)
{
    API_RESULT retval;
    int  choice;
    MS_LIGHT_XYL_DEFAULT_SET_STRUCT  param;
    CONSOLE_OUT
    (">> Send Light Xyl Default Set Unacknowledged\n");
    CONSOLE_OUT
    ("Enter Lightness (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.lightness = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL x (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y = (UINT16)choice;
    retval = MS_light_xyl_default_set_unacknowledged(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Get */
void appl_send_light_xyl_get(void)
{
    API_RESULT retval;
    CONSOLE_OUT
    (">> Send Light Xyl Get\n");
    retval = MS_light_xyl_get();
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Range Get */
void appl_send_light_xyl_range_get(void)
{
    API_RESULT retval;
    CONSOLE_OUT
    (">> Send Light Xyl Range Get\n");
    retval = MS_light_xyl_range_get();
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Range Set */
void appl_send_light_xyl_range_set(void)
{
    API_RESULT retval;
    int  choice;
    MS_LIGHT_XYL_RANGE_SET_STRUCT  param;
    CONSOLE_OUT
    (">> Send Light Xyl Range Set\n");
    CONSOLE_OUT
    ("Enter xyL x Range Min (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x_range_min = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL x Range Max (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x_range_max = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y Range Min (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y_range_min = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y Range Max (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y_range_max = (UINT16)choice;
    retval = MS_light_xyl_range_set(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Range Set Unacknowledged */
void appl_send_light_xyl_range_set_unacknowledged(void)
{
    API_RESULT retval;
    int  choice;
    MS_LIGHT_XYL_RANGE_SET_STRUCT  param;
    CONSOLE_OUT
    (">> Send Light Xyl Range Set Unacknowledged\n");
    CONSOLE_OUT
    ("Enter xyL x Range Min (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x_range_min = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL x Range Max (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x_range_max = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y Range Min (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y_range_min = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y Range Max (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y_range_max = (UINT16)choice;
    retval = MS_light_xyl_range_set_unacknowledged(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Set */
void appl_send_light_xyl_set(void)
{
    API_RESULT retval;
    int  choice;
    MS_LIGHT_XYL_SET_STRUCT  param;
    CONSOLE_OUT
    (">> Send Light Xyl Set\n");
    CONSOLE_OUT
    ("Enter xyL Lightness (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_lightness = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL x (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y = (UINT16)choice;
    CONSOLE_OUT
    ("Enter TID (8-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.tid = (UCHAR)choice;
    CONSOLE_OUT
    ("Enter if optional fields to be send (1 - Yes, 0 - No)\n");
    CONSOLE_IN
    ("%d", &choice);

    if(0 == choice)
    {
        param.optional_fields_present = 0x00;
    }
    else
    {
        param.optional_fields_present = 0x01;
        CONSOLE_OUT
        ("Enter Transition Time (8-bit in HEX)\n");
        CONSOLE_IN
        ("%x", &choice);
        param.transition_time = (UCHAR)choice;
        CONSOLE_OUT
        ("Enter Delay (8-bit in HEX)\n");
        CONSOLE_IN
        ("%x", &choice);
        param.delay = (UCHAR)choice;
    }

    retval = MS_light_xyl_set(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Set Unacknowledged */
void appl_send_light_xyl_set_unacknowledged(void)
{
    API_RESULT retval;
    int  choice;
    MS_LIGHT_XYL_SET_STRUCT  param;
    CONSOLE_OUT
    (">> Send Light Xyl Set Unacknowledged\n");
    CONSOLE_OUT
    ("Enter xyL Lightness (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_lightness = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL x (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_x = (UINT16)choice;
    CONSOLE_OUT
    ("Enter xyL y (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.xyl_y = (UINT16)choice;
    CONSOLE_OUT
    ("Enter TID (8-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    param.tid = (UCHAR)choice;
    CONSOLE_OUT
    ("Enter if optional fields to be send (1 - Yes, 0 - No)\n");
    CONSOLE_IN
    ("%d", &choice);

    if(0 == choice)
    {
        param.optional_fields_present = 0x00;
    }
    else
    {
        param.optional_fields_present = 0x01;
        CONSOLE_OUT
        ("Enter Transition Time (8-bit in HEX)\n");
        CONSOLE_IN
        ("%x", &choice);
        param.transition_time = (UCHAR)choice;
        CONSOLE_OUT
        ("Enter Delay (8-bit in HEX)\n");
        CONSOLE_IN
        ("%x", &choice);
        param.delay = (UCHAR)choice;
    }

    retval = MS_light_xyl_set_unacknowledged(&param);
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}

/* Send Light Xyl Target Get */
void appl_send_light_xyl_target_get(void)
{
    API_RESULT retval;
    CONSOLE_OUT
    (">> Send Light Xyl Target Get\n");
    retval = MS_light_xyl_target_get();
    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);
}


/* Get Model Handle */
void appl_light_xyl_client_get_model_handle(void)
{
    API_RESULT retval;
    MS_ACCESS_MODEL_HANDLE model_handle;
    retval = MS_light_xyl_client_get_model_handle(&model_handle);

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT
        (">> Model Handle 0x%04X\n", model_handle);
    }
    else
    {
        CONSOLE_OUT
        (">> Get Model Handle Failed. Status 0x%04X\n", retval);
    }

    return;
}


/* Set Publish Address */
void appl_light_xyl_client_set_publish_address(void)
{
    int  choice;
    API_RESULT retval;
    MS_ACCESS_MODEL_HANDLE model_handle;
    MS_ACCESS_PUBLISH_INFO  publish_info;
    /* Set Publish Information */
    EM_mem_set(&publish_info, 0, sizeof(publish_info));
    publish_info.addr.use_label = MS_FALSE;
    CONSOLE_OUT
    ("Enter Model Handle (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    model_handle = (UINT16)choice;
    CONSOLE_OUT
    ("Enter Light_Xyl client Server Address (16-bit in HEX)\n");
    CONSOLE_IN
    ("%x", &choice);
    publish_info.addr.addr = (UINT16)choice;
    CONSOLE_OUT
    ("Enter AppKey Index\n");
    CONSOLE_IN
    ("%x", &choice);
    publish_info.appkey_index = choice;
    publish_info.remote = MS_FALSE;
    retval = MS_access_cm_set_model_publication
             (
                 model_handle,
                 &publish_info
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT
        (">> Publish Address is set Successfully.\n");
    }
    else
    {
        CONSOLE_OUT
        (">> Failed to set publish address. Status 0x%04X\n", retval);
    }

    return;
}

/**
    \brief Client Application Asynchronous Notification Callback.

    \par Description
    Light_Xyl client calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT appl_light_xyl_client_cb
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
        "[LIGHT_XYL_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_LIGHT_XYL_DEFAULT_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_LIGHT_XYL_DEFAULT_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_LIGHT_XYL_RANGE_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_LIGHT_XYL_RANGE_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_LIGHT_XYL_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_LIGHT_XYL_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_LIGHT_XYL_TARGET_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_LIGHT_XYL_TARGET_STATUS_OPCODE\n");
    }
    break;
    }

    return retval;
}

