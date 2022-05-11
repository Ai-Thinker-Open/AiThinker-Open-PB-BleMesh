/**
    \file cli_light_ctl_client.h

    \brief This file defines the Mesh Light Ctl Model Application Interface
    - includes Data Structures and Methods for Client.
*/

/*
    Copyright (C) 2018. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_CLI_LIGHT_CTL_CLIENT_
#define _H_CLI_LIGHT_CTL_CLIENT_


/* --------------------------------------------- Header File Inclusion */
#include "MS_light_ctl_api.h"
#include "cli_main.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Data Types/ Structures */


/* --------------------------------------------- Function */
/* light_ctl client CLI entry point */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl);

/* light_ctl client CLI entry point */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_setup);

/* Send Light Ctl Default Get */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_default_get);

/* Send Light Ctl Default Set */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_default_set);

/* Send Light Ctl Default Set Unacknowledged */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_default_set_unacknowledged);

/* Send Light Ctl Get */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_get);

/* Send Light Ctl Set */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_set);

/* Send Light Ctl Set Unacknowledged */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_set_unacknowledged);

/* Send Light Ctl Temperature Get */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_temperature_get);

/* Send Light Ctl Temperature Range Get */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_temperature_range_get);

/* Send Light Ctl Temperature Range Set */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_temperature_range_set);

/* Send Light Ctl Temperature Range Set Unacknowledged */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_temperature_range_set_unacknowledged);

/* Send Light Ctl Temperature Set */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_temperature_set);

/* Send Light Ctl Temperature Set Unacknowledged */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_temperature_set_unacknowledged);

/* Get Model Handle */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_get_model_handle);

/* Set Publish Address */
CLI_CMD_HANDLER_DECL(cli_modelc_light_ctl_set_publish_address);

/**
    \brief Client Application Asynchronous Notification Callback.

    \par Description
    Light_Ctl client calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT cli_light_ctl_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
);

#endif /*_H_CLI_LIGHT_CTL_CLIENT_ */
