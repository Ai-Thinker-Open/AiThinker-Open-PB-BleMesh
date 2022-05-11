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


#ifndef _CLI_VENDOR_H
#define _CLI_VENDOR_H

/*********************************************************************
    INCLUDES
*/
#include "types.h"
#include "rf_phy_driver.h"

#include "bleMesh.h"

#include "MS_common.h"

#include "MS_prov_api.h"

#include "nvs.h"

#include "cliface.h"

#include "mesh_clients.h"
#include "access_extern.h"
#include "MS_config_api.h"
#include "vendormodel_server.h"

/*********************************************************************
    LOCAL FUNCTIONS
*/
API_RESULT cli_raw_data(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_get_information(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_disp_key(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_demo_reset(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_internal_status(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_modelc_config_heartbeat_publication_set(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_start(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_on(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_off(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_seek(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_group_select(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_core_modelc_config_key_refresh_phase_set(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_core_modelc_config_netkey_update(UINT32 argc, UCHAR* argv[]);

API_RESULT cli_demo_help(UINT32 argc, UCHAR* argv[]);

#endif

