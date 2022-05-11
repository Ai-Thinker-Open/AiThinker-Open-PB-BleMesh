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


#include "aliGenie_appl.h"
#include "aliGenie_appl_Generic.h"
#include "aliGenie_appl_Light.h"
extern UINT16 g_subscription_addr;

static void vm_vendor_mode_indication (MS_ACCESS_VENDOR_MODEL_STATE_PARAMS cur_state);
//void vm_subscriptiong_add (MS_NET_ADDR addr);

UINT16 aligenie_addr = 0xF000;

uint8 vendor_msg_tid=0;

EM_timer_handle thandle;

void timeout_cb (void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;

    if(UI_prov_state==ALIG_IN_PROV)
    {
        return;
    }
    else
    {
        UI_reinit();
    }
}

//--------------------------Config server-------------------------------------------------------

void UI_config_appkey_add (void* args, UINT16 size)
{
    TRACELOG_PRINT();
    MS_ACCESS_MODEL_HANDLE model_handle_list[MODEL_TREE_MAXNUM];
    UINT16 models_cnt = 0;
    API_RESULT retval;
    UINT16 handle = 0x0000;
    MS_ACCESS_ADDRESS       sub_addr;
    thandle = EM_TIMER_HANDLE_INIT_VAL;

    if (API_SUCCESS != UI_check_app_key())
    {
        ERROR_PRINT("INVALID App Key %04x\n",retval);
        return;
    }

    //find all registered models and bind to appkey handle
    models_cnt = find_model_handles(model_handle_list);
    ms_ps_store_disable_flag = 1;

    for (int i = 0; i < models_cnt; i++)
    {
        retval=MS_access_bind_model_app(model_handle_list[i], handle);
        DEBUG_PRINT("BINDING App Key %04x (%04x %04x)\n",retval,model_handle_list[i],handle);
    }

    ms_ps_store_disable_flag = 0;
    MS_access_ps_store_all_record();
    ms_ps_store_disable_flag = 1;
    sub_addr.use_label = 0;
    sub_addr.addr = g_subscription_addr;

    for (int i = 0; i < models_cnt; i++)
    {
        retval=MS_access_cm_add_model_subscription(model_handle_list[i],&sub_addr);
        DEBUG_PRINT("INIT SUBSCRIPTION %04x (%04x %04x)\n",retval,model_handle_list[i],sub_addr.addr);
    }

    sub_addr.use_label = 0;
    sub_addr.addr = 0xCFFF;

    for (int i = 0; i < models_cnt; i++)
    {
        retval=MS_access_cm_add_model_subscription(model_handle_list[i],&sub_addr);
        DEBUG_PRINT("INIT SUBSCRIPTION %04x (%04x %04x)\n",retval,model_handle_list[i],sub_addr.addr);
    }

    UI_get_device_key();
    MS_access_cm_set_default_ttl(0x08);
    /* Enable Relay feature */
    MS_ENABLE_RELAY_FEATURE();
    //HQ
    UI_prov_state = ALIG_SILENCE_ADV;
    //MS_DISABLE_RELAY_FEATURE();
    //Composite state (3-bitsLSB of Tx Count and 5-bitsMSB of Tx Interval Steps)
    MS_access_cm_set_transmit_state(MS_RELAY_TX_STATE, (5<<3)|0);
    ms_ps_store_disable_flag = 0;
    MS_access_ps_store_all_record();
    return;
}


void vm_subscription_add ( MS_ACCESS_MODEL_HANDLE model_handle,MS_NET_ADDR addr)
{
    TRACELOG_PRINT();
    //CONSOLE_OUT("vm_subscription_add:%x\n",addr);
    MS_ACCESS_ADDRESS       sub_addr;
    sub_addr.use_label=0;
    sub_addr.addr=addr;
    ms_ps_store_disable_flag = 1;
    MS_access_cm_add_model_subscription(model_handle,&sub_addr);
    ms_ps_store_disable_flag = 0;
    MS_access_ps_store_all_record();
}

void vm_subscription_delete (MS_ACCESS_MODEL_HANDLE model_handle,MS_NET_ADDR addr)
{
    TRACELOG_PRINT();
    MS_ACCESS_ADDRESS       sub_addr;
    sub_addr.use_label=0;
    sub_addr.addr=addr;
    ms_ps_store_disable_flag = 1;
    MS_access_cm_delete_model_subscription(model_handle,&sub_addr);
    ms_ps_store_disable_flag = 0;
    MS_access_ps_store_all_record();
}


void vm_power_on_indication_cb (void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_NET_ADDR               daddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    /* IN */ UCHAR                     data_param[5];
    /* IN */ UINT16                    data_len;
//    API_RESULT               retval;
    TRACELOG_PRINT();
    MS_access_cm_get_primary_unicast_address(&saddr);
    daddr = aligenie_addr;
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    data_len = 4;
    data_param[0]=++vendor_msg_tid;
    data_param[1]=0x09;
    data_param[2]=0xF0;
    data_param[3]=0x01;//0x03;
    data_param[4]=25;
    MS_access_send_pdu
    (
        saddr,
        daddr,
        subnet_handle,
        appkey_handle,
        ttl,
        opcode,
        data_param,
        data_len,
        MS_TRUE
    );
}

void vm_device_reset_cb(void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_NET_ADDR               daddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    /* IN */ UCHAR                     data_param[4];
    /* IN */ UINT16                    data_len;
//    API_RESULT               retval;
    MS_access_cm_get_primary_unicast_address(&saddr);
    daddr = aligenie_addr;
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    data_len = 4;
    data_param[0]=++vendor_msg_tid;
    data_param[1]=0x09;
    data_param[2]=0xF0;
    data_param[3]=0x06;
    MS_access_send_pdu
    (
        saddr,
        daddr,
        subnet_handle,
        appkey_handle,
        ttl,
        opcode,
        data_param,
        data_len,
        MS_TRUE
    );
    LIGHT_ONLY_RED_ON;
    WaitMs(300);
    LIGHT_ON_OFF(0,0,0);
    WaitMs(300);
    LIGHT_ONLY_BLUE_ON;
    WaitMs(300);
    LIGHT_ON_OFF(0,0,0);
    WaitMs(300);
    LIGHT_ONLY_GREEN_ON;
    WaitMs(300);
    LIGHT_ON_OFF(0,0,0);
    WaitMs(300);
    MS_access_cm_reset();
    UI_prov_state = ALIG_UN_PROV;
    DEBUG_PRINT("[ON_OFF_RESET]%02x\n",g_dev_reset_flg);
    g_dev_reset_flg=0;
    EM_start_timer (&thandle, 1, timeout_cb, NULL, 0);
}




API_RESULT appl_get_sig_model_handle_by_element_addr
(
    /* IN */  MS_NET_ADDR element_addr,
    /* IN */  UINT32 id,
    /* IN */  UCHAR id_type,
    /* OUT */ MS_ACCESS_MODEL_HANDLE* model_handle
)
{
    API_RESULT retval;
    MS_ACCESS_MODEL_ID model_id;
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    model_id.id = id;
    model_id.type = id_type;
    DEBUG_PRINT("%d\n",id_type);
    *model_handle = 0x0000;
    retval = MS_access_get_element_handle(element_addr, &element_handle);

    if(retval != API_SUCCESS)
    {
        ERROR_PRINT("MS_access_get_element_handle(element_addr = 0x%04X) = 0x%04X\n",element_addr,retval);
        return retval;
    }

    retval = MS_access_get_model_handle
             (
                 element_handle,
                 model_id,
                 model_handle
             );

    if(retval != API_SUCCESS)
    {
        ERROR_PRINT("MS_access_get_model_handle(element_handle = 0x%04X, model_id = 0x%08X) = 0x%04X\n",element_handle,id,retval);
    }

    return retval;
}


API_RESULT UI_app_config_server_callback (
    /* IN */ MS_ACCESS_MODEL_HANDLE*   config_model_handle,
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
    MS_ACCESS_ADDRESS         addr;
    UINT16 sig_model_id;
    UINT32 vendor_model_id;
    MS_NET_ADDR element_addr;
    MS_ACCESS_MODEL_HANDLE model_handle;
    INFO_PRINT("UI_app_config_server_callback\n");
    INFO_PRINT("model = 0x%04X, saddr = 0x%04X,daddr = 0x%04X, subnete = 0x%04X, appkey = 0x%04X, opcode = 0x%04X\n",*config_model_handle,saddr,daddr,subnet_handle,appkey_handle,opcode);
    appl_dump_bytes(data_parm, data_len);
    aligenie_addr = saddr;

    switch (opcode)
    {
    case MS_ACCESS_CONFIG_NODE_RESET_OPCODE:
        light_blink_set(LIGHT_RED,LIGHT_BLINK_FAST,  3);
        ERROR_PRINT("[ST TimeOut CB]\n");
        UI_prov_state=0;
        EM_start_timer (&thandle, 5, timeout_cb, NULL, 0);
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE:
        MS_UNPACK_LE_2_BYTE(&element_addr, data_parm);
        MS_UNPACK_LE_2_BYTE(&addr.addr,    data_parm + 2);

        if(6 == data_len)
        {
            MS_UNPACK_LE_2_BYTE(&sig_model_id, data_parm + 4);
            retval = appl_get_sig_model_handle_by_element_addr(element_addr,sig_model_id,MS_ACCESS_MODEL_TYPE_SIG,&model_handle);
        }
        else if(8 == data_len)
        {
            MS_UNPACK_LE_4_BYTE(&vendor_model_id, data_parm + 4);
            retval = appl_get_sig_model_handle_by_element_addr(element_addr,vendor_model_id,MS_ACCESS_MODEL_TYPE_VENDOR,&model_handle);
        }
        else
        {
            WHAT_THE();
        }

        if(retval != API_SUCCESS)
        {
            ERROR_PRINT("[CONFIG SUBS ADD ERROR]:element_addr =  0x%04X addr.addr = 0x%04X, model_handle = 0x%04X\n",element_addr, addr.addr,model_handle);
            return retval;
        }

        DEBUG_PRINT("[CONFIG SUBS ADD]: 0x%04X into 0x%04X\n",addr.addr,model_handle);
        set_element_addr_by_model_handle(model_handle,element_addr);
        vm_subscription_add(model_handle,addr.addr);
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE:
        MS_UNPACK_LE_2_BYTE(&element_addr, data_parm);
        MS_UNPACK_LE_2_BYTE(&addr.addr,    data_parm + 2);

        if(6 == data_len)
        {
            MS_UNPACK_LE_2_BYTE(&sig_model_id,     data_parm + 4);
            retval = appl_get_sig_model_handle_by_element_addr(element_addr,sig_model_id,MS_ACCESS_MODEL_TYPE_SIG,&model_handle);
        }
        else if(8 == data_len)
        {
            MS_UNPACK_LE_4_BYTE(&vendor_model_id,      data_parm + 4);
            retval = appl_get_sig_model_handle_by_element_addr(element_addr,vendor_model_id,MS_ACCESS_MODEL_TYPE_VENDOR,&model_handle);
        }
        else
        {
            WHAT_THE();
        }

        if(retval != API_SUCCESS)
        {
            ERROR_PRINT("[CONFIG SUBS DEL ERROR]:element_addr =  0x%04X addr.addr = 0x%04X, model_handle = 0x%04X\n",element_addr, addr.addr,model_handle);
            return retval;
        }

        DEBUG_PRINT("[CONFIG SUBS DEL]: 0x%04X from 0x%04X\n",addr.addr,model_handle);
        set_element_addr_by_model_handle(model_handle,element_addr);
        vm_subscription_delete(model_handle,addr.addr);
        break;

    case MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE:
        DEBUG_PRINT("[CONFIG APPKEY ADD]\n");
        EM_start_timer (&thandle, 3, UI_config_appkey_add, (void*)&model_handle, sizeof(MS_ACCESS_MODEL_HANDLE));
        break;

    default:
        break;
    }

    return API_SUCCESS;
}



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
    INFO_PRINT(
        "Health Server Callback. Not handled. Returning\n");
    return API_SUCCESS;
}


/* Configuration Server */
MS_ACCESS_MODEL_HANDLE   UI_config_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_health_server_model_handle;

static API_RESULT UI_register_foundation_model_servers
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    API_RESULT retval;
    /* Health Server */
    UINT16                       company_id;
    MS_HEALTH_SERVER_SELF_TEST* self_tests;
    UINT32                       num_self_tests;
    DEBUG_PRINT("In Model Server - Foundation Models\n");
    retval = MS_config_server_init(element_handle, &UI_config_server_model_handle);
    DEBUG_PRINT("Config Model Server Registration Status: 0x%04X\n", retval);
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
        DEBUG_PRINT(
            "Health Server Initialized. Model Handle: 0x%04X\n",
            UI_health_server_model_handle);
    }
    else
    {
        ERROR_PRINT(
            "Sensor Server Initialization Failed. Result: 0x%04X\n",
            retval);
    }

    return retval;
}



/* Model state Initialization */
static void UI_model_states_initialization(void)
{
    /* Vendor Defined States */
    UI_vendor_defined_model_states_initialization();
}


API_RESULT UI_get_device_key(void )
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
API_RESULT UI_check_app_key(void)
{
    MS_APPKEY_HANDLE  handle;
    UINT8*              key;
    UINT8             aid;
//    UINT32            i;
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
        CONSOLE_OUT("App Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
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

API_RESULT UI_binding_app_key(MS_ACCESS_MODEL_HANDLE model_handle)
{
    MS_APPKEY_HANDLE  handle;
    UINT8*              key;
    UINT8             aid;
//    UINT32            i;
    DECL_CONST UINT8  t_key[16] = {0};
    API_RESULT retval;
    DEBUG_PRINT("Fetching App Key for Handle 0x0000\n");
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
        CONSOLE_OUT("App Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    handle, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                    key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

        if (0 == EM_mem_cmp(key, t_key, 16))
        {
            /* NO AppKey Bound */
            retval = API_FAILURE;
            ERROR_PRINT("UI_binding_app_key: NO AppKey Bound:\n");
            ERROR_PRINT("App Key[0x%02X]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                        handle, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                        key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);
        }
    }

    return retval;
}


API_RESULT UI_set_brr_scan_rsp_data (void)
{
    /**
        Currently setting MT-MESH-SAMPLE-8 as Complete Device Name!
        This can be updated to each individual devices as per requirement.
    */
    UCHAR UI_brr_scanrsp_data[] =
    {
        0x0b,0x09,'A','L','I','P','H','Y','M','E','S','H',
        0x0f,0xff,
        0xa8,0x01,0x85,0x0f,
        0xff,0xff,0xff,0xff,
        0xff,0xff,0xff,0xff,0xff,0xff,

    };
    osal_revmemcpy(UI_brr_scanrsp_data+15,ali_genie_pid,4);
    osal_memcpy(UI_brr_scanrsp_data+22,ali_genie_mac,6);
    //EM_mem_copy(UI_brr_scanrsp_data+11, ali_genie_macStr, 12);
    CONSOLE_OUT("\n Setting PHY Genie as Complete Device Name!\n");
    /* Set the Scan Response Data at the Bearer Layer */
    blebrr_set_adv_scanrsp_data_pl
    (
        UI_brr_scanrsp_data,
        sizeof(UI_brr_scanrsp_data)
    );
    return API_SUCCESS;
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
        CONSOLE_OUT("\n >> GATT Bearer BLE Link Layer Disconnection Event Received!\n");
        //UI_sample_reinit();
        EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
        break;

    /* GATT Bearer BLE Link Layer Connected */
    case BLEBRR_GATT_IFACE_UP:
        CONSOLE_OUT("\n >> GATT Bearer BLE Link Layer Connection Event Received!\n");

        /* Do Nothing! */
        if (BLEBRR_GATT_PROV_MODE == blebrr_gatt_mode_get())
        {
            MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);
        }
        else if (BLEBRR_GATT_PROXY_MODE == blebrr_gatt_mode_get())
        {
            #ifdef MS_PROXY_SERVER
            MS_proxy_server_adv_stop();
            #endif
        }

        break;

    case BLEBRR_GATT_IFACE_ENABLE:
        CONSOLE_OUT("\n >> GATT Bearer Active Event Received!\n");
        {
            if (BLEBRR_GATT_PROV_MODE == ev_param)
            {
                /* Call to bind with the selected device */
                UI_prov_bind(PROV_BRR_GATT, 0);
            }
        }
        break;

    case BLEBRR_GATT_IFACE_DISABLE:
        CONSOLE_OUT("\n >> GATT Bearer Inactive Event Received!\n");
        break;

    /* Unknown Event! */
    default:
        CONSOLE_OUT("\n >> GATT Bearer BLE Link Layer Unknown Event 0x%02X Received!\n", ev_name);
        /* Do Nothing! */
        break;
    }
}



void appl_mesh(void)
{
    MS_ACCESS_NODE_ID node_id;
    MS_ACCESS_ELEMENT_DESC   element;
    UINT8 ele_cnt;
    MS_ACCESS_ELEMENT_HANDLE element_handle1;
    API_RESULT retval;
//    UCHAR role, brr;
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
        TBD: Define GATT Namespace Descriptions from
        https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors

        Using 'main' (0x0106) as Location temporarily.
    */
    element.loc = 0x0106;
    retval = MS_access_register_element
             (
                 node_id,
                 &element,
                 &element_handle1
             );

    if (API_SUCCESS == retval)
    {
        /* Register foundation model servers */
        retval = UI_register_foundation_model_servers(element_handle1);
    }

    APP_config_server_CB_init(UI_app_config_server_callback);

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle1,LIGHT_GREEN);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_generic_scene_model_server_create(element_handle1);
        //UI_generic_scene_model_states_initialization();
    }

    if (API_SUCCESS == retval)
    {
        retval = UI_light_lightness_model_server_create(element_handle1);
    }

    if (API_SUCCESS == retval)
    {
        retval = UI_light_hsl_model_server_create(element_handle1);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle1,LIGHT_GREEN);
    }

    #ifdef ELEMENT_2nd
    MS_ACCESS_ELEMENT_HANDLE element_handle2;
    element.loc = 0x0107;
    retval = MS_access_register_element
             (
                 node_id,
                 &element,
                 &element_handle2
             );

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle2,LIGHT_BLUE);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle2,LIGHT_BLUE);
    }

    #endif
    #ifdef ELEMENT_3rd
    MS_ACCESS_ELEMENT_HANDLE element_handle3;
    element.loc = 0x0108;
    retval = MS_access_register_element
             (
                 node_id,
                 &element,
                 &element_handle3
             );

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle3,LIGHT_RED);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle3,LIGHT_RED);
    }

    #endif

    if (API_SUCCESS == retval)
    {
        /* Initialize model states */
        UI_model_states_initialization();
    }

    /* Configure as provisionee/device */
    UI_register_prov();
    MS_access_cm_get_element_count(&ele_cnt);
    DEBUG_PRINT("element_cnt = %d\n",ele_cnt);
    /**
        Set Scan Response Data Before Starting Provisioning.
        This is optional/additional set of Data that the device can
        set to enhance the User Experience.
        For Example, set a specific device name or URL as part of the
        Scan Response Data when awaiting connections over GATT bearer.
    */
    UI_set_brr_scan_rsp_data();
    //UI_sample_reinit();
    EM_start_timer (&thandle, (300 | EM_TIMEOUT_MILLISEC), timeout_cb, NULL, 0);
    return;
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
static UCHAR last_brr=0;//PROV_BRR_GATT;//PROV_BRR_GATT;// PROV_BRR_GATT
void UI_reinit(void)
{
    API_RESULT  retval;
    MS_NET_ADDR addr;
    UCHAR       is_prov_req;
    UCHAR       role;
    UCHAR       state;
    TRACELOG_PRINT();
    last_brr=0;

    if(UI_prov_state==ALIG_IN_PROV)
    {
        WARN_PRINT("[RE_INIT] IN PORV %d %d >> Return \n\r",UI_prov_state,last_brr);
        return;
    }

    retval      = API_SUCCESS;
    is_prov_req = MS_TRUE;
    retval = MS_access_cm_get_primary_unicast_address(&addr);

    if (API_SUCCESS == retval)
    {
        if (MS_NET_ADDR_UNASSIGNED != addr)
        {
            if(API_SUCCESS == UI_check_app_key())
            {
                /* Set Provisioning is not Required */
                is_prov_req = MS_FALSE;
                DEBUG_PRINT("Provisioning is not Required\n");
            }
            else
            {
                MS_access_cm_reset();
            }
        }
    }

    TRACELOG_PRINT();
    DEBUG_PRINT("[RE_INIT] ret%04x addr%04x prov_req %04x brr%d\n",retval,addr,is_prov_req,last_brr);

    if (MS_TRUE == is_prov_req)
    {
        TRACELOG_PRINT();
        /* Start Provisioning over GATT here */
        /**
            setup <role:[1 - Device, 2 - Provisioner]> <bearer:[1 - Adv, 2 - GATT]
        */
        role = PROV_ROLE_DEVICE;

        if(last_brr==PROV_BRR_GATT)
        {
            last_brr = PROV_BRR_ADV;
            EM_start_timer (&thandle, (2000| EM_TIMEOUT_MILLISEC), timeout_cb, NULL, 0);
            CONSOLE_OUT("[GATT->ADV1] %d %d\n",retval,blebrr_state);
            retval=MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);
            CONSOLE_OUT("[GATT->ADV] %d %d\n",retval,blebrr_state);
            UI_setup_prov(role, last_brr);
            UI_prov_bind(last_brr, 0x00);
        }
        else if(last_brr==PROV_BRR_ADV)
        {
            last_brr = PROV_BRR_GATT;
            EM_start_timer (&thandle, (2000| EM_TIMEOUT_MILLISEC), timeout_cb, NULL, 0);
            CONSOLE_OUT("[ADV->GATT1] %d %d\n",retval,blebrr_state);
            /* End beaconing */
            retval=MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_PASSIVE);
            CONSOLE_OUT("[ADV->GATT] %d %d\n",retval,blebrr_state);
            UI_setup_prov(role, last_brr);
        }
        else
        {
            last_brr = PROV_BRR_ADV;
            UI_setup_prov(role, last_brr);
            UI_prov_bind(last_brr, 0x00);
        }

        CONSOLE_OUT("\n Setting up as an Unprovisioned Device %d\n",last_brr);
        //LIGHT_ONLY_RED_ON;//¨¦¨¨¡À??a??¨¢¨¢o¨¬¦Ì? change by johhn
    }
    else
    {
        TRACELOG_PRINT();
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
        if (API_SUCCESS == UI_check_app_key())
        {
            TRACELOG_PRINT();

            if (MS_ENABLE == state)
            {
                TRACELOG_PRINT();
                CONSOLE_OUT("\n Provisioned Device - Starting Proxy with NetID on Subnet 0x0000!\n");
                /* Start Proxy ADV with Network ID here */
                UI_proxy_start_adv(0x0000, MS_PROXY_NET_ID_ADV_MODE);
                light_blink_set(LIGHT_GREEN,LIGHT_BLINK_FAST,  5);
            }
            else
            {
                TRACELOG_PRINT();
                /**
                    Do Nothing!
                    Already Scaning is Enabled at Start Up
                */
                MS_access_cm_get_features_field(&state, MS_FEATURE_RELAY);

                if(MS_ENABLE==state)
                {
                    LIGHT_ONLY_BLUE_ON;
                    WaitMs(1000);
                }

                blebrr_scan_enable();
                CONSOLE_OUT("\n ReInit\n");

                if(g_dev_reset_flg>5)
                {
                    TRACELOG_PRINT();
                    EM_start_timer (&thandle, (100| EM_TIMEOUT_MILLISEC), vm_device_reset_cb, NULL, 0);
                }
                else
                {
                    TRACELOG_PRINT();
                    light_blink_set(LIGHT_GREEN,LIGHT_BLINK_FAST,  3);
                    EM_start_timer (&thandle, 6, vm_power_on_indication_cb, NULL, 0);
                }
            }
        }
        else
        {
            TRACELOG_PRINT();
            /**
                Provisioned but not configured device.
                Still checking if the PROXY Feature is enabled or not.
                Depending on the state of Proxy Feature:
                 - If enabled, Start Proxy ADV with Network ID
                 - Else, Start Proxy ADV with Node Identity.
            */
            (MS_ENABLE == state) ?
            UI_proxy_start_adv(0x0000, MS_PROXY_NET_ID_ADV_MODE):
            UI_proxy_start_adv(0x0000, MS_PROXY_NODE_ID_ADV_MODE);
            light_blink_set(LIGHT_BLUE,LIGHT_BLINK_FAST,  3);
        }
    }
}



