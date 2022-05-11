/* ----------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_config_api.h"
#include "MS_generic_onoff_api.h"
#include "blebrr.h"
#include "nvsto.h"

#include "access_extern.h"
#include "gpio.h"
#include "ltrn_extern.h"
#include "net_extern.h"
#include "vendormodel_client.h"

#include "flash.h"
#include "ll_def.h"
#include "ll_enc.h"
#include "EXT_cbtimer.h"


#undef USE_HSL                 // enable Light HSL server model
#undef USE_LIGHTNESS            // enable Light Lightness server model
#undef  USE_CTL                 // disable Light CTL server model
#undef USE_SCENE               // enable Light Scene server model
#define USE_VENDORMODEL         // enable Light vendormodel server model

#ifdef USE_VENDORMODEL
    #define  EASY_BOUNDING
#endif



/* Console Input/Output */
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)

void appl_dump_bytes(UCHAR* buffer, UINT16 length);
void appl_mesh_sample (void);
MS_NET_ADDR     ui_raw_data_destnation_address;

/* Macro to get dst address */
#define UI_GET_RAW_DATA_DST_ADDR(addr) \
    (addr) = ui_raw_data_destnation_address

/* Macro to set dst address */
#define UI_SET_RAW_DATA_DST_ADDR(addr) \
    (ui_raw_data_destnation_address) = addr



#define VENDOR_PRODUCT_MAC_ADDR         0x4000
#define PROCFG_COMPLETE_TIMEOUT         10


#define UI_PROV_START_ADDRESS           0x0002
#define UI_PROV_STOP_ADDRESS            0x7FFF


/* Provisioner */
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

#define UI_DEVICE_UUID      {0x05, 0x04, 0x62, 0x12, 0x00, 0x00, 0x00, 0x01, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00, 0x00}

/* AppKey to be used by the configuration client */
#define UI_APPKEY           {0x50, 0x48, 0x59, 0x50, 0x4C, 0x55, 0x53, 0x49, 0x6e, 0x63, 0x41, 0x70, 0x70, 0x4b, 0x65, 0x79}



/* ----------------------------------------- External Global Variables */
extern uint8 g_buttonCounter;




/* ----------------------------------------- Exported Global Functions */



/* ----------------------------------------- Static Global Variables */
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

/** Data exchanged during Provisiong procedure */
PROV_DATA_S UI_prov_data =
{
    /** NetKey */
    { 0x50, 0x48, 0x59, 0x50, 0x4C, 0x55, 0x53, 0x49, 0x6e, 0x63, 0x4e, 0x65, 0x74, 0x4b, 0x65, 0x79 },

    /** Index of the NetKey */
    0x0000,

    /** Flags bitmask */
    0x00,

    /** Current value of the IV index */
    0x00000000,

    /** Unicast address of the primary element */
    0x0002
};

/** Default Provisioning method to start */
DECL_STATIC PROV_METHOD_S UI_prov_method =
{
    /** Algorithm */
    PROV_ALGO_EC_FIPS_P256,

    /** Public Key */
    PROV_PUBKEY_NO_OOB,

    /** Authentication Method */
    PROV_AUTH_OOB_NONE,

    /** OOB Information */
    {0x0000, 0x00}
};

/** Current role of application - Provisioner/Device */
DECL_STATIC UCHAR UI_prov_role;

/** Check subnet is valid */
DECL_STATIC UCHAR UI_subnet_valid;


///** Provisioning Handle */
//DECL_STATIC PROV_HANDLE UI_prov_handle;

/** Device UUID Identifier */
DECL_STATIC UCHAR UI_device_uuid[MS_DEVICE_UUID_SIZE] = UI_DEVICE_UUID;

/* Configuration Client Model Handle */
MS_ACCESS_MODEL_HANDLE   UI_config_client_model_handle;

/* Provsion timeout handle */
DECL_STATIC EM_timer_handle procfg_timer_handle;


/** Appkey to be used for model binding */
DECL_STATIC UCHAR UI_appkey[MS_ACCESS_APPKEY_SIZE] = UI_APPKEY;

/* Generic ONOFF Client Model Handle */
DECL_STATIC MS_ACCESS_MODEL_HANDLE   UI_generic_onoff_client_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_client_model_handle;




typedef struct MS_state_vendor_example_struct
{
    UINT16  maun_opcode;
    UCHAR  value;

} MS_STATE_VENDOR_EXAMPLE_STRUCT;

typedef struct MS_state_vendor_model_hsl_struct
{
    /** The perceived lightness of a light emitted by the element */
    UINT16 hsl_lightness;

    /** The 16-bit value representing the hue */
    UINT16 hsl_hue;

    /** The saturation of a color light */
    UINT16 hsl_saturation;
} MS_STATE_VENDOR_MODEL_HSL_STRUCT;


typedef struct MS_state_vendor_example_test_struct
{
    UINT16  total_len;
    UINT8   is_to_be_ack;
    UINT16  message_index;
    UINT16  osal_tick;
    UINT8   ttl;
    UINT8   data_len;

} MS_STATE_VENDOR_EXAMPLE_TEST_STRUCT;


extern UCHAR blebrr_prov_started;
extern uint32            osal_sys_tick;

MS_PROV_DEV_ENTRY g_prov_dev_list[MS_MAX_DEV_KEYS];
UINT16  g_num_entries;



static API_RESULT ui_check_destnation_address(MS_NET_ADDR check_dst_addr)
{
    API_RESULT retval;
    UINT8   saddr_type;
    MS_PROV_DEV_ENTRY prov_dev_list[MS_MAX_DEV_KEYS];
    UINT16            num_entries;
    UINT16            pointer_entries;
    retval = API_FAILURE;
    saddr_type = MS_net_get_address_type(check_dst_addr);

    if (MS_NET_ADDR_TYPE_UNICAST == saddr_type)
    {
        retval = MS_access_cm_get_prov_devices_list
                 (
                     &prov_dev_list[0],
                     &num_entries,
                     &pointer_entries
                 );

        if((retval == API_SUCCESS) && (num_entries != 0))
        {
            for(UINT16 i = 0; i < num_entries; i++)
            {
                if(prov_dev_list[i].uaddr == check_dst_addr)
                {
                    retval = API_SUCCESS;
                    break;
                }
            }
        }
    }
    else
    {
        retval = API_SUCCESS;
    }

    return retval;
}

static void UI_netkey_generate(UINT8* netkey)
{
    UINT8   old_key[16];
    uint32 address = VENDOR_PRODUCT_MAC_ADDR;
    hal_flash_read(address ++, &UI_device_uuid[10], 1);
    hal_flash_read(address ++, &UI_device_uuid[11], 1);
    hal_flash_read(address ++, &UI_device_uuid[12], 1);
    hal_flash_read(address ++, &UI_device_uuid[13], 1);
    hal_flash_read(address ++, &UI_device_uuid[8], 1);
    hal_flash_read(address ++, &UI_device_uuid[9], 1);
//    UI_device_uuid[10] = (uint8_t)ReadFlash(address ++);
//    UI_device_uuid[11] = (uint8_t)ReadFlash(address ++);
//    UI_device_uuid[12] = (uint8_t)ReadFlash(address ++);
//    UI_device_uuid[13] = (uint8_t)ReadFlash(address ++);
//    UI_device_uuid[8]  = (uint8_t)ReadFlash(address ++);
//    UI_device_uuid[9]  = (uint8_t)ReadFlash(address ++);
    UI_device_uuid[14] = osal_sys_tick & 0xff;
    UI_device_uuid[15] = (osal_sys_tick>>8) & 0xff;
    EM_mem_copy(old_key, netkey, 16);
    LL_ENC_AES128_Encrypt(old_key,UI_device_uuid,netkey);
}

API_RESULT UI_set_provision_data(MS_NET_ADDR provision_addr)
{
    /* Holding a temporary structure for local prov data */
    PROV_DATA_S temp_ps_prov_data;
    API_RESULT retval;
    EM_mem_copy
    (
        &temp_ps_prov_data,
        &UI_prov_data,
        sizeof(UI_prov_data)
    );
    /**
        Assigning the Local Unicast Address of the Provisioner
        to the Access Layer along with other related keys.
    */
    temp_ps_prov_data.uaddr = provision_addr;
    temp_ps_prov_data.ivindex = ms_iv_index.iv_index;
    /* Provide Provisioning Data to Access Layer */
    retval = MS_access_cm_set_prov_data_provsioner
             (
                 &temp_ps_prov_data
             );
    return retval;
}




/* ----------------------------------------- Functions */
/* Model Client - Configuration Models */
/* Send Config Composition Data Get */
void UI_config_client_get_composition_data(UCHAR page)
{
    API_RESULT retval;
    ACCESS_CONFIG_COMPDATA_GET_PARAM  param;
    CONSOLE_OUT
    ("Send Config Composition Data Get\n");
    param.page = page;
    retval = MS_config_client_composition_data_get(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

API_RESULT UI_sample_binding_app_key(void)
{
    MS_APPKEY_HANDLE  handle;
    UINT8*              key;
    UINT8             aid;
    DECL_CONST UINT8  t_key[16] = {0};
    API_RESULT retval;
    handle = 0x0000;
    retval = MS_access_cm_get_app_key
             (
                 handle,
                 &key,
                 &aid
             );

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
            #ifdef  USE_VENDORMODEL
            retval=MS_access_bind_model_app(UI_vendor_defined_client_model_handle, handle);
            CONSOLE_OUT("BINDING App Key %04x (%04x %04x)\n",retval,UI_vendor_defined_client_model_handle,handle);
            #endif
        }
    }

    return retval;
}


/* Send Config Appkey Add */
void UI_config_client_appkey_binding(UINT16 netkey_index, UINT16 appkey_index, UCHAR* appkey)
{
    ACCESS_CONFIG_APPKEY_ADD_PARAM  param;
    CONSOLE_OUT
    ("Binding Config Appkey Add\n");
    param.netkey_index = netkey_index;
    param.appkey_index = appkey_index;
    EM_mem_copy(param.appkey, appkey, MS_ACCESS_APPKEY_SIZE);
    /* Update local database */
    MS_access_cm_add_appkey
    (
        0, /* subnet_handle */
        param.appkey_index, /* appkey_index */
        &param.appkey[0] /* app_key */
    );
    UI_sample_binding_app_key();
}



/* Send Config Appkey Add */
void UI_config_client_appkey_add(UINT16 netkey_index, UINT16 appkey_index, UCHAR* appkey)
{
    API_RESULT retval;
    ACCESS_CONFIG_APPKEY_ADD_PARAM  param;
    CONSOLE_OUT
    ("Send Config Appkey Add\n");
    param.netkey_index = netkey_index;
    param.appkey_index = appkey_index;
    EM_mem_copy(param.appkey, appkey, MS_ACCESS_APPKEY_SIZE);
    /* Update local database */
//    MS_access_cm_add_appkey
//    (
//        0, /* subnet_handle */
//        param.appkey_index, /* appkey_index */
//        &param.appkey[0] /* app_key */
//    );
    retval = MS_config_client_appkey_add(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

/* Send Config Relay Set */
void UI_config_proxy_set(UINT8 en)
{
    ACCESS_CONFIG_GATT_PROXY_SET_PARAM  param;
    CONSOLE_OUT
    (">> Send Config Gatt Proxy Set\n");
    param.proxy = (UCHAR)en;
    MS_config_client_gatt_proxy_set(&param);
}


void UI_config_client_model_app_bind(UINT16 addr, UINT16 appkey_index, UCHAR model_type, UINT32 model_id)
{
    API_RESULT retval;
    ACCESS_CONFIG_MODEL_APP_BIND_PARAM  param;
    CONSOLE_OUT
    ("Send Config Model App Bind\n");
    param.element_address = addr;
    param.appkey_index = appkey_index;
    param.model.type = model_type;
    param.model.id = model_id;
    retval = MS_config_client_model_app_bind(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

/* Set Publish Address */
void UI_set_publish_address(UINT16 addr, MS_ACCESS_MODEL_HANDLE model_handle,UINT8 config_mode)
{
    API_RESULT retval;
    MS_ACCESS_PUBLISH_INFO  publish_info;
    MS_ACCESS_DEV_KEY_HANDLE dev_key_handle;
    /* Set Publish Information */
    EM_mem_set(&publish_info, 0, sizeof(publish_info));
    publish_info.addr.use_label = MS_FALSE;
    publish_info.addr.addr = addr;

    if(config_mode)
    {
        publish_info.remote = MS_FALSE;
        retval = MS_access_cm_get_device_key_handle
                 (
                     publish_info.addr.addr,
                     &dev_key_handle
                 );

        if (API_SUCCESS == retval)
        {
            publish_info.appkey_index = MS_CONFIG_LIMITS(MS_MAX_APPS) + dev_key_handle;
            CONSOLE_OUT("DevKey -> AppKey Index: 0x%04X\n", publish_info.appkey_index);
        }
    }
    else
    {
        publish_info.remote = MS_TRUE;
        publish_info.appkey_index = 0;
        CONSOLE_OUT("AppKey Index: 0x%04X\n", publish_info.appkey_index);
    }

    retval = MS_access_cm_set_model_publication
             (
                 model_handle,
                 &publish_info
             );

    if (API_SUCCESS == retval)
    {
        CONSOLE_OUT
        ("Publish Address is set Successfully.\n");
    }
    else
    {
        CONSOLE_OUT
        ("Failed to set publish address. Status 0x%04X\n", retval);
    }

    return;
}

void UI_generic_onoff_set(UCHAR state)
{
    API_RESULT retval;
    MS_GENERIC_ONOFF_SET_STRUCT  param;
    CONSOLE_OUT
    ("Send Generic Onoff Set\n");
    param.onoff = state;
    param.tid = 0;
    param.optional_fields_present = 0x00;
    retval = MS_generic_onoff_set(&param);
    CONSOLE_OUT
    ("Retval - 0x%04X\n", retval);
}

extern uint32_t vendor_set_index;

void UI_vendor_model_set_raw_addr(void)
{
//    printf("set 0x%04X\n",g_prov_dev_list[vendor_set_index].uaddr);
    UI_SET_RAW_DATA_DST_ADDR(g_prov_dev_list[vendor_set_index++].uaddr);

    if(vendor_set_index >= g_num_entries)
    {
        vendor_set_index = 0;
    }
}


// ================= Vendor model Client model Functions  ========
void UI_vendor_model_set(UCHAR ack_en,UCHAR test_len,UINT16 test_index)
{
    API_RESULT retval;
    MS_NET_ADDR addr;
    UCHAR   is_prov_req;
    UCHAR      buffer[256+2];
    UINT16      marker;
    MS_NET_ADDR    dst_addr;
    MS_ACCESS_VENDORMODEL_STATE_PARAMS set_param;
    is_prov_req = MS_TRUE;
    marker = 0;
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
        UI_GET_RAW_DATA_DST_ADDR(dst_addr);
        retval =  ui_check_destnation_address(dst_addr);

        if(retval == API_SUCCESS)
        {
            #if 0
            MS_ACCESS_PHY_MODEL_STATE_PARAMS set_param;
            set_param.phy_mode_param = NULL;
            set_param.phy_mode_type = MS_STATE_VENDOR_ONOFF_T;
            set_param.phy_mode_param = NULL;
            retval=MS_phymodel_client_send_reliable_pdu(MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE,&set_param);
            #endif
            #if 1
            UINT8      ttl;
            MS_STATE_VENDOR_EXAMPLE_TEST_STRUCT  param;
            ACCESS_CM_GET_DEFAULT_TTL(ttl);
            param.total_len = test_len+7;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param.total_len);
            marker += 2;
            param.is_to_be_ack = ack_en;
            MS_PACK_LE_1_BYTE_VAL(&buffer[marker], param.is_to_be_ack);
            marker++;
            param.message_index = test_index;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param.message_index);
            marker += 2;
            param.osal_tick = osal_sys_tick&0xffff;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param.osal_tick);
            marker += 2;
            param.ttl = ttl;
            MS_PACK_LE_1_BYTE_VAL(&buffer[marker], param.ttl);
            marker++;
            param.data_len = test_len;
            MS_PACK_LE_1_BYTE_VAL(&buffer[marker], param.data_len);
            marker++;

            if(param.data_len)
            {
                EM_mem_set(&buffer[marker], 0, param.data_len);
                marker += param.data_len;
            }

            set_param.vendormodel_param = &buffer[0];
            CONSOLE_OUT("[PDU_Tx] Pkt.INDEX:0x%04X,SRC:0x%04X,DST:0x%04X,TTL:0x%02X,TICK:0x%08X\r\n",
                        param.message_index,addr,dst_addr,param.ttl,osal_sys_tick);
            retval = MS_vendormodel_client_send_reliable_pdu
                     (
                         MS_ACCESS_VENDORMODEL_NOTIFY_OPCODE,
                         &set_param,
                         dst_addr
                     );

            if(retval != API_SUCCESS)
            {
                printf("send error %x\n",retval);
            }

            #endif
            #if 0
            set_param.vendormodel_param = NULL;
            set_param.phy_mode_type = MS_STATE_VENDORMODEL_ONOFF_T;

            switch(set_param.vendormodel_type)
            {
            case MS_STATE_PHYPLUSMODEL_ONOFF_T:
            {
                set_param.vendormodel_param = EM_alloc_mem(1);
                set_param.vendormodel_param[0] = (g_buttonCounter++)&0x01;
            }
            }

            retval=MS_vendormodel_client_send_reliable_pdu(MS_ACCESS_PHYPLUSMODEL_SET_OPCODE,&set_param,0,dst_addr);

            if(NULL != set_param.vendormodel_param)
            {
                EM_free_mem(set_param.phy_mode_param);
            }

            #endif
            #if 0
            MS_STATE_VENDOR_MODEL_HSL_STRUCT param_hsl;
            set_param.phy_mode_type = MS_STATE_PHY_MODEL_HSL_T;
            g_buttonCounter++;

            if((g_buttonCounter&0x03) == 0x00)
            {
                param_hsl.hsl_lightness = 0x8000;
                param_hsl.hsl_hue = 0x0000;
                param_hsl.hsl_saturation = 0xFFFF;
            }
            else if((g_buttonCounter&0x03) == 0x01)
            {
                param_hsl.hsl_lightness = 0x7FFF;
                param_hsl.hsl_hue = 0xAAAA;
                param_hsl.hsl_saturation = 0xFFFF;
            }
            else if((g_buttonCounter&0x03) == 0x02)
            {
                param_hsl.hsl_lightness = 0x7FFF;
                param_hsl.hsl_hue = 0x5555;
                param_hsl.hsl_saturation = 0xFFFF;
            }
            else
            {
                param_hsl.hsl_lightness = 0xFFFF;
                param_hsl.hsl_hue = 0xFFFF;
                param_hsl.hsl_saturation = 0xFFFF;
            }

            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_hsl.hsl_lightness);
            marker += 2;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_hsl.hsl_hue);
            marker += 2;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_hsl.hsl_saturation);
            marker += 2;
            set_param.vendormodel_param = &buffer[0];
            retval=MS_vendormodel_client_send_reliable_pdu(MS_ACCESS_PHYPLUSMODEL_SET_OPCODE,&set_param,dst_addr);
            #endif
        }
        else
        {
            CONSOLE_OUT
            ("Error destnation address\n");
        }
    }
    else
    {
//        osal_stop_timerEx(bleMesh_TaskID, BLEMESH_PDU_TX_OVERRUN);
        CONSOLE_OUT
        ("An Unprovisioned Device\n");
    }
}

// ================= Vendor model Client model Functions  ========
void UI_vendor_model_reset(MS_NET_ADDR src_addr)
{
    API_RESULT retval;
    MS_NET_ADDR addr;
    UCHAR   is_prov_req;
    MS_NET_ADDR    dst_addr;
    UCHAR      buffer[8];
    UINT16      marker;
    UCHAR*        pdu_ptr;
//    UINT16      key_handle;
    marker = 0;
    buffer[marker++] = ++vendor_tid;
    MS_PACK_LE_2_BYTE_VAL(&buffer[marker], MS_STATE_VENDORMODEL_RESET_T);
    marker += 2;
    is_prov_req = MS_TRUE;

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

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
        dst_addr = src_addr;
        retval = MS_access_publish_ex
                 (
                     &UI_vendor_defined_client_model_handle,
                     MS_ACCESS_VENDORMODEL_WRITECMD_OPCODE,
                     dst_addr,
                     pdu_ptr,
                     marker,
                     MS_FALSE
                 );
    }
    else
    {
        CONSOLE_OUT
        ("An Unprovisioned Device\n");
    }
}

#if (CFG_HEARTBEAT_MODE)
void UI_update_heartbeat_flag_with_uaddr(MS_NET_ADDR saddr)
{
    API_RESULT    retval;
    UINT16 i;
    MS_PROV_DEV_ENTRY prov_dev_list[MS_MAX_DEV_KEYS];
    UINT16            num_entries;
    UINT16            pointer_entries;
    retval = MS_access_cm_get_prov_devices_list
             (
                 &prov_dev_list[0],
                 &num_entries,
                 &pointer_entries
             );
    retval = API_FAILURE;

    for(i = 0; i < num_entries; i++)
    {
        if(prov_dev_list[i].uaddr == saddr)
        {
            retval = API_SUCCESS;
            break;
        }
    }

    if(retval == API_FAILURE)
    {
        printf("reset\n");
        blebrr_scan_pl(FALSE);  //by hq
        UI_vendor_model_reset(saddr);
        return;
    }

    for(i = 0; i < g_num_entries; i++)
    {
        if(g_prov_dev_list[i].uaddr == saddr)
        {
            g_prov_dev_list[i].rcv_flag = 1;
            break;
        }
    }

    /*Cannot find unicast address,Send reset with vendor model*/
}
#endif


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
API_RESULT UI_phy_model_client_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*         ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*             msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*               req_type,
    /* IN */ MS_ACCESS_VENDORMODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*              ext_params
)
{
    API_RESULT retval;
    UINT32  opcode;
    MS_NET_ADDR              saddr;
    UINT16  marker = 0;
    retval = API_SUCCESS;
    opcode = msg_raw->opcode;
    saddr = ctx->saddr;

//    CONSOLE_OUT(
//    "[VENDOR_EXAMPLE] %04x (d%04x s%04x).\n",ctx->handle,ctx->daddr,ctx->saddr);
//    CONSOLE_OUT(
//    "[VENDOR_EXAMPLE] opcode %08x.\n",msg_raw->opcode);
    switch(opcode)
    {
    case MS_ACCESS_VENDORMODEL_STATUS_OPCODE:
    {
    }
    break;

    case MS_ACCESS_VENDORMODEL_INDICATION_OPCODE:
    {
    }
    break;

    case MS_ACCESS_VENDORMODEL_WRITECMD_OPCODE:
    {
//            CONSOLE_OUT(
//            "MS_ACCESS_PHY_MODEL_WRITECMD_OPCODE\n");
        switch(state_params->vendormodel_type)
        {
        case MS_STATE_VENDORMODEL_HB_CALLBACK_T:
        {
            #if (CFG_HEARTBEAT_MODE)
            UI_update_heartbeat_flag_with_uaddr(saddr);
            #endif
        }
        break;
        }
    }
    break;

    case MS_ACCESS_VENDORMODEL_NOTIFY_OPCODE:
    {
        uint16      message_index;
        UINT8       ttl;
        marker = 2;
        MS_UNPACK_LE_2_BYTE(&message_index, msg_raw->data_param+marker);
        marker += 4;
        MS_UNPACK_LE_1_BYTE(&ttl, msg_raw->data_param+marker);
        marker ++;
        CONSOLE_OUT("[PDU_Rx] Pkt.INDEX:0x%04X,SRC:0x%04X,DST:0x%04X,TTL:0x%02X,TICK:0x%08X\r\n",
                    message_index,ctx->daddr,saddr,ttl,osal_sys_tick);
    }
    break;

    default:
        CONSOLE_OUT(
            "MS_ACCESS_PHY_MODEL_NONE_OPCODE\n");
        break;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[PHY_MODEL] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        retval = MS_vendormodel_client_state_update(ctx, state_params, NULL, 0, NULL,marker);
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
    retval = MS_vendormodel_client_init
             (
                 element_handle,
                 &UI_vendor_defined_client_model_handle,
                 UI_phy_model_client_cb
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


/**
    \brief Client Application Asynchronous Notification Callback.

    \par Description
    Configuration client calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT UI_config_client_cb
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
        "[CONFIG_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_CONFIG_GATT_PROXY_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_GATT_PROXY_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_SIG_MODEL_APP_LIST_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_SIG_MODEL_APP_LIST_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_COMPOSITION_DATA_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_COMPOSITION_DATA_STATUS_OPCODE\n");
        /* Add Appkey */
        UI_config_client_appkey_add(0, 0, UI_appkey);
    }
    break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_NETKEY_LIST_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_NETKEY_LIST_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_LIST_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_LIST_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_LIST_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_LIST_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_APPKEY_LIST_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_APPKEY_LIST_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_APPKEY_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_APPKEY_STATUS_OPCODE\n");
        #ifdef EASY_BOUNDING
        UI_SET_RAW_DATA_DST_ADDR(UI_prov_data.uaddr);
        MS_access_cm_set_transmit_state(MS_NETWORK_TX_STATE, (0<<3)|0);
        /* Set provision started */
        blebrr_prov_started = MS_FALSE;
        EM_stop_timer(&procfg_timer_handle);
        CONSOLE_OUT(
            "PROVISION AND CONFIG DONE!!!\n");
//        UI_config_proxy_set(0);
        /* Send a Generic ON */
        #else
        /* Bind the model to Appkey */
        UI_config_client_model_app_bind(UI_prov_data.uaddr, 0, MS_ACCESS_MODEL_TYPE_SIG, MS_MODEL_ID_GENERIC_ONOFF_SERVER);
        #endif
    }
    break;

    case MS_ACCESS_CONFIG_RELAY_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_RELAY_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_FRIEND_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_FRIEND_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_MODEL_APP_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_MODEL_APP_STATUS_OPCODE\n");
        #ifndef EASY_BOUNDING
//            /* Set the Publish address for Config Client */
//            UI_set_publish_address(UI_prov_data.uaddr, UI_generic_onoff_client_model_handle,MS_FALSE);
//            UI_set_publish_address(UI_prov_data.uaddr, UI_vendor_defined_client_model_handle,MS_FALSE);
        UI_SET_RAW_DATA_DST_ADDR(UI_prov_data.uaddr);
        MS_access_cm_set_transmit_state(MS_NETWORK_TX_STATE, (0<<3)|0);
        /* Set provision started */
        blebrr_prov_started = MS_FALSE;
        EM_stop_timer(procfg_timer_handle);
        procfg_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
        CONSOLE_OUT(
            "PROVISION AND CONFIG DONE!!!\n");
        /* Send a Generic ON */
        #endif
    }
    break;

    case MS_ACCESS_CONFIG_NODE_RESET_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_NODE_RESET_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_DEFAULT_TTL_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_DEFAULT_TTL_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_NETWORK_TRANSMIT_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_BEACON_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_BEACON_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE\n");
    }
    break;

    case MS_ACCESS_CONFIG_VENDOR_MODEL_APP_LIST_OPCODE:
    {
        CONSOLE_OUT(
            "MS_ACCESS_CONFIG_VENDOR_MODEL_APP_LIST_OPCODE\n");
    }
    break;
    }

    return retval;
}

API_RESULT UI_register_config_model_client
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    /* Configuration Client */
    API_RESULT retval;
    CONSOLE_OUT("In Model Client - Configuration Models\n");
    retval = MS_config_client_init
             (
                 element_handle,
                 &UI_config_client_model_handle,
                 UI_config_client_cb
             );
    CONSOLE_OUT("Config Model Client Registration Status: 0x%04X\n", retval);
    return retval;
}


/* ---- Generic OnOff Handlers */

/* Generic OnOff Model Client */
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
    /* Generic OnOff Client */
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

void UI_provcfg_complete_timeout_handler(void* args, UINT16 size)
{
    MS_ACCESS_DEV_KEY_HANDLE dev_key_handle;
    MS_access_cm_get_device_key_handle
    (
        UI_prov_data.uaddr,
        &dev_key_handle
    );
    MS_access_cm_delete_device_key(dev_key_handle);
    net_delete_from_cache(UI_prov_data.uaddr);
    ltrn_delete_from_reassembled_cache(UI_prov_data.uaddr);
    ltrn_delete_from_replay_cache(UI_prov_data.uaddr);
    MS_config_client_node_reset();
    printf("Delete Uaddr:0x%04X\n",UI_prov_data.uaddr);
    /* Set provision started */
    blebrr_prov_started = MS_FALSE;
    printf("Provisining and config Complete Timeout\n");
}

#if (CFG_HEARTBEAT_MODE)
API_RESULT UI_trn_stop_heartbeat_publication(void)
{
    API_RESULT retval = API_SUCCESS;
    trn_stop_heartbeat_pub_timer();
    UI_subnet_valid = 1;
    return retval;
}

API_RESULT UI_trn_set_heartbeat_publication(void)
{
    API_RESULT retval;
    MS_TRN_HEARTBEAT_PUBLICATION_INFO hb_info;
    UINT8 feature;
    MS_SUBNET_HANDLE   subnet_handle;
    hb_info.daddr = 0xCFFF;
    hb_info.count_log = 0xFF;
    hb_info.period_log = 0x03;
    MS_access_cm_get_features(&feature);
    hb_info.features = feature;
    subnet_handle = 0;
    hb_info.netkey_index = subnet_handle;
    hb_info.ttl = 0x05;
    retval = MS_trn_set_heartbeat_publication(&hb_info);
    return retval;
}
#endif

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
    PROV_OOB_TYPE_S* oob_info;
    API_RESULT retval;
    MS_PROV_DEV_ENTRY prov_dev_list[MS_MAX_DEV_KEYS];
    UCHAR authstr[PROV_AUTHVAL_SIZE_PL << 1];
    UINT32 authnum;
    UCHAR authtype;
    UCHAR* pauth;
    UINT16 authsize;
    UINT8 key_refresh_state;
    UINT16            num_entries;
    UINT16            pointer_entries;

    switch (event_type)
    {
    case PROV_EVT_UNPROVISIONED_BEACON:
        /* Reference the beacon pointer */
        rdev = (PROV_DEVICE_S*)event_data;
        retval = API_SUCCESS;

        if((blebrr_prov_started == MS_TRUE) ||
                (blebrr_get_queue_depth()>(BLEBRR_QUEUE_SIZE>>1))
                || (MS_key_refresh_active == MS_TRUE))
        {
            break;
        }

        if (0 != EM_mem_cmp(rdev->uuid, UI_device_uuid, 4))
        {
            //CONSOLE_OUT ("ERR Recvd PROV_EVT_UNPROVISIONED_BEACON\n");
            /* Beacon not from device of interest. Do no process */
            break;
        }

        CONSOLE_OUT ("Recvd PROV_EVT_UNPROVISIONED_BEACON\n");
        CONSOLE_OUT ("Status - 0x%04X\n", event_result);
        CONSOLE_OUT ("Mac: %02X %02X %02X %02X %02X %02X\n",rdev->uuid[8],
                     rdev->uuid[9],rdev->uuid[10],rdev->uuid[11],rdev->uuid[12],rdev->uuid[13]);
        CONSOLE_OUT ("Status - 0x%04X\n", event_result);
        /* Set provision started */
        blebrr_prov_started = MS_TRUE;
        /* Bind to the device */
        retval = MS_prov_bind(PROV_BRR_ADV, rdev, UI_PROV_DEVICE_ATTENTION_TIMEOUT, phandle);
        CONSOLE_OUT("Retval - 0x%04X\n", retval);
        break;

    case PROV_EVT_PROVISIONING_SETUP:
        CONSOLE_OUT("Recvd PROV_EVT_PROVISIONING_SETUP\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);

        /* Decipher the data based on the role */
        if (PROV_ROLE_PROVISIONER == UI_prov_role)
        {
            /* Display the capabilities */
            rcap = (PROV_CAPABILITIES_S*)event_data;
            CONSOLE_OUT ("Remote Device Capabilities:\n");
            CONSOLE_OUT ("\tNum Elements     - %d\n", rcap->num_elements);
            CONSOLE_OUT ("\tSupp Algorithms  - 0x%04X\n", rcap->supp_algorithms);
            CONSOLE_OUT ("\tSupp PublicKey   - 0x%02X\n", rcap->supp_pubkey);
            CONSOLE_OUT ("\tSupp Static OOB  - 0x%02X\n", rcap->supp_soob);
            CONSOLE_OUT ("\tOutput OOB Size  - %d\n", rcap->ooob.size);
            CONSOLE_OUT ("\tOutput OOB Action- 0x%04X\n", rcap->ooob.action);
            CONSOLE_OUT ("\tInput OOB Size   - %d\n", rcap->ioob.size);
            CONSOLE_OUT ("\tInput OOB Action - 0x%04X\n", rcap->ioob.action);
            /* Start with default method */
            CONSOLE_OUT ("Start Provisioning with default method...\n");
            retval = MS_prov_start (phandle, &UI_prov_method);
            CONSOLE_OUT ("Retval - 0x%04X\n", retval);
        }
        else
        {
            /* Display the attention timeout */
            CONSOLE_OUT("Attention TImeout - %d\n", *((UCHAR*)event_data));
        }

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
        retval = MS_prov_set_authval(phandle, pauth, authsize);
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

    case PROV_EVT_PROVDATA_INFO_REQ:
        CONSOLE_OUT ("Recvd PROV_EVT_PROVDATA_INFO_REQ\n");
        CONSOLE_OUT ("Status - 0x%04X\n", event_result);
        /* Send Provisioning Data */
        CONSOLE_OUT ("Send Provisioning Data...\n");
        /* Update the next device address if provisioned devices are present in database */
        retval = MS_access_cm_get_prov_devices_list
                 (
                     &prov_dev_list[0],
                     &num_entries,
                     &pointer_entries
                 );

        if ((API_SUCCESS == retval) /*&&
                (0 != num_entries)*/)
        {
//                UI_prov_data.uaddr = prov_dev_list[num_entries - 1].uaddr +
//                                     prov_dev_list[num_entries - 1].num_elements;
            UI_prov_data.uaddr = pointer_entries;
        }

        printf("Updating Provisioning Start Addr to 0x%04X\n", UI_prov_data.uaddr);
        //Get network key
        MS_access_cm_get_netkey_at_offset(0,0,UI_prov_data.netkey);
        //Get key refresh state
        MS_access_cm_get_key_refresh_phase(0,&key_refresh_state);
        key_refresh_state = (MS_ACCESS_KEY_REFRESH_PHASE_2 == key_refresh_state) ? 0x01 : 0x00;
        UI_prov_data.ivindex = ms_iv_index.iv_index;
        UI_prov_data.flags = ((ms_iv_index.iv_update_state & 0x01) << 1) | key_refresh_state;
        blebrr_scan_pl(FALSE);  //by hq
        retval = MS_prov_data (phandle, &UI_prov_data);
        CONSOLE_OUT ("Retval - 0x%04X\n", retval);
        break;

    case PROV_EVT_PROVISIONING_COMPLETE:
        CONSOLE_OUT("Recvd PROV_EVT_PROVISIONING_COMPLETE\n");
        CONSOLE_OUT("Status - 0x%04X\n", event_result);

        if (API_SUCCESS == event_result)
        {
            if (PROV_ROLE_PROVISIONER == UI_prov_role)
            {
                retval = EM_start_timer
                         (
                             &procfg_timer_handle,
                             PROCFG_COMPLETE_TIMEOUT,
                             UI_provcfg_complete_timeout_handler,
                             NULL,
                             0
                         );
//                    if (0x0001 != UI_prov_data.uaddr)
//                    {
//                        /* Holding a temporary structure for local prov data */
//                        PROV_DATA_S temp_ps_prov_data;
//                        EM_mem_copy
//                        (
//                            &temp_ps_prov_data,
//                            &UI_prov_data,
//                            sizeof(UI_prov_data)
//                        );
//                        /**
//                         * Assigning the Local Unicast Address of the Provisioner
//                         * to the Access Layer along with other related keys.
//                         */
//                        temp_ps_prov_data.uaddr = 0x0001;
//                        blebrr_scan_pl(FALSE);    //by hq
//                        hal_gpio_toggle(P3);
//                        /* Provide Provisioning Data to Access Layer */
//                        MS_access_cm_set_prov_data
//                        (
//                            &temp_ps_prov_data
//                        );
//                        hal_gpio_toggle(P3);
//                        /**
//                         *  NOTE:
//                         *  Increment the appl_prov_data.uaddr for the next
//                         *  set of device which is getting provisioned based on
//                         *  the address and number of elements present in the last
//                         *  provisioned device.
//                         */
//                    }
                /* Set the Publish address for Config Client */
                UI_set_publish_address(UI_prov_data.uaddr, UI_config_client_model_handle,MS_TRUE);
                net_delete_from_cache(UI_prov_data.uaddr);
                ltrn_delete_from_reassembled_cache(UI_prov_data.uaddr);
                ltrn_delete_from_replay_cache(UI_prov_data.uaddr);

                if(UI_subnet_valid)
                {
                    #if (CFG_HEARTBEAT_MODE)
                    UI_trn_set_heartbeat_publication();
                    #endif
                    UI_subnet_valid = 0;
                    MS_net_start_snb_timer(0);
                }

                #ifndef EASY_BOUNDING
                /* Get the Composition data */
                UI_config_client_get_composition_data(0x00);
                #else
                UI_config_client_appkey_add(0, 0, UI_appkey);
                #endif
            }
        }
        else
        {
            blebrr_prov_started = MS_FALSE;
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

    if (PROV_ROLE_PROVISIONER == role)
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
        CONSOLE_OUT("Retval - 0x%04X\n", retval);
    }
}

void appl_mesh_sample (void)
{
    MS_ACCESS_NODE_ID node_id;
    MS_ACCESS_ELEMENT_DESC   element;
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    API_RESULT retval;
    UCHAR role, brr;
    MS_CONFIG* config_ptr;
    MS_PROV_DEV_ENTRY prov_dev_list[MS_MAX_DEV_KEYS];
    UINT8 key_refresh_state;
    UINT16            num_entries;
    UINT16            pointer_entries;
    MS_access_ps_store_disable(MS_TRUE);
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
    ms_start_unicast_addr = UI_PROV_START_ADDRESS;
    ms_stop_unicast_addr = UI_PROV_STOP_ADDRESS;
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
        /* Register Configuration model client */
        retval = UI_register_config_model_client(element_handle);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model client */
        retval = UI_register_generic_onoff_model_client(element_handle);
    }

    #ifdef  USE_VENDORMODEL

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_register_vendor_defined_model_client(element_handle);
    }

    #endif
    UI_config_client_appkey_binding(0,0,UI_appkey);
    printf("0x%08X 0x%08X 0x%02X\n",ms_iv_index.iv_index,ms_iv_index.iv_expire_time,ms_iv_index.iv_update_state);
    UI_prov_data.uaddr = 0x0001;
    //Get network key
    retval = MS_access_cm_get_netkey_at_offset(0,0,UI_prov_data.netkey);

    if(retval == ACCESS_NO_MATCH)     //if can't find netkey,generate it
    {
        UI_netkey_generate(UI_prov_data.netkey);
    }

    //Get key refresh state
    MS_access_cm_get_key_refresh_phase(0,&key_refresh_state);
    key_refresh_state = (MS_ACCESS_KEY_REFRESH_PHASE_2 == key_refresh_state) ? 0x01 : 0x00;
    UI_prov_data.ivindex = ms_iv_index.iv_index;
    UI_prov_data.flags = ((ms_iv_index.iv_update_state & 0x01) << 1) | key_refresh_state;
    retval = UI_set_provision_data(UI_prov_data.uaddr);
    /* Set the Publish address for onoff and vendor model Client */
    UI_prov_data.uaddr = 0xCFFF;
    UI_set_publish_address(UI_prov_data.uaddr, UI_generic_onoff_client_model_handle,MS_FALSE);
    UI_set_publish_address(UI_prov_data.uaddr, UI_vendor_defined_client_model_handle,MS_FALSE);
    /* Configure as provisioner */
    UI_register_prov();
    /**
        setup <role:[1 - Device, 2 - Provisioner]> <bearer:[1 - Adv, 2 - GATT]
    */
    role = PROV_ROLE_PROVISIONER;
    brr = PROV_BRR_ADV;
    UI_setup_prov(role, brr);
    blebrr_scan_enable();
    //IV index check
    MS_ENABLE_SNB_FEATURE();

    if((ms_iv_index.iv_expire_time!=0)&&(ms_iv_index.iv_expire_time!=0xffffffff))
    {
        MS_net_start_iv_update_timer(ms_iv_index.iv_update_state,MS_TRUE);
    }

    //Key refresh
    MS_net_key_refresh_init();
    //HERATBEAT function check
    retval = MS_access_cm_get_prov_devices_list
             (
                 &prov_dev_list[0],
                 &num_entries,
                 &pointer_entries
             );

    if((retval == API_SUCCESS) && (num_entries != 0))
    {
        UI_SET_RAW_DATA_DST_ADDR(prov_dev_list[0].uaddr);
        #if (CFG_HEARTBEAT_MODE)
        retval = UI_trn_set_heartbeat_publication();
        #endif
        MS_net_start_snb_timer(0);
    }
    else
    {
        UI_subnet_valid = 1;
    }

    MS_access_ps_store_disable(MS_FALSE);
    MS_access_ps_store_all_record();
    return;
}

