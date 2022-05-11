#include "aliGenie_appl.h"
#include "aliGenie_appl_Prov.h"
uint8 UI_prov_state;
#define  NUM_ELEMENTS 0x03

/** Provisioning capabilities of local device */
DECL_STATIC PROV_CAPABILITIES_S UI_prov_capab =
{
    /** Number of Elements      */      NUM_ELEMENTS,

    /** Supported algorithms    */      PROV_MASK_ALGO_EC_FIPS_P256,

    /** Public key type         */      0,  //PROV_MASK_PUBKEY_OOBINFO,

    /** Static OOB type         */      1,  //PROV_MASK_STATIC_OOBINFO,

    /** Output OOB information  */      { 0x00, UI_PROV_OUTPUT_OOB_SIZE },

    /** Input OOB information   */      { 0x00, UI_PROV_INPUT_OOB_SIZE },
};


/** Unprovisioned device identifier */
PROV_DEVICE_S UI_lprov_device =
{
    /** UUID */
    {0xa8, 0x01, 0xb1, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0x02, 0x00, 0x00},

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

/** Provisioning Handle */
DECL_STATIC PROV_HANDLE UI_prov_handle;

/** Beacon interleave in mseconds */
#define UI_PROV_SETUP_GATT_MSECS                2000
#define UI_PROV_SETUP_ADV_MSECS                 2000

DECL_STATIC uint8 UI_prov_state=ALIG_UN_PROV;
//DECL_STATIC UCHAR event_type;
//DECL_STATIC UCHAR event_name[50] = {0};

void UI_prov_cb_PROVISIONING_SETUP(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    //LIGHT_ONLY_BLUE_ON;?'l??è?
    UI_prov_state=ALIG_IN_PROV;
}
void UI_prov_cb_OOB_DISPLAY(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    PROV_OOB_TYPE_S* oob_info;
    UINT32 authnum;
    UINT16 authsize;
    UCHAR* pauth;
    UCHAR authtype;
    UCHAR authstr[PROV_AUTHVAL_SIZE_PL << 1];
    API_RESULT retval;
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
}

void UI_prov_cb_OOB_ENTRY(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    PROV_OOB_TYPE_S* oob_info;
    /* Reference the Authenticatio Type information */
    oob_info = (PROV_OOB_TYPE_S*)event_data;
    CONSOLE_OUT("Authenticaion Action - 0x%02X\n", oob_info->action);
    CONSOLE_OUT("Authenticaion Size - 0x%02X\n", oob_info->size);
}
void UI_prov_cb_DEVINPUT_COMPLETE(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    return;
}
void UI_prov_cb_PROVDATA_INFO(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    PROV_DATA_S* rdata;
    /* Reference the Provisioning Data */
    rdata = (PROV_DATA_S*)event_data;
    CONSOLE_OUT("NetKey  : ");
    appl_dump_bytes(rdata->netkey, PROV_KEY_NETKEY_SIZE);
    CONSOLE_OUT("Key ID  : 0x%04X\n", rdata->keyid);
    CONSOLE_OUT("Flags	 : 0x%02X\n", rdata->flags);
    CONSOLE_OUT("IVIndex : 0x%08X\n", rdata->ivindex);
    CONSOLE_OUT("UAddr	 : 0x%04X\n", rdata->uaddr);
    /* Provide Provisioning Data to Access Layer */
    MS_access_cm_set_prov_data( rdata);
}
void UI_prov_cb_PROVISIONING_COMPLETE(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    if (API_SUCCESS == event_result)
    {
        /* Already Set while handling PROV_EVT_PROVDATA_INFO */
        MS_ENABLE_PROXY_FEATURE();
        /* LED ON/OFF for Provisioning Indication Abstraction Call */
        mesh_model_device_provisioned_ind_pl();
        //light_blink_set(LIGHT_BLUE, LIGHT_BLINK_FAST,3);l???????
        UI_prov_state=ALIG_CONFIG;
    }
    else
    {
        UI_prov_state=ALIG_UN_PROV;
        light_blink_set(LIGHT_RED, LIGHT_BLINK_SLOW,2);
        EM_start_timer (&thandle, 5, timeout_cb, NULL, 0);
    }
}




void UI_prov_cb_default(PROV_HANDLE* phandle,UCHAR event_type,API_RESULT event_result,void* event_data,UINT16 event_datalen)
{
    CONSOLE_OUT("Unknown Event - 0x%02X\n", event_type);
}


DECL_CONST prov_event_tbl_t prov_event_tbl[PROV_EVT_TBL_CNT]=
{
    {PROV_EVT_UNPROVISIONED_BEACON, "PROV_EVT_UNPROVISIONED_BEACON",    NULL},
    {PROV_EVT_PROVISIONING_SETUP,   "PROV_EVT_PROVISIONING_SETUP",      UI_prov_cb_PROVISIONING_SETUP},
    {PROV_EVT_OOB_DISPLAY,          "PROV_EVT_OOB_DISPLAY",             UI_prov_cb_OOB_DISPLAY},
    {PROV_EVT_OOB_ENTRY,            "PROV_EVT_OOB_ENTRY",               UI_prov_cb_OOB_ENTRY},
    {PROV_EVT_DEVINPUT_COMPLETE,    "PROV_EVT_DEVINPUT_COMPLETE",       UI_prov_cb_DEVINPUT_COMPLETE},
    {PROV_EVT_PROVDATA_INFO_REQ,    "PROV_EVT_PROVDATA_INFO_REQ",       NULL},
    {PROV_EVT_PROVDATA_INFO,        "PROV_EVT_PROVDATA_INFO",           UI_prov_cb_PROVDATA_INFO},
    {PROV_EVT_PROVISIONING_COMPLETE,"PROV_EVT_PROVISIONING_COMPLETE",   UI_prov_cb_PROVISIONING_COMPLETE},
    {PROV_HANDLE_INVALID,           "PROV_HANDLE_INVALID",              NULL}
};




static API_RESULT UI_prov_callback
(
    PROV_HANDLE* phandle,
    UCHAR         event_type,
    API_RESULT    event_result,
    void*         event_data,
    UINT16        event_datalen
)
{
//    API_RESULT retval;
    UCHAR i;

    for( i = 0; i < PROV_EVT_TBL_CNT; i++)
    {
        if(event_type == prov_event_tbl[i].event_type )
        {
            if(NULL != prov_event_tbl[i].func)
            {
                CONSOLE_OUT("Recvd event_type  %s\n",prov_event_tbl[i].event_name);
                CONSOLE_OUT("Status - 0x%04X\n", event_result);
                prov_event_tbl[i].func(phandle, event_type, event_result,event_data,event_datalen);
                break;
            }
        }
    }

    if(i>= PROV_EVT_TBL_CNT)
    {
        UI_prov_cb_default(phandle, event_type,event_result, event_data,event_datalen);
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

    if (PROV_BRR_GATT & brr)
    {
        blebrr_gatt_mode_set(BLEBRR_GATT_PROV_MODE);
    }

    if (PROV_ROLE_PROVISIONER != role)
    {
        CONSOLE_OUT("Setting up Device for Provisioning ...\n");
        UI_lprov_device.uuid[0]=0xa8;
        UI_lprov_device.uuid[1]=0x01;
        UI_lprov_device.uuid[2]=0xb1;
        UI_lprov_device.uuid[3]=ali_genie_pid[3];
        UI_lprov_device.uuid[4]=ali_genie_pid[2];
        UI_lprov_device.uuid[5]=ali_genie_pid[1];
        UI_lprov_device.uuid[6]=ali_genie_pid[0];
        memcpy(&UI_lprov_device.uuid[7],ali_genie_mac,6);
        prov_set_static_oob_auth_pl(ali_genie_auth, sizeof(ali_genie_auth));
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
                     0,
                     0
                 );
        UI_prov_role = PROV_ROLE_PROVISIONER;
    }

    if (PROV_BRR_GATT != brr)
    {
        //  UI_prov_brr_handle = PROV_BRR_ADV;
        UI_prov_bind(brr & PROV_BRR_ADV, 0x00);
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

