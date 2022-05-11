#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"

UINT16 g_subscription_addr = 0xC005;
void UI_wireless_indication_click(void* args, UINT16 size);

void UI_wireless_indication_low_battery(void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    /* IN */ UCHAR                     data_param[4];
    /* IN */ UINT16                    data_len;
    //    API_RESULT               retval;
    TRACELOG_PRINT();
    MS_access_cm_get_primary_unicast_address(&saddr);
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    data_param[0]=++vendor_msg_tid;
    data_param[1] = 0x01;
    data_param[2] = 0x00;
    data_param[3] = 25;
    data_len = 4;
    cfg_retry_flag = 1;
    MS_access_send_pdu
    (
        saddr,
        aligenie_addr,
        subnet_handle,
        appkey_handle,
        ttl,
        opcode,
        data_param,
        data_len,
        MS_TRUE
    );
    EM_start_timer (&thandle, 10, UI_wireless_indication_click, NULL, 0);
}

void UI_wireless_indication_fault(void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    /* IN */ UCHAR                     data_param[4];
    /* IN */ UINT16                    data_len;
    //    API_RESULT               retval;
    TRACELOG_PRINT();
    MS_access_cm_get_primary_unicast_address(&saddr);
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    data_param[0]=++vendor_msg_tid;
    data_param[1] = 0x09;
    data_param[2] = 0xF0;
    data_param[3] = 128;
    data_len = 4;
    cfg_retry_flag = 1;
    MS_access_send_pdu
    (
        saddr,
        aligenie_addr,
        subnet_handle,
        appkey_handle,
        ttl,
        opcode,
        data_param,
        data_len,
        MS_TRUE
    );
    EM_start_timer (&thandle, 10, UI_wireless_indication_low_battery, NULL, 0);
}

void UI_wireless_indication_double_click(void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    /* IN */ UCHAR                     data_param[4];
    /* IN */ UINT16                    data_len;
    //    API_RESULT               retval;
    TRACELOG_PRINT();
    MS_access_cm_get_primary_unicast_address(&saddr);
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    data_param[0]=++vendor_msg_tid;
    data_param[1] = 0x06;
    data_param[2] = 0x00;
    data_len = 3;
    cfg_retry_flag = 1;
    MS_access_send_pdu
    (
        saddr,
        aligenie_addr,
        subnet_handle,
        appkey_handle,
        ttl,
        opcode,
        data_param,
        data_len,
        MS_TRUE
    );
    EM_start_timer (&thandle, 10, UI_wireless_indication_fault, NULL, 0);
}


void UI_wireless_indication_click(void* args, UINT16 size)
{
    thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    /* IN */ UCHAR                     data_param[4];
    /* IN */ UINT16                    data_len;
    //    API_RESULT               retval;
    TRACELOG_PRINT();
    MS_access_cm_get_primary_unicast_address(&saddr);
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    data_param[0]=++vendor_msg_tid;
    data_param[1] = 0x05;
    data_param[2] = 0x00;
    data_len = 3;
    cfg_retry_flag = 1;
    MS_access_send_pdu
    (
        saddr,
        aligenie_addr,
        subnet_handle,
        appkey_handle,
        ttl,
        opcode,
        data_param,
        data_len,
        MS_TRUE
    );
    EM_start_timer (&thandle, 10, UI_wireless_indication_double_click, NULL, 0);
}


//initial Implementation for wirelessbutton
API_RESULT init_attr_tbl(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 1;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("attr_tbl init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++],0x0100,    0,  BOOL,NULL);                 //powerstate    0 - ¹Ø±Õ 1 - ´ò¿ª
    return API_SUCCESS;
}


