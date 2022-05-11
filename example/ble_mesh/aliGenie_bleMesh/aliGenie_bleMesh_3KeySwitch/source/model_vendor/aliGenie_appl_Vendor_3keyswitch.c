

#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"

UINT16 g_subscription_addr = 0xC001;

API_RESULT powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    TRACELOG_PRINT();
    MS_NET_ADDR saddr;
    UINT8 state = (UINT8)(data & 0x00000001);
    UINT8 io_num = me->generic_onoff_me->io_num;

    if (state)
    {
        light_ctrl(io_num, LIGHT_TOP_VALUE-1);
    }
    else
    {
        light_ctrl(io_num, 0);
    }

    MS_ACCESS_ELEMENT_HANDLE element_handle;
    find_element_by_model(me->model_handle,&element_handle);
    MS_access_cm_get_primary_unicast_address(&saddr);
    saddr = saddr + element_handle;
    vendor_Aligenie_indication(saddr,0x0100,state,attr_len_def[BOOL]);
    return API_SUCCESS;
}
API_RESULT UI_HARD_generic_onoff_set (UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state)
{
    return powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
}

API_RESULT init_attr_tbl(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 5;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("attr_tbl init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++],0x0100,    0,  BOOL,powerstate_set);                   //powerstate    0 - ¹Ø±Õ 1 - ´ò¿ª
    return API_SUCCESS;
}


