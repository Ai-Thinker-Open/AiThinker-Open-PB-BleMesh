#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"



/************  HEATER productKey  ***********/
DECL_CONST UCHAR    socketproductKey[] = "a1OgJ2jXDEy";

#define  SOCKET_POWERSTATE_OFF   0
#define  SOCKET_POWERSTATE_ON    1

#define  SOCKET_ALIPOWERSTATE_OFF    0
#define  SOCKET_ALIPOWERSTATE_ON     1

#define SOCKET_MODE_NUM 2
#define SOCKET_MODE_ON 0
#define SOCKET_MODE_OFF 1
DECL_CONST UINT16 socket_mode_list[SOCKET_MODE_NUM] =
{
    SOCKET_MODE_ON,
    SOCKET_MODE_OFF,

};


API_RESULT heater_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case SOCKET_POWERSTATE_OFF:
        break;

    case SOCKET_POWERSTATE_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}

API_RESULT heater_alipowerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case SOCKET_ALIPOWERSTATE_OFF:
        break;

    case SOCKET_ALIPOWERSTATE_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}


API_RESULT UI_HARD_generic_onoff_set (UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state)
{
    return heater_powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
}

API_RESULT init_attr_tbl_socket(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 5;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("SOCKET init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++],0x0100,    0,  BOOL,NULL);                 //powerstate    0 - ¹Ø±Õ 1 - ´ò¿ª
    return API_SUCCESS;
}


