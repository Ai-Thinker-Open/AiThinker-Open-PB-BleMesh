
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Vendor_fanlight.h"


/************  FANLIGHT productKey  ***********/
DECL_CONST UCHAR    fanlightproductKey[] = "a1uqUbLdeB5";

/************  FANLIGHT Properties  ***********/

API_RESULT fanlight_mainLight_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FANLIGHT_MAINLIGHT_OFF:
        break;

    case FANLIGHT_MAINLIGHT_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}

API_RESULT fanlight_backLight_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FANLIGHT_BACKLIGHT_OFF:
        break;

    case FANLIGHT_BACKLIGHT_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}


void fanlight_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32 data)
{
    UI_DATA_ALIGENIE_MODEL_T* me;
    me = find_model_private_data(recalled_model_handle);

    if(NULL == me)
        ERROR_PRINT("fanlight_recall_mode_set NULL == me,recalled_model_handle = 0x%04X\r\n",recalled_model_handle);

    light_mode_set(me,data);
}

void fanlight_recall_mode_set2(MS_ACCESS_MODEL_HANDLE recalled_model_handle, void* data2)
{
    UINT32 data;
    data = *(UINT32*)data2;
    fanlight_recall_mode_set(recalled_model_handle, data);
}

API_RESULT init_attr_tbl_fanlight(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 6;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("CURTAIN init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++], 0x0100, 0,                        BOOL,   light_powerstate_set);      //开关
    init_attr(&attr_tbl[i++], 0x0123, 0,                        PLD6,   light_color_set);           //颜色
    init_attr(&attr_tbl[i++], 0xF004, LIGHT_MODE_353,           U16,    light_mode_set);            //模式
    init_attr(&attr_tbl[i++], 0x0121, LIGHT_BRIGHTNESS_DEFAULT, U16,    light_brightness_set);      //亮度
    init_attr(&attr_tbl[i++], 0x0534, 0, BOOL, fanlight_mainLight_set);//主灯
    init_attr(&attr_tbl[i++], 0x0533, 0, BOOL, fanlight_backLight_set);//背光灯
    aliGenie_bleMesh_Generic_Scene_register_Recall_cb(me->model_handle,fanlight_recall_mode_set2);
    return API_SUCCESS;
}

