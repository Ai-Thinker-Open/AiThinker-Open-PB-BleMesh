
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Vendor_fogger.h"


/************  fogger productKey  ***********/
DECL_CONST UCHAR    foggerproductKey[] = "a1peegbeV6r";

DECL_CONST UINT16 fogger_mode_list[FOGGER_MODE_NUM] =
{
    2,  // - 自动模式
    13, // - 除湿模式
    14, // - 睡眠模式
    32, // - 智能模式
    39, // - 普通模式
    48, // - 标准模式
    201,// - 加湿模式
    332 // - 待机模式
};


API_RESULT fogger_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
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
    return fogger_powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
}


API_RESULT fogger_targethumidity_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    double val = (UINT32)data;
    DEBUG_PRINT("[VAL] = %f\n",val);

    if( (val < -30)||(val > 45) )
    {
        ERROR_PRINT("INVALID val = %f\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}

API_RESULT fogger_temperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    double val = (UINT32)data;
    DEBUG_PRINT("[VAL] = %f\n",val);

    if( (val < FOGGER_TEMPERATURE_MIN)||(val > FOGGER_TEMPERATURE_MAX) )
    {
        ERROR_PRINT("INVALID val = %f\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}

API_RESULT fogger_fog_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT8)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);
    //TODO: do the action
    return API_SUCCESS;
}

API_RESULT fogger_fogLevel_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x000000FF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if( (val < FOGGER_LEVEL_MIN)||(val > FOGGER_LEVEL_MAX) )
    {
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT fogger_waterShortage_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);
    return API_SUCCESS;
}

API_RESULT fogger_filterExpired_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    MS_NET_ADDR saddr;
    DEBUG_PRINT("[fogger_filterExpired]\n");
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    find_element_by_model(me->model_handle,&element_handle);
    MS_access_cm_get_primary_unicast_address(&saddr);
    saddr = saddr + element_handle;
    vendor_Aligenie_indication(saddr,ALIGENIE_ATTRCODE_FILTEREXPIRED,0,attr_len_def[0]);
    return API_SUCCESS;
}

API_RESULT fogger_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);
    return API_SUCCESS;
}

void fogger_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32 data)
{
    UI_DATA_ALIGENIE_MODEL_T* me;
    me = find_model_private_data(recalled_model_handle);

    if(NULL == me)
        ERROR_PRINT("fan_recall_mode_set NULL == me,recalled_model_handle = 0x%04X\r\n",recalled_model_handle);

    fogger_mode_set(me,data);
}

void fogger_recall_mode_set2(MS_ACCESS_MODEL_HANDLE recalled_model_handle, void* data2)
{
    UINT32 data;
    data = *(UINT32*)data2;
    fogger_recall_mode_set(recalled_model_handle, data);
}

API_RESULT init_attr_tbl_fogger(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 10;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("CURTAIN init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_TARGETHUMIDITY,     FOGGER_HUMIDITY_DEFAULT,    U16,    fogger_targethumidity_set); //湿度
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_TEMPERATURE,        FOGGER_TEMPERATURE_DEFAULT, U32,    fogger_temperature_set);    //温度
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_POWERSTATE,         0,                          BOOL,   fogger_powerstate_set);     //开关
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_FOG,                0,                          U16,    fogger_fog_set);            //档位
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_FOGLEVEL,           1,                          U8,     fogger_fogLevel_set);       //档位
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_WATERSHORTAGE,      0,                          BOOL,   fogger_waterShortage_set);  //缺水
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_MODE,               2,                          U8,     fogger_mode_set);           //模式
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_FILTEREXPIRED,      0,                          0,      fogger_filterExpired_set);  //滤芯过期
    aliGenie_bleMesh_Generic_Scene_register_Recall_cb(me->model_handle,fogger_recall_mode_set2);
    return API_SUCCESS;
}

