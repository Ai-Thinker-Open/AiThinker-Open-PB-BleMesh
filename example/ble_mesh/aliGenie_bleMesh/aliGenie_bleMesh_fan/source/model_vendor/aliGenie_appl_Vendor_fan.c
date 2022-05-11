
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Vendor_fan.h"

DECL_CONST UINT16 fan_mode_list[FAN_MODE_NUM] =
{
    FAN_MODE_23,    //舒适风模式
    FAN_MODE_153,   //标准风模式
    FAN_MODE_165,   //智能风模式
    FAN_MODE_13,    //除湿模式
    FAN_MODE_164,   //暴风模式
    FAN_MODE_20,    //自然风模式
    FAN_MODE_17,    //静音模式
    FAN_MODE_39,    //普通模式
    FAN_MODE_19,    //正常风模式
    FAN_MODE_167,   //老人风模式
    FAN_MODE_230,   //节能模式
    FAN_MODE_166,   //循环风模式
    FAN_MODE_21,    //睡眠风模式
    FAN_MODE_22     //静音风模式
};

/************  FAN productKey  ***********/
DECL_CONST UCHAR    fanproductKey[] = "a1wHF3buTs2";
API_RESULT fan_childLockOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FAN_CHILDLOCK_OFF:
        break;

    case FAN_CHILDLOCK_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}
API_RESULT fan_heaterPower_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x000000FF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if( (val < FAN_HEATERPOWER_MIN)||(val > FAN_HEATERPOWER_MAX) )
    {
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT fan_ReserveLeftTime_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if( (val < FAN_RESERVELEFTTIME_MIN)||(val > FAN_RESERVELEFTTIME_MAX) )
    {
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT fan_direction_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FAN_DIRECTION_FORWORD:
        break;

    case FAN_DIRECTION_REVERSE:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}
API_RESULT fan_angleAutoLROnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FAN_ANGLEAUTOLR_OFF:
        break;

    case FAN_ANGLEAUTOLR_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}
API_RESULT fan_angleAutoAllOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FAN_ANGLEAUTOALL_OFF:
        break;

    case FAN_ANGLEAUTOALL_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}
API_RESULT fan_angleAutoUDOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FAN_ANGLEAUTOUD_OFF:
        break;

    case FAN_ANGLEAUTOUD_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}

API_RESULT fan_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
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
    return fan_powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
}

API_RESULT fan_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT32 val = (UINT32)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    //case FAN_MODE_xxx:
    //break;
    default:
        //ERROR_PRINT("INVALID val = 0x%02X\n",val);
        //return API_FAILURE;
        break;
    }

    return API_SUCCESS;
}
API_RESULT fan_windspeed_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT8)(data & 0x000000FF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case FAN_WINDSPEED_256: //最高档
        break;

    case FAN_WINDSPEED_255: //自动档
        break;

    case FAN_WINDSPEED_155: //增强档
        break;

    case FAN_WINDSPEED_156: //超强档
        break;

    case FAN_WINDSPEED_151: //低档
        break;

    default:
        if(val > FAN_WINDSPEED_256)
        {
            ERROR_PRINT("INVALID val = 0x%02X\n",val);
        }

        return API_FAILURE;
    }

    return API_SUCCESS;
}
API_RESULT fan_humidity_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if( (val < FAN_HUMIDITY_MIN)||(val > FAN_HUMIDITY_MAX) )
    {
        ERROR_PRINT("INVALID val = 0x%08X\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT fan_temperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if( (val < FAN_TEMPERATURE_MIN)||(val > FAN_TEMPERATURE_MAX) )
    {
        ERROR_PRINT("INVALID val = 0x%08X\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}

void fan_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32 data)
{
    UI_DATA_ALIGENIE_MODEL_T* me;
    me = find_model_private_data(recalled_model_handle);

    if(NULL == me)
        ERROR_PRINT("fan_recall_mode_set NULL == me,recalled_model_handle = 0x%04X\r\n",recalled_model_handle);

    fan_mode_set(me,data);
}

void fan_recall_mode_set2(MS_ACCESS_MODEL_HANDLE recalled_model_handle, void* data2)
{
    UINT32 data;
    data = *(UINT32*)data2;
    fan_recall_mode_set(recalled_model_handle, data);
}

API_RESULT init_attr_tbl_fan(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 12;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("CURTAIN init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++], 0x050C, 0, U8, fan_childLockOnOff_set);//童锁功能
    init_attr(&attr_tbl[i++], 0x0174, FAN_HEATERPOWER_DEFAULT, U8, fan_heaterPower_set);//加热档位
    init_attr(&attr_tbl[i++], 0x0550, FAN_RESERVELEFTTIME_DEFAULT, U16, fan_ReserveLeftTime_set);//预约剩余时间
    init_attr(&attr_tbl[i++], 0x0521, 0, U8, fan_direction_set);//方向
    init_attr(&attr_tbl[i++], 0x0500, 0, U8, fan_angleAutoLROnOff_set);//左右旋转/摇头/摆风
    init_attr(&attr_tbl[i++], 0x0626, 1, U8, fan_angleAutoAllOnOff_set);//三百六十度摇头
    init_attr(&attr_tbl[i++], 0x0501, 0, U8, fan_angleAutoUDOnOff_set);//上下旋转/摇头/摆风
    init_attr(&attr_tbl[i++], 0x0100, 0, U8, fan_powerstate_set);//开关
    init_attr(&attr_tbl[i++], 0xF004, FAN_MODE_153, U16, fan_mode_set);//模式
    init_attr(&attr_tbl[i++], 0x010A, FAN_WINDSPEED_151, U8, fan_windspeed_set);//风速
    init_attr(&attr_tbl[i++], 0x010F, FAN_HUMIDITY_DEFAULT, U16, fan_humidity_set);//湿度
    init_attr(&attr_tbl[i++], 0x010F, FAN_TEMPERATURE_DEFAULT, U16, fan_temperature_set);//温度
    aliGenie_bleMesh_Generic_Scene_register_Recall_cb(me->model_handle,fan_recall_mode_set2);
    return API_SUCCESS;
}
