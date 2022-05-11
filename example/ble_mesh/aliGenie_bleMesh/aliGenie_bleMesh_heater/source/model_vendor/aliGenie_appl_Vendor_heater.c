
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Vendor_heater.h"


/************  HEATER productKey  ***********/
DECL_CONST UCHAR    heaterproductKey[] = "a1peegbeV6r";


API_RESULT heater_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
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
    return heater_powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
}


API_RESULT heater_humidity_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    double val = (UINT32)data;
    DEBUG_PRINT("[VAL] = %f\n",val);

    if( (val < HEATER_HUMIDITY_MIN)||(val > HEATER_HUMIDITY_MAX) )
    {
        ERROR_PRINT("INVALID val = %f\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT heater_errorCode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    default:
        ERROR_PRINT("INVALID val = 0x%04X\n",val);
        return API_FAILURE;
    }
}
API_RESULT heater_temperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    double val = (UINT32)data;
    DEBUG_PRINT("[VAL] = %f\n",val);

    if( (val < HEATER_TEMPERATURE_MIN)||(val > HEATER_TEMPERATURE_MAX) )
    {
        ERROR_PRINT("INVALID val = %f\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}

API_RESULT heater_heaterPower_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    int32 val = (UINT8)(data & 0x000000FF);
    DEBUG_PRINT("[VAL] = %ld\n",val);

    if( (val < HEATER_HEATERPOWER_MIN)||(val > HEATER_HEATERPOWER_MAX) )
    {
        ERROR_PRINT("INVALID val = %ld\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT heater_rateofwork_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    double val = (UINT32)data;
    DEBUG_PRINT("[VAL] = %f\n",val);

    if( (val < HEATER_RATEOFWORK_MIN)||(val > HEATER_RATEOFWORK_MAX) )
    {
        ERROR_PRINT("INVALID val = %f\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT heater_heatStatus_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    int32 val = (UINT8)(data & 0x000000FF);
    DEBUG_PRINT("[VAL] = %ld\n",val);

    if( (val < HEATER_HEATSTATUS_MIN)||(val > HEATER_HEATSTATUS_MAX) )
    {
        ERROR_PRINT("INVALID val = %ld\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT heater_childLockOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case HEATER_CHILDLOCKONOFF_OFF:
        break;

    case HEATER_CHILDLOCKONOFF_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}
API_RESULT heater_targetTemperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    double val = (UINT32)data;
    DEBUG_PRINT("[VAL] = %f\n",val);

    if( (val < HEATER_TARGETTEMPERATURE_MIN)||(val > HEATER_TARGETTEMPERATURE_MAX) )
    {
        ERROR_PRINT("INVALID val = %f\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}
API_RESULT heater_humidificationSwitch_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x00000001);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    switch(val)
    {
    case HEATER_HUMIDIFICATIONSWITCH_OFF:
        break;

    case HEATER_HUMIDIFICATIONSWITCH_ON:
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}

API_RESULT init_attr_tbl_heater(UI_DATA_ALIGENIE_MODEL_T* me)
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
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_HUMIDITY,           HEATER_HUMIDITY_DEFAULT,    U32, heater_humidity_set);//湿度
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_ERRORCODE,          0,                          U8, heater_errorCode_set);//错误码
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_TEMPERATURE,        HEATER_TEMPERATURE_DEFAULT, U32, heater_temperature_set);//温度
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_POWERSTATE,         0,                          BOOL, heater_powerstate_set);//开关
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_HEATERPOWER,        HEATER_HEATERPOWER_DEFAULT, U8, heater_heaterPower_set);//加热档位
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_RATEOFWORK,         HEATER_RATEOFWORK_DEFAULT,  U32, heater_rateofwork_set);//功率
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_HEATSTATUS,         HEATER_HEATSTATUS_DEFAULT,  U8, heater_heatStatus_set);//加热状态
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_CHILDLOCKONOFF,     0,                          BOOL, heater_childLockOnOff_set);//童锁功能
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_TARGETTEMPERATURE,  HEATER_TARGETTEMPERATURE_DEFAULT, U32, heater_targetTemperature_set);//目标温度
    init_attr(&attr_tbl[i++], ALIGENIE_ATTRCODE_HUMIDIFICATIONSWITCH, 0,                        BOOL, heater_humidificationSwitch_set);//加湿开关
    return API_SUCCESS;
}

