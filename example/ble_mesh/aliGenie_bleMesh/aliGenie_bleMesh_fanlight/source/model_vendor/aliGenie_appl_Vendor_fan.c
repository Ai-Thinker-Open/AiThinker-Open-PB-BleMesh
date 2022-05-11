
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Vendor_fan.h"

DECL_CONST UINT16 fan_mode_list[FAN_MODE_NUM] =
{
    FAN_MODE_23,    //���ʷ�ģʽ
    FAN_MODE_153,   //��׼��ģʽ
    FAN_MODE_165,   //���ܷ�ģʽ
    FAN_MODE_13,    //��ʪģʽ
    FAN_MODE_164,   //����ģʽ
    FAN_MODE_20,    //��Ȼ��ģʽ
    FAN_MODE_17,    //����ģʽ
    FAN_MODE_39,    //��ͨģʽ
    FAN_MODE_19,    //������ģʽ
    FAN_MODE_167,   //���˷�ģʽ
    FAN_MODE_230,   //����ģʽ
    FAN_MODE_166,   //ѭ����ģʽ
    FAN_MODE_21,    //˯�߷�ģʽ
    FAN_MODE_22     //������ģʽ
};

/************  FAN productKey  ***********/

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
//static API_RESULT UI_HARD_generic_onoff_set (UI_DATA_GENERIC_ONOFF_MODEL_T * me, UINT8 state)
//{
//  return fan_powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
//}

API_RESULT fan_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);
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
    case FAN_WINDSPEED_256: //��ߵ�
        break;

    case FAN_WINDSPEED_255: //�Զ���
        break;

    case FAN_WINDSPEED_155: //��ǿ��
        break;

    case FAN_WINDSPEED_156: //��ǿ��
        break;

    case FAN_WINDSPEED_151: //�͵�
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
    init_attr(&attr_tbl[i++], 0x050C, 0, U8, fan_childLockOnOff_set);//ͯ������
    init_attr(&attr_tbl[i++], 0x0174, FAN_HEATERPOWER_DEFAULT, U8, fan_heaterPower_set);//���ȵ�λ
    init_attr(&attr_tbl[i++], 0x0550, FAN_RESERVELEFTTIME_DEFAULT, U16, fan_ReserveLeftTime_set);//ԤԼʣ��ʱ��
    init_attr(&attr_tbl[i++], 0x0521, 0, U8, fan_direction_set);//����
    init_attr(&attr_tbl[i++], 0x0500, 0, U8, fan_angleAutoLROnOff_set);//������ת/ҡͷ/�ڷ�
    init_attr(&attr_tbl[i++], 0x0626, 1, U8, fan_angleAutoAllOnOff_set);//������ʮ��ҡͷ
    init_attr(&attr_tbl[i++], 0x0501, 0, U8, fan_angleAutoUDOnOff_set);//������ת/ҡͷ/�ڷ�
    init_attr(&attr_tbl[i++], 0x0100, 0, U8, fan_powerstate_set);//����
    init_attr(&attr_tbl[i++], 0xF004, FAN_MODE_153, U16, fan_mode_set);//ģʽ
    init_attr(&attr_tbl[i++], 0x010A, FAN_WINDSPEED_151, U8, fan_windspeed_set);//����
    init_attr(&attr_tbl[i++], 0x010F, FAN_HUMIDITY_DEFAULT, U16, fan_humidity_set);//ʪ��
    init_attr(&attr_tbl[i++], 0x010F, FAN_TEMPERATURE_DEFAULT, U16, fan_temperature_set);//�¶�
    aliGenie_bleMesh_Generic_Scene_register_Recall_cb(me->model_handle,fan_recall_mode_set2);
    return API_SUCCESS;
}
