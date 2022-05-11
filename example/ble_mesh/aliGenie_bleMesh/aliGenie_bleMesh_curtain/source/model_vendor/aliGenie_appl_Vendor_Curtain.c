

#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"




//Curtain Control 0x0547
#define CURTAIN_CONTROL_CLOSE   0
#define CURTAIN_CONTROL_OPEN    1
#define CURTAIN_CONTROL_STOP    2
API_RESULT curtain_control_set(UI_DATA_ALIGENIE_MODEL_T* me,UINT32 data)
{
    UINT8 val = (UINT8)data;
    DEBUG_PRINT("[VAL] = 0x%08x, io_num = %d\n",val,me->io_num);

    switch(val)
    {
    case CURTAIN_CONTROL_CLOSE:
        light_ctrl(me->io_num, 0);
        break;

    case CURTAIN_CONTROL_OPEN:
        light_ctrl(me->io_num, LIGHT_TOP_VALUE - 1);
        break;

    case CURTAIN_CONTROL_STOP:
        light_ctrl(me->io_num, LIGHT_TOP_VALUE/2 - 1);
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}

//Curtain Position 0x0548
API_RESULT curtain_position_set(UI_DATA_ALIGENIE_MODEL_T* me,UINT32 data)
{
    UINT8 val = (UINT8)data;
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if(val > 100)
    {
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    //TODO: calculate position by step
    light_ctrl(me->io_num, val * LIGHT_TOP_VALUE/100 -1 );
    return API_SUCCESS;
}

//Curtain Workstatus
#define CURTAIN_WORKSTATUS_STOPPED      0 // ֹͣ
#define CURTAIN_WORKSTATUS_GO           1 // ��ʼ(����)
#define CURTAIN_WORKSTATUS_PAUSE        2 // ��ͣ
#define CURTAIN_WORKSTATUS_IDLE         3 // ����
#define CURTAIN_WORKSTATUS_BOOK         5 // ԤԼ
#define CURTAIN_WORKSTATUS_ERROR        26 // ����
#define CURTAIN_WORKSTATUS_WIFISETTING  27 // WiFi����
#define CURTAIN_WORKSTATUS_POWEROFF     28 // �ػ�
#define CURTAIN_WORKSTATUS_WORKING      29 // ����
#define CURTAIN_WORKSTATUS_POWERON      36 // �ϵ�
#define CURTAIN_WORKSTATUS_RUNNING      37 // ����
#define CURTAIN_WORKSTATUS_NORMAL       52 // ����
#define CURTAIN_WORKSTATUS_CHARGING     62 // ���
#define CURTAIN_WORKSTATUS_CHARGEOK     63 // ������
#define CURTAIN_WORKSTATUS_LOWBATTERY   64 // ��������

//Curtain Mode 0xF004
#define CURTAIN_MODE_WAKEUP         8       //��ģʽ
#define CURTAIN_MODE_SLEEP          14      //˯��ģʽ
#define CURTAIN_MODE_REVERSE        351     //��תģʽ
#define CURTAIN_MODE_CALIBRATION    352     //У׼ģʽ
#define CURTAIN_MODE_NORMAL         353     //����ģʽ
#define CURTAIN_MODE_VISITOR        395     //���ģʽ
#define CURTAIN_MODE_WEEKLIGHT      396     //΢��ģʽ

#define CURTAIN_MODE_NUM    7
DECL_CONST UINT16 curtain_mode_list[CURTAIN_MODE_NUM] =
{
    CURTAIN_MODE_WAKEUP,            //��ģʽ
    CURTAIN_MODE_SLEEP,             //˯��ģʽ
    CURTAIN_MODE_REVERSE,           //��תģʽ
    CURTAIN_MODE_CALIBRATION,       //У׼ģʽ
    CURTAIN_MODE_NORMAL,            //����ģʽ
    CURTAIN_MODE_VISITOR,           //���ģʽ
    CURTAIN_MODE_WEEKLIGHT          //΢��ģʽ
};

API_RESULT UI_HARD_generic_onoff_set (UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state)
{
    return API_SUCCESS;
}

API_RESULT curtain_mode_set(UI_DATA_ALIGENIE_MODEL_T* me,UINT32 data)
{
    UINT16 val = (UINT16)data;
    DEBUG_PRINT("[VAL] = 0x%04x\n",val);

    switch(val)
    {
    case CURTAIN_MODE_WAKEUP:
        light_ctrl(me->io_num, 3*LIGHT_TOP_VALUE/4);
        break;

    case CURTAIN_MODE_SLEEP:
        light_ctrl(me->io_num, 0);
        break;

    case CURTAIN_MODE_NORMAL:
        light_ctrl(me->io_num, LIGHT_TOP_VALUE/2 - 1);
        break;

    default:
        ERROR_PRINT("INVALID val = 0x%04X\n",val);
        return API_FAILURE;
    }

    return API_SUCCESS;
}

void curtain_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32 data)
{
    UI_DATA_ALIGENIE_MODEL_T* me;
    me = find_model_private_data(recalled_model_handle);

    if(NULL == me)
        ERROR_PRINT("curtain_recall_mode_set NULL == me,recalled_model_handle = 0x%04X\r\n",recalled_model_handle);

    curtain_mode_set(me, data);
}

void curtain_recall_mode_set2(MS_ACCESS_MODEL_HANDLE recalled_model_handle, void* data2)
{
    UINT32 data;
    data = *(UINT32*)data2;
    curtain_recall_mode_set(recalled_model_handle, data);
}
//static void UI_generic_scene_model_states_initialization(MS_ACCESS_MODEL_HANDLE model_handle)
//{

//    for(int i = 0; i < CURTAIN_MODE_NUM; i++ )
//    {
//      ms_scene_store(&model_handle,curtain_mode_list[i]);
//    }

//}

//initial Implementation for CURTAIN
API_RESULT init_attr_tbl_curtain(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 5;
    attr_tbl = EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("CURTAIN init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++],0x0100,    0,  BOOL,NULL);                 //powerstate    0 - �ر� 1 - ��
    init_attr(&attr_tbl[i++],0xF004,    353,U16,curtain_mode_set);      //mode
    //  8 - ��ģʽ
    //  14 - ˯��ģʽ
    //  351 - ��תģʽ
    //  352 - У׼ģʽ
    //  353 - ����ģʽ
    //  395 - ���ģʽ
    //  396 - ΢��ģʽ
    init_attr(&attr_tbl[i++],0xF001,    0,  U8,NULL);                   //workstatus
    //  0 - ֹͣ
    //  1 - ��ʼ(����)
    //  2 - ��ͣ
    //  3 - ����
    //  5 - ԤԼ
    //  26 - ����
    //  27 - WiFi����
    //  28 - �ػ�
    //  29 - ����
    //  36 - �ϵ�
    //  37 - ����
    //  52 - ����
    //  62 - ���
    //  63 - ������
    //  64 - ��������
    init_attr(&attr_tbl[i++],0x0547,    2,  U8,curtain_control_set);    //curtainConrtol    //0 - �رմ��� 1 - �򿪴��� 2 - ֹͣ����
    init_attr(&attr_tbl[i++],0x0548,    0,  U8,curtain_position_set);   //curtainPosition   //0~100, devided by step(default =1)
    aliGenie_bleMesh_Generic_Scene_register_Recall_cb(me->model_handle,curtain_recall_mode_set2);
    return API_SUCCESS;
}




