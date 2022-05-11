
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Generic.h"
#include "aliGenie_appl_Vendor_light.h"
#include "lib_light.h"


/************  LIGHT productKey  ***********/
DECL_CONST UCHAR    lightproductKey[] = "a1cqfD6Vxkc";

/************  LIGHT Properties  ***********/

/******  Spec of mode( 模式 )  *****/
DECL_CONST UINT16 light_mode_list[LIGHT_MODE_NUM] =
{
    LIGHT_MODE_652, //炫彩模式
    LIGHT_MODE_15,  //生活模式
    SCENE_CINEMATIC_MODE, //
    SCENE_GETUP_MODE,
    LIGHT_MODE_353, //正常模式
    LIGHT_MODE_653, //摇一摇模式
    LIGHT_MODE_6,   //夜灯模式
    LIGHT_MODE_3,   //阅读模式
    LIGHT_MODE_14,  //睡眠模式
    LIGHT_MODE_5,   //温暖模式
    LIGHT_MODE_33,  //音乐模式
    LIGHT_MODE_651, //麦克风模式
    LIGHT_MODE_57,  //护眼模式
    LIGHT_MODE_226, //明亮模式
    LIGHT_MODE_378, //休闲模式
    LIGHT_MODE_409  //定时休息模式

};

API_RESULT light_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
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
    return light_powerstate_set(me->vendor_me,((UINT32)state)&& 0x00000001);
}


#define light_color_scale(x) ((x)>1?((x)-1):0)

API_RESULT light_color_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    COLOR_RGB_T color_rgb;
    calc_color_rgb(data, &color_rgb);
    EM_free_mem((void*)data);
    DEBUG_PRINT("[COLOR] R=0x%04X, G=0x%04X, B=0x%04X\n",color_rgb.r,color_rgb.g,color_rgb.b);
    light_ctrl(LIGHT_RED,light_color_scale(color_rgb.r));
    light_ctrl(LIGHT_GREEN,light_color_scale(color_rgb.g));
    light_ctrl(LIGHT_BLUE,light_color_scale(color_rgb.b));
    return API_SUCCESS;
}
API_RESULT light_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT16 val = (UINT16)(data & 0x0000FFFF);

    if(val >= LIGHT_MODE_NUM)
    {
        ERROR_PRINT("[MODEL_HANDLE] = 0x%04X INVALID val = 0x%04X\n",me->model_handle,val);
        return API_FAILURE;
    }

    DEBUG_PRINT("[MODEL_HANDLE] = 0x%04X SCENE MODE IS SET TO  0x%04x\n",me->model_handle,light_mode_list[val]);
    return API_SUCCESS;
}
API_RESULT light_brightness_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data)
{
    UINT8 val = (UINT8)(data & 0x000000FF);
    DEBUG_PRINT("[VAL] = 0x%08x\n",val);

    if( (val < LIGHT_BRIGHTNESS_MIN)||(val > LIGHT_BRIGHTNESS_MAX) )
    {
        ERROR_PRINT("INVALID val = 0x%02X\n",val);
        return API_FAILURE;
    }

    //TODO: do the action
    return API_SUCCESS;
}

void light_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32* data)
{
    UI_DATA_ALIGENIE_MODEL_T* me;
    me = find_model_private_data(recalled_model_handle);

    if(NULL == me)
        ERROR_PRINT("light_recall_mode_set NULL == me,recalled_model_handle = 0x%04X\r\n",recalled_model_handle);

    light_mode_set(me,*data);
}

API_RESULT init_attr_tbl_light(UI_DATA_ALIGENIE_MODEL_T* me)
{
    int i = 0;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    me->attr_tbl_num = 4;
    attr_tbl = (UI_ALIGENIE_ELEMENT_ATTR_T*)EM_alloc_mem(sizeof(UI_ALIGENIE_ELEMENT_ATTR_T) * me->attr_tbl_num);

    if(NULL == attr_tbl)
    {
        ERROR_PRINT("LIGHT init failed, EM_alloc_mem == NULL!\n");
        return API_FAILURE;
    }

    me->attr_tbl = attr_tbl;
    init_attr(&attr_tbl[i++], 0x0100, 0,                        BOOL,   light_powerstate_set);      //开关
    init_attr(&attr_tbl[i++], 0x0123, 0,                        PLD6,   light_color_set);           //颜色
    init_attr(&attr_tbl[i++], 0xF004, LIGHT_MODE_353,           U16,    light_mode_set);            //模式
    init_attr(&attr_tbl[i++], 0x0121, LIGHT_BRIGHTNESS_DEFAULT, U16,    light_brightness_set);      //亮度
    print_attr_tbl(attr_tbl,4);
    aliGenie_bleMesh_Generic_Scene_register_Recall_cb(me->model_handle,light_recall_mode_set);
    return API_SUCCESS;
}

