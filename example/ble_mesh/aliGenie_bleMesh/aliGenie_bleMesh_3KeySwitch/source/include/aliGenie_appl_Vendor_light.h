
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Generic.h"
#include "lib_light.h"
#ifndef _ALIGEN_APPL_VENDOR_LIGHT_H_
#define _ALIGEN_APPL_VENDOR_LIGHT_H_

/************  LIGHT Properties  ***********/

/******  ALIGENIE_ATTRIBUTE_CODE  *****/
#define  ALIGENIE_ATTRCODE_POWERSTATE
#define  ALIGENIE_ATTRCODE_COLOR
#define  ALIGENIE_ATTRCODE_MODE
#define  ALIGENIE_ATTRCODE_BRIGHTNESS

/******  ALIGENIE_PROPERTY  *****/
/******  Spec of powerstate( 开关 )  *****/
#define  LIGHT_POWERSTATE_OFF    0  //关闭
#define  LIGHT_POWERSTATE_ON     1  //打开

/******  Spec of color( 颜色 )  *****/
typedef enum
{
    //颜色
    LIGHT_COLOR_10824234    = 10824234, //棕色
    LIGHT_COLOR_16711680    = 16711680, //红色
    LIGHT_COLOR_4915330     = 4915330,  //靛青
    LIGHT_COLOR_16761035    = 16761035, //粉红
    LIGHT_COLOR_65280       = 65280,    //绿色
    LIGHT_COLOR_16753920    = 16753920, //橙色
    LIGHT_COLOR_16777215    = 16777215, //白色
    LIGHT_COLOR_255         = 255,  //蓝色
    LIGHT_COLOR_35723       = 35723,    //深青色
    LIGHT_COLOR_16776960    = 16776960, //黄色
    LIGHT_COLOR_8388736     = 8388736,  //紫色
    LIGHT_COLOR_16711935    = 16711935, //洋红
    LIGHT_COLOR_139         = 139,  //深蓝色
    LIGHT_COLOR_65535       = 65535,    //青色
    LIGHT_COLOR_7048739     = 7048739,  //浅褐色
    LIGHT_COLOR_0           = 0,    //黑色
    LIGHT_COLOR_15631086    = 15631086, //紫罗兰
    LIGHT_COLOR_8900331     = 8900331   //天蓝色
} LIGHT_COLOR_T;

/******  Spec of mode( 模式 )  *****/
typedef enum
{
    //模式
    LIGHT_MODE_652 = 652,   //炫彩模式
    LIGHT_MODE_15 = 15, //生活模式
    LIGHT_MODE_353 = 353,   //正常模式
    LIGHT_MODE_653 = 653,   //摇一摇模式
    LIGHT_MODE_6 = 6,   //夜灯模式
    LIGHT_MODE_3 = 3,   //阅读模式
    LIGHT_MODE_14 = 14, //睡眠模式
    LIGHT_MODE_5 = 5,   //温暖模式
    LIGHT_MODE_33 = 33, //音乐模式
    LIGHT_MODE_651 = 651,   //麦克风模式
    LIGHT_MODE_57 = 57, //护眼模式
    LIGHT_MODE_226 = 226,   //明亮模式
    LIGHT_MODE_378 = 378,   //休闲模式
    LIGHT_MODE_409 = 409    //定时休息模式
} LIGHT_MODE_T;


/******  Spec of brightness( 亮度 )  *****/
#define LIGHT_BRIGHTNESS_DEFAULT    100
#define LIGHT_BRIGHTNESS_UNIT   '%'
#define LIGHT_BRIGHTNESS_UNITNAME   "百分比"
#define LIGHT_BRIGHTNESS_MIN    1
#define LIGHT_BRIGHTNESS_MAX    100
#define LIGHT_BRIGHTNESS_STEP   25


#define LIGHT_MODE_NUM  16
extern DECL_CONST UINT16 light_mode_list[];

API_RESULT light_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT UI_HARD_generic_onoff_set (UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state);
API_RESULT light_color_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT light_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT light_brightness_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
void light_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32* data);
API_RESULT init_attr_tbl_light(UI_DATA_ALIGENIE_MODEL_T* me);

#endif //_ALIGEN_APPL_VENDOR_LIGHT_H_
