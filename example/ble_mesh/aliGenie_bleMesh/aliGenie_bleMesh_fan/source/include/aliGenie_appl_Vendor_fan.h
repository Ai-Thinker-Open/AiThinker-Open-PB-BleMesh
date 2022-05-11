
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#ifndef _ALIGENIE_APPL_VENDOR_FAN_H_
#define _ALIGENIE_APPL_VENDOR_FAN_H_

/************  FAN Properties  ***********/

/******  ALIGENIE_ATTRIBUTE_CODE  *****/
#define  ALIGENIE_ATTRCODE_CHILDLOCKONOFF
#define  ALIGENIE_ATTRCODE_HEATERPOWER
#define  ALIGENIE_ATTRCODE_RESERVELEFTTIME
#define  ALIGENIE_ATTRCODE_DIRECTION
#define  ALIGENIE_ATTRCODE_ANGLEAUTOLRONOFF
#define  ALIGENIE_ATTRCODE_ANGLEAUTOALLONOFF
#define  ALIGENIE_ATTRCODE_ANGLEAUTOUDONOFF
#define  ALIGENIE_ATTRCODE_POWERSTATE
#define  ALIGENIE_ATTRCODE_MODE
#define  ALIGENIE_ATTRCODE_WINDSPEED
#define  ALIGENIE_ATTRCODE_HUMIDITY
#define  ALIGENIE_ATTRCODE_TEMPERATURE

/******  ALIGENIE_PROPERTY  *****/
/******  Spec of childLockOnOff(0x050C 童锁功能 )  *****/
#define  FAN_CHILDLOCK_OFF   0  //关闭
#define  FAN_CHILDLOCK_ON    1  //开启

/******  Spec of heaterPower( 加热档位 )  *****/
#define FAN_HEATERPOWER_DEFAULT     1
#define FAN_HEATERPOWER_UNIT    "gear"
#define FAN_HEATERPOWER_UNITNAME    "档"
#define FAN_HEATERPOWER_MIN     1
#define FAN_HEATERPOWER_MAX     10
#define FAN_HEATERPOWER_STEP    1

/******  Spec of ReserveLeftTime( 预约剩余时间 )  *****/
#define FAN_RESERVELEFTTIME_DEFAULT     0
#define FAN_RESERVELEFTTIME_UNIT    "min"
#define FAN_RESERVELEFTTIME_UNITNAME    "分钟"
#define FAN_RESERVELEFTTIME_MIN     1
#define FAN_RESERVELEFTTIME_MAX     1440
#define FAN_RESERVELEFTTIME_STEP    1

/******  Spec of direction( 方向 )  *****/
#define  FAN_DIRECTION_FORWORD   0  //正转
#define  FAN_DIRECTION_REVERSE   1  //反转

/******  Spec of angleAutoLROnOff( 左右旋转/摇头/摆风 )  *****/
#define  FAN_ANGLEAUTOLR_OFF     0  //关闭
#define  FAN_ANGLEAUTOLR_ON      1  //开启

/******  Spec of angleAutoAllOnOff( 三百六十度摇头 )  *****/
#define  FAN_ANGLEAUTOALL_OFF    0  //关闭
#define  FAN_ANGLEAUTOALL_ON     1  //打开

/******  Spec of angleAutoUDOnOff( 上下旋转/摇头/摆风 )  *****/
#define  FAN_ANGLEAUTOUD_OFF     0  //关闭
#define  FAN_ANGLEAUTOUD_ON      1  //开启

/******  Spec of powerstate( 开关 )  *****/
#define  FAN_POWERSTATE_OFF      0  //关闭
#define  FAN_POWERSTATE_ON       1  //打开

/******  Spec of mode( 模式 )  *****/
typedef enum
{
    //模式
    FAN_MODE_23 = 23,   //舒适风模式
    FAN_MODE_153 = 153, //标准风模式
    FAN_MODE_165 = 165, //智能风模式
    FAN_MODE_13 = 13,   //除湿模式
    FAN_MODE_164 = 164, //暴风模式
    FAN_MODE_20 = 20,   //自然风模式
    FAN_MODE_17 = 17,   //静音模式
    FAN_MODE_39 = 39,   //普通模式
    FAN_MODE_19 = 19,   //正常风模式
    FAN_MODE_167 = 167, //老人风模式
    FAN_MODE_230 = 230, //节能模式
    FAN_MODE_166 = 166, //循环风模式
    FAN_MODE_21 = 21,   //睡眠风模式
    FAN_MODE_22 = 22    //静音风模式
} FAN_MODE_T;

//模式
#define FAN_MODE_NUM    14
extern DECL_CONST UINT16 fan_mode_list[];


/******  Spec of windspeed( 风速 )  *****/
typedef enum
{
    //风速
    FAN_WINDSPEED_44 = 44,  //四十四档
    FAN_WINDSPEED_6 = 6,    //六档
    FAN_WINDSPEED_51 = 51,  //五十一档
    FAN_WINDSPEED_7 = 7,    //七档
    FAN_WINDSPEED_68 = 68,  //六十八档
    FAN_WINDSPEED_16 = 16,  //十六档
    FAN_WINDSPEED_38 = 38,  //三十八档
    FAN_WINDSPEED_34 = 34,  //三十四档
    FAN_WINDSPEED_99 = 99,  //九十九档
    FAN_WINDSPEED_3 = 3,    //三档
    FAN_WINDSPEED_256 = 256,    //最高档
    FAN_WINDSPEED_8 = 8,    //八档
    FAN_WINDSPEED_95 = 95,  //九十五档
    FAN_WINDSPEED_32 = 32,  //三十二档
    FAN_WINDSPEED_18 = 18,  //十八档
    FAN_WINDSPEED_255 = 255,    //自动档
    FAN_WINDSPEED_21 = 21,  //二十一档
    FAN_WINDSPEED_97 = 97,  //九十七档
    FAN_WINDSPEED_12 = 12,  //十二档
    FAN_WINDSPEED_81 = 81,  //八十一档
    FAN_WINDSPEED_71 = 71,  //七十一档
    FAN_WINDSPEED_84 = 84,  //八十四档
    FAN_WINDSPEED_90 = 90,  //九十档
    FAN_WINDSPEED_74 = 74,  //七十四档
    FAN_WINDSPEED_64 = 64,  //六十四档
    FAN_WINDSPEED_67 = 67,  //六十七档
    FAN_WINDSPEED_36 = 36,  //三十六档
    FAN_WINDSPEED_50 = 50,  //五十档
    FAN_WINDSPEED_31 = 31,  //三十一档
    FAN_WINDSPEED_23 = 23,  //二十三档
    FAN_WINDSPEED_98 = 98,  //九十八档
    FAN_WINDSPEED_53 = 53,  //五十三档
    FAN_WINDSPEED_29 = 29,  //二十九档
    FAN_WINDSPEED_96 = 96,  //九十六档
    FAN_WINDSPEED_24 = 24,  //二十四档
    FAN_WINDSPEED_1 = 1,    //一档
    FAN_WINDSPEED_82 = 82,  //八十二档
    FAN_WINDSPEED_69 = 69,  //六十九档
    FAN_WINDSPEED_27 = 27,  //二十七档
    FAN_WINDSPEED_65 = 65,  //六十五档
    FAN_WINDSPEED_78 = 78,  //七十八档
    FAN_WINDSPEED_153 = 153,    //中档
    FAN_WINDSPEED_25 = 25,  //二十五档
    FAN_WINDSPEED_13 = 13,  //十三档
    FAN_WINDSPEED_35 = 35,  //三十五档
    FAN_WINDSPEED_46 = 46,  //四十六档
    FAN_WINDSPEED_61 = 61,  //六十一档
    FAN_WINDSPEED_52 = 52,  //五十二档
    FAN_WINDSPEED_88 = 88,  //八十八档
    FAN_WINDSPEED_5 = 5,    //五档
    FAN_WINDSPEED_48 = 48,  //四十八档
    FAN_WINDSPEED_94 = 94,  //九十四档
    FAN_WINDSPEED_56 = 56,  //五十六档
    FAN_WINDSPEED_41 = 41,  //四十一档
    FAN_WINDSPEED_14 = 14,  //十四档
    FAN_WINDSPEED_2 = 2,    //二档
    FAN_WINDSPEED_15 = 15,  //十五档
    FAN_WINDSPEED_10 = 10,  //十档
    FAN_WINDSPEED_49 = 49,  //四十九档
    FAN_WINDSPEED_91 = 91,  //九十一档
    FAN_WINDSPEED_4 = 4,    //四档
    FAN_WINDSPEED_45 = 45,  //四十五档
    FAN_WINDSPEED_62 = 62,  //六十二档
    FAN_WINDSPEED_33 = 33,  //三十三档
    FAN_WINDSPEED_17 = 17,  //十七档
    FAN_WINDSPEED_39 = 39,  //三十九档
    FAN_WINDSPEED_89 = 89,  //八十九档
    FAN_WINDSPEED_79 = 79,  //七十九档
    FAN_WINDSPEED_156 = 156,    //超强档
    FAN_WINDSPEED_75 = 75,  //七十五档
    FAN_WINDSPEED_83 = 83,  //八十三档
    FAN_WINDSPEED_100 = 100,    //一百档
    FAN_WINDSPEED_63 = 63,  //六十三档
    FAN_WINDSPEED_77 = 77,  //七十七档
    FAN_WINDSPEED_92 = 92,  //九十二档
    FAN_WINDSPEED_57 = 57,  //五十七档
    FAN_WINDSPEED_42 = 42,  //四十二档
    FAN_WINDSPEED_66 = 66,  //六十六档
    FAN_WINDSPEED_86 = 86,  //八十六档
    FAN_WINDSPEED_11 = 11,  //十一档
    FAN_WINDSPEED_40 = 40,  //四十档
    FAN_WINDSPEED_43 = 43,  //四十三档
    FAN_WINDSPEED_20 = 20,  //二十档
    FAN_WINDSPEED_55 = 55,  //五十五档
    FAN_WINDSPEED_19 = 19,  //十九档
    FAN_WINDSPEED_73 = 73,  //七十三档
    FAN_WINDSPEED_47 = 47,  //四十七档
    FAN_WINDSPEED_28 = 28,  //二十八档
    FAN_WINDSPEED_59 = 59,  //五十九档
    FAN_WINDSPEED_93 = 93,  //九十三档
    FAN_WINDSPEED_80 = 80,  //八十档
    FAN_WINDSPEED_54 = 54,  //五十四档
    FAN_WINDSPEED_26 = 26,  //二十六档
    FAN_WINDSPEED_151 = 151,    //低档
    FAN_WINDSPEED_58 = 58,  //五十八档
    FAN_WINDSPEED_60 = 60,  //六十档
    FAN_WINDSPEED_85 = 85,  //八十五档
    FAN_WINDSPEED_9 = 9,    //九档
    FAN_WINDSPEED_155 = 155,    //高档
    FAN_WINDSPEED_72 = 72,  //七十二档
    FAN_WINDSPEED_76 = 76,  //七十六档
    FAN_WINDSPEED_30 = 30,  //三十档
    FAN_WINDSPEED_70 = 70,  //七十档
    FAN_WINDSPEED_22 = 22,  //二十二档
    FAN_WINDSPEED_87 = 87   //八十七档
} FAN_WINDSPEED_T;

/******  Spec of humidity( 湿度 )  *****/
#define FAN_HUMIDITY_DEFAULT    100.0
#define FAN_HUMIDITY_MIN    0.0
#define FAN_HUMIDITY_MAX    100.0
#define FAN_HUMIDITY_STEP   10.0

/******  Spec of temperature( 温度 )  *****/
#define FAN_TEMPERATURE_DEFAULT     0.0
#define FAN_TEMPERATURE_MIN     -38.0
#define FAN_TEMPERATURE_MAX     655.35
#define FAN_TEMPERATURE_STEP    1.0

API_RESULT fan_childLockOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_heaterPower_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_ReserveLeftTime_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_direction_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_angleAutoLROnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_angleAutoAllOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_angleAutoUDOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_windspeed_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_humidity_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fan_temperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT init_attr_tbl_fan(UI_DATA_ALIGENIE_MODEL_T* me);
#endif //_ALIGENIE_APPL_VENDOR_FAN_H_
