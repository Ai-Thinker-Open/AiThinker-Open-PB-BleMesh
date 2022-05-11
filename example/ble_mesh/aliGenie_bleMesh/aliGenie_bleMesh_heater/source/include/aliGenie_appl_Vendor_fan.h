
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
/******  Spec of childLockOnOff(0x050C ͯ������ )  *****/
#define  FAN_CHILDLOCK_OFF   0  //�ر�
#define  FAN_CHILDLOCK_ON    1  //����

/******  Spec of heaterPower( ���ȵ�λ )  *****/
#define FAN_HEATERPOWER_DEFAULT     1
#define FAN_HEATERPOWER_UNIT    "gear"
#define FAN_HEATERPOWER_UNITNAME    "��"
#define FAN_HEATERPOWER_MIN     0
#define FAN_HEATERPOWER_MAX     10
#define FAN_HEATERPOWER_STEP    1

/******  Spec of ReserveLeftTime( ԤԼʣ��ʱ�� )  *****/
#define FAN_RESERVELEFTTIME_DEFAULT     0
#define FAN_RESERVELEFTTIME_UNIT    "min"
#define FAN_RESERVELEFTTIME_UNITNAME    "����"
#define FAN_RESERVELEFTTIME_MIN     0
#define FAN_RESERVELEFTTIME_MAX     1440
#define FAN_RESERVELEFTTIME_STEP    1

/******  Spec of direction( ���� )  *****/
#define  FAN_DIRECTION_FORWORD   0  //��ת
#define  FAN_DIRECTION_REVERSE   1  //��ת

/******  Spec of angleAutoLROnOff( ������ת/ҡͷ/�ڷ� )  *****/
#define  FAN_ANGLEAUTOLR_OFF     0  //�ر�
#define  FAN_ANGLEAUTOLR_ON      1  //����

/******  Spec of angleAutoAllOnOff( ������ʮ��ҡͷ )  *****/
#define  FAN_ANGLEAUTOALL_OFF    0  //�ر�
#define  FAN_ANGLEAUTOALL_ON     1  //��

/******  Spec of angleAutoUDOnOff( ������ת/ҡͷ/�ڷ� )  *****/
#define  FAN_ANGLEAUTOUD_OFF     0  //�ر�
#define  FAN_ANGLEAUTOUD_ON      1  //����

/******  Spec of powerstate( ���� )  *****/
#define  FAN_POWERSTATE_OFF      0  //�ر�
#define  FAN_POWERSTATE_ON       1  //��

/******  Spec of mode( ģʽ )  *****/
typedef enum
{
    //ģʽ
    FAN_MODE_23 = 23,   //���ʷ�ģʽ
    FAN_MODE_153 = 153, //��׼��ģʽ
    FAN_MODE_165 = 165, //���ܷ�ģʽ
    FAN_MODE_13 = 13,   //��ʪģʽ
    FAN_MODE_164 = 164, //����ģʽ
    FAN_MODE_20 = 20,   //��Ȼ��ģʽ
    FAN_MODE_17 = 17,   //����ģʽ
    FAN_MODE_39 = 39,   //��ͨģʽ
    FAN_MODE_19 = 19,   //������ģʽ
    FAN_MODE_167 = 167, //���˷�ģʽ
    FAN_MODE_230 = 230, //����ģʽ
    FAN_MODE_166 = 166, //ѭ����ģʽ
    FAN_MODE_21 = 21,   //˯�߷�ģʽ
    FAN_MODE_22 = 22    //������ģʽ
} FAN_MODE_T;

//ģʽ
#define FAN_MODE_NUM    14
extern DECL_CONST UINT16 fan_mode_list[];


/******  Spec of windspeed( ���� )  *****/
typedef enum
{
    //����
    FAN_WINDSPEED_44 = 44,  //��ʮ�ĵ�
    FAN_WINDSPEED_6 = 6,    //����
    FAN_WINDSPEED_51 = 51,  //��ʮһ��
    FAN_WINDSPEED_7 = 7,    //�ߵ�
    FAN_WINDSPEED_68 = 68,  //��ʮ�˵�
    FAN_WINDSPEED_16 = 16,  //ʮ����
    FAN_WINDSPEED_38 = 38,  //��ʮ�˵�
    FAN_WINDSPEED_34 = 34,  //��ʮ�ĵ�
    FAN_WINDSPEED_99 = 99,  //��ʮ�ŵ�
    FAN_WINDSPEED_3 = 3,    //����
    FAN_WINDSPEED_256 = 256,    //��ߵ�
    FAN_WINDSPEED_8 = 8,    //�˵�
    FAN_WINDSPEED_95 = 95,  //��ʮ�嵵
    FAN_WINDSPEED_32 = 32,  //��ʮ����
    FAN_WINDSPEED_18 = 18,  //ʮ�˵�
    FAN_WINDSPEED_255 = 255,    //�Զ���
    FAN_WINDSPEED_21 = 21,  //��ʮһ��
    FAN_WINDSPEED_97 = 97,  //��ʮ�ߵ�
    FAN_WINDSPEED_12 = 12,  //ʮ����
    FAN_WINDSPEED_81 = 81,  //��ʮһ��
    FAN_WINDSPEED_71 = 71,  //��ʮһ��
    FAN_WINDSPEED_84 = 84,  //��ʮ�ĵ�
    FAN_WINDSPEED_90 = 90,  //��ʮ��
    FAN_WINDSPEED_74 = 74,  //��ʮ�ĵ�
    FAN_WINDSPEED_64 = 64,  //��ʮ�ĵ�
    FAN_WINDSPEED_67 = 67,  //��ʮ�ߵ�
    FAN_WINDSPEED_36 = 36,  //��ʮ����
    FAN_WINDSPEED_50 = 50,  //��ʮ��
    FAN_WINDSPEED_31 = 31,  //��ʮһ��
    FAN_WINDSPEED_23 = 23,  //��ʮ����
    FAN_WINDSPEED_98 = 98,  //��ʮ�˵�
    FAN_WINDSPEED_53 = 53,  //��ʮ����
    FAN_WINDSPEED_29 = 29,  //��ʮ�ŵ�
    FAN_WINDSPEED_96 = 96,  //��ʮ����
    FAN_WINDSPEED_24 = 24,  //��ʮ�ĵ�
    FAN_WINDSPEED_1 = 1,    //һ��
    FAN_WINDSPEED_82 = 82,  //��ʮ����
    FAN_WINDSPEED_69 = 69,  //��ʮ�ŵ�
    FAN_WINDSPEED_27 = 27,  //��ʮ�ߵ�
    FAN_WINDSPEED_65 = 65,  //��ʮ�嵵
    FAN_WINDSPEED_78 = 78,  //��ʮ�˵�
    FAN_WINDSPEED_153 = 153,    //�е�
    FAN_WINDSPEED_25 = 25,  //��ʮ�嵵
    FAN_WINDSPEED_13 = 13,  //ʮ����
    FAN_WINDSPEED_35 = 35,  //��ʮ�嵵
    FAN_WINDSPEED_46 = 46,  //��ʮ����
    FAN_WINDSPEED_61 = 61,  //��ʮһ��
    FAN_WINDSPEED_52 = 52,  //��ʮ����
    FAN_WINDSPEED_88 = 88,  //��ʮ�˵�
    FAN_WINDSPEED_5 = 5,    //�嵵
    FAN_WINDSPEED_48 = 48,  //��ʮ�˵�
    FAN_WINDSPEED_94 = 94,  //��ʮ�ĵ�
    FAN_WINDSPEED_56 = 56,  //��ʮ����
    FAN_WINDSPEED_41 = 41,  //��ʮһ��
    FAN_WINDSPEED_14 = 14,  //ʮ�ĵ�
    FAN_WINDSPEED_2 = 2,    //����
    FAN_WINDSPEED_15 = 15,  //ʮ�嵵
    FAN_WINDSPEED_10 = 10,  //ʮ��
    FAN_WINDSPEED_49 = 49,  //��ʮ�ŵ�
    FAN_WINDSPEED_91 = 91,  //��ʮһ��
    FAN_WINDSPEED_4 = 4,    //�ĵ�
    FAN_WINDSPEED_45 = 45,  //��ʮ�嵵
    FAN_WINDSPEED_62 = 62,  //��ʮ����
    FAN_WINDSPEED_33 = 33,  //��ʮ����
    FAN_WINDSPEED_17 = 17,  //ʮ�ߵ�
    FAN_WINDSPEED_39 = 39,  //��ʮ�ŵ�
    FAN_WINDSPEED_89 = 89,  //��ʮ�ŵ�
    FAN_WINDSPEED_79 = 79,  //��ʮ�ŵ�
    FAN_WINDSPEED_156 = 156,    //��ǿ��
    FAN_WINDSPEED_75 = 75,  //��ʮ�嵵
    FAN_WINDSPEED_83 = 83,  //��ʮ����
    FAN_WINDSPEED_100 = 100,    //һ�ٵ�
    FAN_WINDSPEED_63 = 63,  //��ʮ����
    FAN_WINDSPEED_77 = 77,  //��ʮ�ߵ�
    FAN_WINDSPEED_92 = 92,  //��ʮ����
    FAN_WINDSPEED_57 = 57,  //��ʮ�ߵ�
    FAN_WINDSPEED_42 = 42,  //��ʮ����
    FAN_WINDSPEED_66 = 66,  //��ʮ����
    FAN_WINDSPEED_86 = 86,  //��ʮ����
    FAN_WINDSPEED_11 = 11,  //ʮһ��
    FAN_WINDSPEED_40 = 40,  //��ʮ��
    FAN_WINDSPEED_43 = 43,  //��ʮ����
    FAN_WINDSPEED_20 = 20,  //��ʮ��
    FAN_WINDSPEED_55 = 55,  //��ʮ�嵵
    FAN_WINDSPEED_19 = 19,  //ʮ�ŵ�
    FAN_WINDSPEED_73 = 73,  //��ʮ����
    FAN_WINDSPEED_47 = 47,  //��ʮ�ߵ�
    FAN_WINDSPEED_28 = 28,  //��ʮ�˵�
    FAN_WINDSPEED_59 = 59,  //��ʮ�ŵ�
    FAN_WINDSPEED_93 = 93,  //��ʮ����
    FAN_WINDSPEED_80 = 80,  //��ʮ��
    FAN_WINDSPEED_54 = 54,  //��ʮ�ĵ�
    FAN_WINDSPEED_26 = 26,  //��ʮ����
    FAN_WINDSPEED_151 = 151,    //�͵�
    FAN_WINDSPEED_58 = 58,  //��ʮ�˵�
    FAN_WINDSPEED_60 = 60,  //��ʮ��
    FAN_WINDSPEED_85 = 85,  //��ʮ�嵵
    FAN_WINDSPEED_9 = 9,    //�ŵ�
    FAN_WINDSPEED_155 = 155,    //�ߵ�
    FAN_WINDSPEED_72 = 72,  //��ʮ����
    FAN_WINDSPEED_76 = 76,  //��ʮ����
    FAN_WINDSPEED_30 = 30,  //��ʮ��
    FAN_WINDSPEED_70 = 70,  //��ʮ��
    FAN_WINDSPEED_22 = 22,  //��ʮ����
    FAN_WINDSPEED_87 = 87   //��ʮ�ߵ�
} FAN_WINDSPEED_T;

/******  Spec of humidity( ʪ�� )  *****/
#define FAN_HUMIDITY_DEFAULT    100.0
#define FAN_HUMIDITY_MIN    0.0
#define FAN_HUMIDITY_MAX    100.0
#define FAN_HUMIDITY_STEP   10.0

/******  Spec of temperature( �¶� )  *****/
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
