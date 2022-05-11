
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
/******  Spec of powerstate( ���� )  *****/
#define  LIGHT_POWERSTATE_OFF    0  //�ر�
#define  LIGHT_POWERSTATE_ON     1  //��

/******  Spec of color( ��ɫ )  *****/
typedef enum
{
    //��ɫ
    LIGHT_COLOR_10824234    = 10824234, //��ɫ
    LIGHT_COLOR_16711680    = 16711680, //��ɫ
    LIGHT_COLOR_4915330     = 4915330,  //����
    LIGHT_COLOR_16761035    = 16761035, //�ۺ�
    LIGHT_COLOR_65280       = 65280,    //��ɫ
    LIGHT_COLOR_16753920    = 16753920, //��ɫ
    LIGHT_COLOR_16777215    = 16777215, //��ɫ
    LIGHT_COLOR_255         = 255,  //��ɫ
    LIGHT_COLOR_35723       = 35723,    //����ɫ
    LIGHT_COLOR_16776960    = 16776960, //��ɫ
    LIGHT_COLOR_8388736     = 8388736,  //��ɫ
    LIGHT_COLOR_16711935    = 16711935, //���
    LIGHT_COLOR_139         = 139,  //����ɫ
    LIGHT_COLOR_65535       = 65535,    //��ɫ
    LIGHT_COLOR_7048739     = 7048739,  //ǳ��ɫ
    LIGHT_COLOR_0           = 0,    //��ɫ
    LIGHT_COLOR_15631086    = 15631086, //������
    LIGHT_COLOR_8900331     = 8900331   //����ɫ
} LIGHT_COLOR_T;

/******  Spec of mode( ģʽ )  *****/
typedef enum
{
    //ģʽ
    LIGHT_MODE_652 = 652,   //�Ų�ģʽ
    LIGHT_MODE_15 = 15, //����ģʽ
    LIGHT_MODE_353 = 353,   //����ģʽ
    LIGHT_MODE_653 = 653,   //ҡһҡģʽ
    LIGHT_MODE_6 = 6,   //ҹ��ģʽ
    LIGHT_MODE_3 = 3,   //�Ķ�ģʽ
    LIGHT_MODE_14 = 14, //˯��ģʽ
    LIGHT_MODE_5 = 5,   //��ůģʽ
    LIGHT_MODE_33 = 33, //����ģʽ
    LIGHT_MODE_651 = 651,   //��˷�ģʽ
    LIGHT_MODE_57 = 57, //����ģʽ
    LIGHT_MODE_226 = 226,   //����ģʽ
    LIGHT_MODE_378 = 378,   //����ģʽ
    LIGHT_MODE_409 = 409    //��ʱ��Ϣģʽ
} LIGHT_MODE_T;


/******  Spec of brightness( ���� )  *****/
#define LIGHT_BRIGHTNESS_DEFAULT    100
#define LIGHT_BRIGHTNESS_UNIT   '%'
#define LIGHT_BRIGHTNESS_UNITNAME   "�ٷֱ�"
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
void light_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32 data);
API_RESULT init_attr_tbl_light(UI_DATA_ALIGENIE_MODEL_T* me);

#endif //_ALIGEN_APPL_VENDOR_LIGHT_H_
