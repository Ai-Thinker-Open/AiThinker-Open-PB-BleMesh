
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"
#include "aliGenie_appl_Vendor_light.h"
#include "aliGenie_appl_Vendor_fan.h"

#ifndef _ALIGENIE_APPL_VENDOR_FANLIGHT_H_
#define _ALIGENIE_APPL_VENDOR_FANLIGHT_H_

/************  FANLIGHT Properties  ***********/

/******  Spec of powerstate_2( ����_2 )  *****/
#define  FANLIGHT_POWERSTATE_2_OFF   0  //�ر�
#define  FANLIGHT_POWERSTATE_2_ON    1  //��

/******  Spec of mainLight( ���� )  *****/
#define  FANLIGHT_MAINLIGHT_OFF      0  //�ر�
#define  FANLIGHT_MAINLIGHT_ON   1  //��


/******  Spec of backLight( ����� )  *****/
#define  FANLIGHT_BACKLIGHT_OFF      0  //�ر�
#define  FANLIGHT_BACKLIGHT_ON   1  //��


/******  Spec of LightType( ������ )  *****/
enum FANLIGHT_LIGHTTYPE_T
{
    //������
    FANLIGHT_LIGHTTYPE_0 = 0,   //C
    FANLIGHT_LIGHTTYPE_1 = 1,   //CW
    FANLIGHT_LIGHTTYPE_2 = 2,   //RGB
    FANLIGHT_LIGHTTYPE_3 = 3    //RGBC
};

API_RESULT fanlight_mainLight_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fanlight_backLight_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
void fanlight_recall_mode_set(MS_ACCESS_MODEL_HANDLE recalled_model_handle, UINT32 data);
API_RESULT init_attr_tbl_fanlight(UI_DATA_ALIGENIE_MODEL_T* me);
#endif //_ALIGENIE_APPL_VENDOR_FANLIGHT_H_

