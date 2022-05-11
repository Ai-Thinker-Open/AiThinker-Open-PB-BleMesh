
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"

#ifndef _ALIGENIE_APPL_VENDOR_FOGGER_H_
#define _ALIGENIE_APPL_VENDOR_FOGGER_H_

/************  FOGGER Properties  ***********/

/******  ALIGENIE_ATTRIBUTE_CODE  *****/
#define  ALIGENIE_ATTRCODE_FILTEREXPIRED    0x001A
#define  ALIGENIE_ATTRCODE_TARGETHUMIDITY   0x010E
#define  ALIGENIE_ATTRCODE_TEMPERATURE      0x010D
#define  ALIGENIE_ATTRCODE_POWERSTATE       0x0100
#define  ALIGENIE_ATTRCODE_FOG              0x0164
#define  ALIGENIE_ATTRCODE_FOGLEVEL         0x0176
#define  ALIGENIE_ATTRCODE_WATERSHORTAGE    0x0414
#define  ALIGENIE_ATTRCODE_MODE             0xF004

/******  ALIGENIE_PROPERTY  *****/
/******  Spec of humidity( 湿度 )  *****/
#define FOGGER_HUMIDITY_DEFAULT     100.0
#define FOGGER_HUMIDITY_MIN     0.0
#define FOGGER_HUMIDITY_MAX     100.0
#define FOGGER_HUMIDITY_STEP    10.0


/******  Spec of temperature( 温度 )  *****/
#define FOGGER_TEMPERATURE_DEFAULT  0
#define FOGGER_TEMPERATURE_MIN  -30
#define FOGGER_TEMPERATURE_MAX  45
#define FOGGER_TEMPERATURE_STEP     1

/******  Spec of LEVEL( 雾量档位 )  *****/
#define FOGGER_LEVEL_MIN    1
#define FOGGER_LEVEL_MAX    5

/******  Spec of powerstate( 开关 )  *****/
#define  FOGGER_POWERSTATE_OFF   0  //关闭
#define  FOGGER_POWERSTATE_ON    1  //打开

#define  FOGGER_MODE_NUM    8
extern DECL_CONST UINT16 fogger_mode_list[];

API_RESULT fogger_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT UI_HARD_generic_onoff_set(UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state);
API_RESULT fogger_targethumidity_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fogger_temperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fogger_fog_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fogger_fogLevel_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT fogger_waterShortage_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT init_attr_tbl_fogger(UI_DATA_ALIGENIE_MODEL_T* me);
API_RESULT fogger_mode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);

#endif //_ALIGENIE_APPL_VENDOR_FOGGER_H_

