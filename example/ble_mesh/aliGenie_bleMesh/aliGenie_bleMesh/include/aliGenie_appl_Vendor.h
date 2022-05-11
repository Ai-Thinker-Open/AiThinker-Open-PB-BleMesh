

#ifndef _ALIGENIE_APPL_VENDOR_H_
#define _ALIGENIE_APPL_VENDOR_H_

#include "aliGenie_appl.h"
#include "aliGenie_appl_Generic.h"


//Attribute data type
#define BOOL    1
#define U8      2
#define S8      3
#define U16     4
#define S16     5
#define U32     6
#define S32     7
#define U64     8
#define PLD6    9
extern DECL_CONST UINT8 attr_len_def[];

#define RPL_MAX_LEN 15*4

typedef struct ATTR_TBL
{
    UINT16  attr_code;
    UINT32  attr_val;
    UINT8   attr_type; //0=invalid, 1=BOOL, 2=u8, 3=s8,4=u16,5=s16,6=u32,7=s32,8=STRUCT,9=POINTER
} ATTR_TBL_T;

struct ui_aligenie_element_attr;
struct ui_data_aligenie_model;
typedef struct ui_aligenie_element_attr UI_ALIGENIE_ELEMENT_ATTR_T;
typedef struct ui_data_aligenie_model UI_DATA_ALIGENIE_MODEL_T;


struct ui_aligenie_element_attr
{
    UINT16  attr_code;
    UINT32  attr_val;
    UINT8   attr_type; //0=invalid, 1=BOOL, 2=u8, 3=s8,4=u16,5=s16,6=u32,7=s32,8=POINTER
    API_RESULT  (*attr_set_action)(UI_DATA_ALIGENIE_MODEL_T* private_data,UINT32 val);
    API_RESULT  (*attr_get_action)(UI_DATA_ALIGENIE_MODEL_T* private_data,UINT32* val);

};


struct ui_data_aligenie_model
{
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    struct ui_data_generic_onoff_model* generic_onoff_me;
    UINT8   io_num;
    EM_timer_handle tmrhdl_delay;
    EM_timer_handle tmrhdl_trans;
    UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl;
    UINT8                       attr_tbl_num;

};



typedef struct
{
    MS_ACCESS_MODEL_HANDLE*     handle;
    MS_NET_ADDR                 saddr;
    MS_NET_ADDR                 daddr;
    MS_SUBNET_HANDLE            subnet_handle;
    MS_APPKEY_HANDLE            appkey_handle;
    UINT32                      opcode;
    UCHAR*                     data_param;
    UINT16                      data_len;

} MS_CALLBACK_DATA_T;

extern API_RESULT (*pinit_attr_tbl)(UI_DATA_ALIGENIE_MODEL_T* me);

UINT32 parse_attr_val(UCHAR* data_param,   UINT8 attr_type);
int set_attr(UI_DATA_ALIGENIE_MODEL_T* me, UINT16 attr_code, UCHAR* data_ptr, int* attr_len);
API_RESULT Get_attr(UI_DATA_ALIGENIE_MODEL_T* me, UINT16 attr_code, UINT32* val);
API_RESULT init_attr(UI_ALIGENIE_ELEMENT_ATTR_T* attr,UINT16 code, UINT32 val, UINT8 type, API_RESULT (*attr_set_action)(UI_DATA_ALIGENIE_MODEL_T* private_data,UINT32 val));
API_RESULT init_attr_tbl(UI_DATA_ALIGENIE_MODEL_T* private_data);
void print_attr_tbl(UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl,UINT8 num);
void make_attr_pdu(UCHAR* rpl_data, UINT16 attr_code, UINT32 attr_val, int attr_len);

API_RESULT vendor_Aligenie_indication(MS_NET_ADDR saddr,UINT16 attr_code, UINT32 attr_val,int attr_len);


#endif  //_ALIGENIE_APPL_VENDOR_H_
