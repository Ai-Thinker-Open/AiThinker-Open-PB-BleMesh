
#ifndef _MS_GENERIC_ONOFF_API_H_
#define _MS_GENERIC_ONOFF_API_H_

#include "MS_generic_onoff_api.h"
#include "aliGenie_appl_vendor.h"

typedef struct ui_data_generic_onoff_model
{
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_STATE_GENERIC_ONOFF_STRUCT   UI_state_generic_onoff;
    struct ui_data_aligenie_model* vendor_me;
    UINT8   io_num;
    EM_timer_handle tmrhdl_delay;
    EM_timer_handle tmrhdl_trans;


} UI_DATA_GENERIC_ONOFF_MODEL_T;

typedef struct state_msg_8202
{
    UINT8   target_onoff;
    UINT8   tid;
    UINT8   transition_time;
    UINT8   delay;
} STATE_MSG_8202_T;

extern MS_ACCESS_MODEL_HANDLE   UI_generic_scene_server_model_handle;
extern MS_ACCESS_MODEL_HANDLE   UI_generic_scene_setup_server_model_handle;
extern void UI_generic_onoff_model_states_initialization(void);
extern API_RESULT UI_generic_onoff_model_state_get(UI_DATA_GENERIC_ONOFF_MODEL_T* me,UINT16 state_t, UINT16 state_inst, UCHAR* pdu, UINT8 direction);
extern API_RESULT UI_generic_onoff_model_state_set(UI_DATA_GENERIC_ONOFF_MODEL_T* me, MS_ACCESS_MODEL_REQ_MSG_RAW* msg_raw);

extern API_RESULT UI_generic_onoff_model_server_cb (
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
);
extern API_RESULT UI_generic_onoff_model_server_create(
    MS_ACCESS_ELEMENT_HANDLE element_handle,
    UINT8   io_num,
    UI_DATA_GENERIC_ONOFF_MODEL_T * * pme
);



#define MS_SCENE_NUMBER_MAX                 16
#define MS_SCENE_MODEL_MAX                  3

typedef struct UI_state_scene_struct
{
    UINT32                          index;
    MS_STATE_LIGHT_HSL_STRUCT       light_hsl_state;
    MS_STATE_GENERIC_ONOFF_STRUCT   onoff_state;
} UI_STATE_SCENE_STRUCT;

typedef struct ui_data_generic_scene_model UI_DATA_GENERIC_SCENE_MODEL_T;

struct ui_data_generic_scene_model
{
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_ACCESS_MODEL_HANDLE          setup_model_handle;
    MS_ACCESS_MODEL_HANDLE  recalled_model_handle[MS_SCENE_MODEL_MAX];  // one recall callback for one model, process all scenes
    void (*recalled_cb[MS_SCENE_MODEL_MAX])(MS_ACCESS_MODEL_HANDLE recalled_model_handle,void* context);

};

extern API_RESULT UI_generic_scene_model_server_create(MS_ACCESS_ELEMENT_HANDLE element_handle,UINT8 scene_num,DECL_CONST UINT16* scene_list);
extern API_RESULT aliGenie_bleMesh_Generic_Scene_register_Recall_cb(MS_ACCESS_MODEL_HANDLE recalled_model_handle,void (*recalled_cb)(MS_ACCESS_MODEL_HANDLE recalled_model_handle, void* context));
extern API_RESULT UI_HARD_generic_onoff_set (UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state);


#endif //_MS_GENERIC_ONOFF_API_H_

