#include "aliGenie_appl.h"
#include "aliGenie_appl_Light.h"

API_RESULT UI_light_hsl_model_state_get(UI_DATA_LIGHT_HSL_MODEL_T* me,UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_LIGHT_HSL_T:
    {
        MS_STATE_LIGHT_HSL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_HSL_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        *param_p = me->UI_light_hsl;
    }
    break;

    default:
        break;
    }

    return API_SUCCESS;
}

void UI_light_hsl_set_actual(UINT16 state_inst, UINT16 lightness, UINT16 hue, UINT16 saturation, UCHAR forced_publish)
{
//    printf("appl_light_hsl_set_actual: L:0x%04X, H:0x%04X, S:0x%04X, F:%d\n",
//    lightness, hue, saturation, forced_publish);
//    if (lightness != appl_light_hsl[state_inst].hsl_lightness)
//    {
//        appl_light_hsl[state_inst].hsl_lightness = lightness;
//        appl_light_hsl[state_inst].hsl_hue = hue;
//        appl_light_hsl[state_inst].hsl_saturation = saturation;
//
//        appl_light_hsl_server_publish(MS_STATE_LIGHT_HSL_T, state_inst);
//
//        /* appl_light_lightness[state_inst].light_lightness_actual.lightness_actual = param_p->hsl_lightness; */
//        appl_light_lightness_set_actual(state_inst, lightness, MS_FALSE);
//    }
//    else if (MS_TRUE == forced_publish)
//    {
//        appl_light_hsl[state_inst].hsl_lightness = lightness;
//        appl_light_hsl[state_inst].hsl_hue = hue;
//        appl_light_hsl[state_inst].hsl_saturation = saturation;
//
//        appl_light_hsl_server_publish(MS_STATE_LIGHT_HSL_T, state_inst);
//    }
    light_hsl_set_pl(hue,saturation,lightness);
}

API_RESULT UI_light_hsl_model_state_set(UI_DATA_LIGHT_HSL_MODEL_T* me,UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    API_RESULT retval;
    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_LIGHT_HSL_T:
    {
        MS_STATE_LIGHT_HSL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_HSL_STRUCT*)param;
        {
            printf("\n*************************\
		\n*************************\
		\n*************************\
		\n UI_light_hsl_model_state_set  param_p->hsl_saturation:%d\n\
		\n*************************\
		\n*************************\
		\n*************************\n",param_p->hsl_saturation);
            me->UI_light_hsl.hsl_lightness = param_p->hsl_lightness;
            me->UI_light_hsl.hsl_hue = param_p->hsl_hue;
            me->UI_light_hsl.hsl_saturation = param_p->hsl_saturation;
            me->UI_light_hsl.tid = param_p->tid;
            /*
                appl_light_hsl[0].hsl_lightness = param_p->hsl_lightness;
                appl_light_hsl[0].hsl_lightness = param_p->hsl_lightness;
                appl_light_hsl[0].hsl_lightness = param_p->hsl_lightness;
            */
            //appl_light_lightness[state_inst].light_lightness_actual.lightness_actual = param_p->hsl_lightness;
        }
        UI_light_hsl_set_actual(0,me->UI_light_hsl.hsl_lightness,me->UI_light_hsl.hsl_hue,me->UI_light_hsl.hsl_saturation,0);
        *param_p = me->UI_light_hsl;
        CONSOLE_OUT("[state] current Hue: 0x%04X\n", me->UI_light_hsl.hsl_hue);
        CONSOLE_OUT("[state] current Saturation: 0x%04X\n", me->UI_light_hsl.hsl_saturation);
        CONSOLE_OUT("[state] current Lightness: 0x%04X\n", me->UI_light_hsl.hsl_lightness);
        CONSOLE_OUT("[state] target Hue: 0x%04X\n", me->UI_light_hsl.target_hsl_hue);
        CONSOLE_OUT("[state] target Saturation: 0x%04X\n", me->UI_light_hsl.target_hsl_saturation);
        CONSOLE_OUT("[state] target Lightness: 0x%04X\n", me->UI_light_hsl.target_hsl_lightness);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", me->UI_light_hsl.transition_time);
        /* Ignoring Instance and direction right now */
    }
    break;

    default:
        break;
    }

    return retval;
}





/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Light_Hsl server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
API_RESULT UI_light_hsl_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_ACCESS_MODEL_STATE_PARAMS         current_state_params;
    MS_STATE_LIGHT_HSL_STRUCT            param;
    MS_STATE_LIGHT_HSL_RANGE_STRUCT      param_range;
    MS_STATE_LIGHT_HSL_DEFAULT_STRUCT    param_default;
    void*                                param_p;
    API_RESULT                           retval;
    retval = API_SUCCESS;
    UI_DATA_LIGHT_HSL_MODEL_T*            me;
    appl_dump_bytes(msg_raw->data_param, msg_raw->data_len);
    aligenie_addr = ctx->saddr;
    me = (UI_DATA_LIGHT_HSL_MODEL_T*)find_model_private_data(ctx->handle);

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_HSL] GET Request.\n");

        switch (state_params->state_type)
        {
        case MS_STATE_LIGHT_HSL_T:
        case MS_STATE_LIGHT_HSL_HUE_T:
        case MS_STATE_LIGHT_HSL_SATURATION_T:
        case MS_STATE_LIGHT_HSL_TARGET_T:
        {
            param_p = &param;
        }
        break;

        case MS_STATE_LIGHT_HSL_DEFAULT_T:
        {
            param_p = &param_default;
        }
        break;

        case MS_STATE_LIGHT_HSL_RANGE_T:
        {
            param_p = &param_range;
        }
        break;

        default:
            break;
        }

        UI_light_hsl_model_state_get(me,state_params->state_type, 0, param_p, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = param_p;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_HSL] SET Request.\n");
        UI_light_hsl_model_state_set(me,state_params->state_type, 0, state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = state_params->state;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[LIGHT_HSL] Sending Response.\n");

        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        for(uint8_t i=0; i<6; i++)
        {
            retval = MS_light_hsl_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
        }
    }

    return retval;
}

API_RESULT UI_light_hsl_model_server_create
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    API_RESULT retval;
    UI_DATA_LIGHT_HSL_MODEL_T* me = NULL;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_ACCESS_MODEL_HANDLE          setup_model_handle;
    me = (UI_DATA_LIGHT_HSL_MODEL_T* )EM_alloc_mem(sizeof(UI_DATA_LIGHT_HSL_MODEL_T));

    if(NULL == me)
    {
        ERROR_PRINT("Light hsl Server Model Create failed! [EM_alloc_mem]!\n");
        return API_FAILURE;
    }

    EM_mem_set(me,0,sizeof(UI_DATA_LIGHT_HSL_MODEL_T));
    retval = MS_light_hsl_server_init(element_handle,&model_handle,&setup_model_handle,UI_light_hsl_server_cb);

    if (API_SUCCESS != retval)
    {
        ERROR_PRINT("Light hsl Server Model Create failed. [MS_light_hsl_server_init]: 0x%04X\n",retval);
        EM_free_mem(me);
        return retval;
    }

    if(FALSE == appl_model_tree_add(element_handle,model_handle,(void*)me))
    {
        ERROR_PRINT("Light hsl Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    if(FALSE == appl_model_tree_add(element_handle,setup_model_handle,(void*)me))
    {
        ERROR_PRINT("Light hsl Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, setup_model_handle = 0x%04X\n",element_handle,setup_model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    me->element_handle = element_handle;
    me->model_handle = model_handle;
    me->setup_model_handle = setup_model_handle;
    INFO_PRINT( "Light hsl Server Model Create Successed. [model_handle]: 0x%04X\n",me->model_handle);
    return API_SUCCESS;
}


