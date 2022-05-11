#include "aliGenie_appl.h"
#include "aliGenie_appl_Light.h"


/* Light Lightness Model Get Handler */
API_RESULT UI_light_lightness_model_state_get(UI_DATA_LIGHT_LIGHTNESS_MODEL_T* me,UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    MS_STATE_LIGHT_LIGHTNESS_STRUCT* param_p;
    API_RESULT retval;
    param_p = (MS_STATE_LIGHT_LIGHTNESS_STRUCT*)param;
    retval = API_SUCCESS;
    printf("\n*************************\
									\n*************************\
									\n*************************\
									\n UI_light_lightness_model_state_get state_t=%d\n\
									\n*************************\
									\n*************************\
									\n*************************\n",state_t);//change by johhn		

    switch(state_t)
    {
    case MS_STATE_LIGHT_LIGHTNESS_DEFAULT_T:
    {
        /* Ignoring Instance and direction right now */
        param_p->light_lightness_default = me->UI_light_lightness.light_lightness_default;
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_RANGE_T:
    {
        /* Ignoring Instance and direction right now */
        param_p->light_lightness_range = me->UI_light_lightness.light_lightness_range;
        param_p->range_status = 0x00;
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_LINEAR_T:
    {
        /* Ignoring Instance and direction right now */
        param_p->light_lightness_linear = me->UI_light_lightness.light_lightness_linear;
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_LAST_T:
    {
        /* Ignoring Instance and direction right now */
        param_p->light_lightness_last = me->UI_light_lightness.light_lightness_last;
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_ACTUAL_T:
    {
        /* Ignoring Instance and direction right now */
        param_p->light_lightness_actual = me->UI_light_lightness.light_lightness_actual;
    }
    break;

    default:
        break;
    }

    return retval;
}

/* Light Lightness Model Set Handler */
/* Todo: Remove the dependency */
#include "math.h"

static void UI_light_lightness_set_actual(UI_DATA_LIGHT_LIGHTNESS_MODEL_T* me,UINT16 state_inst, UINT16 actual)
{
    UINT16 min, max;
    /* Generic OnOff binding */
    min = me->UI_light_lightness.light_lightness_range.lightness_range_min;
    max = me->UI_light_lightness.light_lightness_range.lightness_range_max;

    if ((0 != min) && (actual < min))
    {
        actual = min;
    }
    else if ((0 != max) && (actual > max))
    {
        actual = max;
    }

    /* If Lightness Actual is non-zero, save as Lightness Last */
    if (0x0000 != actual)
    {
        me->UI_light_lightness.light_lightness_last.lightness_last = actual;
    }

    me->UI_light_lightness.light_lightness_actual.lightness_actual = actual;
    /* Light Lightness Linear = ((Actual)^2) / 65535 */
    me->UI_light_lightness.light_lightness_linear.lightness_linear = ((actual * actual) + 65534) / 65535;
}

static void UI_light_lightness_set_linear(UI_DATA_LIGHT_LIGHTNESS_MODEL_T* me,UINT16 state_inst, UINT16 linear)
{
    UINT16 actual;
    UINT32 mul_val;
//    long double d;
    mul_val = linear * 65535;
    actual = (UINT16)sqrt(mul_val);
    /* Light Lightness actual = sqrt(Linear * 65535) */
    UI_light_lightness_set_actual(me,state_inst, actual);
}

API_RESULT UI_light_lightness_model_state_set(UI_DATA_LIGHT_LIGHTNESS_MODEL_T* me,UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    MS_STATE_LIGHT_LIGHTNESS_STRUCT* param_p;
    API_RESULT retval;
    param_p = (MS_STATE_LIGHT_LIGHTNESS_STRUCT*)param;
    retval = API_SUCCESS;
    printf("\n*************************\
									\n*************************\
									\n*************************\
									\n UI_light_lightness_model_state_set state_t:%d\n\
									\n*************************\
									\n*************************\
									\n*************************\n",state_t);//change by johhn		

    switch (state_t)
    {
    case MS_STATE_LIGHT_LIGHTNESS_DEFAULT_T:
    {
        /* Ignoring Instance and direction right now */
        me->UI_light_lightness.light_lightness_default = param_p->light_lightness_default;
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_RANGE_T:
    {
        /* Check range min and max */
        if (param_p->light_lightness_range.lightness_range_min > param_p->light_lightness_range.lightness_range_max)
        {
            /* TODO: add macro define */
            /**
                Table 7.2:
                0x00 - Success
                0x01 - Cannot Set Range Min
                0x02 - Cannot Set Range Max
            */
            param_p->range_status = 0x01;
        }
        else
        {
            /* Ignoring Instance and direction right now */
            me->UI_light_lightness.light_lightness_range = param_p->light_lightness_range;
            param_p->range_status = 0x00;
        }
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_LINEAR_T:
    {
        /* Instantaneous Change */
        UI_light_lightness_set_linear(me,0, param_p->light_lightness_linear.lightness_linear);
        *param_p = me->UI_light_lightness;
        CONSOLE_OUT("[state] current: 0x%02X\n", param_p->light_lightness_linear.lightness_linear);
        CONSOLE_OUT("[state] target: 0x%02X\n", param_p->light_lightness_linear.lightness_target);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", param_p->light_lightness_linear.transition_time);
        /* Ignoring Instance and direction right now */
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_LAST_T:
    {
        /* Ignoring Instance and direction right now */
        me->UI_light_lightness.light_lightness_last = param_p->light_lightness_last;
    }
    break;

    case MS_STATE_LIGHT_LIGHTNESS_ACTUAL_T://ÁÁ¶È
    {
        /* Instantaneous Change */
        UI_light_lightness_set_actual(me,0, param_p->light_lightness_actual.lightness_actual);
        *param_p = me->UI_light_lightness;
        CONSOLE_OUT("[state] current: 0x%02X\n", param_p->light_lightness_actual.lightness_actual);
        CONSOLE_OUT("[state] target: 0x%02X\n", param_p->light_lightness_actual.lightness_target);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", param_p->light_lightness_actual.transition_time);
        printf("\n*************************\
	\n*************************\
	\n*************************\
	\n heightness:%d heightness1:%d\n\
	\n*************************\
	\n*************************\
	\n*************************\n",param_p->light_lightness_actual.lightness_actual,param_p->light_lightness_actual.lightness_actual/257);//change by johhn		
        light_lightness_set_pl(param_p->light_lightness_actual.lightness_actual);
    }
    break;

    default:
        break;
    }

    return retval;
}




/* Light Ligthness Model Server */
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Light_Lightness server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
void UI_light_lightness_scene_recalled_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE          model_handle,
    /* IN */ void*       pdu
)
{
//  UI_DATA_LIGHT_LIGHTNESS_MODEL_T              *me;
//  me = (UI_DATA_LIGHT_LIGHTNESS_MODEL_T *)find_model_private_data(model_handle);
    DEBUG_PRINT("lightness_scene_recalled_cb");
}


/* Light Ligthness Model Server */
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Light_Lightness server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/
API_RESULT UI_light_lightness_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_STATE_LIGHT_LIGHTNESS_STRUCT param;
    MS_ACCESS_MODEL_STATE_PARAMS                    current_state_params;
    API_RESULT retval;
    UI_DATA_LIGHT_LIGHTNESS_MODEL_T*              me;
    appl_dump_bytes(msg_raw->data_param, msg_raw->data_len);
    aligenie_addr = ctx->saddr;
    me = (UI_DATA_LIGHT_LIGHTNESS_MODEL_T*)find_model_private_data(ctx->handle);
    set_element_addr_by_model_handle(ctx->handle, ctx->daddr);
    retval = API_SUCCESS;

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_LIGHTNESS] GET Request.\n");
        UI_light_lightness_model_state_get(me,state_params->state_type, 0, &param, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = &param;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_LIGHTNESS] SET Request.\n");
        UI_light_lightness_model_state_set(me,state_params->state_type, 0, (MS_STATE_LIGHT_LIGHTNESS_STRUCT*)state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = (MS_STATE_LIGHT_LIGHTNESS_STRUCT*)state_params->state;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[LIGHT_LIGHTNESS] Sending Response.\n");

        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        for(uint8_t i=0; i<6; i++)
        {
            retval = MS_light_lightness_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
        }
    }

    return retval;
}




API_RESULT UI_light_lightness_model_server_create
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    API_RESULT retval;
    UI_DATA_LIGHT_LIGHTNESS_MODEL_T* me = NULL;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    me = (UI_DATA_LIGHT_LIGHTNESS_MODEL_T* )EM_alloc_mem(sizeof(UI_DATA_LIGHT_LIGHTNESS_MODEL_T));

    if(NULL == me)
    {
        ERROR_PRINT("light lightness Server Model Create failed! [EM_alloc_mem]!\n");
        EM_free_mem(me);
        return API_FAILURE;
    }

    EM_mem_set(me,0,sizeof(UI_DATA_LIGHT_LIGHTNESS_MODEL_T));
    retval = MS_light_lightness_server_init(element_handle,&model_handle,UI_light_lightness_server_cb);

    if (API_SUCCESS != retval)
    {
        ERROR_PRINT("light lightness Server Model Create failed. [MS_light_lightness_server_init]: 0x%04X\n",retval);
        EM_free_mem(me);
        return retval;
    }

    if(FALSE == appl_model_tree_add(element_handle,model_handle,(void*)me))
    {
        ERROR_PRINT("light lightness Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    me->element_handle = element_handle;
    me->model_handle = model_handle;

    if (API_FAILURE == aliGenie_bleMesh_Generic_Scene_register_Recall_cb(model_handle,UI_light_lightness_scene_recalled_cb))
    {
        ERROR_PRINT("aliGenie_bleMesh_Generic_Scene_register_Recall_cb failed! element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    INFO_PRINT( "light lightness Server Model Create Successed. [model_handle]: 0x%04X\n",me->model_handle);
    return API_SUCCESS;
}

