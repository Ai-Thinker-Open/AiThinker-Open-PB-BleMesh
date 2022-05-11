#include "aliGenie_appl.h"
#include "aliGenie_appl_Light.h"

typedef struct
{
    uint16_t present_lightness;
    uint16_t present_temp;
    uint16_t target_lightness;
    uint16_t target_temp;
    uint8_t remain_t;
} mesh_cmd_light_ctl_st_t;



/* Model Server - Foundation Models */


/**
    \brief Server Application Asynchronous Notification Callback. //´ý¶¨

    \par Description
    Light_Ctl server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/

API_RESULT MS_light_ctl_server_state_update_johhn
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params,
    /* IN */ UINT8         count
)
{
    //vm_vendor_mode_indication(*current_state_params);
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];//={0xeb,0x11,0xe0,0x03,0x00,0x00,0xe0,0x03,0x00,0x00};
    UCHAR*     pdu_ptr;
    UINT16     marker;
    UINT32     opcode;
    opcode=MS_ACCESS_LIGHT_CTL_STATUS_OPCODE;
    //LOG("color opcode1:%x \n",opcode);
    retval = API_FAILURE;
    marker = 0;
    CONSOLE_OUT(
        "[VENDOR_ALIGENIE_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    }

    /* Publish - reliable */
    //if (0 == marker)
    //{
    //pdu_ptr = NULL;
    // }
    //else
    {
        //mesh_cmd_light_ctl_st_t test={0};
        //test.present_temp=20;
        //test.present_lightness=50;
        marker=count;
        UCHAR* p=buffer;
        p+=4;
        UCHAR* q=(UCHAR*)(current_state_params->state);
        //q+=2;
        //marker=sizeof(mesh_cmd_light_ctl_st_t);
        memcpy(buffer,current_state_params->state,4);
        memcpy(p,q,4);
        //  memcpy(&buffer[0],((UCHAR*)(current_state_params->state)),3);
        pdu_ptr = buffer;
    }
    retval = MS_access_reply
             (
                 &ctx->handle,
                 ctx->daddr,
                 ctx->saddr,
                 ctx->subnet_handle,
                 ctx->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 pdu_ptr,
                 marker
             );
//LOG("color opcode3:%x \n",opcode);
    return retval;
}



/** --------------------- Light - CTL ---- */
API_RESULT UI_light_ctl_model_state_get(UI_DATA_LIGHT_CTL_MODEL_T* me,UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction)
{
    switch(state_t)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_CTL_STRUCT*)param;
        /* Ignoring Instance and direction right now */
        *param_p = me->UI_light_ctl;
    }
    break;

    default:
        break;
    }

    return API_SUCCESS;
}
//static void UI_light_ctl_temp_set_actual(UI_DATA_LIGHT_CTL_MODEL_T *me,UINT16 state_inst, UINT16 actual)
//{
//    UINT16 min, max;

//    min = 0x0320;
//    max = 0x4E20;

//    /* Generic OnOff binding */
//    if (0x0000 == actual)
//    {
//        /* appl_generic_onoff[state_inst].onoff = 0x00; */
//    }
//    else
//    {
//        if ((0 != min) && (actual < min))
//        {
//            actual = min;
//        }
//        else if ((0 != max) && (actual > max))
//        {
//            actual = max;
//        }

//        /* appl_generic_onoff[state_inst].onoff = 0x01; */

//        /* appl_light_lightness[state_inst].light_lightness_last.lightness_last = actual; */
//    }

//    me->UI_light_ctl.ctl_temperature = actual;
//    light_ctl_set_pl(actual,0);
//    /* Light Lightness Linear = ((Actual)^2) / 65535 */
//    /* appl_light_lightness[state_inst].light_lightness_linear.lightness_linear = ((actual * actual) + 65534) / 65535; */

//    /* Generic Level = (Light CTL Temperature - T _MIN) * 65535 / (T_MAX - T_MIN) - 32768 */
//    //appl_generic_level_info[state_inst].generic_level.level = (((actual - min) * 0xFFFF) / (max - min)) - 32768;
//}
API_RESULT UI_light_ctl_model_state_set(UI_DATA_LIGHT_CTL_MODEL_T* me,UINT16 state_t, UINT16 state_inst, void* param, UINT8 direction) //è‰²æ¸©
{
    API_RESULT retval;
    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT* param_p;
        param_p = (MS_STATE_LIGHT_CTL_STRUCT*)param;
        {
            /* Instantaneous Change */
            /*
                UI_light_ctl = *param_p;
                UI_light_ctl_temp_set_actual(0, param_p->ctl_temperature);
                UI_light_lightness_set_actual(0, param_p->ctl_lightness);
                light_lightness_set_pl(param_p->ctl_lightness);
                UI_light_lightness.light_lightness_actual.lightness_actual = param_p->ctl_lightness;
            */
        }
        {
            /* Instantaneous Change */
            me->UI_light_ctl = *param_p;
            //light_temperature_set_pl(param_p->ctl_temperature);  //lhj remove temperature
            //UI_light_lightness_set_actual(0, param_p->ctl_lightness);
            //light_lightness_set_pl(param_p->ctl_lightness);
            //UI_light_lightness.light_lightness_actual.lightness_actual = param_p->ctl_lightness;
        }
        printf("\n*************************\
	\n*************************\
	\n*************************\
	\nlight_lightness_set_pl cold hot \n\n\
	\n*************************\
	\n*************************\
	\n*************************\n");//change by johhn
        *param_p = me->UI_light_ctl;
        CONSOLE_OUT("[state] current Lightness: 0x%02X\n", me->UI_light_ctl.ctl_lightness);
        CONSOLE_OUT("[state] target Lightness: 0x%02X\n", me->UI_light_ctl.target_ctl_lightness);
        CONSOLE_OUT("[state] current Temperature: 0x%02X\n", me->UI_light_ctl.ctl_temperature);
        CONSOLE_OUT("[state] target Temperature: 0x%02X\n", me->UI_light_ctl.target_ctl_temperature);
        CONSOLE_OUT("[state] remaining_time: 0x%02X\n", me->UI_light_ctl.transition_time);
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
    Light_Ctl server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/

API_RESULT UI_light_ctl_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    MS_ACCESS_MODEL_STATE_PARAMS                   current_state_params;
    MS_STATE_LIGHT_CTL_STRUCT                      light_ctl_params;
    MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_STRUCT    light_ctl_temperature_range_params;
    MS_STATE_LIGHT_CTL_DEFAULT_STRUCT              light_ctl_default_params;
    MS_STATE_LIGHT_CTL_TEMPERATURE_STRUCT          light_ctl_temperature_params;
    void*                                          param_p;
    API_RESULT                                     retval;
    MS_IGNORE_UNUSED_PARAM(msg_raw);
    MS_IGNORE_UNUSED_PARAM(ext_params);
    UI_DATA_LIGHT_CTL_MODEL_T*            me;
    appl_dump_bytes(msg_raw->data_param, msg_raw->data_len);
    aligenie_addr = ctx->saddr;
    me = (UI_DATA_LIGHT_CTL_MODEL_T*)find_model_private_data(ctx->handle);
    retval = API_SUCCESS;

    /* Check message type */
    if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_CTL] GET Request.\n");

        switch (state_params->state_type)
        {
        case MS_STATE_LIGHT_CTL_DEFAULT_T:
        {
            param_p = &light_ctl_default_params;
        }
        break;

        case MS_STATE_LIGHT_CTL_T:
        {
            param_p = &light_ctl_params;
        }
        break;

        case MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_T:
        {
            param_p = &light_ctl_temperature_range_params;
        }
        break;

        case MS_STATE_LIGHT_CTL_TEMPERATURE_T:
        {
            param_p = &light_ctl_temperature_params;
        }
        break;

        default:
            break;
        }

        UI_light_ctl_model_state_get(me,state_params->state_type, 0, param_p, 0);
        current_state_params.state_type = state_params->state_type;
        current_state_params.state = param_p;
    }
    else if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
    {
        CONSOLE_OUT(
            "[LIGHT_CTL] SET Request.\n");
        UI_light_ctl_model_state_set(me,state_params->state_type, 0, state_params->state, 0);
        current_state_params.state_type = state_params->state_type;
        //current_state_params.state = state_params->state;
        current_state_params.state =msg_raw->data_param;
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        CONSOLE_OUT(
            "[LIGHT_CTL] Sending Response.\n");
        /* Parameters: Request Context, Current State, Target State (NULL: to be ignored), Remaining Time (0: to be ignored), Additional Parameters (NULL: to be ignored) */
        //retval = MS_light_ctl_server_state_update(ctx, &current_state_params, NULL, 0, NULL);
        retval = MS_light_ctl_server_state_update_johhn(ctx, &current_state_params, NULL, 0, NULL,msg_raw->data_len);
    }

    return retval;
}


API_RESULT UI_light_ctl_model_server_create
(
    MS_ACCESS_ELEMENT_HANDLE element_handle
)
{
    API_RESULT retval;
    UI_DATA_LIGHT_CTL_MODEL_T* me = NULL;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_ACCESS_MODEL_HANDLE          setup_model_handle;
    me = (UI_DATA_LIGHT_CTL_MODEL_T* )EM_alloc_mem(sizeof(UI_DATA_LIGHT_CTL_MODEL_T));

    if(NULL == me)
    {
        ERROR_PRINT("Light ctl Server Model Create failed! [EM_alloc_mem]!\n");
        return API_FAILURE;
    }

    EM_mem_set(me,0,sizeof(UI_DATA_LIGHT_CTL_MODEL_T));
    retval = MS_light_ctl_server_init(element_handle,&model_handle,&setup_model_handle,UI_light_ctl_server_cb);

    if (API_SUCCESS != retval)
    {
        ERROR_PRINT("Light ctl Server Model Create failed. [MS_light_ctl_server_init]: 0x%04X\n",retval);
        EM_free_mem(me);
        return retval;
    }

    if(FALSE == appl_model_tree_add(element_handle,model_handle,(void*)me))
    {
        ERROR_PRINT("Light ctl Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    if(FALSE == appl_model_tree_add(element_handle,setup_model_handle,(void*)me))
    {
        ERROR_PRINT("Light ctl Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, setup_model_handle = 0x%04X\n",element_handle,setup_model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    me->element_handle = element_handle;
    me->model_handle = model_handle;
    me->setup_model_handle = setup_model_handle;
    INFO_PRINT( "Light ctl Server Model Create Successed. [model_handle]: 0x%04X\n",me->model_handle);
    return API_SUCCESS;
}

