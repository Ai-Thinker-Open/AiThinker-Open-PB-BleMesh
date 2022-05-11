
#include "aliGenie_appl.h"
#include "aliGenie_appl_Generic.h"
MS_ACCESS_MODEL_HANDLE   UI_generic_scene_server_model_handle;
MS_ACCESS_MODEL_HANDLE   UI_generic_scene_setup_server_model_handle;

extern  void UI_generic_scene_model_states_initialization(MS_ACCESS_MODEL_HANDLE model_handle);


API_RESULT aliGenie_bleMesh_Generic_Scene_register_Recall_cb(MS_ACCESS_MODEL_HANDLE recalled_model_handle,void (*recalled_cb)(MS_ACCESS_MODEL_HANDLE recalled_model_handle, void* context))
{
    bool ret;
    UI_DATA_GENERIC_SCENE_MODEL_T* me;
    MS_ACCESS_MODEL_HANDLE generic_scene_model_handle;
    DEBUG_PRINT("Generic_Scene_register recalled_model_handle =  0x%04X\r\n",recalled_model_handle)
    ret = copyget_generic_scene_modele_handle(recalled_model_handle,&generic_scene_model_handle);

    if(!ret)
    {
        ERROR_PRINT("recalled_model_handle 0x%04X NOT FOUND!\r\n",recalled_model_handle);
        return API_FAILURE;
    }

    //find me of scene model and set callback
    me = (UI_DATA_GENERIC_SCENE_MODEL_T*)find_model_private_data( generic_scene_model_handle);

    if(NULL == me)
    {
        return API_FAILURE;
    }

    //DEBUG_PRINT("FOUND generic_scene_model_handle =  0x%04X\r\n",generic_scene_model_handle)
    for(int i = 0; i < MS_SCENE_MODEL_MAX; i++)
    {
        if(0 == me->recalled_model_handle[i])
        {
            if( NULL == me->recalled_cb[i])
            {
                me->recalled_model_handle[i] = recalled_model_handle;
                me->recalled_cb[i] = recalled_cb;
                return API_SUCCESS;
            }
            else
            {
                ERROR_PRINT("WRONG recalled_cb for index = %d, MODEL_HANDLE = 0x%04X\r\n",i,me->model_handle);
                return API_FAILURE;
            }
        }
    }

    ERROR_PRINT("REGISTER recalled_cb FAILED because of FULL!\r\n");
    return API_FAILURE;
}
void aliGenie_bleMesh_Generic_Scene_Recall(UI_DATA_GENERIC_SCENE_MODEL_T* me, void* context)
{
    int i;

    for(i = 0; i < MS_SCENE_MODEL_MAX; i++)
    {
        if((0 != me->recalled_model_handle[i]) && (NULL != me->recalled_cb[i]))
        {
            (me->recalled_cb[i])(me->recalled_model_handle[i],context);
        }
    }
}


/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Scene server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/

void* UI_generic_scene_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*      handle,
    /* IN */ UINT8                      event_type,
    /* IN */ void*                        event_param,
    /* IN */ UINT16                     event_length,
    /* IN */ void*                        context
)
{
    UI_DATA_GENERIC_SCENE_MODEL_T* me;
    printf("[GENERIC_SCENE] event_type [%04X].\n",event_type);
    appl_dump_bytes(event_param,event_length);
    void*       param_p;
    UINT32     index;
    UI_STATE_SCENE_STRUCT*   state_p;
//  MS_STATE_LIGHT_HSL_STRUCT *state_hsl;
    param_p = NULL;
    (void)handle;
    me = (UI_DATA_GENERIC_SCENE_MODEL_T*)find_model_private_data(* handle);

    switch(event_type)
    {
    case MS_SCENE_EVENT_STORE:
    {
        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        //printf("store scene, index = %d \n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)EM_alloc_mem(sizeof(UI_STATE_SCENE_STRUCT));

        if (NULL == state_p)
        {
            //printf("Allocate memory fail, size = %d\n", sizeof(UI_STATE_SCENE_STRUCT));
            return NULL;
        }

        #if 0 //lhj move to hsl
        // save current states
        state_p->index = index;

        switch(state_p->index)
        {
        case 0:
        {
            UI_light_hsl.hsl_hue = 0x8000;
            UI_light_hsl.hsl_saturation =  0xffff;
            UI_light_hsl.hsl_lightness = 0x8000 ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        case 1:
        {
            UI_light_hsl.hsl_hue = 0x2aaa;
            UI_light_hsl.hsl_saturation =  0xffff;
            UI_light_hsl.hsl_lightness = 0x8000 ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        case 2:
        {
            UI_light_hsl.hsl_hue = 0x0000;
            UI_light_hsl.hsl_saturation =  0xffff;
            UI_light_hsl.hsl_lightness = 0x8000 ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        case 3:
        {
            UI_light_hsl.hsl_hue = 0x0000;
            UI_light_hsl.hsl_saturation =  0x0000;
            UI_light_hsl.hsl_lightness = 0xffff ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        case 4:
        {
            UI_light_hsl.hsl_hue = 0x1b9b;
            UI_light_hsl.hsl_saturation =  0xffff;
            UI_light_hsl.hsl_lightness = 0x8000 ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        case 5:
        {
            UI_light_hsl.hsl_hue = 0xaaaa;
            UI_light_hsl.hsl_saturation =  0xffff;
            UI_light_hsl.hsl_lightness = 0x8000 ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        case 6:
        {
            UI_light_hsl.hsl_hue = 0x5555;
            UI_light_hsl.hsl_saturation =  0xffff;
            UI_light_hsl.hsl_lightness = 0x8000 ;
            EM_mem_copy((void*)&(state_p->light_hsl_state), (void*)&UI_light_hsl, sizeof(MS_STATE_LIGHT_HSL_STRUCT));
            UI_generic_onoff.onoff = 1;
            EM_mem_copy((void*)&(state_p->onoff_state), (void*)&UI_generic_onoff, sizeof(MS_STATE_GENERIC_ONOFF_STRUCT));
        }
        break;

        default:
            break;
        }

        #endif //lhj move to hsl
        param_p = (void*)state_p;
    }
    break;

    case MS_SCENE_EVENT_DELETE:
    {
        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("delete scene, index = %d \n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            EM_free_mem(state_p);
            printf("scene %d deleted\n", index);
        }
    }
    break;

    case MS_SCENE_EVENT_RECALL_START:
    {
        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("start recalling scene, index = %d \n", index);
        aliGenie_bleMesh_Generic_Scene_Recall(me,&index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            //state_hsl = &(state_p->light_hsl_state);
            //light_hsl_set_pl(state_hsl->hsl_hue, state_hsl->hsl_saturation, state_hsl->hsl_lightness);
        }
    }
    break;

    case MS_SCENE_EVENT_RECALL_COMPLETE:
    {
        printf("MS_SCENE_EVENT_RECALL_COMPLETE \n");

        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("complete recalling scene, index = %d \n", index);
        state_p = (UI_STATE_SCENE_STRUCT*)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            // nothing to do here
        }
    }
    break;

    case MS_SCENE_EVENT_RECALL_IMMEDIATE:
    {
        printf("MS_SCENE_EVENT_RECALL_IMMEDIATE \n");

        if (event_length != 4)           // UINT32
            return NULL;

        index = *(UINT32*)event_param;
        printf("complete recalling scene, index = %d \n", index);
        aliGenie_bleMesh_Generic_Scene_Recall(me,&index);

        //state_p = (UI_STATE_SCENE_STRUCT *)context;

        if (state_p != NULL
                && (state_p->index == index))
        {
            //    state_hsl = &(state_p->light_hsl_state);
            //    printf("state hsl status, hue: = 0x%04X, saturation: = 0x%04X lightness: = 0x%04X \n", state_hsl->hsl_hue,state_hsl->hsl_saturation,state_hsl->hsl_lightness);
            //  light_hsl_set_pl(state_hsl->hsl_hue, state_hsl->hsl_saturation, state_hsl->hsl_lightness);
        }
    }
    break;
    }

    return param_p;
}

/*

    API_RESULT UI_generic_scene_recall_callback_register(UI_DATA_GENERIC_SCENE_MODEL_T * me, UINT16 scene_num,  void (*scene_recall_callback)(UINT16 UI_DATA_GENERIC_SCENE_MODEL_T * me))
    {
    if (0 == scene_num || NULL == scene_recall_callback || NULL == me)
    {
        ERROR_PRINT("UI_generic_scene_recall_callback_register failied! [INVALID PARAMETER,me = 0x%04X, scene_num = 0x%04X, scene_recall_callback = 0x%04X]!\n",me, scene_num, scene_recall_callback);
        return API_FAILURE;
    }

    for(int i = 0; i < MS_SCENE_NUMBER_MAX; i++ )
    {
        if (me->scenes[i] == 0)
        {
            me->scenes[i] = scene_num;
            me->scene_recall_callback[i] = scene_recall_callback;
            return API_SUCCESS;
        }
        else if (me->scenes[i] == scene_num)
        {

            ERROR_PRINT("UI_generic_scene_recall_callback_register failied! [USED SCENE_NUM, 0x%04X]!\n",scene_num);
            return API_FAILURE;
        }
    }

    ERROR_PRINT("UI_generic_scene_recall_callback_register failed! [NO SPACE]!\n");
    return API_FAILURE;


    }
*/
API_RESULT UI_generic_scene_model_server_create
(
    MS_ACCESS_ELEMENT_HANDLE element_handle,
    UINT8 scene_num,
    DECL_CONST UINT16* scene_list

)
{
    API_RESULT retval;
    UI_DATA_GENERIC_SCENE_MODEL_T* me = NULL;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    MS_ACCESS_MODEL_HANDLE          setup_model_handle;
    me = (UI_DATA_GENERIC_SCENE_MODEL_T* )EM_alloc_mem(sizeof(UI_DATA_GENERIC_SCENE_MODEL_T));

    if(NULL == me)
    {
        ERROR_PRINT("Generic scene Server Model Create failed! [EM_alloc_mem]!\n");
        //EM_free_mem(me);
        return API_FAILURE;
    }

    EM_mem_set(me,0,sizeof(UI_DATA_GENERIC_SCENE_MODEL_T));
    retval = MS_scene_server_init(element_handle,&model_handle,&setup_model_handle,UI_generic_scene_server_cb);

    if (API_SUCCESS != retval)
    {
        ERROR_PRINT("Generic Scene Server Model Create failed. [MS_generic_Scene_server_init]: 0x%04X\n",retval);
        EM_free_mem(me);
        return retval;
    }

    if(FALSE == appl_model_tree_add(element_handle,model_handle,(void*)me))
    {
        ERROR_PRINT("Generic Scene Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    if(FALSE == appl_model_tree_add(element_handle,setup_model_handle,(void*)me))
    {
        ERROR_PRINT("Generic Scene Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, setup_model_handle = 0x%04X\n",element_handle,setup_model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    me->element_handle = element_handle;
    me->model_handle = model_handle;
    me->setup_model_handle = setup_model_handle;
    set_generic_scene_modele_handle(model_handle);

    for(int i = 0; i < scene_num; i++ )
    {
        ms_scene_store(&model_handle,scene_list[i]);
    }

    INFO_PRINT( "Generic Scene Server Model Create Successed. [model_handle]: 0x%04X\n",me->model_handle);
    return API_SUCCESS;
}

