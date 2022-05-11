
#include "aliGenie_appl.h"
#include "aliGenie_appl_Generic.h"


DECL_CONST UINT32 step_resolution[4] = {100, 1000, 10000, 10*60*1000};
UINT8 anticalc_trans_time(UINT32 time_ms, UINT8 step_unit)
{
    UINT8 steps;
    steps =  (UINT8)(time_ms / step_resolution[step_unit]) & 0x3F;
    steps =  steps | ((step_unit<<6)&0x3F);
    DEBUG_PRINT("[ANTI_CALC]time_ms = 0x%02X ,step_unit = 0x%02X, steps = 0x%02X\r\n",time_ms,step_unit,steps);
    return steps;
}

API_RESULT UI_generic_onoff_send_sts_now (MS_NET_ADDR daddr,UI_DATA_GENERIC_ONOFF_MODEL_T* me)
{
    /* IN */ MS_NET_ADDR               saddr;
    /* IN */ MS_SUBNET_HANDLE          subnet_handle;
    /* IN */ MS_APPKEY_HANDLE          appkey_handle;
    /* IN */ UINT8                     ttl;
    /* IN */ UINT32                    opcode;
    UCHAR pdu[1];
    pdu[0] = me->UI_state_generic_onoff.target_onoff;
    API_RESULT               retval;
    MS_ACCESS_ELEMENT_HANDLE element_handle;
    find_element_by_model(me->model_handle,&element_handle);
    MS_access_cm_get_primary_unicast_address(&saddr);
    saddr = saddr + element_handle;
    DEBUG_PRINT("[GENERIC ONOFF]STS NOW, daddr = 0x%04X, saddr = 0x%04X\r\n[PDU]:\r\n",daddr,saddr);
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_GENERIC_ONOFF_STATUS_OPCODE;
    appl_dump_bytes(pdu, 1);
    retval = MS_access_send_pdu
             (
                 saddr,
                 daddr,
                 subnet_handle,
                 appkey_handle,
                 ttl,
                 opcode,
                 pdu,
                 1,
                 MS_FALSE
             );
    return retval;
}
void UI_ACTION_generic_onoff_trans_cb(void* args, UINT16 size)
{
    UI_DATA_GENERIC_ONOFF_MODEL_T* me;
    EM_mem_copy(&me,args,size);
    me->tmrhdl_delay = EM_TIMER_HANDLE_INIT_VAL;
    me->UI_state_generic_onoff.onoff = me->UI_state_generic_onoff.target_onoff;
    UI_HARD_generic_onoff_set(me,me->UI_state_generic_onoff.onoff);
    UI_generic_onoff_send_sts_now(aligenie_addr, me);
}

void UI_ACTION_generic_onoff_delay_cb(void* args, UINT16 size)
{
    UI_DATA_GENERIC_ONOFF_MODEL_T* me;
    //me = (UI_DATA_GENERIC_ONOFF_MODEL_T *)(args);
    EM_mem_copy(&me,args,size);
    me->tmrhdl_delay = EM_TIMER_HANDLE_INIT_VAL;

    if (me->UI_state_generic_onoff.transition_time > 0)
    {
        EM_start_timer (&(me->tmrhdl_trans), me->UI_state_generic_onoff.transition_time, UI_ACTION_generic_onoff_trans_cb, &me, sizeof(UI_DATA_GENERIC_ONOFF_MODEL_T*));
    }
    else
    {
        me->UI_state_generic_onoff.onoff = me->UI_state_generic_onoff.target_onoff;
        UI_HARD_generic_onoff_set(me,me->UI_state_generic_onoff.onoff);
        UI_generic_onoff_send_sts_now(aligenie_addr, me);
    }
}


UINT32 calc_trans_time(UINT8 _oct)
{
    UINT8 steps;
    UINT32 timer;
    steps = (_oct & 0x3F);
    timer =  (steps * step_resolution[((_oct >> 6) & 0x03)]);
    return timer;
}


/* Generic OnOff Model Set Handler */
API_RESULT UI_generic_onoff_model_state_set(UI_DATA_GENERIC_ONOFF_MODEL_T* me, MS_ACCESS_MODEL_REQ_MSG_RAW* msg_raw)
{
    //appl_dump_bytes(msg_raw->data_param,msg_raw->data_len);
    DEBUG_PRINT("[ONOFF_SET] target: 0x%02X\r\n", msg_raw->data_param[0]);
    me->UI_state_generic_onoff.target_onoff     = msg_raw->data_param[0];
    me->UI_state_generic_onoff.onoff = me->UI_state_generic_onoff.target_onoff;
    UI_HARD_generic_onoff_set(me,me->UI_state_generic_onoff.target_onoff);
    UI_generic_onoff_send_sts_now(aligenie_addr, me);
    return API_SUCCESS;
}




/* Generic OnOff Model Get Handler */
API_RESULT UI_generic_onoff_model_state_get(UI_DATA_GENERIC_ONOFF_MODEL_T* me,UINT16 state_t, UINT16 state_inst, UCHAR* pdu, UINT8 direction)
{
    API_RESULT retval;
    retval = API_SUCCESS;

    switch(state_t)
    {
    case MS_STATE_GENERIC_ONOFF_T:
    {
        pdu[0] = me->UI_state_generic_onoff.onoff;
        pdu[1] = me->UI_state_generic_onoff.target_onoff;
        //TODO: GET REMAIN TIME
    }
    break;

    default:
        ERROR_PRINT("[GENERIC ONOFF GET] error msg %d\r\n",MS_STATE_GENERIC_ONOFF_T);
        break;
    }

    return retval;
}


/* Generic OnOff Model Server */
/**
    \brief Server Application Asynchronous Notification Callback.

    \par Description
    Generic_Onoff server calls the registered callback to indicate events occurred to the application.

    \param [in] ctx           Context of message received for a specific model instance.
    \param [in] msg_raw       Uninterpreted/raw received message.
    \param [in] req_type      Requested message type.
    \param [in] state_params  Model specific state parameters.
    \param [in] ext_params    Additional parameters.
*/

API_RESULT UI_generic_onoff_model_server_cb
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_RAW*         msg_raw,
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_T*           req_type,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        state_params,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    UI_DATA_GENERIC_ONOFF_MODEL_T* me;
    volatile MS_ACCESS_MODEL_STATE_PARAMS     current_state_params;
    API_RESULT                       retval = API_SUCCESS;
    MS_ACCESS_ELEMENT_HANDLE        element_handle;
    UCHAR   pdu[4] = {0};
    MS_ACCESS_MODEL_HANDLE   model_handle = ctx->handle;
    MS_NET_ADDR              saddr = ctx->saddr;
    MS_NET_ADDR              daddr = ctx->daddr;
    //MS_access_get_element_handle(ctx->daddr,&element_handle);
    find_element_by_model(model_handle,&element_handle);
    INFO_PRINT("[ONOFF MODEL CB: ctx->handle = 0x%04X, saddr = 0x%04X, daddr = 0x%04X, element_handle = 0x%04X\r\nMSG_RAW.OPCODE = 0x%04X:\r\n]",ctx->handle,saddr,daddr,element_handle,msg_raw->opcode);
    appl_dump_bytes(msg_raw->data_param, msg_raw->data_len);
    aligenie_addr = saddr;
    me = (UI_DATA_GENERIC_ONOFF_MODEL_T*)find_model_private_data(model_handle);

    if( NULL == me)
    {
        ERROR_PRINT("Generic Onoff Server Callback Failed. [find_model_private_data NOT FOUND]");
    }
    else
    {
        if(!MS_IS_UNICAST_ADDR(daddr))
        {
            MS_access_cm_get_primary_unicast_address(&daddr);
            daddr = daddr + element_handle;
        }

        /* Check message type */
        if (MS_ACCESS_MODEL_REQ_MSG_T_SET == req_type->type)
        {
            volatile    MS_ACCESS_MODEL_STATE_PARAMS     current_state_params;
            retval = UI_generic_onoff_model_state_set( me, msg_raw);
            current_state_params.state_type = state_params->state_type;
            current_state_params.state = (MS_STATE_GENERIC_ONOFF_STRUCT*)state_params->state;
        }
        else if (MS_ACCESS_MODEL_REQ_MSG_T_GET == req_type->type)
        {
            UI_generic_onoff_model_state_get(me, state_params->state_type, 0, pdu, 0);
            volatile MS_ACCESS_MODEL_STATE_PARAMS     current_state_params;
            current_state_params.state_type = MS_STATE_GENERIC_ONOFF_T;//state_params->state_type;
            current_state_params.state = pdu;
        }
    }

    /* See if to be acknowledged */
    if (0x01 == req_type->to_be_acked)
    {
        pdu[0] = me->UI_state_generic_onoff.target_onoff;
        MS_NET_ADDR addr=0;
        ctx->saddr = ctx->daddr;
        ctx->daddr = aligenie_addr;

        if(!MS_IS_UNICAST_ADDR(ctx->saddr))
        {
            MS_access_cm_get_primary_unicast_address(&addr);
            DEBUG_PRINT("[GENERIC_ONOFF] replace DST. (%04x -> %04x) \r\n",ctx->saddr,addr);
            ctx->saddr = addr + element_handle;
        }

        CONSOLE_OUT("[GENERIC_ONOFF] Sending Response.daddr = 0x%04X, saddr = 0x%04X\r\n",ctx->daddr,ctx->saddr);

        while(1)
        {
//          for(uint8_t i=0;i<1;i++)
//          {
//              retval = MS_generic_onoff_server_state_update(ctx, &current_state_params,NULL, 0, NULL);
//          }
            if(retval==API_SUCCESS)
            {
                break;
            }
            else
            {
                ERROR_PRINT(
                    "Generic Onoff ack Failed. Result: 0x%04X\n",
                    retval);
            }
        }
    }

    return retval;
}



API_RESULT UI_generic_onoff_model_server_create
(
    MS_ACCESS_ELEMENT_HANDLE element_handle,
    UINT8   io_num,
    UI_DATA_GENERIC_ONOFF_MODEL_T * * pme
)
{
    API_RESULT retval;
    MS_ACCESS_MODEL_HANDLE          model_handle;
    UI_DATA_GENERIC_ONOFF_MODEL_T* me;
    me = (UI_DATA_GENERIC_ONOFF_MODEL_T* )EM_alloc_mem(sizeof(UI_DATA_GENERIC_ONOFF_MODEL_T));

    if(NULL == me)
    {
        ERROR_PRINT("Generic Onoff Server Model Create failed! [EM_alloc_mem]!\n");
        EM_free_mem(me);
        return API_FAILURE;
    }

    EM_mem_set(me,0,sizeof(UI_DATA_GENERIC_ONOFF_MODEL_T));
    retval = MS_generic_onoff_server_init(element_handle,&model_handle,UI_generic_onoff_model_server_cb);

    if (API_SUCCESS != retval)
    {
        ERROR_PRINT("Generic Onoff Server Model Create failed. [MS_generic_onoff_server_init]: 0x%04X\n",retval);
        EM_free_mem(me);
        return retval;
    }

    if(FALSE == appl_model_tree_add(element_handle,model_handle,(void*)me))
    {
        ERROR_PRINT("Generic Onoff Server Model Create failed! [appl_model_tree_add]: element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    me->element_handle = element_handle;
    me->model_handle = model_handle;
    me->io_num = io_num;
    *pme = me;
    INFO_PRINT( "Generic Onoff Server Model Create Successed. [model_handle]: 0x%04X\n",me->model_handle);
    return API_SUCCESS;
}



