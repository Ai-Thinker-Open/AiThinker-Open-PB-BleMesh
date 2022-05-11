

#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"

DECL_CONST UINT8 attr_len_def[10] = {0,1,1,1,2,2,4,4,sizeof(UINT64),6};

extern API_RESULT init_attr_tbl(UI_DATA_ALIGENIE_MODEL_T* private_data);


API_RESULT vendor_aligenie_get_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data);
API_RESULT vendor_aligenie_set_unack_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data);
API_RESULT vendor_aligenie_status_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data)
{
    return API_SUCCESS;
}
API_RESULT vendor_aligenie_indication_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data)
{
    return API_SUCCESS;
}
API_RESULT vendor_aligenie_transparent_msg_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data)
{
    return API_SUCCESS;
}
API_RESULT vendor_aligenie_default_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data)
{
    return API_SUCCESS;
}

typedef struct
{
    UINT32 opcode;
    API_RESULT (*hdl) (UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data);

} VENDOR_ALIGENIE_OPCODE_HDL_T;

void print_attr_tbl(UI_ALIGENIE_ELEMENT_ATTR_T* attr_tbl,UINT8 num)
{
    for(int i = 0; i < num; i++)
    {
        INFO_PRINT("[attr_tbl].attr_code = 0x%04X\t, .attr_type = 0x%02X\t, .attr_val = 0x%08X\n", attr_tbl[i].attr_code, attr_tbl[i].attr_type, attr_tbl[i].attr_val);
    }
}

UINT32 parse_attr_val(UCHAR* data_param,   UINT8 attr_type)
{
    UINT32 val = 0xFFFFFFFF;
    UCHAR* pdu;

    switch(attr_type)
    {
    case BOOL:
    case U8:
    case S8:
        MS_UNPACK_LE_1_BYTE(&val, data_param);
        break;

    case U16:
    case S16:
        MS_UNPACK_LE_2_BYTE(&val, data_param);
        break;

    case U32:
    case S32:
        MS_UNPACK_LE_4_BYTE(&val, data_param);
        break;

    case U64:
        val = (UINT32)EM_alloc_mem(sizeof(UINT64));

        if( 0 == val )
        {
            ERROR_PRINT("parse_attr_val EM_alloc_mem FAILED\r\n");
            return 0xFFFFFFFF;
        }

        MS_UNPACK_LE_8_BYTE((UCHAR*)val, data_param);
        break;

    case PLD6:
        pdu = EM_alloc_mem(6);

        if( NULL == pdu )
        {
            ERROR_PRINT("parse_attr_val EM_alloc_mem FAILED\r\n");
            return 0xFFFFFFFF;
        }

        MS_UNPACK_LE_N_BYTE(pdu, data_param,6);
        val = (UINT32)pdu;
        break;

    default:
        ERROR_PRINT("parse_attr_val UNKNOWN ATTR TYPE (0x%08X)\r\n",attr_type);
        break;
    }

    return val;
}

API_RESULT init_attr(UI_ALIGENIE_ELEMENT_ATTR_T* attr,UINT16 code, UINT32 val, UINT8 type, API_RESULT (*attr_set_action)(UI_DATA_ALIGENIE_MODEL_T* private_data,UINT32 val))
{
    if(NULL == attr)
        return API_FAILURE;

    attr->attr_code = code;
    attr->attr_val = val;
    attr->attr_type = type;
    attr->attr_set_action = attr_set_action;
    return API_SUCCESS;
}

int set_attr(UI_DATA_ALIGENIE_MODEL_T* me, UINT16 attr_code, UCHAR* data_ptr, int* attr_len)
{
    int i = 0;
    DEBUG_PRINT("[model_handle] = 0x%04X, [attr_code] = 0x%04x, me->attr_tbl_num = 0x%02X\n",me->model_handle,attr_code,me->attr_tbl_num);
    //DEBUG_PRINT("[&data_ptr] = 0x%08X,[data_ptr] = 0x%08X\n",data_ptr, *data_ptr);
    *attr_len = 0;

    if(NULL == me)
        return API_FAILURE;

    for(i = 0; i < me->attr_tbl_num; i++)
    {
        DEBUG_PRINT("me->attr_tbl[%d].attr_code = 0x%04X,attr_code = 0x%04x\n",i,me->attr_tbl[i].attr_code,attr_code);

        if(me->attr_tbl[i].attr_code == attr_code)
        {
            me->attr_tbl[i].attr_val = parse_attr_val(data_ptr,me->attr_tbl[i].attr_type);
            *attr_len = attr_len_def[me->attr_tbl[i].attr_type];
            //DEBUG_PRINT("[me->attr_tbl[i].attr_val] = 0x%08X,[me->attr_tbl[i].attr_val] = 0x%04X, i = %d,attr_len = %d\n",me->attr_tbl[i].attr_val, *((UINT16*)(me->attr_tbl[i].attr_val)),i,*attr_len);
            break;
        }
    }

    if( i >= me->attr_tbl_num)
    {
        ERROR_PRINT("ATTR_SET: ATTR NOT FOUND, ATTR_CODE = 0x%04X, i=%d, num=%d\n", attr_code,i,me->attr_tbl_num);
        return API_FAILURE;
    }

    if( NULL != me->attr_tbl[i].attr_set_action)
    {
        //DEBUG_PRINT("[attr_set_action] i = 0x%08X\n",i);
        me->attr_tbl[i].attr_set_action(me,me->attr_tbl[i].attr_val);
    }

    print_attr_tbl(me->attr_tbl,me->attr_tbl_num);
    return i;
}
int get_attr(UI_DATA_ALIGENIE_MODEL_T* me, UINT16 attr_code, UINT32* val, int* val_len)
{
    int i = 0;
    DEBUG_PRINT("[attr_code] = 0x%04x\n",attr_code);
    *val = 0;
    *val_len = 0;

    if(NULL == me)
        return API_FAILURE;

    if(NULL == val)
        return API_FAILURE;

    for(i = 0; i <me->attr_tbl_num; i++)
    {
        if(me->attr_tbl[i].attr_code == attr_code)
        {
            *val = me->attr_tbl[i].attr_val;
            *val_len = (int)attr_len_def[me->attr_tbl[i].attr_type];
            break;
        }
    }

    if( i >= me->attr_tbl_num)
    {
        ERROR_PRINT("ATTR_GET: ATTR NOT FOUND, ATTR_CODE = 0x%04X\n", attr_code);
        return API_FAILURE;
    }

    return i;
}


void make_attr_pdu(UCHAR* rpl_data, UINT16 attr_code, UINT32 attr_val, int attr_len)
{
    MS_PACK_LE_2_BYTE_VAL(rpl_data,attr_code);

    switch(attr_len)
    {
    case 1:
        MS_PACK_LE_1_BYTE_VAL(rpl_data+2,(UINT8)(attr_val & 0x000000FF));
        break;

    case 2:
        MS_PACK_LE_2_BYTE_VAL(rpl_data+2,(UINT16)(attr_val & 0x0000FFFF));
        break;

    case 4:
        MS_PACK_LE_4_BYTE_VAL(rpl_data+2,attr_val);
        break;

    case sizeof(UINT64):
        MS_PACK_LE_8_BYTE(rpl_data+2,(UINT64*)attr_val);
        break;

    case 6:
        MS_PACK_LE_N_BYTE(rpl_data+2,(UCHAR*)attr_val,6);
        break;

    default:
        ERROR_PRINT("[VENDOR STS RSP] DATA LEN ERROR, attr_code = 0x%08x, attr_len = 0x%04X\r\n]", attr_code, attr_len);
        return;
    }

    return;
}


API_RESULT vendor_aligenie_set_unack_handler(UI_DATA_ALIGENIE_MODEL_T* me, MS_CALLBACK_DATA_T* ms_callback_data)
{
    int i = -1;
    UCHAR*  data_ptr;
    UINT16  attr_code = 0;
    int     attr_len = 0;
    data_ptr = ms_callback_data->data_param + 1;

    while(data_ptr < ms_callback_data->data_param + ms_callback_data->data_len)
    {
        DEBUG_PRINT("data_ptr = 0x%08x, para_end = 0x%08x\n",data_ptr, ms_callback_data->data_param + ms_callback_data->data_len);
        MS_UNPACK_LE_2_BYTE(&attr_code, data_ptr);
        data_ptr += 2;
        i = set_attr(me,attr_code,data_ptr,&attr_len);

        if(i < 0 || attr_len <= 0 || attr_len > 4)
        {
            ERROR_PRINT("[VENDOR ATTR[%d] SET ERROR, attr_code = 0x%08x, attr_len = 0x%04X\r\n]", i,attr_code, attr_len);
            return API_FAILURE;
        }

        data_ptr += attr_len;
    }

    return API_SUCCESS;
}

API_RESULT vendor_aligenie_set_handler(UI_DATA_ALIGENIE_MODEL_T* me, MS_CALLBACK_DATA_T* ms_callback_data)
{
    int i = -1;
    API_RESULT retval;
    UCHAR*  data_ptr;
    UINT16  attr_code = 0;
    UCHAR   rpl_data[15*4] = {0};
    UINT8   rpl_data_len = 0;
    int     attr_len = 0;
    UINT32  opcode=MS_ACCESS_VENDOR_ALIGENIE_STATUS;
    data_ptr = ms_callback_data->data_param + 1;
    rpl_data[0] = ++vendor_msg_tid;
    rpl_data_len = 1;

    while(data_ptr < ms_callback_data->data_param + ms_callback_data->data_len - 1)
    {
        //DEBUG_PRINT("data_ptr = 0x%08x, para_end = 0x%08x\n",data_ptr, ms_callback_data->data_param + ms_callback_data->data_len);
        MS_UNPACK_LE_2_BYTE(&attr_code, data_ptr);
        data_ptr += 2;
        i = set_attr(me,attr_code,data_ptr,&attr_len);

        if(i < 0 || attr_len <= 0 || attr_len > sizeof(UINT64) )
        {
            ERROR_PRINT("[VENDOR ATTR[%d] SET ERROR, attr_code = 0x%08x, attr_len = 0x%04X\r\n]", i,attr_code, attr_len);
            return API_FAILURE;
        }

        data_ptr += attr_len;
        make_attr_pdu(rpl_data+rpl_data_len, attr_code, me->attr_tbl[i].attr_val, attr_len);
        rpl_data_len += attr_len+2;
    }

    DEBUG_PRINT("[SET RSP] %d========\n",rpl_data_len);
    appl_dump_bytes(rpl_data, rpl_data_len);
    retval = MS_access_reply
             (
                 ms_callback_data->handle,
                 ms_callback_data->daddr,
                 ms_callback_data->saddr,
                 ms_callback_data->subnet_handle,
                 ms_callback_data->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 rpl_data,
                 rpl_data_len
             );
    return retval;
}

API_RESULT vendor_aligenie_get_handler(UI_DATA_ALIGENIE_MODEL_T* me, MS_CALLBACK_DATA_T* ms_callback_data)
{
    int i = -1;
    API_RESULT retval;
    UCHAR*  data_ptr;
    UINT16  attr_code = 0;
    UCHAR   rpl_data[RPL_MAX_LEN] = {0};
    UINT8   rpl_data_len = 0;
    int     attr_len = 0;
    UINT32  val = 0;
    UINT32  opcode=MS_ACCESS_VENDOR_ALIGENIE_STATUS;
    data_ptr = ms_callback_data->data_param + 1;
    rpl_data[0] = ++vendor_msg_tid;
    rpl_data_len = 1;

    while(data_ptr < ms_callback_data->data_param + ms_callback_data->data_len)
    {
        if(rpl_data_len >= RPL_MAX_LEN)
            ERROR_PRINT("[GET RSP] TOO LONG PAYLOAD\r\n]");

        MS_UNPACK_LE_2_BYTE(&attr_code, data_ptr);
        data_ptr += 2;
        i = get_attr(me,attr_code, &val, &attr_len);

        if(i < 0 || attr_len < 1 || attr_len > sizeof(UINT64))
        {
            ERROR_PRINT("[VENDOR ATTR[%d] SET ERROR, attr_code = 0x%08x, attr_len = 0x%04X\r\n]", i,attr_code, attr_len);
            return API_FAILURE;
        }

        make_attr_pdu(rpl_data+rpl_data_len, attr_code, val, attr_len);
        rpl_data_len += attr_len+2;
    }

    DEBUG_PRINT("\r\n[GET RSP] %d========\n",rpl_data_len);
    appl_dump_bytes(rpl_data, rpl_data_len);
    retval = MS_access_reply
             (
                 ms_callback_data->handle,
                 ms_callback_data->daddr,
                 ms_callback_data->saddr,
                 ms_callback_data->subnet_handle,
                 ms_callback_data->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 rpl_data,
                 rpl_data_len
             );
    return retval;
}


API_RESULT vendor_aligenie_confirmation_handler(UI_DATA_ALIGENIE_MODEL_T* private_data, MS_CALLBACK_DATA_T* ms_callback_data)
{
    //UI_DATA_ALIGENIE_MODEL_T * me = private_data;
    INFO_PRINT("[RECEIVED VENDOR CONFIRM] MODEL = 0x%08X, SADDR = 0x%04X, DADDR = 0x%04X\n",*(ms_callback_data->handle),ms_callback_data->saddr,ms_callback_data->daddr);
    return API_SUCCESS;
}


#define VENDOR_ALIGENIE_OPCODE_HDL_CNT  7

DECL_CONST VENDOR_ALIGENIE_OPCODE_HDL_T vendor_aligenie_opcode_hdl_tbl[VENDOR_ALIGENIE_OPCODE_HDL_CNT] =
{
    {MS_ACCESS_VENDOR_ALIGENIE_GET_OPCODE,vendor_aligenie_get_handler},
    {MS_ACCESS_VENDOR_ALIGENIE_SET_OPCODE,vendor_aligenie_set_handler},
    {MS_ACCESS_VENDOR_ALIGENIE_SET_UNACKNOWLEDGED_OPCODE,vendor_aligenie_set_unack_handler},
    {MS_ACCESS_VENDOR_ALIGENIE_STATUS,vendor_aligenie_status_handler},
    {MS_ACCESS_VENDOR_ALIGENIE_INDICATION,vendor_aligenie_indication_handler},
    {MS_ACCESS_VENDOR_ALIGENIE_CONFIRMATION,vendor_aligenie_confirmation_handler},
    {MS_ACCESS_VENDOR_ALIGENIE_TRANSPARENT_MSG,vendor_aligenie_transparent_msg_handler}
};



DECL_CONST UINT32 VENDOR_ALIGENIE_server_opcode_list[VENDOR_ALIGENIE_OPCODE_HDL_CNT] =
{
    MS_ACCESS_VENDOR_ALIGENIE_GET_OPCODE,
    MS_ACCESS_VENDOR_ALIGENIE_SET_OPCODE,
    MS_ACCESS_VENDOR_ALIGENIE_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_VENDOR_ALIGENIE_STATUS,
    MS_ACCESS_VENDOR_ALIGENIE_INDICATION,
    MS_ACCESS_VENDOR_ALIGENIE_CONFIRMATION,
    MS_ACCESS_VENDOR_ALIGENIE_TRANSPARENT_MSG
};




static API_RESULT VENDOR_ALIGENIE_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
);



/* ----------------------------------------- Functions */
/**
    \brief API to send reply or to update state change

    \par Description
    This is to send reply for a request or to inform change in state.

    \param [in] ctx                     Context of the message.
    \param [in] current_state_params    Model specific current state parameters.
    \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
    \param [in] remaining_time          Time from current state to target state (0: to be ignored).
    \param [in] ext_params              Additional parameters (NULL: to be ignored).

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_VENDOR_ALIGENIE_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_VENDOR_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params,
    /* IN */ UINT8       count
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    UINT32     opcode;
    retval = API_FAILURE;
    opcode=MS_ACCESS_VENDOR_ALIGENIE_STATUS;
    marker = 0;
    CONSOLE_OUT(
        "[VENDOR_ALIGENIE_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    }

    /* Publish - reliable */
    // if (0 == marker)
    // {
    //  pdu_ptr = NULL;
    // }
    //else
    {
        marker=count;
        memcpy(buffer,current_state_params->state,count);
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
    return retval;
}


/* Vendor Defined Model state Initialization */
void UI_vendor_defined_model_states_initialization(void)
{
    /* Vendor Defined States */
    //EM_mem_set (&UI_vendor_example, 0, sizeof(UI_vendor_example));
}

typedef struct
{
    UINT16 onoff;
} MS_STATE_VENDOR_ONOFF_DATA_T;


/*********####### BELOW NEED NOT TO BE CHANGED #####**********/

/**
    \brief Access Layer Application Asynchronous Notification Callback.

    \par Description
    Access Layer calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] appkey_handle AppKey Handle.
    \param [in] subnet_handle Subnet Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT VENDOR_ALIGENIE_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    UI_DATA_ALIGENIE_MODEL_T* me;
    MS_CALLBACK_DATA_T ms_callback_data;
    API_RESULT    retval;
    retval = API_SUCCESS;
//   ext_params_p = NULL;
    aligenie_addr = saddr;
    me = (UI_DATA_ALIGENIE_MODEL_T*)find_model_private_data(*handle);

    if( NULL == me)
    {
        ERROR_PRINT("ALIGENIE Server Callback Failed. [find_model_private_data NOT FOUND]");
    }

    ms_callback_data.handle        =  handle;
    ms_callback_data.saddr         =  saddr;
    ms_callback_data.daddr         =  daddr;
    ms_callback_data.subnet_handle =  subnet_handle;
    ms_callback_data.appkey_handle =  appkey_handle;
    ms_callback_data.opcode        =  opcode;
    ms_callback_data.data_param    =  data_param;
    ms_callback_data.data_len      =  data_len;
    /* Request Context */
//   req_context.handle = *handle;
//   req_context.saddr  = saddr;
//   req_context.daddr  = daddr;
//   req_context.subnet_handle = subnet_handle;
//   req_context.appkey_handle = appkey_handle;
    /* Request Raw */
//   req_raw.opcode = opcode;
//   req_raw.data_param = data_param;
//   req_raw.data_len = data_len;
    DEBUG_PRINT("[VENDOR_ALIGENIE_SERVER] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);
    UCHAR i;

    for( i = 0; i < VENDOR_ALIGENIE_OPCODE_HDL_CNT; i++)
    {
        if(opcode == vendor_aligenie_opcode_hdl_tbl[i].opcode)
        {
            if(NULL != vendor_aligenie_opcode_hdl_tbl[i].hdl)
            {
                vendor_aligenie_opcode_hdl_tbl[i].hdl(me,&ms_callback_data);
                break;
            }
        }
    }

    if(i>= VENDOR_ALIGENIE_OPCODE_HDL_CNT)
    {
        vendor_aligenie_default_handler(me,&ms_callback_data);
    }

    /* Application callback */
//  if (NULL != me->UI_cb)
//  {
    //me->UI_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
//  }
    return retval;
}

API_RESULT (*pinit_attr_tbl)(UI_DATA_ALIGENIE_MODEL_T* me) = NULL;

API_RESULT UI_vendor_aliGenie_model_server_create
(
    MS_ACCESS_NODE_ID        node_id,
    MS_ACCESS_ELEMENT_HANDLE element_handle,
    UI_DATA_GENERIC_ONOFF_MODEL_T* generic_onoff_me
)
{
    API_RESULT retval;
    UI_DATA_ALIGENIE_MODEL_T*  me = NULL;
    MS_ACCESS_MODEL_HANDLE      model_handle;
    MS_ACCESS_MODEL             model;
    me = (UI_DATA_ALIGENIE_MODEL_T* )EM_alloc_mem(sizeof(UI_DATA_ALIGENIE_MODEL_T));

    if(NULL == me)
    {
        ERROR_PRINT("ALIGENIE Server Model Create failed! [EM_alloc_mem]!\n");
        EM_free_mem(me);
        return API_FAILURE;
    }

    EM_mem_set(me,0,sizeof(UI_DATA_ALIGENIE_MODEL_T));
    /* Configure Model */
    EM_mem_set(&model,0,sizeof(MS_ACCESS_MODEL));
    model.model_id.id = MS_MODEL_ID_VENDOR_ALIGENIE_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
    model.elem_handle = element_handle;
    /* List of Opcodes */
    model.opcodes = VENDOR_ALIGENIE_server_opcode_list;
    model.num_opcodes = sizeof(VENDOR_ALIGENIE_server_opcode_list) / sizeof(UINT32);
    /* Register Callback */
    model.cb = VENDOR_ALIGENIE_server_cb;
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 &model_handle
             );

    if (API_SUCCESS != retval)
    {
        ERROR_PRINT("ALIGENIE Server Model Create failed. [MS_access_register_model return]: 0x%04X, model.elem_handle = 0x%04X\n",retval, model.elem_handle);
        EM_free_mem(me);
        return retval;
    }

    me->element_handle = element_handle;
    me->model_handle = model_handle;
    me->io_num = generic_onoff_me->io_num;
    me->generic_onoff_me = generic_onoff_me;
    generic_onoff_me->vendor_me = me;

    if(FALSE == appl_model_tree_add(element_handle,model_handle,(void*)me))
    {
        ERROR_PRINT("ALIGENIE Server Model Create failed! [appl_model_tree_add return]: element_handle = 0x%02X, model_handle = 0x%04X\n",element_handle,model_handle);
        EM_free_mem(me);
        return API_FAILURE;
    }

    if(NULL != pinit_attr_tbl)
    {
        if(API_SUCCESS != (*pinit_attr_tbl)(me))
        {
            ERROR_PRINT("ALIGENIE ATTR INIT failed! [EM_alloc_mem]!\n");
            EM_free_mem(me->attr_tbl);
            EM_free_mem(me);
            return API_FAILURE;
        }
    }

    INFO_PRINT( "ALIGENIE Vendor Server Model Create Successed. [model_handle]: 0x%04X\n",me->model_handle);
    return API_SUCCESS;
}



