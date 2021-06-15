/**
 * \file sensor_server.c
 */

/*
 * Copyright (C) 2019. PHYPLUSINC.
 * All rights reserved.
 */



/* --------------------------------------------- Header File Inclusion */
#include "sensor_server.h"
#include "MS_model_states.h"
#include "MS_generic_default_transition_time_api.h"

/* --------------------------------------------- Global Definitions */
/** -- Sensor  Defined States */


uint8 vol_raw_value_x0[VOL_VALUE_LEN];
uint8 vol_column_w0[VOL_VALUE_LEN];
uint8 vol_raw_value_y0[VOL_VALUE_LEN];

uint8 vol_raw_value_x1[VOL_VALUE_LEN];
uint8 vol_column_w1[VOL_VALUE_LEN];
uint8 vol_raw_value_y1[VOL_VALUE_LEN];

uint8 vol_raw_value_x2[VOL_VALUE_LEN];
uint8 vol_column_w2[VOL_VALUE_LEN];
uint8 vol_raw_value_y2[VOL_VALUE_LEN];


uint8 humity_raw_value_x0[HUMITY_VALUE_LEN];
uint8 humity_column_w0[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y0[HUMITY_VALUE_LEN];

uint8 humity_raw_value_x1[HUMITY_VALUE_LEN];
uint8 humity_column_w1[HUMITY_VALUE_LEN];
uint8 humity_raw_value_y1[HUMITY_VALUE_LEN];


uint8 temperature_raw_value_x0[HUMITY_VALUE_LEN];
uint8 temperature_column_w0[HUMITY_VALUE_LEN];
uint8 temperature_raw_value_y0[HUMITY_VALUE_LEN];

uint8 temperature_raw_value_x1[HUMITY_VALUE_LEN];
uint8 temperature_column_w1[HUMITY_VALUE_LEN];
uint8 temperature_raw_value_y1[HUMITY_VALUE_LEN];



uint8 ACC_raw_value_x0[ACC_VALUE_LEN];
uint8 ACC_column_w0[ACC_VALUE_LEN];
uint8 ACC_raw_value_y0[ACC_VALUE_LEN];

uint8 ACC_raw_value_x1[ACC_VALUE_LEN];
uint8 ACC_column_w1[ACC_VALUE_LEN];
uint8 ACC_raw_value_y1[ACC_VALUE_LEN];


uint8 ACC_raw_value_x2[ACC_VALUE_LEN];
uint8 ACC_column_w2[ACC_VALUE_LEN];
uint8 ACC_raw_value_y2[ACC_VALUE_LEN];



MS_STATE_SENSOR_DESCRIPTOR_STRUCT UI_Sensor_Descrip_Infor[SENSOR_NUM]=
{

//VOLTAGE PID
    {
        VOLTAGE_PID,        //sensor_property_id
        0x1234,             //sensor_positive_tolerance
        0x1235,             //sensor_negative_tolerance
        0xff,               //sensor_sampling_function
        0xaa,               //sensor_measurement_period
        0xff,               //sensor_update_interval
        0                   //status
    },
//ACCELERO_METER_PID
    {
        ACCELERO_METER_PID,	    //sensor_property_id
        0x0000, 			    //sensor_positive_tolerance
        0x0000, 			    //sensor_negative_tolerance
        0x00,				    //sensor_sampling_function
        0x00,				    //sensor_measurement_period
        0x00,				    //sensor_update_interval
        0 					    //status
    },

//HUMIDITY_PID
    {
        HUMIDITY_PID,	    //sensor_property_id
        0x5555, 			//sensor_positive_tolerance
        0x5555, 			//sensor_negative_tolerance
        0x66,				//sensor_sampling_function
        0x01,				//sensor_measurement_period
        0x22,				//sensor_update_interval
        0 					//status
    }
		
		//TEMPERATURE_PID
    {
        TEMPERATURE_PID,	    //sensor_property_id
        0x5555, 			//sensor_positive_tolerance
        0x5555, 			//sensor_negative_tolerance
        0x66,				//sensor_sampling_function
        0x01,				//sensor_measurement_period
        0x22,				//sensor_update_interval
        0 					//status
    }
};







MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Vol_Sensor_series_column[VOL_COLUMN_NUM]=
{
    {
        VOLTAGE_PID,
        vol_raw_value_x0,
        sizeof(vol_raw_value_x0),

        vol_column_w0,
        sizeof(vol_column_w0),

        vol_raw_value_y0,
        sizeof(vol_raw_value_y0),

        0
    },

    {
        VOLTAGE_PID,
        vol_raw_value_x1,
        sizeof(vol_raw_value_x1),

        vol_column_w1,
        sizeof(vol_column_w1),

        vol_raw_value_y1,
        sizeof(vol_raw_value_y1),

        0
    },

    {
        VOLTAGE_PID,
        vol_raw_value_x2,
        sizeof(vol_raw_value_x2),

        vol_column_w2,
        sizeof(vol_column_w2),

        vol_raw_value_y2,
        sizeof(vol_raw_value_y2),

        0
    }


};




MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Humity_Sensor_series_column[HUMITY_COLUMN_NUM]=
{
    {
        HUMIDITY_PID,
        humity_raw_value_x0,
        sizeof(humity_raw_value_x0),

        humity_column_w0,
        sizeof(humity_column_w0),

        humity_raw_value_y0,
        sizeof(humity_raw_value_y0),

        0
    },

    {
        HUMIDITY_PID,
        humity_raw_value_x1,
        sizeof(humity_raw_value_x1),

        humity_column_w1,
        sizeof(humity_column_w1),

        humity_raw_value_y1,
        sizeof(humity_raw_value_y1),

        0
    },

};


MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Temperature_Sensor_series_column[TEMPERATURE_COLUMN_NUM]=
{
    {
        TEMPERATURE_PID,
        temperature_raw_value_x0,
        sizeof(temperature_raw_value_x0),

        temperature_column_w0,
        sizeof(temperature_column_w0),

        temperature_raw_value_y0,
        sizeof(temperature_raw_value_y0),

        0
    },

    {
        TEMPERATURE_PID,
        temperature_raw_value_x1,
        sizeof(temperature_raw_value_x1),

        temperature_column_w1,
        sizeof(temperature_column_w1),

        temperature_raw_value_y1,
        sizeof(temperature_raw_value_y1),

        0
    },

};







MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_ACC_Sensor_series_column[ACC_COLUMN_NUM]=
{
    {
        ACCELERO_METER_PID,
        ACC_raw_value_x0,
        sizeof(ACC_raw_value_x0),

        ACC_column_w0,
        sizeof(ACC_column_w0),

        ACC_raw_value_y0,
        sizeof(ACC_raw_value_y0),

        0
    },

    {
        ACCELERO_METER_PID,
        ACC_raw_value_x1,
        sizeof(ACC_raw_value_x1),

        ACC_column_w1,
        sizeof(ACC_column_w1),

        ACC_raw_value_y1,
        sizeof(ACC_raw_value_y1),

        0
    },

   {
        ACCELERO_METER_PID,
        ACC_raw_value_x1,
        sizeof(ACC_raw_value_x1),

        ACC_column_w1,
        sizeof(ACC_column_w1),

        ACC_raw_value_y1,
        sizeof(ACC_raw_value_y1),

        0
    }

};

//===========================cadence setting start===========================

uint8 voltage_trigger_delta_down=50; //50mv
uint8 voltage_trigger_delta_up=50;   //50mv
uint8 voltage_fast_cadence_low[VOL_CADENCE_LEN]={(uint8)(2800&0XFF),(uint8)(2800>>8)};     //2800mv
uint8 voltage_fast_cadence_high[VOL_CADENCE_LEN]={(uint8)(4300&0XFF),(uint8)(4300>>8)};   //4300mv


uint8 humity_trigger_delta_down=10; //10--1%
uint8 humity_trigger_delta_up=10; //10--1%
uint8 humity_fast_cadence_low[HUMITY_CADENCE_LEN]={0};
uint8 humity_fast_cadence_high[HUMITY_CADENCE_LEN]={0};


uint8 temperature_trigger_delta_down=80;
uint8 temperature_trigger_delta_up=80;
uint8 temperature_fast_cadence_low[TEMPERATURE_CADENCE_LEN]={0};
uint8 temperature_fast_cadence_high[TEMPERATURE_CADENCE_LEN]={0};


MS_STATE_SENSOR_CADENCE_STRUCT Sensor_CadenceSetting[SENSOR_CADENCE_NUMBER]=
{
  {
   VOLTAGE_PID,                            //property id
   0X02,                                   //divisor
   1,                                      //trigger type
   &voltage_trigger_delta_down,            //trigger delta down
   sizeof(voltage_trigger_delta_down),     //trigger delta down len
   &voltage_trigger_delta_up,              //trigger delta up
   sizeof(voltage_trigger_delta_up),       //trigger delta up len
   0,                                      // status minimum intervel
   &voltage_fast_cadence_low[0],              // fast cadence low
   VOL_CADENCE_LEN,                        // fast cadence low len
   &voltage_fast_cadence_high[0],             // fast cadence high 
   VOL_CADENCE_LEN,                        // fast cadence high len
   0                                       // not use
  },
  {
   HUMIDITY_PID,                             //property id
   0X02,                                     //divisor
   1,                                        //trigger type
   &humity_trigger_delta_down,               //trigger delta down
   sizeof(humity_trigger_delta_down),        //trigger delta down len
   &humity_trigger_delta_up,                 //trigger delta up
   sizeof(humity_trigger_delta_up),          //trigger delta up len
   0,                                        // status minimum intervel
   &humity_fast_cadence_low[0],              // fast cadence low
   HUMITY_CADENCE_LEN,                       // fast cadence low len
   &humity_fast_cadence_high[0],             // fast cadence high 
   HUMITY_CADENCE_LEN,                       // fast cadence high len
   0                                         // not use
  },
	 {
   TEMPERATURE_PID,                             //property id
   0X02,                                     //divisor
   1,                                        //trigger type
   &temperature_trigger_delta_down,               //trigger delta down
   sizeof(temperature_trigger_delta_down),        //trigger delta down len
   &temperature_trigger_delta_up,                 //trigger delta up
   sizeof(temperature_trigger_delta_up),          //trigger delta up len
   0,                                        // status minimum intervel
   &temperature_fast_cadence_low[0],              // fast cadence low
   TEMPERATURE_CADENCE_LEN,                       // fast cadence low len
   &temperature_fast_cadence_high[0],             // fast cadence high 
   TEMPERATURE_CADENCE_LEN,                       // fast cadence high len
   0                                         // not use
  }
};


//===========================cadence setting end================================

//===========================settings && setting start ===========================

uint8 humity_setting_raw0=0x55;
uint8 humity_setting_raw1=0xAA;
static uint16 settings_num[SENSOR_SETTINGS_NUM]=
{
 0x0001,
 0x0002
};


MS_STATE_SENSOR_SETTING_STRUCT humity_sensor_setting[2]=
{
 {
  HUMIDITY_PID,
  0x0001,
  Sensor_Access_read,
  &humity_setting_raw0,//just for testing
  1,
  0
 },
 {
  HUMIDITY_PID,
   0x0002,
  Sensor_Access_read,
  &humity_setting_raw1,//just for testing
  1,
  0
 }
};

MS_STATE_SENSOR_SETTINGS_STRUCT humity_sensor_settings=
{
 HUMIDITY_PID,
 settings_num,
 SENSOR_SETTINGS_NUM
};







//===========================settings && setting end ===========================




/* --------------------------------------------- Static Global Variables */

static DECL_CONST UINT32 sensor_server_opcode_list[] =
{
    MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE,
    MS_ACCESS_SENSOR_DESCRIPTOR_STATUS_OPCODE,
    MS_ACCESS_SENSOR_GET_OPCODE,
    MS_ACCESS_SENSOR_STATUS_OPCODE,
    MS_ACCESS_SENSOR_COLUMN_GET_OPCODE,
    MS_ACCESS_SENSOR_COLUMN_STATUS_OPCODE,
    MS_ACCESS_SENSOR_SERIES_GET_OPCODE,
    MS_ACCESS_SENSOR_SERIES_STATUS_OPCODE
};

static DECL_CONST UINT32 sensor_setup_server_opcode_list[] =
{
    MS_ACCESS_SENSOR_CADENCE_GET_OPCODE,
    MS_ACCESS_SENSOR_CADENCE_SET_OPCODE,
    MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SENSOR_CADENCE_STATUS_OPCODE,
    MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE,
    MS_ACCESS_SENSOR_SETTINGS_STATUS_OPCODE,
    MS_ACCESS_SENSOR_SETTING_GET_OPCODE,
    MS_ACCESS_SENSOR_SETTING_SET_OPCODE,
    MS_ACCESS_SENSOR_SETTING_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SENSOR_SETTING_STATUS_OPCODE
};

static MS_SENSOR_MODEL_SERVER_CB       Sensor_server_UI_cb;
static MS_SENSOR_MODEL_SERVER_CB       Sensor_setup_server_UI_cb;






/* --------------------------------------------- External Global Variables */

extern void appl_dump_bytes(UCHAR * buffer, UINT16 length);


/* --------------------------------------------- Function */



/**
* \brief Access Layer Application Asynchronous Notification Callback.
*
* \par Description
* Access Layer calls the registered callback to indicate events occurred to the application.
*
* \param [in] handle		Model Handle.
* \param [in] saddr 		16 bit Source Address.
* \param [in] daddr 		16 bit Destination Address.
* \param [in] appkey_handle AppKey Handle.
* \param [in] subnet_handle Subnet Handle.
* \param [in] opcode		Opcode.
* \param [in] data_param	Data associated with the event if any or NULL.
* \param [in] data_len		Size of the event data. 0 if event data is NULL.
*/
API_RESULT Sensor_setup_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE * handle,
    /* IN */ MS_NET_ADDR 			 saddr,
    /* IN */ MS_NET_ADDR 			 daddr,
    /* IN */ MS_SUBNET_HANDLE		 subnet_handle,
    /* IN */ MS_APPKEY_HANDLE		 appkey_handle,
    /* IN */ UINT32					 opcode,
    /* IN */ UCHAR				   * data_param,
    /* IN */ UINT16					 data_len
)
{
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT		   req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW			   req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T			   req_type;

    MS_ACCESS_MODEL_EXT_PARAMS			   * ext_params_p;

    MS_ACCESS_SENSOR_MODEL_STATE_PARAMS	   state_params;
  

    API_RESULT	 retval;

    retval = API_SUCCESS;
    ext_params_p = NULL;

    /* Request Context */
    req_context.handle = *handle;
    req_context.saddr  = saddr;
    req_context.daddr  = daddr;
    req_context.subnet_handle = subnet_handle;
    req_context.appkey_handle = appkey_handle;

    /* Request Raw */
    req_raw.opcode = opcode;
    req_raw.data_param = data_param;
    req_raw.data_len = data_len;

    SENSOR_SERVER_TRC(
        "[SENSOR_SETUP_SERVER] Callback. Opcode 0x%06X\n", opcode);

    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_SENSOR_CADENCE_GET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_CADENCE_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_candece_get_handler);

        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

		state_params.state_type=MS_STATE_SENSOR_CADENCE_T;

    }
    break;
	case MS_ACCESS_SENSOR_CADENCE_SET_OPCODE:
	{
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_CADENCE_SET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_candece_set_handler);

        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;

		state_params.state_type=MS_STATE_SENSOR_CADENCE_T;

	}
	break;
	case MS_ACCESS_SENSOR_CADENCE_SET_UNACKNOWLEDGED_OPCODE:
	{
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_CADENCE_SET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_candece_set_unacknowledged_handler);

        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x00;

		state_params.state_type=MS_STATE_SENSOR_CADENCE_T;
 
	}
	break;
    case MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_SETTINGS_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_settings_get_handler);

        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

		state_params.state_type=MS_STATE_SENSOR_SETTINGS_T;
		
    }
    break;
    case MS_ACCESS_SENSOR_SETTING_GET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_SETTING_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_setting_get_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

		state_params.state_type=MS_STATE_SENSOR_SETTING_T;
    }
    break;
    case MS_ACCESS_SENSOR_SETTING_SET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_SETTING_STATUS_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_setting_status_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;

		state_params.state_type=MS_STATE_SENSOR_SETTING_T;
       
    }
    break;
	case MS_ACCESS_SENSOR_SETTING_SET_UNACKNOWLEDGED_OPCODE:
    {
       SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_SETTING_SET_UNACKNOWLEDGED_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_setting_set_unacknowledged_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x00;

		state_params.state_type=MS_STATE_SENSOR_SETTING_T;
	}
	break;

    default:
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_NONE_OPCODE\n");
        break;


    }

    /* Application callback */
    if (NULL != Sensor_setup_server_UI_cb)
    {
        Sensor_setup_server_UI_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
    }

    return retval;
}






/**
 * \brief Access Layer Application Asynchronous Notification Callback.
 *
 * \par Description
 * Access Layer calls the registered callback to indicate events occurred to the application.
 *
 * \param [in] handle        Model Handle.
 * \param [in] saddr         16 bit Source Address.
 * \param [in] daddr         16 bit Destination Address.
 * \param [in] appkey_handle AppKey Handle.
 * \param [in] subnet_handle Subnet Handle.
 * \param [in] opcode        Opcode.
 * \param [in] data_param    Data associated with the event if any or NULL.
 * \param [in] data_len      Size of the event data. 0 if event data is NULL.
 */
API_RESULT Sensor_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE * handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR                  * data_param,
    /* IN */ UINT16                   data_len
)
{
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT         req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW             req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T               req_type;

    MS_ACCESS_MODEL_EXT_PARAMS              * ext_params_p;

    MS_ACCESS_SENSOR_MODEL_STATE_PARAMS     state_params;
    uint16  marker = 0;

//    UINT16        marker;
    API_RESULT    retval;

    retval = API_SUCCESS;
    ext_params_p = NULL;

    /* Request Context */
    req_context.handle = *handle;
    req_context.saddr  = saddr;
    req_context.daddr  = daddr;
    req_context.subnet_handle = subnet_handle;
    req_context.appkey_handle = appkey_handle;

    /* Request Raw */
    req_raw.opcode = opcode;
    req_raw.data_param = data_param;
    req_raw.data_len = data_len;

    SENSOR_SERVER_TRC(
        "[SENSOR_SERVER] Callback. Opcode 0x%06X\n", opcode);

    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_SENSOR_DESCRIPTOR_GET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_VENDOR_EXAMPLE_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_descr_get_handler);

        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

        if(data_len==0)
            state_params.state_type=MS_STATE_SENSOR_DESCRIPTOR_T;// update the special porperty id information
        else if(data_len==2)
            state_params.state_type=MS_STATE_SENSOR_PROPERTY_ID_T;//update all information



        /* Assign reqeusted state type to the application */
        //state_params.state_type = MS_STATE_VENDOR_EXAMPLE_T;
    }
    break;

    case MS_ACCESS_SENSOR_GET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_get_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

        if(data_len==0)
            state_params.state_type=MS_STATE_SENSOR_DATA_T;// update the special porperty id information
        else if(data_len==2)
            state_params.state_type=MS_STATE_SENSOR_DATA_PROPERTY_ID_T;//update all information

    }
    break;



    case MS_ACCESS_SENSOR_COLUMN_GET_OPCODE:
    {
        
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_COLUMN_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_column_get_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;

        state_params.state_type=MS_STATE_SENSOR_COLUMN_STATUS_T;//get column status
    }
    break;


    case MS_ACCESS_SENSOR_SERIES_GET_OPCODE:
    {
        SENSOR_SERVER_TRC(
            "MS_ACCESS_SENSOR_SERIES_GET_OPCODE\n");

        MODEL_OPCODE_HANDLER_CALL(sensor_series_get_handler);

        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
		state_params.state_type=MS_STATE_SENSOR_SERIES_COLUMN_T;//get column status

    }
    break;

    default:
        SENSOR_SERVER_TRC(
            "MS_ACCESS_VENDOR_EXAMPLE_NONE_OPCODE\n");
        break;


    }

    /* Application callback */
    if (NULL != Sensor_server_UI_cb)
    {
        Sensor_server_UI_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
    }

    return retval;
}









/**
 *  \brief API to initialize MS_Sensor_server_init Server model
 *
 *  \par Description
 *  This is to initialize Sensor_Example_1 Server model and to register with Acess layer.
 *
 *  \param [in] element_handle
 *              Element identifier to be associated with the model instance.
 *
 *  \param [in, out] model_handle
 *                   Model identifier associated with the model instance on successful initialization.
 *                   After power cycle of an already provisioned node, the model handle will have
 *                   valid value and the same will be reused for registration.
 *
 *  \param [in] UI_cb    Application Callback to be used by the Vendor_Example_1 Server.
 *
 *  \return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_Sensor_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE    * model_handle,
    /* IN */    MS_SENSOR_MODEL_SERVER_CB UI_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;

    /* TBD: Initialize MUTEX and other data structures */

    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;

    SENSOR_SERVER_TRC(
        "[sensor server] Registered Element Handle 0x%02X\n", element_handle);

    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SENSOR_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;

    /* Register Callback */
    model.cb = Sensor_server_cb;

    /* List of Opcodes */
    model.opcodes = sensor_server_opcode_list;
    model.num_opcodes = sizeof(sensor_server_opcode_list) / sizeof(UINT32);

    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );

    /* Save Application Callback */
    Sensor_server_UI_cb = UI_cb;

    //    /* TODO: Remove */
    //    vendor_example_server_model_handle = *model_handle;

    return retval;
}


/**
 *	\brief API to initialize MS_Sensor_setup_server_init Server model
 *
 *	\par Description
 *	This is to initialize Sensor_Example_1 Server model and to register with Acess layer.
 *
 *	\param [in] element_handle
 *				Element identifier to be associated with the model instance.
 *
 *	\param [in, out] model_handle
 *					 Model identifier associated with the model instance on successful initialization.
 *					 After power cycle of an already provisioned node, the model handle will have
 *					 valid value and the same will be reused for registration.
 *
 *	\param [in] UI_cb	 Application Callback to be used by the Vendor_Example_1 Server.
 *
 *	\return API_SUCCESS or an error code indicating reason for failure
 */
API_RESULT MS_Sensor_setup_server_init
(
    /* IN */	MS_ACCESS_ELEMENT_HANDLE	element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE	  * model_handle,
    /* IN */	MS_SENSOR_SETUP_SERVER_CB UI_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID		 node_id;
    MS_ACCESS_MODEL 		 model;

    /* TBD: Initialize MUTEX and other data structures */

    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;

    SENSOR_SERVER_TRC(
        "[sensor setup server] Registered Element Handle 0x%02X\n", element_handle);

    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SENSOR_SETUP_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;

    /* Register Callback */
    model.cb = Sensor_setup_server_cb;

    /* List of Opcodes */
    model.opcodes = sensor_setup_server_opcode_list;
    model.num_opcodes = sizeof(sensor_setup_server_opcode_list) / sizeof(UINT32);

    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );

    /* Save Application Callback */
    Sensor_setup_server_UI_cb = UI_cb;

    //	  /* TODO: Remove */
    //	  vendor_example_server_model_handle = *model_handle;

    return retval;
}



