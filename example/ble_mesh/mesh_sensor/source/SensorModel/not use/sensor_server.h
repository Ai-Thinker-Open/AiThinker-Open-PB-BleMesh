/**
 * \file scene_server.h
 */

/*
 * Copyright (C) 2017. PHYPLUSINC Ltd.
 * All rights reserved.
 */

#ifndef _H_SENSOR_SERVER_
#define _H_SENSOR_SERVER_


/* --------------------------------------------- Header File Inclusion */
#include "MS_access_api.h"


enum SENSOR_SETTING_ACCESS
{
 Sensor_Access_Prohibited0=0,
 Sensor_Access_read=1,
 Sensor_Access_Prohibited2=2,
 Sensor_Access_written=3,
 Sensor_Access_Prohibited4=4,
 Sensor_Access_Prohibited_max=0xff
};







/* --------------------------------------------- Global Definitions */
#ifdef  SENSOR_SERVER_NO_DEBUG
#define SENSOR_SERVER_ERR         EM_debug_null
#else /* SENSOR_SERVER_NO_DEBUG */
#ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
#define SENSOR_SERVER_ERR
#else
#define SENSOR_SERVER_ERR(...)     EM_debug_error(MS_MODULE_ID_MESH_MODEL, __VA_ARGS__)
#endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* SENSOR_SERVER_NO_DEBUG */
#ifdef SENSOR_SERVER_DEBUG
#ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
#define SENSOR_SERVER_TRC
#define SENSOR_SERVER_INF
#define SENSOR_SERVER_debug_dump_bytes(data, datalen)
#else
#define SENSOR_SERVER_TRC(...)     EM_debug_trace(MS_MODULE_ID_MESH_MODEL,__VA_ARGS__)
#define SENSOR_SERVER_INF(...)     EM_debug_info(MS_MODULE_ID_MESH_MODEL,__VA_ARGS__)
#define SENSOR_SERVER_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_MESH_MODEL, (data), (datalen))
#endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* SENSOR_SERVER_DEBUG */
#define SENSOR_SERVER_TRC          EM_debug_null
#define SENSOR_SERVER_INF          EM_debug_null
#define SENSOR_SERVER_debug_dump_bytes(data, datalen)
#endif /* SENSOR_SERVER_DEBUG */




/* Sensors Property ID */
#define AMB_TEMPERATURE_PID     0x0071// 0x004F for lighting:Ambient Temperature
#define PRESSURE_PID            0x2A6D
#define TEMPERATURE_PID         0x2A6E
#define HUMIDITY_PID            0x2A6F
#define MAGNETO_METER_PID       0x2AA1
#define ACCELERO_METER_PID      0x2BA1
#define GYROSCOPE_PID           0x2BA2
#define VOLTAGE_PID             0x0005
#define CURRENT_PID             0x0004
#define POWER_FACTOR_PID        0x0072
#define ACTIVE_POWER_PID        0x0073
#define REACTIVE_POWER_PID      0x0074
#define APPARENT_POWER_PID      0x0075
#define ACTIVE_ENERGY_PID       0x0083
#define REACTIVE_ENERGY_PID     0x0084
#define APPARENT_ENERGY_PID     0x0085


// sensor column setting

#define SENSOR_NUM  4


#define VOL_COLUMN_NUM   3
#define VOL_VALUE_LEN   4
extern uint8 vol_raw_value_x0[VOL_VALUE_LEN];
extern uint8 vol_column_w0[VOL_VALUE_LEN];
extern uint8 vol_raw_value_y0[VOL_VALUE_LEN];

extern uint8 vol_raw_value_x1[VOL_VALUE_LEN];
extern uint8 vol_column_w1[VOL_VALUE_LEN];
extern uint8 vol_raw_value_y1[VOL_VALUE_LEN];

extern uint8 vol_raw_value_x2[VOL_VALUE_LEN];
extern uint8 vol_column_w2[VOL_VALUE_LEN];
extern uint8 vol_raw_value_y2[VOL_VALUE_LEN];


#define HUMITY_COLUMN_NUM   2
#define HUMITY_VALUE_LEN   4
extern uint8 humity_raw_value_x0[HUMITY_VALUE_LEN];
extern uint8 humity_column_w0[HUMITY_VALUE_LEN];
extern uint8 humity_raw_value_y0[HUMITY_VALUE_LEN];

extern uint8 humity_raw_value_x1[HUMITY_VALUE_LEN];
extern uint8 humity_column_w1[HUMITY_VALUE_LEN];
extern uint8 humity_raw_value_y1[HUMITY_VALUE_LEN];


#define TEMPERATURE_COLUMN_NUM   2
#define TEMPERATURE_VALUE_LEN   2
extern uint8 temperature_raw_value_x0[TEMPERATURE_VALUE_LEN];
extern uint8 temperature_column_w0[TEMPERATURE_VALUE_LEN];
extern uint8 temperature_raw_value_y0[TEMPERATURE_VALUE_LEN];

extern uint8 temperature_raw_value_x1[TEMPERATURE_VALUE_LEN];
extern uint8 temperature_column_w1[TEMPERATURE_VALUE_LEN];
extern uint8 temperature_raw_value_y1[TEMPERATURE_VALUE_LEN];



#define ACC_COLUMN_NUM   3
#define ACC_VALUE_LEN    2
extern uint8 ACC_raw_value_x0[ACC_VALUE_LEN];
extern uint8 ACC_column_w0[ACC_VALUE_LEN];
extern uint8 ACC_raw_value_y0[ACC_VALUE_LEN];

extern uint8 ACC_raw_value_x1[ACC_VALUE_LEN];
extern uint8 ACC_column_w1[ACC_VALUE_LEN];
extern uint8 ACC_raw_value_y1[ACC_VALUE_LEN];


extern uint8 ACC_raw_value_x2[ACC_VALUE_LEN];
extern uint8 ACC_column_w2[ACC_VALUE_LEN];
extern uint8 ACC_raw_value_y2[ACC_VALUE_LEN];

extern MS_STATE_SENSOR_DESCRIPTOR_STRUCT UI_Sensor_Descrip_Infor[SENSOR_NUM];
extern MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Vol_Sensor_series_column[VOL_COLUMN_NUM];
extern MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Humity_Sensor_series_column[HUMITY_COLUMN_NUM];
extern extern MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_Temperature_Sensor_series_column[TEMPERATURE_COLUMN_NUM];
extern MS_STATE_SENSOR_SERIES_COLUMN_STRUCT UI_ACC_Sensor_series_column[ACC_COLUMN_NUM];


//cadence setting
#define SENSOR_UPDATE_PERIOD  10000 // ms
#define SENSOR_CADENCE_NUMBER  3
#define  VOL_CADENCE_LEN     2
#define  HUMITY_CADENCE_LEN  2
#define  TEMPERATURE_CADENCE_LEN  2



extern uint8 voltage_trigger_delta_down;
extern uint8 voltage_trigger_delta_up;
extern uint8 voltage_fast_cadence_low[];
extern uint8 voltage_fast_cadence_high[];


extern uint8 humity_trigger_delta_down;
extern uint8 humity_trigger_delta_up;
extern uint8 humity_fast_cadence_low[];
extern uint8 humity_fast_cadence_high[];

extern uint8 temperature_trigger_delta_down;
extern uint8 temperature_trigger_delta_up;
extern uint8 temperature_fast_cadence_low[];
extern uint8 temperature_fast_cadence_high[];


extern MS_STATE_SENSOR_CADENCE_STRUCT Sensor_CadenceSetting[];

//cadence setting end

//sensor settings start

#define SENSOR_SETTINGS_NUM  3

extern MS_STATE_SENSOR_SETTING_STRUCT humity_sensor_setting[];

extern MS_STATE_SENSOR_SETTINGS_STRUCT humity_sensor_settings;

//sensor settings end





/* --------------------------------------------- Data Types/ Structures */

/** Sensor Model specific state parameters in a request or response message */
typedef struct _MS_access_Sensor_model_state_params
{
    /** State Type */
    UINT16 state_type;
	UINT16 len;

    /** State pointer */
    void * state;

}MS_ACCESS_SENSOR_MODEL_STATE_PARAMS;



typedef API_RESULT (* MS_SENSOR_MODEL_SERVER_CB)
        (
            MS_ACCESS_MODEL_REQ_MSG_CONTEXT     * ctx,
            MS_ACCESS_MODEL_REQ_MSG_RAW         * msg_raw,
            MS_ACCESS_MODEL_REQ_MSG_T           * req_type,
            MS_ACCESS_SENSOR_MODEL_STATE_PARAMS * state_params,
            MS_ACCESS_MODEL_EXT_PARAMS          * ext_params

        ) DECL_REENTRANT;
		
typedef API_RESULT (* MS_SENSOR_SETUP_SERVER_CB)
        (
            MS_ACCESS_MODEL_REQ_MSG_CONTEXT     * ctx,
            MS_ACCESS_MODEL_REQ_MSG_RAW         * msg_raw,
            MS_ACCESS_MODEL_REQ_MSG_T           * req_type,
            MS_ACCESS_SENSOR_MODEL_STATE_PARAMS * state_params,
            MS_ACCESS_MODEL_EXT_PARAMS          * ext_params

        ) DECL_REENTRANT;		


/** -- Sensor  Defined States */
extern  MS_STATE_SENSOR_DESCRIPTOR_STRUCT UI_Sensor_Descrip_Infor[SENSOR_NUM];







/* --------------------------------------------- Function */
API_RESULT MS_Sensor_server_init
        (
            /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
            /* INOUT */ MS_ACCESS_MODEL_HANDLE    * model_handle,
            /* IN */    MS_SENSOR_MODEL_SERVER_CB UI_cb
        );

API_RESULT MS_Sensor_setup_server_init
		(
			/* IN */	MS_ACCESS_ELEMENT_HANDLE	element_handle,
			/* INOUT */ MS_ACCESS_MODEL_HANDLE	  * model_handle,
			/* IN */	MS_SENSOR_SETUP_SERVER_CB UI_cb
		);





#endif /*_H_SCENE_SERVER_ */
