/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/*******************************************************************************
* @file			bleIIC_Slave.c
* @brief		Contains the bleIIC Slave application definitions and prototypes
* @version	1.0
* @date			2018/7/5  
* @author		Rugooo
* 
* Copyright(C) 2016, PhyPlus Semiconductor All rights reserved.
*******************************************************************************/

/*******************************************************************************
*@ Module    			:  Pre-Compiler
*@ Description    :  Null
*******************************************************************************/
#define _BLE_IIC_SLAVE_CMD_

/*******************************************************************************
*@ Module    			:  Include
*@ Description    :  Null
*******************************************************************************/
#include "types.h"
#include "bcomdef.h"
#include "simpleGATTprofile_ota.h"
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "hal_mcu.h"
#include "pwrmgr.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "bleIIC_Slave.h"
#include "bleIIC_Slave_service.h"
#include "bleIIC_Slave_protocol.h"
#include "log.h"


/*******************************************************************************
*@ Module    			:  Macro Define
*@ Description    :  Relate Device Info Service
*******************************************************************************/
#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0

/*******************************************************************************
*@ Module    			:  Macro Define
*@ Description    :  Relate Broadcast Info
*******************************************************************************/
#define DEFAULT_DISCOVERABLE_MODE         GAP_ADTYPE_FLAGS_GENERAL

/*******************************************************************************
*@ Module    			:  Macro Define
*@ Description    :  Relate Connection
*******************************************************************************/
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     30

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

/*******************************************************************************
*@ Module    			:  Variable Statement
*@ Description    :  	GAP Role state
*******************************************************************************/
static gaprole_States_t gapProfileState = GAPROLE_INIT;

/*******************************************************************************
*@ Module    			:  Variable Statement
*@ Description    :  Broadcasting adv and scanRsp data
*******************************************************************************/
static unsigned char advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x11,
    0x07,//Complete list of 128-bit UUIDs available
    0x55, 0xe4,0x05,0xd2,0xaf,0x9f,0xa9,0x8f,0xe5,0x4a,0x7d,0xfe,0x43,0x53,0x53,0x49,
};

/*******************************************************************************
*@ Module    			:  broadcasting ScanRsp Data
*@ Description    :  None
*******************************************************************************/
static unsigned char scanRspData[] =
{
    // complete name
    12,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'P','h','y',' ','B','L','E',' ','I','I','C',

    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
};

/*******************************************************************************
*@ Module    			:  Variable Statement
*@ Description    :  GAP GATT Attributes
*******************************************************************************/
static unsigned char attDeviceName[GAP_DEVICE_NAME_LEN] = "Phy BLE-IIC RawPass";

/*******************************************************************************
*@ Module    			:  Local Function Statement
*@ Description    :  Notify call back
*******************************************************************************/
static void BLEIIC_Slave_StateNotificationCB( gaprole_States_t newState );
static void on_bleIIC_SlaveServiceEvt(bleIIC_Slave_Evt_t* pev);


/*******************************************************************************
*@ Module    			:  Local Function Statement
*@ Description    :  GAP Role Call Back
*******************************************************************************/
static gapRolesCBs_t bleIIC_RawPass_PeripheralCBs =
{
	BLEIIC_Slave_StateNotificationCB,  	// Profile State Change Callbacks
	NULL							// When a valid RSSI is read from controller (not used by application)
};

/*******************************************************************************
*@ Description    :  BLE IIC Slave Initilization
*@ Input          :	 OSAL Assigned task ID
*@ Output         :  None
*@ Return         :  None
*******************************************************************************/
void BLE_IIC_Slave_Init(unsigned char task_id)
{
	BLE_IIC_TaskID = task_id;

	// Setup the GAP
	VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

	// Setup the GAP Peripheral Role Profile
	{
		// device starts advertising upon initialization
		uint8 initial_advertising_enable = TRUE;

		uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
		uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 
				
		// By setting this to zero, the device will go into the waiting state after
		// being discoverable for 30.72 second, and will not being advertising again
		// until the enabler is set back to TRUE
		uint16 gapRole_AdvertOffTime = 0;
			
		uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
		uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
		uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
		uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
			
		uint8 peerPublicAddr[] = {
		  0x01,
		  0x02,
		  0x03,
		  0x04,
		  0x05,
		  0x06
		};
		GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);

		// set adv channel map
		GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);		 

		// Set the GAP Role Parameters
		GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
		GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

		GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
		GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

		GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
		GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
		GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
		GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
		GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
	}
	// Set the GAP Characteristics
	GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

	// Set advertising interval
	{
		uint16 advInt = 400;   // actual time = advInt * 625us

		GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
		GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
		GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
	}
	  
	// Initialize GATT attributes
	GGS_AddService( GATT_ALL_SERVICES );            // GAP
	GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
	DevInfo_AddService();                           // Device Information Service
	bleIIC_Slave_AddService(on_bleIIC_SlaveServiceEvt);

	BUP_IIC_Slave_init();
	// Setup a delayed profile startup
  osal_set_event( BLE_IIC_TaskID, BUP_OSAL_EVT_START_DEVICE );
}


/*******************************************************************************
*@ Description    :  This Function is called to process all event for the task from 
*					 IIC Slave Application
*@ Input          :	 task id:the OSAL assigned task ID
*					 events:event to process
*@ Return         :  events not processed
*******************************************************************************/
uint16 bleIIC_Slave_ProcessEvent( uint8 task_id, uint16 events )
{
	if ( events & BUP_OSAL_EVT_START_DEVICE )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bleIIC_RawPass_PeripheralCBs );

    return ( events ^ BUP_OSAL_EVT_START_DEVICE );
  }
	// enable adv
  if ( events & BUP_OSAL_EVT_RESET_ADV )
  {
    uint8 initial_advertising_enable = TRUE;
		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
    return ( events ^ BUP_OSAL_EVT_RESET_ADV );
  }
	if(events & BUP_OSAL_EVT_NOTIFY_DATA)
  {
    LOG("BUP_OSAL_EVT_NOTIFY_DATA\n");
    return ( events ^ BUP_OSAL_EVT_NOTIFY_DATA );
  }
	if(events & BUP_OSAL_EVT_NOTIFY_IIC_RX_DATA)
	{
		BUP_data_Notify_IIC_Rx_Data();
		return ( events ^ BUP_OSAL_EVT_NOTIFY_IIC_RX_DATA );
	}
	if(events & BUP_OSAL_EVT_IIC_TO_HOST_TMOUT)
	{
		BLE_IIC_CheckTOMaster_success();
		return ( events ^ BUP_OSAL_EVT_IIC_TO_HOST_TMOUT );
	}
	if(events & BUP_OSAL_EVT_IIC_Notice_Master)
	{
		hal_gpio_write(FLOW_CTRL_IO_NOTICE_HOST,FALSE);
		hal_pwrmgr_unlock(MOD_USR2);
		return ( events ^ BUP_OSAL_EVT_IIC_Notice_Master );
	}
	if( events & BUP_OSAL_EVT_IIC_TX_ABRT)
	{
		BUP_IIC_Slave_status_Check();
		return (events ^ BUP_OSAL_EVT_IIC_TX_ABRT);
	}
	return 0;
}

/*******************************************************************************
*@ Description    :  Get IIC Slave
*@ Input          :
*@ Output         :
*@ Return         :
*******************************************************************************/
unsigned short BLE_IIC_Conn_Interval(void)
{


	return 0;
}


/*******************************************************************************
*@ Description    :  BLE IIC Slave Service Event Process
*@ Input          :  pev:BLE IIC Slave Event
*@ Output         :  None
*@ Return         :  None
*******************************************************************************/
void on_bleIIC_SlaveServiceEvt(bleIIC_Slave_Evt_t* pev)
{
	switch(pev->ev)
	{
		case bleIIC_Slave_EVT_TX_NOTI_DISABLED:
		break;
		case bleIIC_Slave_EVT_TX_NOTI_ENABLED :
			osal_set_event(BLE_IIC_TaskID,BUP_OSAL_EVT_NOTIFY_DATA);
		break;
		case bleIIC_Slave_EVT_BLE_DATA_RECIEVED:
			BUP_data_BLE_Received( (uint8_t*)pev->data, (uint8_t)pev->param);
		break;
		default:
		break;
	}
}

/*******************************************************************************
*@ Description    :  Notification frm the profile of a state change
*@ Input          :  newState:gaprole state
*@ Output         :  None
*@ Return         :  None
*******************************************************************************/
void BLEIIC_Slave_StateNotificationCB( gaprole_States_t newState )
{
	switch ( newState )
	{
		case GAPROLE_STARTED:
		{
			uint8 ownAddress[B_ADDR_LEN];
			uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

			GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

			// use 6 bytes of device address for 8 bytes of system ID value
			systemId[0] = ownAddress[0];
			systemId[1] = ownAddress[1];
			systemId[2] = ownAddress[2];

			// set middle bytes to zero
			systemId[4] = 0x00;
			systemId[3] = 0x00;

			// shift three bytes up
			systemId[7] = ownAddress[5];
			systemId[6] = ownAddress[4];
			systemId[5] = ownAddress[3];

			DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
		}
		break;

		case GAPROLE_ADVERTISING:
			BUP_disconnect_handler();
			LOG("advertising!\n");
		break;

		case GAPROLE_CONNECTED:
			GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
			LOG("connected handle[%d]!\n", gapConnHandle);
		break;

		case GAPROLE_CONNECTED_ADV:
		break;	   
		case GAPROLE_WAITING:
		break;

		case GAPROLE_WAITING_AFTER_TIMEOUT:
		break;

		case GAPROLE_ERROR:
		break;

		default:
		break;		 
	}  
	gapProfileState = newState;

	VOID gapProfileState;	
}

/*******************************************************************************
*@ Description    :  BLE_IIC_CheckTOMaster_success
*@ Input          :  None
*@ Output         :  None
*@ Return         :  None
*******************************************************************************/
void BLE_IIC_CheckTOMaster_success(void)
{
	hal_pwrmgr_unlock(MOD_USR1);
//	LOG("hal_pwrmgr_unlock(MOD_USR1); after send\n");
}
