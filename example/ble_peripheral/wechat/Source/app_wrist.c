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


/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "ota_app_service.h"
#include "gatt_profile_uuid.h"
#include "wristservice.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "app_wrist.h"

#include "led_light.h"
#include "log.h"
#include "ll.h"

#include "weixinservice.h"
#include "WeChatble.h"
#include "ble_wechat_util.h"

/*********************************************************************
 * MACROS
 */

// Convert BPM to RR-Interval for data simulation purposes
#define HEARTRATE_BPM_TO_RR(bpm)              ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             1600

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// How often to perform heart rate periodic event
#define DEFAULT_HEARTRATE_PERIOD              2000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20//200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     30//1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0//1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

#define DEVINFO_SYSTEM_ID_LEN                 8
#define DEVINFO_SYSTEM_ID                     0

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
//#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


// Some values used to simulate measurements
#define BPM_DEFAULT                           73
#define BPM_MAX                               80
#define ENERGY_INCREMENT                      10
#define FLAGS_IDX_MAX                         7

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 AppWrist_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
    // complete name
    0x0B,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
		0x70,   // 'p'
		0x68,   // 'h'
		0x79,   // 'y'
		0x5F,		// '_' 
		0x77,   // 'w'
		0x65,   // 'e'
		0x63,   // 'c'
		0x68,   // 'h'
		0x61,   // 'a'
		0x74,   // 't'

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


static uint8 advertData[] =
{
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
	
		//0x11,
    //0x07,//Complete list of 128-bit UUIDs available
    //0x55, 0xe4,0x05,0xd2,0xaf,0x9f,0xa9,0x8f,0xe5,0x4a,0x7d,0xfe,0x43,0x53,0x53,0x49,  
	
	0x03,
	GAP_ADTYPE_16BIT_MORE,
	LO_UINT16(WEIXIN_SERVICE_UUID),
	HI_UINT16(WEIXIN_SERVICE_UUID),
	
	0x09,
	0xFF,
	0x01,//manufacture 2byte
	0x02,
	0xf1,//mac addr 8byte
	0xf2,
	0xf3,
	0xf4,
	0xf5,
	0xf6
};

// Device name attribute value
//static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PhyLight ";
//static uint8 attDeviceName[] = "bleWheel";
static uint8 attDeviceName[] = "phy_wechat";

// GAP connection handle
static uint16 gapConnHandle;

// Advertising user-cancelled state
static bool WristAdvCancelled = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void appWristProcOSALMsg(osal_event_hdr_t *pMsg);
static void WristGapStateCB(gaprole_States_t newState);
static void wristCB(uint8 event, uint8 param_size, uint8* param);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t WristPeripheralCB =
{
  WristGapStateCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
static const gapBondCBs_t WristBondCB =
{
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_Init
 *
 * @brief   Initialization function for the Heart Rate App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
 
static void pedometer_UpdateGattAttr(void);
static void WeixinBLEPeripheral_simpleProfileCBs( uint8 notifyApp);

void appWristInit(uint8 task_id)
{
  AppWrist_TaskID = task_id;
	
	// Setup the GAP
	VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
	
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;
    uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
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

		uint8 advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
		GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
		
    GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
    // set adv channel map
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof (scanData), scanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof(advertData), advertData );
    
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
      uint16 advInt = 1600;   // actual time = advInt * 625us
  
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
		// Setup the GAP Bond Manager
	{
			uint32 passkey = 0; // passkey "000000"
			uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;
			uint8 mitm = TRUE;
			uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
			uint8 bonding = TRUE;

			GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
			GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
			GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
			GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
			GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
	}
		
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService();
				
  ota_app_AddService();
  wristProfile_AddService(wristCB);
  
	Weixin_AddService(GATT_ALL_SERVICES);
	WeixinProfile_RegisterAppCBs(&WeixinBLEPeripheral_simpleProfileCBs);
		
  //app_datetime_init();
  // Setup a delayed profile startup
  osal_set_event( AppWrist_TaskID, START_DEVICE_EVT);
  //light_init();
}

/*********************************************************************
 * @fn      HeartRate_ProcessEvent
 *
 * @brief   Heart Rate Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 appWristProcEvt( uint8 task_id, uint16 events )
{  
  VOID task_id;

  if (events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;

    if((pMsg = osal_msg_receive( AppWrist_TaskID)) != NULL)
    {
      appWristProcOSALMsg((osal_event_hdr_t *)pMsg);     
      VOID osal_msg_deallocate(pMsg);
    }

    return (events ^ SYS_EVENT_MSG);
  }
 
  if(events & START_DEVICE_EVT)
  {
		VOID GAPRole_StartDevice(&WristPeripheralCB);  
		GAPBondMgr_Register((gapBondCBs_t *)&WristBondCB);
		
		osal_set_event( AppWrist_TaskID, PERIODIC_EVT);
		LOG("wechat_init\n");
		wechat_init();		
    return ( events ^ START_DEVICE_EVT );
  }

  if(events & TIMER_DT_EVT)
  {
    //app_datetime_sync_handler();
    return ( events ^ TIMER_DT_EVT );
  }

  if(events & TIMER_LIGHT_EVT)
  {
   //light_timeout_handle();
    return ( events ^ TIMER_LIGHT_EVT);
  }
  
	if(events & PERIODIC_EVT)
	{
		pedometer_UpdateGattAttr();
		osal_start_timerEx( AppWrist_TaskID, PERIODIC_EVT, PERIODIC_EVT_PERIOD);
		return (events ^ PERIODIC_EVT);
	}
	
  return 0;
}


/*********************************************************************
 * @fn      appWristProcOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */

extern bStatus_t ble_send_indicate_data_conform(void);

void Wrist_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_HANDLE_VALUE_CFM)
  {
			LOG("Wrist_processGATTMsg\n");
      ATT_HandleValueCfm(pMsg->connHandle);
      ble_send_indicate_data_conform();
  }
}

static void appWristProcOSALMsg( osal_event_hdr_t *pMsg )
{
	if(pMsg->event == GATT_MSG_EVENT)
  {
	  Wrist_processGATTMsg((gattMsgEvent_t *)pMsg);
  }
}

/*********************************************************************
 * @fn      WristGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void WristGapStateCB( gaprole_States_t newState )
{
  LOG("WristGapStateCB: %d", newState);
  // if connected
  if (newState == GAPROLE_CONNECTED)
  {
    // get connection handle
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
  }
  // if disconnected
  else if (gapProfileState == GAPROLE_CONNECTED && 
           newState != GAPROLE_CONNECTED)
  {
    uint8 advState = TRUE;

    // reset client characteristic configuration descriptors
    wristProfile_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED);

    if (newState == GAPROLE_WAITING_AFTER_TIMEOUT)
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL);
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL);
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION);
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION);
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState);    
  }    
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING)
  {
    // if advertising stopped by user
    if ( WristAdvCancelled)
    {
      WristAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL)
    {
      uint8 advState = TRUE;
      
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL);
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState);   
    }  
  }
  // if started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
    
    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];
    
    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;
    
    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  gapProfileState = newState;
}

/*********************************************************************
 * @fn      heartRateCB
 *
 * @brief   Callback function for Wrist service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void wristCB(uint8 event, uint8 param_size, uint8* param)
{
	switch(event)
	{
		case WRIST_NOTI_ENABLED:
		{
			// if connected start periodic measurement
			if (gapProfileState == GAPROLE_CONNECTED)
			{
				//osal_start_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
			}
			break;
		}
		case WRIST_NOTI_DISABLED:
		{
			// stop periodic measurement
			//osal_stop_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT );
			break;
		}
	}
}


/*
wechat
*/
/*********************************************************************
 * @fn      bleSmartPeripheral_UpdateGattAttr
 *
 * @brief   Read peripheral and update characteristics of simple GATT profile
 *
 * @param   None
 *
 * @return  none
 */

static void pedometer_UpdateGattAttr(void)
{
	static uint32 stepCount = 88888;
	uint8 charValue[4];

	// TODO: read the data from peripheral
	stepCount++;
	charValue[0] = 0x01;
	memcpy(charValue+1, (uint8 *)&stepCount, 3);

	// update the GATT characteristics' value
	Weixin_SetParameter( WEIXIN_SIMPLEPROFILE_PEDOMETER_CHAR, sizeof (charValue), &charValue);
	
	static uint8 targetVal[4] = {0x01, 0x10, 0x27, 0x00};
	Weixin_SetParameter( WEIXIN_SIMPLEPROFILE_TARGET_CHAR, sizeof ( targetVal ), &targetVal );
}

static void WeixinBLEPeripheral_simpleProfileCBs(uint8 evt)
{
	uint8 newValue[20]={0};
	
	if(evt ==  WECHAT_GATT_WRITE)
	{
		Weixin_GetParameter(WEIXINPROFILE_WRITE_CHAR, newValue);
		wechat_write_cb_consume(newValue);
	}
	
	if(evt ==  WECHAT_GATT_INDICATE)
	{
		device_auth();
	}
}

