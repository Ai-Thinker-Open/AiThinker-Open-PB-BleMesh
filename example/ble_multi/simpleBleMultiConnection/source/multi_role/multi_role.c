/**************************************************************************************************
  Filename:       multi_role.c
  Revised:         
  Revision:        

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the Bluetooth Low Energy Protocol Stack.

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#if (APP_CFG == 5)
#include <stdio.h>
#include <string.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gap.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "multi.h"
#include "gapbondmgr.h"
#include "multiRoleProfile.h"
#include "multi_role.h"
#include "ll_common.h"
#include "log.h"
#include "hci.h"
#include "error.h"
#include "gpio.h"
#include "pwrmgr.h"
//#include "ll_def.h"
//#include "ppsp_impl.h"
#include "hiddev.h"
#include "hidkbdservice.h"
#include "battservice.h"
#include "devinfoservice.h"
#include "gatt_profile_uuid.h"
/*********************************************************************
 * MACROS
 */



/*********************************************************************
 * CONSTANTS
 */
// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1500
#define DEFAULT_CONN_EVT_MARGIN					2

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// for alternate Advertising and Scanning
#define DEFAULT_ADVOFF_TIME_CNT					5		// units DEFAULT_ADVOFF_TIME_INTV
#define DEFAULT_ADVOFF_TIME_INTV				100		// units:ms

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE 

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           200

// Connection Pause Peripheral time value (in 100ms)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         50

// Default passcode
#define DEFAULT_PASSCODE                      123456

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

#define INVALID_STATUS							0xFFFF

// UART-RAWPASS
#define UART_RX_BUF_SIZE						520
#define UART_TX_BUF_SIZE						520

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              0

// Selected HID keycodes
#define KEY_RIGHT_ARROW             0x4F
#define KEY_LEFT_ARROW              0x50
#define KEY_NONE                    0x00

// Selected HID LED bitmaps
#define LED_NUM_LOCK                0x01
#define LED_CAPS_LOCK               0x02

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5


/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16 MR_WakeupCnt = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static	uint8_t MRnotifyBuf[520];

// Task ID for internal task/event processing
uint8 multiRole_TaskId;

// 
MultiRoleApp_Link_t g_MRLink[MAX_CONNECTION_NUM];


// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE MultiRole";

// Multi-role Advertising Data & Scan Response Data 
static uint8_t gapMultiRole_AdvertData[] =
{
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED
};
static uint8_t  gapMultiRole_ScanRspData0[] = 
{
	0x0C,							  // length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,	// AD Type = Complete local name
	'M','u','l','t','i','-','R','o','l','e','0'
};
static uint8_t  gapMultiRole_ScanRspData1[] = 
{
	0x0C,							  // length of this data
	GAP_ADTYPE_LOCAL_NAME_COMPLETE,	// AD Type = Complete local name
	'M','u','l','t','i','-','R','o','l','e','1'
};



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void multiRoleSDPCB( uint16 connHandle );
static void multiRoleRssiCB( uint16 connHandle, int8  rssi );
static void multiRoleEachScanCB( gapDeviceInfoEvent_t *pPkt );
static void	multiRoleScanDoneCB( uint8 scanCnt , MultiRoleADV_ScanData *pPkt );
static uint8 multiRole_filterScanDev( MultiRole_filter_t fPolicy,MultiRoleADV_ScanData *pPkt,uint8 *pData,uint8 len);
static void multiRoleEstablishCB( uint16 connHandle,GAPMultiRole_State_t role,multiRole_states_t newState,uint8 *addr );
static void multiRoleTerminateCB( uint16 connHandle,GAPMultiRole_State_t role,multiRole_states_t newState,uint8 reason );
static void multiRoleApp_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void multiRoleProfileChangeCB( uint16 connHandle,uint16 paramID, uint16 len );
static void  MultiRole_DisplayADVData( uint8 scanCnt , MultiRoleADV_ScanData *pPkt );
static void MultiRole_DisplaySDPInfo( GattScanServer *pPkt );
static uint16  MultiRole_FindCCCD( GattScanServer *pPkt );
static void MultiRole_PeriodProcess( void );
static void multiRoleAppProcessGATTMsg( gattMsgEvent_t *pMsg );
static uint16 MultiRole_FindUUID_Handle( GattScanServer *pPkt ,uint16 uuid );
static uint8_t MultiRole_WriteValue(uint16 connHandle , uint16 uuid,uint16 len,uint8 *pData);
static uint8 MultiRole_EnableCCCD( uint16 connHandle,uint16 uuid);
static void multiRoleAppRecvNotifyData( gattMsgEvent_t *pMsg );
static uint8 MultiRole_WriteChar5Value( uint16 connHandle);
// for multi-role HID
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint16 *pLen, uint8 *pData );
static void hidKbdEvtCB( uint8 evt );
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData );
static void hidKbdSendReport( uint16 connHandle,uint8 keycode );


/*********************************************************************
 * LOCAL VARIABLES
 */
// TRUE if boot mouse enabled
uint8 hidBootMouseEnabled = FALSE;
// HID Dev configuration
static hidDevCfg_t hidKbdCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS 			  // HID feature flags
};

static hidDevCB_t hidKbdHidCBs =
{
  hidKbdRptCB,
  hidKbdEvtCB,
  NULL
};
extern hidDevCfg_t *pHidDevCfg;

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Role Callbacks
static const gapMultiRolesCBs_t multiRoleCB =
{
	multiRoleEachScanCB,
	multiRoleScanDoneCB,
	multiRoleEstablishCB,
	multiRoleTerminateCB,
	multiRoleRssiCB,                        
	multiRoleSDPCB
};



// GATT Profile Callbacks
static multiProfileCBs_t multiRole_ProfileCBs =
{
    multiRoleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void multiRoleApp_Init( uint8 task_id )
{
	multiRole_TaskId = task_id;
	memset(g_MRLink,0,sizeof(g_MRLink));
	// Setup the GAP Multi-Role Profile
	{
		// device starts advertising upon initialization
		uint8 initial_advertising_enable = TRUE;

		uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 
		// margin :2 . 16=20/1.25
//		uint16 desired_min_interval = ((MAX_CONNECTION_NUM + DEFAULT_CONN_EVT_MARGIN) * 16 ) ;
		uint16 desired_min_interval = 80 ;

		uint16 desired_slave_latency = DEFAULT_UPDATE_SLAVE_LATENCY;
		uint16 desired_conn_timeout = DEFAULT_UPDATE_CONN_TIMEOUT;

		// By setting this to zero, the device will go into the waiting state after
		// being discoverable for 30.72 second, and will not being advertising again
		// until the enabler is set back to TRUE
		uint16 	gapRole_AdvertOffTime = DEFAULT_ADVOFF_TIME_CNT * DEFAULT_ADVOFF_TIME_INTV;
		uint8 	advType0 = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;	// it seems a TI bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
//		uint8 	advType1 = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;
		uint8	roleProfile = GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL;

		// Setup the GAP
		uint16 Conndelay = DEFAULT_CONN_PAUSE_PERIPHERAL;
		GAPMultiRole_SetParameter( TGAP_CONN_PAUSE_PERIPHERAL,sizeof(uint16), &Conndelay,0 );
		for( uint8_t i=0;i < MAX_CONNECTION_SLAVE_NUM; i++)
		{
			GAPMultiRole_SetParameter( GAPROLE_PROFILEROLE,sizeof(uint8),&roleProfile,i);
			GAPMultiRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap, i);	   
			GAPMultiRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable, i);
			GAPMultiRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime, i);
			GAPMultiRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( gapMultiRole_AdvertData ), gapMultiRole_AdvertData, i);
			// Set the GAP Role Parameters
			GAPMultiRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval,i );
			GAPMultiRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval,i );
			GAPMultiRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency,i );
			GAPMultiRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout,i );
		}
		GAPMultiRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType0 ,0);
		GAPMultiRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType0 ,1);
		GAPMultiRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( gapMultiRole_ScanRspData0 ), gapMultiRole_ScanRspData0, 0);
		GAPMultiRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( gapMultiRole_ScanRspData1 ), gapMultiRole_ScanRspData1, 1);
	}

	// for multi-role Master
	{
		// Setup GAP
		GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
		GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

		// margin :2 . 16=20/1.25
//		uint16 EstMIN = (MAX_CONNECTION_NUM + DEFAULT_CONN_EVT_MARGIN) * 16;
		uint16 EstMIN = 80;
		GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, EstMIN );      //  * 1.25ms      // 30
		GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, EstMIN );
		GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT,DEFAULT_UPDATE_CONN_TIMEOUT );
	}

	
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN,64);
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX,64);
	GAP_SetParamValue( TGAP_CONN_ADV_INT_MIN,64);
	GAP_SetParamValue( TGAP_CONN_ADV_INT_MAX,64);

//	GAP_SetParamValue( TGAP_SCAN_RSP_RSSI_MIN, -70 );
	
	GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );
	// Setup the GAP Bond Manager
	{
		uint32 passkey = DEFAULT_PASSCODE;
		uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;//GAPBOND_PAIRING_MODE_INITIATE;//DEFAULT_PAIRING_MODE;    // GAPBOND_PAIRING_MODE_NO_PAIRING
		uint8 mitm = DEFAULT_MITM_MODE;
		uint8 ioCap = DEFAULT_IO_CAPABILITIES;
		uint8 bonding = DEFAULT_BONDING_MODE;
		uint8 syncWL = TRUE;
		GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
		GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
		GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
		GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
		GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );

		// If a bond is created, the HID Device should write the address of the
		// HID Host in the HID Device controller's white list and set the HID
		// Device controller's advertising filter policy to 'process scan and
		// connection requests only from devices in the White List'.
		GAPBondMgr_SetParameter( GAPBOND_AUTO_SYNC_WL, sizeof( uint8 ), &syncWL );
	}  

	// Initialize GATT Client
	GATT_InitClient();

	// Register to receive incoming ATT Indications/Notifications
	GATT_RegisterForInd( multiRole_TaskId );

	// Initialize GATT attributes
	GGS_AddService( GATT_ALL_SERVICES );         // GAP
	GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
	DevInfo_AddService();
	Batt_AddService();
	Batt_Register(NULL);
	// Set up HID keyboard service
	HidKbd_AddService();
	// Register for HID Dev callback
	HidDev_Register( &hidKbdCfg, &hidKbdHidCBs );
	
	MultiProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
	MultiProfile_RegisterAppCBs(&multiRole_ProfileCBs);

//    ppsp_impl_ini();
    
	llInitFeatureSetDLE(TRUE);
	#if (DEBUG_INFO==0)
		BUP_init(on_BUP_Evt);
	#endif
	// Setup a delayed profile startup
	osal_set_event( multiRole_TaskId, START_DEVICE_EVT );
}



/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */

uint16 multiRoleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  uint8_t   numConns;

//  AT_LOG("multiRoleApp_ProcessEvent : %d\n", events);
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( multiRole_TaskId )) != NULL )
    {
      multiRoleApp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    GAPMultiRole_StartDevice( (gapMultiRolesCBs_t *)&multiRoleCB, &numConns);

	osal_start_reload_timer( multiRole_TaskId , MULTIROLE_PERIOD_EVT, 500);
	osal_start_timerEx( multiRole_TaskId , BUP_OTA_PERIOD_EVT, 100);
    return ( events ^ START_DEVICE_EVT );
  }

	if( events & MULTIROLE_PERIOD_EVT )
	{
		MultiRole_PeriodProcess();
		return ( events ^ MULTIROLE_PERIOD_EVT );
	}
	if( events & BUP_OTA_PERIOD_EVT )
	{
//		ppsp_impl_appl_timr_hdlr();
		osal_start_timerEx( multiRole_TaskId , BUP_OTA_PERIOD_EVT,200);
//		AT_LOG("ppsp_impl_appl_timr_hdlr ... \n");
		return ( events ^ BUP_OTA_PERIOD_EVT );
	}
	if( events & MULTIROLE_HID_IDLE_EVT )
	{

		return (events ^ MULTIROLE_HID_IDLE_EVT );
	}
	if( events & MULTIROLE_HID_SEND_REPORT_EVT )
	{
//		hidKbdSendReport(0,30);
		hidKbdSendReport(0,0);
		return (events ^ MULTIROLE_HID_SEND_REPORT_EVT );
	}
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void multiRoleApp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
	switch ( pMsg->event )
	{
		case GATT_MSG_EVENT:
			multiRoleAppProcessGATTMsg( (gattMsgEvent_t *) pMsg );
		break;
	}
}

static void multiRoleAppProcessGATTMsg( gattMsgEvent_t *pMsg )
{	
	switch( pMsg->method )
	{
		case ATT_HANDLE_VALUE_NOTI:
			if( pMsg->hdr.status == SUCCESS )
			{
				multiRoleAppRecvNotifyData( pMsg );
			}
		break;
		case ATT_EXCHANGE_MTU_RSP:
			if( g_MRLink[pMsg->connHandle].role == Slave_Role ) 
			{
				g_MRLink[pMsg->connHandle].MTUExchange = TRUE;
				LOG("connHand %d,exchangeMTU status %d,rxMTU %d\n",	pMsg->connHandle,\
																	pMsg->hdr.status,\
																	pMsg->msg.exchangeMTURsp.serverRxMTU);
			}
		break;
		default:
    
		break;
	}
}


void multiRoleProfileChangeCB( uint16 connHandle,uint16 paramID, uint16 len )
{
	uint16 i;
	uint8 newValue[MULTI_ATT_LONG_PKT];
	AT_LOG("connHandle %d,ProfileChange idx=0X%04X,len %d,value:",connHandle,paramID,len);
	switch( paramID )
	{
		case MULTIPROFILE_CHAR5:
			MultiProfile_GetParameter( connHandle,MULTIPROFILE_CHAR5, newValue );
			for(i=0;i<len;i++)
				AT_LOG("0x%02X,",newValue[i]);
		break;
		case MULTIPROFILE_CHAR6:
			LOG("MULTIPROFILE_CHAR6 Notify  \n");
			if( g_MRLink[connHandle].notify )
				g_MRLink[connHandle].notify = FALSE;
			else
			{
				g_MRLink[connHandle].notify = TRUE;
			}
		break;
		default:
		break;
	}
	AT_LOG("\n");

}

static void multiRoleEachScanCB( gapDeviceInfoEvent_t *pPkt )
{
//	AT_LOG(" addr: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",\
//				pPkt->addr[0],pPkt->addr[1],pPkt->addr[2],pPkt->addr[3],\
//				pPkt->addr[4],pPkt->addr[5]);

	// notes : Application can get more info form pointer pPkt	
	// eg:
	// addrType , adv event type , rssi , advData ...

}
static uint8 multiRole_filterScanDev( MultiRole_filter_t fPolicy,MultiRoleADV_ScanData *pPkt,uint8 *pData,uint8 len)
{
	uint8 ret = FALSE;
	switch( fPolicy )
	{
		case Dev_Name:
		{
			if( len > 0 )
			{
				if( strstr( (char*)pPkt->LocalName.Value,(char*)pData) != NULL )
					ret = TRUE;
			}
		}
		break;
		case Dev_MAC:
		{
			if( len > 0 )
			{
				if( 0 == memcmp( pPkt->addr,pData,B_ADDR_LEN )  )
					ret = TRUE;
			}
		}
		break;
		default:
			AT_LOG("fPolicy NOT supported \n");
		break;
	}
	return ret;
}
static void	multiRoleScanDoneCB( uint8 scanCnt , MultiRoleADV_ScanData *pPkt )
{
//	uint8 fData[]={'P','H','Y','+','A','2','-',' ','-','5','6'};
	uint8 addr[3][B_ADDR_LEN]={	{0x00,0x12,0x34,0x56,0x56,0x78},\
								{0x01,0x12,0x34,0x56,0x56,0x78},\
								{0x02,0x12,0x34,0x56,0x56,0x78}};
	// demo test code : check only the high 5 bytes
	for(uint8_t i=0;i< scanCnt ;i++)
	{
		for(uint8_t j=0;j< 3 ;j++)
		{
			if( TRUE == multiRole_filterScanDev( Dev_MAC,&pPkt[i],addr[j],B_ADDR_LEN ) )
			{
				GAPMultiRole_addPeerAddr(pPkt[i].addr,LL_DEV_ADDR_TYPE_PUBLIC,TRUE);
			}
		}
	}
	// Application Code can do other filter
	
	// display scaned device list 
	MultiRole_DisplayADVData( scanCnt , pPkt );
}


static void multiRoleEstablishCB( uint16 connHandle,GAPMultiRole_State_t role,multiRole_states_t newState,uint8 *addr )
{
	g_MRLink[connHandle].state = GAPROLE_CONNECTED;
	g_MRLink[connHandle].role = role;
	LOG("Establish success connHandle %d\n",connHandle);
	LOG("Establish success connHandle %d,role %d,newState %d,peerAddr:\n",connHandle,role,newState);
	for ( uint8 i = 0; i < B_ADDR_LEN ; i++ )
		LOG("0x%02X,",addr[i]);
	LOG("\n");
	
}

static void multiRoleTerminateCB( uint16 connHandle,GAPMultiRole_State_t role,multiRole_states_t newState,uint8 reason )
{
	memset(&g_MRLink[connHandle],0,sizeof(MultiRoleApp_Link_t));
	g_MRLink[connHandle].state = GAPROLE_TERMINATED;

	LOG("Terminate connHandle %d,role %d,newState %d,reason %d\n",connHandle,role,newState,reason );

}

static void multiRoleRssiCB( uint16 connHandle, int8  rssi )
{
	LOG("RSSI CB handle %d,rssi %d\n",connHandle,rssi );
}

static void multiRoleSDPCB( uint16 connHandle )
{
	GattScanServer *pPkt;
	pPkt = GAPMultiRole_GetSDPIdx( connHandle );
	g_MRLink[connHandle].cccdHandle = MultiRole_FindCCCD( pPkt );
	g_MRLink[connHandle].SDPDone = TRUE;
	LOG("SDP Done handle %d,cccdHandle %d\n",connHandle,g_MRLink[connHandle].cccdHandle );	
	MultiRole_DisplaySDPInfo( pPkt );
}
static uint16 MultiRole_FindUUID_Handle( GattScanServer *pPkt , uint16 uuid )
{
	for(uint8 i=0;i<pPkt->PrimSerCnt;i++ )
	{
		for(uint8 k=0;k<pPkt->PrimServ[i].CharacNum;k++)
		{
			if( pPkt->PrimServ[i].Charac[k].charUuid == uuid )
			{
				LOG(" MultiRole_FindUUID_Handle success handle 0x%04X\n",pPkt->PrimServ[i].Charac[k].charStartHandle);
				return pPkt->PrimServ[i].Charac[k].charStartHandle;
			}
		}
	}
	return INVALID_STATUS;
}
static uint8_t MultiRole_WriteValue(uint16 connHandle , uint16 uuid,uint16 len,uint8 *pData)
{
	GattScanServer *pPkt;
	attWriteReq_t *pReq;
	bStatus_t status = FAILURE;

	pPkt = GAPMultiRole_GetSDPIdx( connHandle );
	uint16 charHandle = MultiRole_FindUUID_Handle( pPkt, uuid );

	pReq= osal_mem_alloc(sizeof(attWriteReq_t));
	if( pReq )
	{
		osal_memcpy(pReq->value, pData, len );
		pReq->sig = 0;
		pReq->cmd = 0;
		pReq->len = len;
		pReq->handle = charHandle + 1 ;
		status = GATT_WriteCharValue(connHandle, pReq, multiRole_TaskId );
		if(status == SUCCESS)
		{
			LOG("GATT_WriteCharValue success %d\n",connHandle);
		}
		else
		{
			LOG("GATT_WriteCharValue ERROR %d\r\n",status);
		}
		osal_mem_free(pReq);
	}
	else
	{
		LOG(" osal_mem_alloc ERROR \n");
	}
	return status;
}

static uint8 MultiRole_EnableCCCD( uint16 connHandle,uint16 uuid)
{
	attWriteReq_t *pReq;
	GattScanServer *pPkt;
	pPkt = GAPMultiRole_GetSDPIdx( connHandle );
	uint16 charHandle = MultiRole_FindUUID_Handle( pPkt, uuid );
	uint16 cccdHandle = 0;

	cccdHandle = charHandle + 2;
	pReq = osal_mem_alloc(sizeof(attWriteReq_t));
	pReq->sig = 0;
	pReq->cmd = 0;
	pReq->handle = cccdHandle;
	pReq->len = 2;
	pReq->value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
	pReq->value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
	bStatus_t status = GATT_WriteCharValue(connHandle, pReq, multiRole_TaskId);
	if(status == SUCCESS)
	{
		LOG("GATT_WriteCharValue Notify success handle %d\n",connHandle);
	}
	else
	{
		LOG("GATT_WriteCharValue Notify ERROR %d\r\n",status);
	}
	osal_mem_free(pReq);
	return status;
}

static void multiRoleAppRecvNotifyData( gattMsgEvent_t *pMsg )
{
	LOG("Notify connHanle %d,len %d:",pMsg->connHandle,pMsg->msg.handleValueNoti.len);
	if(pMsg->msg.handleValueNoti.len > 10)
	{
		for(unsigned char i = 0;i < 10; i++)
		{
			LOG( "%X,", pMsg->msg.handleValueNoti.value[i]);
		}
		LOG("\n");
	}
}

static uint16  MultiRole_FindCCCD( GattScanServer *pPkt )
{
	uint8 i=0;
	uint8 k=0;
	uint8 tmpHandle;
	for(i=0;i<pPkt->PrimSerCnt;i++ )
	{		
		for(k=0;k<pPkt->PrimServ[i].CharacNum;k++)
		{
			if( pPkt->PrimServ[i].Charac[k].Properties & GATT_PROP_NOTIFY )
			{
				tmpHandle = pPkt->PrimServ[i].Charac[k].charStartHandle;
				break;
			}				
		}
	}
	return ( tmpHandle + 2 );
}

static void  MultiRole_DisplaySDPInfo( GattScanServer *pPkt )
{
	for(uint8 i=0;i<pPkt->PrimSerCnt;i++ )
	{
		LOG("Service Idx %d/%d\n",i+1,pPkt->PrimSerCnt);
		LOG("Service UUID:");
		for(uint8 j=0;j<pPkt->PrimServ[i].UUID_Len;j++)
			LOG("0x%02X,",pPkt->PrimServ[i].UUID[j]);
		LOG("\n");
		
		for(uint8 k=0;k<pPkt->PrimServ[i].CharacNum;k++)
		{
			LOG("Characteristic Idx %d/%d\n",k+1,pPkt->PrimServ[i].CharacNum);
			LOG("	Char Handle 0x%04X\n",pPkt->PrimServ[i].Charac[k].charStartHandle);
			LOG("	Char uuid 0x%04X\n",pPkt->PrimServ[i].Charac[k].charUuid);
			LOG("	Char Properties 0x%04X\n",pPkt->PrimServ[i].Charac[k].Properties);
		}
	}
}


void MultiRole_DisplayADVData(uint8 scanCnt , MultiRoleADV_ScanData *pPkt )
{
	uint8 i;
	for( i = 0; i < scanCnt ; i++)
	{
		LOG( "Devices idx: %d/%d\n", i+1,scanCnt );
		if( pPkt[i].LocalName.Type )
		{
			LOG("LocalName.Type 0x%02X, Value:",pPkt[i].LocalName.Type);
			char name[31];
			osal_memcpy(name,pPkt[i].LocalName.Value,31);
			LOG(name);
			LOG("\n");
		}
		LOG("Flags %d\n",pPkt[i].Flags);
		LOG("AddrType 0x%02X,MAC Address Value: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",pPkt[i].AddrType,\
					pPkt[i].addr[0],\
					pPkt[i].addr[1],\
					pPkt[i].addr[2],\
					pPkt[i].addr[3],\
					pPkt[i].addr[4],\
					pPkt[i].addr[5]);
		LOG("rssi %d\n",pPkt[i].rssi);
		LOG("TxPower %d\n",pPkt[i].TxPower);
		if( pPkt[i].UUID.Type )
		{
			LOG("UUID.Type %d\n",pPkt[i].UUID.Type);
			for(unsigned char j=0;j<pPkt[i].UUID.Len;j++)
				LOG("0x%02X,",pPkt[i].UUID.Value[j]);
			LOG("\n");
		}
		if( pPkt[i].ConnIntvRange.ConnMin != 0)
		{
			LOG("ConnIntervalRange.ConnMin 0x%04X,Max 0x%04X\n",\
			pPkt[i].ConnIntvRange.ConnMin,pPkt[i].ConnIntvRange.ConnMax);
		}
		if( pPkt[i].ManufactData.Length )
		{
			LOG("ManufactureData Data :");
			for(uint8 k = 0; k< pPkt[i].ManufactData.Length;k++)
				LOG("0x%02X,",pPkt[i].ManufactData.Value[k]);
			LOG("\n");
		}
		LOG("\r\n");
	}
}

extern llConns_t           g_ll_conn_ctx;
static void MultiRole_PeriodProcess( void )
{
	uint16 i=0;
	static uint16 NotifyIdx=0;
	uint8 isAdd=FALSE;
	for( i=0;i < MAX_CONNECTION_NUM ; i++ )
	{
		if( g_MRLink[i].state == GAPROLE_CONNECTED )
		{
			if( ( g_MRLink[i].role == Master_Role ) && ( g_MRLink[i].SDPDone == TRUE ))
			{
				if( g_MRLink[i].enableCCCD == FALSE )
				{
					if( SUCCESS == MultiRole_EnableCCCD(i,0xFFF6 ))
					{
						g_MRLink[i].enableCCCD = TRUE;
					}
				}
				else if( g_MRLink[i].Char5isWrite == FALSE )
				{
					if( SUCCESS == MultiRole_WriteChar5Value(i) )
					{
						g_MRLink[i].Char5isWrite = TRUE;
					}
				}
			}
			else if( ( g_MRLink[i].role == Slave_Role ) )
			{
				if( !g_MRLink[i].dle )
				{
					HCI_LE_SetDataLengthCmd(i,251, 2120);
					g_MRLink[i].dle = TRUE;
				}
				else if( ( g_MRLink[i].notify == TRUE ) )
				{
					if( !isAdd )
					{
						NotifyIdx++;
						isAdd = TRUE;
						MRnotifyBuf[0]=HI_UINT16(NotifyIdx);
						MRnotifyBuf[1]=LO_UINT16(NotifyIdx);
						if( g_MRLink[i].notify )
						{
	                        LOG("MultiProfile_Notify \n");
							MultiProfile_Notify(i,MULTIPROFILE_CHAR6,ATT_GetCurrentMTUSize(i)-3,MRnotifyBuf);
						}
					}
				}
			}
		}
	}
}


static uint8 MultiRole_WriteChar5Value( uint16 connHandle)
{
	uint8 Val[3] = {0x00,0x80,0x01};
	uint8_t status = MultiRole_WriteValue(connHandle , 0xFFF5,3,Val);
	return status;
}

void hidDevStartIdleTimer( void )
{
  if ( pHidDevCfg->idleTimeout > 0 )
  {
    osal_start_timerEx( multiRole_TaskId, MULTIROLE_HID_IDLE_EVT, pHidDevCfg->idleTimeout );
  }
}

void hidDevStopIdleTimer( void )
{
  osal_stop_timerEx( multiRole_TaskId, MULTIROLE_HID_IDLE_EVT );
}

/*********************************************************************
 * @fn      hidKbdRptCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint16 *pLen, uint8 *pData )
{
  uint8 status = SUCCESS;

  // write
  if ( oper == HID_DEV_OPER_WRITE )
  {
    if ( uuid == REPORT_UUID )
    {
      // process write to LED output report; ignore others
      if ( type == HID_REPORT_TYPE_OUTPUT )
      {
        status = hidKbdRcvReport( *pLen, pData );
      }
    }

    if ( status == SUCCESS )
    {
      status = HidKbd_SetParameter( id, type, uuid, *pLen, pData );
    }
  }
  // read
  else if ( oper == HID_DEV_OPER_READ )
  {
    status = HidKbd_GetParameter( id, type, uuid, pLen, pData );
  }
  // notifications enabled
  else if ( oper == HID_DEV_OPER_ENABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = TRUE;
    }
  }
  // notifications disabled
  else if ( oper == HID_DEV_OPER_DISABLE )
  {
    if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
    {
      hidBootMouseEnabled = FALSE;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      hidKbdEvtCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void hidKbdEvtCB( uint8 evt )
{
  // process enter/exit suspend or enter/exit boot mode

  return;
}

/*********************************************************************
 * @fn      hidKbdRcvReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData )
{
  // verify data length
  if ( len == HID_LED_OUT_RPT_LEN )
  {
    // set keyfob LEDs
    //HalLedSet( HAL_LED_1, ((*pData & LED_CAPS_LOCK) == LED_CAPS_LOCK) );
    //HalLedSet( HAL_LED_2, ((*pData & LED_NUM_LOCK) == LED_NUM_LOCK) );

    return SUCCESS;
  }
  else
  {
    return ATT_ERR_INVALID_VALUE_SIZE;
  }
}

static void hidKbdSendReport( uint16 connHandle,uint8 keycode )
{
	AT_LOG("%s\n",__func__);

  uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = 0;         // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report( connHandle,HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf );
}


#endif
/*********************************************************************
*********************************************************************/
