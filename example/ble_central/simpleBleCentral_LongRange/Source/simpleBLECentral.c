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
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "ll_common.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile_ota.h"
#include "simpleBLECentral.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib_LR.h"
#include "rf_phy_driver.h"
/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  30

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 5000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      10

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      300

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           400

// Default passcode
#define DEFAULT_PASSCODE                      0//19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_NO_PAIRING//GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  FALSE //TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// add by zhufei.zhang 2018.10.25
#define PRIMARY_SERVICE_RES			10//0xFFFF
#define Characteristic_LEN					20
#define Characteristic_ValueIndex		1
#define Characteristic_NotifyIndex	2
#define Central_Test_ToDoList				6
/*********************************************************************
 * TYPEDEFS
 */
// add by zhufei.zhang 2018.10.25
// Advertising and Scan Response Data
typedef struct
{
	uint8_t		Type;
	uint8_t		Len;
	uint8_t 	Value[ATT_UUID_SIZE];
}SimpleADVServiceUUIDs;
typedef struct
{
	uint8_t		Type;
	uint8_t		Length;
	uint8_t 	Value[31];		// Max PDU = 31
}SimpleADVLoaclName;
typedef struct{
	uint16_t	ConnMin;
	uint16_t	ConnMax;
}SimpleADVSlaveInterval;
typedef struct
{
	uint8_t		Type;
	uint8_t		Len;
	uint8_t 	Value[ATT_UUID_SIZE];
}SimpleADVServiceSolicitation;
typedef struct
{
	uint8_t		Length;
	uint8_t 	Value[31];		// Max PDU = 31
}SimpleADVServiceDATA;
typedef struct
{
	uint8_t		Length;
	uint8_t 	Value[31];		// Max PDU = 31
}SimpleADVManufactureDATA;
typedef struct
{
	uint8_t													AddrType;
	uint8_t 												addr[B_ADDR_LEN];
	bool														RSP_ReadFlag;
	int8														rssi;
	int8													 	TxPower;
	uint8_t 												Flags;
	SimpleADVServiceUUIDs 					UUID;
	SimpleADVLoaclName							LocalName;
	uint8_t													OOB_Data;
	uint8_t													SM_TK_Value;
	SimpleADVSlaveInterval					ConnIntervalRange;
	SimpleADVServiceSolicitation		ServiceSolicitation;
	SimpleADVServiceDATA						ServiceData;
	SimpleADVManufactureDATA				ManufactureData;
}SimpleClientADV_ScanData;

// Service and Characteristic
typedef struct
{
	uint16_t  charStartHandle;
	uint16_t  charUuid;
	uint8_t		UUID_Len;
	uint8_t 	UUID[ATT_UUID_SIZE];
	uint8_t   Properties;
}SimpleCharacteristic;
typedef struct
{
	// Service Info , User Don't Care
	uint8_t 	Attribute_Data_Len;
	uint16_t 	Attribute_StartHandle;
	uint16_t 	End_Group_Handle;
	// User Care  
	// Caculate UUID Len
	uint8_t		UUID_Len;
	uint8_t 	UUID[ATT_UUID_SIZE];
	// Characteristic
	uint8_t		CharacNum;
	SimpleCharacteristic Characteristic[Characteristic_LEN];
}SimpleGATTReadGroupRsp;
typedef struct
{
	// Primary Service Num
	uint8_t 								PrimaryServiceCnt;
	// Primary Service Characteristic Index
	uint8_t									PrimaryCharacIndex;
	// Characteristic Find Index
	uint8_t									CharacFindIndex;
	SimpleGATTReadGroupRsp 	ServerGroupService[PRIMARY_SERVICE_RES];
}SimpleGattScanServer;

typedef struct
{
  uint32 toDoTick;
  uint16 toDoEvt;
}ctvToDoList_t;

typedef struct
{
    uint8 mtu;
    uint8 pduLen;
    uint8 pduLenSla;
    uint8 phyMode;
    uint8 phyModeSla;
    uint8 connIntv;
    uint8 latency;
    uint8 notfIntv;
    uint8 notfPktNum;
    uint32 testCnt;
    ctvToDoList_t ctvToDoList[Central_Test_ToDoList];

}centralTestVector_t;


typedef struct
{
    uint32 cnt;
    uint32 miss;
    uint32 err;
    uint32 isDone;
}ntfTest_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32 g_osal_mem_allo_cnt;
extern uint32 g_osal_mem_free_cnt;
extern l2capSARDbugCnt_t g_sarDbgCnt;
extern llGlobalStatistics_t g_pmCounters;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
//extern void dbg_time_log(void);
//extern void  osal_memory_statics(void *ptr,uint32* uBlkCnt,uint32* uBlkSize); 
/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;


// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

//// Discovered service start and end handle
//static uint16 simpleBLESvcStartHdl = 0;
//static uint16 simpleBLESvcEndHdl = 0;

//// Discovered characteristic handle
//static uint16 simpleBLECharHdl = 0;

// Value to write
static uint8 simpleBLECharVal = 0;

//// Value read/write toggle
//static bool simpleBLEDoWrite = FALSE;

//// GATT read/write procedure state
//static bool simpleBLEProcedureInProgress = FALSE;

static gapDevRec_t simpleBlePeerTarget; 
//static bool simBlePeerConnFlg=FALSE;
static uint8_t advDataFilterCnt;
static uint16 connIntv = 30;
static uint16 connLatency = 4;
static uint16 connTimeOut = 500;
static uint32 connEventCnt = 0;

static uint16 dleTxOctets=251;
static uint16 dleTxTime=2120;

static uint8 phyModeCtrl=0x01;

static uint16 dleTxOctetsSlave=251;
static uint16 dleTxTimeSlave=2120;

static uint8 phyModeCtrlSlave=0x01;
    
// add by zhufei.zhang 2018.10.25
static SimpleGattScanServer 			SimpleClientInfo;
static SimpleClientADV_ScanData		simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

#define RWN_TEST_VAL_LEN            20
static uint8 rwnTestCnt=0;
static uint8 rwnTestVal[RWN_TEST_VAL_LEN];

ntfTest_t ntfTest;
static uint16 mstWtCnt=0;
#define WTNR_TEST_VAL_LEN 251
uint8 wtnrTestVal[WTNR_TEST_VAL_LEN];
static uint8 slaCtrlCmd=0;
static uint16 wtnrInertvl=0;

static uint16 cTVIdx=0;
static uint16 cTVErrCnt = 0;
static uint16 TestData_Head = 0;
static uint8  TestUpdateFlag = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscoveryService( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( gapCentralRoleEvent_t *pEvent );
char *bdAddr2Str ( uint8 *pAddr );
static uint8_t simpleBLECentral_AdvDataFilterCBack(void);

// add by zhufei.zhang 
static void simpleBLEAnalysisADVDATA(uint8 Index,gapDeviceInfoEvent_t *pData);
static void simpleBLECentral_CharacteristicTest(void);
static void simpleBLECentral_DiscoverDevice(void);
static void simpleBLECentral_LinkDevice(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
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
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  
  GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, 20 );      //  * 1.25ms      // 30
  GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, 20 );
  
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING;//GAPBOND_PAIRING_MODE_INITIATE;//DEFAULT_PAIRING_MODE;    // GAPBOND_PAIRING_MODE_NO_PAIRING
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  for(uint8 i=0;i<RWN_TEST_VAL_LEN-1;i++)
    rwnTestVal[i]=i;

  for(uint8 i=0;i<WTNR_TEST_VAL_LEN-1;i++)
    wtnrTestVal[i]=i;


//  
  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes


  // scanner parameter init
  Gap_ResetPhyParams(PKT_FMT_BLR125K);
	llInitFeatureSetDLE(TRUE);
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );



//  dbg_time_log();

//  HCI_LE_AddWhiteListCmd(LL_DEV_ADDR_TYPE_PUBLIC,simpleBlePeerTarget.addr);

  simpleBlePeerTarget.addr[5]=0x2D;
  simpleBlePeerTarget.addr[4]=0xD5;
  simpleBlePeerTarget.addr[3]=0xDC;
  simpleBlePeerTarget.addr[2]=0x3E;
  simpleBlePeerTarget.addr[1]=0x1A;
  simpleBlePeerTarget.addr[0]=0xF4;
  
	  AT_LOG("[PEER ADDR]");
  for(int i=0;i<6;i++)
  {
//    simpleBlePeerTarget.addr[i]=ReadFlash(0x11004008+i);
    AT_LOG("%02x",simpleBlePeerTarget.addr[i]);
  }
  AT_LOG("\n");

  HCI_LE_AddWhiteListCmd(LL_DEV_ADDR_TYPE_PUBLIC,simpleBlePeerTarget.addr);
  
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
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
        {
            simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

        // Register with bond manager after starting device
        GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

        return ( events ^ START_DEVICE_EVT );
    }

	if( events & CENTRAL_INIT_DONE_EVT)
	{
		AT_LOG("BLE as central Init Done \r\n");

		// re-config g_rfPhyPktFmt, cause after disconnect , reset g_rfPhyPktFmt to 1Mbps
		Gap_ResetPhyParams(PKT_FMT_BLR125K);
		simpleBLECentral_DiscoverDevice();
		return ( events ^ CENTRAL_INIT_DONE_EVT );
	}
	
	if ( events & CENTRAL_DISCOVER_DEVDONE_EVT )
	{
		LOG("CENTRAL_DISCOVER_DEVDONE_EVT \n");
		TestData_Head = 0;
		TestUpdateFlag = FALSE;
		simpleBLECentral_LinkDevice();
		return ( events ^ CENTRAL_DISCOVER_DEVDONE_EVT );
	}
	
  if ( events & START_DISCOVERY_SERVICE_EVT )
  {
//      simpleBLECentralStartDiscoveryService( );
			osal_set_event(simpleBLETaskId, START_CHAR_DATA_TEST);

      return ( events ^ START_DISCOVERY_SERVICE_EVT );
  }
	
    if( events & START_CHAR_DATA_TEST )
	{
//		simpleBLECentral_CharacteristicTest();
			attWriteReq_t *pReq;
			pReq = osal_mem_alloc(sizeof(attWriteReq_t));
			pReq->sig = 0;
			pReq->cmd = 0;
			pReq->handle = 49; 
			pReq->len = 2;
			pReq->value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
			pReq->value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
			bStatus_t status = GATT_WriteCharValue(simpleBLEConnHandle, pReq, simpleBLETaskId);
			if(status == SUCCESS)
			{
				AT_LOG("Notify success pReq->handle %d\r\n",pReq->handle);
			}
			else
			{
				AT_LOG("Notify ERROR %d\r\n",status);
			}
			osal_mem_free(pReq);
			osal_start_timerEx(simpleBLETaskId, SBC_READ_WRITE_TEST_EVT, 1000);
			
		return ( events ^ START_CHAR_DATA_TEST);
	}
	if( events & SBC_PEER_CHECK_STATUS_EVT)
	{
		uint8 Val[2] = {0x01,0xFF};
		attWriteReq_t *pReq;
		pReq = osal_mem_alloc(sizeof(attWriteReq_t));
		pReq->sig = 0;
		pReq->cmd = 0;
		pReq->handle = 45; 
		pReq->len = 2;
		pReq->value[0] = Val[0];
		pReq->value[1] = Val[1];
		bStatus_t status = GATT_WriteCharValue(simpleBLEConnHandle, pReq, simpleBLETaskId);
		if(status == SUCCESS)
		{
			AT_LOG("Write success pReq->handle %d\r\n",pReq->handle);
		}
		else
		{
			AT_LOG("Write ERROR %d\r\n",status);
		}
		osal_mem_free(pReq);
		check_PerStatsProcess();
		return ( events ^ SBC_PEER_CHECK_STATUS_EVT );
	}
	if( events & SBC_PERIODIC_EVT )
	{
		if( !TestUpdateFlag )
		{
			if( TestData_Head > 0xC000)
			{
				TestUpdateFlag = TRUE;
				AT_LOG("++++++++++++++++UPDATED \n");
				ATT_SetMTUSizeMax(197);
				attExchangeMTUReq_t pReq;
				pReq.clientRxMTU = 197;
				uint8 status =GATT_ExchangeMTU(simpleBLEConnHandle,&pReq, simpleBLETaskId);
				GAPCentralRole_UpdateLink(simpleBLEConnHandle,80,80,0,500);
			}
			osal_start_timerEx( simpleBLETaskId, SBC_PERIODIC_EVT,500);
		}		
		return ( events ^ SBC_PERIODIC_EVT );
	}
    if ( events & SBC_CANCEL_CONN )
    {
        if (simpleBLEState != BLE_STATE_CONNECTED)   // not connected
        {
            GAPCentralRole_TerminateLink( simpleBLEConnHandle );
            LOG("Establish Link Time Out.\r\n");
        }
    
        return ( events ^ SBC_CANCEL_CONN );
    }

    if ( events & SBC_TERMINATED_CONN )
    {
        if (simpleBLEState == BLE_STATE_CONNECTED)   // not connected
        {
            GAPCentralRole_TerminateLink( simpleBLEConnHandle);
            
        }

        LOG("Terminated Link .s%d d%d\r\n",simpleBLEState,simpleBLEConnHandle);
        
        return ( events ^ SBC_TERMINATED_CONN );
    }
  
    if ( events & UPD_CHAN_MAP_EVENT )
    {
				uint8 chanMap[5] = {0xBF,0xDF,0xDF,0xDF,0x1F};
        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)  
        {
            uint8 ret = HCI_LE_SetHostChanClassificationCmd(chanMap );
            AT_LOG("CHAN MAP %d\r\n",ret);
        }
				osal_start_timerEx( simpleBLETaskId, SBC_PERIODIC_EVT,1500);
				osal_start_timerEx( simpleBLETaskId, UPD_CONN_PARAM  , 1000 );
        return ( events ^ UPD_CHAN_MAP_EVENT );
    }
  
    if ( events & UPD_CONN_PARAM )
    {
        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)  
        {
            connIntv=((connIntv+DEFAULT_UPDATE_MIN_CONN_INTERVAL)%DEFAULT_UPDATE_MAX_CONN_INTERVAL);
//            connLatency = (connTimeOut/(connIntv<<2));
						connLatency = 0;
            AT_LOG("UPD[ %2d %2d %2d %2d]\r\n",connIntv,connIntv,connLatency,connTimeOut);
            GAPCentralRole_UpdateLink(simpleBLEConnHandle,connIntv,connIntv,connLatency,connTimeOut);          
        }
				osal_start_timerEx( simpleBLETaskId, UPD_DATA_LENGTH_EVT  , 1000 );
        return ( events ^ UPD_CONN_PARAM );
    }

    if ( events & UPD_DATA_LENGTH_EVT )
    {
        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)  
        {
            HCI_LE_SetDataLengthCmd(simpleBLEConnHandle,dleTxOctets,dleTxTime );
            AT_LOG("DLE[ %2d %2d ]\r\n",dleTxOctets,dleTxTime);
        }
				osal_set_event(simpleBLETaskId, START_CHAR_DATA_TEST);
        return ( events ^ UPD_DATA_LENGTH_EVT );
    }

    if ( events & SBC_READ_WRITE_TEST_EVT )
    {
			uint8 Val[2] = {0x00,0xFF};
			attWriteReq_t *pReq;
			pReq = osal_mem_alloc(sizeof(attWriteReq_t));
			pReq->sig = 0;
			pReq->cmd = 0;
			pReq->handle = 45; 
			pReq->len = 2;
			pReq->value[0] = Val[0];
			pReq->value[1] = Val[1];
			bStatus_t status = GATT_WriteCharValue(simpleBLEConnHandle, pReq, simpleBLETaskId);
			if(status == SUCCESS)
			{
				AT_LOG("Write success pReq->handle %d\r\n",pReq->handle);
			}
			else
			{
				AT_LOG("Write ERROR %d\r\n",status);
			}
			osal_mem_free(pReq);
        return ( events ^ SBC_READ_WRITE_TEST_EVT );
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
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}


/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */

static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
    if ( simpleBLEState != BLE_STATE_CONNECTED )
    {
        // In case a GATT message came after a connection has dropped,
        // ignore the message
        return;
    }

    if(pMsg->hdr.status==bleTimeout)
    {
        AT_LOG("[GATT TO] %x\n",pMsg->method);
        return;
    }
		if ( ( pMsg->method == ATT_READ_RSP ) ||
         ( ( pMsg->method == ATT_ERROR_RSP ) &&
           ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
    {
        if ( pMsg->method == ATT_ERROR_RSP )
        {
            //uint8 status = pMsg->msg.errorRsp.errCode;
        }
        else
        {
            // After a successful read, display the read value
            //uint8 valueRead = pMsg->msg.readRsp.value[0];
        }
    }
    else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
               ( ( pMsg->method == ATT_ERROR_RSP ) &&
                 ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
    {
    
        if ( pMsg->method == ATT_ERROR_RSP )
        {
            //uint8 status = pMsg->msg.errorRsp.errCode;
      
            LOG( "Write Error: %d\r\n", pMsg->msg.errorRsp.errCode );
        }
        else
        {
            // After a succesful write, display the value that was written and increment value
            LOG( "Write sent: %d\r\n", simpleBLECharVal++);      
        }  

    }
		else if( pMsg->method == ATT_HANDLE_VALUE_NOTI  ||
               ( ( pMsg->method == ATT_ERROR_RSP ) &&
                 ( pMsg->msg.errorRsp.reqOpcode == ATT_HANDLE_VALUE_NOTI ) ) )
	{
        if ( pMsg->method == ATT_ERROR_RSP || 
            pMsg->msg.handleValueNoti.len > ATT_GetCurrentMTUSize()-3 )
        {
            //uint8 status = pMsg->msg.errorRsp.errCode;
      
            AT_LOG( "Ntf Error: %d\r\n", pMsg->msg.errorRsp.errCode );
        }
        else
        {
					uint8 *pCurValue = (uint8 *)pMsg->msg.handleValueNoti.value;   
					uint8 len = pMsg->msg.handleValueNoti.len;

					uint16 cntHead=BUILD_UINT16(pCurValue[1], pCurValue[0]);
					uint16 cntTail=BUILD_UINT16(pCurValue[len-1], pCurValue[len-2]);
					
					AT_LOG("T:%d,len :%d\n",cntHead,len);
					if( TestData_Head > cntHead )
					{
						osal_set_event(simpleBLETaskId,SBC_PEER_CHECK_STATUS_EVT);
					}
					TestData_Head = cntHead;
        }
	
	}
	else //if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
	{
//			simpleBLEGATTDiscoveryEvent( pMsg );
	}
  
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LOG( "RSSI -dB: %d\r\n", (uint8) (-rssi) );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
//    static int try_num = 0;
    
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
		{
			osal_set_event(simpleBLETaskId,CENTRAL_INIT_DONE_EVT);
		}
		break;

    case GAP_DEVICE_INFO_EVENT:
		{
					// only save connectable adv
//					if (pEvent->deviceInfo.eventType != GAP_ADRPT_ADV_IND
//					 && pEvent->deviceInfo.eventType != GAP_ADRPT_ADV_DIRECT_IND
//					 && pEvent->deviceInfo.eventType != GAP_ADRPT_SCAN_RSP)
//							break;
					simpleBLEAddDeviceInfo( pEvent );
		}
		break;
    
    case GAP_DEVICE_DISCOVERY_EVENT:
    {
        // discovery complete
        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;
        for(unsigned i = 0; i < simpleBLEScanRes ; i++)
        {
        	AT_LOG( "Devices Found: %d/%d\n", i+1,simpleBLEScanRes );
        	if( simpleBLEDevList[i].LocalName.Type )
        	{
        		AT_LOG("simpleBLEDevList.LocalName.Type 0x%02X, Value:",simpleBLEDevList[i].LocalName.Type);
        		char name[31];
        		osal_memcpy(name,simpleBLEDevList[i].LocalName.Value,31);
        		AT_LOG(name);
        		AT_LOG("\n");
        	}
        	AT_LOG("simpleBLEDevList.Flags %d\n",simpleBLEDevList[i].Flags);
        	AT_LOG("simpleBLEDevList.AddrType 0x%02X,MAC Address Value: 0x%02X,0x%02X,\
        				0x%02X,0x%02X,0x%02X,0x%02X\n",simpleBLEDevList[i].AddrType,\
        				simpleBLEDevList[i].addr[0],\
        				simpleBLEDevList[i].addr[1],\
        				simpleBLEDevList[i].addr[2],\
        				simpleBLEDevList[i].addr[3],\
        				simpleBLEDevList[i].addr[4],\
        				simpleBLEDevList[i].addr[5]);
        	AT_LOG("simpleBLEDevList.rssi %d\n",simpleBLEDevList[i].rssi);
        	AT_LOG("simpleBLEDevList.TxPower %d\n",simpleBLEDevList[i].TxPower);
        	if( simpleBLEDevList[i].UUID.Type )
        	{
        		AT_LOG("simpleBLEDevList.UUID.Type %d\n",simpleBLEDevList[i].UUID.Type);
        		for(unsigned char j=0;j<simpleBLEDevList[i].UUID.Len;j++)
        			AT_LOG("0x%02X,",simpleBLEDevList[i].UUID.Value[j]);
        		AT_LOG("\n");
        	}
        	if( simpleBLEDevList[i].ConnIntervalRange.ConnMin != 0)
        	{
        		AT_LOG("simpleBLEDevList.ConnIntervalRange.ConnMin 0x%04X,Max 0x%04X\n",\
        		simpleBLEDevList[i].ConnIntervalRange.ConnMin,simpleBLEDevList[i].ConnIntervalRange.ConnMax);
        	}
        	if( simpleBLEDevList[i].ManufactureData.Length )
        	{
        		AT_LOG("simpleBLEDevList.ManufactureData Data :");
        		for(unsigned char k = 0; k< simpleBLEDevList[i].ManufactureData.Length;k++)
        			AT_LOG("0x%02X,",simpleBLEDevList[i].ManufactureData.Value[k]);
        		AT_LOG("\n");
        	}
        	AT_LOG("\r\n\r\n");
        }
        osal_set_event(simpleBLETaskId,CENTRAL_DISCOVER_DEVDONE_EVT);
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
		{
			g_rfPhyPktFmt = PKT_FMT_BLR125K;
			AT_LOG("\n== GAP_LINK_ESTABLISHED_EVENT ==\r\n");
			if ( pEvent->gap.hdr.status == SUCCESS )
			{
				simpleBLEState = BLE_STATE_CONNECTED;
				simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;  
				
				osal_start_timerEx( simpleBLETaskId, UPD_CHAN_MAP_EVENT  , 1000 );
			    
			}
			else if(pEvent->gap.hdr.status==LL_STATUS_WARNING_WAITING_LLIRQ)
			{
        AT_LOG( "[WAITING LL_IRQ]. " );
				AT_LOG( "Reason: 0x%02x\r\n", pEvent->gap.hdr.status);
				GAPCentralRole_EstablishLink(   DEFAULT_LINK_HIGH_DUTY_CYCLE,\
																		DEFAULT_LINK_WHITE_LIST,\
																		simpleBlePeerTarget.addrType, \
																		simpleBlePeerTarget.addr );
			}
			else
			{
				simpleBLEState = BLE_STATE_IDLE;
				simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
				simpleBLEDiscState = BLE_DISC_STATE_IDLE;


				osal_start_timerEx( simpleBLETaskId, CENTRAL_INIT_DONE_EVT, \
														1000 );
			}

//			osal_start_timerEx(simpleBLETaskId, SBC_SEND_RAW_DATA, 40* 1000);
//        osal_start_reload_timer(simpleBLETaskId, SBC_SEND_RAW_DATA, 8);
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
				g_rfPhyPktFmt = PKT_FMT_BLR125K;
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
				AT_LOG("\n== GAP_LINK_TERMINATED_EVENT ==\r\n");
//        uint32 uOS_size,uOS_cnt;
//        osal_memory_statics(NULL, &uOS_cnt,&uOS_size);
        
//        AT_LOG("[TVEC] %08x DISC.R[0x%2x][%04x %04x]\n",getMcuPrecisionCount(),pEvent->linkTerminate.reason,uOS_cnt,uOS_size);
//        check_PerStatsProcess();

//        AT_LOG("[PMCNT] rdErr %d rstErr %d trgErr %d\n",g_pmCounters.ll_rfifo_read_err,g_pmCounters.ll_rfifo_rst_err,g_pmCounters.ll_trigger_err);
      
					// Terminate Link , memset SimpleGattScanServer structure
					osal_memset(&SimpleClientInfo,0,sizeof(SimpleGattScanServer));
					osal_memset(simpleBLEDevList,0,sizeof(SimpleClientADV_ScanData)*DEFAULT_MAX_SCAN_RES);
					
					osal_start_timerEx( simpleBLETaskId, CENTRAL_INIT_DONE_EVT, \
															10*1000 );
 

      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			LOG( "Server Request for Param Update\r\n");
		}
		break;
      
    default:
			LOG(" simpleBLECentralEventCB --> pEvent->gap.opcode: 0x%02X\r\n", pEvent->gap.opcode);
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
	LOG("simpleBLECentralPairStateCB in param state 0x%02X,status 0x%02X\r\n",state,status);
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LOG( "Pairing started\n" );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LOG( "Pairing success\n" );
    }
    else
    {
      LOG( "Pairing fail\n" );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LOG( "Bonding success\n" );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
    LOG("simpleBLECentralPasscodeCB\r\n");
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscoveryService
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscoveryService( void )
{
	LOG("==>simpleBLECentralStartDiscoveryService\r\n");
	
	// add by zhufei.zhang 2018.10.25
	// before start discovery , memset the variable
	osal_memset(&SimpleClientInfo,0,sizeof(SimpleGattScanServer));
	
//  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
//                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
	GATT_DiscAllPrimaryServices(simpleBLEConnHandle,simpleBLETaskId);
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
//	attReadByTypeReq_t req;
    if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
    {
    	// add by zhufei.zhang 2018/10/24
        if( ( pMsg->method ==  ATT_READ_BY_GRP_TYPE_RSP) && 
        			( pMsg->msg.readByGrpTypeRsp.numGrps > 0 ) )
        {
    		for(unsigned char i = 0 ; i < pMsg->msg.readByGrpTypeRsp.numGrps;i++)
    		{
    			// Current Attribute LEN
    			SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].Attribute_Data_Len = \
    																		pMsg->msg.readByGrpTypeRsp.len;
    			// Current Attribute Handle
    			SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].Attribute_StartHandle = \
    						BUILD_UINT16(	pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i], \
    						pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 1]);
    			// Current Attribute End Group Handle
    			SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].End_Group_Handle = \
    						BUILD_UINT16(	pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i+2], \
    						pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 3]);
    			// Caculate Primary Service UUID Length
    			SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].UUID_Len = \
    						pMsg->msg.readByGrpTypeRsp.len - 4;
    			// Copy UUID
    			osal_memcpy(SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].UUID,\
    									&(pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 4]),\
    									SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].UUID_Len);
    			
    			// Primary Service Count Index ++
    			SimpleClientInfo.PrimaryServiceCnt++;
    		}
        }
        else if ( ( pMsg->method == ATT_READ_BY_GRP_TYPE_RSP  && 
        						 pMsg->hdr.status == bleProcedureComplete ) ||
        					 ( pMsg->method == ATT_ERROR_RSP ) )
        {			
            // Primary Service Discover OK , Prepare Discover Characteristic
        	simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        	if( SimpleClientInfo.PrimaryServiceCnt > 0 )
        	{
        	    // Characteristic Find Index Init
        		SimpleClientInfo.PrimaryCharacIndex = 0;
        		SimpleClientInfo.CharacFindIndex = 0;
        		GATT_DiscAllChars(  simpleBLEConnHandle,
        		                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Attribute_StartHandle,\
        		                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].End_Group_Handle, 
        		                    simpleBLETaskId );
        	}
        }
    }
    else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
    {
        // Characteristic found, store handle
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
             pMsg->msg.readByTypeRsp.numPairs > 0 )
        {
    			// Iterate through all three pairs found.
    			for(unsigned char i = 0; i < pMsg->msg.readByTypeRsp.numPairs ; i++)
    			{
    				// Extract the starting handle, ending handle, and UUID of the current characteristic.
    				// characteristic Handle
    				SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[SimpleClientInfo.CharacFindIndex].charStartHandle = \
    						BUILD_UINT16(	pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i], \
    													pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 1]);
    				
    				// Characteristic Properties
    				SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[SimpleClientInfo.CharacFindIndex].Properties = \
    						pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 2];
    				
    				// Characteristic UUID
    				SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[SimpleClientInfo.CharacFindIndex].charUuid = \
    				BUILD_UINT16(	pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 5], \
    																				pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 6]);
    				
    				SimpleClientInfo.CharacFindIndex++;
    			}
        }
		else if(( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
							 pMsg->hdr.status == bleProcedureComplete ) ||
						 ( pMsg->method == ATT_ERROR_RSP ))
		{
			SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].CharacNum = \
																					SimpleClientInfo.CharacFindIndex;
			SimpleClientInfo.CharacFindIndex = 0;
			
			AT_LOG(" Service 0x%02X%02X Characteristic Find Success \r\n",\
					SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].UUID[1],\
					SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].UUID[0]);
			for(unsigned char i = 0; i< SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].CharacNum; i++)
			{
				AT_LOG("Chars found handle is is:0x%04X,Properties is: 0x%02X,uuid is:0x%04X\r\n", \
				SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[i].charStartHandle,\
				SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[i].Properties,\
				SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[i].charUuid);
			}
			
			if(++SimpleClientInfo.PrimaryCharacIndex < SimpleClientInfo.PrimaryServiceCnt)
			{
				GATT_DiscAllChars(  simpleBLEConnHandle, 
				                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Attribute_StartHandle,\
									SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].End_Group_Handle, 
									simpleBLETaskId );
			}
			else
			{
				AT_LOG("All Characteristic Discover Success \r\n");
				simpleBLEDiscState = BLE_DISC_STATE_IDLE;
				//osal_start_timerEx( simpleBLETaskId, START_CHAR_DATA_TEST,100 );
//				osal_start_timerEx( simpleBLETaskId, SBC_READ_WRITE_TEST_EVT,100 );
			}
		}
    }
	else
	{
		AT_LOG("simpleBLEDiscState = BLE_DISC_STATE_IDLE \r\n");
	}
}

/*********************************************************************
 * @fn      simpleBLECentral_CharacteristicTest
 *
 * @brief   Send Data To BLE Server
 *
 * @return  none
 */

static void simpleBLECentral_CharacteristicTest(void)
{
	attWriteReq_t *pReq;
	attReadReq_t *pReqread;
	uint8 Val[20] = {0x00,0xC8,0x14};
	
	static uint8_t PrimaryServiceCnt = 0;
	static uint8_t PrimaryCharacIndex = 0;
	static uint8_t Properties = 0;
	static uint8_t SameCharacBusy = FALSE;
	
	
	if( PrimaryServiceCnt >= SimpleClientInfo.PrimaryServiceCnt )
		return;
	
	if( !SameCharacBusy )
	{
		if( PrimaryServiceCnt < SimpleClientInfo.PrimaryServiceCnt )
		{
			if( PrimaryCharacIndex < SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].CharacNum )
			{
				Properties = SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].Characteristic[PrimaryCharacIndex].Properties;
//				LOG("Should Update Properties Value \r\n");
				SameCharacBusy = TRUE;
			}
			else
			{
				PrimaryServiceCnt++;
				PrimaryCharacIndex = 0;
				osal_start_timerEx( simpleBLETaskId, START_CHAR_DATA_TEST,100 );
				return;
			}
		}
	}
	LOG("Service UUID : 0x%02X%02X, Characteristic UUID:0x%04X ,PrimaryServiceCnt %d,PrimaryCharacIndex %d,Properties 0x%02X\r\n",\
									SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].UUID[1],\
									SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].UUID[0],\
									SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].Characteristic[PrimaryCharacIndex].charUuid,PrimaryServiceCnt,PrimaryCharacIndex,Properties);
 	
    if( ( Properties & GATT_PROP_BCAST ) || \
		( Properties & GATT_PROP_WRITE_NO_RSP ) || \
		( Properties & GATT_PROP_INDICATE ) || \
		( Properties & GATT_PROP_AUTHEN ) || \
		( Properties & GATT_PROP_EXTENDED ))
	{
		LOG("Come Here \r\n");
		Properties &= ~( 	GATT_PROP_BCAST | GATT_PROP_WRITE_NO_RSP | GATT_PROP_INDICATE | GATT_PROP_AUTHEN | \
											GATT_PROP_EXTENDED);
		osal_start_timerEx( simpleBLETaskId, START_CHAR_DATA_TEST,100 );
	}
	else if( Properties & GATT_PROP_READ )
	{
		Properties &= ~GATT_PROP_READ;
		pReqread = osal_mem_alloc(sizeof(attReadReq_t));
		pReqread->handle = SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].\
		                    Characteristic[PrimaryCharacIndex].charStartHandle + Characteristic_ValueIndex;
		bStatus_t status = GATT_ReadCharValue( simpleBLEConnHandle, pReqread, simpleBLETaskId );
		if(status == SUCCESS)
		{
			LOG("GATT_ReadCharDesc success \r\n");
		}
		else
		{
			LOG("GATT_ReadCharDesc ERROR %d\r\n",status);
        }
		osal_mem_free(pReqread);
	}
	else if( Properties & GATT_PROP_WRITE )
	{
		Properties &= ~GATT_PROP_WRITE;
		pReq = osal_mem_alloc(sizeof(attWriteReq_t));
		osal_memcpy(pReq->value, Val, 3);
		pReq->sig = 0;
		pReq->cmd = 0;
		pReq->handle = SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].\
												Characteristic[PrimaryCharacIndex].charStartHandle + Characteristic_ValueIndex;
		if( PrimaryServiceCnt == 3 && PrimaryCharacIndex == 0 )
			pReq->len = 16;
		else if( PrimaryServiceCnt == 3 && (PrimaryCharacIndex > 0 && PrimaryCharacIndex < 3) )
			pReq->len = 2;
		else if( PrimaryServiceCnt == 3 && PrimaryCharacIndex == 3 )
			pReq->len = 1;
		else
			pReq->len = 3;
		bStatus_t status = GATT_WriteCharValue(simpleBLEConnHandle, pReq, simpleBLETaskId);
		if(status == SUCCESS)
		{
			LOG("GATT_WriteCharValue success \r\n");
        }
		else
		{
			LOG("GATT_WriteCharValue ERROR %d\r\n",status);
        }
		osal_mem_free(pReq);
	}
	else if( Properties & GATT_PROP_NOTIFY )
	{
		Properties &= ~GATT_PROP_NOTIFY;
		pReq = osal_mem_alloc(sizeof(attWriteReq_t));
		pReq->sig = 0;
		pReq->cmd = 0;
		pReq->handle = SimpleClientInfo.ServerGroupService[PrimaryServiceCnt].\
												Characteristic[PrimaryCharacIndex].charStartHandle + Characteristic_NotifyIndex;
		pReq->len = 2;
		pReq->value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
		pReq->value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
		bStatus_t status = GATT_WriteCharValue(simpleBLEConnHandle, pReq, simpleBLETaskId);
		if(status == SUCCESS)
		{
			LOG("GATT_WriteCharValue Notify success \r\n");
        }
		else
		{
			LOG("GATT_WriteCharValue Notify ERROR %d\r\n",status);
		}
		osal_mem_free(pReq);
	}
	else
	{
		PrimaryCharacIndex++;
		SameCharacBusy = FALSE;
		osal_start_timerEx( simpleBLETaskId, START_CHAR_DATA_TEST,100 );
	}
}

/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
    
    LOG("simpleBLEFindSvcUuid\r\n");
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
      
      LOG("==>adLen = %d ", adLen);
    if ( adLen > 0 )
    {
      adType = *pData;
      
        LOG("adType = %d ", adType);
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
    LOG("\r\n");
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( gapCentralRoleEvent_t *pEvent )
{
	const uint8_t ZeroBuf[B_ADDR_LEN]={0,0,0,0,0,0};
	uint8_t Addr_Index=0;

    if(	( pEvent->deviceInfo.opcode != GAP_DEVICE_INFO_EVENT ) || \
			( simpleBLEScanRes >= DEFAULT_MAX_SCAN_RES))
		return;
	// retrieve the ADDR Index
	for(uint8_t i=0;i < DEFAULT_MAX_SCAN_RES;i++)
	{
		if( ( osal_memcmp( simpleBLEDevList[i].addr, ZeroBuf , B_ADDR_LEN ) ) || \
				( osal_memcmp( pEvent->deviceInfo.addr, simpleBLEDevList[i].addr , B_ADDR_LEN )))
		{
			Addr_Index = i;
			break;
		}
	}
	if( Addr_Index < simpleBLEScanRes)
	{
		// update simpleBLEDevList[Addr_Index]
		simpleBLEDevList[Addr_Index].rssi = pEvent->deviceInfo.rssi;
		if( !simpleBLEDevList[Addr_Index].RSP_ReadFlag )
		{
			if( pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP )
			{
				// Analysis Scan RSP data
				simpleBLEAnalysisADVDATA(Addr_Index, &(pEvent->deviceInfo));
				simpleBLEDevList[Addr_Index].RSP_ReadFlag = TRUE;
			}
		}
	}
	else
	{
		// first commit simpleBLEDevList[Addr_Index]
		simpleBLEDevList[Addr_Index].rssi = pEvent->deviceInfo.rssi;
		// Mac Address Type
		simpleBLEDevList[Addr_Index].AddrType = pEvent->deviceInfo.addrType;
		// Mac Address
		osal_memcpy( simpleBLEDevList[Addr_Index].addr, pEvent->deviceInfo.addr, B_ADDR_LEN );
		if( pEvent->deviceInfo.eventType == GAP_ADRPT_ADV_IND )
		{
			// Analysis Adv data
			simpleBLEAnalysisADVDATA(Addr_Index, &(pEvent->deviceInfo));
		}
		// Increment scan result count
        simpleBLEScanRes++;
	}
}

/*********************************************************************
 * @fn      simpleBLEAnalysisADVDATA
 *
 * @brief   Analysis received Advertising data and SCAN RSP Data
 *
 * @return  none
 */
static void simpleBLEAnalysisADVDATA(uint8 Index,gapDeviceInfoEvent_t *pData)
{
	int8 DataLength;
	int8 New_ADStructIndex = 0;
	int8 AD_Length = 0;
	uint8_t AD_Type;
	
	DataLength = pData->dataLen;
	while(DataLength)
	{
		New_ADStructIndex += AD_Length;
		// DATA FORMAT : Length + AD Type + AD Data
		AD_Length = pData->pEvtData[pData->dataLen - DataLength];
		AD_Type = pData->pEvtData[New_ADStructIndex+1];

		if(AD_Length<2 || AD_Length>0x1f)
		{
		    AT_LOG("[AD_TYPE] ERR %02x %02x\n",AD_Type,AD_Length);
		    break;
        }
		switch(AD_Type)
		{
			case GAP_ADTYPE_FLAGS:
				simpleBLEDevList[Index].Flags = pData->pEvtData[New_ADStructIndex+2];
			break;
			case GAP_ADTYPE_16BIT_MORE:
			case GAP_ADTYPE_16BIT_COMPLETE:
			case GAP_ADTYPE_32BIT_MORE:
			case GAP_ADTYPE_32BIT_COMPLETE:
			case GAP_ADTYPE_128BIT_MORE:
			case GAP_ADTYPE_128BIT_COMPLETE: 
				simpleBLEDevList[Index].UUID.Type = AD_Type;
				while( AD_Length - 1 - simpleBLEDevList[Index].UUID.Len)
				{
					if( ( AD_Type == GAP_ADTYPE_16BIT_MORE) || ( AD_Type == GAP_ADTYPE_16BIT_COMPLETE))
					{
						simpleBLEDevList[Index].UUID.Len += ATT_BT_UUID_SIZE;
					}
					else if( ( AD_Type == GAP_ADTYPE_32BIT_MORE) || ( AD_Type == GAP_ADTYPE_32BIT_COMPLETE))
					{
						simpleBLEDevList[Index].UUID.Len += ATT_BT_UUID_SIZE<<1;
					}
					else
					{
						simpleBLEDevList[Index].UUID.Len += ATT_UUID_SIZE;
					}

					if(AD_Length< 1 + simpleBLEDevList[Index].UUID.Len)
					{
                        AT_LOG("[AD_TYPE] ERR %02x %02x\n",AD_Type,AD_Length);
		                break;
					}
				}
				osal_memcpy(	simpleBLEDevList[Index].UUID.Value,\
												&(pData->pEvtData[New_ADStructIndex+2]),\
												simpleBLEDevList[Index].UUID.Len);
			break;
			case GAP_ADTYPE_LOCAL_NAME_SHORT:
			case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
				simpleBLEDevList[Index].LocalName.Type = AD_Type;
				simpleBLEDevList[Index].LocalName.Length = AD_Length - 2;
				osal_memcpy(	simpleBLEDevList[Index].LocalName.Value,&(pData->pEvtData[New_ADStructIndex+2]),\
											simpleBLEDevList[Index].LocalName.Length);
			break;
			case GAP_ADTYPE_POWER_LEVEL:
				simpleBLEDevList[Index].TxPower = pData->pEvtData[New_ADStructIndex+2];
			break;
			case GAP_ADTYPE_OOB_CLASS_OF_DEVICE:
			case GAP_ADTYPE_OOB_SIMPLE_PAIRING_HASHC:
			case GAP_ADTYPE_OOB_SIMPLE_PAIRING_RANDR:
				
			break;
			case GAP_ADTYPE_SM_TK:
				
			break;
			case GAP_ADTYPE_SM_OOB_FLAG:
				
			break;
			case GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE:
				simpleBLEDevList[Index].ConnIntervalRange.ConnMin = BUILD_UINT16(pData->pEvtData[New_ADStructIndex+2],\
																																				 pData->pEvtData[New_ADStructIndex+3]);
				simpleBLEDevList[Index].ConnIntervalRange.ConnMax = BUILD_UINT16(pData->pEvtData[New_ADStructIndex+4],\
																																				 pData->pEvtData[New_ADStructIndex+5]);
			break;
			case GAP_ADTYPE_SIGNED_DATA:
				
			break;
			case GAP_ADTYPE_SERVICES_LIST_16BIT:
			case GAP_ADTYPE_SERVICES_LIST_128BIT:
				simpleBLEDevList[Index].ServiceSolicitation.Type = AD_Type;
				if( AD_Type ==  GAP_ADTYPE_SERVICES_LIST_16BIT)
					simpleBLEDevList[Index].ServiceSolicitation.Len = ATT_BT_UUID_SIZE;
				else
					simpleBLEDevList[Index].ServiceSolicitation.Len = ATT_UUID_SIZE;
				osal_memcpy( 	simpleBLEDevList[Index].ServiceSolicitation.Value,\
											&(pData->pEvtData[New_ADStructIndex+2]),\
											simpleBLEDevList[Index].ServiceSolicitation.Len);
			break;
			case GAP_ADTYPE_SERVICE_DATA:
				
			break;
			case GAP_ADTYPE_APPEARANCE:
				
			break;
			case GAP_ADTYPE_MANUFACTURER_SPECIFIC:
				simpleBLEDevList[Index].ManufactureData.Length = AD_Length - 1;
				osal_memcpy(simpleBLEDevList[Index].ManufactureData.Value,&(pData->pEvtData[New_ADStructIndex+2]),\
											simpleBLEDevList[Index].ManufactureData.Length);
			break;
			default:
				break;
		}
		AD_Length++;
		DataLength -= AD_Length;
	}
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
//  *pStr++ = '0';
//  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}


uint8_t simpleBLECentral_AdvDataFilterCBack(void)
{
    uint8_t* advData;
   
    advData=LL_PLUS_GetAdvDataExtendData();

    if(advData[11]==0x4c && advData[12]==0x00)
    {
        advDataFilterCnt++;
        LL_PLUS_SetScanRequestData(1, &advDataFilterCnt);
        return 1;
    }
    else
    {
        return 0;
    }
    
}

static void simpleBLECentral_DiscoverDevice(void)
{
	simpleBLEScanRes = simpleBLEScanIdx = 0;
	osal_memset(simpleBLEDevList,0,sizeof(SimpleClientADV_ScanData)*DEFAULT_MAX_SCAN_RES);
	
	bStatus_t stu = GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
																 TRUE/*DEFAULT_DISCOVERY_ACTIVE_SCAN*/,      // passive scan
																 FALSE/*DEFAULT_DISCOVERY_WHITE_LIST*/ );
	AT_LOG("simpleBLECentral_DiscoverDevice Return Value :%d\n",stu);
}

static void simpleBLECentral_LinkDevice(void)
{
	int8 Index=-1;
	
	if ( simpleBLEScanRes > 0 )
	{
		for(uint8_t i = 0; i< simpleBLEScanRes;i++)
		{
			if(osal_memcmp(simpleBLEDevList[i].addr,simpleBlePeerTarget.addr, B_ADDR_LEN))
			{
				Index = i;
				break;
			}
		}
	}
	
	AT_LOG("simpleBLEScanRes %d,Index %d\r\n",simpleBLEScanRes,Index);
	if( Index >= 0)
	{
		AT_LOG("Start EstablishLink \r\n");
		simpleBLEState = BLE_STATE_CONNECTING;
		// Note: if the peer addrType is not PUBLIC, the MSB of peerAddr will be adjusted by function gapAddAddrAdj()
		GAPCentralRole_EstablishLink(   DEFAULT_LINK_HIGH_DUTY_CYCLE,\
																		DEFAULT_LINK_WHITE_LIST,\
																		simpleBLEDevList[Index].AddrType, \
																		simpleBLEDevList[Index].addr );
	}
	else
	{
		AT_LOG("simpleBlePeerTarget Device Not found \r\n");
		osal_start_timerEx( simpleBLETaskId, CENTRAL_INIT_DONE_EVT, \
														2*1000 );
	}
}

/*********************************************************************
*********************************************************************/

