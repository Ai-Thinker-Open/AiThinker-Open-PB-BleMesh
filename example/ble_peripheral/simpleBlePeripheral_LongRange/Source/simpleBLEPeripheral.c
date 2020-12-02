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

/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
//#include "OnBoard.h"
#include "gatt.h"
#include "hci.h"
#include "log.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "sbpProfile_ota.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "flash.h"
#include "simpleBLEPeripheral.h"
#include "ll.h"
#include "clock.h"
#include "common.h"
#include "rflib_LR.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0
 

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     24//32//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800//48//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0//0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE //TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Default passcode
#define DEFAULT_PASSCODE                      0//19655

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
 typedef struct
 {
	 uint8_t scan_mac[B_ADDR_LEN];
	 int8_t rssi;
 }Scan_Info;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern wtnrTest_t wtnrTest;
 
extern volatile uint8_t g_current_advType;
extern uint16 g_conn_param_foff;
extern uint8 g_conn_param_carrSens;
extern uint32 counter_tracking;

extern uint32 g_counter_traking_avg;
extern uint32 g_counter_traking_cnt;
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t  g_osal_tick_trim;
extern uint32_t  g_TIM2_IRQ_PendingTick;
extern uint32_t  g_TIM2_wakeup_delay;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;
static  uint8_t notifyBuf[256];
static uint16 notifyInterval = 0;
static uint8 notifyPktNum = 0;
static uint8 connEvtEndNotify =0;
static uint16 notifyCnt = 0x15A0;

#if (DBG_RTC_TEST==1)
static uint32 testRtcCnt=0;
static uint32 testRtcCntLast=0;
static uint32 testRtcCntHigh=0;
#endif
	
Scan_Info g_scan_info;


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x12,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x50,   // 'P'
    0x48,   // 'H'
    0x59,   // 'Y'
    0x2b,   // '+'
    0x41,   // 'A'
    0x32,   // '2'
    0x2d,   // '-'
	0x20,	// ''
	0x2d,	// '-'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
  

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


// advert data for iBeacon
static uint8 advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x1A, // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
    0x4c, // Company ID - Fixed
    0x00, // Company ID - Fixed
    0x02, // Data Type - Fixed
    0x15, // Data Length - Fixed
    0xFD, // UUID  
    0xA5, // UUID 
    0x06, // UUID
    0x93, // UUID
    0xA4, // UUID
    0xE2, // UUID
    0x4F, // UUID
    0xB1, // UUID
    0xAF, // UUID
    0xCF, // UUID
    0xC6, // UUID
    0xEB, // UUID
    0x07, // UUID
    0x64, // UUID
    0x78, // UUID
    0x25, // UUID
    0x27, // Major
    0x74, // Major
    0x6b,//0x04, // Minor
    0xed,//0xb0, // Minor
    0xc5 // Power - The 2's complement of the calibrated Tx Power
};


static uint8 otaAdvIntv         = 100;      //unit is 10ms
static uint8 otaConnIntvMax     = DEFAULT_DESIRED_MIN_CONN_INTERVAL>>2;        //unit is 5ms
static uint8 otaConnIntvMin     = DEFAULT_DESIRED_MAX_CONN_INTERVAL>>2;        //uiit is 5ms
static uint8 otaConnIntvLatency = DEFAULT_DESIRED_SLAVE_LATENCY;        //
static uint8 otaConnTimeOut     = DEFAULT_DESIRED_CONN_TIMEOUT/100;        //unit is second
static uint8 sysClockChangeFlg  = 0;

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PHY+A2- -FFFFFF ";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void updateAdvData(void);
static void peripheralStateReadRssiCB( int8 rssi  );
char *bdAddr2Str( uint8 *pAddr );
static uint8_t simpleBLEPeripheral_ScanRequestFilterCBack(void);
static uint8_t simpleBLEPeripheral_MasterRssiScanCBack(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks, add 2017-11-15
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
    simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void SimpleBLEPeripheral_Init( uint8 task_id )
{
    simpleBLEPeripheral_TaskID = task_id;
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
    // Setup the GAP Peripheral Role Profile
    {
			// device starts advertising upon initialization
			uint8 initial_advertising_enable = FALSE;

			uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
			uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 

			// Set the GAP Characteristics
			GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

			// Set advertising interval
			uint16 advInt = 800;//1600;   // actual time = advInt * 625us

			// extended adv
			uint8_t ExtAdvHandle;
			Gap_ExtAdv_Param ExtAdv;
			ExtAdv.AdvHandle = 1;
			ExtAdv.eventProps = GAP_ADV_PROP_CONNECTABLE;
			ExtAdv.primIntMin = advInt;
			ExtAdv.primIntMax = advInt;
			ExtAdv.primChanMap = advChnMap;
			ExtAdv.ownAddrType = ADDRTYPE_PUBLIC;
			ExtAdv.peerAddrType = ADDRTYPE_PUBLIC;
			ExtAdv.txPower = 0;
			ExtAdv.primPhy = PKT_FMT_BLE1M;
			ExtAdv.secAdvMaxSkip = 0;
			ExtAdv.secPhy = PKT_FMT_BLR125K;
			ExtAdv.sid = 0;
			ExtAdv.scanReqNotifyEnable = TRUE;
			GapAdv_create(NULL,&ExtAdv,&ExtAdvHandle);
			GapAdv_loadByHandle(ExtAdvHandle, GAP_ADV_DATA_TYPE_ADV, sizeof( advertData ), advertData);
			GapAdv_loadByHandle(ExtAdvHandle, GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof( scanRspData ), scanRspData);
			GapAdv_enable(ExtAdvHandle,GAP_ADV_ENABLE_OPTIONS_USE_MAX,0);
			uint8 secondAdvEnable = TRUE;
			GAPRole_SetParameter(GAPROLE_EXT_ADVERT_ENABLED,1,&secondAdvEnable);
		}
		// Setup the GAP Bond Manager, add 2017-11-15
		{
			uint32 passkey = DEFAULT_PASSCODE;//0; // passkey "000000"
			uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
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
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile


    //intial notifyBuf
    for(int i =0 ;i<255;i++)
        notifyBuf[i]=i;

    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

		// peripheral parameter init
		Gap_ResetPhyParams(PKT_FMT_BLR125K);
		llInitFeatureSetDLE(TRUE);

    // Setup a delayed profile startup
    osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );


    // start a 60second timer for enter non conn state
    //VOID osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_ENTER_NOCONN_EVT, 60 * 1000);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
        {
            simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

				// Start Bond Manager, 2017-11-15
				VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

        // Set timer for first periodic event
//        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

        return ( events ^ SBP_START_DEVICE_EVT );
    }
    // enable adv
    if ( events & SBP_RESET_ADV_EVT )
    {
        uint8 initial_advertising_enable = TRUE;
		
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
        return ( events ^ SBP_RESET_ADV_EVT );
    }  
		
    if (events & SBP_SCAN_RSP_EVT )
    {
        //LOG("SBP_SCAN_RSP_EVT address:%02X,%02X,%02X,%02X,%02X,%02X; rssi:%d\n",g_scan_info.scan_mac[0],\
        g_scan_info.scan_mac[1],\
        g_scan_info.scan_mac[2],\
        g_scan_info.scan_mac[3],\
        g_scan_info.scan_mac[4],\
        g_scan_info.scan_mac[5],\
        g_scan_info.rssi);
        return ( events ^ SBP_SCAN_RSP_EVT );
    }

        // notifity
    if ( events & SBP_PERIODIC_EVT )
    {
			
				uint16 connIntv;
        GAPRole_GetParameter(GAPROLE_CONN_INTERVAL,&connIntv);
        connIntv = connIntv + (connIntv >>2); //*1.25
			
				uint8 len = ATT_GetCurrentMTUSize()-3;
				uint8 buf[len];
				notifyCnt++;
				buf[0] = buf[len-2] = HI_UINT16(notifyCnt);
				buf[1] = buf[len-1] = LO_UINT16(notifyCnt);
			
				simpleProfile_Notify(SIMPLEPROFILE_CHAR6,len,&buf);
				
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, connIntv );
				AT_LOG("T:%d\n",notifyCnt);
        return ( events ^ SBP_PERIODIC_EVT );
    } 
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {  
    default:
        // do nothing
        break;
    }
}
/*********************************************************************
 * @fn      peripheralStateReadRssiCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateReadRssiCB( int8  rssi )
{
//    notifyBuf[15]++;
//    notifyBuf[16]=rssi;
//    notifyBuf[17]=HI_UINT16(g_conn_param_foff);
//    notifyBuf[18]=LO_UINT16(g_conn_param_foff);;
//    notifyBuf[19]=g_conn_param_carrSens;
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
    switch ( newState )
    {
        case GAPROLE_STARTED:
        {
            uint8 ownAddress[B_ADDR_LEN];
            uint8 str_addr[14]={0}; 
            uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
            uint8 initial_advertising_enable = FALSE;//true
        
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


            osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
            osal_memcpy(&scanRspData[11],&str_addr[6],8);
            osal_memcpy(&attDeviceName[9],&str_addr[6],8);
        

            GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
            // Set the GAP Characteristics
            GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        }
            break;
        
        case GAPROLE_ADVERTISING:
        {
            osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
            notifyCnt=0x15A0;
            notifyInterval = 0;
            LL_PLUS_PerStatsReset();

            uint32_t rcCalCode = (*(volatile uint32_t *) 0x4000f018 & 0x7f)>>1;
            
            LOG("[RTC CNT]%02x %d %d %d\n",rcCalCode,counter_tracking,g_counter_traking_cnt,g_counter_traking_avg);
            if(sysClockChangeFlg)
            {
                LOG("[NVIC RST]\n");
                sysClockChangeFlg=0;
                NVIC_SystemReset();             
            }

            wtnrTest.cnt=0;
            wtnrTest.miss=0;
            wtnrTest.err=0;

            
         }   
            break;
        
        case GAPROLE_CONNECTED:
            HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, NULL);
            AT_LOG("Device Connected \n");
            break;
        
        case GAPROLE_CONNECTED_ADV:
            break;      
        case GAPROLE_WAITING:
            break;
        
        case GAPROLE_WAITING_AFTER_TIMEOUT:
					AT_LOG("Device Connection TimeOut \n");
					osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
					notifyCnt=0x15A0;
            break;
        
        case GAPROLE_ERROR:
            break;
        
        default:
            break;        
    }  
    gapProfileState = newState;
		
		
    AT_LOG("[GAP ROLE %d]\n",newState);
     
    VOID gapProfileState;     
}


/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue[IBEACON_ATT_LONG_PKT];
    
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR5:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, newValue );
      
      if(newValue[0]== 0x00 )
      {
        uint16 connIntv;
        GAPRole_GetParameter(GAPROLE_CONN_INTERVAL,&connIntv);
				AT_LOG("[WRT_ATT] %02x connIntv %d\n",newValue[0],connIntv);
				
        connIntv = connIntv + (connIntv >>2); //*1.25
				
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, connIntv );        
      }
			if( newValue[0]== 0x01 )
			{
				check_PerStatsProcess();
				AT_LOG("\n\n **-*-*-*-* Test End **-*-*-*-* \n");
				while(1);
			}
      

      break;
    default:
      // not process other attribute change
      break;
  }
}


/*********************************************************************
 * @fn      updateAdvData
 *
 * @brief   update adv data and change the adv type
 *
 * @param   none
 *
 * @return  none
 */
static void updateAdvData(void)
{
    uint8  new_uuid[IBEACON_UUID_LEN];
    uint16  major;
    uint16  minor;
    uint8   power;
    
    // 1. get the new setting from GATT attributes
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, new_uuid );
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, &major );
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &minor );
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, &power );	  
    
    // 2. update adv data storage
    //set UUID
    VOID osal_memcpy(&advertData[9], new_uuid, IBEACON_UUID_LEN);
    // set major
    advertData[25] = LO_UINT16( major );
    advertData[26] = HI_UINT16( major );	  
    // set minor	  
    advertData[27] = LO_UINT16( minor );
    advertData[28] = HI_UINT16( minor );	
    // set power
    advertData[29] = power;
	
    // 3. disconnect all connection
    GAPRole_TerminateConnection();
		
    // 4. close advert
    uint8 initial_advertising_enable = FALSE;		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
        
    // 5. update adv data
    // 5.1 update adv type
    uint8 advType = g_current_advType;    
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );	  

    uint16 advInt = otaAdvIntv<<4;
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );

    // 5.2 update advert broadcast
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );	

    // 5.3 set TxPower
    g_rfPhyTxPower = power;
    rf_phy_set_txPower(power);

    // 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
    osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT,5000);    
}

/*********************************************************************
* @fn      bdAddr2Str
*
* @brief   Convert Bluetooth address to string. Only needed when
*          LCD display is used.
*
* @return  none
*/
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
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


uint8_t simpleBLEPeripheral_ScanRequestFilterCBack(void)
{
	uint8_t scanReq[32];
    uint8_t scanAddr[6];
    uint8_t scanReqLen;
    scanReqLen=LL_PLUS_GetScanRequestExtendData(scanReq);
    if(scanReqLen>0 )
    {
        if(scanReq[0]%2==0)
        {
            LL_PLUS_SetScanRsqDataByIndex(sizeof ( scanRspData )-1, scanReq[0]);
            return 1;
        }
        else
        {
            return 0;
        }
        
    }
    else
    {
        LL_PLUS_SetScanRsqDataByIndex(sizeof ( scanRspData )-1, 0);
        LL_PLUS_GetScanerAddr(scanAddr);
        return 1;
    }
}

/*********************************************************************
* @fn      simpleBLEPeripheral_MasterRssiScanCBack 
*
* @brief   get master scan addr and rssi
*
* @return  none

*/
uint8_t simpleBLEPeripheral_MasterRssiScanCBack(void)
{
	LL_PLUS_GetScanerAddr(g_scan_info.scan_mac);
	g_scan_info.rssi = LL_PLUS_GetCurrentRSSI();
	osal_set_event(simpleBLEPeripheral_TaskID,SBP_SCAN_RSP_EVT);
	return 1;
}

/*********************************************************************
*********************************************************************/
