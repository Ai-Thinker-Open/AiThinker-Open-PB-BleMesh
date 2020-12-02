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
  Filename:       bleSmartPeripheral.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
//#include "OnBoard.h"
#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "bleSmartPeripheral.h"
#include "battery.h"
#include "log.h"
#include "touch_key.h"
#include "switch.h"
#include "pwm_ctrl.h"



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   1000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0
 

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

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
uint8 bleSmartPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x10,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x50,   // 'P'
    0x48,   // 'H'
    0x59,   // 'Y'
    0x2B,   // '+'
    0x20,	//
    0x50,   // 'P'
    0x65,   // 'e'
    0x72,   // 'r'
    0x69,   // 'i'
    0x70,   // 'p'
    0x68,   // 'h'
    0x65,   // 'e'
    0x72,   // 'r'
    0x61,   // 'a'
    0x6c,   // 'l'
  
    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
};

//define led config
switch_cfg_t sw[]={{P21,HIGH},{P22,HIGH}};


// advert data 
static uint8 advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    
    // User coudl add customer data below
    // length
    // AD_type
    // value
    
    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm    
};

// GAP GATT Attributes
static uint8 attDeviceName[] = "bleSmartPeripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleSmartPeripheral_InitGattAttr(void);
static void bleSmartPeripheral_UpdateGattAttr(void);
static void bleSmartPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
//static void simpleProfileChangeCB( uint8 paramID );
static void updateAdvData(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bleSmartPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t bleSmartPeripheral_BondMgrCBs =
{
    NULL,                     // Passcode callback (not used by application)
    NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
//static simpleProfileCBs_t bleSmartPeripheral_SimpleProfileCBs =
//{
//    simpleProfileChangeCB    // Charactersitic value change callback
//};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      bleSmartPeripheral_Init
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
void bleSmartPeripheral_Init( uint8 task_id )
{
    bleSmartPeripheral_TaskID = task_id;

    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
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

        uint8 advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
        GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
        
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
    {
        GGS_AddService( GATT_ALL_SERVICES );            // GAP
        GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
        DevInfo_AddService();                           // Device Information Service
        SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile

        // Setup the SimpleProfile Characteristic Values
        VOID bleSmartPeripheral_InitGattAttr();

        // Register callback with SimpleGATTprofile
//        VOID SimpleProfile_RegisterAppCBs( &bleSmartPeripheral_SimpleProfileCBs );
    }

    //peripheral initial,you can select the module you use
	batt_init();   //initial battery
  
  	switch_init(sw); //initial sw
    
  	pwm_init();    //initial pwm control
    
    
  	// Setup a delayed profile startup
  	osal_set_event( bleSmartPeripheral_TaskID, SBP_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      bleSmartPeripheral_ProcessEvent
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
uint16 bleSmartPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( bleSmartPeripheral_TaskID )) != NULL )
        {
            bleSmartPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &bleSmartPeripheral_PeripheralCBs );

        // Start Bond Manager
        VOID GAPBondMgr_Register( &bleSmartPeripheral_BondMgrCBs );

        // Set timer for first periodic event
        osal_start_timerEx( bleSmartPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

        return ( events ^ SBP_START_DEVICE_EVT );
    }

    if ( events & SBP_PERIODIC_EVT )
    {
//        batt_measure();
        // Read peripheral & update GATT characteristics
        bleSmartPeripheral_UpdateGattAttr();

        // Set timer for first periodic event
        osal_start_timerEx( bleSmartPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
        
        return ( events ^ SBP_PERIODIC_EVT );
    }
    
     if( events & GPIO_TIMER_KEY_EVT)
     {
          gpio_key_timer_handler();
               
          return (events ^ GPIO_TIMER_KEY_EVT);
    }
    
//    // change to no conn adv
//    if ( events & SBP_ENTER_NOCONN_EVT )
//    {
//        updateAdvData();

//        return ( events ^ SBP_ENTER_NOCONN_EVT );
//    }  

    // enable adv
    if ( events & SBP_RESET_ADV_EVT )
    {
        uint8 initial_advertising_enable = TRUE;
		
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
        return ( events ^ SBP_RESET_ADV_EVT );
    }  

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      bleSmartPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void bleSmartPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {  
    default:
        // do nothing
        break;
    }
}

/*********************************************************************
 * @fn      bleSmartPeripheral_InitGattAttr
 *
 * @brief   Initial Gatt characteristics
 *
 * @param   None
 *
 * @return  none
 */
static void bleSmartPeripheral_InitGattAttr(void)
{
    uint16 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3[] = {5, 6, 11, 22, 0, 254};
    
    uint8 charValue4 = 4;
    uint8 charValue5 = 5;
        
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint16 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( charValue3 ), charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, sizeof ( uint8 ), &charValue5 );
    
}

/*********************************************************************
 * @fn      bleSmartPeripheral_UpdateGattAttr
 *
 * @brief   Read peripheral and update characteristics of simple GATT profile
 *
 * @param   None
 *
 * @return  none
 */
static void bleSmartPeripheral_UpdateGattAttr(void)
{
    static uint16 charValue1=0x00;
//    uint8 charValue2 = 2;
    uint8 charValue3[] = {15, 16, 111, 122, 10, 254};
    
    batt_measure();

//    charValue1 = (uint16)(batt_voltage()*1000);
//    charValue1 = ((charValue1&0xff)<<8)|((charValue1>>8)&0xff);

    
    
    // TODO: read the data from peripheral
    charValue1 ++;
    charValue3[0]=charValue1;
    
    simpleProfile_Notify(SIMPLEPROFILE_CHAR3, sizeof ( charValue3 ), charValue3);
    
    // update the GATT characteristics' value
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint16 ), &charValue1 );
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( charValue3 ), charValue3 );
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
            break;
        
        case GAPROLE_CONNECTED:
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

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
//static void simpleProfileChangeCB( uint8 paramID )
//{
//  uint8 newValue;
//    
//  switch( paramID )
//  {
//    case SIMPLEPROFILE_CHAR2:
//      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, &newValue );
//      if(newValue==0x01){
//        switch_on(led[0]);
//      }else if(newValue==0x00){
//        switch_off(led[0]);
//      }

//      break;
//    
//    case SIMPLEPROFILE_CHAR4:
//      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, &newValue );
//      if(newValue==0x01){
////        LOG("oo");
//        pwm_on(0);
//      }else if(newValue==0x00){
//        light_off(0);
//      }

//      break;
//      
//    case SIMPLEPROFILE_CHAR5:
//      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, &newValue );
//      pwm_ctrl(0,newValue);

//      break;
//      
//    default:
//      // not process other attribute change
//      break;
//  }
//}

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
//    uint8  new_uuid[16];
//    uint16  major;
//    uint16  minor;
//    uint8   power;
    
    // 1. get the new setting from GATT attributes
    
    // 2. update adv data storage
    //set UUID
//    VOID osal_memcpy(&advertData[9], new_uuid, 16);
	
    // 3. IMPORTANT: should disconnect all connection before enable ADV again
    GAPRole_TerminateConnection();
		
    // 4. close advert
    uint8 initial_advertising_enable = FALSE;		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
        
    // 5. update adv data
    // 5.1 update adv type
    uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;    
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );	  

    // 5.2 update advert broadcast
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );	

    // 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
    osal_set_event(bleSmartPeripheral_TaskID, SBP_RESET_ADV_EVT);    
}


/*********************************************************************
*********************************************************************/
