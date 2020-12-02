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
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
//#include "OnBoard.h"
#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice_bell.h"
#include "simpleGATTprofile.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#include "gpio.h"
#include "battery.h"
#include "led.h"
#include "log.h"
#include "flash.h"
#include "common.h"

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

// default advertising interval in 625us units
#define DEFAULT_ADV_INTERVAL                  1600


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
//extern volatile uint8_t g_current_advType;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static uint32_t adv_num=0;			//adv index

static uint16 g_battvalue=0;


static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x14,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x50,   // 'P'
    0x68,   // 'h'
    0x79,   // 'y'
    0x70,   // 'p'
    0x6c,   // 'l'
    0x75,   // 'u'
    0x73,   // 's'
	0x20,	//
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

uint8	uuid_value0[IBEACON_UUID_LEN],uuid_value1[IBEACON_UUID_LEN],uuid_value2[IBEACON_UUID_LEN];
uint16 	major0,major1,major2;
uint16 	minor0,minor1,minor2;
volatile bool	connect_flag=FALSE;
//const	uint8 default_power=0x05;
uint16	adv_interval=1000;		//default adv interval(1s,3 adv channel)
uint8 power = 0x05;				//defalut tx power(0dbm,index is 0x05)
uint32 parament_lib[56];


// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Phyplu BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void updateAdvData(void);
static void set_tx_power(uint8 tx_power);
static void set_adv_interval(uint16 interval);
static void change_parament_flash(void);
static void	set_device_para(uint8 param,uint8 *string);



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
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


        uint8 advType =GAP_ADTYPE_ADV_IND;// LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
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
        uint16 advInt = 4*DEFAULT_ADV_INTERVAL;   // actual time = advInt * 625us

        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
	
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile

	// Get parament information,uuid,major,minor
	//uuid address:uuid1(0x11005000-503c),uuid2(0x11005048-5084),uuid(0x11005090-50cc)
	//major address:major0(0x11005040),major1(0x11005088),major2(0x110050d0)
	//major address:minor0(0x11005044),minor1(0x1100508c),minor2(0x110050d4)
	//power address:0x110050d8;
	//adv interval:0x110050dc;
    {
		for (uint8 i = 0; i < PARAM_LEN; ++i)
			{
			parament_lib[i]=read_reg(0x11005000+(i<<2));
			}
		for (uint8 i = 0; i < IBEACON_UUID_LEN; ++i)
			{
			uuid_value0[i]=parament_lib[i]&0xff;
			}
		major0=parament_lib[16]&0xffff;
		minor0=parament_lib[17]&0xffff;

		for (uint8 i = 0; i < IBEACON_UUID_LEN; ++i)
			{
			uuid_value1[i]=parament_lib[i+18]&0xff;
			}
		major1=parament_lib[34]&0xffff;
		minor1=parament_lib[35]&0xffff;

		for (uint8 i = 0; i < IBEACON_UUID_LEN; ++i)
			{
			uuid_value2[i]=parament_lib[i+36]&0xff;
			}
		major2=parament_lib[52]&0xffff;
		minor2=parament_lib[53]&0xffff;
		power=parament_lib[54]&0xff;
		set_tx_power(power);
		adv_interval=parament_lib[55]&0xffff;
		set_adv_interval(adv_interval);
    	
    	
    }
    // Setup the SimpleProfile Characteristic Values
    {
        
        uint8 reset = 0x0;
	
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, IBEACON_UUID_LEN, uuid_value0);
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint16 ), &major0 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint16 ), &minor0 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, IBEACON_UUID_LEN, uuid_value1);
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, sizeof ( uint16 ), &major1 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, sizeof ( uint16 ), &minor1 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, IBEACON_UUID_LEN, uuid_value2);
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR8, sizeof ( uint16 ), &major2 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR9, sizeof ( uint16 ), &minor2 );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR10, sizeof ( uint8 ), &power );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR12, sizeof ( uint8 ), &reset );
		SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR11, sizeof ( uint16 ), &adv_interval );
    }

    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

	// Setup a delayed profile startup
    osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

	//green led on 2 sencond
	led_on(LEDG, 2000);	
    
    //peripheral initial
    //batt init
    batt_init();
	
    

	osal_start_timerEx(simpleBLEPeripheral_TaskID,BEACON_START_ADV,1*1500);
    // Setup a delayed profile startup
    //osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

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

//	LOG("ProcessEvent: 0x%x\n",events);

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

        // Start Bond Manager
        //  VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

        // Set timer for first periodic event
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

        return ( events ^ SBP_START_DEVICE_EVT );
    }

	if ( events & SBP_PERIODIC_EVT )
    {
    	osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
		if (g_battvalue<3000)
			{
			osal_set_event(simpleBLEPeripheral_TaskID, BATT_LOW_LIGHT_ON);
			}

        return ( events ^ SBP_PERIODIC_EVT );
    }

    // change to no conn adv
    if ( events & SBP_ENTER_NOCONN_EVT )
    {
        updateAdvData();

        return ( events ^ SBP_ENTER_NOCONN_EVT );
    }  

    // enable adv
    if ( events & SBP_RESET_ADV_EVT )
    {
        uint8 initial_advertising_enable = TRUE;
		
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
        return ( events ^ SBP_RESET_ADV_EVT );
    } 

	//start battery detect
	if( events & TIMER_BATT_EVT ){
    batt_measure();
//    WaitMs(100);
		float batt= batt_voltage();
		g_battvalue=(uint16)(1000*batt);
//		if (batt<2.5)       //when battery low,led on 100ms,off 4900ms
//			{
//				osal_set_event(simpleBLEPeripheral_TaskID, BATT_LOW_LIGHT_ON);
//			}
		
		LOG("%d\n",(int)(batt*1000));
		
		//osal_start_timerEx(simpleBLEPeripheral_TaskID, TIMER_BATT_EVT, 60*60*);

		return ( events ^ TIMER_BATT_EVT );
	
		}

	//battery low led on
	if( events & BATT_LOW_LIGHT_ON ){
		
		led_pull_light_interval_on(LEDR,100,BATT_LOW_LIGHT_OFF);   //100

		return ( events ^ BATT_LOW_LIGHT_ON );
			
		}

	//battery low led off
	if( events & BATT_LOW_LIGHT_OFF ){

		led_pull_light_off(LEDR);
		
		return ( events ^ BATT_LOW_LIGHT_OFF );
		}

	//connect led on
	if( events & CONNECT_LIGHT_ON ){
		
		led_pull_light_interval_on(LEDG,100,CONNECT_LIGHT_OFF);

		return ( events ^ CONNECT_LIGHT_ON );
			
		}

	//connect led on
	if( events & CONNECT_LIGHT_OFF ){

		led_pull_light_interval_off(LEDG, 1900,CONNECT_LIGHT_ON);
		
		return ( events ^ CONNECT_LIGHT_OFF );
		}

	if ( events & BEACON_START_ADV )
  {
        osal_start_timerEx(simpleBLEPeripheral_TaskID,BEACON_START_ADV,adv_interval);
        updateAdvData();
   
        // return unprocessed events
        return (events ^ BEACON_START_ADV);
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
        	//set device information ID 
            uint8 ownAddress[B_ADDR_LEN];
            uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
			uint8 dev_string[DEVINFO_MAX_LEN];
        
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
            
      	uint8 i=0;
		uint32 ret=0;
      	uint8 para_length;
		//device address:
		//Model number:0x11006000
		//Serial Number:0x11006100
		//Firmware Revision:0x11006200
		//Hardware Revision:0x11006300
		//Software Revision:0x11006400
		//Manufacturer Name:0x11006500
		//11073Cert:0x11006600
		//PnpId:0x11006700
		//Production Date:0x11006f00
		//set device information Model number
		set_device_para(DEVINFO_MODEL_NUMBER,&dev_string[0]);
		set_device_para(DEVINFO_FIRMWARE_REV, &dev_string[0]);
		set_device_para(DEVINFO_HARDWARE_REV, &dev_string[0]);
		set_device_para(DEVINFO_SOFTWARE_REV, &dev_string[0]);
		set_device_para(DEVINFO_MANUFACTURER_NAME, &dev_string[0]);

		//set production date
		para_length=read_reg(0x11006f00)&0xff;
		if(para_length!=0xff){
        	if(para_length>PRODUCTION_MAX_LEN){
          	para_length=PRODUCTION_MAX_LEN;
        	}
        	for(i=0;i<para_length;i++){
          	ret=read_reg(0x11006f04+(i<<2));
          	dev_string[i]=ret&0xff;
        	}
        	for(i=para_length;i<PRODUCTION_MAX_LEN;i++){
          	dev_string[i]=0;
        	}
        	SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR13, PRODUCTION_MAX_LEN, dev_string);
          }
    	}
            break;
        
        case GAPROLE_ADVERTISING:	
		if (connect_flag==TRUE)
			{
			connect_flag=FALSE;
			osal_set_event(simpleBLEPeripheral_TaskID, BEACON_START_ADV);
			led_pull_light_off(LEDG);
			}		
            break;
        
        case GAPROLE_CONNECTED:
			connect_flag=TRUE;
			osal_stop_timerEx(simpleBLEPeripheral_TaskID, BEACON_START_ADV);
			osal_set_event(simpleBLEPeripheral_TaskID, CONNECT_LIGHT_ON);
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
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue1;
  uint16 newValue2;
  uint8 newVaule3[IBEACON_UUID_LEN];
    
  switch( paramID )
  {
  	case SIMPLEPROFILE_CHAR1:    
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newVaule3 );
		for (uint8	i = 0; i < IBEACON_UUID_LEN; ++i)
			{
			parament_lib[i]=newVaule3[i];
			}
	  	VOID osal_memcpy(uuid_value0, newVaule3, IBEACON_UUID_LEN);
		adv_num=0;
	  	change_parament_flash();
 	break;

	case SIMPLEPROFILE_CHAR2:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, &newValue2 );
	  	parament_lib[16]=newValue2;
		major0=newValue2;

	  	change_parament_flash();
	
  	break;

	case SIMPLEPROFILE_CHAR3:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue2 );
	  	parament_lib[17]=newValue2;
		minor0=newValue2;

	  	change_parament_flash();
	
  	break;

	case SIMPLEPROFILE_CHAR4:    
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, &newVaule3 );
		for (uint8	i = 0; i < IBEACON_UUID_LEN; ++i)
			{
			parament_lib[i+18]=newVaule3[i];
			}
	  	VOID osal_memcpy(uuid_value1, newVaule3, IBEACON_UUID_LEN);
		adv_num=1;
	  	change_parament_flash();
 	break;

	case SIMPLEPROFILE_CHAR5:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, &newValue2 );
	  	parament_lib[34]=newValue2;
		major1=newValue2;

	  	change_parament_flash();
	
  	break;

	case SIMPLEPROFILE_CHAR6:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, &newValue2 );
	  	parament_lib[35]=newValue2;
		minor1=newValue2;

	  	change_parament_flash();
	
  	break;

	case SIMPLEPROFILE_CHAR7:    
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR7, &newVaule3 );
		for (uint8	i = 0; i < IBEACON_UUID_LEN; ++i)
			{
			parament_lib[i+36]=newVaule3[i];
			}
	  	VOID osal_memcpy(uuid_value2, newVaule3, IBEACON_UUID_LEN);
		adv_num=2;
	  	change_parament_flash();
 	break;

	case SIMPLEPROFILE_CHAR8:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR8, &newValue2 );
	  	parament_lib[52]=newValue2;
		major2=newValue2;

	  	change_parament_flash();
	
  	break;

	case SIMPLEPROFILE_CHAR9:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR9, &newValue2 );
	  	parament_lib[53]=newValue2;
		minor2=newValue2;

	  	change_parament_flash();
	
  	break;
	
  	case SIMPLEPROFILE_CHAR10:    
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR10, &newValue1 );
	  	parament_lib[54]=newValue1;
	  	change_parament_flash();
	  	set_tx_power(newValue1);
   	break;

	case SIMPLEPROFILE_CHAR11:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR11, &newValue2 );
	  	parament_lib[55]=newValue2;

	  	change_parament_flash();
	  
	  	set_adv_interval(newValue2);
	
  	break;

	case SIMPLEPROFILE_CHAR12:
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR12, &newValue1 );

      	if (newValue1 == 0x01)
      	{
          	// option:
          	// 1. reset
          	// 2. reset advertisement
            // 3. disconnect all connection
            newValue1=0;
			SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR12, sizeof ( uint8 ), &newValue1 );
            GAPRole_TerminateConnection();	  
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
	// update adv data storage
	if(adv_num==0){
		//set UUID
    	VOID osal_memcpy(&advertData[9], uuid_value0, IBEACON_UUID_LEN);
    	// set major
    	advertData[25] = LO_UINT16( major0 );
    	advertData[26] = HI_UINT16( major0 );	  
    	// set minor	  
    	advertData[27] = LO_UINT16( minor0 );
    	advertData[28] = HI_UINT16( minor0 );
		// set power
    	//advertData[29] = power;
    	adv_num++;
		}else if(adv_num==1){
			//set UUID
    		VOID osal_memcpy(&advertData[9], uuid_value1, IBEACON_UUID_LEN);
    		// set major
    		advertData[25] = LO_UINT16( major1 );
    		advertData[26] = HI_UINT16( major1 );	  
    		// set minor	  
    		advertData[27] = LO_UINT16( minor1 );
    		advertData[28] = HI_UINT16( minor1 );
    		// set power
    		//advertData[29] = power;
    		adv_num++;	
		}else if(adv_num==2){
			//set UUID
    		VOID osal_memcpy(&advertData[9], uuid_value2, IBEACON_UUID_LEN);
    		// set major
    		advertData[25] = LO_UINT16( major2 );
    		advertData[26] = HI_UINT16( major2 );	  
    		// set minor	  
    		advertData[27] = LO_UINT16( minor2 );
    		advertData[28] = HI_UINT16( minor2 );
    		// set power
    		//advertData[29] = power;
    		adv_num=0;	
				}

			
    	advertData[29] = power;
	
   
		
    // 4. close advert
    uint8 initial_advertising_enable = FALSE;		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
        
    // 5. update adv data
    // 5.1 update adv type
    uint8 advType = GAP_ADTYPE_ADV_IND;    
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );	  

    // 5.2 update advert broadcast
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );	

    // 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
    osal_set_event(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT);    
}

//set tx power
static void set_tx_power(uint8 tx_power){
	uint8 default_tx_power=0x05;
	switch (tx_power)
		  {
		  case	0:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_MIN;
			break;
		  case	1:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_N20DBM;
			break;
		  case	2:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_N15DBM;
			break;
		  case	3:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_N10DBM;
		  	break;
		  case	4:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_N5DBM;
		  	break;
		  case	5:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_0DBM;
		  	break;
		  case	6:
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_5DBM;
		  	break;
		  
		  default:
        	power=default_tx_power;
		  	g_rfPhyTxPower=RF_PHY_TX_POWER_0DBM;
			SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR10, sizeof ( uint8 ), &power );
		  	break;
		  }
	}

//change adv interval
static void set_adv_interval(uint16 interval){
	if ( interval<50	||	interval>4000	)
		  {
		  adv_interval=DEFAULT_ADV_INTERVAL;
		  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR11, sizeof ( uint16 ), &adv_interval );
		  }else{
        adv_interval=interval;
      }
	}

//when change the adv parament,you should write corresponding flash
static void change_parament_flash(){
	flash_sector_erase(0x11005000);			//erase flash of parament area
	for (uint16 i = 0; i < 56; ++i)
		{
		WriteFlash((0x11005000+(i<<2)), parament_lib[i]);     //write parament to flash
		}
	}

//set device parament
static void set_device_para(uint8 param,uint8 *string){
	uint8 i=0;
	uint32 ret=0;
	uint8 para_length;

	para_length=read_reg(0x11006000+(param-1)*0x100)&0xff;
    if(para_length!=0xff){
        if(para_length>DEVINFO_MAX_LEN){
          para_length=DEVINFO_MAX_LEN;
        }
     	for(i=0;i<para_length;i++){
          	ret=read_reg(0x11006004+(param-1)*0x100+(i<<2));
          	string[i]=ret&0xff;
        	}
        for(i=para_length;i<DEVINFO_MAX_LEN;i++){
          	string[i]=0;
        	}
        DevInfo_SetParameter(param, DEVINFO_MAX_LEN, string);
          }
	}





/*********************************************************************
*********************************************************************/
