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
  Filename:       bleuart.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/


#include "types.h"
#include "bcomdef.h"
#include "simpleGATTprofile_ota.h"
#include "bleuart_service.h"
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "bleuart_service.h"

#include "peripheral.h"
#include "gapbondmgr.h"


#include "bleuart.h"
#include "bleuart_service.h"
#include "bleuart_protocol.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define BUP_PERIODIC_EVT_PERIOD                   5000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0
 

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

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


// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15



uint8 bleuart_TaskID;   // Task ID for internal task/event processing

uint16 gapConnHandle;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    13,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'P','h','y',' ','B','L','E',' ','U','a','r','t',

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



// advert data for bleuart
static uint8 advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x11,
    0x07,//Complete list of 128-bit UUIDs available
    0x55, 0xe4,0x05,0xd2,0xaf,0x9f,0xa9,0x8f,0xe5,0x4a,0x7d,0xfe,0x43,0x53,0x53,0x49,
};


// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Phy BLE-Uart";



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleuart_StateNotificationCB( gaprole_States_t newState );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t bleuart_PeripheralCBs =
{
    bleuart_StateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void on_bleuartServiceEvt(bleuart_Evt_t* pev)
{
  LOG("on_bleuartServiceEvt:%d\n", pev->ev);
  switch(pev->ev){
  case bleuart_EVT_TX_NOTI_DISABLED:
    BUP_disconnect_handler();
    break;
  case bleuart_EVT_TX_NOTI_ENABLED :
    BUP_connect_handler();
    osal_set_event(bleuart_TaskID,BUP_OSAL_EVT_NOTIFY_DATA);
    break;
  case bleuart_EVT_BLE_DATA_RECIEVED:
    BUP_data_BLE_to_uart( (uint8_t*)pev->data, (uint8_t)pev->param);
    break;
  default:
    break;
  }
}

void on_BUP_Evt(BUP_Evt_t* pev)
{
  switch(pev->ev){
    

  }
}


void bleuart_Init( uint8 task_id )
{
  bleuart_TaskID = task_id;

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
    //    uint8 advType = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;
    //    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
    
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
  bleuart_AddService(on_bleuartServiceEvt);
  BUP_init(on_BUP_Evt);
	
  // Setup a delayed profile startup
  osal_set_event( bleuart_TaskID, BUP_OSAL_EVT_START_DEVICE );
  //osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_BLE_TIMER, 1 );

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
uint16 bleuart_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & BUP_OSAL_EVT_START_DEVICE )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &bleuart_PeripheralCBs );

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

  if( events & BUP_OSAL_EVT_BLE_TIMER)
  {
    LOG("BUP_OSAL_EVT_BLE_TIMER\n");
    BUP_data_BLE_to_uart_send();
    return ( events ^ BUP_OSAL_EVT_BLE_TIMER );
  }
  if( events & BUP_OSAL_EVT_UARTRX_TIMER)
  {
    LOG("BUP_OSAL_EVT_UARTRX_TIMER\n");
    BUP_data_uart_to_BLE_send();
    return ( events ^ BUP_OSAL_EVT_UARTRX_TIMER );
  }

  if( events & BUP_OSAL_EVT_UART_TX_COMPLETE)
  {
    LOG("BUP_OSAL_EVT_UART_TX_COMPLETE\n");
    BUP_data_BLE_to_uart_completed();
    return ( events ^ BUP_OSAL_EVT_UART_TX_COMPLETE);
  }
  if( events & BUP_OSAL_EVT_UART_TO_TIMER)
  {
    LOG("BUP_OSAL_EVT_UART_TO_TIMER\n");
    BUP_data_uart_to_BLE();
    return ( events ^ BUP_OSAL_EVT_UART_TO_TIMER);
  }
  // Discard unknown events
  return 0;
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
static void bleuart_StateNotificationCB( gaprole_States_t newState )
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
          FLOW_CTRL_BLE_DISCONN();
          BUP_disconnect_handler();
          FLOW_CTRL_UART_TX_UNLOCK();
          FLOW_CTRL_BLE_TX_UNLOCK();
          LOG("advertising!\n");
          break;
        
        case GAPROLE_CONNECTED:
          GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
          FLOW_CTRL_BLE_CONN();
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


uint16_t bleuart_conn_interval(void)
{
    uint16_t interval, latency;
    GAPRole_GetParameter(GAPROLE_CONNECTION_INTERVAL, &interval);
    GAPRole_GetParameter(GAPROLE_CONNECTION_LATENCY, &latency);
    return ((1+latency)*interval*5/4);
}


/*********************************************************************
*********************************************************************/
