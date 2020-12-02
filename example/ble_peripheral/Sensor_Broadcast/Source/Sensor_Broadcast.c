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
  Filename:       sensor_broadcast.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
//#include "OnBoard.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "Sensor_Broadcast.h"
#include "log.h"
#include "gpio.h"
#include "flash.h"
#include "ll_sleep.h"
#include "pwrmgr.h"
#include "global_config.h"
#include "touch_key.h"
#include "battery.h"
#include "CT1710_sensor.h"


extern uint32_t osal_sys_tick;

extern uint32 *pGlobal_config;

extern key_contex_t key_state;

/*********************************************************************
 * MACROS
 */

void IOWakeupInit(GPIO_Pin_e pin,IO_Wakeup_Pol_e type){
    hal_gpio_pull_set(pin,PULL_DOWN);      //pull down the wake up pin
    hal_gpio_wakeup_set(pin,type);       //set wakeup source and plority
}

void enableSystemOff(void){
    hal_gpio_write(P15,0);
    write_reg(0x4000f000,0x5a5aa5a5);  //system off
}

void enableSystemSleep(void){
    hal_gpio_write(P15,0);
    write_reg(0x4000f004,0xa5a55a5a);  //system sleep
}

uint8_t io_wakeup_flag = 0;
static uint32_t adv_num=0;

void updateAdvData(void);
void AdvDataStart(void);

/*********************************************************************
 * PROFILE CALLBACKS
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
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// default advertising interval in 625us units
#define DEFAULT_ADV_INTERVAL                  16000



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
uint8 Sensor_Broadcast_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x0a,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'P',
  'h',
  'y',
  '+',
  ' ',
  'B',
  'r',
  'i',
  'd',
};

uint8 advertData[] = 
{ 
  // flags
  0x02,
  0x01,
  0x06,
  0x03,
  0x02,
  0xf0,
  0xff,
  0x0f,
  0xff,
  
  //device ID
  0x01,
  0x00,
  0x00,
  0x00,
  
  //MAC address
  0x50,
  0x65,
  0x83,
  0x05,
  0x78,
  0xf2,
  
  //batt value
  0x00,
  0x00,
  
  //temp vaule
  0xff,
  0x7f,
  
};


// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PhyRawAdv";

static void RawAdvNotificationCB( gaprole_States_t newState );

// GAP connection handle
// GAP Role Callbacks
static gapRolesCBs_t RawAdvPeripheralCBs =
{
    RawAdvNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};

static void key_press_evt(key_evt_t key_evt)
{
  if(key_evt == TOUCH_EVT_SHORT_PRESS){
    osal_set_event(Sensor_Broadcast_TaskID, KEY_SHORT_PRESS_EVT);
  }else if(key_evt == TOUCH_EVT_LONG_PRESS){
    osal_set_event(Sensor_Broadcast_TaskID, KEY_LONG_PRESS_EVT);
  }else if(key_evt == TOUCH_EVT_PRESS){
    osal_set_event(Sensor_Broadcast_TaskID, KEY_PRESS_EVT);
  }else if(key_evt == TOUCH_EVT_RELEASE){
    osal_set_event(Sensor_Broadcast_TaskID, KEY_RELEASE_EVT);
  }
  
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Sensor_Broadcast_Init
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
void Sensor_Broadcast_Init( uint8 task_id )
{
  Sensor_Broadcast_TaskID = task_id;

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
		

    pGlobal_config[ADV_CHANNEL_INTERVAL] = 600;//6250; 

    GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
    // set adv channel map
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
//    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  //get mac address
//  {
//    uint8 ownAddress[B_ADDR_LEN];
//    GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
//    LOG("addr:%d",ownAddress[0]);
//    for(uint8_t i=0;i<B_ADDR_LEN;i++){
//      advertData[13+i]=ownAddress[i]&0xff;
//    }
// }
//  advertData[13] = ownAddress[0]&0xff;
//  advertData[14] = ownAddress[1]&0xff;
//  advertData[15] = ownAddress[1]&0xff;
//  advertData[16] = ownAddress[1]&0xff;
//  advertData[17] = ownAddress[1]&0xff;
//  advertData[18] = ownAddress[1]&0xff;
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  
  // Set advertising interval
  {
      uint16 advInt = DEFAULT_ADV_INTERVAL;   // actual time = advInt * 625us
  
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  DevInfo_AddService();
  
  //indicate system on
  hal_gpio_pull_set(P23,STRONG_PULL_UP);
  WaitMs(1000);
  hal_gpio_pull_set(P23,PULL_DOWN);
  
  // Setup a delayed profile startup
  osal_set_event( Sensor_Broadcast_TaskID, START_DEVICE_EVT );
  
  //get batt and sensor temp
  touch_init(key_press_evt);
  batt_init();
  CT1710_sensor_init();
//  uint32_t batt_degree = batt_voltage();
  uint32_t temp_degree = CT1710_read_temp_degree();
//  WaitMs(10);
//  LOG("temp:%d",temp_degree);
  uint32_t batt_degree = batt_voltage();
//  while(1);
  
  advertData[19] = batt_degree&0xff;
  advertData[20] = (batt_degree>>8)&0xff;
  advertData[21] = temp_degree&0xff;
  advertData[22] = (temp_degree>>8)&0xff;
  
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
  
//  LOG("degree:%d",temp_degree);

//{
//    uint8 ownAddress[B_ADDR_LEN];
//    GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
//    LOG("addr:%d",ownAddress[0]);
//    for(uint8_t i=0;i<B_ADDR_LEN;i++){
//      advertData[13+i]=ownAddress[i]&0xff;
//    }
// }

    osal_start_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV,1*1500);
//    hal_gpio_pull_set(P23,STRONG_PULL_UP);
//    WaitMs(1000);
//    hal_gpio_pull_set(P23,PULL_DOWN);
  
}

/*********************************************************************
 * @fn      Sensor_Broadcast_ProcessEvent
 *
 * @brief   RawAdv Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 Sensor_Broadcast_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
//  LOG("ProcessEvent: 0x%x\n",events);
    
  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
        VOID GAPRole_StartDevice( &RawAdvPeripheralCBs );
 
        return ( events ^ START_DEVICE_EVT );
  }

  
  if ( events & SENSOR_START_ADV )
  {
        adv_num++;
        osal_start_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV,1*1000);
        advertData[12] = 0x00;
        updateAdvData();
//        osal_start_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV,1*1000);
   
        // return unprocessed events
        return (events ^ SENSOR_START_ADV);
  }
  
     if( events & TIMER_KEY_EVT)
     {
          gpio_key_timer_handler();
               
          return (events ^ TIMER_KEY_EVT);
    }
   
    if( events & KEY_SHORT_PRESS_EVT)
     {
          hal_gpio_pull_set(P23,PULL_DOWN);
          adv_num++;
          osal_start_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV,1*1000);
          advertData[12] = 0x80;
          updateAdvData();    
          return (events ^ KEY_SHORT_PRESS_EVT);
    }
     
    if( events & KEY_LONG_PRESS_EVT)
     {
          hal_gpio_pull_set(P23,PULL_DOWN);
          pwroff_cfg_t pwr_wkp_cfg[]={{P14,POSEDGE}};
          hal_pwrmgr_poweroff(pwr_wkp_cfg,sizeof(pwr_wkp_cfg)/sizeof(pwr_wkp_cfg[0]));
          
          return (events ^ KEY_LONG_PRESS_EVT);
    }
     
    if( events & KEY_PRESS_EVT)
     {
          hal_gpio_pull_set(P23,STRONG_PULL_UP);
          return (events ^ KEY_PRESS_EVT);
    }
     
      if( events & KEY_RELEASE_EVT)
     {
          hal_gpio_pull_set(P23,PULL_DOWN);
          adv_num++;
          osal_start_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV,1*1000);
          updateAdvData();
          return (events ^ KEY_RELEASE_EVT);
    }
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
 * @fn      RawAdvNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void RawAdvNotificationCB( gaprole_States_t newState )
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
            
            for(uint8_t i=0;i<B_ADDR_LEN;i++){
              advertData[13+i]=ownAddress[i]&0xff;
            }
          
//          LOG("ADDR:%d",ownAddress[3]);
        
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        }
            break;
        
        case GAPROLE_ADVERTISING:
            break;
        
        case GAPROLE_CONNECTED:
            osal_stop_timerEx(Sensor_Broadcast_TaskID,SENSOR_START_ADV);
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
 * @fn      updateAdvData
 *
 * @brief   update adv data and change the adv type
 *
 * @param   none
 *
 * @return  none
 */
void updateAdvData(void)
{
//    hal_gpio_pin_init(P31,OEN);
//    hal_gpio_write(P31,1);
//    // 4. close advert
    uint8 initial_advertising_enable = FALSE;		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
        
    // 5. update adv data
    // 5.1 update adv type
    uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;    
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );	  

    if(adv_num%3){
    }else{
      batt_measure();
      CT1710_sensor_init();
//      WaitMs(10);
      uint32_t temp_degree = CT1710_read_temp_degree();
      uint32_t batt_degree = batt_voltage();
      advertData[19] = batt_degree&0xff;
      advertData[20] = (batt_degree>>8)&0xff;
      advertData[21] = temp_degree&0xff;
      advertData[22] = (temp_degree>>8)&0xff;
    }
    
  
    // 5.2 update advert broadcast
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

//    initial_advertising_enable = TRUE;
//    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
//    hal_gpio_write(P31,0);



//    // 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
    osal_set_event(Sensor_Broadcast_TaskID, SBP_RESET_ADV_EVT);    
}



/*********************************************************************
*********************************************************************/
