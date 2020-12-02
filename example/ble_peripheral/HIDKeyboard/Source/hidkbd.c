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

/******************************************************************************


 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "peripheral.h"
#include "hidkbdservice.h"
#include "devinfoservice.h"
#include "hiddev.h"
#include "hidkbd.h"
#include "touch_key.h"
#include "ble_ancs.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */

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
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              0

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     30

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
//#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 hidKbdTaskId;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x0D,                             // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'H',
  'I',
  'D',
  ' ',
  'K',
  'e',
  'y',
  'b',
  'o',
  'a',
  'r',
  'd'
};

// Advertising data
static uint8 advData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
  HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

  // service UUIDs
  0x05,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERV_UUID),
  HI_UINT16(HID_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID)
};

// Device name attribute value
static CONST uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";

// HID Dev configuration
static hidDevCfg_t hidKbdCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

// TRUE if boot mouse enabled
uint8 hidBootMouseEnabled = FALSE;





static ancs_notify_evt_t m_notification_current;
static ancs_notify_evt_t m_notification_latest;
static ancs_attr_evt_t  m_notif_attr_latest;                      /**< Local copy of the newest notification attribute. */
static ancs_attr_evt_t  m_notif_attr_app_id_latest;               /**< Local copy of the newest app attribute. */

static uint8_t m_attr_appid[ANCS_ATTR_DATA_MAX];                            /**< Buffer to store attribute data. */
static uint8_t m_attr_title[ANCS_ATTR_DATA_MAX];							   /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ANCS_ATTR_DATA_MAX]; 						   /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ANCS_ATTR_DATA_MAX];							   /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ANCS_ATTR_DATA_MAX]; 					   /**< Buffer to store attribute data. */
static uint8_t m_attr_date[ANCS_ATTR_DATA_MAX]; 							   /**< Buffer to store attribute data. */
static uint8_t m_attr_posaction[ANCS_ATTR_DATA_MAX];						   /**< Buffer to store attribute data. */
static uint8_t m_attr_negaction[ANCS_ATTR_DATA_MAX];						   /**< Buffer to store attribute data. */
//static uint8_t m_attr_disp_name[ANCS_ATTR_DATA_MAX];                        /**< Buffer to store attribute data. */

static const char * lit_catid[ANCS_NB_OF_CATEGORY_ID] =
{
	"Other",
	"Incoming Call",
	"Missed Call",
	"Voice Mail",
	"Social",
	"Schedule",
	"Email",
	"News",
	"Health And Fitness",
	"Business And Finance",
	"Location",
	"Entertainment"
};

static const char * lit_eventid[ANCS_NB_OF_EVT_ID] =
{
	"Added",
	"Modified",
	"Removed"
};

static const char * lit_attrid[ANCS_NB_OF_NOTIF_ATTR] =
{
	"App Identifier",
	"Title",
	"Subtitle",
	"Message",
	"Message Size",
	"Date",
	"Positive Action Label",
	"Negative Action Label"
};

static const char * lit_appid[ANCS_NB_OF_APP_ATTR] =
{
    "Display Name"
};



/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void hid_stateChangeCB( gaprole_States_t newState );
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs);
static void AncsApp_processPasscode(uint8_t uiOutputs);
static void hidKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void hidKbd_ProcessGattMsg( gattMsgEvent_t *pMsg );
static void hidKbdSendReport( uint8 keycode );
void hidKbdSendMouseReport( uint8 buttons );
static uint8 hidKbdRcvReport( uint8 len, uint8 *pData );
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                             uint8 oper, uint8 *pLen, uint8 *pData );
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void hidKbdEvtCB( uint8 evt );

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidKbdHidCBs =
{
  hidKbdRptCB,
  hidKbdEvtCB,
  NULL
};


static gapRolesCBs_t hid_gapRoleCBs =
{
  hid_stateChangeCB     // GAPRole State Change Callbacks
};

static gapBondCBs_t ancsApp_BondMgrCBs =
{
  (pfnPasscodeCB_t) AncsApp_passcodeCB, // Passcode callback
  AncsApp_pairStateCB                   // Pairing / Bonding state Callback
};

static void on_key(touch_evt_t key_evt)
{
  osal_set_event(hidKbdTaskId, HID_TEST_EVT);
}

static uint8 s_cnt = 0;
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs)
{
  hidDevPasscodeCB(deviceAddr, connHandle,uiInputs, uiOutputs);
  AncsApp_processPasscode(uiOutputs);
}


static void AncsApp_processPasscode(uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = 1234;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    LOG("Passcode: %d\n", passcode);
  }

  uint16_t connectionHandle;
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}


static void AncsApp_processPairState(uint8_t state, uint8_t status)
{
  uint16 conn_handle;
  LOG("AncsApp_processPairState %d,%d\n",state, status);
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    LOG("Pairing started\n");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
      LOG("Pairing Successful\n");

      // Now that the device has successfully paired to the iPhone,
      // the subscription will not fail due to insufficient authentication.
     ble_ancs_start_descovery(conn_handle);
    }
    else
    {
      LOG("Pairing fail: %d\n", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
      LOG("Bonding Successful\n");
      ble_ancs_start_descovery(conn_handle);
    }
  }

}

static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
  hidDevPairStateCB(connHandle, state, status);
  AncsApp_processPairState(state, status);
}
                                            
static void notif_print(ancs_notify_evt_t * p_notif)
{
    LOG("\r\nNotification\r\n");
    LOG("Event:       %s\r\n", lit_eventid[p_notif->eventID]);
    LOG("Category ID: %s\r\n", lit_catid[p_notif->categoryID]);
    LOG("Category Cnt:%u\r\n", (unsigned int) p_notif->categoryCount);
    LOG("UID:         %u\r\n", (unsigned int) p_notif->notifUID);

    LOG("Flags:\r\n");
    if(p_notif->eventFlag & EVENT_FLAG_silent)
    {
        LOG(" Silent\r\n");
    }
    if(p_notif->eventFlag & EVENT_FLAG_important)
    {
        LOG(" Important\r\n");
    }
    if(p_notif->eventFlag & EVENT_FLAG_pre_existing)
    {
        LOG(" Pre-existing\r\n");
    }
    if(p_notif->eventFlag & EVENT_FLAG_positive_action)
    {
        LOG(" Positive Action\r\n");
    }
    if(p_notif->eventFlag & EVENT_FLAG_negative_action)
    {
        LOG(" Negative Action\r\n");
    }
}

static void notif_attr_print(ancs_attr_evt_t * p_attr)
{
    if (p_attr->attr_len != 0)
    {
        LOG("%s: %s\r\n", lit_attrid[p_attr->attr_id], (char *)p_attr->p_attr_data);
    }
    else if (p_attr->attr_len == 0)
    {
        LOG("%s: (N/A)\r\n", lit_attrid[p_attr->attr_id]);
    }
}
static void app_attr_print(ancs_attr_evt_t * p_attr)
{
    if (p_attr->attr_len != 0)
    {
        LOG("%s: %s\r\n", (uint32_t)lit_appid[p_attr->attr_id], (uint32_t)p_attr->p_attr_data);
    }
    else if (p_attr->attr_len == 0)
    {
        LOG("%s: (N/A)\r\n", (uint32_t) lit_appid[p_attr->attr_id]);
    }
}

static void ancs_process_notif(ancs_notify_evt_t* p_notif)
{
	LOG("ancs_process_notif\n");
	if(p_notif->eventFlag & EVENT_FLAG_silent || p_notif->eventFlag & EVENT_FLAG_pre_existing) 
		return;
	if(p_notif->eventID == BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED) 
		return;
  m_notification_current = *p_notif;
  ble_ancs_get_notif_attrs((const uint8_t *)(m_notification_current.notifUID));
}

static void on_ancs_evt(ancs_evt_t * p_evt)
{

	switch (p_evt->type)
	{
		case BLE_ANCS_EVT_DISCOVERY_COMPLETE:
			LOG("Apple Notification Service discovered on the server.\r\n");
			break;

		case BLE_ANCS_EVT_NOTIF:
		{
		  ancs_notify_evt_t* p_notif = (ancs_notify_evt_t*)(p_evt->msg);
		  m_notification_latest = *p_notif;
			notif_print(&m_notification_latest);
			ancs_process_notif(p_evt->msg);
			break;
    }
		case BLE_ANCS_EVT_NOTIF_ATTRIBUTE:
    {
      ancs_attr_evt_t* p_notf_attr = (ancs_attr_evt_t*)(p_evt->msg);
      m_notif_attr_latest = *p_notf_attr;
      notif_attr_print(&m_notif_attr_latest);
      if(p_notf_attr->attr_id == BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER)
      {
          m_notif_attr_app_id_latest = *p_notf_attr;
          ble_ancs_get_app_attrs(m_notif_attr_app_id_latest.app_id, osal_strlen((char*)(m_notif_attr_app_id_latest.app_id)));
      }
      break;
    }
		case BLE_ANCS_EVT_DISCOVERY_FAILED:
			LOG("Apple Notification Service not discovered on the server.\r\n");
			break;
    case BLE_ANCS_EVT_APP_ATTRIBUTE:
    {
      ancs_attr_evt_t* p_notf_attr = (ancs_attr_evt_t*)(p_evt->msg);
      app_attr_print(p_notf_attr);
      break;
    }
    case BLE_ANCS_EVT_NP_ERROR:
      //err_code_print(p_evt->err_code_np);
      break;

    default:
      // No implementation needed.
      break;
  }
}

static void hid_stateChangeCB( gaprole_States_t newState )
{
  hidDevGapStateCB(newState);
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmuKbd_Init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
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
void HidKbd_Init( uint8 task_id )
{
  hidKbdTaskId = task_id;
  LOG("%s\n",__FUNCTION__);
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  if(hal_gpio_read(P14)){
    LOG("Erase all bounding\n");
    GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
  }
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *) attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }


  {
    // Use the same interval for general and limited advertising.
    // Note that only general advertising will occur based on the above configuration
    uint16_t advInt = 160;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Set up HID keyboard service
  HidKbd_AddService( );

  // Register for HID Dev callback
  HidDev_Register( &hidKbdCfg, &hidKbdHidCBs );

  touch_init(on_key);
 
  ble_ancs_init(on_ancs_evt, hidKbdTaskId);
 
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,m_attr_appid,ANCS_ATTR_DATA_MAX);
  
  //ble_ancs_attr_add(BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME,m_attr_disp_name,sizeof(m_attr_disp_name));
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_TITLE,m_attr_title,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE,m_attr_message,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,m_attr_subtitle,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,m_attr_message_size,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_DATE,m_attr_date,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,m_attr_posaction,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,m_attr_negaction,ANCS_ATTR_DATA_MAX);


  
  // Setup a delayed profile startup
  osal_set_event( hidKbdTaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      HidEmuKbd_ProcessEvent
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 HidKbd_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function
  LOG("%s\n",__FUNCTION__);

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( hidKbdTaskId )) != NULL )
    {
      hidKbd_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
      GAPRole_StartDevice( &hid_gapRoleCBs );
        GATT_InitClient();
        GATT_RegisterForInd(hidKbdTaskId);
        
    
    VOID GAPBondMgr_Register(&ancsApp_BondMgrCBs);
    return ( events ^ START_DEVICE_EVT );
  }
  
  if ( events & HID_TEST_EVT )
  {
   //hidKbdSendReport(0x50);
    s_cnt++;
    if((s_cnt%5) == 0){
      hidKbdSendReport(0x28);
      hidKbdSendReport(0);
    }else{
      hidKbdSendReport(4+(s_cnt%5));
      hidKbdSendReport(0);
    }

    /*uint16 conn_handle;
     GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
      LOG("Bonding Successful\n");
      ble_ancs_start_descovery(conn_handle);*/
    return ( events ^ HID_TEST_EVT );
  }

   return 0;
}

/*********************************************************************
 * @fn      hidKbd_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void hidKbd_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {

    case GATT_MSG_EVENT:
      hidKbd_ProcessGattMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      break;
  }
}


/*********************************************************************
 * @fn      hidKbd_ProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void hidKbd_ProcessGattMsg( gattMsgEvent_t *pMsg )
{
  ble_ancs_handle_gatt_event(pMsg);
  //GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      hidKbdSendReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void hidKbdSendReport( uint8 keycode )
{
  uint8 buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = 0;         // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report( HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf );
}

/*********************************************************************
 * @fn      hidKbdSendMouseReport
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   buttons - Mouse button code
 *
 * @return  none
 */
void hidKbdSendMouseReport( uint8 buttons )
{
  uint8 buf[HID_MOUSE_IN_RPT_LEN];

  buf[0] = buttons;   // Buttons
  buf[1] = 0;         // X
  buf[2] = 0;         // Y
  buf[3] = 0;         // Wheel
  buf[4] = 0;         // AC Pan

  HidDev_Report( HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                 HID_MOUSE_IN_RPT_LEN, buf );
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
                             uint8 oper, uint8 *pLen, uint8 *pData )
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
*********************************************************************/
