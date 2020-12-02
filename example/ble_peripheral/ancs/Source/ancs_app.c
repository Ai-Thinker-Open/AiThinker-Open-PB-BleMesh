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
#include <string.h>
#include "stdio.h" 

#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "gatt_profile_uuid.h"
#include "devinfoservice.h"
#include "ota_app_service.h"

#include "ll.h"

#include "hci_tl.h"
#include "gatt_uuid.h"

#include "peripheral.h"
#include "osal_snv.h"
#include "log.h"




// ANCS App includes.
#include "ancs_app.h"
#include "ble_ancs.h"


/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL                  160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE                     GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL             40

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL             60

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY                 0

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT                  1000

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST                 TRUE//GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL                 6

// How often to perform periodic event (in msec)
#define ANCSAPP_PERIODIC_EVT_PERIOD                   5000

// Application specific event ID for HCI Connection Event End Events
#define ANCSAPP_HCI_CONN_EVT_END_EVT                  0x0001



// Task configuration
#define ANCSAPP_TASK_PRIORITY                         1

#define IO_BUTTON_LEFT                                8

#define IO_BUTTON_RIGHT                               16

#define IO_BUTTON_BOTH                                24

// ANCS: 7905F431-B5CE-4E99-A40F-4B1E122D00D0
#define ANCSAPP_ANCS_SVC_UUID 0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4, 0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79
// Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD (notifiable)
#define ANCSAPP_NOTIF_SRC_CHAR_UUID                   0x1DBD
// Control point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (writable with response)
#define ANCSAPP_CTRL_PT_CHAR_UUID                     0xD9D9
// Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable)
#define ANCSAPP_DATA_SRC_CHAR_UUID                    0x7BFB

#define CHAR_DESC_HDL_UUID128_LEN                     21 // (5 + 16) bytes = 21 bytes.

#define NUMBER_OF_ANCS_CHARS                          3

#define LAST_ANCS_CHAR                                1

#ifdef USE_WATCHDOG_TIMER
  #define WATCHDOG_TIMER_TIMEOUT_PERIOD                 1500000 * 5 // 1 second * 5
  #define ANCSAPP_PERIODIC_EVT                          Event_Id_02
#endif

// Application events
#define ANCSAPP_STATE_CHANGE_EVT                      0x0001
#define ANCSAPP_CHAR_CHANGE_EVT                       0x0002
#define ANCSAPP_PAIRING_STATE_EVT                     0x0004
#define ANCSAPP_PASSCODE_NEEDED_EVT                   0x0008
#define ANCSAPP_KEY_CHANGE_EVT                        0x0010


//task event
#define START_DEVICE_EVT                              0x0001
#define ANCSAPP_START_DISC_EVT                        0x0002


// Internal Events for RTOS application.
#define ANCSAPP_QUEUE_EVT                             0x61 // Event_Id_30




/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  osal_event_hdr_t hdr; //!< GATT_MSG_EVENT and status
  uint8_t evt;
  uint8_t *pData;  // event data
} ancsAppEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Store discovered handles
uint16_t Ancs_handleCache[HDL_CACHE_LEN];

// Display Interface
//Display_Handle dispHandle = NULL;

// Watchdog handle
//Watchdog_Handle watchdogHandle;
/*********************************************************************
 * LOCAL VARIABLES
 */






// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x0A,// length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'A', 'N', 'C', 'S', ' ', 'D', 'e', 'm', 'o',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,// length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // Service Solicitation: this peripheral (NC) is looking for the ANCS
  // on the iOS device. As per Apple Bluetooth Design Guidelines, soliciting
  // the ANCS will cause the device to show up in the iOS settings app.
  0x11, // length of this data
  GAP_ADTYPE_SERVICES_LIST_128BIT,
  // The ANCS's UUID.
  ANCSAPP_ANCS_SVC_UUID
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "ANCS Demo";

uint16_t gapConnHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

//static uint8_t AncsApp_processStackMsg(ICall_Hdr *pMsg);
static uint8_t AncsApp_processGATTMsg(gattMsgEvent_t *pMsg);
static void AncsApp_processStateChangeEvt(gaprole_States_t newState);
static void AncsApp_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle, uint8_t uiInputs, uint8_t uiOutputs);
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void AncsApp_processPairState(uint8_t state, uint8_t status);
static void AncsApp_processPasscode(uint16_t connHandle, uint8_t uiOutputs);
static void AncsApp_stateChangeCB(gaprole_States_t newState);
/********************ANCS APP FUNCTIONS********************/

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
 * PROFILE CALLBACKS
 */

uint8 AncsApp_TaskID;

// Peripheral GAPRole Callbacks
static gapRolesCBs_t ancsApp_gapRoleCBs =
{
  AncsApp_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t ancsApp_BondMgrCBs =
{
  (pfnPasscodeCB_t) AncsApp_passcodeCB, // Passcode callback
  AncsApp_pairStateCB                   // Pairing / Bonding state Callback
};

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

/*********************************************************************
 * @fn      AncsApp_createTask
 *
 * @brief   Task creation function for the ANCS app.
 *
 * @param   None.
 *
 * @return  None.
 */

/*********************************************************************
 * @fn      AncsApp_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
void AncsApp_init( uint8 task_id )
{
  AncsApp_TaskID = task_id;
  LOG_INIT(); 
  LOG("AncsApp_init\n"); 

  LOG("start!\n");

  if(hal_gpio_read(P14)){
    LOG("Erase all bounding\n");
    GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
  }


  // Set GAP Parameters: After a connection was established, delay in seconds
  // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
  // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
  // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
  // For current defaults, this has no effect.
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);


  {
    // Device starts advertising upon initialization of GAP
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until re-enabled by the application
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the Peripheral GAPRole Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  {
    // Use the same interval for general and limited advertising.
    // Note that only general advertising will occur based on the above configuration
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  {
    uint32 passkey = 0; // passkey "000000"
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    // Use authenticated pairing: require passcode.

#if(USE_PASSKEY==0)
    uint8_t mitm = FALSE;
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
#else
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
#endif
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service

  ota_app_AddService();

  // For ANCS, the device must register an a GATT client, whereas the
  // iPhone acts as a GATT server.

   ble_ancs_init(on_ancs_evt, AncsApp_TaskID);
 
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,m_attr_appid,ANCS_ATTR_DATA_MAX);
  
  //ble_ancs_attr_add(BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME,m_attr_disp_name,sizeof(m_attr_disp_name));
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_TITLE,m_attr_title,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE,m_attr_message,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,m_attr_subtitle,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,m_attr_message_size,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_DATE,m_attr_date,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,m_attr_posaction,ANCS_ATTR_DATA_MAX);
  
  ble_ancs_attr_add(BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,m_attr_negaction,ANCS_ATTR_DATA_MAX);



  LOG("ANCS Demo\n");
  osal_set_event( AncsApp_TaskID, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      AncsApp_taskFxn
 *
 * @brief   Application task entry point for the ANCS App.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
uint16  AncsApp_ProcessEvent( uint8 task_id, uint16 events )
{

      // Fetch any available messages that might have been sent from the stack
      if ( events & SYS_EVENT_MSG )
      {
        uint8 *pMsg;
        
        if ( (pMsg = osal_msg_receive( AncsApp_TaskID )) != NULL )
        {

          AncsApp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
          //AncsApp_processStackMsg(pMsg);
          osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
      }

      if ( events & START_DEVICE_EVT )
      {
        GATT_InitClient();
        GATT_RegisterForInd(AncsApp_TaskID);
        
        // Start the Device
        VOID GAPRole_StartDevice(&ancsApp_gapRoleCBs);
        
        // Start Bond Manager and register callback
        VOID GAPBondMgr_Register(&ancsApp_BondMgrCBs);
        
        //GAPRole_StartDevice( &ancsApp_gapRoleCBs );
      
        // Register with bond manager after starting device
        //GAPBondMgr_Register( (gapBondCBs_t *) &ancsApp_BondMgrCBs );
        
        return ( events ^ START_DEVICE_EVT );
      }
      // Service discovery event.
      return 0;
}



static void AncsApp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case GATT_MSG_EVENT:
      AncsApp_processGATTMsg((gattMsgEvent_t *)pMsg );
      break;
    
    default:
      // Do nothing.
      break;
  }
}




static uint8_t AncsApp_processGATTMsg(gattMsgEvent_t *pMsg)
{

  ble_ancs_handle_gatt_event(pMsg);

  //ANCS requires authentication, if the NP attempts to read/write chars on the
  //NP without proper authentication, the NP will respond with insufficent_athen
  //error to which we must respond with a slave security request
  if  (pMsg->method == ATT_ERROR_RSP &&
            pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ &&
            pMsg->msg.errorRsp.errCode == ATT_ERR_INSUFFICIENT_AUTHEN)
  {
    uint16 conn_handle;
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &conn_handle);
    uint8_t mitm;
    uint8_t bonding;
    GAPBondMgr_GetParameter(GAPBOND_MITM_PROTECTION, &mitm);
    GAPBondMgr_GetParameter(GAPBOND_BONDING_ENABLED, &bonding);
    uint8_t authRequest = ((mitm & 0x01) << 2) | ((bonding & 0x01) << 1) | (bonding & 0x01);

    GAP_SendSlaveSecurityRequest(conn_handle, authRequest);
  }


  // It's safe to free the incoming message
  return (TRUE);
}



/*********************************************************************
 * @fn      AncsApp_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void AncsApp_stateChangeCB(gaprole_States_t newState)
{
  AncsApp_processStateChangeEvt(newState);
}

/*********************************************************************
 * @fn      AncsApp_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void AncsApp_processStateChangeEvt(gaprole_States_t newState)
{

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

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

        // Display device address
        //LOG(Util_convertBdAddr2Str(ownAddress));
        LOG("Initialized\n");
      }
      break;

    case GAPROLE_ADVERTISING:

      LOG("Advertising\n");
      break;

/*#ifdef PLUS_BROADCASTER
    // After a connection is dropped, a device in PLUS_BROADCASTER will continue
    // sending non-connectable advertisements and shall send this change of
    // state to the application.  These are then disabled here so that sending
    // connectable advertisements can resume.
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        AncsApp_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER*/

    case GAPROLE_CONNECTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        // get connection handle
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);

        uint8_t peerAddress[B_ADDR_LEN];
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        LOG("Connected\n");
        //LOG(Util_convertBdAddr2Str(peerAddress));


        LOG("Discovery Progress:\t0\n");
        LOG("Discovery State:\tConnected\n");



      }
      break;

    case GAPROLE_CONNECTED_ADV:
      LOG("Connected Advertising\n");
      break;

    case GAPROLE_WAITING:
      // Free the ATT response, clear the handle cache, and clear the ANCS notification queue.
      //AncsApp_freeAttRsp(bleNotConnected);
      VOID memset(Ancs_handleCache, '\0', HDL_CACHE_LEN*2);
      LOG("Disconnected\n");
      LOG("Discovery Progress:\t0\n");
      LOG("Discovery State:\tDisconnected\n");

      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      // Free the ATT response, clear the handle cache, and clear the ANCS notification queue.

      //AncsApp_freeAttRsp(bleNotConnected);
      VOID memset(Ancs_handleCache, '\0', HDL_CACHE_LEN*2);
      LOG("Timed Out");
      LOG("Discovery Progress:\t0\n");
      LOG("Discovery State:\tTimed out\n");

      break;

    case GAPROLE_ERROR:
      LOG("Error\n");
      break;

    default:
      break;
  }

}

/*********************************************************************
 * @fn      AncsApp_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void AncsApp_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
  AncsApp_processPairState(state, status);
}

/*********************************************************************
 * @fn      AncsApp_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void AncsApp_processPairState(uint8_t state, uint8_t status)
{
  LOG("AncsApp_processPairState %d,%d\n",state, status);
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    LOG("Pairing started\n");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      LOG("Pairing Successful\n");

      // Now that the device has successfully paired to the iPhone,
      // the subscription will not fail due to insufficient authentication.
     ble_ancs_start_descovery(gapConnHandle);
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
      LOG("Bonding Successful\n");
      ble_ancs_start_descovery(gapConnHandle);
    }
  }

}

static void AncsApp_processPasscode(uint16_t connHandle, uint8_t uiOutputs)
{
  // This app uses a default passcode. A real-life scenario would handle all
  // pairing scenarios and likely generate this randomly.
  uint32_t passcode = 0;
  char passcode_str[32];
  LL_Rand((uint8_t*)(&passcode), 4);
	passcode = (passcode %1000000);
  sprintf(passcode_str, "Passcode: %.6d\n", passcode);
  // Display passcode to user
  if (uiOutputs != 0)
  {
    LOG(passcode_str);
  }


  // Send passcode response
  GAPBondMgr_PasscodeRsp(connHandle, SUCCESS, passcode);
}

static void AncsApp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs)
{
  AncsApp_processPasscode(connHandle, uiOutputs);
}




/*********************************************************************
*********************************************************************/
