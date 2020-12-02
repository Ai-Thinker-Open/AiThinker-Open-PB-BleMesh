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
  Filename:       simpleEddystoneBeacon.c

  Description:    This file contains the Simple Eddystone Beacon sample
                  application for use with the CC2650 Bluetooth Low Energy
                  Protocol Stack.


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "hci.h"
#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "eddystoneCfg.h"


#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"

   
#include "eddystoneCfg.h"
#include "simpleEddystoneBeacon.h"

#include "global_config.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800


// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Task configuration
#define SEB_TASK_PRIORITY                     1
  
// Internal Events for RTOS application
#define SEB_STATE_CHANGE_EVT                  0x0001
#define SEB_KEY_CHANGE_EVT                    0x0002
#define SEB_CONN_EVT_END_EVT                  0x0008
#define SEB_CHAR_CHANGE_EVT                   0x0010

// Eddystone definitions
#define EDDYSTONE_SERVICE_UUID                  0xFEAA
   
#define EDDYSTONE_FRAME_TYPE_UID                0x00
#define EDDYSTONE_FRAME_TYPE_URL                0x10
#define EDDYSTONE_FRAME_TYPE_TLM                0x20
  
#define EDDYSTONE_FRAME_OVERHEAD_LEN            8
#define EDDYSTONE_SVC_DATA_OVERHEAD_LEN         3
#define EDDYSTONE_MAX_URL_LEN                   18

// # of URL Scheme Prefix types
#define EDDYSTONE_URL_PREFIX_MAX        4
// # of encodable URL words
#define EDDYSTONE_URL_ENCODING_MAX      14

// Eddystone advertise selection, could be UID only, URL only and (UID + URL)
#define EDDYSTONE_FRAME_UID             0
#define EDDYSTONE_FRAME_URL             1
#define EDDYSTONE_FRAME_UID_URL         2
  
/*********************************************************************
 * TYPEDEFS
 */
// Eddystone UID frame
typedef struct
{
  uint8_t   frameType;      // UID
  int8_t    rangingData;
  uint8_t   namespaceID[10];
  uint8_t   instanceID[6];
  uint8_t   reserved[2];
} eddystoneUID_t;

// Eddystone URL frame
typedef struct
{
  uint8_t   frameType;      // URL | Flags
  int8_t    txPower;
  uint8_t   encodedURL[EDDYSTONE_MAX_URL_LEN];  // the 1st byte is prefix
} eddystoneURL_t;

// Eddystone TLM frame
typedef struct
{
  uint8_t   frameType;      // TLM
  uint8_t   version;        // 0x00 for now
  uint8_t   vBatt[2];       // Battery Voltage, 1mV/bit, Big Endian
  uint8_t   temp[2];        // Temperature. Signed 8.8 fixed point
  uint8_t   advCnt[4];      // Adv count since power-up/reboot
  uint8_t   secCnt[4];      // Time since power-up/reboot
                            // in 0.1 second resolution
} eddystoneTLM_t;

typedef union
{
  eddystoneUID_t        uid;
  eddystoneURL_t        url;
  eddystoneTLM_t        tlm;
} eddystoneFrame_t;

typedef struct
{
  uint8_t               length1;        // 2
  uint8_t               dataType1;      // for Flags data type (0x01)
  uint8_t               data1;          // for Flags data (0x04)
  uint8_t               length2;        // 3
  uint8_t               dataType2;      // for 16-bit Svc UUID list data type (0x03)
  uint8_t               data2;          // for Eddystone UUID LSB (0xAA)
  uint8_t               data3;          // for Eddystone UUID MSB (0xFE)
  uint8_t               length;         // Eddystone service data length
  uint8_t               dataType3;      // for Svc Data data type (0x16)
  uint8_t               data4;          // for Eddystone UUID LSB (0xAA)
  uint8_t               data5;          // for Eddystone UUID MSB (0xFE)
  eddystoneFrame_t      frame;
} eddystoneAdvData_t;

typedef struct
{
  uint8_t               length1;        // 2
  uint8_t               dataType1;      // for Flags data type (0x01)
  uint8_t               data1;          // for Flags data (0x06)
  uint8_t               length2;        // 17
  uint8_t               dataType2;      // for 128-bit Svc UUID list data type (0x07)
  uint8_t               data2[16];      // for Eddystone Cfg service UUID
  uint8_t               length3;        // 2
  uint8_t               dataType3;      // for Power Level data type (0x0a)
  int8_t                powerLevel;     // for Eddystone UUID LSB (0xAA)
} eddystoneCfgAdvData_t;

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
static uint8 simpleEddystone_TaskID;   // Task ID for internal task/event processing

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static eddystoneAdvData_t eddystoneAdv = 
{ 
  // Flags; this sets the device to use general discoverable mode
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  
  // Complete list of 16-bit Service UUIDs
  0x03,   // length of this data including the data type byte
  GAP_ADTYPE_16BIT_COMPLETE,
  LO_UINT16(EDDYSTONE_SERVICE_UUID),
  HI_UINT16(EDDYSTONE_SERVICE_UUID),
  
  // Service Data
  0x03, // to be set properly later
  GAP_ADTYPE_SERVICE_DATA,
  LO_UINT16(EDDYSTONE_SERVICE_UUID),
  HI_UINT16(EDDYSTONE_SERVICE_UUID)
};

eddystoneUID_t   eddystoneUID;
eddystoneURL_t   eddystoneURL;
eddystoneTLM_t   eddystoneTLM;
uint8            frameCombine = 2;
uint32           advPeriod = 1000;     // advertisement period, in ms

static eddystoneCfgAdvData_t eddystoneCfgAdv = 
{ 
  // Flags; this sets the device to use general discoverable mode
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  
  // Complete list of 128-bit Service UUIDs
  0x11,   // length of this data
  GAP_ADTYPE_128BIT_COMPLETE,
  {EDDYSTONE_BASE_UUID_128(URLCFG_SVC_UUID)},
  
  // Power Level
  0x02, // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  -2  // To be set properly later
};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x16,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'E',
  'd',
  'd',
  'y',
  's',
  't',
  'o',
  'n',
  'e',
  'B',
  'e',
  'a',
  'c',
  'o',
  'n',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
};

// Array of URL Scheme Prefices
static char* eddystoneURLPrefix[EDDYSTONE_URL_PREFIX_MAX] =
{
  "http://www.",
  "https://www.",
  "http://",
  "https://"
};

// Array of URLs to be encoded
static char* eddystoneURLEncoding[EDDYSTONE_URL_ENCODING_MAX] =
{
  ".com/",
  ".org/",
  ".edu/",
  ".net/",
  ".info/",
  ".biz/",
  ".gov/",
  ".com",
  ".org",
  ".edu",
  ".net",
  ".info",
  ".biz",
  ".gov"
};

static uint32 advCount = 0;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple ES Beacon";


// Eddystone frame type currently used
static uint8 currentFrameType = EDDYSTONE_FRAME_TYPE_UID;

// Eddystone Configuration mode
static uint8 edsCfgMode = FALSE;

// Connection status
static uint8 ConnectedInCfgMode = FALSE;

extern uint32 *pGlobal_config;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

 void SimpleEddystoneBeacon_init(uint8 task_id);

static void SimpleEddystoneBeacon_ProcessOSALMsg( osal_event_hdr_t *pMsg );

static void SimpleEddystoneBeacon_processStateChangeEvt(gaprole_States_t newState);
static void SimpleEddystoneBeacon_processCharValueChangeEvt(uint8_t paramID);
static void SimpleEddystoneBeacon_processAdvCompleteEvt(void);

static void SimpleEddystoneBeacon_charValueChangeCB(uint8_t paramID);

static void SimpleEddystoneBeacon_updateTLM(void);
static void SimpleEddystoneBeacon_initUID(void);
static void SimpleEddystoneBeacon_initConfiguration(void);
static void SimpleEddystoneBeacon_applyConfiguration(void);
static void SimpleEddystoneBeacon_selectFrame(uint8 frameType);
static void SEB_startRegularAdv(void);
static void SimpleEddystoneBeacon_startConfigAdv(void);


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleEddystoneBeacon_gapRoleCBs =
{
    SimpleEddystoneBeacon_processStateChangeEvt
};

// GAP Bond Manager Callbacks
static gapBondCBs_t SimpleEddystoneBeacon_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Eddystone URL Configuration Service Callbacks
static edsCfgSvcCBs_t SimpleEddystoneBeacon_edsCfgCBs =
{
  SimpleEddystoneBeacon_charValueChangeCB // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      SimpleEddystoneBeacon_init
 *
 * @brief   Initialization function for the Simple Eddystone Beacon App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
void SimpleEddystoneBeacon_init(uint8 task_id)
{
    simpleEddystone_TaskID = task_id;
    // Setup the GAP
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

    // Setup the GAP Peripheral Role Profile
    {
        // Don't start advertising upon initialization
        uint8_t initialAdvertEnable = FALSE;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 

        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16_t advertOffTime = 0;
      
        uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        // Initialize UID frame
        SimpleEddystoneBeacon_initUID();

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), 
                         &initialAdvertEnable);
        
        // set adv channel map
        GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);     
        
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t), 
                         &advertOffTime);
    
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), 
                         scanRspData);
    
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

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  
    // Setup the GAP Bond Manager
    {
        uint32_t passkey = 0;                       // passkey "000000"
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t mitm = TRUE;
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8_t bonding = TRUE;

        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
//    DevInfo_AddService();                        // Device Information Service

    VOID edsCfgSvc_AddService();                 // URL Configuration Service
  
    // Setup the URL Configuration Characteristic Values
    SimpleEddystoneBeacon_initConfiguration();

    // Register callback with SimpleGATTprofile
    edsCfgSvc_RegisterAppCBs(&SimpleEddystoneBeacon_edsCfgCBs);
  
    // Start the Device
    VOID GAPRole_StartDevice(&SimpleEddystoneBeacon_gapRoleCBs);
  
    // Start Bond Manager
    VOID GAPBondMgr_Register(&SimpleEddystoneBeacon_BondMgrCBs);
  
    // start a 5ms timer for broadcast eddystone-TLM frame
    VOID osal_start_timerEx( simpleEddystone_TaskID, SEB_START_DEVICE_EVT, 5);  
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processEvent
 *
 * @brief   Application task entry point for the Simple Eddystone Beacon.
 *
 * @param   none
 *
 * @return  none
 */
uint16 SimpleEddystoneBeacon_processEvent( uint8 task_id, uint16 events )
{
    VOID task_id;
    
    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( simpleEddystone_TaskID )) != NULL )
        {
            SimpleEddystoneBeacon_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & SEB_START_DEVICE_EVT)
    {
        SimpleEddystoneBeacon_startConfigAdv(); 
        
        VOID osal_start_timerEx( simpleEddystone_TaskID, SEB_ENTER_NON_CONN_ADV_EVT, 10 * 1000);
        return (events ^ SEB_START_DEVICE_EVT);
    }    

    if (events & SEB_ENTER_NON_CONN_ADV_EVT)
    {
        if (TRUE == ConnectedInCfgMode)      // if in connect state, wait 30s and query again
            VOID osal_start_timerEx( simpleEddystone_TaskID, SEB_ENTER_NON_CONN_ADV_EVT, 120 * 1000);
        else
        {
            // adv channel interval
            pGlobal_config[ADV_CHANNEL_INTERVAL] = 800;//6250;
            
            // start regular Eddystone broadcast
            SEB_startRegularAdv();
            VOID osal_start_timerEx( simpleEddystone_TaskID, SEB_ADV_COMPLETE_EVT, advPeriod);
        }
        return (events ^ SEB_ENTER_NON_CONN_ADV_EVT);
    }    
    
    if (events & SEB_ADV_COMPLETE_EVT)
    {
        uint8 advertEnabled = FALSE;
        
        // change advertisement frame type
        SimpleEddystoneBeacon_processAdvCompleteEvt();
        
        // Stop connectable advertising
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &advertEnabled);        
         
        // restart advertisement
        osal_set_event(simpleEddystone_TaskID, SEB_RESTART_ADV_EVT); 
        
        // next time to change adv type
        VOID osal_start_timerEx( simpleEddystone_TaskID, SEB_ADV_COMPLETE_EVT, advPeriod);
        return (events ^ SEB_ADV_COMPLETE_EVT);
    }

    // enable adv
    if ( events & SEB_RESTART_ADV_EVT )
    {
        uint8 initial_advertising_enable = TRUE;
		
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
        return ( events ^ SEB_RESTART_ADV_EVT );
    }      
            
    return 0;
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_initUID
 *
 * @brief   initialize UID frame
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_initUID(void)
{
    // Set Eddystone UID frame with meaningless numbers for example.
    // This need to be replaced with some algorithm-based formula
    // for production.
    eddystoneUID.namespaceID[0] = 0x00;
    eddystoneUID.namespaceID[1] = 0x01;
    eddystoneUID.namespaceID[2] = 0x02;
    eddystoneUID.namespaceID[3] = 0x03;
    eddystoneUID.namespaceID[4] = 0x04;
    eddystoneUID.namespaceID[5] = 0x05;
    eddystoneUID.namespaceID[6] = 0x06;
    eddystoneUID.namespaceID[7] = 0x07;
    eddystoneUID.namespaceID[8] = 0x08;
    eddystoneUID.namespaceID[9] = 0x09;
  
    eddystoneUID.instanceID[0] = 0x04;
    eddystoneUID.instanceID[1] = 0x51;
    eddystoneUID.instanceID[2] = 0x40;
    eddystoneUID.instanceID[3] = 0x00;
    eddystoneUID.instanceID[4] = 0xB0;
    eddystoneUID.instanceID[5] = 0x00;
    
    eddystoneUID.reserved[0] = 0x00;
    eddystoneUID.reserved[1] = 0x00;
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_encodeURL
 *
 * @brief   Encodes URL in accordance with Eddystone URL frame spec
 *
 * @param   urlOrg - Plain-string URL to be encoded
 *          urlEnc - Encoded URL. Should be EDSCFG_CHAR_URI_DATA_LEN-long.
 *
 * @return  0 if the prefix is invalid
 *          The length of the encoded URL including prefix otherwise
 */
uint8 SimpleEddystoneBeacon_encodeURL(char* urlOrg, uint8* urlEnc)
{
    uint8 i, j;
    uint8 urlLen;
    uint8 tokenLen;
  
    urlLen = (uint8) strlen(urlOrg);
  
    // search for a matching prefix
    for (i = 0; i < EDDYSTONE_URL_PREFIX_MAX; i++)
    {
        tokenLen = strlen(eddystoneURLPrefix[i]);
        if (strncmp(eddystoneURLPrefix[i], urlOrg, tokenLen) == 0)
        {
            break;
        }
    }
  
    if (i == EDDYSTONE_URL_PREFIX_MAX)
    {
        return 0;       // wrong prefix
    }
        
    // use the matching prefix number
    urlEnc[0] = i;
    urlOrg += tokenLen;
    urlLen -= tokenLen;
  
    // search for a token to be encoded
    for (i = 0; i < urlLen; i++)
    {
        for (j = 0; j < EDDYSTONE_URL_ENCODING_MAX; j++)
        {
            tokenLen = strlen(eddystoneURLEncoding[j]);
            if (strncmp(eddystoneURLEncoding[j], urlOrg + i, tokenLen) == 0)
            {
                // matching part found
                break;
            }
        }
    
        if (j < EDDYSTONE_URL_ENCODING_MAX)
        {
            memcpy(&urlEnc[1], urlOrg, i);
            // use the encoded byte
            urlEnc[i + 1] = j;
            break;
        }
    }
  
    if (i < urlLen)
    {
        memcpy(&urlEnc[i + 2],
           urlOrg + i + tokenLen, urlLen - i - tokenLen);
        return urlLen - tokenLen + 2;
    }

    memcpy(&urlEnc[1], urlOrg, urlLen);
    return urlLen + 1;
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_updateTLM
 *
 * @brief   Update TLM elements
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_updateTLM(void)
{
    uint32 time100MiliSec;
    uint32 batt;
  
    // Voltage, in mV
    batt = 1000;    // unit: mV
    eddystoneTLM.vBatt[0] = HI_UINT16(batt);
    eddystoneTLM.vBatt[1] = LO_UINT16(batt);
    
    // Temperature - 19.5 (Celcius) for example
    eddystoneTLM.temp[0] = 19;
    eddystoneTLM.temp[1] = 256 / 2;
    
    // advertise packet cnt;
    eddystoneTLM.advCnt[0] = BREAK_UINT32(advCount, 3);
    eddystoneTLM.advCnt[1] = BREAK_UINT32(advCount, 2);
    eddystoneTLM.advCnt[2] = BREAK_UINT32(advCount, 1);
    eddystoneTLM.advCnt[3] = BREAK_UINT32(advCount, 0);
    
    // running time
    time100MiliSec = osal_GetSystemClock() / 100; 
    eddystoneTLM.secCnt[0] = BREAK_UINT32(time100MiliSec, 3);
    eddystoneTLM.secCnt[1] = BREAK_UINT32(time100MiliSec, 2);
    eddystoneTLM.secCnt[2] = BREAK_UINT32(time100MiliSec, 1);
    eddystoneTLM.secCnt[3] = BREAK_UINT32(time100MiliSec, 0);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_initConfiguration
 *
 * @brief   set all URL Configuration characteristics to default values
 *
 * @param   none
 *
 * @return  none
 */
void SimpleEddystoneBeacon_initConfiguration(void)
{
    uint8 tempURLEnc[EDSCFG_CHAR_URI_DATA_LEN];
    uint8 tempUID[EDSCFG_CHAR_UID_DATA_LEN];  
    uint8 temp8;
    uint16 temp16;

    // set URI Data
    temp8 = SimpleEddystoneBeacon_encodeURL(EDSCFG_CHAR_URI_DATA_DEFAULT,
                                          tempURLEnc);
    edsCfgSvc_SetParameter(EDSCFG_URI_DATA, temp8, tempURLEnc);
    
    memcpy(tempUID, &eddystoneUID.namespaceID[0], EDSCFG_CHAR_UID_DATA_LEN);
    edsCfgSvc_SetParameter(EDSCFG_UID_DATA, EDSCFG_CHAR_UID_DATA_LEN, tempUID);    

    // set Flags
    temp8 = EDSCFG_CHAR_FLAGS_DEFAULT;
    edsCfgSvc_SetParameter(EDSCFG_FLAGS, 1, &temp8);

    // set TX Power Mode
    temp8 = EDSCFG_CHAR_TX_POWER_MODE_DEFAULT;
    edsCfgSvc_SetParameter(EDSCFG_TX_POWER_MODE, 1, &temp8);

    // set Beacon Period
    temp16 = EDSCFG_CHAR_BEACON_PERIOD_DEFAULT;
    edsCfgSvc_SetParameter(EDSCFG_BEACON_PERIOD, 2, &temp16);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_applyConfiguration
 *
 * @brief   Apply the changes maded in URL Configuration mode
 *
 * @param   none
 *
 * @return  none
 */
void SimpleEddystoneBeacon_applyConfiguration(void)
{
    int8 tempPwrLvls[4];
    uint8 tempPowerMode;
    int8 tempPower;
    uint16 tempPeriod;
    
    // update URL frame
    edsCfgSvc_GetParameter(EDSCFG_URI_DATA, eddystoneURL.encodedURL);
  
    // update TX power
    edsCfgSvc_GetParameter(EDSCFG_ADV_TX_PWR_LVLS, tempPwrLvls);
    edsCfgSvc_GetParameter(EDSCFG_TX_POWER_MODE, &tempPowerMode);
    tempPower = tempPwrLvls[tempPowerMode];

    eddystoneUID.rangingData = tempPower;
    eddystoneURL.txPower = tempPower;
      
    // update UID data
    edsCfgSvc_GetParameter(EDSCFG_UID_DATA, eddystoneUID.namespaceID);
      
    // update combination  
    edsCfgSvc_GetParameter(EDSCFG_FRAME_COMB_DATA, &frameCombine);
  
    // update adv period
    edsCfgSvc_GetParameter(EDSCFG_BEACON_PERIOD, &tempPeriod);
    if (tempPeriod != 0)
    {
        advPeriod = tempPeriod;
        // convert from ms into multiple of 625us
        tempPeriod = (uint16) (tempPeriod * 8L / 5);
        
        tempPeriod = tempPeriod * 3;
        
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, tempPeriod); 
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, tempPeriod); 
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, tempPeriod); 
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, tempPeriod); 
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_selectFrame
 *
 * @brief   Selecting the type of frame to be put in the service data
 *
 * @param   frameType - Eddystone frame type
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_selectFrame(uint8 frameType)
{
    if (frameType == EDDYSTONE_FRAME_TYPE_UID ||
      frameType == EDDYSTONE_FRAME_TYPE_URL ||
      frameType == EDDYSTONE_FRAME_TYPE_TLM)
    {
        eddystoneFrame_t*   pFrame;
        uint8               frameSize;
        uint8               temp;

        eddystoneAdv.length = EDDYSTONE_SVC_DATA_OVERHEAD_LEN;
        // Fill with 0s first
        memset((uint8*) &eddystoneAdv.frame, 0x00, sizeof(eddystoneFrame_t));
    
        switch (frameType)
        {
        case EDDYSTONE_FRAME_TYPE_UID:
          eddystoneUID.frameType = EDDYSTONE_FRAME_TYPE_UID;
          frameSize = sizeof(eddystoneUID_t);
          pFrame = (eddystoneFrame_t *) &eddystoneUID;
          break;

        case EDDYSTONE_FRAME_TYPE_URL:
          eddystoneURL.frameType = EDDYSTONE_FRAME_TYPE_URL;
          edsCfgSvc_GetParameter(EDSCFG_URI_DATA_LEN, &temp);
          frameSize = sizeof(eddystoneURL_t) - EDDYSTONE_MAX_URL_LEN + temp;
          pFrame = (eddystoneFrame_t *) &eddystoneURL;  
          break;

        case EDDYSTONE_FRAME_TYPE_TLM:
          eddystoneTLM.frameType = EDDYSTONE_FRAME_TYPE_TLM;
          frameSize = sizeof(eddystoneTLM_t);
          SimpleEddystoneBeacon_updateTLM();
          pFrame = (eddystoneFrame_t *) &eddystoneTLM;        
          break;
        }
    
        memcpy((uint8 *) &eddystoneAdv.frame, (uint8 *) pFrame, frameSize);
        eddystoneAdv.length += frameSize;
    
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA,
                         EDDYSTONE_FRAME_OVERHEAD_LEN + eddystoneAdv.length,
                         &eddystoneAdv);
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_startRegularAdv
 *
 * @brief   Start regular advertise.
 *          If configuration mode was on going, stop it.
 *
 * @param   none
 *
 * @return  none
 */
static void SEB_startRegularAdv(void)
{
    uint8 advertEnabled = FALSE;
    uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;
    uint8 tempPeriod;

    // apply the configuration defined in GATT attributes
    SimpleEddystoneBeacon_applyConfiguration();

    // Stop connectable advertising
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &advertEnabled);

    // if BeaconPeriod is 0, don't advertise.
    edsCfgSvc_GetParameter(EDSCFG_BEACON_PERIOD, &tempPeriod);
    if (tempPeriod == 0)
        return;
  
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);
  
    // Select UID or URL frame as adv data initially
    SimpleEddystoneBeacon_selectFrame(currentFrameType);
  
    // delay to wait LL response of disable adv
//    VOID osal_start_timerEx( simpleEddystone_TaskID, SEB_RESTART_ADV_EVT, 5);  
    osal_set_event(simpleEddystone_TaskID, SEB_RESTART_ADV_EVT);     
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_startConfigAdv
 *
 * @brief   Start advertising in configuration mode
 *          If regular advertising was on going, stop it.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_startConfigAdv(void)
{
    uint8 advertEnabled;
    uint8 advType;
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
    uint8 pwrLvls[4];
      
    advType = GAP_ADTYPE_ADV_IND;
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);
  
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  
    // update TX power
    edsCfgSvc_GetParameter(EDSCFG_ADV_TX_PWR_LVLS, pwrLvls);
     
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(eddystoneCfgAdv),
                         &eddystoneCfgAdv);
  
    advertEnabled = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), 
                         &advertEnabled);
}


/*********************************************************************
 * @fn      SimpleEddystoneBeacon_ProcessOSALMsg
 *
 * @brief   Process an incoming OSAL message.
 *
 * @param   pMsg - message to process
 *
 * @return  None
 */
static void SimpleEddystoneBeacon_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message

      }
      break;
      
    default:
      // do nothing
      break;
  }
  
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processAdvCompleteEvt
 *
 * @brief   Notification of a compleletion of advertise packet transmission.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_processAdvCompleteEvt(void)
{
  advCount++;
    
  if (edsCfgMode != TRUE)
  {
    if ((advCount % 10) == 0)
    {
      // Send TLM frame every 10 advertise packets
      SimpleEddystoneBeacon_selectFrame(EDDYSTONE_FRAME_TYPE_TLM);
    }
    else
    {
        // Send UID or URL
        if (EDDYSTONE_FRAME_UID_URL != frameCombine)
        {
            if (EDDYSTONE_FRAME_UID == frameCombine)
            {
                currentFrameType = EDDYSTONE_FRAME_TYPE_UID;
            }
            else
            {
                currentFrameType = EDDYSTONE_FRAME_TYPE_URL;
            }
        }     // if not URL UID combine, not change the frame type
        else if (currentFrameType == EDDYSTONE_FRAME_TYPE_URL)
        {
            currentFrameType = EDDYSTONE_FRAME_TYPE_UID;
        }
        else
        {
            currentFrameType = EDDYSTONE_FRAME_TYPE_URL;
        }
        SimpleEddystoneBeacon_selectFrame(currentFrameType);
    }
  }
}


/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_processStateChangeEvt(gaprole_States_t newState)
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

//        // Start advertising
//        SEB_startRegularAdv();
      }
      break;

    case GAPROLE_ADVERTISING:
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        ConnectedInCfgMode = TRUE;
      }
      break;

    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      if (ConnectedInCfgMode == TRUE)
      {
        ConnectedInCfgMode = FALSE;
        edsCfgMode = FALSE;
      }
      break;

    case GAPROLE_ERROR:

      break;

    default:
      break;
  }

}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_charValueChangeCB(uint8_t paramID)
{
    SimpleEddystoneBeacon_processCharValueChangeEvt(paramID);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_processCharValueChangeEvt(uint8_t paramID)
{
  switch(paramID)
  {
    case EDSCFG_RESET:
      // Initialize UID frame
      SimpleEddystoneBeacon_initUID();        
      SimpleEddystoneBeacon_initConfiguration();
      break;

    default:
      // should not reach here!
      break;
  }
}


/*********************************************************************
*********************************************************************/
