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

#include <string.h>
#include "types.h"
#include "bcomdef.h"
#include "simpleGATTprofile_ota.h"
#include "bleuart_service.h"
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "pwrmgr.h"

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

#include "bleuart_at_cmd.h"
#include "cliface.h"
#include "led.h"
#include "bleuart_at_dma.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
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

// advertising data length. 3+4+18 = 25
#define BLEUART_ADV_DATA_LEN                  25

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15



uint8 bleuart_TaskID;   // Task ID for internal task/event processing

uint8 led_flag;   // Task ID for internal task/event processing


uint16 gapConnHandle;

static bool g_uart_at_mod = true;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[AT_MODULE_NAME_SIZE + 11] = {0};  // 11 = 2+6+3 is fixed number.

// advert data for bleuart
static uint8 advertData[BLEUART_ADV_DATA_LEN] = {0};  // 3+4+18 = 25 is fixed number.
static uint16 company_ID = 0x6222;
// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "phy_01";



/*********************************************************************
    LOCAL FUNCTIONS
*/
static void bleuart_StateNotificationCB( gaprole_States_t newState );

/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t bleuart_PeripheralCBs =
{
    bleuart_StateNotificationCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller (not used by application)
};


/*********************************************************************
    PUBLIC FUNCTIONS
*/
bool get_uart_at_mod()
{
    return g_uart_at_mod;
}

void set_uart_at_mod(bool at_mod)
{
    g_uart_at_mod = at_mod;
}

void on_bleuartServiceEvt(bleuart_Evt_t* pev)
{
    switch(pev->ev)
    {
    case bleuart_EVT_TX_NOTI_DISABLED:
        //BUP_disconnect_handler();
        osal_set_event(bleuart_TaskID,BUP_OSAL_EVT_NOTIFY_DISABLE);
        break;

    case bleuart_EVT_TX_NOTI_ENABLED :
        //BUP_connect_handler();
        osal_set_event(bleuart_TaskID,BUP_OSAL_EVT_NOTIFY_ENABLE);
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
    switch(pev->ev)
    {
    }
}

// In case Name, connect interval, and RF_power are changed, this data should be re-build again.
void bleuart_gen_scanRspData(uint8_t* name, uint16_t* cint, uint8_t rf_pw)
{
    uint8 len = 0;
    uint8 idx = 0;
    uint8 real_rfpw;

    switch(rf_pw)
    {
    case 0:
        real_rfpw = RF_PHY_TX_POWER_5DBM;  //RF_PHY_TX_POWER_5DBM. <rf_phy_driver.h>
        break;

    case 1:
        real_rfpw = RF_PHY_TX_POWER_0DBM;  //RF_PHY_TX_POWER_0DBM
        break;

    case 2:
        real_rfpw = RF_PHY_TX_POWER_N5DBM;  //RF_PHY_TX_POWER_N5DBM
        break;

    case 3:
        real_rfpw = RF_PHY_TX_POWER_N20DBM;  //RF_PHY_TX_POWER_N20DBM
        break;

    case 4:
        real_rfpw = RF_PHY_TX_POWER_EXTRA_MAX;  //RF_PHY_TX_POWER_EXTRA_MAX
        break;

    default:
        real_rfpw = RF_PHY_TX_POWER_5DBM;  //use default power value in case not valid. Should not happen.
    }

    VOID osal_memset( scanRspData, 0, AT_MODULE_NAME_SIZE + 11 );
    scanRspData[idx++] = AT_MODULE_NAME_SIZE + 1;
    scanRspData[idx++] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    len = strlen((char*)name);  //GAP_ADTYPE_LOCAL_NAME_COMPLETE

    if(len > AT_MODULE_NAME_SIZE)
        len = AT_MODULE_NAME_SIZE;

    strncpy((char*)(&scanRspData[idx]), (char*)name, len);
    idx = AT_MODULE_NAME_SIZE + 2;
    scanRspData[idx++] = 0x05; // len of following 5 bytes.
    scanRspData[idx++] = GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE;
    scanRspData[idx++] = LO_UINT16(cint[0]);
    scanRspData[idx++] = HI_UINT16(cint[0]);
    scanRspData[idx++] = LO_UINT16(cint[1]);
    scanRspData[idx++] = HI_UINT16(cint[1]);
    scanRspData[idx++] = 0x02;  // len of following 2 bytes.
    scanRspData[idx++] = GAP_ADTYPE_POWER_LEVEL;
    scanRspData[idx++] = real_rfpw;
}

// In case search_uuid and adv_data are re-set, this data should be re-build again.
// 3+4+18 = 25
void bleuart_gen_AdvData(uint16_t search_uuid, uint8_t* adv_data)
{
    uint8 len = 0;
    uint8 idx = 0;
    VOID osal_memset( advertData, 0, BLEUART_ADV_DATA_LEN );
    advertData[idx++] = 0x02;  // len of following 2 bytes.
    advertData[idx++] = GAP_ADTYPE_FLAGS;
    advertData[idx++] = GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED;
    advertData[idx++] = 0x03;  // len of following 3 bytes.
    advertData[idx++] = GAP_ADTYPE_16BIT_MORE;
    advertData[idx++] = LO_UINT16(search_uuid);
    advertData[idx++] = HI_UINT16(search_uuid);
    advertData[idx++] = 0x11;  // len of following 17 bytes.
    advertData[idx++] = GAP_ADTYPE_MANUFACTURER_SPECIFIC;
    advertData[idx++] = LO_UINT16(company_ID);
    advertData[idx++] = HI_UINT16(company_ID);
    len = strlen((char*)adv_data);   //AT_ADV_DATA_SIZE

    if(len > AT_ADV_DATA_SIZE)
        len = AT_ADV_DATA_SIZE;

    advertData[idx++] = len;
    strncpy((char*)(&advertData[idx]), (char*)adv_data, len);
}

// This function would be called by AT cmds.
// true: update scan rsp data parameters.
void update_AdvDataFromAT(bool is_scan_rsp)
{
    if(is_scan_rsp)
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    else
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
}

void update_dle()
{
    uint8_t dle_max = 251;
    llInitFeatureSet2MPHY(TRUE);
    llInitFeatureSetDLE(FALSE);
    llInitFeatureSetDLE(TRUE);
    uint16 txTime = (dle_max+10+4)<<3;
    HCI_LE_SetDataLengthCmd(0,dle_max, txTime);
}

void bleuart_Init( uint8 task_id )
{
    uint8_t ret = 1;
    AT_ctx_t m_at_ctx = {0};
    bleuart_TaskID = task_id;
    uint8_t t_rfpw = 0;
    at_initialize_fs();  // initialize the fs before further OPs.
    ret = at_snv_read_flash(); // read data from flash

    if(SUCCESS == ret)
    {
        // Get parameters from at module for seting up connection
        at_get_ctx(&m_at_ctx);
    }
    else
    {
        // Use default value in case read flash fail
        at_default(0,NULL);
        at_get_ctx_def(&m_at_ctx);
    }

    #ifdef BLEUART_DEDICATE

    if(m_at_ctx.baudrate[0] == 0 )   //make sure we have default baudrate at the first boot.
    {
    #else

    if(m_at_ctx.baudrate == 0 )   //make sure we have default baudrate at the first boot.
    {
    #endif
        at_default(0,NULL);
        at_get_ctx_def(&m_at_ctx);
    }

    if(at_get_led_mode())
    {
        led_initial(LED_GPIO_PIN);
        led_set_status(LED_STATUS_ON);
        led_set_status(LED_STATUS_OFF);
        led_set_status(LED_STATUS_ON);
        led_set_status(LED_STATUS_OFF);
    }

    /* update bd_addr set by AT cmd. */
    at_update_bd_addr();
    //update advertise data.
    bleuart_gen_scanRspData(m_at_ctx.mod_name, m_at_ctx.conn_int, m_at_ctx.rfpw);
    bleuart_gen_AdvData(m_at_ctx.search_uuid, attDeviceName);

    //update RF power.
    switch(m_at_ctx.rfpw)
    {
    case 0:
        t_rfpw = RF_PHY_TX_POWER_5DBM;
        break;

    case 1:
        t_rfpw = RF_PHY_TX_POWER_0DBM;
        break;

    case 2:
        t_rfpw = RF_PHY_TX_POWER_N5DBM;
        break;

    case 3:
        t_rfpw = RF_PHY_TX_POWER_N20DBM;
        break;

    case 4:
        t_rfpw = RF_PHY_TX_POWER_EXTRA_MAX;
        break;

    default: //should not be here.
        t_rfpw = RF_PHY_TX_POWER_5DBM;
        break;
    }

    rf_phy_set_txPower(t_rfpw);
    // update bleuart profile attributes table.
    update_Bleuart_ProfileAttrTbl( m_at_ctx.srv_uuid, m_at_ctx.pt_uuid);
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
        uint16 desired_min_interval = m_at_ctx.conn_int[0];
        uint16 desired_max_interval = m_at_ctx.conn_int[1];
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = m_at_ctx.conn_timeout;
        uint8 peerPublicAddr[] =
        {
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06
        };

        // slave only at the moment. if connectable is true, set advType into GAP_ADTYPE_ADV_SCAN_IND.
        if(m_at_ctx.connectable)
        {
            uint8 advType = GAP_ADTYPE_ADV_SCAN_IND;
            GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
        }

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
    strcpy((char*)attDeviceName, (char*)(m_at_ctx.mod_name));    // Harris. 0821
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Set advertising interval
    {
        uint16_t advInt = 0;
        advInt = m_at_ctx.adv_int;
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP  0xFFFFFFFF
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    bleuart_AddService(on_bleuartServiceEvt);
    //update_mtu_llPHY_DLE();  // update mtu=247, llPHY 2M
    at_Init(); // initial uart for AT cmd first.

    if(m_at_ctx.auto_slp_time == 0) // Use default value(20s)in case it's still not set before.
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, 20 * 1000 );
    else
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, m_at_ctx.auto_slp_time * 1000 );

    hal_pwrmgr_register(MOD_USR1, gpio_sleep_handle, gpio_wakeup_handle);
    hal_pwrmgr_lock(MOD_USR1);
    // Setup a delayed profile startup
    osal_set_event( bleuart_TaskID, BUP_OSAL_EVT_START_DEVICE );
}

/*********************************************************************
    @fn      SimpleBLEPeripheral_ProcessEvent

    @brief   Simple BLE Peripheral Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 bleuart_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( bleuart_TaskID )) != NULL )
        {
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & BUP_OSAL_EVT_START_DEVICE )
    {
        VOID GAPRole_StartDevice( &bleuart_PeripheralCBs );
        return ( events ^ BUP_OSAL_EVT_START_DEVICE );
    }

//    case BUP_OSAL_EVT_BLE_TIMER:  //
    if ( events & BUP_OSAL_EVT_BLE_TIMER )
    {
        LOG("BUP_OSAL_EVT_BLE_TIMER\n");
        BUP_data_BLE_to_uart_send();
        return ( events ^ BUP_OSAL_EVT_BLE_TIMER );
    }

//    case BUP_OSAL_EVT_RESET_ADV:  // enable adv
    if ( events & BUP_OSAL_EVT_RESET_ADV )
    {
        uint8 initial_advertising_enable = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        return ( events ^ BUP_OSAL_EVT_RESET_ADV );
    }

//    case BUP_OSAL_EVT_UARTRX_TIMER:  //
    if ( events & BUP_OSAL_EVT_UARTRX_TIMER )
    {
        LOG("BUP_OSAL_EVT_UARTRX_TIMER\n");
        BUP_data_uart_to_BLE_send();
        return ( events ^ BUP_OSAL_EVT_UARTRX_TIMER );
    }

//    case BUP_OSAL_EVT_UART_TX_COMPLETE:  //
    if ( events & BUP_OSAL_EVT_UART_TX_COMPLETE )
    {
        LOG("BUP_OSAL_EVT_UART_TX_COMPLETE\n");
        BUP_data_BLE_to_uart_completed();
        return ( events ^ BUP_OSAL_EVT_UART_TX_COMPLETE);
    }

//    case BUP_OSAL_EVT_UART_TO_TIMER:  // uart-ble-app path.
    if ( events & BUP_OSAL_EVT_UART_TO_TIMER )
    {
        LOG("BUP_OSAL_EVT_UART_TO_TIMER\n");
        BUP_data_uart_to_BLE();
        return ( events ^ BUP_OSAL_EVT_UART_TO_TIMER);
    }

//    case BUP_OSAL_EVT_AT_UART_RX_CMD:  // Handle AT cmds.
    if ( events & BUP_OSAL_EVT_AT_UART_RX_CMD )
    {
        LOG("BUP_OSAL_EVT_AT_UART_RX_EVT\n");

        if (('\r' == cmdstr[cmdlen - 1]) || ('\n' == cmdstr[cmdlen - 1]) || (' ' == cmdstr[cmdlen - 1]))
        {
            //cmdstr[cmdlen - 1] = '\0';
            CLI_process_line
            (
                cmdstr,
                cmdlen,
                (CLI_COMMAND*) cli_cmd_list,
                (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND))
            );
            cmdlen = 0;
            memset(cmdstr, 0, 64); // clean the cdmstr with fixed len = 64.
        }

        return ( events ^ BUP_OSAL_EVT_AT_UART_RX_CMD);
    }

//    case BUP_OSAL_EVT_AT_AUTO_SLEEP:  // switch into corresponding power mode once AT timeout(20s by default)
    if ( events & BUP_OSAL_EVT_AT_AUTO_SLEEP )
    {
        uint8_t mpw_mod = 0;
        mpw_mod = at_get_pw_mode();
        AT_LOG("pw_mod: %0x\n",mpw_mod);

        switch(mpw_mod)
        {
        case 0:  // normal mode. make sure it would not enable sleep
        {
            hal_pwrmgr_lock(MOD_USR1); //disable sleep here.
            break;
        }

        case 1:
        case 2:  // sleep mode.
        {
            if(at_get_led_mode())
            {
                led_set_status(LED_STATUS_OFF);
            }

            hal_pwrmgr_unlock(MOD_USR1); //enable sleep here.
            break;
        }

        default: // Do nothing.
            AT_LOG("ERR_PW_MOD\n");
            break;
        }

        return ( events ^ BUP_OSAL_EVT_AT_AUTO_SLEEP);
    }

//    case BUP_OSAL_EVT_AT_BLE_CONNECT:  // module is connected. no power save mode.
    if ( events & BUP_OSAL_EVT_AT_BLE_CONNECT )
    {
        osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP); // stop AT timer.
        BUP_connect_handler(); // set mBUP_Ctx.conn_state to TRUE
        update_dle();  //comment it out as it's not stable.
        BUP_init();  // reinitialize the uart port on pass-through purpose.

        //set_uart_at_mod(false);
        if(at_get_led_mode() && (at_get_pw_mode() == 0))
        {
            led_set_status(LED_STATUS_BLINK);
        }

        if(at_get_dma_flag())  // for DMA PT
        {
            if(at_get_rxpath_flag())  // initialize DMA ch0 here.
            {
                at_dma_rx_init();
                at_dma_uart_to_BLE_DMA_rx();
            }
            else
            {
                at_dma_tx_init();
            }
        }

        return ( events ^ BUP_OSAL_EVT_AT_BLE_CONNECT );
    }

//    case BUP_OSAL_EVT_AT_BLE_DISCONNECT:  // module is disconnected
    if ( events & BUP_OSAL_EVT_AT_BLE_DISCONNECT )
    {
        uint32_t m_auto_slp_time = at_get_auto_slp_time();
        BUP_disconnect_handler(); // set mBUP_Ctx.conn_state to false

        if(at_get_dma_flag())  // for DMA PT
        {
            at_dma_deinit();
        }

        at_Init(); // initial uart for AT cmd first.
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, m_auto_slp_time * 1000 );

        if(at_get_led_mode() && (at_get_pw_mode() == 0))
        {
            osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_LED_BLK_TIMER);
            led_set_status(LED_STATUS_OFF);
        }

        return ( events ^ BUP_OSAL_EVT_AT_BLE_DISCONNECT );
    }

//    case BUP_OSAL_EVT_NOTIFY_ENABLE:  // Notify enabled.
    if ( events & BUP_OSAL_EVT_NOTIFY_ENABLE )
    {
        LOG("BUP_OSAL_EVT_NOTIFY_ENABLE\n");
        BUP_connect_handler(); // set mBUP_Ctx.conn_state to TRUE
        set_Bleuart_Notify();
        return ( events ^ BUP_OSAL_EVT_NOTIFY_ENABLE );
    }

//    case BUP_OSAL_EVT_NOTIFY_DISABLE:  // Notify disabled.
    if ( events & BUP_OSAL_EVT_NOTIFY_DISABLE )
    {
        LOG("BUP_OSAL_EVT_NOTIFY_DISABLE\n");
        clear_Bleuart_Notify();
        return ( events ^ BUP_OSAL_EVT_NOTIFY_DISABLE );
    }

//    case BUP_OSAL_EVT_LED_BLK_TIMER:  // LED blinking.
    if ( events & BUP_OSAL_EVT_LED_BLK_TIMER )
    {
        LOG("BUP_OSAL_EVT_LED_BLK_TIMER\n");

        //if(u_parity == LED_PWR_ON){ // switch LED status regularly. per 1S
        if(led_flag == 0)
        {
            led_set_status(LED_STATUS_OFF);
            led_flag = 1;
        }
        else
        {
            led_set_status(LED_STATUS_ON);
            led_flag = 0;
        }

        osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_LED_BLK_TIMER);
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_LED_BLK_TIMER, 447 );
        return ( events ^ BUP_OSAL_EVT_LED_BLK_TIMER );
    }

    // This msg is dup-used by Rx p2m DMA and Tx copy BLE APP data
//    case BUP_OSAL_EVT_UART_DATA_RX:  // for DMA only
    if ( events & BUP_OSAL_EVT_UART_DATA_RX )
    {
        LOG("BUP_OSAL_EVT_UART_DATA_RX\n");

        if(at_get_rxpath_flag())   // For Rx path, triggered by ch0 p2m DMA callback -- dma_rx_cb0()
        {
            at_dma_uart_to_BLE_DMA_rx();

            if(!at_dma_get_notify_flag())
            {
                at_dma_set_notify_flag(true);
                at_dma_uart_to_BLE_notify_data();
            }
        }
        else   // // For Tx path, triggered by BLE APP data -- BUP_data_BLE_to_uart()
        {
            if(!at_dma_get_send_flag())
            {
                at_dma_set_send_flag(true);
                at_dma_BLE_to_uart_DMA_tx();
            }
        }

        return ( events ^ BUP_OSAL_EVT_UART_DATA_RX );
    }

    /*  Skip these events as they are not used at the moment.
            case BUP_OSAL_EVT_CCCD_UPDATE:{ //
                LOG("BUP_OSAL_EVT_CCCD_UPDATE\n");
                return ( events ^ BUP_OSAL_EVT_CCCD_UPDATE );
            }
    */
//    default: // do nothing
//        break;
//    }
    // Discard unknown events
    return 0;
}




/*********************************************************************
    @fn      peripheralStateNotificationCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
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
        AT_LOG("advertising!\n");
        break;

    case GAPROLE_CONNECTED:
        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );
        osal_set_event( bleuart_TaskID, BUP_OSAL_EVT_AT_BLE_CONNECT );
        AT_LOG("connected handle[%d]!\n", gapConnHandle);
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
