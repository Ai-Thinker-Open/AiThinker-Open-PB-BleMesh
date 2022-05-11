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
    Filename:       bleMesh.c
    Revised:
    Revision:

    Description:    This file contains the BLE Mesh application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "pwrmgr.h"

#include "gatt.h"
#include "gatt_uuid.h"
#include "hci.h"
#include "hci_tl.h"
#include "ll.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile_ota.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "bleMesh.h"
#include "uart.h"
#include "led_light.h"


#include "EM_os.h"
#include "EM_debug.h"
#include "EM_timer.h"

#include "MS_common.h"
#include "MS_prov_api.h"
#include "MS_net_api.h"
#include "MS_access_api.h"


#include "nvs.h"

#include "blebrr.h"

#include "cliface.h"

#include "mesh_clients.h"

#include "dongleKey.h"
#include "hal_keyboard_matrix.h"
#include "cli_model.h"

extern void appl_mesh_sample (void);
extern void appl_dump_bytes(UCHAR* buffer, UINT16 length);
extern void timeout_cb (void* args, UINT16 size);


/*********************************************************************
    MACROS
*/




/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
uint8 g_buttonCounter = 0;
UINT16 bleMesh_pdu_time;


/*********************************************************************
    EXTERNAL VARIABLES
*/
extern PROV_DEVICE_S UI_lprov_device;
extern  key_contex_t key_state;
extern Keys_message KeyCode;
extern uint8 enable_sleep_flag;
extern UCHAR blebrr_sleep;
extern EM_timer_handle thandle;



/*********************************************************************
    EXTERNAL FUNCTIONS
*/
void blebrr_handle_evt_adv_complete (UINT8 enable);
void blebrr_handle_evt_adv_report (gapDeviceInfoEvent_t* adv);
void blebrr_handle_evt_scan_complete (UINT8 enable);

API_RESULT blebrr_handle_le_connection_pl(uint16_t  conn_idx, uint16_t  conn_hndl, uint8_t   peer_addr_type, uint8_t*    peer_addr);

API_RESULT blebrr_handle_le_disconnection_pl(uint16_t  conn_idx, uint16_t  conn_hndl, uint8_t   reason);

void UI_lpn_seek_friend(void);

void UI_generic_onoff_set(UCHAR state);
void UI_generic_onoff_client_set(MS_NET_ADDR pub_addr,UCHAR state);

void UI_light_hsl_client_set(MS_NET_ADDR pub_addr,UINT16 lightness, UINT16 hue, UINT16 saturation);

void bleMesh_key_process(uint8 key,MS_NET_ADDR* pub_addr);

void bleMesh_GAPMsg_Timeout_Process(void);
extern void UI_vendor_model_set(UCHAR test_len,UINT16 test_index);


/*********************************************************************
    LOCAL VARIABLES
*/
uint8 bleMesh_TaskID;   // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
//static uint8 scanRspData[B_MAX_ADV_LEN];

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PHYPLUS BLE Mesh";

static gapDevDiscReq_t bleMesh_scanparam;
static gapAdvertisingParams_t bleMesh_advparam;

static UCHAR bleMesh_DiscCancel = FALSE;             // HZF? not use???


UCHAR cmdstr[64];
UCHAR cmdlen;
#if (SDK_VER_CHIP == __DEF_CHIP_QFN32__)
    #define GPIO_GREEN    P32
    #define GPIO_BLUE     P33
    #define GPIO_RED      P31
#else
    #define GPIO_GREEN    P3
    #define GPIO_BLUE     P7
    #define GPIO_RED      P2
#endif
static gpio_pin_e led_pins[3] = {GPIO_GREEN,GPIO_BLUE,GPIO_RED};
DECL_CONST CLI_COMMAND cli_cmd_list[] =
{
    /* Help */
    { "help", "Help on this CLI Demo Menu", cli_demo_help },

    /* UUID */
    { "start", "Start specifying 1st octet of UUID", cli_start },

    /* ON */
    { "on", "Send Generic ON to the Publish address configured", cli_on },

    /* OFF */
    { "off", "Send Generic OFF to the Publish address configured", cli_off },

    /* Seek for Friend */
    { "seek", "Setup Friendship", cli_seek },

//    /* Scene Store */
//    { "store", "Scene Store", cli_scene_store },
//
//    /* Scene delete */
//    { "delete", "Scene Delete", cli_scene_delete },

//    /* Scene Recall */
//    { "recall", "Scene Recall", cli_scene_recall },

//    /* HSL set */
//    { "hsl", "HSL set", cli_hsl_set },
//
//    /* CTL set */
//    { "ctl", "CTL set", cli_ctl_set },

    /* Select Group */
    { "group", "Select Mesh Group", cli_group_select },


    /* Reset */
    { "reset", "Reset the device", cli_demo_reset },

    /* internal status */
    { "status", "internal status", cli_internal_status }
};

/*********************************************************************
    LOCAL FUNCTIONS
*/
static void bleMesh_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void bleMesh_ProcessGAPMsg( gapEventHdr_t* pMsg );
static void bleMesh_ProcessL2CAPMsg( gapEventHdr_t* pMsg );
static void bleMesh_ProcessGATTMsg( gattMsgEvent_t* pMsg );
//static void key_press_process(key_evt_t key_evt,uint8 index);
void bleMesh_uart_init(void);

void UI_set_uuid_octet (UCHAR uuid_0);
void UI_light_hsl_set(UINT16 lightness, UINT16 hue, UINT16 saturation);

uint32  osal_memory_statics(void);
/*********************************************************************
    PROFILE CALLBACKS
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      bleMesh_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void bleMesh_Init( uint8 task_id )
{
    bleMesh_TaskID = task_id;
    // Register for direct HCI messages
    //HCI_GAPTaskRegister(bleMesh_TaskID);
    GAP_ParamsInit (bleMesh_TaskID, (GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL));
    GAP_CentDevMgrInit(0x80);
    GAP_PeriDevMgrInit();
    GAP_CentConnRegister();
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    GATTServApp_RegisterForMsg(task_id);
    bleMesh_uart_init();
    osal_set_event( bleMesh_TaskID, BLEMESH_START_DEVICE_EVT );
    TRNG_INIT();
    // For DLE
    llInitFeatureSet2MPHY(TRUE);
    llInitFeatureSetDLE(TRUE);
    // HCI_LE_SetDefaultPhyMode(0, 0, 0x01, 0x01);
    hal_pwrmgr_register(MOD_USR1, NULL, NULL);
    hal_pwrmgr_lock(MOD_USR1);
}


/*********************************************************************
    @fn      bleMesh_ProcessEvent

    @brief   Simple BLE Peripheral Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/

UINT8 g_counter1_on_off,g_counter2_on_off,g_counter3_on_off,g_counter4_on_off;

uint16 bleMesh_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    /* printf("\r\n >> bleMesh_ProcessEvent- TaskID %d : events 0x%04X\r\n", task_id, events); */

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( bleMesh_TaskID )) != NULL )
        {
            bleMesh_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & BLEMESH_LIGHT_PRCESS_EVT)
    {
        light_blink_porcess_evt();
        return (events ^ BLEMESH_LIGHT_PRCESS_EVT);
    }

    if ( events & BLEMESH_START_DEVICE_EVT )
    {
        /* Register for GATT Client */
        GATT_InitClient();
        GATT_RegisterForInd(bleMesh_TaskID);
        light_init(led_pins,3);
        light_blink_evt_cfg(bleMesh_TaskID,BLEMESH_LIGHT_PRCESS_EVT);
        LIGHT_ONLY_RED_ON;
        CLI_init((CLI_COMMAND*)cli_cmd_list,(sizeof (cli_cmd_list)/sizeof(CLI_COMMAND)));
        appl_mesh_sample();
//          cli_demo_help(0, NULL);
        return ( events ^ BLEMESH_START_DEVICE_EVT );
    }

    if (events & BLEMESH_UART_RX_EVT)
    {
        if ('\r' == cmdstr[cmdlen - 1])
        {
            cmdstr[cmdlen - 1] = '\0';
            printf("%s", cmdstr);
            CLI_process_line
            (
                cmdstr,
                cmdlen
            );
            cmdlen = 0;
        }

        return ( events ^ BLEMESH_UART_RX_EVT );
    }

    if (events & BLEMESH_GAP_SCANENABLED)
    {
        if(BLEBRR_STATE_ADV_ENABLED != BLEBRR_GET_STATE())
            blebrr_handle_evt_scan_complete(0x01);

        return (events ^ BLEMESH_GAP_SCANENABLED);
    }

    if (events & BLEMESH_ECDH_PROCESS)
    {
        printf("Waring: CRY_ECDH_TIMESLICE not define, no ecdh time slice process\r\n");
        #ifdef CRY_ECDH_TIMESLICE
        cry_ecdh_process_secret();
        #endif
        return (events ^ BLEMESH_ECDH_PROCESS);
    }

    if (events & BLEMESH_GAP_TERMINATE)
    {
        blebrr_disconnect_pl();
        return (events ^ BLEMESH_GAP_TERMINATE);
    }

    if (events & BLEMESH_GAP_MSG_EVT)
    {
        bleMesh_GAPMsg_Timeout_Process();
        return (events ^ BLEMESH_GAP_MSG_EVT);
    }

    if (events & BLEMESH_KEY_PRESS_PRO_EVT)
    {
        for (uint8 i = 0; i < KEY_NUM; ++i)
        {
            if ((key_state.in_enable[i]==TRUE)||
                    (key_state.state[i]==STATE_KEY_RELEASE_DEBONCE))
            {
                gpio_key_timer_handler(i);
            }
        }

        return (events ^ BLEMESH_KEY_PRESS_PRO_EVT);
    }

    if (events & BLEMESH_HAL_KEY_MATRIX_EVT)
    {
        hal_pwrmgr_lock(MOD_USR1);
        #ifdef BLEBRR_LP_SUPPORT

        if(enable_sleep_flag == 1)
        {
            enable_sleep_flag = 0;
            printf("BLEMESH_HAL_KEY_MATRIX_EVT\n");
            blebrr_wakeup_handler();
            osal_start_timerEx(bleMesh_TaskID, BLEMESH_APPL_IDLE_EVT, 60*1000);
        }

        #endif
        MS_NET_ADDR pub_addr;
        bleMesh_key_process(KeyCode.key,&pub_addr);
        uint8 row_index = (KeyCode.key-1)>>2;
        uint8 col_index = (KeyCode.key-1) & 0x03;

        if(row_index == 0)
        {
            if(col_index == 0)
            {
                UI_generic_onoff_client_set(pub_addr,(g_counter1_on_off++)&0x01);
            }
            else if(col_index == 1)
            {
                UI_generic_onoff_client_set(pub_addr,(g_counter2_on_off++)&0x01);
            }
            else if(col_index == 2)
            {
                UI_generic_onoff_client_set(pub_addr,(g_counter3_on_off++)&0x01);
            }
            else if(col_index == 3)
            {
                UI_generic_onoff_client_set(pub_addr,(g_counter4_on_off++)&0x01);
            }
        }
        else if(row_index == 1)
        {
            UI_light_hsl_client_set(pub_addr,0x7fff,0x0000,0xffff);
        }
        else if(row_index == 2)
        {
            UI_light_hsl_client_set(pub_addr,0x7fff,0x5555,0xffff);
        }
        else if(row_index == 3)
        {
            UI_light_hsl_client_set(pub_addr,0x7fff,0xaaaa,0xffff);
        }

        return (events ^ BLEMESH_HAL_KEY_MATRIX_EVT);
    }

    if (events & BLEMESH_PDU_TX_OVERRUN)
    {
        if(((KeyCode.key-1) & 0x03)==0)
        {
            UI_vendor_model_set(0,bleMesh_pdu_time);
        }
        else if(((KeyCode.key-1) & 0x03)==1)
        {
            UI_vendor_model_set(8,bleMesh_pdu_time);
        }
        else if(((KeyCode.key-1) & 0x03)==2)
        {
            UI_vendor_model_set(21,bleMesh_pdu_time);
        }
        else if(((KeyCode.key-1) & 0x03)==3)
        {
            UI_vendor_model_set(29,bleMesh_pdu_time);
        }

        if(bleMesh_pdu_time == 99)
        {
            bleMesh_pdu_time = 0;
            osal_stop_timerEx(bleMesh_TaskID, BLEMESH_PDU_TX_OVERRUN);
        }

        bleMesh_pdu_time++;
        return (events ^ BLEMESH_PDU_TX_OVERRUN);
    }

    if ( events & BLEMESH_PROV_COMP_EVT )
    {
        printf("BLEMESH_PROV_COMP_EVT >>>");
        osal_start_timerEx(bleMesh_TaskID, BLEMESH_APPL_IDLE_EVT, 10000);
        return (events ^ BLEMESH_PROV_COMP_EVT);
    }

    if ( events & BLEMESH_APPL_IDLE_EVT )
    {
        printf("BLEMESH_APPL_IDLE_EVT >>>");
        #ifdef BLEBRR_LP_SUPPORT
        hal_pwrmgr_unlock(MOD_USR1);
        enable_sleep_flag = 1;
        blebrr_sleep_handler();
        printf("sleep mode:%d\n", isSleepAllow());
        #endif
        return (events ^ BLEMESH_APPL_IDLE_EVT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      bleMesh_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void bleMesh_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case GAP_MSG_EVENT:
        bleMesh_ProcessGAPMsg( (gapEventHdr_t*)pMsg );
        break;

    case L2CAP_SIGNAL_EVENT:
        bleMesh_ProcessL2CAPMsg( (gapEventHdr_t*)pMsg );
        break;

    case GATT_MSG_EVENT:
        bleMesh_ProcessGATTMsg( (gattMsgEvent_t*)pMsg );
        /* Invoke the Mesh Client GATT Msg Handler */
        mesh_client_process_gattMsg((gattMsgEvent_t*)pMsg, bleMesh_TaskID);
        break;

    default:
        break;
    }
}

static void bleMesh_ProcessGATTMsg( gattMsgEvent_t* pMsg )
{
    // Process the GATT server message
    switch ( pMsg->method )
    {
    case ATT_EXCHANGE_MTU_RSP:
        break;

    default:
        break;
    }
}

static void bleMesh_ProcessL2CAPMsg( gapEventHdr_t* pMsg )
{
    l2capSignalEvent_t* pPkt = (l2capSignalEvent_t*)pMsg;

    // Process the Parameter Update Response
    if ( pPkt->opcode == L2CAP_PARAM_UPDATE_RSP )
    {
        l2capParamUpdateRsp_t* pRsp = (l2capParamUpdateRsp_t*)&(pPkt->cmd.updateRsp);

        if (pRsp->result == L2CAP_CONN_PARAMS_ACCEPTED )
        {
            printf ("L2CAP Connection Parameter Updated!\r\n");
        }
    }
}

extern uint8 llState;
extern uint8 llSecondaryState;


static void bleMesh_ProcessGAPMsg( gapEventHdr_t* pMsg )
{
    hciStatus_t ret;
    gapDeviceInfoEvent_t* dev;

    if ((pMsg->hdr.status != SUCCESS)
            && (pMsg->hdr.status != bleGAPUserCanceled || pMsg->opcode != GAP_DEVICE_DISCOVERY_EVENT))
    {
//        printf ("GAP Event - %02X, status = %X\r\n", pMsg->opcode, pMsg->hdr.status);
    }

    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        gapDeviceInitDoneEvent_t* pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        if ( pPkt->hdr.status == SUCCESS )
        {
            // Save BD Address information in pPkt->devAddr for B_ADDR_LEN
        }
    }
    break;

    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:  //adv start data ready
        osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT);

        if (SUCCESS != pMsg->hdr.status)
        {
            ret = GAP_MakeDiscoverable(bleMesh_TaskID, &bleMesh_advparam);

            if (SUCCESS != ret)
            {
                printf ("GAP_MakeDiscoverable Failed - %d\r\n", ret);
            }
            else
            {
                osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT, 10*1000);
            }
        }
        else
        {
            blebrr_handle_evt_adv_complete(1);
        }

        break;

    case GAP_END_DISCOVERABLE_DONE_EVENT:
        //printf ("->%d\r\n", pMsg->opcode);
        //printf ("ED Status - %d", pMsg->hdr.status);
        osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT);

        if (SUCCESS != pMsg->hdr.status)
        {
            ret = GAP_EndDiscoverable(bleMesh_TaskID);

            if (SUCCESS != ret)
            {
                printf ("GAP_EndDiscoverable Failed - %d\r\n", ret);
            }
            else
            {
                osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT, 10*1000);
            }
        }
        else
        {
            blebrr_handle_evt_adv_complete(0);
        }

        break;

    case GAP_DEVICE_DISCOVERY_EVENT:    //scan start/stop
        if (TRUE == bleMesh_DiscCancel)
        {
            osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT);

            if ((bleGAPUserCanceled != pMsg->hdr.status) && (SUCCESS != pMsg->hdr.status))
            {
                printf ("GAP_DeviceDiscoveryCancel Retry - 0x%02X 0x%02X 0x%02X\r\n", pMsg->hdr.status,llState,llSecondaryState);
                GAP_DeviceDiscoveryCancel(bleMesh_TaskID);
                osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT, 10*1000);
            }
            else
            {
                bleMesh_DiscCancel = FALSE;
                blebrr_handle_evt_scan_complete(0);
            }
        }
        else
        {
            if (SUCCESS == pMsg->hdr.status)
            {
                GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);
            }
            //else if(LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE == pMsg->hdr.status)
            else if((LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE == pMsg->hdr.status) || (bleMemAllocError == pMsg->hdr.status) )
            {
                printf ("GAP_DeviceDiscoveryRequest Retry - 0x%02X\r\n", pMsg->hdr.status);
                GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);
            }
        }

        break;

    case GAP_DEVICE_INFO_EVENT:
        dev = (gapDeviceInfoEvent_t*)pMsg;
        blebrr_handle_evt_adv_report(dev);
        break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT);
        blebrr_timer_stop();
        gapEstLinkReqEvent_t* pPkt = (gapEstLinkReqEvent_t*)pMsg;
        printf("\r\n GAP_LINK_ESTABLISHED_EVENT received! \r\n");

        if ( pPkt->hdr.status == SUCCESS )
        {
            /* Send Connection Complete to  Ble Bearer PL layer */
            blebrr_handle_le_connection_pl
            (
                0x00, /* Dummy Static Connection Index */
                pPkt->connectionHandle,
                pPkt->devAddrType,
                pPkt->devAddr
            );
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t* pPkt = (gapTerminateLinkEvent_t*)pMsg;
        /* printf("\r\n GAP_LINK_TERMINATED_EVENT received! \r\n"); */
        blebrr_handle_le_disconnection_pl
        (
            0x00, /* Dummy Static Connection Index */
            pPkt->connectionHandle,
            pPkt->reason
        );
    }
    break;
    #if 0

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        gapLinkUpdateEvent_t* pPkt = (gapLinkUpdateEvent_t*)pMsg;
        l2capParamUpdateReq_t updateReq;
        uint16 timeout = GAP_GetParamValue( TGAP_CONN_PARAM_TIMEOUT );
        printf("\r\n GAP_LINK_PARAM_UPDATE_EVENT received! \r\n");
        updateReq.intervalMin = 0x28;
        updateReq.intervalMax = 0x38;
        updateReq.slaveLatency = 0;
        updateReq.timeoutMultiplier = 0x0c80;
        L2CAP_ConnParamUpdateReq( pPkt->connectionHandle, &updateReq, bleMesh_TaskID );
    }
    break;
        #endif /* 0 */

    default:
        break;
    }
}

bStatus_t BLE_gap_set_scan_params
(
    uint8_t scan_type,
    uint16_t scan_interval,
    uint16_t scan_window,
    uint8_t scan_filterpolicy
)
{
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN_WIND, scan_window );
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN_INT, scan_interval );
    GAP_SetParamValue( TGAP_FILTER_ADV_REPORTS, FALSE);
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN, 5000 );
    GAP_SetParamValue( TGAP_CONN_SCAN_INT,scan_interval );
    GAP_SetParamValue( TGAP_CONN_SCAN_WIND,scan_window);
    //GAP_SetParamValue( TGAP_LIM_DISC_SCAN, 0xFFFF );
    bleMesh_scanparam.activeScan = scan_type;
    bleMesh_scanparam.mode = DEVDISC_MODE_ALL; //DEVDISC_MODE_GENERAL;
    bleMesh_scanparam.whiteList = scan_filterpolicy;
    bleMesh_scanparam.taskID = bleMesh_TaskID;
    return 0x00;
}


bStatus_t BLE_gap_set_scan_enable
(
    uint8_t scan_enable
)
{
    bStatus_t ret;

    if (0x00 != scan_enable)
    {
        ret = GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);

        if (0 == ret)
        {
            osal_set_event (bleMesh_TaskID, BLEMESH_GAP_SCANENABLED);
        }
    }
    else
    {
        ret = GAP_DeviceDiscoveryCancel(bleMesh_TaskID);

        if (0 == ret)
        {
            bleMesh_DiscCancel = TRUE;
            osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT, 10*1000);
        }
    }

    return ret;
}

bStatus_t BLE_gap_set_adv_params
(
    uint8_t adv_type,
    uint16_t adv_intervalmin,
    uint16_t adv_intervalmax,
    uint8_t adv_filterpolicy
)
{
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, adv_intervalmin );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, adv_intervalmax );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, 0);
    bleMesh_advparam.channelMap = GAP_ADVCHAN_ALL;
    bleMesh_advparam.eventType = adv_type;
    bleMesh_advparam.filterPolicy = adv_filterpolicy;
    bleMesh_advparam.initiatorAddrType = 0x00;
    osal_memset(bleMesh_advparam.initiatorAddr, 0x00, B_ADDR_LEN);
    return SUCCESS;
}

bStatus_t BLE_gap_set_advscanrsp_data
(
    uint8_t   type,
    uint8_t* adv_data,
    uint16_t  adv_datalen
)
{
    return GAP_UpdateAdvertisingData(bleMesh_TaskID, type, adv_datalen, adv_data);
}

bStatus_t BLE_gap_connect
(
    uint8_t   whitelist,
    uint8_t* addr,
    uint8_t   addr_type
)
{
    gapEstLinkReq_t params;
    params.taskID = bleMesh_TaskID;
    params.highDutyCycle = TRUE;
    params.whiteList = whitelist;
    params.addrTypePeer = addr_type;
    VOID osal_memcpy( params.peerAddr, addr, B_ADDR_LEN );
    return GAP_EstablishLinkReq( &params );
}

bStatus_t BLE_gap_disconnect(uint16_t   conn_handle)
{
    return GAP_TerminateLinkReq( bleMesh_TaskID, conn_handle, HCI_DISCONNECT_REMOTE_USER_TERM ) ;
}

bStatus_t BLE_gap_set_adv_enable
(
    uint8_t adv_enable
)
{
    bStatus_t ret;

    if (0x00 != adv_enable)
    {
        ret = GAP_MakeDiscoverable(bleMesh_TaskID, &bleMesh_advparam);
    }
    else
    {
        ret = GAP_EndDiscoverable(bleMesh_TaskID);
    }

    if(ret == 0)
        osal_start_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT, 10*1000);

    return ret;
}

void bleMesh_GAPMsg_Timeout_Process(void)
{
    UCHAR state;
    UCHAR ret;
    state = BLEBRR_GET_STATE();

    switch (state)
    {
    case BLEBRR_STATE_IN_SCAN_ENABLE:
        break;

    case BLEBRR_STATE_IN_SCAN_DISABLE:
        ret = BLE_gap_set_scan_enable (0x00);

        if((ret == 0x12) && (TRUE == bleMesh_DiscCancel))
        {
            bleMesh_DiscCancel = FALSE;
            blebrr_handle_evt_scan_complete(0);
        }

        break;

    case BLEBRR_STATE_IN_ADV_ENABLE:
        ret = BLE_gap_set_adv_enable(0x01);

        if(ret == 0x11)
            blebrr_handle_evt_adv_complete(1);

        break;

    case BLEBRR_STATE_IN_ADV_DISABLE:
        ret = BLE_gap_set_adv_enable(0x00);

        if(ret == 0x12)
            blebrr_handle_evt_adv_complete(0);

        break;

    default :
        break;
    }
}

static void ProcessUartData(uart_Evt_t* evt)
{
    osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
    cmdlen += evt->len;
    osal_set_event( bleMesh_TaskID, BLEMESH_UART_RX_EVT );
}

void bleMesh_uart_init(void)
{
    hal_uart_deinit(UART0);
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessUartData,
    };
    hal_uart_init(cfg,UART0);//uart init
}


static MS_NET_ADDR __gkey_addr[5] = { 0xC000, 0xC001, 0xC002, 0xC003, 0xC004, };
MS_NET_ADDR bleMesh_gkey_get(uint8 indx)
{
    MS_NET_ADDR rslt;

    if (/*0 <= indx &&*/ 5 >  indx )
    {
        rslt = __gkey_addr[indx];
    }

    printf("__gkey_addr = 0x%04x\r\n", rslt);
    return ( rslt );
}

void bleMesh_gkey_set(uint8 indx, MS_NET_ADDR addr)
{
    if (/*0 <= indx &&*/ 5 >  indx )
    {
        __gkey_addr[indx] = addr;
    }

    printf("__gkey_addr = 0x%04x\r\n", __gkey_addr[indx]);
}

void bleMesh_key_process(uint8 key,MS_NET_ADDR* pub_addr)
{
    MS_NET_ADDR addr;
    addr = __gkey_addr[(key-1)&0x03];
    /*  switch((key-1)&0x03)
        {
        case 0:
          addr = 0xC000;
        break;
        case 1:
          addr = 0xC001;
        break;
        case 2:
          addr = 0xC002;
        break;
        case 3:
          addr = 0xC003;
        break;
        default:
        break;
        } */
    *pub_addr = addr;
    printf("key  %04X\n",key);
    printf("addr %04X\n",addr);
}



/*********************************************************************
*********************************************************************/
