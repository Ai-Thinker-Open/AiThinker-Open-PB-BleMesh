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
 * INCLUDES
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

#include "nvs.h"

#include "blebrr.h"

#include "cliface.h"

#include "mesh_clients.h"

#include "dongleKey.h"

#include "MS_access_api.h"
#include "MS_config_api.h"
#include "net_extern.h"
#include "ltrn_extern.h"
#include "access_extern.h"



extern void appl_mesh_sample (void);
extern void appl_dump_bytes(UCHAR * buffer, UINT16 length);
extern API_RESULT UI_trn_stop_heartbeat_publication(void);
extern void UI_set_publish_address(UINT16 addr, MS_ACCESS_MODEL_HANDLE model_handle,UINT8 config_mode);
extern API_RESULT UI_set_provision_data(MS_NET_ADDR provision_addr);


/*********************************************************************
 * MACROS
 */
// #define KEY_REFRESH_BEACON_METHOD  MS_FALSE




/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 g_buttonCounter = 0;
UINT16 bleMesh_pdu_time;
uint32_t g_count_timer=5000;
uint32_t vendor_set_index=0;
UINT8   g_remove_node;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
//extern PROV_DEVICE_S UI_lprov_device;
extern  key_contex_t key_state;

extern MS_PROV_DEV_ENTRY g_prov_dev_list[MS_MAX_DEV_KEYS];
extern UINT16  g_num_entries;

extern UCHAR blebrr_prov_started;

extern MS_ACCESS_MODEL_HANDLE   UI_config_client_model_handle;

extern PROV_DATA_S UI_prov_data;




/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
void blebrr_handle_evt_adv_complete (UINT8 enable);
void blebrr_handle_evt_adv_report (gapDeviceInfoEvent_t * adv);
void blebrr_handle_evt_scan_complete (UINT8 enable);
void bleMesh_GAPMsg_Timeout_Process(void);

API_RESULT blebrr_handle_le_connection_pl(uint16_t  conn_idx, uint16_t  conn_hndl, uint8_t   peer_addr_type, uint8_t   * peer_addr);

API_RESULT blebrr_handle_le_disconnection_pl(uint16_t  conn_idx, uint16_t  conn_hndl, uint8_t   reason);

void UI_lpn_seek_friend(void);

void UI_generic_onoff_set(UCHAR state);

UINT8 bleMesh_check_node_inline(void);


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 bleMesh_TaskID;   // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
//static uint8 scanRspData[B_MAX_ADV_LEN];

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Phyplus BLE Mesh";

static gapDevDiscReq_t bleMesh_scanparam;
static gapAdvertisingParams_t bleMesh_advparam;

UCHAR bleMesh_DiscCancel = FALSE;             // HZFï¿? not use???

API_RESULT cli_demo_reset(UINT32 argc, UCHAR *argv[]);
API_RESULT cli_demo_help(UINT32 argc, UCHAR *argv[]);
API_RESULT cli_start(UINT32 argc, UCHAR *argv[]);

// add by HZF
API_RESULT cli_internal_status(UINT32 argc, UCHAR *argv[]);

API_RESULT cli_scene_store(UINT32 argc, UCHAR *argv[]);
API_RESULT cli_scene_delete(UINT32 argc, UCHAR *argv[]);
API_RESULT cli_scene_recall(UINT32 argc, UCHAR *argv[]);

API_RESULT cli_hsl_set(UINT32 argc, UCHAR *argv[]);

API_RESULT cli_ctl_set(UINT32 argc, UCHAR *argv[]);

API_RESULT cli_group_select(UINT32 argc, UCHAR *argv[]);

API_RESULT cli_on(UINT32 argc, UCHAR *argv[]);
API_RESULT cli_off(UINT32 argc, UCHAR *argv[]);



API_RESULT cli_seek(UINT32 argc, UCHAR *argv[]);

API_RESULT cli_core_modelc_config_key_refresh_phase_set(UINT32 argc, UCHAR * argv[]);
API_RESULT cli_core_modelc_config_netkey_update(UINT32 argc, UCHAR * argv[]);




UCHAR cmdstr[64];
UCHAR cmdlen;
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
    
    /* Scene Store */
    { "store", "Scene Store", cli_scene_store },    
    
    /* Scene delete */
    { "delete", "Scene Delete", cli_scene_delete },    

    /* Scene Recall */
    { "recall", "Scene Recall", cli_scene_recall },    

    /* HSL set */
    { "hsl", "HSL set", cli_hsl_set },      
    
    /* CTL set */
    { "ctl", "CTL set", cli_ctl_set },     
    
    /* Select Group */
    { "group", "Select Mesh Group", cli_group_select },      
    

    /* Reset */
    { "reset", "Reset the device", cli_demo_reset },
    
    /* internal status */
    { "status", "internal status", cli_internal_status },

    /* Send Config Key Refresh Phase Set */
    { "keyrefreshphaseset", "Send Config Key Refresh Phase Set", cli_core_modelc_config_key_refresh_phase_set},

    /* Send Config Netkey Update */
    { "netkeyupdate", "Send Config Netkey Update", cli_core_modelc_config_netkey_update}
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void bleMesh_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void bleMesh_ProcessGAPMsg( gapEventHdr_t *pMsg );
static void bleMesh_ProcessL2CAPMsg( gapEventHdr_t *pMsg );
static void bleMesh_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void key_press_process(key_evt_t key_evt,uint8 index);
void bleMesh_uart_init(void);

void UI_set_uuid_octet (UCHAR uuid_0);
void UI_light_hsl_set(UINT16 lightness, UINT16 hue, UINT16 saturation);
extern void UI_vendor_model_set(UCHAR ack_en,UCHAR test_len,UINT16 test_index);
extern void UI_vendor_model_set_raw_addr(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      bleMesh_Init
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
void bleMesh_Init( uint8 task_id )
{
    bStatus_t ret;

    bleMesh_TaskID = task_id;

    // Register for direct HCI messages
    //HCI_GAPTaskRegister(bleMesh_TaskID);
    ret = GAP_ParamsInit (bleMesh_TaskID, (GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL));
    /* printf ("Ret - %02x\r\n", ret); */
    ret = GAP_CentDevMgrInit(0xFF);
    /* printf ("Ret - %02x\r\n", ret); */
    ret = GAP_PeriDevMgrInit();
    /* printf ("Ret - %02x\r\n", ret); */
    GAP_CentConnRegister();
    
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    
    GATTServApp_RegisterForMsg(task_id);
    
    // For DLE
    llInitFeatureSet2MPHY(TRUE);
    llInitFeatureSetDLE(TRUE);    
    
    HCI_LE_SetDefaultPhyMode(0,0,0x01,0x01);

    osal_set_event( bleMesh_TaskID, BLEMESH_START_DEVICE_EVT );
    
    hal_pwrmgr_lock(MOD_USR1);
    
    dongleKey_init(key_press_process);

    osal_set_event(bleMesh_TaskID, BLEMESH_CHECK_NODE_EVT);
    
    (void)ret;
}

/*********************************************************************
 * @fn      bleMesh_ProcessEvent
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
uint8_t raw_data_onoff;
uint16 bleMesh_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    /* printf("\r\n >> bleMesh_ProcessEvent- TaskID %d : events 0x%04X\r\n", task_id, events); */

    if ( events & SYS_EVENT_MSG )
    {
        
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( bleMesh_TaskID )) != NULL )
        {
            bleMesh_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

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


        light_init();
        light_blink_evt_cfg(bleMesh_TaskID,BLEMESH_LIGHT_PRCESS_EVT);
        LIGHT_ONLY_RED_ON;

        CLI_init();

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
                cmdlen,
                (CLI_COMMAND *) cli_cmd_list,
                (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND))
            );

            cmdlen = 0;
        }
        return ( events ^ BLEMESH_UART_RX_EVT );
    }

    if (events & BLEMESH_GAP_SCANENABLED)
    {
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
    if (events & BLEMESH_KEY_PRESS_EVT_LOOP)
    {
//        g_count_timer%=3000;

//        g_count_timer+=631;

        if(blebrr_prov_started == MS_FALSE)
        {
            UI_vendor_model_set_raw_addr();
            UI_vendor_model_set(1,248,bleMesh_pdu_time++);
        }
        
        if(raw_data_onoff&0x01)
        {
            osal_start_timerEx(bleMesh_TaskID, BLEMESH_KEY_PRESS_EVT_LOOP, g_count_timer);
        }

        return (events ^ BLEMESH_KEY_PRESS_EVT_LOOP);
    } 

    if (events & BLEMESH_KEY_PRESS_EVT)
    {
//            UINT16  lightness, hue, saturation;
//            
//            lightness = 0x7fff;
//            saturation = 0xffff;
//            hue = 0;
//            
//            switch (g_buttonCounter)
//            {
//                case 0:           // RED
//                    hue = 0;
//                    break;
//                case 1:           // GREEN
//                    hue = 0x5555;
//                    break;
//                case 2:           // BLUE
//                    hue = 0xAAAA;
//                    break;
//                case 3:           // OFF
//                    lightness = 0;
//                    hue = 0;
//                    saturation = 0;
//                    break;
//                default:
//                    break;                    
//            }                
        
        //UI_light_hsl_set(lightness, hue, saturation);

//            UI_generic_onoff_set(0x01&g_buttonCounter);


//        osal_start_timerEx(bleMesh_TaskID, BLEMESH_KEY_PRESS_EVT_LOOP, g_count_timer);
        osal_set_event(bleMesh_TaskID, BLEMESH_KEY_PRESS_EVT_LOOP);
        raw_data_onoff++;

        
        

//        UI_vendor_model_set(248,bleMesh_pdu_time);
//            if(bleMesh_pdu_time == 99)
//            {
//                bleMesh_pdu_time = 0;
//                osal_stop_timerEx(bleMesh_TaskID, BLEMESH_PDU_TX_OVERRUN);
//            }
//            bleMesh_pdu_time++;
        
//            UI_vendor_model_set(0x03&g_buttonCounter);
        
        return (events ^ BLEMESH_KEY_PRESS_EVT);
    }   

    if (events & BLEMESH_CHECK_NODE_EVT)
    {             
        UINT16  pointer_entries;       
#if (CFG_HEARTBEAT_MODE) 
        API_RESULT retval; 
        UINT16  num;
        UINT8 remove_node_flag = 0;
        MS_NET_ADDR key_refresh_whitelist[MS_MAX_DEV_KEYS];
#endif         
        
        
#if (CFG_HEARTBEAT_MODE)        
        if((blebrr_prov_started == MS_FALSE)&&(MS_key_refresh_active == MS_FALSE))
        {
            remove_node_flag = bleMesh_check_node_inline();
        }        
        
        /* Update the next device address if provisioned devices are present in database */
        retval = MS_access_cm_get_prov_devices_list
                (
                    &g_prov_dev_list[0],
                    &g_num_entries,
                    &pointer_entries
                );
#else
        MS_access_cm_get_prov_devices_list
        (
            &g_prov_dev_list[0],
            &g_num_entries,
            &pointer_entries
        );
#endif        
        printf(
            "[CM] Max List size: 0x%04X\n", g_num_entries);

#if (CFG_HEARTBEAT_MODE)
        if (API_SUCCESS == retval)
        {
            for(num = 0; num < g_num_entries;num ++)
            {
                g_prov_dev_list[num].rcv_flag = 0;
            }
            if(remove_node_flag)
            {
                printf("Key refresh start\n");
                g_remove_node = 0;
                UI_prov_data.netkey[0]++;
                if(g_num_entries)
                {
                    for(num = 0; num < g_num_entries;num ++)
                    {
                        key_refresh_whitelist[num] = g_prov_dev_list[num].uaddr;
                    }
                    MS_net_key_refresh(key_refresh_whitelist,num,&UI_prov_data.netkey[0]);
                }
                else
                {
                    UI_prov_data.uaddr = 0x0001;
                    //Get key refresh state
                    UI_prov_data.ivindex = ms_iv_index.iv_index;
                    UI_prov_data.flags = ((ms_iv_index.iv_update_state & 0x01) << 1);
                    retval = UI_set_provision_data(UI_prov_data.uaddr);
                }
                
            }
        }
        else
        {
            UI_trn_stop_heartbeat_publication();
        }
#endif        
#if (CFG_HEARTBEAT_MODE)
        if(g_num_entries)
        {
            osal_start_timerEx(bleMesh_TaskID, BLEMESH_CHECK_NODE_EVT, (((g_num_entries-1)>>5) + 1)*30*1000);
        }
        else
        {
#endif            
            osal_start_timerEx(bleMesh_TaskID, BLEMESH_CHECK_NODE_EVT, 30*1000);
#if (CFG_HEARTBEAT_MODE)
        }
#endif        
               
        return (events ^ BLEMESH_CHECK_NODE_EVT);
    } 
    if (events & BLEMESH_GAP_MSG_EVT)
    {
        bleMesh_GAPMsg_Timeout_Process();
        return (events ^ BLEMESH_GAP_MSG_EVT);
    }

    // Discard unknown events
    return 0;
}

void bleMesh_delete_device_from_list(MS_NET_ADDR uaddr)
{
    API_RESULT retval;
    
    MS_ACCESS_DEV_KEY_HANDLE dev_key_handle; 

    retval = MS_access_cm_get_device_key_handle
        (
            uaddr,
            &dev_key_handle
        );

    if(retval != API_SUCCESS)
    {
        printf("Error fine device key handle\n");
        return;
    }
    MS_access_cm_delete_device_key(dev_key_handle);

    net_delete_from_cache(uaddr);
    ltrn_delete_from_reassembled_cache(uaddr);
    ltrn_delete_from_replay_cache(uaddr);
}


UINT8 bleMesh_check_node_inline(void)
{
    UINT16 i;
    UINT8   delete_count = 0;
    
    for(i = 0; i < g_num_entries; i++)
    {
        if(g_prov_dev_list[i].rcv_flag == 0)
        {  
            g_remove_node = 1;
            if(delete_count++ < 8)
            {
                blebrr_scan_pl(FALSE);  //by hq
                printf("Delete index 0x%04X uaddr 0x%04X\n",i,g_prov_dev_list[i].uaddr);
                bleMesh_delete_device_from_list(g_prov_dev_list[i].uaddr);
            }
            else
            {
                printf("Delete 0x%02X addresses;If more,delete next time",delete_count-1);
                return 0;
            }
        }
    }

    return g_remove_node;
}


/*********************************************************************
 * @fn      bleMesh_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void bleMesh_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {
        case GAP_MSG_EVENT:
            bleMesh_ProcessGAPMsg( (gapEventHdr_t *)pMsg );
            break;

        case L2CAP_SIGNAL_EVENT:
            bleMesh_ProcessL2CAPMsg( (gapEventHdr_t *)pMsg );
            break;

        case GATT_MSG_EVENT:
            bleMesh_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
            /* Invoke the Mesh Client GATT Msg Handler */
            mesh_client_process_gattMsg((gattMsgEvent_t *)pMsg, bleMesh_TaskID);
            break;

        default:
            break;
    }
}

static void bleMesh_ProcessGATTMsg( gattMsgEvent_t *pMsg )
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

static void bleMesh_ProcessL2CAPMsg( gapEventHdr_t *pMsg )
{
    l2capSignalEvent_t *pPkt = (l2capSignalEvent_t *)pMsg;

    // Process the Parameter Update Response
    if ( pPkt->opcode == L2CAP_PARAM_UPDATE_RSP )
    {
        l2capParamUpdateRsp_t *pRsp = (l2capParamUpdateRsp_t *)&(pPkt->cmd.updateRsp);

        if (pRsp->result == L2CAP_CONN_PARAMS_ACCEPTED )
        {
                      printf ("L2CAP Connection Parameter Updated!\r\n");
        }
    }
}

static void bleMesh_ProcessGAPMsg( gapEventHdr_t *pMsg )
{
    hciStatus_t ret;
    gapDeviceInfoEvent_t * dev;
    
    if ((pMsg->hdr.status != SUCCESS)
        && (pMsg->hdr.status != bleGAPUserCanceled || pMsg->opcode != GAP_DEVICE_DISCOVERY_EVENT))
        printf ("GAP Event - 0x%02X, status = 0x%X\r\n", pMsg->opcode, pMsg->hdr.status); 


    switch (pMsg->opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
            {
                 gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *) pMsg;

                if ( pPkt->hdr.status == SUCCESS )
                {
                    // Save BD Address information in pPkt->devAddr for B_ADDR_LEN
                }
            }
            break;

        case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
            //printf ("->%d\r\n", pMsg->opcode);
            //printf ("MD Status - %d", pMsg->hdr.status);
            osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT);
      
            if (SUCCESS != pMsg->hdr.status)
            {
                //printf ("MDReq\r\n");
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
                //printf ("EDReq\r\n");
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

        case GAP_DEVICE_DISCOVERY_EVENT:
//            printf("\r\nIn GAP_DEVICE_DISCOVERY_EVENT...\r\n");
            if (TRUE == bleMesh_DiscCancel)
            {
                osal_stop_timerEx(bleMesh_TaskID, BLEMESH_GAP_MSG_EVT);
                if (bleGAPUserCanceled != pMsg->hdr.status)
                {
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
//                printf (">>> %d\r\n", pMsg->hdr.status);
                if (SUCCESS == pMsg->hdr.status)
                {
                    GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);
                }
                else if(LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE == pMsg->hdr.status)
                {   
                    printf ("GAP_DeviceDiscoveryRequest Retry - %02X\r\n", pMsg->hdr.status);
                    GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);
                }
            }
            break;

        case GAP_DEVICE_INFO_EVENT:
            dev = (gapDeviceInfoEvent_t *)pMsg;

            blebrr_handle_evt_adv_report(dev);
            break;

        case GAP_LINK_ESTABLISHED_EVENT:
            {
                gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;
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
             gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

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
             gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg; 
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
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN, 30000 );
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
              uint8_t * adv_data,
              uint16_t  adv_datalen
          )
{
      return GAP_UpdateAdvertisingData(bleMesh_TaskID, type, adv_datalen, adv_data);
}

bStatus_t BLE_gap_connect
          (
              uint8_t   whitelist,
              uint8_t * addr,
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

void BLE_ecdh_yield (void)
{
    osal_set_event (bleMesh_TaskID, BLEMESH_ECDH_PROCESS);
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


static void ProcessUartData(uart_Evt_t *evt)
{
    osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
    cmdlen += evt->len;

    osal_set_event( bleMesh_TaskID, BLEMESH_UART_RX_EVT );
}

void bleMesh_uart_init(void)
{
    uart_Cfg_t cfg = {
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

    hal_uart_init(cfg);//uart init
}


API_RESULT cli_start(UINT32 argc, UCHAR *argv[])
{
    int val;

    if (1 != argc)
    {
        printf("Usage: start <octet_0>\r\n");
        return API_FAILURE;
    }

    val = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);

    UI_set_uuid_octet ((UCHAR)val);

    appl_mesh_sample();
    
    return API_SUCCESS;
}


API_RESULT cli_on(UINT32 argc, UCHAR *argv[])
{
//    UI_generic_onoff_set(0x01);
    
    return API_SUCCESS;
}

API_RESULT cli_off(UINT32 argc, UCHAR *argv[])
{
//    UI_generic_onoff_set(0x00);
    
    return API_SUCCESS;
}


API_RESULT cli_seek(UINT32 argc, UCHAR *argv[])
{
//    UI_lpn_seek_friend();
    
    return API_SUCCESS;
}


API_RESULT cli_demo_reset(UINT32 argc, UCHAR *argv[])
{
    nvs_reset(NVS_BANK_PERSISTENT);
    MS_access_cm_reset();
    printf ("Done\r\n");
    
    return API_SUCCESS;
}

//void UI_scene_store(UINT16 scene_number);

//void UI_scene_delete(UINT16 scene_number);

//void UI_scene_recall(UINT16 scene_number);

//void UI_light_ctl_set(UINT16 lightness, UINT16 temperature, UINT16 deltaUv);

API_RESULT cli_scene_store(UINT32 argc, UCHAR *argv[])
{
//    int val;

//    if (1 != argc)
//    {
//        printf("Usage: store <octet_0>\r\n");
//        return API_FAILURE;
//    }

//    val = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);

    // TODO: add scene OP
//    UI_scene_store((UINT16)val);
    
    return API_SUCCESS;
}

API_RESULT cli_scene_delete(UINT32 argc, UCHAR *argv[])
{
//    int val;

//    if (1 != argc)
//    {
//        printf("Usage: delete <octet_0>\r\n");
//        return API_FAILURE;
//    }

//    val = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    
//    UI_scene_delete((UINT16)val);
    
    return API_SUCCESS;
}

API_RESULT cli_scene_recall(UINT32 argc, UCHAR *argv[])
{
//    int val;

//    if (1 != argc)
//    {
//        printf("Usage: recall <octet_0>\r\n");
//        return API_FAILURE;
//    }

//    val = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    
//    UI_scene_recall((UINT16)val);
    
    return API_SUCCESS;
}

API_RESULT cli_hsl_set(UINT32 argc, UCHAR *argv[])
{
//    UINT16  lightness, hue, saturation;
//    if (3 != argc)
//    {
//        printf("Usage: hsl <Lightness> <Hue> <Saturation>\r\n");
//        return API_FAILURE;
//    }
//    
//    lightness  = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
//    hue        = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
//    saturation = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
    
//    UI_light_hsl_set(lightness, hue, saturation);
    
    return API_SUCCESS;
}

API_RESULT cli_ctl_set(UINT32 argc, UCHAR *argv[])
{
//    UINT16  lightness, temperature, deltaUv;
//    if (3 != argc)
//    {
//        printf("Usage: ctl <Lightness> <temperature> <deltaUV>\r\n");
//        return API_FAILURE;
//    }
//    
//    lightness   = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
//    temperature = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
//    deltaUv     = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
    
//    UI_light_ctl_set(lightness, temperature, deltaUv);
    
    return API_SUCCESS;
}

API_RESULT cli_group_select(UINT32 argc, UCHAR *argv[])
{
    UINT16  group;
    if (1 != argc)
    {
        printf("Usage: group <Group Idx>\r\n");
        return API_FAILURE;
    }
    
    group  = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    
    printf("select Mesh Group %d, behavior to be implemented\r\n", group);
    
    return API_SUCCESS;
}

API_RESULT cli_demo_help(UINT32 argc, UCHAR *argv[])
{
    UINT32 index;

    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);

    printf("\r\nCLI Demo\r\n");

    /* Print all the available commands */
    for (index = 0; index < (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND)); index++)
    {
        printf("    %s: %s\n",
        cli_cmd_list[index].cmd,
        cli_cmd_list[index].desc);
    }

    return API_SUCCESS;
}


void UI_set_uuid_octet (UCHAR uuid_0)
{

}


extern uint8 llState;
extern uint8 llSecondaryState;
extern llGlobalStatistics_t g_pmCounters;
extern UCHAR blebrr_state;  
extern uint32 blebrr_advscan_timeout_count;
extern UINT32 blebrr_scanTimeOut;

API_RESULT cli_internal_status(UINT32 argc, UCHAR *argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);

    printf("\r\n===== internal status ============\r\n");
    printf("llState = %d, llSecondaryState = %d\r\n", llState, llSecondaryState);
    
    printf("conn_event cnt = %d, crc err count = %d\r\n", g_pmCounters.ll_conn_event_cnt, 
             g_pmCounters.ll_recv_crcerr_event_cnt);

	printf("blebrr_state = %d\r\n", blebrr_state);
	printf("blebrr_scanTimOut = %d\r\n", blebrr_scanTimeOut);
    printf("blebrr_advscan_timeout_count = %d\r\n", blebrr_advscan_timeout_count);
    
    osal_memory_statics();    
    
//    printf("Mac Address: %2X %2X %2X %2X %2X %2X", UI_lprov_device.uuid[8], UI_lprov_device.uuid[9],
//                                                       UI_lprov_device.uuid[10], UI_lprov_device.uuid[11],
//                                                       UI_lprov_device.uuid[12], UI_lprov_device.uuid[13]);    
    printf("\r\n");



    return API_SUCCESS;    
}


/* Send Config Key Refresh Phase Set */
API_RESULT cli_core_modelc_config_key_refresh_phase_set(UINT32 argc, UCHAR * argv[])
{
    API_RESULT retval;
    int  choice;
    ACCESS_CONFIG_KEYREFRESH_PHASE_SET_PARAM  param;

    printf
    (">> Send Config Key Refresh Phase Set\n");

    if (2 == argc)
    {
        choice = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        param.netkey_index = (UINT16)choice;
        printf("NetKeyIndex (16-bit in HEX): 0x%04X\n", param.netkey_index);

        choice = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
        param.transition = (UCHAR)choice;
        printf("Transition (8-bit in HEX): 0x%02X\n", param.transition);
    }
    else
    {
        printf("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return API_FAILURE;
    }

    /* Change Local State as well */
    MS_access_cm_set_key_refresh_phase
    (
        0, /* subnet_handle */
        &param.transition /* key_refresh_state */
    );

    param.transition = (UCHAR)choice;
    retval = MS_config_client_keyrefresh_phase_set(&param);

    printf
    ("retval = 0x%04X\n", retval);

    return retval;
}

/* Send Config Netkey Update */
API_RESULT cli_core_modelc_config_netkey_update(UINT32 argc, UCHAR * argv[])
{
    API_RESULT retval;
    int  choice;
    ACCESS_CONFIG_NETKEY_UPDATE_PARAM  param;

    printf
    (">> Send Config Netkey Update\n");

    if (2 == argc)
    {
        choice = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        param.netkey_index = (UINT16)choice;
        printf("NetKeyIndex (16-bit in HEX): 0x%04X\n", param.netkey_index);

        CLI_strtoarray
        (
            argv[1],
            CLI_strlen(argv[1]),
            &param.netkey[0],
            16
        );
    }
    else
    {
        printf("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return API_FAILURE;
    }

    /* Set Local NetKey */
    MS_access_cm_add_update_netkey
    (
        0, /* netkey_index */
        MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE, /* opcode */
        &param.netkey[0] /* net_key */
    );

    retval = MS_config_client_netkey_update(&param);

    printf
    ("retval = 0x%04X\n", retval);

    return retval;
}



static void key_press_process(key_evt_t key_evt,uint8 index)
{
//    printf("[key_press_process]\n");
    if (index == 0)
    {
        if(key_evt == TOUCH_EVT_SHORT_PRESS)
        {  
        }
        else if(key_evt == TOUCH_EVT_LONG_PRESS)
        {
        }
        else if(key_evt == TOUCH_EVT_PRESS)
        {
            g_buttonCounter ++;
            bleMesh_pdu_time = 0;
//            printf("====> key press: counter = %d", g_buttonCounter);    
            g_buttonCounter &= 0x03;
            osal_set_event(bleMesh_TaskID,BLEMESH_KEY_PRESS_EVT);           
        }
    } 
}


/*********************************************************************
*********************************************************************/
