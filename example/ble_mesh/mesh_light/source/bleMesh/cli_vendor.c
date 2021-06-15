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
#include "cli_vendor.h"
#include "phymodel_server.h"

#define CONSOLE_OUT(...)    printf(__VA_ARGS__)
#define CONSOLE_IN(...)     scanf(__VA_ARGS__)


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern EM_timer_handle thandle;
extern uint8 llState;
extern uint8 llSecondaryState;
extern llGlobalStatistics_t g_pmCounters;
extern UCHAR blebrr_state;  
extern uint32 blebrr_advscan_timeout_count;
extern UINT32 blebrr_scanTimeOut;

extern llGlobalStatistics_t g_pmCounters;
extern uint32_t g_stop_scan_t1;
extern uint32_t g_stop_scan_t1_err;
extern uint8_t llModeDbg[6];

extern PROV_DEVICE_S UI_lprov_device;

extern MS_ACCESS_MODEL_HANDLE   UI_vendor_defined_server_model_handle;


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern API_RESULT UI_sample_get_net_key(void );
extern API_RESULT UI_sample_get_device_key(void);
extern API_RESULT UI_sample_check_app_key(void);

extern void timeout_cb (void * args, UINT16 size);

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static API_RESULT cli_vendormodel_send_reliable_pdu(
                /* IN */ UINT32    req_opcode,
                /* IN */ UINT16    dest_addr,
                /* IN */ UINT16    appKey_index,
                /* IN */ void    * param,
                /* IN */ UINT16    len
            )
{
    API_RESULT retval;
    
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[256];
    UCHAR    * pdu_ptr;
    UINT16     marker;

    retval = API_FAILURE;
    marker = 0;

    switch(req_opcode)
    {                      
        case MS_ACCESS_PHY_MODEL_WRITECMD_OPCODE:
        {
            EM_mem_copy(&buffer[marker], param, len);
            marker += len;
        }
        break;
        default:
        break;
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = MS_access_raw_data
            (
                &UI_vendor_defined_server_model_handle,
                req_opcode,
                dest_addr,
                appKey_index,
                pdu_ptr,
                marker,
                MS_FALSE
            );
    return retval;
}


API_RESULT cli_raw_data(UINT32 argc, UCHAR *argv[])
{
    API_RESULT retval;
    UINT16  destnation_address;
    UINT16   data_len,appKeyIndex;
    UINT8   buffer[256];
    
    if( argc < 3 )
    {
        printf("Invaild RAW DATA Paraments\n");
        return API_FAILURE;
    }

    destnation_address = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    
    appKeyIndex = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
    data_len = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);

    if(data_len == 0)
    {
        printf("No RAW DATA,Return\n");
        return API_FAILURE;
    }

    retval = CLI_strtoarray
    (
        argv[3],
        CLI_strlen(argv[3]),
        buffer,
        data_len
    );

    if(retval != API_SUCCESS)
    {
        return retval;
    }

    cli_vendormodel_send_reliable_pdu(
        MS_ACCESS_PHY_MODEL_WRITECMD_OPCODE,
        destnation_address,
        appKeyIndex,
        buffer,
        data_len                   
    );

    printf("destnation_address 0x%04X data_len 0x%02X\n",destnation_address,data_len);
    
    return API_SUCCESS;
}

API_RESULT cli_get_information(UINT32 argc, UCHAR *argv[])
{
    API_RESULT retval;
    MS_NET_ADDR addr;
    UINT8 features;
    
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);

    retval = MS_access_cm_get_primary_unicast_address(&addr);
    if(retval != API_SUCCESS)
    {
        return API_FAILURE;
    }

    MS_access_cm_get_features(&features);

    phy_printf("[ATMSH81]%04X%02X",addr,features);

    return API_SUCCESS;
}


API_RESULT cli_disp_key(UINT32 argc, UCHAR *argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);

    UI_sample_get_net_key();
    UI_sample_get_device_key();
    UI_sample_check_app_key();

    return API_SUCCESS;    
}

static void ll_dumpConnectionInfo(void )
{
    printf("========== LL PM counters ================\r\n");
    printf("ll_send_undirect_adv_cnt = %d\r\n", g_pmCounters.ll_send_undirect_adv_cnt);
    printf("ll_send_nonconn_adv_cnt = %d\r\n", g_pmCounters.ll_send_nonconn_adv_cnt);
    printf("ll_recv_scan_req_cnt = %d\r\n", g_pmCounters.ll_recv_scan_req_cnt);
    printf("ll_send_scan_rsp_cnt = %d\r\n", g_pmCounters.ll_send_scan_rsp_cnt);
    printf("ll_recv_conn_req_cnt = %d\r\n", g_pmCounters.ll_recv_conn_req_cnt);
    
    printf("ll_filter_scan_req_cnt = %d\r\n", g_pmCounters.ll_filter_scan_req_cnt);
    printf("ll_filter_conn_req_cnt = %d\r\n", g_pmCounters.ll_filter_conn_req_cnt);
    
    printf("ll_recv_adv_pkt_cnt = %d\r\n", g_pmCounters.ll_recv_adv_pkt_cnt);
    printf("ll_recv_scan_rsp_cnt = %d\r\n", g_pmCounters.ll_recv_scan_rsp_cnt);
    
    printf("ll_conn_succ_cnt = %d\r\n", g_pmCounters.ll_conn_succ_cnt);
    printf("ll_link_lost_cnt = %d\r\n", g_pmCounters.ll_link_lost_cnt);
    printf("ll_link_estab_fail_cnt = %d\r\n", g_pmCounters.ll_link_estab_fail_cnt);
    printf("ll_recv_ctrl_pkt_cnt = %d\r\n", g_pmCounters.ll_recv_ctrl_pkt_cnt);
    printf("ll_recv_data_pkt_cnt = %d\r\n", g_pmCounters.ll_recv_data_pkt_cnt);
    printf("ll_recv_invalid_pkt_cnt = %d\r\n", g_pmCounters.ll_recv_invalid_pkt_cnt);
    
    printf("ll_recv_abnormal_cnt = %d\r\n", g_pmCounters.ll_recv_abnormal_cnt);
    printf("ll_send_data_pkt_cnt = %d\r\n", g_pmCounters.ll_send_data_pkt_cnt);
    printf("ll_conn_event_cnt = %d\r\n", g_pmCounters.ll_conn_event_cnt);
    printf("ll_recv_crcerr_event_cnt = %d\r\n", g_pmCounters.ll_recv_crcerr_event_cnt);
    printf("ll_conn_event_timeout_cnt = %d\r\n", g_pmCounters.ll_conn_event_timeout_cnt);
    
    printf("ll_to_hci_pkt_cnt = %d\r\n", g_pmCounters.ll_to_hci_pkt_cnt);
    printf("ll_hci_to_ll_pkt_cnt = %d\r\n", g_pmCounters.ll_hci_to_ll_pkt_cnt);
    printf("ll_hci_buffer_alloc_err_cnt = %d\r\n", g_pmCounters.ll_hci_buffer_alloc_err_cnt);

    printf("ll_ScanStop t1 %d \r\n",g_stop_scan_t1);
    
    printf("ll_trigger_err = %d %d %d \r\n", g_pmCounters.ll_trigger_err,0x0f&g_pmCounters.ll_trigger_err,g_stop_scan_t1_err);
    printf("llModeDbg[] ");
    for(uint8 i=0;i<16;i++)
        printf("[%02d] %02x, ",i,llModeDbg[i]);

    printf("\r\n ");
    
}

API_RESULT cli_demo_reset(UINT32 argc, UCHAR *argv[])
{
    UCHAR  proxy_state,proxy;
    MS_access_cm_get_features_field(&proxy, MS_FEATURE_PROXY);
    if(MS_TRUE == proxy)
    {
        MS_proxy_fetch_state(&proxy_state);
        if(proxy_state == MS_PROXY_CONNECTED)
        {
            blebrr_disconnect_pl();
        }
        else
        {
            EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
        }
    }
    else
    {
        EM_start_timer (&thandle, 3, timeout_cb, NULL, 0);
    }
    
    nvs_reset(NVS_BANK_PERSISTENT);
    MS_access_cm_reset();
    
    printf ("Done\r\n");
    
    return API_SUCCESS;
}



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
    
    printf("Mac Address: %2X %2X %2X %2X %2X %2X", UI_lprov_device.uuid[8], UI_lprov_device.uuid[9],
                                                       UI_lprov_device.uuid[10], UI_lprov_device.uuid[11],
                                                       UI_lprov_device.uuid[12], UI_lprov_device.uuid[13]);    
    printf("\r\n");

    ll_dumpConnectionInfo();

    return API_SUCCESS;    
}

/* Send Config Heartbeat Publication Set */
API_RESULT cli_modelc_config_heartbeat_publication_set(UINT32 argc, UCHAR * argv[])
{
    API_RESULT retval;
    int  choice;
    ACCESS_CONFIG_HEARTBEATPUB_SET_PARAM  param;

    CONSOLE_OUT
    (">> Send Config Heartbeat Publication Set\n");

    if (6 == argc)
    {
        choice = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        param.destination = (UINT16)choice;
        CONSOLE_OUT("Destination (16-bit in HEX): 0x%04X\n", param.destination);

        choice = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
        param.countlog = (UCHAR)choice;
        CONSOLE_OUT("CountLog (8-bit in HEX): 0x%02X\n", param.countlog);

        choice = CLI_strtoi(argv[2], CLI_strlen(argv[2]), 16);
        param.periodlog = (UCHAR)choice;
        CONSOLE_OUT("PeriodLog (8-bit in HEX): 0x%02X\n", param.periodlog);

        choice = CLI_strtoi(argv[3], CLI_strlen(argv[3]), 16);
        param.ttl = (UCHAR)choice;
        CONSOLE_OUT("TTL (8-bit in HEX): 0x%02X\n", param.ttl);

        choice = CLI_strtoi(argv[4], CLI_strlen(argv[4]), 16);
        param.features = (UINT16)choice;
        CONSOLE_OUT("Features (16-bit in HEX): 0x%04X\n", param.features);

        choice = CLI_strtoi(argv[5], CLI_strlen(argv[5]), 16);
        param.netkey_index = (UINT16)choice;
        CONSOLE_OUT("NetKeyIndex (16-bit in HEX): 0x%04X\n", param.netkey_index);
    }
    else
    {
        CONSOLE_OUT("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return API_FAILURE;
    }

    retval = MS_config_client_heartbeat_publication_set(&param);

    CONSOLE_OUT
    ("retval = 0x%04X\n", retval);

    return retval;
}


/*********************************************************************
*********************************************************************/
