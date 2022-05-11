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
    Filename:       bleuart_at_dma.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "string.h"
#include "OSAL.h"
#include "bleuart_at_dma.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "dma.h"
#include "flash.h"
#include "bleuart_protocol.h"
#include "bleuart_at_cmd.h"

#define AT_UART_RX_BUF_SIZE  1024
//#define AT_UART_RX_BUF_SIZE  512

uint8_t g_data_buf[AT_UART_RX_BUF_SIZE] = {0};
AT_BUF_DIV_e g_buf_div_e = AT_BUF_DIV_256;
uint16_t g_buf_div = 256;
uint8_t g_buf_len = 244;  // MTU(247) - 3
uint8_t g_buf_num = AT_UART_RX_BUF_SIZE/256;
uint8_t g_buf_rx_idx = 0;
uint8_t g_buf_tx_idx = 0;
volatile uint32_t g_buf_status = 0;
DMA_CH_CFG_t g_dma_cfg_ch0 = {0};
AP_UART_TypeDef* cur_uart = (AP_UART_TypeDef*)AP_UART0_BASE;
uint32_t g_pkt_cnt = 0;
uint32_t g_error   = 0;
// used by uart to ble path only. indicate whether there is data sending to APP via BLE
bool g_notify_flag = false;
// used by ble to uart path only. indicate whether there is data sending to UART via DMA
bool g_send_flag = false;

/*
    Public APIs.
*/
AT_BUF_DIV_e at_dma_get_div()
{
    return g_buf_div_e;
}

uint8_t at_dma_get_buf_len()
{
    return g_buf_len;
}

void at_dma_set_buf_len(uint8_t b_len)
{
    g_buf_len = b_len;
}

bool at_dma_get_notify_flag()
{
    return g_notify_flag;
}

void at_dma_set_notify_flag(bool n_flag)
{
    g_notify_flag = n_flag;
}

bool at_dma_get_send_flag()
{
    return g_send_flag;
}

void at_dma_set_send_flag(bool n_flag)
{
    g_send_flag = n_flag;
}

uint32_t at_get_count()
{
    #if 0
    uint8_t i=0;
    AT_LOG("dbg buf:\n"); // debug only.

    for(i=0; i<20; i++)
        AT_LOG("0x%x ",g_data_buf[i]);

    #endif
    AT_LOG("err:0x%x\n",g_error);
    return g_pkt_cnt;
}


void at_dma_set_div(AT_BUF_DIV_e div)
{
    switch(div)
    {
    case AT_BUF_DIV_256:
        g_buf_div_e = AT_BUF_DIV_256;
        g_buf_div = 256;
        g_buf_len = 244;  // MTU(247) - 3
        g_buf_num = AT_UART_RX_BUF_SIZE/256;
        break;

    case AT_BUF_DIV_128:
        g_buf_div_e = AT_BUF_DIV_128;
        g_buf_div = 128;
        g_buf_len = 128;
        g_buf_num = AT_UART_RX_BUF_SIZE/128;
        break;

    case AT_BUF_DIV_64:
        g_buf_div_e = AT_BUF_DIV_64;
        g_buf_div = 64;
        g_buf_len = 64;
        g_buf_num = AT_UART_RX_BUF_SIZE/64;
        break;

    case AT_BUF_DIV_32:
        g_buf_div_e = AT_BUF_DIV_32;
        g_buf_div = 32;
        g_buf_len = 32;
        g_buf_num = AT_UART_RX_BUF_SIZE/32;

    case AT_BUF_DIV_16:
        g_buf_div_e = AT_BUF_DIV_16;
        g_buf_div = 16;
        g_buf_len = 16;
        g_buf_num = (AT_UART_RX_BUF_SIZE/16 > 32)? 32: AT_UART_RX_BUF_SIZE/16;
        break;

    default:
        g_buf_div_e = AT_BUF_DIV_256;
        g_buf_div = 256;
        g_buf_len = 244;  // MTU(247) - 3
        g_buf_num = AT_UART_RX_BUF_SIZE/256;
        break;
    }
}

// Local callback. use in UART --> MODULE --> BLE path
void dma_rx_cb0(DMA_CH_t ch)
{
    //gpio_write(P23, 1);
    //gpio_write(P23, 0);
    g_buf_status |= BIT(g_buf_rx_idx); // set g_buf_rx_idx bit
    g_buf_rx_idx++;
    osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_DATA_RX);
}

// Local callback. use in BLE --> MODULE --> UART path
void dma_tx_cb0(DMA_CH_t ch)  // tbd
{
//  gpio_write(P23, 1);
//  gpio_write(P23, 0);
    g_buf_status &= ~BIT(g_buf_tx_idx); // clear g_buf_tx_idx bit
    g_send_flag = false; // clear flag once sending data done
    g_buf_tx_idx++;
    g_pkt_cnt++;  // count total pkt number on debug purpose
    osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_DATA_RX);
}

// use in BLE --> MODULE --> UART path
void at_dma_rx_init()
{
    HAL_DMA_t ch_cfg;
    hal_dma_init();
    ch_cfg.dma_channel = DMA_CH_0;
    ch_cfg.evt_handler = dma_rx_cb0;
    hal_dma_init_channel(ch_cfg);
    g_buf_rx_idx  = 0;
    g_buf_tx_idx  = 0;
    g_buf_status  = 0;
    g_notify_flag = false;
    g_pkt_cnt     = 0; //debug
    g_error       = 0; //debug
    memset(g_data_buf, 0, AT_UART_RX_BUF_SIZE);

    // get uart idx
    if(UART0 == get_uart_idx())
        cur_uart = (AP_UART_TypeDef*)AP_UART0_BASE;
    else
        cur_uart = (AP_UART_TypeDef*)AP_UART1_BASE;

    // Initialize the channel configuration array
    g_dma_cfg_ch0.transf_size = 0x0; // Update by start later
    g_dma_cfg_ch0.sinc = DMA_INC_NCHG;
    g_dma_cfg_ch0.src_tr_width = DMA_WIDTH_BYTE;
    g_dma_cfg_ch0.src_msize = DMA_BSIZE_1; // DMA_BSIZE_32
    //g_dma_cfg_ch0.src_addr = cur_uart->RBR;
    g_dma_cfg_ch0.src_addr = (uint32_t)0x40004000;
    g_dma_cfg_ch0.dinc = DMA_INC_INC;
    g_dma_cfg_ch0.dst_tr_width = DMA_WIDTH_BYTE;
    g_dma_cfg_ch0.dst_msize = DMA_BSIZE_1; // DMA_BSIZE_32
    g_dma_cfg_ch0.dst_addr = NULL; // Update by APP later
    g_dma_cfg_ch0.enable_int = true;
}

// use in UART --> MODULE --> BLE path
void at_dma_tx_init()
{
    HAL_DMA_t ch_cfg;
    hal_dma_init();
    ch_cfg.dma_channel = DMA_CH_0;
    ch_cfg.evt_handler = dma_tx_cb0;
    hal_dma_init_channel(ch_cfg);
    g_buf_rx_idx = 0;
    g_buf_tx_idx = 0;
    g_buf_status = 0;
    g_send_flag  = false;
    g_pkt_cnt    = 0; //debug
    g_error      = 0; //debug
    memset(g_data_buf, 0, AT_UART_RX_BUF_SIZE);

    // get uart idx
    if(UART0 == get_uart_idx())
        cur_uart = (AP_UART_TypeDef*)AP_UART0_BASE;
    else
        cur_uart = (AP_UART_TypeDef*)AP_UART1_BASE;

    // Initialize the channel configuration array
    g_dma_cfg_ch0.transf_size = 0x0; // Update by start later
    g_dma_cfg_ch0.sinc = DMA_INC_INC;
    g_dma_cfg_ch0.src_tr_width = DMA_WIDTH_BYTE;
    g_dma_cfg_ch0.src_msize = DMA_BSIZE_1; // DMA_BSIZE_32
    //g_dma_cfg_ch0.src_addr = cur_uart->RBR;
    g_dma_cfg_ch0.src_addr = NULL;
    g_dma_cfg_ch0.dinc = DMA_INC_NCHG; //DMA_INC_NCHG
    g_dma_cfg_ch0.dst_tr_width = DMA_WIDTH_BYTE;
    g_dma_cfg_ch0.dst_msize = DMA_BSIZE_1; // DMA_BSIZE_32
    //g_dma_cfg_ch0.dst_addr = cur_uart->RBR; // (uint32_t)0x40004000
    g_dma_cfg_ch0.dst_addr = (uint32_t)0x40004000;  //
    g_dma_cfg_ch0.enable_int = true;
}

void at_dma_deinit()
{
    hal_dma_stop_channel(DMA_CH_0);
    hal_dma_deinit();
}

uint8_t at_dma_start(uint32_t tgt_addr)
{
    uint8_t ret;
    g_dma_cfg_ch0.transf_size = g_buf_len;

    if(at_get_rxpath_flag())
    {
        g_dma_cfg_ch0.dst_addr = tgt_addr;
    }
    else
    {
        g_dma_cfg_ch0.src_addr = tgt_addr;
    }

    ret = hal_dma_config_channel(DMA_CH_0,&g_dma_cfg_ch0);

    if(ret == PPlus_SUCCESS)
    {
        hal_dma_start_channel(DMA_CH_0);
//              gpio_write(P20, 1);  // Notify_L1_S, Notify_L2_S
//              gpio_write(P20, 0);
        return PPlus_SUCCESS;
    }
    else
    {
        hal_dma_stop_channel(DMA_CH_0);
        AT_LOG("[err]ret:%d\n",ret);
        return PPlus_ERR_INTERNAL;
    }
}

void at_dma_uart_to_BLE_DMA_rx()
{
    if(g_buf_rx_idx == g_buf_num)  // round robin.
        g_buf_rx_idx = 0;

    if(g_buf_status & BIT(g_buf_rx_idx))  // The data in this buf idx is still not sent to BLE.
    {
        //g_buf_status &= ~BIT(g_buf_rx_idx); // **Note**: overwrite the forma data?? TBD.
        g_error++; // set error number
        AT_LOG("Rx Data overflow!");
    }

    at_dma_start((uint32_t)(g_data_buf + g_buf_rx_idx * g_buf_div));

    if(!g_notify_flag && g_buf_status)  // Notify is not ongoing and buf is not empty, try to notify data
    {
        g_notify_flag = true;
        at_dma_uart_to_BLE_notify_data();
    }
}

/*  The caller should set g_notify_flag = true
    to indicate the notify is ongoing
    before calling it
*/
void at_dma_uart_to_BLE_notify_data(void)
{
    bStatus_t ret;

    if(g_buf_tx_idx == g_buf_num)  // round robin.
        g_buf_tx_idx = 0;

    //HAL_ENTER_CRITICAL_SECTION();

    if(g_buf_status & BIT(g_buf_tx_idx))  // handle buf data if not NULL; else do nothing
    {
        // Notes: it would be interruptted by DMA IRQ;
        //        Remember to set g_notify_flag = false(inform APP can do notify data again) in case it would return here.
        ret = bleuart_notify_data(g_buf_len,(uint8_t*)(g_data_buf + g_buf_tx_idx * g_buf_div));

        if(SUCCESS != ret)   // re-send data in case Notify data error! Send data to APP via BLE
        {
            ret = bleuart_notify_data(g_buf_len,(uint8_t*)(g_data_buf + g_buf_tx_idx * g_buf_div));

            if(SUCCESS != ret)
            {
                //gpio_write(P18, 1);  // LED pin
                //gpio_write(P18, 0);
                g_error |= BIT(31); // set error number
                g_notify_flag = false;
                //HAL_EXIT_CRITICAL_SECTION();
                return;
            }
        }

        //gpio_write(P20, 1);  // Notify_L1_S, Notify_L2_S
        //gpio_write(P20, 0);
        HAL_ENTER_CRITICAL_SECTION();
        g_buf_status &= ~BIT(g_buf_tx_idx);
        g_buf_tx_idx++;
        g_pkt_cnt++;  // count total pkt number on debug purpose
        HAL_EXIT_CRITICAL_SECTION();
    }

    //gpio_write(P24, 1);  // Notify_L1_S, Notify_L2_S
    //gpio_write(P24, 0);
    g_notify_flag = false;  // clear flag, so it's ready to notify next pkt
    //HAL_EXIT_CRITICAL_SECTION();
}


// use in BLE --> MODULE --> UART path
void at_dma_move_ble_data(uint8_t* pdata)
{
    if(g_buf_rx_idx == g_buf_num)  // round robin.
        g_buf_rx_idx = 0;

    if(g_buf_status & BIT(g_buf_rx_idx))   // The data in this buf idx is still not sent to UART.
    {
        AT_LOG("Tx data overflow!");         // **Note**: overwrite the forma data?? TBD. No flow control for now.
    }

    memcpy((uint8_t*)(g_data_buf + g_buf_rx_idx * g_buf_div), pdata, g_buf_len);
    g_buf_status |= BIT(g_buf_rx_idx);
    g_buf_rx_idx++;
}

// use in BLE --> MODULE --> UART path
/*  The caller should set g_send_flag = true
    to indicate the notify is ongoing
    before calling it
*/
void at_dma_BLE_to_uart_DMA_tx()
{
    if(g_buf_tx_idx == g_buf_num)  // round robin.
        g_buf_tx_idx = 0;

    if(g_buf_status & BIT(g_buf_tx_idx))  // If there is data in buf, send it to UART.
    {
//      gpio_write(P24, 1);  // Notify_L1_S, Notify_L2_S
//      gpio_write(P24, 0);
        at_dma_start((uint32_t)(g_data_buf + g_buf_tx_idx * g_buf_div));
    }
    else
    {
        g_send_flag = false; // clear flag if no data done
    }
}
