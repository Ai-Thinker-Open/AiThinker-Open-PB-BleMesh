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


#include <string.h>
#include "types.h"
#include "att.h"
#include "peripheral.h"
#include "uart.h"
#include "OSAL.h"
#include "bcomdef.h"
#include "pwrmgr.h"
#include "bleuart.h"
#include "error.h"
#include "log.h"
#include "bleuart_protocol.h"
#include "bleuart.h"
#include "bleuart_service.h"
#include "bleuart_at_cmd.h"
#include "bleuart_at_dma.h"

//#define UART_RX_BUF_SIZE  512
//#define UART_TX_BUF_SIZE  512
#define UART_RX_BUF_SIZE  48
#define UART_TX_BUF_SIZE  48

#define UART_RX_PREAMBLE_CODE  0xfe
#define UART_RX_PREAMBLE_SIZE  16
//#define AT_CMD_PT_LEN_MAX      20

enum
{
    BUP_RX_ST_IDLE = 0,
    BUP_RX_ST_DELAY_SLOT,
    BUP_RX_ST_SENDING
};

enum
{
    BUP_TX_ST_IDLE = 0,
    BUP_TX_ST_DELAY_SLOT,
    BUP_TX_ST_SENDING
};

typedef struct
{
    bool    conn_state;
    //uart_rx
    uint8_t rx_state;
    uint8_t rx_size;
    uint8_t rx_offset;
    uint8_t rx_buf[UART_RX_BUF_SIZE];

    //uart tx
    uint8_t tx_state;
    uint8_t tx_size;
    uint8_t tx_buf[UART_TX_BUF_SIZE];


    uint8_t hal_uart_rx_size;
    uint8_t hal_uart_rx_buf[UART_RX_BUF_SIZE];
    uint8_t hal_uart_tx_buf[UART_TX_BUF_SIZE];

} BUP_ctx_t;

BUP_ctx_t mBUP_Ctx = {0};

uint8_t ptcmd[] = "at+reset";
//uint8_t at_flag  = 0;
uint8_t g_uart_idx = UART0;
attHandleValueNoti_t notify_data= {0};


int BUP_disconnect_handler(void);


static void uartrx_timeout_timer_start(void)
{
    LOG("BLE start Timer\n");
    //osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UART_TO_TIMER, 10);
    osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UART_TO_TIMER, 8);
}

static void uartrx_timeout_timer_stop(void)
{
    osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_UART_TO_TIMER);
}

uint8_t get_uart_idx()
{
    return g_uart_idx;
}

void set_uart_idx(uint8_t idx)
{
    switch(idx)
    {
    case UART0:
        g_uart_idx = UART0;
        break;

    case UART1:
        g_uart_idx = UART1;
        break;

    default:
        g_uart_idx = UART0;
        break;
    }
}

/*  uint8_t get_rx_idle_timeout(void)
    {
    return gRxIdleTimeout;
    }

    void set_rx_idle_timeout(uint8_t idleTimeout)
    {
    gRxIdleTimeout = idleTimeout;
    }
*/
static void uartrx_idle_timer_start(void)
{
    uint32_t m_aust = at_get_auto_slp_time();
    LOG("BLE start Rx idle Timer\n");
    //osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_RX_IDLE_TIMER, m_aust * 1000);
    osal_start_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, m_aust * 1000);
}

static void uartrx_idle_timer_stop(void)
{
    //osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_RX_IDLE_TIMER);
    osal_stop_timerEx(bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);
}


__ATTR_SECTION_SRAM__ void uart_evt_hdl(uart_Evt_t* pev)
{
    BUP_ctx_t* pctx = & mBUP_Ctx;
    uint16_t conn_hdl;
    uint8_t i = 0;
    bool skip_flag = false;
    //gpio_write(P24, 1);
    //gpio_write(P24, 0);
    GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);

    if(conn_hdl == INVALID_CONNHANDLE)
    {
        BUP_disconnect_handler();
        return;
    }

    switch(pev->type)
    {
    case  UART_EVT_TYPE_RX_DATA:
    case  UART_EVT_TYPE_RX_DATA_TO:

        //          if(!mBUP_Ctx.conn_state){ // Not in connected status, handle at cmds only.
//              uartrx_timeout_timer_stop();
//        osal_memcpy((cmdstr + cmdlen), pev->data, pev->len);
//        cmdlen += pev->len;
//        osal_set_event( bleuart_TaskID, BUP_OSAL_EVT_AT_UART_RX_CMD );
//              break;
//          }
//          else{
        if((pctx->hal_uart_rx_size + pev->len)>=UART_RX_BUF_SIZE)
            break;

        // Skip preamble data.
        /*              if(at_get_pw_mode() && ((uint8_t)(UART_RX_PREAMBLE_SIZE/2) < pev->len) && (pev->len < UART_RX_PREAMBLE_SIZE) ){
                            len = (uint8_t)(UART_RX_PREAMBLE_SIZE/2);
                            // Skip first half data to avoid dis-ordered code. ?? To be verified.
                            skip_flag = true;
                            for(i=0; i<len; i++){
                                if(pev->data[pev->len - i - 1] != UART_RX_PREAMBLE_CODE){
                                    skip_flag = false;
                                    break;
                                }
                                else
                                    continue;
                            }
                        }
        */
        if(at_get_pw_mode())
        {
            uartrx_idle_timer_stop();
            uartrx_idle_timer_start();

            if((pev->len == 1) && (pctx->hal_uart_rx_size == 0))  // preamble code is the only data for this Rx data 'packet', skip it.
            {
                if(pev->data[0] == UART_RX_PREAMBLE_CODE)
                {
                    skip_flag = true;
                }
            }
            else if((1 < pev->len) && (pev->len < UART_RX_PREAMBLE_SIZE))  // skip the 'packets' ended with 2 bytes of preamble code too.
            {
                skip_flag = true;

                for(i=0; i<2; i++)
                {
                    if(pev->data[pev->len - i - 1] != UART_RX_PREAMBLE_CODE)
                    {
                        skip_flag = false;
                        break;
                    }
                    else
                        continue;
                }
            }
        }

        uartrx_timeout_timer_stop();
        uartrx_timeout_timer_start();

        // Re-start uartrx timer even if it's skipped. ?? To be verified.
        if(skip_flag)
            break;

        // Identify AT cmd in PT mode.
        /*              switch(pev->len){
                            case 1:{
                                if(pctx->hal_uart_rx_size == 0 || at_flag == 1){
                                    switch(ptcmdlen){
                                        case 0:
                                            if(pev->data[ptcmdlen] == ptcmd[ptcmdlen]){
                                                at_flag = 1;
                                                skip_flag = true;
                                                ptcmdstr[ptcmdlen] = ptcmd[ptcmdlen];
                                                ptcmdlen++;
                                            }
                                            break;
                                        case 1:
                                            if(pev->data[ptcmdlen] == ptcmd[ptcmdlen]){
                                                at_flag = 1;
                                                skip_flag = true;
                                                ptcmdstr[ptcmdlen] = ptcmd[ptcmdlen];
                                                ptcmdlen++;
                                            }
                                            else{
                                                at_flag = 0;
                                            }
                                            break;
                                        case 2:
                                            if(pev->data[ptcmdlen] == ptcmd[ptcmdlen]){
                                                at_flag = 1;
                                                skip_flag = true;
                                                ptcmdstr[ptcmdlen] = ptcmd[ptcmdlen];
                                                ptcmdlen++;
                                            }
                                            else{ // run at cmd and clear the buffer.
                                                at_flag = 0;
                                                at_at(0,NULL);
                                                ptcmdstr[0] = 0;
                                                ptcmdlen = 0;
                                            }
                                            break;
                                        case 3:
                                        case 4:
                                        case 5:
                                        case 6:
                                        case 7:
                                            if(pev->data[ptcmdlen] == ptcmd[ptcmdlen]){
                                                at_flag = 1;
                                                skip_flag = true;
                                                ptcmdstr[ptcmdlen] = ptcmd[ptcmdlen];
                                                ptcmdlen++;
                                            }
                                            else{
                                                at_flag = 0;
                                            }
                                            break;
                                        default:
                                            break;
                                    }
                                }
                                break;
                            }
                            case 2:{
                                if(pctx->hal_uart_rx_size == 0){
                                    if(pev->data[0] == 'a' && \
                                         pev->data[1] == 't'){
                                        skip_flag = true;
                                        at_at(0,NULL);
                                    }
                                }
                                break;
                            }
                            case 8:{
                                if(pctx->hal_uart_rx_size == 0){
                                    skip_flag = true;
                                    for(int i=0; i<8; i++){
                                        if(pev->data[i] != ptcmd[i]){
                                            skip_flag = false;
                                            break;
                                        }
                                        else
                                            continue;
                                    }
                                    if(skip_flag)
                                        at_reset(0,NULL);
                                }
                                break;
                            }
                            default:
                                break;
                      }

                        if(skip_flag)
                            break;

                        if(ptcmdlen != 0){ // restore the data skipped before.
                          memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, ptcmdstr, ptcmdlen);
                          pctx->hal_uart_rx_size += ptcmdlen;
                            memset(ptcmdstr, 0, ptcmdlen);
                            ptcmdlen = 0;
                            at_flag = 0;
                        }
        */
        memcpy(pctx->hal_uart_rx_buf + pctx->hal_uart_rx_size, pev->data, pev->len);
        pctx->hal_uart_rx_size += pev->len;
        break;

    case  UART_EVT_TYPE_TX_COMPLETED:
        osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_TX_COMPLETE);
        break;

    default:
        break;
    }
}

__ATTR_SECTION_SRAM__ void gpio_sleep_handle()
{
    //hal_gpio_write(P23,1);
    //hal_gpio_write(P23,0);
    hal_gpio_pin_init(FLOW_CTRL_IO_HOST_WAKEUP, GPIO_INPUT);
    hal_gpio_fmux(FLOW_CTRL_IO_HOST_WAKEUP, Bit_DISABLE);
    //hal_gpio_pin_assi_set(FLOW_CTRL_IO_HOST_WAKEUP, GPIO_INPUT);
    extern void hal_gpioin_set_flag(gpio_pin_e pin);  // in gpio.c file.
    hal_gpioin_set_flag(FLOW_CTRL_IO_HOST_WAKEUP);
    hal_gpio_wakeup_set(FLOW_CTRL_IO_HOST_WAKEUP, NEGEDGE); //  NEGEDGE
}

__ATTR_SECTION_SRAM__ void gpio_wakeup_handle()
{
    int i,flag = 0;
    uint8_t ret = 0;
    hal_gpio_pin_init(FLOW_CTRL_IO_HOST_WAKEUP, GPIO_INPUT);
    hal_gpioin_disable(FLOW_CTRL_IO_HOST_WAKEUP);

    if(hal_pwrmgr_is_lock(MOD_USR1) == FALSE)
    {
        for(i = 0; i< 200; i++)
        {
            if(hal_gpio_read(P10)==0)
            {
                //hal_gpio_write(P14,1);
                //hal_gpio_write(P14,0);
                flag = TRUE;
                break;
            }
        }

        if( flag )
        {
            for(i = 0; i< 400; i++)
            {
                if(hal_gpio_read(P10)==1)
                {
                    //hal_gpio_write(P15,1);
                    //hal_gpio_write(P15,0);
                    hal_pwrmgr_lock(MOD_USR1);
                    at_initialize_fs();  // initialize the fs
                    ret = at_snv_read_flash(); // re-read data from flash in case wakeup

                    if(SUCCESS != ret)
                    {
                        at_default(0,NULL);
                    }

                    if(get_uart_at_mod())
                    {
                        uint32_t m_auto_slp_time = at_get_auto_slp_time();
                        at_Init();
                        osal_stop_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP);
                        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_AT_AUTO_SLEEP, ((m_auto_slp_time == 0)?(20):m_auto_slp_time) * 1000 );
                    }
                    else
                    {
                        BUP_connect_handler();
                        BUP_init();
                    }

                    break;
                }
            }
        }
    }
}

int BUP_data_BLE_to_uart_completed(void)
{
    BUP_ctx_t* pctx = &mBUP_Ctx;

    if(pctx->tx_size)
    {
        BUP_data_BLE_to_uart_send();
        return PPlus_SUCCESS;
    }

    //case no data in buffer
    pctx->tx_state = BUP_TX_ST_IDLE;
    return PPlus_SUCCESS;
}

int BUP_data_BLE_to_uart_send(void)
{
    BUP_ctx_t* pctx = &mBUP_Ctx;

    if(pctx->tx_state != BUP_TX_ST_IDLE && pctx->tx_size)
    {
        // run at cmd.
        if((pctx->tx_size) == 2 && \
                (pctx->tx_buf[0] == 'a') && \
                (pctx->tx_buf[1] == 't'))
        {
            at_at(0,NULL);
            pctx->tx_state = BUP_TX_ST_IDLE;
            pctx->tx_size = 0;
            return PPlus_SUCCESS;
        }
        // run at+reset cmd.
        else if(pctx->tx_size == 8)
        {
            bool flag = true;

            for(int i=0; i<8; i++)
            {
                if(pctx->tx_buf[i] != ptcmd[i])
                {
                    flag = false;
                    break;
                }
                else
                    continue;
            }

            if(flag)
                at_reset(0,NULL);
        }

        hal_uart_send_buff((UART_INDEX_e)g_uart_idx, pctx->tx_buf, pctx->tx_size);
        pctx->tx_size = 0;
        pctx->tx_state = BUP_TX_ST_SENDING;
        return PPlus_SUCCESS;
    }

    LOG("BUP_data_BLE_to_uart_send: incorrect state\n");
    return PPlus_ERR_INVALID_STATE;
}

int BUP_data_BLE_to_uart(uint8_t* pdata, uint8_t size)
{
    //DMA_CH_t ch;
    if(at_get_dma_flag())  // use M2P DMA copy for block data. 'at+dma' cmd
    {
        if(size != at_dma_get_buf_len())  // skip the pkt with wrong size? TBD may be <end of file>
        {
//          gpio_write(P20, 1);  // Notify_L1_S, Notify_L2_S
//          gpio_write(P20, 0);
            AT_LOG("Err blk size");
            return PPlus_ERR_NOT_SUPPORTED;
        }
        else
        {
            at_dma_move_ble_data(pdata);
            osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_DATA_RX);
            return PPlus_SUCCESS;
        }
    }

    BUP_ctx_t* pctx = &mBUP_Ctx;

    //AT_LOG("t_1");
    switch(pctx->tx_state)
    {
    case BUP_TX_ST_IDLE:
    {
        if((pctx->tx_size + size)>=UART_TX_BUF_SIZE) // data overflow. 0304
        {
            break;
        }

        HAL_ENTER_CRITICAL_SECTION();
        memcpy(pctx->tx_buf + pctx->tx_size, pdata, size);
        pctx->tx_size += size;
        pctx->tx_state = BUP_TX_ST_DELAY_SLOT;
        HAL_EXIT_CRITICAL_SECTION();
        //tx_start_timer(1);  //1ms delay slot
        BUP_data_BLE_to_uart_send();
        break;
    }

    case BUP_TX_ST_DELAY_SLOT:
    case BUP_TX_ST_SENDING:
    {
        if((pctx->tx_size + size)>=UART_TX_BUF_SIZE) // data overflow. 0304
        {
//                  hal_gpio_write(P18,1);
//                  hal_gpio_write(P18,0);
            break;
        }

        HAL_ENTER_CRITICAL_SECTION();
        memcpy(pctx->tx_buf + pctx->tx_size, pdata, size);
        pctx->tx_size += size;
        HAL_EXIT_CRITICAL_SECTION();
        break;
    }

    default:
        mBUP_Ctx.tx_state = BUP_TX_ST_IDLE;
        mBUP_Ctx.tx_size = 0;
        break;
    }

    return PPlus_SUCCESS;
}

bStatus_t bleuart_notify_data(uint8_t n_size, uint8_t* n_data)
{
    if(n_size > ATT_MTU_SIZE-3)
        notify_data.len = ATT_MTU_SIZE-3;
    else
        notify_data.len = n_size;

    memcpy(notify_data.value, n_data, notify_data.len);
    return bleuart_Notify( gapConnHandle, &notify_data, bleuart_TaskID );
}

int BUP_data_uart_to_BLE_send(void)
{
    BUP_ctx_t* pctx = &mBUP_Ctx;
//  bool start_flg = FALSE;

    //AT_LOG("r_1\n");
    if(pctx->rx_state != BUP_RX_ST_IDLE && pctx->rx_size)
    {
        // run at cmd.
        if((pctx->rx_size) == 2 && \
                (pctx->rx_buf[0] == 'a') && \
                (pctx->rx_buf[1] == 't'))
        {
            at_at(0,NULL);
            pctx->rx_state = BUP_RX_ST_IDLE;
            pctx->rx_offset = 0;
            pctx->rx_size = 0;

            if(at_get_pw_mode())
            {
                uartrx_idle_timer_stop();
                uartrx_idle_timer_start();
            }

            return PPlus_SUCCESS;
        }
        // run at+reset cmd.
        else if(pctx->rx_size == 8)
        {
            bool flag = true;

            for(int i=0; i<8; i++)
            {
                if(pctx->rx_buf[i] != ptcmd[i])
                {
                    flag = false;
                    break;
                }
                else
                    continue;
            }

            if(flag)
                at_reset(0,NULL);
        }

        if(bleuart_NotifyIsReady() == FALSE)
        {
            BUP_connect_handler();
            return PPlus_ERR_BLE_NOT_READY;
        }

        if(pctx->rx_state == BUP_RX_ST_DELAY_SLOT)
        {
            //start_flg = TRUE;
            pctx->rx_state = BUP_RX_ST_SENDING;
        }

        while(1)
        {
            uint8_t size =0;
            bStatus_t ret = 0;
            attHandleValueNoti_t notify_data= {0};
            HAL_ENTER_CRITICAL_SECTION();
            size = ((pctx->rx_size - pctx->rx_offset) > 244) ? 244 : pctx->rx_size - pctx->rx_offset; //ATT_MTU_SIZE  20  192
            memcpy(notify_data.value,pctx->rx_buf + pctx->rx_offset, size);
            notify_data.len = size;
            HAL_EXIT_CRITICAL_SECTION();
            ret = bleuart_Notify(gapConnHandle, &notify_data, bleuart_TaskID);

            //AT_LOG("bleuart_Notify: %d, %d, %d\n", ret,pctx->rx_offset, pctx->rx_size);
            //printf("ret:%d\n",ret);
            if(ret == SUCCESS)
            {
                pctx->rx_offset += size;
            }
            else
            {
                if(ret == MSG_BUFFER_NOT_AVAIL)
                {
//          if(start_flg){
//                      //AT_LOG("=NY1:");
//            rx_start_timer(1);
//          }
//          else
//          {
//                      //AT_LOG("=NY2:");
//            rx_start_timer(bleuart_conn_interval()-1);
//          }
                    return PPlus_SUCCESS;
                }
                else
                {
                    return PPlus_ERR_BUSY;
                }
            }

            if(pctx->rx_offset == pctx->rx_size)
            {
                LOG("Success\n");
                pctx->rx_state = BUP_RX_ST_IDLE;
                pctx->rx_offset = 0;
                pctx->rx_size = 0;

                if(at_get_pw_mode())
                {
                    uartrx_idle_timer_stop();
                    uartrx_idle_timer_start();
                }

                return PPlus_SUCCESS;
            }
        }
    }

    LOG("BUP_data_uart_to_BLE_send: incorrect state\n");
    return PPlus_ERR_INVALID_STATE;
}

int BUP_data_uart_to_BLE(void)
{
    BUP_ctx_t* pctx = &mBUP_Ctx;

    //AT_LOG("BUP_data_uart_to_BLE\n");
    if(pctx->conn_state == FALSE)
    {
        pctx->rx_size = 0;
        pctx->rx_offset = 0;
        pctx->hal_uart_rx_size = 0;
        return PPlus_ERR_INVALID_STATE;
    }

//  if(pctx->rx_offset != 0){
//    return PPlus_ERR_BUSY;
//  }
    if((pctx->rx_size + pctx->hal_uart_rx_size) >= UART_TX_BUF_SIZE) // skip over-size data
    {
        return PPlus_ERR_DATA_SIZE;
    }

    HAL_ENTER_CRITICAL_SECTION();
    memcpy(pctx->rx_buf + pctx->rx_size, pctx->hal_uart_rx_buf, pctx->hal_uart_rx_size);
    pctx->rx_size += pctx->hal_uart_rx_size;
    pctx->hal_uart_rx_size = 0;
    HAL_EXIT_CRITICAL_SECTION();

    switch(pctx->rx_state)
    {
    case BUP_RX_ST_IDLE:
        pctx->rx_state = BUP_RX_ST_DELAY_SLOT;
        //rx_start_timer(1);  //1ms delay slot
        BUP_data_uart_to_BLE_send();
        break;

    case BUP_RX_ST_DELAY_SLOT:
    case BUP_RX_ST_SENDING:
        BUP_data_uart_to_BLE_send();
        break;

    default:
        //drop data
        LOG("BUP_data_uart_to_BLE error st\n");
        pctx->rx_size = 0;
        pctx->rx_offset = 0;
        pctx->hal_uart_rx_size = 0;
        return PPlus_ERR_INVALID_STATE;
    }

    return PPlus_SUCCESS;
}

int BUP_disconnect_handler(void)
{
    memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));

//  hal_gpio_write(FLOW_CTRL_IO_BLE_CONNECTION, 0);
    if(at_get_pw_mode())
    {
        uartrx_idle_timer_stop();
    }

    return PPlus_SUCCESS;
}

int BUP_connect_handler(void)
{
    memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));
    mBUP_Ctx.conn_state = TRUE;

//    hal_gpio_write(FLOW_CTRL_IO_BLE_CONNECTION, 0);
    if(at_get_pw_mode())
    {
        uartrx_idle_timer_stop();
        uartrx_idle_timer_start();

        if(hal_pwrmgr_is_lock(MOD_USR1) == FALSE)
        {
            hal_pwrmgr_lock(MOD_USR1);
        }
    }

    return PPlus_SUCCESS;
}

int BUP_init()
{
    BUP_ctx_t* pctx = &mBUP_Ctx;
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        //.use_fifo = FALSE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = TRUE,
        .parity     = FALSE,
        .evt_handler = uart_evt_hdl,
    };
    cfg.baudrate = at_get_baudrate();
    hal_uart_deinit((UART_INDEX_e)g_uart_idx);
    hal_uart_init(cfg, (UART_INDEX_e)g_uart_idx);
    #ifdef BLEUART_DEDICATE

    switch(at_get_uart_parity())
    {
    case (uint32_t)UART_PARITY_O:
        AP_UART0->LCR = 0x0b; //8bit, 1 stop odd parity
        break;

    case (uint32_t)UART_PARITY_E:
        AP_UART0->LCR = 0x1b; //8bit, 1 stop even parity
        break;

    default:
        break;
    }

    #endif
    hal_uart_set_tx_buf((UART_INDEX_e)g_uart_idx, pctx->hal_uart_tx_buf, UART_TX_BUF_SIZE);
    AT_LOG("PT Mod\n");
    set_uart_at_mod(false);
    return PPlus_SUCCESS;
}


