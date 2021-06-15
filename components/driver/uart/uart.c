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

/*******************************************************************************
* @file		uart.c
* @brief	Contains all functions support for uart driver
* @version	0.0
* @date		19. Oct. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include <string.h>
#include "ap_cp.h"
//#include "config.h"
#include "uart.h"
#include "gpio.h"
#include "common.h"
#include "OSAL.h"
#include "clock.h"
#include "pwrmgr.h"
#include "error.h"
#include "hal_defs.h"
#include "hal_mcu.h"

#define UART_TX_BUFFER_SIZE   256






typedef struct _uart_Context{
  bool        enable;
  uint8_t     tx_state;
  uart_Tx_Buf_t tx_buf;
  uart_Cfg_t  cfg;
}uart_Ctx_t;

static uart_Ctx_t m_uartCtx = {
  .enable = FALSE,
};

static int txmit_buf_use_tx_buf(uint8_t *buf,uint16_t len)
{
  uart_Tx_Buf_t* p_txbuf = &m_uartCtx.tx_buf;
  uint8_t* p_data;
  if(len == 0 || buf == NULL)
    return PPlus_ERR_INVALID_PARAM;
  if(p_txbuf->tx_state == TX_STATE_UNINIT)
    return PPlus_ERR_NO_MEM;
  if(p_txbuf->tx_state != TX_STATE_IDLE)
    return PPlus_ERR_BUSY;
  if(p_txbuf->tx_buf_size < len)
    return PPlus_ERR_NO_MEM;
  
  memcpy(p_txbuf->tx_buf, buf, len);
  p_txbuf->tx_data_size = len;
  p_txbuf->tx_data_offset = 0;
  p_txbuf->tx_state = TX_STATE_TX;

  p_data = p_txbuf->tx_buf;
  len = p_txbuf->tx_data_size - p_txbuf->tx_data_offset;
  len = len > UART_TX_FIFO_SIZE ? UART_TX_FIFO_SIZE : len;

  AP_UART0->IER &= ~(IER_ETBEI);
  
  while(len--){
    AP_UART0->THR = p_data[p_txbuf->tx_data_offset++];
  }

	hal_pwrmgr_lock(MOD_UART);
	AP_UART0->IER |= IER_ETBEI;

	return PPlus_SUCCESS;
}

 int txmit_buf_polling(uint8_t *buf,uint16_t len)
{
  
  volatile int timeout=0;     

  HAL_WAIT_CONDITION_TIMEOUT(!(AP_UART0->USR & USR_BUSY), 100000);

  while(len--)
  {  
    HAL_WAIT_CONDITION_TIMEOUT((AP_UART0->LSR & LSR_THRE), 100000);

    AP_UART0->THR = *buf++;
    timeout=0;
  }
  //wait shift register empty
  HAL_WAIT_CONDITION_TIMEOUT((AP_UART0->LSR & LSR_TEMT), 100000);
  return PPlus_SUCCESS;;
}


static void irq_rx_handler(uint8_t flg)
{
  int i;
  uint8_t data[UART_RX_FIFO_SIZE];
  uint8_t len;

  if(m_uartCtx.cfg.use_fifo){
    len = AP_UART0->RFL;
    for(i = 0; i< len; i++){
      data[i] = (uint8_t)(AP_UART0->RBR & 0xff);
    }
  }else{
    len = 1;
    AP_UART0->LSR;  //clear interrupt
    data[0] = (uint8_t)(AP_UART0->RBR & 0xff);
  }

  if(m_uartCtx.cfg.evt_handler){
    uart_Evt_t evt;
    evt.type = flg;
    evt.data = data;
    evt.len = len;
    m_uartCtx.cfg.evt_handler(&evt);
  }
}



static void irq_tx_empty_handler(void)
{
  uart_Tx_Buf_t* p_txbuf = &m_uartCtx.tx_buf;
  uint8_t* p_data;
  uint16_t len;

  if(m_uartCtx.enable == FALSE)
    return;
  if(m_uartCtx.cfg.use_fifo == FALSE)
    return;
  if(m_uartCtx.cfg.use_tx_buf == FALSE)
    return;
  if(p_txbuf->tx_state != TX_STATE_TX)
    return;

  p_data = p_txbuf->tx_buf;
  len = p_txbuf->tx_data_size - p_txbuf->tx_data_offset;
  len = len > UART_TX_FIFO_SIZE ? UART_TX_FIFO_SIZE : len;
  if(len == 0){
    p_txbuf->tx_state = TX_STATE_IDLE;
    p_txbuf->tx_data_offset = 0;
    p_txbuf->tx_data_size = 0;
    if(m_uartCtx.cfg.evt_handler){
      uart_Evt_t evt = {
          .type = UART_EVT_TYPE_TX_COMPLETED,
          .data = NULL,
          .len = 0,
        };
      m_uartCtx.cfg.evt_handler(&evt);
    }
    //when uart tx completed, release sleeping lock
    hal_pwrmgr_unlock(MOD_UART);
    return;
  }

  while(len--){
    AP_UART0->THR = p_data[p_txbuf->tx_data_offset++];
  }  
}

static void uart_hw_config(void)
{
  uart_Cfg_t* pcfg = &(m_uartCtx.cfg);
  int pclk = clk_ap_pclk();
  uint32_t dll;
  
  //enable clk gate
  clk_gate_enable(MOD_UART);
  clk_reset(MOD_UART);
  
  if(m_uartCtx.enable == FALSE){
    hal_gpio_fmux(P9, Bit_DISABLE);   //set fmux(uart tx)
    hal_gpio_fmux(P10, Bit_DISABLE);   //set fmux(uart rx)
  }
  

  
  AP_UART0->LCR =0;
  
  //temp1=  pclk % (16* pcfg->baudrate); 
  //dll = ((16*pcfg->baudrate) > 2* temp1) ? (pclk / (16*pcfg->baudrate)): (pclk / (16*pcfg->baudrate)+1);  
  dll = ((pclk>>4)+(pcfg->baudrate>>1))/pcfg->baudrate;
  
  AP_UART0->MCR=0x0;
  AP_UART0->LCR=0x80;
  //AP_UART0->DLL=dll;   
	AP_UART0->DLM=(dll & 0xFF00) >> 8;   
	AP_UART0->DLL=(dll & 0xFF);   
	
//  AP_UART0->LCR = 0x3;  //8bit, 1 stop no parity

  if(pcfg->parity)
    AP_UART0->LCR = 0x1b;  //8bit, 1 stop even parity
  else
    AP_UART0->LCR = 0x3;  //8bit, 1 stop no parity

  //set fifo
  if(pcfg->use_fifo)
  {
    //enable tx FIFO mode(empty trigger), rx FIFO mode(1/2 trigger)
    AP_UART0->FCR= FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER;
  }
  else 
  {
    AP_UART0->FCR=0;
  }
  //enable Received Data Available Interrupt
  AP_UART0->IER = IER_ERBFI;
  
  if(pcfg->use_fifo)
    AP_UART0->IER |= IER_PTIME;
  
  if(pcfg->use_tx_buf)
    AP_UART0->IER |= IER_ETBEI;
  
  //enable uart irq
  NVIC_EnableIRQ((IRQn_Type)UART_IRQ);
  NVIC_SetPriority((IRQn_Type)UART_IRQ, IRQ_PRIO_HAL);

	hal_gpio_fmux_set(pcfg->tx_pin, UART_TX);   //set fmux(uart tx)
	hal_gpio_fmux_set(pcfg->rx_pin, UART_RX);   //set fmux(uart rx)
  
}


static int uart_hw_deinit(void)
{
    AP_UART0->LCR=0x80; 
    AP_UART0->DLM=0;   
    AP_UART0->DLL=0;   
    AP_UART0->LCR =0;  

    AP_UART0->FCR=0;       
    AP_UART0->IER = 0;
    NVIC_DisableIRQ((IRQn_Type)UART_IRQ);
    hal_gpio_fmux(m_uartCtx.cfg.tx_pin,Bit_DISABLE);
    hal_gpio_fmux(m_uartCtx.cfg.rx_pin,Bit_DISABLE);

    clk_reset(MOD_UART);
    clk_gate_disable(MOD_UART);
    
    return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_UART0_IRQHandler
 *
 * @brief       This function process for uart interrupt 
 *
 * input parameters
 *
 * @param       None.      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((used)) hal_UART0_IRQHandler(void)
{
    uint8_t IRQ_ID=GetIRQ_ID;
    switch(IRQ_ID)
    {
    case TIMEOUT_IRQ:
      irq_rx_handler(UART_EVT_TYPE_RX_DATA_TO);
      break;
    case RDA_IRQ:
      irq_rx_handler(UART_EVT_TYPE_RX_DATA);
      break;
    case THR_EMPTY:
      irq_tx_empty_handler();
      break;
    case RLS_IRQ:
      break;  
    case BUSY_IRQ:
      VOID AP_UART0->USR;
      break;		
    }
}

int hal_uart_deinit(void)
{
    uart_hw_deinit();
    memset(&m_uartCtx, 0, sizeof(m_uartCtx));
    hal_pwrmgr_unregister(MOD_UART);
    return PPlus_SUCCESS;
}

int hal_uart_init(uart_Cfg_t cfg)
{
    
    if(m_uartCtx.enable)
      return PPlus_ERR_BUSY;

    memset(&m_uartCtx, 0, sizeof(m_uartCtx));
    
//    if(cfg.hw_fwctrl || cfg.parity)
//      return PPlus_ERR_NOT_SUPPORTED;

    if(cfg.hw_fwctrl)
      return PPlus_ERR_NOT_SUPPORTED;

    m_uartCtx.cfg = cfg;
    
		uart_hw_config();

    m_uartCtx.enable = TRUE;
    
    hal_pwrmgr_register(MOD_UART, NULL, uart_hw_config);

    return PPlus_SUCCESS;

}


int hal_uart_set_tx_buf(uint8_t* buf, uint16_t size)
{
  uart_Tx_Buf_t* p_txbuf = &m_uartCtx.tx_buf;
  if(m_uartCtx.enable == FALSE)
    return PPlus_ERR_INVALID_STATE;
  
  if(m_uartCtx.cfg.use_tx_buf == FALSE)
    return PPlus_ERR_NOT_SUPPORTED;

  if(p_txbuf->tx_state != TX_STATE_UNINIT)
    return PPlus_ERR_INVALID_STATE;

  HAL_ENTER_CRITICAL_SECTION();
  p_txbuf->tx_buf = buf;
  p_txbuf->tx_buf_size = size;
  p_txbuf->tx_data_offset = 0;
  p_txbuf->tx_data_size= 0;
  p_txbuf->tx_state = TX_STATE_IDLE;
  HAL_EXIT_CRITICAL_SECTION();
	return PPlus_SUCCESS;
}


int hal_uart_get_tx_ready(void)
{
  if(m_uartCtx.cfg.use_tx_buf == FALSE)
    return PPlus_SUCCESS;

  if(m_uartCtx.tx_buf.tx_state == TX_STATE_IDLE)
    return PPlus_SUCCESS;

  return PPlus_ERR_BUSY;
}

int hal_uart_send_buff(uint8_t *buff,uint16_t len)
{
  if(m_uartCtx.cfg.use_tx_buf){
    return txmit_buf_use_tx_buf(buff,len);
  }

  return txmit_buf_polling(buff,len);

}


int hal_uart_send_byte(unsigned char data)
{

    HAL_WAIT_CONDITION_TIMEOUT((AP_UART0->LSR & LSR_THRE), 10000);

    AP_UART0->THR=data;

    HAL_WAIT_CONDITION_TIMEOUT((AP_UART0->LSR & LSR_TEMT), 10000);
    return PPlus_SUCCESS;				
}	








