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
    Filename:       bleuart_at_dma.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

#ifndef __BLEUART_AT_DMA_H__
#define __BLEUART_AT_DMA_H__

#include "types.h"
#include "dma.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
    @brief DMAC Channel configuration structure type definition for bleuart_at module
*/
typedef enum
{
    AT_BUF_DIV_256=0,
    AT_BUF_DIV_128,
    AT_BUF_DIV_64,
    AT_BUF_DIV_32,
    AT_BUF_DIV_16,
    AT_BUF_DIV_MAX,
} AT_BUF_DIV_e;

/*********************************************************************
    FUNCTIONS
*/
void at_dma_rx_init(void);
void at_dma_tx_init(void);
void at_dma_deinit(void);
uint8_t at_dma_start(uint32_t dst_addr);
void at_dma_uart_to_BLE_DMA_rx(void);
void at_dma_uart_to_BLE_notify_data(void);
bool at_dma_get_notify_flag(void);
void at_dma_set_notify_flag(bool n_flag);
AT_BUF_DIV_e at_dma_get_div(void);
void at_dma_set_div(AT_BUF_DIV_e div);
uint32_t at_get_count(void);
uint8_t at_dma_get_buf_len(void);
void at_dma_set_buf_len(uint8_t b_len);
void at_dma_move_ble_data(uint8_t* pdata);
void at_dma_BLE_to_uart_DMA_tx(void);
bool at_dma_get_send_flag(void);
void at_dma_set_send_flag(bool n_flag);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __BLEUART_AT_DMA_H__ */
