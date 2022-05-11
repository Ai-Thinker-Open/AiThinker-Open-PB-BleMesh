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



#ifndef _BLE_UART_PROTOCOL_H
#define _BLE_UART_PROTOCOL_H

#include "types.h"
#include "bleuart.h"

typedef struct
{
    uint8_t ev;

} BUP_Evt_t;


typedef void (*BUP_CB_t)(BUP_Evt_t* pev);

int BUP_disconnect_handler(void);
int BUP_connect_handler(void);
int BUP_data_BLE_to_uart_completed(void);
int BUP_data_BLE_to_uart_send(void);
int BUP_data_BLE_to_uart(uint8_t* pdata, uint8_t size);
int BUP_data_uart_to_BLE_send(void);
int BUP_data_uart_to_BLE(void);
int BUP_init(void);
void gpio_wakeup_handle(void);
void gpio_sleep_handle(void);
uint8_t get_uart_idx(void);
void set_uart_idx(uint8_t idx);
bStatus_t bleuart_notify_data(uint8_t n_size, uint8_t* n_data);
#endif /*_BLE_UART_PROTOCOL_H*/

