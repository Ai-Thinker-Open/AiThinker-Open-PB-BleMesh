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

#ifndef __DONGLEKEY_H__
#define __DONGLEKEY_H__

#include "types.h"
#include "gpio.h"

#define KEY_NUM 1

typedef enum
{
    STATE_KEY_IDLE = 0x00,
    STATE_KEY_PRESS_DEBONCE = 0x01,
    STATE_KEY_PRESS = 0x02,
    STATE_KEY_RELEASE_DEBONCE = 0x03,
} key_state_e;

typedef enum
{
    TOUCH_EVT_PRESS   = 0x01,
    TOUCH_EVT_RELEASE,
    TOUCH_EVT_SHORT_PRESS,
    TOUCH_EVT_LONG_PRESS,
} key_evt_t;

typedef void (* key_callbank_hdl_t)(key_evt_t,uint8);

typedef struct key_state
{
    uint32_t timer_tick[KEY_NUM];
    uint8    state[KEY_NUM];
    bool     in_enable[KEY_NUM];
    key_callbank_hdl_t key_callbank;
} key_contex_t;

typedef struct key_pin
{
    GPIO_Pin_e pin[KEY_NUM];
} key_pin_t;


int dongleKey_init(key_callbank_hdl_t hdl);
void gpio_key_timer_handler(uint8 index);

#endif
