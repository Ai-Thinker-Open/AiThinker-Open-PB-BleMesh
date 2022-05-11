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


#ifndef _LED_LIGHT_H
#define _LED_LIGHT_H

#include "types.h"

#define LIGHT_TOP_VALUE                 256
#define LIGHT_TURN_ON                  (LIGHT_TOP_VALUE-1)
#define LIGHT_TURN_OFF                  0


#define LIGHT_BLINK_EXTRA_FAST          1
#define LIGHT_BLINK_FAST                3
#define LIGHT_BLINK_SLOW                10
#define LIGHT_BLINK_EXTRA_SLOW          30


#define LIGHT_GREEN          0
#define LIGHT_BLUE           1
#define LIGHT_RED            2

#define LIGHT_WARM           3
#define LIGHT_COLD           4

typedef struct
{
    uint8_t light;
    uint8_t curCnt;
    uint8_t tagCnt;
    uint8_t intv;
    uint16_t val0;
    uint16_t val1;
    uint8_t status;
    uint8_t task_id;
    uint16_t event_id;
} light_blink_cfg_t;

extern light_blink_cfg_t s_lightBlink;

void light_timeout_handle(void);
int light_ctrl(uint8_t ch, uint16_t value);

int light_config(uint8_t ch, uint16_t value);
int light_init(void);
void light_reflash(void);
int light_blink_evt_cfg(uint8_t task_id,uint16_t event_id);
int light_blink_set(uint8_t light,uint8 blinkIntv,uint8 blinkCnt);
void light_blink_porcess_evt(void);



#define LIGHT_ONLY_RED_ON       \
    { \
        light_config(LIGHT_RED    ,LIGHT_TURN_ON);\
        light_config(LIGHT_GREEN  ,LIGHT_TURN_OFF);\
        light_config(LIGHT_BLUE   ,LIGHT_TURN_OFF);\
        light_reflash();\
        \
    }

#define LIGHT_ONLY_GREEN_ON     \
    { \
        light_config(LIGHT_RED    ,LIGHT_TURN_OFF);\
        light_config(LIGHT_GREEN  ,LIGHT_TURN_ON);\
        light_config(LIGHT_BLUE   ,LIGHT_TURN_OFF);\
        light_reflash();\
        \
    }

#define LIGHT_ONLY_BLUE_ON      \
    { \
        light_config(LIGHT_RED    ,LIGHT_TURN_OFF);\
        light_config(LIGHT_GREEN  ,LIGHT_TURN_OFF);\
        light_config(LIGHT_BLUE   ,LIGHT_TURN_ON);\
        light_reflash();\
        \
    }

#define LIGHT_ON_OFF(r,g,b)      \
    { \
        light_config(LIGHT_RED    ,r);\
        light_config(LIGHT_GREEN  ,g);\
        light_config(LIGHT_BLUE   ,b);\
        light_reflash();\
        \
    }
#endif

