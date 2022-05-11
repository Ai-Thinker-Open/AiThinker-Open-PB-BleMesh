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
#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "bus_dev.h"

#define WDG_2S   0
#define WDG_4S   1
#define WDG_8S   2
#define WDG_16S  3
#define WDG_32S  4
#define WDG_64S  5
#define WDG_128S 6
#define WDG_256S 7

#define WDG_USE_POLLING_MODE 0//this mode is recommended
#define WDG_USE_INT_MODE     1

#define HAL_WDG_CFG_MODE        WDG_USE_POLLING_MODE
/*
    hal watchdog init function.it will be regist in wakeupinit .
    watchdog will be restored in wakeup process
*/
__ATTR_SECTION_SRAM__ void hal_watchdog_init(void);

/*
    watchdog interrupt function.
    in this function,feed watchdog and clear int flag.
*/
void hal_WATCHDOG_IRQHandler(void);


/*
    watchdog feed function.
    we also can feed watchdog in our code.
    for example,if disable all int for a long time,but we want to avoid the watchdog reset.
    in most case,it is not needed.
*/
void hal_watchdog_feed(void);

/*
    watchdog init function.it runs in polling mode.
    if use watchdog,please init it in main before system run,valid parameter 0~7.
    if not,do not init in main.
*/
int watchdog_config(uint8 cycle);

#ifdef __cplusplus
}
#endif

#endif
