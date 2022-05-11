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
    Filename:       watchdog_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "watchdog_demo.h"
#include "log.h"
#include "gpio.h"
#include "clock.h"
#include "pwrmgr.h"

static volatile uint8_t watchdog_demo_TaskID;
#define DEMO_1000MS_EVENT    0x0001
#define DEMO_1000MS_CYCLE    1000

void wdg_reg_print(void)
{
    LOG("CR:0x%x\n",AP_WDT->CR);//rw
    LOG("TORR:0x%x\n",AP_WDT->TORR);//rw
    LOG("CCVR:0x%x\n",AP_WDT->CCVR);//r: Current Counter Value Register
    LOG("STAT:0x%x\n",AP_WDT->STAT);//r
    LOG("EOI:0x%x\n",AP_WDT->EOI);//r
}

void wdg_state_print(void)
{
    LOG("\nfun:%s\n",__func__);

    if(AP_WDT->CR & 0x01)
    {
        LOG("watchdog enable\n");
    }
    else
    {
        LOG("watchdog disable\n");
    }

    LOG("watchdog clock gate:0x%x\n",AP_PCR->SW_CLK1&_CLK_WDT);

    if(AP_PCRM->CLKSEL & (1UL<<16))
    {
        LOG("watchdog clock src:XTAL 32k\n");
    }
    else
    {
        LOG("watchdog clock src:RC 32k\n");
    }
}

void Watchdog_Demo_Init( uint8 task_id )
{
    watchdog_demo_TaskID = task_id;
    LOG("watchdog demo start:\n");

    if(CFG_SLEEP_MODE == PWR_MODE_NO_SLEEP)
    {
        LOG("system no sleep\n");
    }
    else if(CFG_SLEEP_MODE == PWR_MODE_SLEEP)
    {
        LOG("system enable sleep\n");
    }

    wdg_state_print();
    osal_start_reload_timer(watchdog_demo_TaskID, DEMO_1000MS_EVENT, DEMO_1000MS_CYCLE);
}


uint16 Watchdog_Demo_ProcessEvent( uint8 task_id, uint16 events )
{
    static uint8_t counter = 0;

    if(events & DEMO_1000MS_EVENT)
    {
        counter++;

        if(counter > 1)
        {
            LOG("system will disable sleep\n");
            wdg_state_print();
            LOG("while(1);");

            while(1);;

//          HAL_ENTER_CRITICAL_SECTION();
//          while(1)
//          {
//              hal_watchdog_feed();
//          }
        }
        else
        {
            LOG("system is running:%d\n",counter);
        }

        return (events ^ DEMO_1000MS_EVENT);
    }

    return 0;
}
