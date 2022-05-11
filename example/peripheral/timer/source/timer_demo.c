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
    Filename:       timer_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "timer_demo.h"
#include "log.h"

static uint8 timer_TaskID;
static uint8_t s_testCase = 0;
static void timer_test(uint8_t testCase);

void timer_int_process(uint8_t evt)
{
    switch(evt)
    {
    case HAL_EVT_TIMER_5:
        LOG("t5\n");
        break;

    case HAL_EVT_TIMER_6:
        LOG("t6\n");
        break;

    case HAL_EVT_WAKEUP:
        LOG("wakeup\n");
        LOG("timer will disable when sleep,so if you want it work please init it when wakeup");
        break;

    case HAL_EVT_SLEEP:
        LOG("sleep\n");
        break;

    default:
        LOG("err ");
        break;
    }
}

/*********************************************************************
    @fn      AP_TIMER_Demo_Init

    @brief

    @param

    @return
*/
void timer_Init( uint8 task_id )
{
    timer_TaskID = task_id;
    LOG("when test this case,you can uncomment comment which in timer int function\n");
    osal_start_reload_timer( timer_TaskID, TIMER_1000_MS_EVT, 1000);//osal_start_timerEx
    hal_timer_init(timer_int_process);
}

static void timer_test(uint8_t testCase)
{
    switch(testCase)
    {
    case 0:
        LOG("\ninit timer5~timer6\n");
        hal_timer_init(timer_int_process);
        hal_timer_set(AP_TIMER_ID_5,1000000);
        hal_timer_set(AP_TIMER_ID_6,2000000);
        break;

    case 1:
        LOG("\nmask timer5~timer6 init\n");
        hal_timer_mask_int(AP_TIMER_ID_5,1);
        hal_timer_mask_int(AP_TIMER_ID_6,1);
        break;

    case 2:
        LOG("\nunmask timer5~timer6 init\n");
        hal_timer_mask_int(AP_TIMER_ID_5,0);
        hal_timer_mask_int(AP_TIMER_ID_6,0);
        break;

    case 5:
        LOG("\nstop timer5~timer6 int\n");
        hal_timer_stop(AP_TIMER_ID_5);
        hal_timer_stop(AP_TIMER_ID_6);
        break;

    case 6:
        LOG("\ninit timer5~timer6\n");
        hal_timer_init(timer_int_process);
        hal_timer_set(AP_TIMER_ID_5,1000000);
        hal_timer_set(AP_TIMER_ID_6,2000000);
        break;

    case 7:
        LOG("\ndeinit timer0~timer4\n");
        hal_timer_deinit();
        break;

    default:
        break;
    }
}

uint16 timer_ProcessEvent( uint8 task_id, uint16 events )
{
    static uint8_t min_count = 5;

    if (events & TIMER_1000_MS_EVT )
    {
        if(min_count == 5)
        {
            timer_test(s_testCase);
            min_count = 0;

            if(s_testCase < 7)
                s_testCase++;
            else
                s_testCase = 0;
        }

        min_count++;
        return (events ^ TIMER_1000_MS_EVT);
    }

    return 0;
}

