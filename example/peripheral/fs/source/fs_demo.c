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
    Filename:       fs_demo.c
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
#include "fs_test.h"
#include "fs_demo.h"
#include "log.h"

static uint8 fs_TaskID;


/*********************************************************************
    @fn      AP_TIMER_Demo_Init

    @brief

    @param

    @return
*/

void fs_Init( uint8 task_id )
{
    fs_TaskID = task_id;
    #if (FS_TEST_TYPE == FS_MODULE_TEST)
    osal_start_timerEx(fs_TaskID, FS_TEST_EVT,1000);
    #elif (FS_TEST_TYPE == FS_EXAMPLE)
    osal_start_timerEx(fs_TaskID, FS_EXAMPLE_EVT,1000);
    #elif (FS_TEST_TYPE == FS_TIMING_TEST)
    osal_start_timerEx(fs_TaskID, FS_TIMING_EVT,1000);
    #elif (FS_TEST_TYPE == FS_XIP_TEST)
    osal_start_timerEx(fs_TaskID, FS_XIP_EVT,1000);
    #else
#error please check your config parameter
    #endif
}
uint16 fs_ProcessEvent( uint8 task_id, uint16 events )
{
    if (events & FS_TEST_EVT)
    {
        #if (FS_TEST_TYPE == FS_MODULE_TEST)
        LOG("ftcase_write_del_test\n");
        ftcase_write_del_test();
        osal_start_timerEx(fs_TaskID, FS_EXAMPLE_EVT,10000);
        #endif
        return (events ^ FS_TEST_EVT);
    }

    if (events & FS_EXAMPLE_EVT)
    {
        #if (FS_TEST_TYPE == FS_EXAMPLE)
        LOG("fs_example\n");
        fs_example();
        osal_start_timerEx(fs_TaskID, FS_EXAMPLE_EVT,500);
        #endif
        return (events ^ FS_EXAMPLE_EVT);
    }

    if (events & FS_XIP_EVT)
    {
        #if (FS_TEST_TYPE == FS_XIP_TEST)
        LOG("fs_xip_test\n");
        fs_xip_test();
        osal_start_timerEx(fs_TaskID, FS_XIP_EVT,500);
        #endif
        return (events ^ FS_EXAMPLE_EVT);
    }

    if (events & FS_TIMING_EVT)
    {
        #if (FS_TEST_TYPE == FS_TIMING_TEST)
        LOG("fs_timing_test\n");
        fs_timing_test();
        #endif
        return (events ^ FS_TIMING_EVT);
    }

    return 0;
}

