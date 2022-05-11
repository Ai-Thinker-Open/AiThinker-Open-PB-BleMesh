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


// this file define the call proxy of ROM function, not change by user
//

/*******************************************************************************
    INCLUDES
*/
#include "jump_function.h"
#include "global_config.h"
#include "aliGenie_bleMesh.h"
#include "OSAL_Tasks.h"
#include "rf_phy_driver.h"

/*******************************************************************************
    MACROS
*/

extern void wakeupProcess0(void);

extern void debug_print0(uint32 state);
extern void output_optimus0(uint32 state);
extern void ll_read_rxfifo1(void);
extern llStatus_t LL_SetAdvControl0( uint8 advMode );

void app_wakeup_process1(void);
//void config_RTC1(uint32 time);
//void wakeup_init1(void);
//void enterSleepProcess1(uint32 time);
void rf_calibrate(void);

void UART0_IRQHandler0(void);
void LL_IRQHandler(void);
int TIM0_IRQHandler(void);
void HardFault_IRQHandler(void);

void hal_UART0_IRQHandler(void);

/*******************************************************************************
    CONSTANTS
*/
const uint32_t* const jump_table_base[256] __attribute__((section("jump_table_mem_area"))) =
{
    (const uint32_t*)0,                         // 0. write Log
    (const uint32_t*)osalInitTasks,             // 1. init entry of app
    (const uint32_t*)tasksArr,                  // 2. task list
    (const uint32_t*)& tasksCnt,                // 3. task count
    (const uint32_t*)& tasksEvents,             // 4. task events
    0, 0, 0, 0, 0,                              // 5 - 9, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 10 - 19, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 20 - 29, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0,                     // <30 - - 37>
    0, 0,
    0, 0, 0, 0, 0, 0, //40 - 45
    0, 0, 0, 0,                                 //46 - 49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 50 - 59, reserved by phyplus
    0,   // < 60 -
    0,
    0,
    0,
    0, 0, 0, 0, 0, 0,                           //  -69>, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 70 -79, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 80 - 89, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 90 - 99, reserved by phyplus
    0,         // <100 -
    0,
    (const uint32_t*)rf_phy_ini,
    0,
    0,
    0,
    0, 0, 0, 0,                       // - 109, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 110 -119, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 120 -129, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 130 -139, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 140 -149, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 150 -159, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 160 -169, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 170 -179, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 180 -189, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 190 -199, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 200 - 209, reserved by phyplus
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 210 - 219, reserved by phyplus
    (const uint32_t*)HardFault_IRQHandler, 0, 0, 0, 0, 0, (const uint32_t*)0, 0, (const uint32_t*)0, 0,       // 220 - 229
    0, 0, 0, 0, 0,  // 230 - 234
    (const uint32_t*)hal_UART0_IRQHandler,   // 235
    // 0,
    0, 0, 0, 0,     // 236 - 239
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 240 - 249
    0, 0, 0, 0, 0, 0                  // 250 - 255
};



/*******************************************************************************
    Prototypes
*/


/*******************************************************************************
    LOCAL VARIABLES
*/


/*********************************************************************
    EXTERNAL VARIABLES
*/
uint32 global_config[SOFT_PARAMETER_NUM] __attribute__((section("global_config_area")));
