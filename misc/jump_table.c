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
  Filename:       jump_table.c
  Revised:        
  Revision:       

  Description:    Jump table that holds function pointers and veriables used in ROM code.
                  

**************************************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "jump_function.h"
#include "global_config.h"
#include "OSAL_Tasks.h"
#include "rf_phy_driver.h"
#include "pwrmgr.h"
#include "adc.h"
#include "gpio.h"
#include "uart.h"
#include "i2c_s.h"
#include "kscan.h"
#include "rflib.h"
#include "log.h"
#include "spi.h"
#include "watchdog.h"
#include "ap_timer.h"

/*******************************************************************************
 * MACROS
 */


static void hard_fault(void)
{
	unsigned int cur_sp = __current_sp();
  LOG("Hard Fault SP is %x\n",cur_sp);
	for(int i = 0; i< 0x10; i++){
	 LOG("0x%x,", ((uint32_t*)cur_sp)[i]);
	}
	while(1){
    ;
	}
}

/*******************************************************************************
 * CONSTANTS
 */
// jump table, this table save the function entry which will be called by ROM code
// item 1 - 4 for OSAL task entry
// item 224 - 255 for ISR(Interrupt Service Routine) entry
// others are reserved by ROM code
const uint32_t* const jump_table_base[256] __attribute__((section("jump_table_mem_area"))) =
{
	(const uint32_t*)0,                         // 0. write Log
	(const uint32_t*)osalInitTasks,             // 1. init entry of app
	(const uint32_t*)tasksArr,                  // 2. task list
	(const uint32_t*)&tasksCnt,                 // 3. task count
	(const uint32_t*)&tasksEvents,              // 4. task events
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
	(const uint32_t*)hal_pwrmgr_sleep_process,         // <100 -
	(const uint32_t*)hal_pwrmgr_wakeup_process,
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
    (const uint32_t*)hard_fault, 0, 0, 0, 0, 0, 0, 0,           // 220 - 227
    0, 
		(const uint32_t*)hal_KSCAN_IRQHandler,       // 228 - 229
    0, 0, 0, (const uint32_t*)AP_TIMER_IRQHandler, 0,  // 230 - 234       
    (const uint32_t*)hal_UART0_IRQHandler,      // 235 uart irq handler
		(const uint32_t*)hal_I2C0_IRQHandler,
		(const uint32_t*)hal_I2C1_IRQHandler,
		(const uint32_t*)hal_SPI0_IRQHandler, 
		(const uint32_t*)hal_SPI1_IRQHandler,     // 236 - 239
    (const uint32_t*)hal_GPIO_IRQHandler, //240 gpio interrupt handler
    0, 0, 0, 0, 0, 0, 0, 0, 0,     // 241 - 249, for ISR entry
    0, 0, 0, (const uint32_t*)hal_ADC_IRQHandler, 0, 0                  // 250 - 255, for ISR entry
};



/*******************************************************************************
 * Prototypes
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */
uint32 global_config[SOFT_PARAMETER_NUM] __attribute__((section("global_config_area")));



