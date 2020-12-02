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
#include "watchdog.h"
#include "error.h"
#include "hal_mcu.h"
#include "pwrmgr.h"
#include "clock.h"
#include "OSAL_Timers.h"

static uint8 watchdog_TaskID; 

#define WATCHDOG_1000MS_EVENT    0x0001 
#define WATCHDOG_1000MS_CYCLE    1000 

static void hal_watchdog_feed(void)
{
	AP_WDT->CRR = 0x76;
}

static void watchdog_init(void)
{
	volatile uint32_t a;

	clk_gate_enable(MOD_WDT);

	a = AP_WDT->EOI;
	AP_WDT->CRR = 0x76;
	AP_WDT->TORR = 0x00;
	AP_WDT->CR = 0x1D;

	AP_PCR->RESET2 |= 0x4;
	NVIC_DisableIRQ((IRQn_Type)WDT_IRQ);
	
	osal_start_reload_timer(watchdog_TaskID, WATCHDOG_1000MS_EVENT, WATCHDOG_1000MS_CYCLE);
}	

static void hal_watchdog_wakeup_handler(void)
{
	watchdog_init();
}

static void hal_watchdog_sleep_handler(void)
{
	osal_stop_timerEx(watchdog_TaskID, WATCHDOG_1000MS_EVENT);
}

void Watchdog_Init(uint8 task_id)
{
	watchdog_TaskID = task_id;

	watchdog_init();
	hal_pwrmgr_register(MOD_WDT, hal_watchdog_sleep_handler, hal_watchdog_wakeup_handler);
}

uint16 Watchdog_ProcessEvent(uint8 task_id, uint16 events)
{
	if(events & WATCHDOG_1000MS_EVENT){

		hal_watchdog_feed();
	  return (events ^ WATCHDOG_1000MS_EVENT);
	}
  return 0;
}

