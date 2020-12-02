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
 * INCLUDES
 */

#include "OSAL.h"
#include "watchdog_demo.h"
#include "log.h"
#include "gpio.h"
#include "clock.h"
#include "ap_cp.h"

static uint8_t watchdog_demo_TaskID;
#define DEMO_1000MS_EVENT    0x0001 
#define DEMO_1000MS_CYCLE    1000 

void Watchdog_Demo_Init( uint8 task_id )
{
	watchdog_demo_TaskID = task_id;

	LOG("\n-watchdog demo start-\n");
	LOG("when p14 and p15 are alwayt 0,feed watchdog,systye will not reset\n");
	LOG("when p14 is 1,watchdog timeout(2s),reset system\n");
	LOG("when p15 is 1,exception,reset system\n");
	
	hal_gpio_pin_init(GPIO_P14,IE);
	hal_gpio_pin_init(GPIO_P15,IE);
	
	osal_start_reload_timer(watchdog_demo_TaskID, DEMO_1000MS_EVENT, DEMO_1000MS_CYCLE);
}


uint16 Watchdog_Demo_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint32_t counter = 0;
	
	if(events & DEMO_1000MS_EVENT)
	{
		LOG("%d ",counter++);

		if(hal_gpio_read(GPIO_P14) == 1) 
		{
			LOG("system will reset(watchdog reset) after 2s\n");
			while(1);
		}
		
		if(hal_gpio_read(GPIO_P15) == 1) 
		{
			LOG("system will reset(hard fault,pc point an unknow zone)\n");
			((void(*)(void))0x10000)();
		}
		
		return (events ^ DEMO_1000MS_EVENT);
	}
	
  return 0;
}
