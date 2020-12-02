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
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "ap_timer_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "spi.h"
#include "error.h"
#include "spiflash.h"
#include "pwrmgr.h"
#include "ap_timer.h"

static uint8 ap_timer_TaskID; 


void timer_int_process(uint8_t evt)
{
	switch(evt)
	{
		case APTM_EVT_TIMER_1:LOG("t1 ");break;
		case APTM_EVT_TIMER_2:LOG("t2 ");break;
		case APTM_EVT_TIMER_3:LOG("t3 ");break;
		case APTM_EVT_TIMER_4:LOG("t4 ");break;
		case APTM_EVT_WAKEUP: LOG("wakeup");break;
		case APTM_EVT_SLEEP:  LOG("sleep");break;		
		
		default:LOG("err ");break;
	}
}
 
/*********************************************************************
 * @fn      AP_TIMER_Demo_Init
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 */
 void AP_TIMER_Demo_Init( uint8 task_id ){
	ap_timer_TaskID = task_id;
	 
	LOG("when test this case,you can uncomment comment which in timer int function\n");
  osal_start_reload_timer( ap_timer_TaskID, TIMER_1000_MS_EVT, 1000);//osal_start_timerEx
}
 
void ap_timer_test(uint8_t testCase)
{
	LOG("\n\ntest_case:%d",testCase);
	
	switch(testCase)
	{
		case 0:
			LOG("\ninit timer0~timer4\n");
			ap_timer_init(timer_int_process);
			ap_timer_set(0,1000000);
			ap_timer_set(1,2000000);
			ap_timer_set(2,3000000);
			ap_timer_set(3,4000000);			
			break;
		
		case 1:
			LOG("\nmask timer0~timer4 init\n");
			ap_timer_int_mask(0,1);
			ap_timer_int_mask(1,1);
			ap_timer_int_mask(2,1);
			ap_timer_int_mask(3,1);
			break;
		
		case 2:
			LOG("\nunmask timer0~timer4 init\n");
			ap_timer_int_mask(0,0);
			ap_timer_int_mask(1,0);
			ap_timer_int_mask(2,0);
			ap_timer_int_mask(3,0);
			break;
		
		case 5:
			LOG("\nclear timer0~timer4 int\n");
			ap_timer_clear(0);
			ap_timer_clear(1);
			ap_timer_clear(2);
			ap_timer_clear(3);
			break;
		
		case 6:
			LOG("\ninit timer0~timer4\n");
			ap_timer_init(timer_int_process);
			ap_timer_set(0,1000000);
			ap_timer_set(1,2000000);
			ap_timer_set(2,3000000);
			ap_timer_set(3,4000000);
			break;
				
		case 7:
			LOG("\ndeinit timer0~timer4\n");
			ap_timer_deinit();
			break;
						
		default:
			break;
	}
}

uint16 AP_TIMER_Demo_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint8_t s_testCase = 0;
	static uint8_t min_count = 12;
	
	if (events & TIMER_1000_MS_EVT )
	{
		if(min_count == 12)
		{
			ap_timer_test(s_testCase);
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
