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


#include "OSAL.h"
#include "gpio.h"
//#include "common.h"
#include "simpleBLEPeripheral.h"
#include "error.h"
#include "pwrmgr.h"
#include "led.h"
#include "common.h"

extern uint8 simpleBLEPeripheral_TaskID;

static int led_timer(uint32_t intval_ms,uint16 event_flag)
{
   osal_start_timerEx(simpleBLEPeripheral_TaskID, event_flag, intval_ms);
    return 0;
}



int led_on(GPIO_Pin_e pin,uint32 timer){
	if (pin>34)
		{
			return PPlus_ERR_INVALID_PARAM;
		}
	else
		{
			hal_gpio_write(pin, 1);
			WaitMs(timer);
			hal_gpio_write(pin, 0);
		}

	return PPlus_SUCCESS;
	}

int led_pull_light_interval_on(GPIO_Pin_e pin,uint32 on_timer,uint16 event_flag){
	if (pin>34)
		{
			return PPlus_ERR_INVALID_PARAM;
		}
	else
		{
			hal_gpio_pull_set(pin, STRONG_PULL_UP);
			led_timer(on_timer,event_flag);
		}

	return PPlus_SUCCESS;
	}

int led_pull_light_interval_off(GPIO_Pin_e pin,uint32 off_timer,uint16 event_flag){
		if (pin>34)
			{
				return PPlus_ERR_INVALID_PARAM;
			}
		else
			{
				hal_gpio_pull_set(pin, PULL_DOWN);
				led_timer(off_timer,event_flag);
			}
	
		return PPlus_SUCCESS;
	}

int led_pull_light_off(GPIO_Pin_e pin){
		if (pin>34)
			{
				return PPlus_ERR_INVALID_PARAM;
			}
		else
			{
				osal_stop_timerEx(simpleBLEPeripheral_TaskID, CONNECT_LIGHT_ON);
				osal_stop_timerEx(simpleBLEPeripheral_TaskID, CONNECT_LIGHT_OFF);
				hal_gpio_pull_set(pin, PULL_DOWN);
			}
		
		return PPlus_SUCCESS;
	}





