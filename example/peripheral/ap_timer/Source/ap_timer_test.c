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
#include "ap_cp.h"
#include "timer.h"
#include "ap_timer.h"
#include "ll.h"
#include "ll_def.h"
#include "log.h"
#include "hal_mcu.h"
#include "jump_function.h"
#include "pwrmgr.h"
#include "clock.h"

extern void clear_timer(AP_TIM_TypeDef *TIMx);
extern int  clearTimerInt(AP_TIM_TypeDef *TIMx);

bool flag = 0;
uint8_t counter = 0;
bool flag2= 0;

void set_ap_timer_enable(AP_TIM_TypeDef *TIMx, uint8_t en)
{
	if(en == 1)
		TIMx->ControlReg |= 0x01;
	else
		TIMx->ControlReg &= ~0x01;
}

void mask_ap_timer_int(AP_TIM_TypeDef *TIMx, uint8_t en)
{
	if(en == 1)
		TIMx->ControlReg |= (1 << 2);
	else
		TIMx->ControlReg &= ~(1 << 2);
}


int get_ap_timer_int_status(AP_TIM_TypeDef *TIMx)
{
	return (TIMx->status &0x1);
}

void __attribute__((used)) AP_TIMER_IRQHandler(void)
{
	HAL_ENTER_CRITICAL_SECTION();

	if(AP_TIM1->status&0x1)   
	{	
		clearTimerInt(AP_TIM1);
		LOG("[AP_TIM1]\n");
		
		if(flag == 0)
		{
			hal_gpio_write(GPIO_P25,1);
			flag = 1;
		}
		else
		{
			hal_gpio_write(GPIO_P25,0);
			flag = 0;
		}
		if(counter < 10)
		{
			counter++;
		}
		else
		{
			counter = 0;
			
			if(flag2 == 1)
			{
				LOG("T2 en\n");
				flag2 = 0;
//				set_ap_timer(2,1000000);
//				set_ap_timer_enable(AP_TIM2,1);
				
				set_ap_timer(2,1000000);
				mask_ap_timer_int(AP_TIM2,0);
			}
			else
			{
				LOG("T2 dis\n");
				flag2 = 1;
				//set_ap_timer_enable(AP_TIM2,0);
				
				mask_ap_timer_int(AP_TIM2,1);
			}
		}
	}

	if(AP_TIM2->status&0x1)   
	{	
		clearTimerInt(AP_TIM2);
		LOG("[AP_TIM2]\n");
	}
	
	if(AP_TIM3->status&0x1)
	{		 
		clearTimerInt(AP_TIM3);
		LOG("[AP_TIM3]\n");
	}
	
	if(AP_TIM4->status & 0x1)
	{		 
		clearTimerInt(AP_TIM4);   
		LOG("[AP_TIM4]\n");
	}
	
	HAL_EXIT_CRITICAL_SECTION();
}

/////////////////////  timer1~4  ///////////////////////////////
void ap_timer_init(void)
{
	HAL_ENTER_CRITICAL_SECTION();
	NVIC_EnableIRQ((IRQn_Type)TIMER_IRQ);
	NVIC_SetPriority((IRQn_Type)TIMER_IRQ, IRQ_PRIO_HAL);
	HAL_EXIT_CRITICAL_SECTION();
	
	hal_pwrmgr_register(MOD_TIMER, NULL, NULL);
}

void ap_timer_deinit(void)
{
	HAL_ENTER_CRITICAL_SECTION();
	NVIC_DisableIRQ((IRQn_Type)TIMER_IRQ);
	HAL_EXIT_CRITICAL_SECTION();
	clk_gate_disable(MOD_TIMER);
}

static void set_ap_loadtimer(AP_TIM_TypeDef *TIMx, int time)
{
	if(time>0)
	{
		TIMx->ControlReg = 0x0;
		TIMx->ControlReg = 0x2;
		TIMx->LoadCount = 4*time;           // 4MHz system timer, * 4 to convert to 1MHz timer
		TIMx->ControlReg = 0x3;
	}
	else
	{
		TIMx->ControlReg = 0x0;
	}	
}

void set_ap_timer(uint8  timeId, int time)
{
	clk_gate_enable(MOD_TIMER);
	switch(timeId)
	{
		case 1:
				set_ap_loadtimer(AP_TIM1, time);
				break;
		case 2:
				set_ap_loadtimer(AP_TIM2, time);
				break;
		case 3:
				set_ap_loadtimer(AP_TIM3, time);
				break;
		case 4:
				set_ap_loadtimer(AP_TIM4, time);
		break;
				default:
		break;
	}
}

//timeId:1~4
void clear_ap_timer(uint8 timeId)
{
	switch(timeId)
	{
		case 1:
				clear_timer(AP_TIM1);
				break;
		case 2:
				clear_timer(AP_TIM2);
				break;
		case 3:
				clear_timer(AP_TIM3);
				break;
		case 4:
				clear_timer(AP_TIM4);
		break;
				default:
		break;
	}
}
