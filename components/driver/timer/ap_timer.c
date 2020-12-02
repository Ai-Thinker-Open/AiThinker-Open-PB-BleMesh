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
#include "error.h"

extern void clear_timer(AP_TIM_TypeDef *TIMx);
extern int  clearTimerInt(AP_TIM_TypeDef *TIMx);

AP_TIM_TypeDef* const TimerIndex[4]= {AP_TIM1,AP_TIM2,AP_TIM3,AP_TIM4};
static ap_tm_hdl_t s_ap_callback = NULL;

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


void __attribute__((used)) AP_TIMER_IRQHandler(void)
{
  //HAL_ENTER_CRITICAL_SECTION();

  if(AP_TIM1->status&0x1)   
  { 
    clearTimerInt(AP_TIM1);
    if(s_ap_callback)
      s_ap_callback(APTM_EVT_TIMER_1);
  }

  if(AP_TIM2->status&0x1)   
  { 
    clearTimerInt(AP_TIM2);
    if(s_ap_callback)
      s_ap_callback(APTM_EVT_TIMER_2);
  }
  
  if(AP_TIM3->status&0x1)
  {    
    clearTimerInt(AP_TIM3);
    if(s_ap_callback)
      s_ap_callback(APTM_EVT_TIMER_3);
  }
  
  if(AP_TIM4->status & 0x1)
  {    
    clearTimerInt(AP_TIM4);
    if(s_ap_callback)
      s_ap_callback(APTM_EVT_TIMER_4);
  }
  
  //HAL_EXIT_CRITICAL_SECTION();
}


void ap_timer_wakeup_handler(void)
{
  clk_gate_enable(MOD_TIMER);
  //ap_timer_clear(0);
  //ap_timer_clear(1);
  //ap_timer_clear(2);
  //ap_timer_clear(3);
  NVIC_EnableIRQ((IRQn_Type)TIMER_IRQ);
  NVIC_SetPriority((IRQn_Type)TIMER_IRQ, IRQ_PRIO_HAL);

  if(s_ap_callback)
  {
    s_ap_callback(APTM_EVT_WAKEUP);
  }
}
void ap_timer_sleep_handler(void)
{
  if(s_ap_callback)
  {
    s_ap_callback(APTM_EVT_SLEEP);
  }
}

int ap_timer_int_mask(uint8_t timeId, bool en)
{
  AP_TIM_TypeDef *TIMx;
  
  if(timeId >=4)
    return PPlus_ERR_INVALID_PARAM;

  TIMx = TimerIndex[timeId];
  if(en)
      TIMx->ControlReg |= (1 << 2);
  else
    TIMx->ControlReg &= ~(1 << 2);

  return PPlus_SUCCESS;
}

int ap_timer_set(uint8_t timeId, uint32_t us)
{
  uint32_t time = us;
  switch(timeId)
  {
  case 0:
    set_ap_loadtimer(AP_TIM1, time);
    break;
  case 1:
    set_ap_loadtimer(AP_TIM2, time);
    break;
  case 2:
    set_ap_loadtimer(AP_TIM3, time);
    break;
  case 3:
    set_ap_loadtimer(AP_TIM4, time);
    break;
  default:
    return PPlus_ERR_INVALID_PARAM;
  }
  return PPlus_SUCCESS;
  
}

//timeId:1~4
int ap_timer_clear(uint8_t timeId)
{
  switch(timeId)
  {
  case 0:
    clear_timer(AP_TIM1);
    break;
  case 1:
    clear_timer(AP_TIM2);
    break;
  case 2:
    clear_timer(AP_TIM3);
    break;
  case 3:
    clear_timer(AP_TIM4);
    break;
  default:
    return PPlus_ERR_INVALID_PARAM;
  }
  return PPlus_SUCCESS;
  
}

int ap_timer_init(ap_tm_hdl_t callback)
{
  s_ap_callback = callback;

  clk_gate_enable(MOD_TIMER);

  ap_timer_clear(0);
  ap_timer_clear(1);
  ap_timer_clear(2);
  ap_timer_clear(3);
  
  HAL_ENTER_CRITICAL_SECTION();
  NVIC_EnableIRQ((IRQn_Type)TIMER_IRQ);
  NVIC_SetPriority((IRQn_Type)TIMER_IRQ, IRQ_PRIO_HAL);
  HAL_EXIT_CRITICAL_SECTION();
  
  return hal_pwrmgr_register(MOD_TIMER, ap_timer_sleep_handler, ap_timer_wakeup_handler);
  
}

int ap_timer_deinit(void)
{

  s_ap_callback = NULL;
  
  HAL_ENTER_CRITICAL_SECTION();
  NVIC_DisableIRQ((IRQn_Type)TIMER_IRQ);
  HAL_EXIT_CRITICAL_SECTION();
  clk_gate_disable(MOD_TIMER);
  
  return hal_pwrmgr_unregister(MOD_TIMER);
}




