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


#include "pwm_ctrl.h"
#include "pwm.h"
#include "OSAL.h"
#include "gpio.h"
//#include "common.h"
#include "bleSmartPeripheral.h"
#include "error.h"
#include "pwrmgr.h"

#define GPIO_GREEN    P22
#define GPIO_YELLOW   P23
#define GPIO_RED      P30

static uint16_t s_light[3];
//static uint8_t s_light_en[3];

//static void light_start_timer(void)
//{
//  osal_start_timerEx(bleSmartPeripheral_TaskID, TIMER_LIGHT_EVT, 30*1000);
//}
//static void light_stop_timer(void)
//{
//  osal_stop_timerEx(bleSmartPeripheral_TaskID, TIMER_LIGHT_EVT);
//}


static void light_reflash(uint8_t ch)
{
  switch(ch){
    case 0:
      hal_pwm_set_count_val(PWM_CH0, s_light[0], CNTTOPVALUE);
      break;
    case 1:
      hal_pwm_set_count_val(PWM_CH1, s_light[1], CNTTOPVALUE);
      break;
    case 2:
      hal_pwm_set_count_val(PWM_CH2, s_light[2], CNTTOPVALUE);
      break;
    default:
      break;
  }  
}

void pwm_timeout_handle(void)
{
  s_light[0] = 0;
  s_light[1] = 0;
  s_light[2] = 0;
  hal_pwm_close_channel(PWM_CH0);
  hal_pwm_destroy(PWM_CH0);
  hal_pwm_close_channel(PWM_CH1);
  hal_pwm_destroy(PWM_CH1);
  hal_pwm_close_channel(PWM_CH2);
  hal_pwm_destroy(PWM_CH2);
  hal_pwm_stop();
  
}

int pwm_ctrl(uint8_t ch, uint16_t value)
{
  if(ch >2 || (value > CNTTOPVALUE))
    return PPlus_ERR_INVALID_PARAM;
  switch (ch)
	  {
	case 0:
		if(value==0){
			hal_gpio_write(GPIO_GREEN, 0);
			}else{
			hal_pwm_open_channel(PWM_CH0, GPIO_GREEN);
			s_light[0] = value;
				}   
      break;
    case 1:
      if(value==0){
			hal_gpio_write(GPIO_YELLOW, 0);
			}else{
			hal_pwm_open_channel(PWM_CH1, GPIO_YELLOW);
			s_light[1] = value;
				}
      break;
    case 2:
      if(value==0){
			hal_gpio_write(GPIO_RED, 0);
			}else{
			hal_pwm_open_channel(PWM_CH2, GPIO_RED);
			s_light[2] = value;
				}
      break;
    default:
      break;
	  }
  //s_light[ch] = (uint16_t)value;

  light_reflash(ch);
	return PPlus_SUCCESS;
}

int pwm_on(uint8_t ch)
{
  if(ch >2)
    return PPlus_ERR_INVALID_PARAM;

  s_light[ch] = (uint16_t)(CNTTOPVALUE>>1);
  
  switch(ch){
    case 0:
      hal_pwm_init(PWM_CH0, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_FALLING);
      hal_pwm_set_count_val(PWM_CH0, s_light[0], CNTTOPVALUE);
      hal_pwm_open_channel(PWM_CH0, GPIO_GREEN);
      break;
    case 1:
      hal_pwm_init(PWM_CH1, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_FALLING);
      hal_pwm_set_count_val(PWM_CH1, s_light[1], CNTTOPVALUE);
      hal_pwm_open_channel(PWM_CH1, GPIO_YELLOW);
      break;
    case 2:
      hal_pwm_init(PWM_CH2, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_FALLING);
      hal_pwm_set_count_val(PWM_CH2, s_light[2], CNTTOPVALUE);
      hal_pwm_open_channel(PWM_CH2, GPIO_RED);
      break;
    default:
      break;
  }
  
//  PWM_ENABLE_CH(ch);
//  hal_pwrmgr_lock(MOD_PWM);
  
  hal_pwm_start();

//  light_reflash();
	return PPlus_SUCCESS;
}

int pwm_off(uint8_t ch)
{
  if(ch >2)
    return PPlus_ERR_INVALID_PARAM;

  s_light[ch] = 0;
  
  switch(ch){
    case 0:
      s_light[0] = 0;
      hal_pwm_close_channel(PWM_CH0);
      hal_pwm_destroy(PWM_CH0);
      break;
    case 1:
      s_light[1] = 0;
      hal_pwm_close_channel(PWM_CH1);
      hal_pwm_destroy(PWM_CH1);
      break;
    case 2:
      s_light[2] = 0;
      hal_pwm_close_channel(PWM_CH2);
      hal_pwm_destroy(PWM_CH2);
      break;
    default:
      break;
  }

//  PWM_DISABLE_CH(ch);
  //check light all off
  if((s_light[0] + s_light[1] + s_light[2])==0){
    hal_pwm_stop();
  }
	return PPlus_SUCCESS;
}



int pwm_init(void)
{
  s_light[0] = 0;
  s_light[1] = 0;
  s_light[2] = 0;

  return PPlus_SUCCESS;
}

