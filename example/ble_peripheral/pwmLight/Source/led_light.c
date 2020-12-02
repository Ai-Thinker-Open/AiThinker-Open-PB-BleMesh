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


#include "led_light.h"
#include "pwm.h"
#include "OSAL.h"
#include "gpio.h"
#include "app_wrist.h"
#include "error.h"

#define GPIO_GREEN    P31//P23
#define GPIO_YELLOW   P23//P31
#define GPIO_RED      P32

static uint16_t s_light[3];
//static uint8_t s_light_en[3];

static void light_start_timer(void)
{
  osal_start_timerEx(AppWrist_TaskID, TIMER_LIGHT_EVT, 30*1000);
}
static void light_stop_timer(void)
{
  osal_stop_timerEx(AppWrist_TaskID, TIMER_LIGHT_EVT);
}


static void light_reflash(void)
{
  if(s_light[0] + s_light[1] + s_light[2]){
    
    hal_pwm_close_channel(PWM_CH0);
    hal_pwm_destroy(PWM_CH0);
    hal_pwm_close_channel(PWM_CH1);
    hal_pwm_destroy(PWM_CH1);
    hal_pwm_close_channel(PWM_CH1);
    hal_pwm_destroy(PWM_CH1);
    hal_pwm_stop();
    hal_gpio_pin_init(GPIO_GREEN, IE);
    hal_gpio_pin_init(GPIO_RED, IE);
    hal_gpio_pin_init(GPIO_YELLOW, IE);
    hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_YELLOW, WEAK_PULL_UP);
    
    if(s_light[0]){
      hal_pwm_init(PWM_CH0, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
      hal_pwm_set_count_val(PWM_CH0, s_light[0], 100);
      hal_pwm_open_channel(PWM_CH0, GPIO_GREEN);
    }
    
    if(s_light[1]){
      hal_pwm_init(PWM_CH1, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
      hal_pwm_set_count_val(PWM_CH1, s_light[1], 100);
      hal_pwm_open_channel(PWM_CH1, GPIO_YELLOW);
    }
    
    if(s_light[2]){
      hal_pwm_init(PWM_CH2, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
      hal_pwm_set_count_val(PWM_CH2, s_light[2], 100);
      hal_pwm_open_channel(PWM_CH2, GPIO_RED);
    }
    
    hal_pwm_start();
    light_stop_timer();
    light_start_timer();
  }
  else
  {
    hal_pwm_close_channel(PWM_CH0);
    hal_pwm_destroy(PWM_CH0);
    hal_pwm_close_channel(PWM_CH1);
    hal_pwm_destroy(PWM_CH1);
    hal_pwm_close_channel(PWM_CH1);
    hal_pwm_destroy(PWM_CH1);
    hal_pwm_stop();
    hal_gpio_pin_init(GPIO_GREEN, IE);
    hal_gpio_pin_init(GPIO_RED, IE);
    hal_gpio_pin_init(GPIO_YELLOW, IE);
    hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_YELLOW, WEAK_PULL_UP);
    light_stop_timer();
  }
}

void light_timeout_handle(void)
{
  s_light[0] = 0;
  s_light[1] = 0;
  s_light[2] = 0;
  hal_pwm_close_channel(PWM_CH0);
  hal_pwm_destroy(PWM_CH0);
  hal_pwm_close_channel(PWM_CH1);
  hal_pwm_destroy(PWM_CH1);
  hal_pwm_close_channel(PWM_CH1);
  hal_pwm_destroy(PWM_CH1);
  hal_pwm_stop();
  hal_gpio_pin_init(GPIO_GREEN, IE);
  hal_gpio_pin_init(GPIO_RED, IE);
  hal_gpio_pin_init(GPIO_YELLOW, IE);
  hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_YELLOW, WEAK_PULL_UP);
  
}

int light_ctrl(uint8_t ch, uint8_t value)
{
  if(ch >2 || (value > 100))
    return PPlus_ERR_INVALID_PARAM;

  s_light[ch] = (uint16_t)value;

  light_reflash();
	return PPlus_SUCCESS;
}


int light_init(void)
{
  s_light[0] = 0;
  s_light[1] = 0;
  s_light[2] = 0;
  hal_gpio_pin_init(GPIO_GREEN, IE);
  hal_gpio_pin_init(GPIO_RED, IE);
  hal_gpio_pin_init(GPIO_YELLOW, IE);
  hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
  hal_gpio_pull_set(GPIO_YELLOW, WEAK_PULL_UP);
  //hal_gpio_pull_set(GPIO_GREEN, STRONG_PULL_UP);
  //hal_gpio_pull_set(GPIO_RED, STRONG_PULL_UP);
  //hal_gpio_pull_set(GPIO_YELLOW, STRONG_PULL_UP);
  return PPlus_SUCCESS;
}

