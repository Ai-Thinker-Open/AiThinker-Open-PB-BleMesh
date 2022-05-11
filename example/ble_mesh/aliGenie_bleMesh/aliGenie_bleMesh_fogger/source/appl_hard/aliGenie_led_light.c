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



#include "aliGenie_led_light.h"
#include "pwm.h"
#include "OSAL.h"
#include "gpio.h"
//#include "app_led.h"
#include "error.h"


#define GPIO_GREEN    P32
#define GPIO_BLUE     P23
#define GPIO_RED      P31




static uint16_t s_light[3];
static light_blink_cfg_t s_lightBlink;

static void light_start_timer(void)
{
    //osal_start_timerEx(AppWrist_TaskID, TIMER_LIGHT_EVT, 30*1000);
}
static void light_stop_timer(void)
{
    //osal_stop_timerEx(AppWrist_TaskID, TIMER_LIGHT_EVT);
}


void light_reflash(void)
{
    if(s_light[LIGHT_RED] + s_light[LIGHT_GREEN] + s_light[LIGHT_BLUE])
    {
        hal_pwm_close_channel(PWM_CH0);
        hal_pwm_destroy(PWM_CH0);
        hal_pwm_close_channel(PWM_CH1);
        hal_pwm_destroy(PWM_CH1);
        hal_pwm_close_channel(PWM_CH2);
        hal_pwm_destroy(PWM_CH2);
        hal_pwm_stop();
        hal_gpio_pin_init(GPIO_GREEN, IE);
        hal_gpio_pin_init(GPIO_RED, IE);
        hal_gpio_pin_init(GPIO_BLUE, IE);
        hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
        hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
        hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);

        if(s_light[LIGHT_GREEN])
        {
            hal_pwm_init(PWM_CH0, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
            hal_pwm_set_count_val(PWM_CH0, s_light[LIGHT_GREEN], LIGHT_TOP_VALUE);
            hal_pwm_open_channel(PWM_CH0, GPIO_GREEN);
        }

        if(s_light[LIGHT_BLUE])
        {
            hal_pwm_init(PWM_CH1, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
            hal_pwm_set_count_val(PWM_CH1, s_light[LIGHT_BLUE], LIGHT_TOP_VALUE);
            hal_pwm_open_channel(PWM_CH1, GPIO_BLUE);
        }

        if(s_light[LIGHT_RED])
        {
            hal_pwm_init(PWM_CH2, PWM_CLK_DIV_64, PWM_CNT_UP, PWM_POLARITY_RISING);
            hal_pwm_set_count_val(PWM_CH2, s_light[LIGHT_RED], LIGHT_TOP_VALUE);
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
        hal_pwm_close_channel(PWM_CH2);
        hal_pwm_destroy(PWM_CH2);
        hal_pwm_stop();
        hal_gpio_pin_init(GPIO_GREEN, IE);
        hal_gpio_pin_init(GPIO_RED, IE);
        hal_gpio_pin_init(GPIO_BLUE, IE);
        hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
        hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
        hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
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
    hal_pwm_close_channel(PWM_CH2);
    hal_pwm_destroy(PWM_CH2);
    hal_pwm_stop();
    hal_gpio_pin_init(GPIO_GREEN, IE);
    hal_gpio_pin_init(GPIO_RED, IE);
    hal_gpio_pin_init(GPIO_BLUE, IE);
    hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
}

int light_config(uint8_t ch, uint16_t value)
{
    if(ch >2 || (value > LIGHT_TOP_VALUE))
        return PPlus_ERR_INVALID_PARAM;

    s_light[ch] = (uint16_t)value;
    return PPlus_SUCCESS;
}
int light_ctrl(uint8_t ch, uint16_t value)
{
    if(ch >2 || (value > LIGHT_TOP_VALUE))
        return PPlus_ERR_INVALID_PARAM;

    s_light[ch] = (uint16_t)value;
    light_reflash();
    return PPlus_SUCCESS;
}


int light_init(void)
{
    s_light[LIGHT_GREEN] = 0;
    s_light[LIGHT_BLUE] = 0;
    s_light[LIGHT_RED] = 0;
    hal_gpio_pin_init(GPIO_GREEN, IE);
    hal_gpio_pin_init(GPIO_RED, IE);
    hal_gpio_pin_init(GPIO_BLUE, IE);
    hal_gpio_pull_set(GPIO_GREEN, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_RED, WEAK_PULL_UP);
    hal_gpio_pull_set(GPIO_BLUE, WEAK_PULL_UP);
    //hal_gpio_pull_set(GPIO_GREEN, STRONG_PULL_UP);
    //hal_gpio_pull_set(GPIO_RED, STRONG_PULL_UP);
    //hal_gpio_pull_set(GPIO_YELLOW, STRONG_PULL_UP);
    osal_memset(&s_lightBlink, 0, sizeof(s_lightBlink));
    s_lightBlink.val0=LIGHT_TURN_OFF;
    s_lightBlink.val1=LIGHT_TURN_ON;
    return PPlus_SUCCESS;
}

int light_blink_evt_cfg(uint8_t task_id,uint16_t event_id)
{
    if(s_lightBlink.status==0)
    {
        s_lightBlink.task_id=task_id;
        s_lightBlink.event_id=event_id;
        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_BUSY;
    }
}
int light_blink_set(uint8_t light,uint8 blinkIntv,uint8 blinkCnt)
{
    if(s_lightBlink.status==0)
    {
        s_lightBlink.light=light;
        s_lightBlink.tagCnt=blinkCnt;
        s_lightBlink.intv = blinkIntv;
        s_lightBlink.status=1;

        if(s_lightBlink.task_id>0 && s_lightBlink.event_id>0)
        {
            light_ctrl(LIGHT_RED,0);
            light_ctrl(LIGHT_GREEN,0);
            light_ctrl(LIGHT_BLUE,0);
            s_lightBlink.curCnt=0;
            osal_set_event( s_lightBlink.task_id, s_lightBlink.event_id);
        }
        else
        {
            return PPlus_ERR_NOT_FOUND;
        }

        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_BUSY;
    }
}
void light_blink_porcess_evt(void)
{
    if(s_lightBlink.curCnt==(s_lightBlink.tagCnt*2) )
    {
        light_ctrl(LIGHT_RED,0);
        light_ctrl(LIGHT_GREEN,0);
        light_ctrl(LIGHT_BLUE,0);
        osal_stop_timerEx( s_lightBlink.task_id, s_lightBlink.event_id);
        s_lightBlink.status=0;
    }
    else
    {
        if(s_lightBlink.curCnt&0x01)
        {
            light_ctrl(s_lightBlink.light,s_lightBlink.val1);
        }
        else
        {
            light_ctrl(s_lightBlink.light,s_lightBlink.val0);
        }

        s_lightBlink.curCnt++;
        osal_start_timerEx(s_lightBlink.task_id, s_lightBlink.event_id,s_lightBlink.intv*100);
    }
}
