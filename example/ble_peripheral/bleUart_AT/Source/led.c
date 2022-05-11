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

/*******************************************************************************
    @file     led.c
    @brief    Contains all functions support for led driver
    @version  0.1
    @date     14. Sep. 2020
    @author   bingliang.lou

    Copyright(C) 2016, PhyPlus Semiconductor
    All rights reserved.

*******************************************************************************/
#include "led.h"
//#include "common.h"
#include "error.h"
#include "bus_dev.h"
#include "bleuart.h"
/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
GPIO_Pin_e led_Pin;
LED_PWR_e led_pwr = LED_PWR_OFF;

/*********************************************************************
    LOCAL FUNCTIONS
*/
/*********************************************************************
    PUBLIC FUNCTIONS
*/
void set_led(LED_PWR_e led_pwr)
{
    switch(led_pwr)
    {
    case LED_PWR_OFF:
        led_pwr = LED_PWR_OFF;
        break;

    case LED_PWR_ON:
        led_pwr = LED_PWR_ON;
        break;

    default:
        led_pwr = LED_PWR_OFF;
        break;
    }
}

LED_PWR_e get_led(void)
{
    return led_pwr;
}

/*********************************************************************
    @fn      led_initial

    @brief   Initialize GPIO pin for LED.

    @params  pin: GPIO pin for controling LED.

    @return  N/A
*/
void led_initial(GPIO_Pin_e pin)
{
    led_Pin = pin;
    hal_gpio_pin_init(led_Pin, OEN);
    hal_gpio_write(led_Pin, 0);
    hal_gpio_pull_set(led_Pin, PULL_DOWN);
}

/*********************************************************************
    @fn      led_set_status

    @brief   Set LED status.
            No blink status here to make thing simple.
            Task can use event+timer for blink status by itself instead.

    @params  led_status: GPIO pin for controlling LED.

    @return  N/A
*/
void led_set_status(LED_STATUS_e led_status)
{
    switch(led_status)
    {
    case LED_STATUS_OFF:
        hal_gpio_write(led_Pin, 0);
        hal_gpio_pull_set(led_Pin, PULL_DOWN);
        set_led(LED_PWR_OFF);
        break;

    case LED_STATUS_ON:
        hal_gpio_write(led_Pin, 1);
        hal_gpio_pull_set(led_Pin, STRONG_PULL_UP); //STRONG_PULL_UP / WEAK_PULL_UP
        set_led(LED_PWR_ON);
        break;

    case LED_STATUS_BLINK:
        osal_start_timerEx( bleuart_TaskID, BUP_OSAL_EVT_LED_BLK_TIMER, 247 );
        break;

    default:
        hal_gpio_pull_set(led_Pin, PULL_DOWN);
        break;
    }
}
