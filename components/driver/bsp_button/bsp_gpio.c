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

/**************************************************************


    Module Name: bsp_gpio
    File name:   bsp_gpio.c
    Brief description:
    key driver module
    Data:    2020-06-30
    Revision:V0.01
****************************************************************/
#include "bsp_gpio.h"

extern void gpio_btn_pin_event_handler(gpio_pin_e pin,IO_Wakeup_Pol_e type);

static Gpio_Btn_Info* s_gpio_btn_ptr = NULL;
int hal_gpio_btn_init(Gpio_Btn_Info* gpio_btn_ptr)
{
    if((gpio_btn_ptr == NULL)||(GPIO_SINGLE_BTN_NUM == 0))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if((gpio_btn_ptr->s_key == NULL) || (gpio_btn_ptr->cb == NULL))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if(GPIO_SINGLE_BTN_NUM != sizeof(gpio_btn_ptr->s_key)/sizeof(gpio_btn_ptr->s_key[0]))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    for(int i = 0; i < GPIO_SINGLE_BTN_NUM; i++)
    {
        if(GPIO_SINGLE_BTN_IDLE_LEVEL == 0)
        {
            hal_gpio_pull_set(*(gpio_btn_ptr->s_key+i),PULL_DOWN);
        }
        else
        {
            hal_gpio_pull_set(*(gpio_btn_ptr->s_key+i),WEAK_PULL_UP);
        }

        hal_gpioin_register(*(gpio_btn_ptr->s_key+i),gpio_btn_pin_event_handler, gpio_btn_pin_event_handler);
    }

    s_gpio_btn_ptr = gpio_btn_ptr;
    return PPlus_SUCCESS;
}

int hal_gpio_btn_get_index(gpio_pin_e pin,uint8_t* index)
{
    if(s_gpio_btn_ptr == NULL)
    {
        return PPlus_ERR_NOT_SUPPORTED;
    }

    for(int i = 0; i < GPIO_SINGLE_BTN_NUM; i++)
    {
        if(pin == s_gpio_btn_ptr->s_key[i])
        {
            *index = i;
            return PPlus_SUCCESS;
        }
    }

    return PPlus_ERR_NOT_FOUND;
}

void hal_gpio_btn_cb(uint8_t ucKeyCode)
{
    if(s_gpio_btn_ptr != NULL)
    {
        s_gpio_btn_ptr->cb(ucKeyCode);
    }
}
