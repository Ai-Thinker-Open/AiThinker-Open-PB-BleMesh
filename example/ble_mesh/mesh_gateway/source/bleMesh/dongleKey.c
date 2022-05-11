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
#include "dongleKey.h"
#include "log.h"
#include "OSAL.h"
#include "pwrmgr.h"
//#include "common.h"
#include "error.h"
#include "EM_platform.h"
#include "bleMesh.h"

extern uint8 g_buttonCounter;
extern uint8 bleMesh_TaskID;


key_contex_t key_state;
key_pin_t   m_key_pin;


extern uint32 getMcuPrecisionCount(void);

static int key_timer_start(uint32 intval_ms)
{
    osal_start_timerEx(bleMesh_TaskID, BLEMESH_KEY_PRESS_PRO_EVT, intval_ms);
    return 0;
}

static void key_idle_handler(uint8 index,IO_Wakeup_Pol_e type)
{
    if(type == NEGEDGE)
    {
        hal_pwrmgr_lock(MOD_USR2);
        key_state.state[index]=STATE_KEY_PRESS_DEBONCE;
        key_state.in_enable[index]=TRUE;
        key_timer_start(20);
    }
}


static void key_press_debonce_handler(IO_Wakeup_Pol_e type)
{
    if(type == NEGEDGE)
    {
        key_timer_start(20);
    }
}


static void key_press_handler(uint8 index,IO_Wakeup_Pol_e type)
{
    if(type == POSEDGE)
    {
        hal_pwrmgr_lock(MOD_USR1);
        key_state.state[index]=STATE_KEY_RELEASE_DEBONCE;
        key_timer_start(20);
    }
}


static void key_release_debonce_handler(IO_Wakeup_Pol_e type)
{
    if(type == POSEDGE)
    {
        key_timer_start(20);
    }
}

static uint8 get_gpioin_index(GPIO_Pin_e in_pin)
{
    uint8 i;

    for (i = 0; i < KEY_NUM; ++i)
    {
        if(m_key_pin.pin[i]==in_pin)
        {
            return i;
        }
    }

    return 0xff;    //invalid result
}

static void pin_event_handler(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    uint8 index;
    index=get_gpioin_index(pin);

    if (index < KEY_NUM)
    {
        switch(key_state.state[index])
        {
        case STATE_KEY_IDLE:
            key_idle_handler(index,type);
            break;

        case STATE_KEY_PRESS_DEBONCE:
            key_press_debonce_handler(type);
            break;

        case STATE_KEY_PRESS:
            key_press_handler(index,type);
            break;

        case STATE_KEY_RELEASE_DEBONCE:
            key_release_debonce_handler(type);
            break;

        default:
            break;
        }
    }
    else
    {
        return;
    }
}

int dongleKey_init(key_callbank_hdl_t hdl)
{
    uint32 ret;
    uint8 i;
    GPIO_Pin_e pin[KEY_NUM]= {P14};
    hal_gpio_pull_set(P14, STRONG_PULL_UP);
    key_state.key_callbank=hdl;

    for (i = 0; i < KEY_NUM; ++i)
    {
        m_key_pin.pin[i]=pin[i];
        key_state.state[i]=STATE_KEY_IDLE;
        key_state.timer_tick[i]=0;
        ret = hal_gpioin_register(m_key_pin.pin[i], pin_event_handler, pin_event_handler);
    }

    hal_pwrmgr_register(MOD_USR2, NULL, NULL);
    return ret;
}

static void key_press_debonce_timer_handler(uint8 index)
{
    if(hal_gpio_read(m_key_pin.pin[index])==0)
    {
        hal_pwrmgr_unlock(MOD_USR2);
        key_state.state[index]=STATE_KEY_PRESS;
        key_state.timer_tick[index]=getMcuPrecisionCount();

        //osal_start_timerEx(Sensor_Broadcast_TaskID,KEY_LONG_PRESS_EVT,3*1000);
        if(key_state.key_callbank!=NULL)
        {
            key_state.key_callbank(TOUCH_EVT_PRESS,index);
        }
    }
    else if(hal_gpio_read(m_key_pin.pin[index])==1)
    {
        key_state.state[index]=STATE_KEY_IDLE;
        key_state.in_enable[index]=FALSE;
    }
}

static void key_release_debonce_timer_handler(uint8 index)
{
    if (index == 0)
    {
        if(hal_gpio_read(m_key_pin.pin[index])==1)
        {
            key_state.in_enable[index]=FALSE;
            osal_stop_timerEx(bleMesh_TaskID,BLEMESH_KEY_LONG_PRESS_EVT);
            uint32_t hold_tick = (getMcuPrecisionCount()-key_state.timer_tick[index])*625;
            hal_pwrmgr_unlock(MOD_USR2);
            key_state.state[index]=STATE_KEY_IDLE;

            if(key_state.key_callbank!=NULL)
            {
                if(hold_tick <= 1*1000*1000)
                {
                    //LOG("short");
                    key_state.key_callbank(TOUCH_EVT_SHORT_PRESS,index);
                }
                else
                {
                    //LOG("release");
                    key_state.key_callbank(TOUCH_EVT_RELEASE,index);
                }
            }
        }
        else
        {
            key_state.state[index]=STATE_KEY_PRESS;
        }
    }
    else
    {
        if(hal_gpio_read(m_key_pin.pin[index])==1)
        {
            key_state.in_enable[index]=FALSE;
            hal_pwrmgr_unlock(MOD_USR2);
            key_state.state[index]=STATE_KEY_IDLE;
        }
        else
        {
            key_state.state[index]=STATE_KEY_PRESS;
        }
    }
}

void gpio_key_timer_handler(uint8 index)
{
    switch(key_state.state[index])
    {
    case STATE_KEY_PRESS_DEBONCE:
        key_press_debonce_timer_handler(index);
        break;

    case STATE_KEY_RELEASE_DEBONCE:
        key_release_debonce_timer_handler(index);
        break;

    default:
        break;
    }
}


