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
    Filename:       gpio_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"
#include "OSAL.h"
#include "halPeripheral.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "global_config.h"
#include "cliface.h"
#include "key.h"
#include "led_light.h"
#include "halPeripheral.h"


uint8 Hal_TaskID;

uint8_t cmdstr[64];
uint8_t cmdlen;

extern key_contex_t key_state;

uint16_t cli_demo_help(uint32_t argc, uint8_t* argv[]);
uint16_t cli_demo_light_ctrl(uint32_t argc, uint8_t* argv[]);

const CLI_COMMAND cli_cmd_list[] =
{
    /* Help */
    { "help", "Help on this CLI Demo Menu", cli_demo_help },

    /* Light control */
    { "light", "This is CLI light control", cli_demo_light_ctrl },
};

static void ProcessUartData(uart_Evt_t* evt)
{
    if(evt->len)
    {
        osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
        cmdlen += evt->len;
        osal_set_event( Hal_TaskID, KEY_DEMO_UART_RX_EVT );
    }
}

static void key_press_evt(uint8_t i,key_evt_t key_evt)
{
    LOG("\nkey index:%d gpio:%d ",i,key_state.key[i].pin);

    switch(key_evt)
    {
    case HAL_KEY_EVT_PRESS:
        LOG("key(press down)\n");
        #ifdef HAL_KEY_SUPPORT_LONG_PRESS
        osal_start_timerEx(key_state.task_id,KEY_DEMO_LONG_PRESS_EVT,HAL_KEY_LONG_PRESS_TIME);
        #endif
        break;

    case HAL_KEY_EVT_RELEASE:
        LOG("key(press release)\n");
        break;
        #ifdef HAL_KEY_SUPPORT_LONG_PRESS

    case HAL_KEY_EVT_LONG_RELEASE:
        hal_pwrmgr_unlock(MOD_USR1);
        LOG("key(long press release)\n");
        break;
        #endif

    default:
        LOG("unexpect\n");
        break;
    }
}

void peripheral_uart_init(void)
{
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessUartData,
    };
    hal_uart_init(cfg,UART0);//uart init
}

void HalPeripheral_Init(uint8 task_id)
{
    uint8_t i;
    Hal_TaskID = task_id;
    hal_uart_deinit(UART0);
    peripheral_uart_init();
    light_init();
    light_blink_evt_cfg(Hal_TaskID,LIGHT_PRCESS_EVT);
    key_state.key[0].pin = GPIO_P11;

    for(i = 0; i < HAL_KEY_NUM; ++i)
    {
        key_state.key[i].state = HAL_STATE_KEY_IDLE;
        key_state.key[i].idle_level = HAL_HIGH_IDLE;
    }

    key_state.task_id = Hal_TaskID;
    key_state.key_callbank = key_press_evt;
    key_init();
}

uint16 HalPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != Hal_TaskID)
    {
        return 0;
    }

    if( events & KEY_DEMO_UART_RX_EVT)
    {
        if ('\r' == cmdstr[cmdlen - 1])
        {
            cmdstr[cmdlen - 1] = '\0';
            LOG("%s", cmdstr);
            CLI_process_line
            (
                cmdstr,
                cmdlen,
                (CLI_COMMAND*) cli_cmd_list,
                (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND))
            );
            cmdlen = 0;
        }

        return (events ^ KEY_DEMO_UART_RX_EVT);
    }

    if(events & LIGHT_PRCESS_EVT)
    {
        light_blink_porcess_evt();
        return (events ^ LIGHT_PRCESS_EVT);
    }

    if( events & HAL_KEY_EVENT)                                                     //do not modify,key will use it
    {
        LOG("evt\n");

        for (uint8 i = 0; i < HAL_KEY_NUM; ++i)
        {
            if ((key_state.temp[i].in_enable == TRUE)||
                    (key_state.key[i].state == HAL_STATE_KEY_RELEASE_DEBOUNCE))
            {
                gpio_key_timer_handler(i);
            }
        }

        return (events ^ HAL_KEY_EVENT);
    }

    #ifdef HAL_KEY_SUPPORT_LONG_PRESS

    if( events & KEY_DEMO_LONG_PRESS_EVT)
    {
        for (int i = 0; i < HAL_KEY_NUM; ++i)
        {
            if(key_state.key[i].state == HAL_KEY_EVT_PRESS)
            {
                LOG("key:%d gpio:%d	",i,key_state.key[i].pin);
                LOG("key(long press down)");
                osal_start_timerEx(key_state.task_id,KEY_DEMO_LONG_PRESS_EVT,HAL_KEY_LONG_PRESS_TIME);//2s
                //user app code long press down process
            }
        }

        return (events ^ KEY_DEMO_LONG_PRESS_EVT);
    }

    #endif
    // Discard unknown events
    return 0;
}

uint16_t cli_demo_help(uint32_t argc, uint8_t* argv[])
{
    uint32_t index;
    LOG("\r\nCLI Demo\r\n");

    /* Print all the available commands */
    for (index = 0; index < (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND)); index++)
    {
        LOG("    %s: %s\n",
            cli_cmd_list[index].cmd,
            cli_cmd_list[index].desc);
    }

    return 0;
}

uint16_t cli_demo_light_ctrl(uint32_t argc, uint8_t* argv[])
{
    uint8_t mode;
    LOG("\r\nLight Demo\r\n");

    if(argc == 1)
    {
        mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);

        if(LIGHT_COLOR_NUM < mode)
        {
            LOG("Invalid Parament:0x%04X. Returning.\n", mode);
            return 0xffff;
        }

        light_color_quickSet((light_color_t)mode);
    }
    else
    {
        LOG("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return 0xffff;
    }

    mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
    return 0;
}
/*********************************************************************
*********************************************************************/
