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
#include "watchdog.h"
#include "error.h"
#include "clock.h"
#include "jump_function.h"

extern volatile uint8 g_clk32K_config;
extern uint32_t s_config_swClk1;
uint8_t g_wdt_cycle = 0xFF;//valid value:0~7.0xFF:watchdog disable.

void hal_WATCHDOG_IRQHandler(void)
{
    volatile uint32_t a;
    a = AP_WDT->EOI;
    AP_WDT->CRR = 0x76;
    //LOG("WDT IRQ[%08x]\n",rtc_get_counter());
}

__ATTR_SECTION_SRAM__ void hal_watchdog_init(void)
{
    volatile uint32_t a;
    uint8_t delay;

    if(g_wdt_cycle > 7)
        return ;

    if(g_clk32K_config == CLK_32K_XTAL)//rtc use 32K XOSC,watchdog use the same
    {
        AP_PCRM->CLKSEL |= (1UL<<16);
    }
    else
    {
        AP_PCRM->CLKSEL &= ~(1UL<<16); //rtc use 32K RCOSC,watchdog use the same
    }

    hal_clk_gate_enable(MOD_WDT);
    s_config_swClk1|=_CLK_WDT; //add watchdog clk in pwrmg wakeup restore clk;

    if((AP_PCR->SW_RESET0 & 0x04)==0)
    {
        AP_PCR->SW_RESET0 |= 0x04;
        delay = 20;

        while(delay-->0);
    }

    if((AP_PCR->SW_RESET2 & 0x04)==0)
    {
        AP_PCR->SW_RESET2 |= 0x04;
        delay=20;

        while(delay-->0);
    }

    AP_PCR->SW_RESET2 &= ~0x20;
    delay=20;

    while(delay-->0);

    AP_PCR->SW_RESET2 |= 0x20;
    delay=20;

    while(delay-->0);

    a = AP_WDT->EOI;
    AP_WDT->TORR = g_wdt_cycle;
    #if (HAL_WDG_CFG_MODE==WDG_USE_INT_MODE)
    NVIC_SetPriority((IRQn_Type)WDT_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)WDT_IRQn);
    JUMP_FUNCTION(WDT_IRQ_HANDLER) = (uint32_t)&hal_WATCHDOG_IRQHandler;
    AP_WDT->CR = 0x1F;//use int
    #else
    AP_WDT->CR = 0x1D;//not use int
    #endif
    AP_WDT_FEED;
}

void hal_watchdog_feed(void)
{
    AP_WDT_FEED;
}

int watchdog_config(uint8 cycle)
{
    if(cycle > 7)
        return PPlus_ERR_INVALID_PARAM;
    else
        g_wdt_cycle = cycle;

    hal_watchdog_init();
    JUMP_FUNCTION(HAL_WATCHDOG_INIT) = (uint32_t)&hal_watchdog_init;
    return PPlus_SUCCESS;
}
