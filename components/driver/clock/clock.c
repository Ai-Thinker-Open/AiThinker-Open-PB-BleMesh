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
#include "hal_defs.h"
#include "hal_mcu.h"
#include "clock.h"
#include "types.h"
#include "gpio.h"
#include "global_config.h"

extern uint32_t hclk,pclk;
extern uint32_t osal_sys_tick;


void clk_gate_enable(MODULE_e module)
{
  if(module <3)
    return;
  AP_PCR->CLKG |= BIT(module);
}

void clk_gate_disable(MODULE_e module)
{
  if(module <3)
    return;
  AP_PCR->CLKG &= ~(BIT(module));
}

void clk_reset(MODULE_e module)
{
  AP_PCR->RESET &= ~(BIT(module));
  AP_PCR->RESET |= BIT(module);
}

uint32_t clk_hclk(void)
{
  return hclk;
}
uint32_t clk_pclk(void)
{
  return pclk;
}


/**************************************************************************************
 * @fn          hal_rtc_clock_config
 *
 * @brief       This function process for 32768Hz Clock configuation
 *
 * input parameters
 *
 * @param       CLK32K_e clk32Mode: CLK_32K_RC_LOWPOWER --> use 32K RC osc for extra low power
                                  : CLK_32K_RC_STABLE   --> use 32K RC osc for stable conectivity
                                  : CLK_32K_XTAL        --> use 32K Xtal
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None
 **************************************************************************************/

void hal_rtc_clock_config(uint8_t clk32Mode)
{
    if(clk32Mode == CLK_32K_RCOSC)
    {
        subWriteReg(0x4000f014,31,27,0x05);
        subWriteReg(0x4000f01c,16,7,0x3fb);   //software control 32k_clk
        subWriteReg(0x4000f01c,6,6 ,0x01);    //enable software control
        
        pGlobal_config[LL_SWITCH]|=RC32_TRACKINK_ALLOW|LL_RC32K_SEL;

//        //disable smart windwo for stable conectivity not higher power consumption
//        pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]=0;
    }
    else if(clk32Mode == CLK_32K_XTAL)
    {
        // P16 P17 for 32K XTAL input
        hal_gpio_pull_set(P16,FLOATING);
        hal_gpio_pull_set(P17,FLOATING);

        subWriteReg(0x4000f01c,9,8,0x03);   //software control 32k_clk
        subWriteReg(0x4000f01c,6,6,0x00);   //disable software control

        subWriteReg(0x4000f014,31,27,0x16);
        pGlobal_config[LL_SWITCH]&=0xffffffee;
    }

}

uint32_t hal_systick(void)
{
  return osal_sys_tick;
}


uint32_t hal_ms_intv(uint32_t tick)
{
  uint32_t diff = 0;
  if(osal_sys_tick < tick){
    diff = 0xffffffff- tick;
    diff = osal_sys_tick + diff;
  }
  else
  {
    diff = osal_sys_tick - tick;
  }
  return diff*625/1000;
}

