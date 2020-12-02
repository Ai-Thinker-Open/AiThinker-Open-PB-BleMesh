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
* @file		log.h
* @brief	Contains all functions support for uart driver
* @version	0.0
* @date		31. Jan. 2018
* @author	eagle.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#ifndef __LOG_H__
#define __LOG_H__
#include "uart.h"
int phy_printf(const char *format, ...);
void phy_printf_init(void);

#ifndef DEBUG_INFO
#error "DEBUG_INFO undefined!"
#endif


#if(DEBUG_INFO == 1)
    #define AT_LOG(...)
	#define LOG(...)  phy_printf(##__VA_ARGS__)
	#define LOG_INIT() phy_printf_init()
#elif(DEBUG_INFO == 2)

	#define AT_LOG(...)  phy_printf(##__VA_ARGS__)
	#define LOG(...)
	#define LOG_INIT() phy_printf_init()
#elif(DEBUG_INFO == 3)
    #define LOG(...)  phy_printf(##__VA_ARGS__)
    #define AT_LOG(...)  phy_printf(##__VA_ARGS__)
	#define LOG_INIT() phy_printf_init()
#else
    #define AT_LOG(...)
	#define LOG(...)
	#define LOG_INIT()  //{clk_gate_enable(MOD_UART);clk_reset(MOD_UART);clk_gate_disable(MOD_UART);}
#endif
#endif //__LOG_H__
