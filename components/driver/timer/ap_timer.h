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

#ifndef __AP_TIMER_H__
#define __AP_TIMER_H__

#include "osal.h"
#include "types.h"

enum{
  APTM_EVT_TIMER_1 = 1,
  APTM_EVT_TIMER_2,
  APTM_EVT_TIMER_3,
  APTM_EVT_TIMER_4,
  APTM_EVT_WAKEUP= 0x10,
  APTM_EVT_SLEEP
};

enum{
  AP_TM_ID_1 = 0,
  AP_TM_ID_2,
  AP_TM_ID_3,
  AP_TM_ID_4,
};

typedef void(*ap_tm_hdl_t)(uint8_t evt);

//ap timer interrupt process function.
void __attribute__((weak)) AP_TIMER_IRQHandler(void);

//enable ap timer interrupt.
int ap_timer_init(ap_tm_hdl_t callback);

//disable ap timer interrupt and disable its clock.
int ap_timer_deinit(void);

//set timer registers,ready to run.
//timeId should form 1 to 4.time mutiple four should not more than 0xffffff.
int ap_timer_set(uint8_t timeId, uint32_t us);

//clear timer registers,stop it.
int ap_timer_clear(uint8 timeId);


int ap_timer_int_mask(uint8 timeId, bool en);

#endif

