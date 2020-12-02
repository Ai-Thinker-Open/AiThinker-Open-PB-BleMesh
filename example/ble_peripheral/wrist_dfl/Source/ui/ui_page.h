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
 *
 * Module Name:	UI page management
 * File name:	ui_page.h
 * Brief description:
 *    UI page management
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/

#ifndef __UI_PAGE_HEAD
#define __UI_PAGE_HEAD
#include "dfl.h"
#include "ui_task.h"
#include "battery.h"


typedef void (*_call_ui_page_init_t)(void);
typedef int (*_call_ui_fsm_run_t)(ui_ev_t* pev);
typedef void (*_call_ui_page_timer_evt_t)(void* pctx);
typedef void (*_call_ui_batt_event_t)(batt_evt_t evt);
typedef void (*_call_ui_key_evt_t)(void* pctx);
typedef void (*_call_ui_accelerator_event_t)(void* pacc);


#define ui_page_init()  ((_call_ui_page_init_t)DFL_FUNC(0))()
#define ui_fsm_run(pev)   ((_call_ui_fsm_run_t)DFL_FUNC(1))(pev)
#define ui_page_timer_evt(p) ((_call_ui_page_timer_evt_t)DFL_FUNC(2))(p)
#define ui_batt_event(p) ((_call_ui_batt_event_t)DFL_FUNC(3))(p)
#define ui_key_evt(p) ((_call_ui_key_evt_t)DFL_FUNC(4))(p)
#define ui_accelerator_event(p) ((_call_ui_accelerator_event_t)DFL_FUNC(5))(p)

#endif


