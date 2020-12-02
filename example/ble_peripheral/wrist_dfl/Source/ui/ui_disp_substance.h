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
 * Module Name:	display substance
 * File name:	ui_disp_substance.h 
 * Brief description:
 *    UI display substance.
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/

#ifndef _UI_SUBST_HEAD
#define _UI_SUBST_HEAD

#include "types.h"
#include "app_datetime.h"
#include <time.h>

void ui_normal_call_icon(void);
void ui_normal_call_info(uint16_t frame_id, const char* title);

void ui_normal_blank_on_style_simple(datetime_t* pdtm, uint8_t* p_addr);
void ui_normal_acc_init(void);
void ui_normal_acc(int x, int y, int z);
void ui_normal_pedometer_key_press_long(void);
void ui_normal_pedometer_steps(uint16_t steps, uint8_t frame_id);
void ui_normal_calorie_value(uint16_t calorie);
void ui_normal_hr_blink(uint8_t frame_id);
void ui_normal_hr_failure(void);
void ui_normal_hr_heartrate(uint8_t value);
void ui_normal_humiture_icon(uint8_t frame_id);
void ui_normal_cup_icon(uint8_t frame_id);
void ui_normal_batt_icon(uint8_t frame_id);
void ui_normal_batt_reflash(uint8_t frame_id, uint8_t percent);
void ui_normal_alarm_clk_icon(void);
void ui_normal_msg_title(uint16_t frame_id, uint16_t ev_id, char* title);
void ui_normal_msg_info(uint16_t ev_id, uint16_t** u_str_tbl,bool end);

void ui_firmware_upgrade(void);

void ui_normal_new_theme(uint8_t style);

#endif


