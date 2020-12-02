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
 * Module Name:	touch key 
 * File name:	touch_key.h 
 * Brief description:
 *    key driver module
 * Author:	Eagle.Lao
 * Data:	2017-07-01
 * Revision:V0.01
****************************************************************/

#ifndef _TOUCH_KEY_H_FILE
#define _TOUCH_KEY_H_FILE
#include "types.h"

typedef enum{
  STATE_KEY_IDLE = 0x00,
  STATE_KEY_PRESS_DEBONCE,
  STATE_KEY_PRESS,
  STATE_KEY_RELEASE_DEBONCE,
}key_state_e;

typedef enum{
	TOUCH_EVT_PRESS   = 0x01,
  TOUCH_EVT_RELEASE,
  TOUCH_EVT_SHORT_PRESS,
	TOUCH_EVT_LONG_PRESS,
} key_evt_t;

typedef void (* key_callbank_hdl_t)(key_evt_t);

typedef struct key_state{
  uint32_t timer_tick;
  uint8    state;
  key_callbank_hdl_t key_callbank;
}key_contex_t;





//#define 	IO_TOUCH_1_PIN			25u
//#define 	TOUCH_INT_GPIOTE_USER_ID   (1<<(IO_TOUCH_1_PIN))

//typedef void (* touch_evt_hdl_t)(key_evt_t key_evt);

int touch_init(key_callbank_hdl_t hdl);
void gpio_key_timer_handler(void);

#endif

