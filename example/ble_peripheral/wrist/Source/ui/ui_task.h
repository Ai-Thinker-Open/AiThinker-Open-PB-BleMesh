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
 * Module Name:	UI task
 * File name:	ui_task.h
 * Brief description:
 *    UI task, event driver
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/

#ifndef UI_TASK_HEAD__
#define UI_TASK_HEAD__
#include <stddef.h>
#include <stdint.h>
#include "app_err.h"

enum{
	UI_EV_KEY_PRESS = 1,
	UI_EV_KEY_PRESS_LONG,
	UI_EV_KEY_PRESS_LL,

	UI_EV_HAND_UP,
	UI_EV_HAND_DOWN,
	

	UI_EV_BATT_CHARGE,
	UI_EV_BATT_CHARGE_OFF,
  UI_EV_BATT_VOLTAGE,

	UI_EV_TIMER,

	UI_EV_FIT_STEP_CHG,	//step change
	
  UI_EV_ACCELERATOR, //accelerator data


	UI_EV_HEARTRATE,
	UI_EV_HEARTRATE_FAILED,
	UI_EV_HEARTRATE_TIMEOUT,

	UI_EV_HUMITURE,
	UI_EV_HUMITURE_FAILED,
	UI_EV_HUMITURE_TIMEOUT,

	UI_EV_BLE_CONNECT,
	UI_EV_BLE_DISCONNECT,
	UI_EV_BLE_CALL,
	UI_EV_BLE_CALL_INFO,
	UI_EV_BLE_CALL_OFF,
	UI_EV_BLE_SMS,
	UI_EV_BLE_WECHAT,
	UI_EV_BLE_QQ,
	UI_EV_BLE_MAIL,
	UI_EV_BLE_MSG_NOTIFY,

	UI_EV_PAGE_CONSTRUCT = 0x80,
	UI_EV_PAGE_DESTRUCT
};


typedef struct{
	uint16_t ev;
	uint16_t param;
	uint8_t* data;
}ui_ev_t;


int ui_init(void);


#endif

