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

#ifndef CONFIG_H
#define CONFIG_H
#include "types.h"

/*
gpio indication
GPIO 14: connection state indication(down: advertisting; up: connected)
GPIO 20: sleep indication(used to debug the sleep, down: in sleep, up: wakeup)
GPIO 18: has data send to external mcu indication, has data need to send, pull up,  the data has send complete, pull down
GPIO 23: the data send to remote state indication(pull up: start send; pull down: send finish)
*/
#define GPIO_INDICATION
#define ENABLE_SLEEP
#ifdef ENABLE_SLEEP
/*
GPIO 15: used to wakeup chip when it is in sleep status. the down edge active
if the external mcu  want to send data to uart, should first pull up then pull down, the system will ready for uart in 1ms.
after the system wakeup, external mcu can pull up this gpio.
if remain pull down state, the system will always wakeup, will not enter sleep.
if pull up these gpio, system will enter sleep if the uart is idle up 6S.
*/
#ifndef GPIO_INDICATION
#define GPIO_INDICATION
#endif
#define GPIO_WAKEUP
#define GPIO_DEBUG_SLEEP
#endif
//#define OPEN_LOG

#define DEBUG_UART
#define DEBUG_SEND_SPEED

#define READY_TO_SEND_WAIT_TIME 1
#define SENDTOREMOTE_INDICATE_TIME_MIN  1
#define NEED_GPIOWAITUP_OUT_MCU
#ifdef NEED_GPIOWAITUP_OUT_MCU
#define BEFORE_TO_SEND2OUT_WAIT_TIME 0
#ifndef GPIO_INDICATION
#define GPIO_INDICATION
#endif
#endif
//#define SLEEP_TIMER
#ifdef ENABLE_SLEEP
#define SLEEP_WAIT_COUNT    6000
extern void notify_app_uart_run(void);
extern void notify_app_uart_run_force(void);
#endif
#ifdef GPIO_INDICATION
extern   void update_connect_state(uint8_t is_connected);
extern   void send_data_2_out_start(void);
extern   void send_data_2_out_end(void);
extern   void send_data_2_remote_start(void);
extern   void send_data_2_remote_end(void);
#endif
#endif
