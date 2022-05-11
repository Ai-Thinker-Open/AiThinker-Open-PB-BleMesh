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


    Module Name: hal_keyboard_matrix
    File name:   hal_keyboard_matrix.h
    Brief description:
      key driver module
    Author:  Teddy.Deng
    Data:    2018-12-20
    Revision:V0.01
****************************************************************/

#ifndef _HAL_KEYBOARD_MATRIX_H_
#define _HAL_KEYBOARD_MATRIX_H_
#include "types.h"
#include "kscan.h"


#define HAL_KEYSCAN_TIMEOUT_EVT  0X1000
#define HAL_KEYSCAN_POLLING_EVT  0X2000

#define HAL_INTERUPT_HANDLER_EVT  0X4000


#define HAL_KEYSCAN_POLLING_TIMEOUT 40


#define COMBINE_KEY_NUMBER  4//4





typedef enum
{
    VK_NC            = 0x00,
    VK_POWER         = 0x01,
    VK_PAGE_UP       = 0x02,
    VK_PAGE_DOWN     = 0x03,
    VK_MENU          = 0x04,
    VK_HOME          = 0x05,
    VK_VOICE         = 0x06,
    VK_ENTER         = 0x07,
    VK_EXIT          = 0x08,
    VK_LEFT          = 0x09,
    VK_RIGHT         = 0x0A,
    VK_UP            = 0x0B,
    VK_DOWN          = 0x0C,
    VK_MOUSE_EN      = 0x0D,
    VK_VOLUME_MUTE   = 0x0E,
    VK_VOLUME_UP     = 0x0F,
    VK_VOLUME_DOWN   = 0x10,
    VK_TV            = 0X11,
    VK_MAX           = VK_TV+1,


} VirtualKeyDefs;

typedef enum
{
    VK_COMBINE_1 = VK_TV+1,
    VK_COMBINE_2 = VK_COMBINE_1+1,
    VK_COMBINE_3 = VK_COMBINE_2+1,
    VK_COMBINE_4 = VK_COMBINE_3+1,

} CombineKeyDefs;

typedef enum
{
    INIT_ACTIVE = 0,
    DEPEND_ACTIVE =1,
    PRESS_ACTIVED =2,
    COMBINE_ACTIVED = 3,
    NO_ACITVE       =4

} KeyProcessFlags;




typedef struct
{
    uint8 Is_Combine_KeyFlag;// init it.TRUE set as combine key
    uint8 status;//press or release status
    uint8 ProcessFlag; // 0,no need to process,1:it can be processed by press,release ,and combine key.2:processed by combine key
    uint32 StartTick;
} Keys_inf;

typedef struct
{
    uint8 Subkey0;//assign other key as combine key
    uint8 Subkey1;//assign other key as combine key
    uint8 Combine_Key;//target key
    uint16 timeout;// ms,if 0,no need to delay.process right now
} CombineKeys_inf;


typedef struct
{
    uint8 key;// init it.TRUE set as combine key
    uint8 status;//press or release status
    uint32 time;
} Keys_message;

extern Keys_message KeyCode;
extern const uint8 keymap[NUM_KEY_ROWS][NUM_KEY_COLS];


extern const CombineKeys_inf user_combineKey[COMBINE_KEY_NUMBER];


extern uint8 halKeyboardMatrix_TaskID;   // Task ID for internal task/event processing
extern void hal_keyboard_matrix_task_init( uint8 task_id );

extern uint16 hal_keyboard_matrix_task_ProcessEvent( uint8 task_id, uint16 events );



#endif


