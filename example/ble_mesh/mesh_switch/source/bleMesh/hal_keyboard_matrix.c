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

#include <string.h>
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "pwrmgr.h"
//#include "common.h"
#include "gpio.h"
#include "kscan.h"




#include "hal_keyboard_matrix.h"
#include "log.h"

#include "EM_debug.h"
#include "bleMesh.h"



/*********************************************************************
    TYPEDEFS
*/




const uint8 keymap[NUM_KEY_ROWS][NUM_KEY_COLS]=
{

    {VK_POWER,    VK_PAGE_UP,     VK_PAGE_DOWN,  VK_MENU},

    {VK_HOME,     VK_VOICE,       VK_ENTER,      VK_EXIT},

    {VK_LEFT,     VK_RIGHT,       VK_UP,         VK_DOWN},

    {VK_MOUSE_EN, VK_VOLUME_MUTE, VK_VOLUME_UP,  VK_VOLUME_DOWN}

};


const CombineKeys_inf user_combineKey[COMBINE_KEY_NUMBER]=
{
    {VK_MENU, VK_LEFT,  VK_COMBINE_1,  2000},  //
    {VK_MENU, VK_RIGHT, VK_COMBINE_2,  2000},  //
    {VK_MENU, VK_UP,    VK_COMBINE_3,  2000},  //
    {VK_MENU, VK_DOWN,  VK_COMBINE_4,  2000}   //
};


Keys_inf user_normalKey[VK_MAX];

Keys_message KeyCode;
uint8 LastCombine_key=0XFF;

uint8 reCheck_key_number=0;


kscan_Evt_t hal_keyScanEvt;



/*********************************************************************
    GLOBAL VARIABLES
*/
uint8 halKeyboardMatrix_TaskID;   // Task ID for internal task/event processing


/*********************************************************************
    EXTERNAL VARIABLES
*/


/*********************************************************************
    FUNCTIONS
*/
static void hal_keyboard_matrix_task_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void kscan_evt_handler(kscan_Evt_t* evt);

static uint8 key_hold_num=0;


/*********************************************************************
    LOCAL VARIABLES
*/
#if 0 //32pin
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P15,KEY_ROW_P18};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P31};
#else //48 pin
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P25,KEY_ROW_P18};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P20};
#endif

/*  ------------------------------------------------------------------------------------------
    const static uint8_t KSCAN_ROW_GPIO[16] = {0,2,5,7,10,12,15,18,19,22,23,25,27,29,32,34};
    const static uint8_t KSCAN_ROW_MK[16] = {0,1,10,11,4,12,2,5,13,14,6,3,9,15,7,8};
    const static uint8_t KSCAN_COL_GPIO[18] = {1,3,4,6,9,11,13,14,16,17,20,21,24,26,28,30,31,33};
    const static uint8_t KSCAN_COL_MK[18] = {0,1,9,10,4,11,12,2,16,17,5,13,3,14,8,15,7,6};
    -------------------------------------------------------------------------------------------*/

#if 0//32pin
uint8 KscanMK_row[16]= {0,1,2,0XFF,0XFF,3,0xFF,0xFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF};
uint8 KscanMK_col[18]= {0,1,0XFF,2,0XFF,0XFF,0XFF,3,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF};
#else //48 pin
uint8 KscanMK_row[16]= {0,1,0XFF,2,0XFF,3,0xFF,0xFF,0XFF,0XFF,0XFF,0xff,0XFF,0XFF,0XFF,0XFF};
uint8 KscanMK_col[18]= {0,1,0XFF,2,0XFF,3,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF};
#endif



void kscan_evt_hand_hook(kscan_Evt_t* evt)
{
    uint8 key;
    uint8 multi_key1,multi_key2;
    uint8 row,col;
    uint8 release_number=0;
    uint8 press_number=0;
    uint8 reCheck_key_value[10]= {0};
    printf("key change num=%d\n\r",evt->num);

    for(uint8_t i=0; i<evt->num; i++)
    {
        //LOG("index:%d\n\r",i);
        row=KscanMK_row[evt->keys[i].row];
        col=KscanMK_col[evt->keys[i].col];
        key=keymap[row][col];
        #if 1

        if(evt->keys[i].type == KEY_PRESSED)
        {
            press_number++;

            if(user_normalKey[key].Is_Combine_KeyFlag==TRUE)
            {
                // combine key
                if(user_normalKey[key].status==KEY_RELEASED&&user_normalKey[key].ProcessFlag==INIT_ACTIVE)
                {
                    printf("key=%d,combine key Press\n\r",key);
                    user_normalKey[key].ProcessFlag=DEPEND_ACTIVE;
                    user_normalKey[key].status=KEY_PRESSED;
                    user_normalKey[key].StartTick=osal_GetSystemClock();
                    reCheck_key_value[reCheck_key_number]=key;
                    reCheck_key_number++;
                }
                else
                {
                    //key still pressed,but update by other key unnormally.so do not need process.
                }
            }
            else
            {
                //it's not combine key,so direct post event or ignore event
                if(user_normalKey[key].status==KEY_RELEASED&&user_normalKey[key].ProcessFlag==INIT_ACTIVE/*&&key_hold_num==0*/)
                {
                    user_normalKey[key].StartTick=osal_GetSystemClock();
                    user_normalKey[key].ProcessFlag=PRESS_ACTIVED;
                    user_normalKey[key].status=KEY_PRESSED;
                    KeyCode.key=key;
                    KeyCode.status=KEY_PRESSED;
                    KeyCode.time=user_normalKey[key].StartTick;
                    //post_message dircet
                    //osal_set_event(Hal_TaskID, HAL_KEY_DELAY_EVT);
                    printf("key=%d,Press,time=%d\n\r",key,KeyCode.time);
                    osal_set_event(bleMesh_TaskID, BLEMESH_HAL_KEY_MATRIX_EVT);
                }
                else
                {
                    //key still pressed,but update by other key unnormally.so do not need process.
                }
            }
        }
        else
        {
            release_number++;

            if(user_normalKey[key].status==KEY_PRESSED)
            {
                user_normalKey[key].StartTick=osal_GetSystemClock();
                user_normalKey[key].status=KEY_RELEASED;

                if(user_normalKey[key].ProcessFlag==PRESS_ACTIVED)
                {
                    KeyCode.key=key;
                    KeyCode.status=KEY_RELEASED;
                    KeyCode.time=user_normalKey[key].StartTick;
                    //printf("key=%d,release,time=%d\n\r",key,KeyCode.time);
                    //post_message dircet
                    //osal_set_event(Hal_TaskID, HAL_KEY_DELAY_EVT);
                    //printf("Qing add your event 1\n\r");
                }
                else if(user_normalKey[key].ProcessFlag==COMBINE_ACTIVED)
                {
                    if(LastCombine_key!=0XFF)
                    {
                        KeyCode.key=LastCombine_key;
                        KeyCode.status=KEY_RELEASED;
                        KeyCode.time=osal_GetSystemClock();
                        LastCombine_key=0XFF;
                        //printf("key=%d,release,time=%d\n\r",KeyCode.key,KeyCode.time);
                        //post_message dircet
                        //osal_set_event(Hal_TaskID, HAL_KEY_DELAY_EVT);
                        //printf("Qing add your event 1\n\r");
                    }
                }
            }

            //reset
            user_normalKey[key].status=KEY_RELEASED;
            user_normalKey[key].ProcessFlag=INIT_ACTIVE;
            user_normalKey[key].StartTick=0;
        }

        #endif
        //  LOG("key=%d %s\n\r",key,evt->keys[i].type == KEY_PRESSED ? "pressed":"released");

        if(evt->keys[i].type==KEY_PRESSED)
        {
            key_hold_num++;
        }
        else
        {
            key_hold_num>0?(key_hold_num--):0;
        }
    }

    if(reCheck_key_number>0)
    {
        if(reCheck_key_number==1)
        {
            key=reCheck_key_value[0];
            user_normalKey[key].ProcessFlag=PRESS_ACTIVED;
            user_normalKey[key].status=KEY_PRESSED;
            KeyCode.key=key;
            KeyCode.status=KEY_PRESSED;
            KeyCode.time=user_normalKey[key].StartTick;
            reCheck_key_value[0]=0;
            reCheck_key_number=0;
        }
        else if(reCheck_key_number==2)
        {
            multi_key1=reCheck_key_value[0];
            multi_key2=reCheck_key_value[1];

            for(uint8 i=0; i<COMBINE_KEY_NUMBER; i++)
            {
                if((multi_key1==user_combineKey[i].Subkey0&&multi_key2==user_combineKey[i].Subkey1)||\
                        (multi_key1==user_combineKey[i].Subkey1&&multi_key2==user_combineKey[i].Subkey0))
                {
                    user_normalKey[multi_key1].ProcessFlag=COMBINE_ACTIVED;
                    user_normalKey[multi_key2].ProcessFlag=COMBINE_ACTIVED;
                    KeyCode.key=user_combineKey[i].Combine_Key;
                    KeyCode.status=KEY_PRESSED;
                    KeyCode.time=osal_GetSystemClock();
                    LastCombine_key=user_combineKey[i].Combine_Key;//recorde key value for release
                    reCheck_key_number=0;
                    //post_message dircet
                    printf("key=%d,press,time=%d\n\r",user_combineKey[i].Combine_Key,KeyCode.time);
                    //osal_set_event(Hal_TaskID, HAL_KEY_DELAY_EVT);
                    printf("111Qing add your event 1\n\r");
                    break;
                }
            }

            if(reCheck_key_number==2)
            {
                multi_key1=reCheck_key_value[0];
                multi_key2=reCheck_key_value[1];
                user_normalKey[multi_key1].ProcessFlag=NO_ACITVE;
                user_normalKey[multi_key2].ProcessFlag=NO_ACITVE;
            }
        }
        else
        {
            for(uint8 i=0; i<reCheck_key_number; i++)
            {
                user_normalKey[reCheck_key_value[i]].ProcessFlag=NO_ACITVE;
            }

            reCheck_key_number=0;
        }
    }

    //printf("key_hold_num:%d\n\r",key_hold_num);
}


static void kscan_evt_handler(kscan_Evt_t* evt)
{
    //LOG("kscan_evt_handler\n");
//  hal_keyScanEvt=evt;
    osal_memcpy(&hal_keyScanEvt, evt, sizeof(kscan_Evt_t));
    osal_set_event(halKeyboardMatrix_TaskID, HAL_INTERUPT_HANDLER_EVT);
    LOG("key change num=%d\n\r",evt->num);
    //LOG("key_hold_num:%d\n\r",key_hold_num);
}






/*********************************************************************
    @fn      hal_keyboard_matrix_task_init


    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/

void hal_keyboard_matrix_task_init( uint8 task_id )
{
    uint8 i;
    halKeyboardMatrix_TaskID=task_id;
    key_hold_num=0;
    kscan_Cfg_t cfg;
    cfg.ghost_key_state = NOT_IGNORE_GHOST_KEY;
    cfg.key_rows = rows;
    cfg.key_cols = cols;
    cfg.interval = 50;//50
    cfg.evt_handler = kscan_evt_handler;
    hal_kscan_init(cfg, halKeyboardMatrix_TaskID, HAL_KEYSCAN_TIMEOUT_EVT);
    memset(&user_normalKey[0],0,sizeof(Keys_inf)*VK_MAX);

    for(i=0; i<COMBINE_KEY_NUMBER; i++)
    {
        user_normalKey[user_combineKey[i].Subkey0].Is_Combine_KeyFlag=FALSE;
        user_normalKey[user_combineKey[i].Subkey1].Is_Combine_KeyFlag=FALSE;
    }

    printf("hal keyboard matrix init*******\n\r");
}





uint16 hal_keyboard_matrix_task_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( halKeyboardMatrix_TaskID )) != NULL )
        {
            hal_keyboard_matrix_task_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & HAL_KEYSCAN_TIMEOUT_EVT )
    {
        // Perform kscan timeout task
        hal_kscan_timeout_handler();
        return (events ^ HAL_KEYSCAN_TIMEOUT_EVT);
    }

    if(events & HAL_KEYSCAN_POLLING_EVT)
    {
        for(uint8 i=0; i<COMBINE_KEY_NUMBER; i++)
        {
        }

        return (events ^ HAL_KEYSCAN_POLLING_EVT);
    }

    if(events & HAL_INTERUPT_HANDLER_EVT)
    {
        kscan_evt_hand_hook(&hal_keyScanEvt);
        return (events ^ HAL_INTERUPT_HANDLER_EVT);
    }

    return 0;
}


/*********************************************************************
    @fn      bleSmartPeripheral_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void hal_keyboard_matrix_task_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    default:
        // do nothing
        break;
    }
}




