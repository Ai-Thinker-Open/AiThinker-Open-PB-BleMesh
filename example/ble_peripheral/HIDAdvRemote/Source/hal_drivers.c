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


#include "gpio.h"


#include "log.h"
#include "OSAL.h"

#include "pwrmgr.h"
#include "common.h"
#include "hidkbd.h"

#include "hal_drivers.h"

#include "flash.h"
#include <stdlib.h>

//#include "touch_key.h"
#include "voice_task.h"



#include "Voice_Queue.h"
#include "hci.h"
#include "hal_keyboard_matrix.h"

#define KEY_MODE MOD_USR6

uint8 Hal_TaskID;




/**************************************************************************************************
 * @fn      Hal_Init
 *
 * @brief   Hal Initialization function.
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void Hal_task_Init ( uint8 task_id )
{
	Hal_TaskID=task_id;


}


/**************************************************************************************************
 * @fn      Hal_ProcessEvent
 *
 * @brief   Hal Process Event
 *
 * @param   task_id - Hal TaskId
 *          events - events
 *
 * @return  None
 **************************************************************************************************/
uint16 Hal_ProcessEvent( uint8 task_id, uint16 events )
{
    uint8 *msgPtr;

    (void)task_id;  // Intentionally unreferenced parameter
    
     LOG("Hal_Process:%x\n",events);
    //=============================================================================================
    if ( events & SYS_EVENT_MSG )
    {
        msgPtr = osal_msg_receive(Hal_TaskID);

        while (msgPtr)
        {
            /* Do something here - for now, just deallocate the msg and move on */

            /* De-allocate */
            osal_msg_deallocate( msgPtr );
            /* Next */
            //msgPtr = osal_msg_receive( Hal_TaskID );
        }
        return events ^ SYS_EVENT_MSG;
    }

	if(events &HAL_KEY_EVENT)
	{
    // gpio_key_timer_handler();
     return events ^ HAL_KEY_EVENT;
	}

	if(events &HAL_KEY_PRESS_EVT)
	{
     LOG("Key press\n\r");
	 hal_pwrmgr_lock(KEY_MODE);

	 #if VOICE_FROM_FLASH
	 osal_start_timerEx(hidKbdTaskId, VOICE_PROCESS_EVT, 10);
	 #else
	 //start Dmic record
	 osal_set_event(voice_TaskID, VOICE_RECORD_START_EVT);
	
	 #endif
	 
     return events ^ HAL_KEY_PRESS_EVT;
	}

	if(events &HAL_KEY_RELEASE_EVT)
	{
      LOG("Key release\n\r");
	  hal_pwrmgr_unlock(KEY_MODE);

	  VoiceQueue.VoiceSendFlg=0;
	  #if VOICE_FROM_FLASH
       osal_set_event(hidKbdTaskId, VOICE_TRANSF_EVT);
	   osal_start_timerEx(hidKbdTaskId, VOICE_STOP_EVT, 100);
	   
	   HCI_PPLUS_ConnEventDoneNoticeCmd( hidKbdTaskId, 0);
	  #else
	  HCI_PPLUS_ConnEventDoneNoticeCmd( hidKbdTaskId, 0);
      osal_set_event(voice_TaskID, VOICE_RECORD_STOP_EVT);
	  osal_stop_timerEx(hidKbdTaskId, VOICE_TRANSF_EVT);
	  hidKbdSendVoiceCMDtReport(0x00);
	  osal_start_timerEx(hidKbdTaskId, VOICE_STOP_EVT, 30);
	  #endif

	 
	 
     return events ^ HAL_KEY_RELEASE_EVT;
	}
	
	 if(events &HAL_KEY_DELAY_EVT)
	 {
	   
	    uint8 cmd =0;
		
		if(KeyCode.key==VK_VOICE&&KeyCode.status==KEY_PRESSED)
		{
		 osal_set_event(Hal_TaskID, HAL_KEY_PRESS_EVT);
		}
		else if(KeyCode.key==VK_VOICE&&KeyCode.status==KEY_RELEASED)
		{
		  osal_set_event(Hal_TaskID, HAL_KEY_RELEASE_EVT);
		}
		else if(KeyCode.key==VK_VOLUME_UP&&KeyCode.status==KEY_PRESSED)
		{
		   cmd=9;
          hidCCSendReportKey(cmd,KEY_PRESSED);
		}
		else if(KeyCode.key==VK_VOLUME_UP&&KeyCode.status==KEY_RELEASED)
		{
		    cmd=0;
          hidCCSendReportKey(cmd,KEY_RELEASED);
		}
		else if(KeyCode.key==VK_VOLUME_DOWN&&KeyCode.status==KEY_PRESSED)
		{
		    cmd=10;
          hidCCSendReportKey(cmd,KEY_PRESSED);
		}
		else if(KeyCode.key==VK_VOLUME_DOWN&&KeyCode.status==KEY_RELEASED)
		{
		      cmd=0;
          hidCCSendReportKey(cmd,KEY_RELEASED);
		}
		else
		{
          //other key ,wait to process
		}
		 return events ^ HAL_KEY_DELAY_EVT;
	 }

	return 0;
}

