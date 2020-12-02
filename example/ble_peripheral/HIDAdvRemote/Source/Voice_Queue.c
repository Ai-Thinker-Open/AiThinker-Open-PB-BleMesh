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

#include "Voice_Queue.h"
#include "osal.h"
#include "log.h"
#include "hidkbd.h"
Queue VoiceQueue;

uint8 Voicebuf[VOICE_QUEUE_MAX_LENGTH*VOICE_REPORT_FRAME_SIZE]= {0,1,23};

#if VOICE_MTU_SIZE_FIXED_20_BYTES

uint8 VoiceSend_SubIndex=0;


#endif



/**
 * @brief init voice queue
 * @param uint8 * voice_buf--Buf For queue
 * @return none
 */
void InitQueue(void)
{
    VoiceQueue.queuesize = VOICE_QUEUE_MAX_LENGTH;
    VoiceQueue.SendIdx   = 0;
    VoiceQueue.StoreIdx  = 0;
    VoiceQueue.VoicePackageSN = 0;
    //VoiceQueue.StoreCnt  = 0;
    VoiceQueue.VoiceFlg  = TRUE;
    VoiceQueue.VoiceSendFlg = FALSE;
    VoiceQueue.ReleaseSendFlg = FALSE;
    VoiceQueue.ReleaseTimerFlg = FALSE;
    VoiceQueue.VoiceQueue = (uint8 *)Voicebuf;
}



/**
 * @brief in voice queue
 * @param buffer - buffer data to be stored
 * @return none
 */
void InQueue(uint8 *buffer)
{
    uint32 Tail;
#if VOICE_MTU_SIZE_FIXED_20_BYTES
    Tail = (VoiceQueue.StoreIdx + 1) % VoiceQueue.queuesize;

    if (Tail == VoiceQueue.SendIdx)
    {
        LOG("Voice Queue is full\n\r");
    }
    else
    {

        osal_memcpy(&VoiceQueue.VoiceQueue[VoiceQueue.StoreIdx * VOICE_REPORT_FRAME_SIZE],buffer, VOICE_REPORT_FRAME_SIZE);
        VoiceQueue.StoreIdx = Tail;
        VoiceQueue.VoicePackageSN++;
        //LOG("Voice InQueue\n\r");
    }



#else


#endif
}

/**
 * @brief out voice queue
 * @param uint8 * send_buf
 * @return none
 */
uint8 OutQueue(uint8 * send_buf)
{
    if (VoiceQueue.StoreIdx == VoiceQueue.SendIdx)
    {
        LOG("Voice Queue is empty\n\r");
#if VOICE_FROM_FLASH
        VoiceQueue.SendIdx=0; //teddy add for test，set data transf continue
#endif
        return 1;
    }
    else
    {
        //SendData
        //ProfileAPI_SendData(gHIDServiceId, GATT_SRV_HID_VENDOR_INPUT_INDEX_2, VoiceQueue.VoiceQueue + VoiceQueue.SendIdx * VOICE_REPORT_FRAME_SIZE, VOICE_REPORT_FRAME_SIZE);
        osal_memcpy(send_buf, VoiceQueue.VoiceQueue+VoiceQueue.SendIdx * VOICE_REPORT_FRAME_SIZE, VOICE_REPORT_FRAME_SIZE);
        VoiceQueue.SendIdx = (VoiceQueue.SendIdx + 1) % VoiceQueue.queuesize;
        return 0;
    }
}

/**
 * @brief check if voice queue is full
 * @param none
 * @return true or false
 */
uint8 IsQueueFull(void)
{
    if ((VoiceQueue.StoreIdx + 1) % VoiceQueue.queuesize == VoiceQueue.SendIdx)
    {
        //DBG_BUFFER(MODULE_DRIVERTASK, LEVEL_INFO, "Voice Queue is full.",0);
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief check if voice queue is empty
 * @param none
 * @return true or false
 */
uint8 IsQueueEmpty(void)
{
    if (VoiceQueue.StoreIdx == VoiceQueue.SendIdx)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
