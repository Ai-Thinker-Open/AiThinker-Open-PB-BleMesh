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

#ifndef VOICE_QUEUE_H
#define VOICE_QUEUE_H

#include "types.h"

#define VOICE_QUEUE_MAX_LENGTH 100 //300

#define VOICE_MTU_SIZE_FIXED_20_BYTES 1
#define VOICE_REPORT_FRAME_SIZE 100

#define VOICE_BLESEND_FRAME_SIZE 20




typedef struct queue
{
    uint8 VoiceFlg;
    uint8 VoiceSendFlg; //voice buffered to send
    uint8 ReleaseSendFlg; //release key buffered to send
    uint8 ReleaseTimerFlg;//timer to process abnormal flg.

    uint32 VoicePackageSN;
    uint32 queuesize;
    uint32 SendIdx;    //head
    uint32 StoreIdx;   //tail
    uint8 *VoiceQueue;
} Queue;

extern Queue VoiceQueue;

#if VOICE_MTU_SIZE_FIXED_20_BYTES 

extern uint8 VoiceSend_SubIndex;


#endif


extern void  InitQueue(void);
extern void InQueue(uint8 *buffer);
extern uint8 OutQueue(uint8 * send_buf);
extern uint8 IsQueueFull(void);
extern uint8 IsQueueEmpty(void);




#endif
