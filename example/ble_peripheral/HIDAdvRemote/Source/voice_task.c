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

/**************************************************************************************************
  Filename:       voice_task.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "adc.h"
#include "OSAL.h"
//#include "adc_demo.h"
#include "voice_task.h"
#include "log.h"
#include "voice.h"
#include "error.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "voice_circ_buff.h"
#include "hidkbd.h"


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

#define SET_LEFT_VOICE  1

#if VOICE_FROM_FLASH
#define    MAX_VOICE_BUF_SIZE		 2//36000
#define    VOICE_RAW_POST_SIZE       1//12000 //200ms

#else
#define    MAX_VOICE_BUF_SIZE		   6000//18000//36000
#define    VOICE_RAW_POST_SIZE     3000 // 6000//12000 //200ms


#endif

tCircularBuffer * VoiceRaw_FiFO;

static uint8_t voiceLeftBuf[MAX_VOICE_BUF_SIZE];



 uint8 voice_TaskID;   // Task ID for internal task/event processing

 uint8 voice_triggle_flag=0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void voice_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void voiceCaptureInit( void );
static void voiceCaptureStart(void);
static void voiceCaptureStop(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */



void voice_task_Init( uint8 task_id )
{
  voice_TaskID = task_id;

 // voiceCaptureInit();

  VoiceRaw_FiFO=(tCircularBuffer*)malloc(sizeof(tCircularBuffer));

}


uint16 voice_task_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  LOG("voice_task_ProcessEvent: 0x%x\n\r",events);
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( voice_TaskID )) != NULL )
    {
      voice_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
 

  if ( events &  VOICE_RECORD_START_EVT )
  {
    LOG("VOICE_RECORD_START_EVT\n\r");
    voiceCaptureStart();
    return (events ^ VOICE_RECORD_START_EVT);
  }  
	
	if ( events & VOICE_RECORD_STOP_EVT )
  {
    
    LOG("voiceCaptureStop\n\r");
    voiceCaptureStop();
    
    return (events ^ VOICE_RECORD_STOP_EVT);
  }  
	
  if(events &VOICE_TRANSF_ENCODE_EVT)
  {
    osal_set_event(hidKbdTaskId, VOICE_PROCESS_EVT);
    LOG("VOICE_TRANSF_ENCODE_EVT\n\r");
    return (events ^ VOICE_TRANSF_ENCODE_EVT);
  }
 
  return 0;
}


static void voice_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
   switch ( pMsg->event )
  {
      
    default:
      break;
  }
}


//output raw data
static void voice_evt_handler_adpcm(voice_Evt_t *pev)
{
  uint8_t leftbuf[2];
  uint8_t rightbuf[2];
  uint8_t left_right_chanle;

  uint32_t voiceSampleDual;
  int voiceSampleRight;
  int voiceSampleLeft;

  uint32_t i=0;

  left_right_chanle=SET_LEFT_VOICE;
 
  if(pev->type == HAL_VOICE_EVT_DATA)
  {

	for(i=0;i < pev->size;i++) 
	{
		voiceSampleDual = pev->data[i];

		voiceSampleRight = (int16)(voiceSampleDual & 65535);
		voiceSampleLeft = (int16)((voiceSampleDual >> 16) & 65535);
		
		if(left_right_chanle==1)
		{
     leftbuf[0]= voiceSampleLeft;
		 leftbuf[1]= voiceSampleLeft>>8;
		 FillBuffer(VoiceRaw_FiFO, leftbuf, 2);
		}
		else
		{
		 rightbuf[0]= voiceSampleRight;
		 rightbuf[1]= voiceSampleRight>>8;
		 FillBuffer(VoiceRaw_FiFO, rightbuf, 2);
		}
	}
 
	if(VoiceRaw_FiFO->postFlag==0)
	{
	  //LOG("voice_evt_handler_adpcm\n\r");
	if(IsBufferVacant(VoiceRaw_FiFO,VoiceRaw_FiFO->TransThreshold_size))
	{
	VoiceRaw_FiFO->postFlag=1;
	osal_set_event(voice_TaskID, VOICE_TRANSF_ENCODE_EVT);
	}
	  	
	}

 }

  
}


static void voiceCaptureInit( void )
{	
  LOG("Voice capture init\n");	
  voice_Cfg_t cfg;
	cfg.voiceSelAmicDmic = 1;
	cfg.dmicDataPin = P4;
	cfg.dmicClkPin = P5;
	cfg.amicGain = 0;
	cfg.voiceGain = 36;//40;//40--0db£¬(v-40)*0.5db
	cfg.voiceEncodeMode = VOICE_ENCODE_BYP;
	cfg.voiceRate = VOICE_RATE_16K;//VOICE_RATE_8K
	cfg.voiceAutoMuteOnOff = 1;
	
  volatile int voiceConfigStatus = hal_voice_config(cfg, voice_evt_handler_adpcm);
  
  if(voiceConfigStatus) {
		LOG("Voice configuration failed\n");
    return;
	}
	else {
		LOG("Voice configuration succeeded\n");
	}

}

static void voiceCaptureStart( void )
{
  voiceCaptureInit();
  InitCircularBuffer(VoiceRaw_FiFO,voiceLeftBuf,MAX_VOICE_BUF_SIZE,VOICE_RAW_POST_SIZE);
  hal_voice_start();
  
}


static void voiceCaptureStop( void )
{      
  hal_voice_stop();
  InitCircularBuffer(VoiceRaw_FiFO,voiceLeftBuf,MAX_VOICE_BUF_SIZE,VOICE_RAW_POST_SIZE);
}


uint8 voice_requeset_data(uint8* buf,uint16 len)
{
      if(len==0)
	  	return 0;
	  
      if(IsBufferSizeFilled(VoiceRaw_FiFO,len))
      {
         if(ReadBuffer(VoiceRaw_FiFO,buf,len)==len)
         return 1;


	  }
 
    return 0;
}


void triggle_voice_capture(void)
{
  if(voice_triggle_flag==0)
	{
	 voiceCaptureStart();
		voice_triggle_flag=1;
	}
}

/*********************************************************************
*********************************************************************/
