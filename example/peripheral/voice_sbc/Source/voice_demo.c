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
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "adc.h"
#include "OSAL.h"
#include "adc_demo.h"
#include "voice_demo.h"
#include "log.h"
#include "voice.h"
#include "error.h"
#include "stdio.h"
#include "types.h"

/*#include "sbc_tables.h"
#include "sbc_math.h"
#include "sbc_primitives.h"
#include "sbc_private.h"*/
#include "sbc.h"


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
#define    MAX_VOICE_BUF_SIZE		24576
#define		 MAX_VOICE_FIFO_MSBC_SIZE		128*15


static uint8 voiceDemo_TaskID;   // Task ID for internal task/event processing

static int voiceWriteID;

static uint32_t voiceBuf[MAX_VOICE_BUF_SIZE];

static int voiceBufIndex;

// ADPCM parameters
static int indexTable[16] = {
-1, -1, -1, -1, 2, 4, 6, 8,
-1, -1, -1, -1, 2, 4, 6, 8,
};

static int stepsizeTable[89] = {
7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

// Use for IMA-ADPCM stereo
static int predictedSampleRight;
static int indexRight;
static int predictedSampleLeft;
static int indexLeft;

// User for mSBC
static uint16_t voiceFIFOmSBC[MAX_VOICE_FIFO_MSBC_SIZE];
static int voiceFIFOmSBCWriteIndex;
static int voiceFIFOmSBCReadIndex;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void voice_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void voiceCaptureTask( void );
static void voiceStopTask( void );
static uint8_t voice_adpcm_encoder(int currentSample, int *predictedSample, int *index);

/*********************************************************************
 * PROFILE CALLBACKS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */



void voice_Init( uint8 task_id )
{
  voiceDemo_TaskID = task_id;
	
	voiceWriteID = 0;
	
	voiceBufIndex = 0;
	
	predictedSampleRight = 0;
	indexRight = 0;
	predictedSampleLeft = 0;
	indexLeft = 0;
	
	voiceFIFOmSBCWriteIndex = 0;
	voiceFIFOmSBCReadIndex = 0;
	
	voiceCaptureTask();
	 
}


uint16 voice_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  LOG("voice_ProcessEvent: 0x%x\n",events);
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( voiceDemo_TaskID )) != NULL )
    {
      voice_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
 
/*  if ( events & 0x20 )
  {
    // Perform periodic heart rate task
    LOG("20\n");
    osal_start_timerEx( voiceDemo_TaskID, 0x20, 2000);
    
    return (events ^ 0x20);
  }  
*/
  if ( events == voiceCaptureTask_EVT )
  {
    LOG("voiceCaptureTask_EVT\n");
    voiceCaptureTask();
    
    return (events ^ voiceCaptureTask_EVT);
  }  
	
	if ( events == voiceStopTask_EVT )
  {
    // Perform periodic heart rate task
    LOG("voiceStopTask_EVT\n");
    voiceStopTask();
    
    return (events ^ voiceStopTask_EVT);
  }  
  
  // Discard unknown events
  return 0;
}


static void voice_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
}

static void voice_evt_handler(voice_Evt_t *pev)
{
  if(pev->type == HAL_VOICE_EVT_DATA){
		LOG("Type matched\n");
    if (hal_uart_get_tx_ready() == PPlus_SUCCESS) {
			LOG("UART ready\n");
			//if (hal_uart_send_buff((uint8_t *)pev->data, 4*(pev->size)) == PPlus_SUCCESS) {
				voiceWriteID++;
				LOG("Voice data transmitted successfully:");
				LOG("%d\n", voiceWriteID);
			//}
		}				
  }
//	LOG("Voice data transmitted successfully\n");	
}


static void voice_evt_handler_buf(voice_Evt_t *pev)
{
  if(pev->type == HAL_VOICE_EVT_DATA){
//		LOG("Type matched\n");
    int n;
		for (n = 0; n < pev->size; n++) {
			voiceBuf[voiceBufIndex] = pev->data[n];
			voiceBufIndex++;
		}
		voiceWriteID++;
		LOG("Voice data saved successfully:");
		LOG("%d %d\n", voiceWriteID, pev->data[pev->size-1]);
	}		
}

static void voice_evt_handler_adpcm(voice_Evt_t *pev)
{
  if(pev->type == HAL_VOICE_EVT_DATA){
    int n;
		int m;
		uint32_t voiceSampleDual;
		int voiceSampleRight;
		int voiceSampleLeft;
		uint8_t voiceAdpcmRight;
		uint8_t voiceAdpcmLeft;
		uint16_t voiceAdpcm4Right;
		uint16_t voiceAdpcm4Left;
		
		n = 0;
		m = 0;
		voiceAdpcm4Right = 0;
		voiceAdpcm4Left = 0;
//		LOG("%d %d\n", predictedSampleRight, predictedSampleLeft);
		while (n < pev->size) {
			voiceSampleDual = pev->data[n];
			n++;
			voiceSampleRight = (int16)(voiceSampleDual & 65535);
			voiceSampleLeft = (int16)((voiceSampleDual >> 16) & 65535);
			voiceAdpcmRight = voice_adpcm_encoder(voiceSampleRight, &predictedSampleRight, &indexRight);
			voiceAdpcmLeft = voice_adpcm_encoder(voiceSampleLeft, &predictedSampleLeft, &indexLeft);
			voiceAdpcm4Right |= voiceAdpcmRight << (12-m*4);
			voiceAdpcm4Left |= voiceAdpcmLeft << (12-m*4);
			m++;
			if (m>3) {
				voiceBuf[voiceBufIndex] = (voiceAdpcm4Left << 16) | voiceAdpcm4Right;
				voiceBufIndex++;
				voiceAdpcm4Right = 0;
				voiceAdpcm4Left = 0;
				m = 0;
			}
		}	
		voiceWriteID++;
//		LOG("Voice data saved successfully:");
//		LOG("%d %x %d %d %d\n", voiceWriteID, voiceSampleDual, voiceSampleRight, predictedSampleRight, indexRight);
	}		
}

// ADPCM voice encoder based on IMA-ADPCM algorithm
static uint8_t voice_adpcm_encoder(int currentSample, int *predictedSample, int *index)
{			
	uint8_t delta;	/* Current adpcm output value */
	int diff;				/* Difference between currentSample and predictedSample */
	int step;				/* Stepsize */
	int diffPredict; /* Current change to predictedSample */
	
	step = stepsizeTable[*index];
	
	// Step 1 - compute difference with previous value 
	diff = currentSample - *predictedSample;
	if(diff < 0) {
		delta = 8;
		diff = -diff;
	} 
	else {
		delta = 0;
	}
	//uint8_t delta0 = delta;
	// Step 2 - Divide and clamp 
	// This code *approximately* computes: delta = diff*4/step; diffPredict = (delta+0.5)*step/4;
	diffPredict = (step >> 3);
	if ( diff >= step ) {
		delta |= 4;
		diff -= step;
		diffPredict += step;
	}
	step >>= 1;
	if ( diff >= step ) {
		delta |= 2;
		diff -= step;
		diffPredict += step;
	}
	step >>= 1;
	if ( diff >= step ) {
		delta |= 1;
		diffPredict += step;
	}
	//Step 3 - Update previous value & clamp previous value to 16 bits 
	if ( (delta & 8) != 0 ) {
		*predictedSample -= diffPredict;
		if ( *predictedSample < -32768 )
			*predictedSample = -32768;
	}
	else {
		*predictedSample += diffPredict;
		if ( *predictedSample > 32767 )
			*predictedSample = 32767;
	}
	// Step 4 - Assemble value, update index and step values
	*index += indexTable[delta];
	if ( *index < 0 ) 
		*index = 0;
	else if ( *index > 88 ) 
		*index = 88;
	
	// Step 5 - Output value
	return delta;
}


static void voice_evt_handler_sbc(voice_Evt_t *pev)
{
  if(pev->type == HAL_VOICE_EVT_DATA){
//		LOG("Type matched\n");
    int n;
		for (n = 0; n < pev->size; n++) {
			voiceFIFOmSBC[voiceFIFOmSBCWriteIndex] = (uint16_t)pev->data[n];
			voiceFIFOmSBCWriteIndex++;
		}
		
		sbc_t sbc;
		int size, srate, codesize, nframes;
		ssize_t encoded;
		ssize_t len;
		
		sbc_init_msbc(&sbc, 0);
		sbc.endian = SBC_BE;

		codesize = sbc_get_codesize(&sbc);
		nframes = (voiceFIFOmSBCWriteIndex - voiceFIFOmSBCReadIndex) / codesize;
		
		while (1) {
			unsigned char *inp, *outp;
			
			/* read data for up to 'nframes' frames of input data */
			size = codesize*nframes;
			
			if (size < 0) {
				/* Something really bad happened */
				LOG("Can't read audio data");
				break;
			}
			if (size < codesize) {
				/* Not enough data for encoding even a single frame */
				break;
			}
			
			/* encode all the data from the input buffer in a loop */
			inp = (unsigned char*)&voiceFIFOmSBC[voiceFIFOmSBCReadIndex];
			outp = (unsigned char*)&voiceBuf[voiceBufIndex];
			while (size >= codesize) {
				len = sbc_encode(&sbc, inp, codesize, outp, MAX_VOICE_BUF_SIZE-voiceBufIndex+1, &encoded);
				if (len != codesize || encoded <= 0) {
					LOG("sbc_encode fail, len=%zd, encoded=%lu\n", len, (unsigned long) encoded);
					break;
				}
				size -= len;
				inp += len;
				outp += encoded;
				voiceFIFOmSBCReadIndex += codesize/2;
				voiceBufIndex += encoded/4;
			}
/*			len = write(fileno(stdout), output, outp - output);
			if (len != outp - output) {
				LOG("Can't write SBC output");
				break;
			}
*/
			if (size != 0) {
				/*
				* sbc_encode failure has been detected earlier or end
				* of file reached (have trailing partial data which is
				* insufficient to encode SBC frame)
				*/
				break;
			}
		}

		sbc_finish(&sbc);
		
//		LOG("Voice data saved successfully:");
//		LOG("%d %d\n", voiceWriteID, pev->data[pev->size-1]);
	}		
}

static void voiceCaptureTask( void )
{		
  voice_Cfg_t cfg;
	cfg.voiceSelAmicDmic = 1;
	cfg.dmicDataPin = P4;
	cfg.dmicClkPin = P5;
	cfg.amicGain = 0;
	cfg.voiceGain = 40;
	cfg.voiceEncodeMode = VOICE_ENCODE_BYP;
	cfg.voiceRate = VOICE_RATE_8K;
	cfg.voiceAutoMuteOnOff = 1;
	
  volatile int voiceConfigStatus = hal_voice_config(cfg, voice_evt_handler_adpcm);
  if(voiceConfigStatus) {
		LOG("Voice configuration failed\n");
    return;
	}
	else {
		LOG("Voice configuration succeeded\n");
	}

	
	LOG("Voice capture started\n");
	
	
 // Restart timer
 osal_start_timerEx( voiceDemo_TaskID, voiceStopTask_EVT, 10000 );
 
 hal_voice_start();
}

static void voiceStopTask( void )
{      
  hal_voice_stop();
	
	LOG("Voice capture stopped\n");
	LOG("Voice data word saved:");
	LOG("%d\n", voiceBufIndex);
	
	if (hal_uart_get_tx_ready() == PPlus_SUCCESS) {
		int voiceTxmitIndex;
		voiceTxmitIndex = 0;
		while (voiceTxmitIndex < voiceBufIndex) {
			LOG("%x\n",voiceBuf[voiceTxmitIndex]);
			voiceTxmitIndex++;			
		}
		LOG("Voice data byte transmitted:");
		LOG("%d\n", voiceTxmitIndex);
		
//		LOG("UART ready\n");
//		if (hal_uart_send_buff((uint8_t *)voiceBuf, 4*voiceBufIndex) == PPlus_SUCCESS) {

		LOG("Voice data buffer transmitted successfully");
	}
		
 // hal_voice_start();
 // Restart timer
// osal_start_timerEx( voiceDemo_TaskID, voiceCaptureTask_EVT, 2000 );
}


/*********************************************************************
*********************************************************************/
