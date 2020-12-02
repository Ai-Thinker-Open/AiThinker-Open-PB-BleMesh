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

//*****************************************************************************
// voice_circ_buff.h
//

//*****************************************************************************

#ifndef __CIRCULAR_BUFFER_API_H__
#define __CIRCULAR_BUFFER_API_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
  
/****************************************************************************/
/*				TYPEDEFS										*/
/****************************************************************************/
typedef struct CircularBuffer
{   
	  unsigned char  postFlag;
    unsigned short TransThreshold_size;
    unsigned char *pucReadPtr;
    unsigned char *pucWritePtr;
    unsigned char *pucBufferStartPtr;
    unsigned long ulBufferSize;
    unsigned char *pucBufferEndPtr;
}tCircularBuffer;

#ifndef TRUE
#define TRUE                    1
#endif

#ifndef FALSE
#define FALSE                   0
#endif

//*****************************************************************************
//
// Define a boolean type, and values for true and false.
//
//*****************************************************************************
typedef unsigned int tboolean;

/****************************************************************************/
/*		        FUNCTION PROTOTYPES							*/
/****************************************************************************/
extern tCircularBuffer *InitCircularBuffer(tCircularBuffer*pTempBuff, unsigned char *userbuf,unsigned long ulBufferSize,unsigned short threshold_size);
extern tCircularBuffer* CreateCircularBuffer(unsigned long ulBufferSize);
extern void DestroyCircularBuffer(tCircularBuffer *pCircularBuffer);
extern unsigned char* GetReadPtr(tCircularBuffer *pCircularBuffer);
extern unsigned char* GetWritePtr(tCircularBuffer *pCircularBuffer);
extern long FillBuffer(tCircularBuffer *pCircularBuffer,
                       unsigned char *pucBuffer, unsigned int uiBufferSize);
extern void UpdateReadPtr(tCircularBuffer *pBuffer, unsigned int uiDataSize);
extern void UpdateWritePtr(tCircularBuffer *pCircularBuffer,
                           unsigned int uiPacketSize);
extern long ReadBuffer(tCircularBuffer *pCircularBuffer,
                       unsigned char *pucBuffer, unsigned int uiDataSize);
extern long FillZeroes(tCircularBuffer *pCircularBuffer,
					   unsigned int uiPacketSize);
extern unsigned int GetBufferSize(tCircularBuffer *pCircularBuffer);
extern unsigned int GetBufferEmptySize(tCircularBuffer *pCircularBuffer);
extern tboolean IsBufferEmpty(tCircularBuffer *pCircularbuffer);
extern tboolean IsBufferSizeFilled(tCircularBuffer *pCircularBuffer,
                             unsigned long ulThresholdHigh);
extern tboolean IsBufferVacant(tCircularBuffer *pCircularBuffer,
                               unsigned long ulThresholdLow);

#ifdef __cplusplus
}
#endif

#endif // __CIRCULAR_BUFFER_API_H__

