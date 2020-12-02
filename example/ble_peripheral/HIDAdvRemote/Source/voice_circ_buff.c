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
// voice_circ_buff.c
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup wifi_audio_app
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//App include
#include "voice_circ_buff.h"
//*****************************************************************************
//
//!  Initializing the Circular Buffer
//!
//! \param ulBufferSize is the size allocated to the circular buffer.
//! \param userbuf is a Array,user has defined .
//! This function  
//!       
//!     2. Initializes the control structure for circular buffer.
//!
//! \return pointer to the Circular buffer Control Structure.
//
//*****************************************************************************
tCircularBuffer*
InitCircularBuffer(tCircularBuffer*pTempBuff, unsigned char *userbuf,unsigned long ulBufferSize,unsigned short threshold_size)
{
  //  tCircularBuffer *pTempBuff;
    pTempBuff->postFlag=0;
	  pTempBuff->TransThreshold_size=threshold_size;
    pTempBuff->pucBufferStartPtr = userbuf;
    pTempBuff->pucReadPtr = pTempBuff->pucBufferStartPtr;
    pTempBuff->pucWritePtr = pTempBuff->pucBufferStartPtr;
    pTempBuff->ulBufferSize = ulBufferSize;
    pTempBuff->pucBufferEndPtr = (pTempBuff->pucBufferStartPtr + ulBufferSize);

	memset(userbuf,0,ulBufferSize);
    return(pTempBuff);
}

//*****************************************************************************
//
//! Creating and Initializing the Circular Buffer
//!
//! \param ulBufferSize is the size allocated to the circular buffer.
//!
//! This function  
//!        1. Allocates memory for the circular buffer.
//!     2. Initializes the control structure for circular buffer.
//!
//! \return pointer to the Circular buffer Control Structure.
//
//*****************************************************************************
tCircularBuffer*
CreateCircularBuffer(unsigned long ulBufferSize)
{
    tCircularBuffer *pTempBuff;
    pTempBuff = (tCircularBuffer*)malloc(sizeof(tCircularBuffer));
    if(pTempBuff == NULL)
    {
        return NULL;
    }

    pTempBuff->pucBufferStartPtr = (unsigned char*)malloc(ulBufferSize);
    pTempBuff->pucReadPtr = pTempBuff->pucBufferStartPtr;
    pTempBuff->pucWritePtr = pTempBuff->pucBufferStartPtr;
    pTempBuff->ulBufferSize = ulBufferSize;
    pTempBuff->pucBufferEndPtr = (pTempBuff->pucBufferStartPtr + ulBufferSize);
    return(pTempBuff);
}

//*****************************************************************************
//
//! Creating and Initializing the Circular Buffer
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//!
//! This function  
//!    1. free memory allocated to the the circular buffer and its control
//!        structure.
//!
//! \return none
//
//*****************************************************************************
void
DestroyCircularBuffer(tCircularBuffer *pCircularBuffer)
{
    if(pCircularBuffer->pucBufferStartPtr)
    {
        free(pCircularBuffer->pucBufferStartPtr);
    }

    free(pCircularBuffer);
}

//*****************************************************************************
//
//! return the current value of the read pointer.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//!
//! \return current value of the read pointer.
//
//*****************************************************************************
unsigned char*
GetReadPtr(tCircularBuffer *pCircularBuffer)
{
    return(pCircularBuffer->pucReadPtr);
}

//*****************************************************************************
//
//! return the current value of the write pointer.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//!
//! \return current value of the write pointer.
//
//*****************************************************************************
unsigned char*
GetWritePtr(tCircularBuffer *pCircularBuffer)
{
    return(pCircularBuffer->pucWritePtr);
}

//*****************************************************************************
//
//! fills the one buffer by the content of other buffer.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param pucBuffer is the pointer to the buffer from where the buffer is
//!        filled.
//! \param uiPacketSize is the amount of data need to be copied from pucBuffer
//!        to the Circular Buffer.
//!
//! This function  
//!        1. Copies the uiPacketSize of data from pucBuffer to the 
//!        pCircularBuffer.
//!
//! \return amount of data copied.
//
//*****************************************************************************
long
FillBuffer(tCircularBuffer *pCircularBuffer, unsigned char *pucBuffer,
           unsigned int uiPacketSize)
{
    int uiOffset;
    uiOffset=((pCircularBuffer->pucWritePtr+uiPacketSize) -
              pCircularBuffer->pucBufferEndPtr);

    if(IsBufferVacant(pCircularBuffer,pCircularBuffer->ulBufferSize))
    {
        if(uiOffset <= 0)
        {
            memcpy(pCircularBuffer->pucWritePtr, pucBuffer, uiPacketSize);
        }
        else
        {
            memcpy(pCircularBuffer->pucWritePtr, pucBuffer,
                   (uiPacketSize - uiOffset));
            memcpy(pCircularBuffer->pucBufferStartPtr,
                   ((pucBuffer + uiPacketSize) - uiOffset), uiOffset);
        }
        UpdateWritePtr(pCircularBuffer, uiPacketSize);
        return(uiPacketSize);
    }
    else
    {
        return(-1);
    }
}

//*****************************************************************************
//
//! updates the read pointer.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param uiDataSize is the amount by which the read pointer is to be
//!        increased.
//!
//! This function  
//!        1. increase the read pointer according to the parameters.
//!
//! \return none.
//
//*****************************************************************************
void
UpdateReadPtr(tCircularBuffer *pCircularBuffer, unsigned int uiDataSize)
{
    int uiOffset = 0;

    uiOffset = ((pCircularBuffer->pucReadPtr + uiDataSize) -
                pCircularBuffer->pucBufferEndPtr);
    if(uiOffset <= 0)
    {
        pCircularBuffer->pucReadPtr = (pCircularBuffer->pucReadPtr +
                                        uiDataSize);
        if(pCircularBuffer->pucReadPtr == pCircularBuffer->pucBufferEndPtr)
        {
            pCircularBuffer->pucReadPtr = pCircularBuffer->pucBufferStartPtr;
        }
    }
    else
    {
        pCircularBuffer->pucReadPtr = (pCircularBuffer->pucBufferStartPtr +
                                        uiOffset);
    }

}

//*****************************************************************************
//
//! updates the write pointer.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param uiDataSize is the amount by which the write pointer is to be
//!        increased.
//!
//! This function  
//!        1. increase the write pointer according to the parameters.
//!
//! \return none.
//
//*****************************************************************************
void
UpdateWritePtr(tCircularBuffer *pCircularBuffer, unsigned int uiPacketSize)
{
    int uiOffset = 0;

    uiOffset = ((pCircularBuffer->pucWritePtr + uiPacketSize) -
                pCircularBuffer->pucBufferEndPtr);
    if(uiOffset <= 0)
    {
        pCircularBuffer->pucWritePtr = (pCircularBuffer->pucWritePtr +
                                        uiPacketSize);
        if(pCircularBuffer->pucWritePtr == pCircularBuffer->pucBufferEndPtr)
        {
            pCircularBuffer->pucWritePtr = pCircularBuffer->pucBufferStartPtr;
        }
    }
    else
    {
        pCircularBuffer->pucWritePtr = (pCircularBuffer->pucBufferStartPtr +
                                        uiOffset);
    }
}

//*****************************************************************************
//
//! reads the content of one buffer into the other.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param pucBuffer is the pointer to the buffer to which the content is
//!        copied.
//! \param uiPacketSize is the amount of data need to be copied from Circular
//!        Buffer to the pucBuffer.
//!
//! This function  
//!        1. Copies the uiPacketSize of data from pCircularBuffer to the 
//!        pucBuffer.
//!
//! \return amount of data copied.
//
//*****************************************************************************
long
ReadBuffer(tCircularBuffer *pCircularBuffer, unsigned char *pucBuffer,
           unsigned int uiDataSize)
{
    int uiOffset;

    uiOffset=((pCircularBuffer->pucReadPtr+uiDataSize) -
              pCircularBuffer->pucBufferEndPtr);

    if(uiOffset <= 0)
    {
        memcpy(pucBuffer, pCircularBuffer->pucReadPtr, uiDataSize);
    }
    else
    {
        memcpy(pucBuffer, pCircularBuffer->pucReadPtr,
                   (uiDataSize - uiOffset));
        memcpy(((pucBuffer + uiDataSize) - uiOffset),
                   pCircularBuffer->pucBufferStartPtr, uiOffset);
    }
    UpdateReadPtr(pCircularBuffer, uiDataSize);
    return(uiDataSize);
}

//*****************************************************************************
//
//! fills the buffer with zeores.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param uiPacketSize is the amount of space needed to be filled with zeroes.
//!
//! This function  
//!        1. fill uiPacketSize amount of memory with zeroes.
//!
//! \return amount of zeroes filled.
//
//*****************************************************************************
long
FillZeroes(tCircularBuffer *pCircularBuffer, unsigned int uiPacketSize)
{
    memset(pCircularBuffer->pucWritePtr, 0, uiPacketSize);
    UpdateWritePtr(pCircularBuffer, uiPacketSize);
    return(uiPacketSize);
}

//****************************************************************************
//
//! checks how much buffer is filled.
//! 
//! \param pCircularBuffer is a pointer to the control structure for Circular.
//!    Buffer.
//!
//! This function  
//!        1. Retreives the amount of Buffer filled.
//!
//! \return the amount of buffer filled.
//
//****************************************************************************
unsigned int
GetBufferSize(tCircularBuffer *pCircularBuffer)
{
    unsigned int uiBufferFilled;
    if(pCircularBuffer->pucReadPtr <= pCircularBuffer->pucWritePtr)
    {
        uiBufferFilled = (pCircularBuffer->pucWritePtr -
                          pCircularBuffer->pucReadPtr);
    }
    else
    {
        uiBufferFilled = ((pCircularBuffer->pucWritePtr -
                           pCircularBuffer->pucBufferStartPtr) +
                          (pCircularBuffer->pucBufferEndPtr -
                           pCircularBuffer->pucReadPtr));
    }

    return uiBufferFilled;
}

//****************************************************************************
//
//! checks how much buffer is Empty.
//! 
//! \param pCircularBuffer is a pointer to the control structure for Circular.
//!    Buffer.
//!
//! This function  
//!        1. Retreives the amount of Buffer Empty.
//!
//! \return the amount of buffer Empty.
//
//****************************************************************************
unsigned int
GetBufferEmptySize(tCircularBuffer *pCircularBuffer)
{
    unsigned int uiBufferFilled;
    uiBufferFilled = GetBufferSize(pCircularBuffer);
    return (pCircularBuffer->ulBufferSize - uiBufferFilled);
}

//*****************************************************************************
//
//! check if the buffer is empty or not.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//!
//! This function  
//!        1. check if the buffer is empty or not.
//!
//! \return true if buffer is empty othr wise returns false.
//
//*****************************************************************************
tboolean
IsBufferEmpty(tCircularBuffer *pCircularbuffer)
{
    if(pCircularbuffer->pucReadPtr == pCircularbuffer->pucWritePtr)
    {
        return(TRUE);
    }
    else
    {
        return(FALSE);
    }
}

//*****************************************************************************
//
//! check if the specified amount of buffer is filled or not.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param ulsize is the amount of data which is to checked. 
//!
//! This function  
//!  1. check if the buffer is filled with specified amount of data or not.
//!
//! \return true if buffer is filled with specifed amount of data or more,
//!         otherwise false.
//
//*****************************************************************************
tboolean
IsBufferSizeFilled(tCircularBuffer *pCircularBuffer, unsigned long ulSize)
{
    unsigned int uiBufferFilled;
    uiBufferFilled = GetBufferSize(pCircularBuffer);
    if(uiBufferFilled >= ulSize)
    {
        return(TRUE);
    }
    return(FALSE);
}

//*****************************************************************************
//
//! check if the buffer is filled less than the specified amount or not.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param ulsize is the amount of data which is to checked.
//!
//! This function  
//!  1. checks if the buffer is filled less than the specified amount or not.
//!
//! \return true if buffer is filled less than the specifed amount, otherwise
//!         false.
//
//*****************************************************************************
tboolean
IsBufferVacant(tCircularBuffer *pCircularBuffer, unsigned long ulSize)
{
    unsigned int uiBufferFilled;
    uiBufferFilled = GetBufferSize(pCircularBuffer);
    if(uiBufferFilled <= ulSize)
    {
        return(TRUE);
    }
    return(FALSE);
}
//*****************************************************************************
//
//! check if the buffer is filled less than the specified amount or not.
//!
//! \param pCircularBuffer is the pointer to the control structure for circular
//!        buffer.
//! \param ulsize is the amount of data which is to checked.
//!
//! This function  
//!  1. checks if the buffer is filled less than the specified amount or not.
//!
//! \return true if buffer is filled less than the specifed amount, otherwise
//!         false.
//
//*****************************************************************************
tboolean
IsBufferOverflow(tCircularBuffer *pCircularBuffer, unsigned long ulSize)
{
    unsigned int uiBufferFilled;
    uiBufferFilled = GetBufferSize(pCircularBuffer);
    if(uiBufferFilled + ulSize >= pCircularBuffer->ulBufferSize)
    {
        return(TRUE);
    }
    return(FALSE);
}

//*****************************************************************************
//*****************************************************************************

