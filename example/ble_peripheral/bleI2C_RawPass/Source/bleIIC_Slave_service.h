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

/*******************************************************************************
* @file			bleIIC_Slave service.h
* @brief		Contains the bleIIC Slave service definitions and prototypes
* @version	1.0
* @date			2018/7/5  
* @author		Rugooo
* 
* Copyright(C) 2016, PhyPlus Semiconductor All rights reserved.
*******************************************************************************/
#ifndef __BLEIIC_SLAVE_SERVICE_H__
#define __BLEIIC_SLAVE_SERVICE_H__

/*******************************************************************************
*@ Module    			:  Pre-Compiler
*@ Description    :  NULL
*******************************************************************************/
#ifdef _BLE_IIC_SLAVE_SERVICE_CMD_
	#define BLEIIC_SS_Ext	
#else
	#define BLEIIC_SS_Ext extern
#endif

/*******************************************************************************
*@ Module    			:  Includes
*@ Description    :  None
*******************************************************************************/
#include "att.h"

/*******************************************************************************
*@ Module    			:  Macro Define
*@ Description    :  
*******************************************************************************/
// Profile Parameters
#define PROFILE_RAWPASS_CHAR_RX                   	0  // RW uint8 - Profile Characteristic 1 value 
#define PROFILE_RAWPASS_CHAR_TX                   	1  // RW uint8 - Profile Characteristic 2 value
// Simple Keys Profile Services bit fields
#define PROFILE_RAWPASS_SERVICE               		0x00000001
#define RAWPASS_RX_BUFF_SIZE                  		1
	
/*******************************************************************************
*@ Module    			:  Structure
*@ Description    :  Upload event type
*******************************************************************************/
enum{
  bleIIC_Slave_EVT_TX_NOTI_DISABLED = 1,
  bleIIC_Slave_EVT_TX_NOTI_ENABLED,
  bleIIC_Slave_EVT_BLE_DATA_RECIEVED,
};
  
typedef struct{
  unsigned char	ev;
  unsigned short int param;
  void* 	data;
}bleIIC_Slave_Evt_t;

/*******************************************************************************
*@ Module    			:  Functions
*@ Description    :  
*******************************************************************************/
typedef void (*bleIIC_Slave_ProfileChangeCB_t)(bleIIC_Slave_Evt_t* pev);
BLEIIC_SS_Ext bStatus_t bleIIC_Slave_AddService( bleIIC_Slave_ProfileChangeCB_t cb);
BLEIIC_SS_Ext uint8 bleIIC_Slave_NotifyIsReady(void);
BLEIIC_SS_Ext bStatus_t bleIIC_Slave_Notify( unsigned short int connHandle, \
													attHandleValueNoti_t *pNoti, 
													unsigned char taskId );

#endif

