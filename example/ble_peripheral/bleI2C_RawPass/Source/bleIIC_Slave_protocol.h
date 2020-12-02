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
* @file			bleIIC_Slave Protocol.h
* @brief		Contains the bleIIC Slave Protocol definitions and prototypes
* @version	1.0
* @date			2018/7/5  
* @author		Rugooo
* 
* Copyright(C) 2016, PhyPlus Semiconductor All rights reserved.
*******************************************************************************/
#ifndef __BLEIIC_SLAVE_PROTOCOL_H__
#define __BLEIIC_SLAVE_PROTOCOL_H__

/*******************************************************************************
*@ Module    			:  Pre-Compiler
*@ Description    :  NULL
*******************************************************************************/
#ifdef _BLE_IIC_SLAVE_PROTOCOL_CMD_
	#define BLEIIC_SP_Ext	
#else
	#define BLEIIC_SP_Ext extern
#endif


/*******************************************************************************
*@ Module    			:  Struct
*@ Description    :  Upload event type
*******************************************************************************/
typedef struct {
  unsigned char ev;
}BUP_Evt_t;

/*******************************************************************************
*@ Module    			:  Global Function
*@ Description    :  
*******************************************************************************/
int BUP_disconnect_handler(void);
int BUP_data_BLE_Received(unsigned char* pdata, unsigned char size);
void BUP_data_Notify_IIC_Rx_Data(void);
int BUP_IIC_Slave_init(void);
void BUP_IIC_Slave_status_Check(void);

#endif

