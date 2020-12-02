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
* @file			bleIIC_Slave.h
* @brief		Contains the bleIIC Slave application definitions and prototypes
* @version	1.0
* @date			2018/7/5  
* @author		Rugooo
* 
* Copyright(C) 2016, PhyPlus Semiconductor All rights reserved.
*******************************************************************************/

#ifndef __BLEIIC_SLAVE_H__
#define __BLEIIC_SLAVE_H__

/*******************************************************************************
*@ Module    			:  Pre-Compiler
*@ Description    :  NULL
*******************************************************************************/
#ifdef _BLE_IIC_SLAVE_CMD_
	#define BLEIIC_Ext	
#else
	#define BLEIIC_Ext extern
#endif

/*******************************************************************************
*@ Module    			: Macro Define
*@ Description    :  BLE IIC SLAVE Task Events
*******************************************************************************/
#define BUP_OSAL_EVT_START_DEVICE                         0x0001
//#define BUP_OSAL_EVT_BLE_TIMER                            0x0002
//#define BUP_OSAL_EVT_ENTER_NOCONN                         0x0004
#define BUP_OSAL_EVT_RESET_ADV                            0x0008
	
//#define BUP_OSAL_EVT_CCCD_UPDATE                          0x0010
#define BUP_OSAL_EVT_NOTIFY_DATA                          0x0040
	
#define BUP_OSAL_EVT_NOTIFY_IIC_RX_DATA						  			0x0400
#define BUP_OSAL_EVT_IIC_TO_HOST_TMOUT					  				0x0800
#define BUP_OSAL_EVT_IIC_Notice_Master										0x1000
#define BUP_OSAL_EVT_IIC_TX_ABRT													0x2000
	
/*******************************************************************************
*@ Module    			:  Macro Define
*@ Description    :  GPIO to communication with IIC Master(for wake up)
*******************************************************************************/
#define FLOW_CTRL_IO_NOTICE_HOST       	P18			// BLE Central --> IIC Slave Module --> IIC Master
#define FLOW_CTRL_IO_MODUAL_WAKEUP   		P15			// Indicate BLE Central and IIC Slave Module Connected


/*******************************************************************************
*@ Module    			:  Variable Statement
*@ Description    :  	Null
*******************************************************************************/
BLEIIC_Ext unsigned char  BLE_IIC_TaskID;
BLEIIC_Ext unsigned short gapConnHandle;

/*******************************************************************************
*@ Module    			:  Functions Statement
*@ Description    :  Null
*******************************************************************************/
BLEIIC_Ext unsigned short bleIIC_Slave_ProcessEvent(unsigned char task_id,unsigned short events);
BLEIIC_Ext void BLE_IIC_Slave_Init(unsigned char task_id);
BLEIIC_Ext unsigned short BLE_IIC_Conn_Interval(void);
BLEIIC_Ext void BLE_IIC_CheckTOMaster_success(void);


#endif
