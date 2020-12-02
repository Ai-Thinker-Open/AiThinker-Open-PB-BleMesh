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
* @file		i2c_slave.c
* @brief	i2c slave function
* @version	1.0
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*******************************************************************************/

/*******************************************************************************
*@ Module    	  	:  Includes
*@ Description    :  NULL
*******************************************************************************/
#include "i2c_slave.h"
#include "gpio.h"
#include "error.h"
#include "osal.h"
#include "clock.h"
#include "log.h"

/*******************************************************************************
*@ Module    	  	:  IIC_Slave status
*@ Description    :  NULL
*******************************************************************************/
typedef enum
{
	Slave_Closed = 0,
	Slave_Opened,
}IIC_Slave_status;

/*******************************************************************************
*@ Module    	  	:  IIC_Slave_cfg structure
*@ Description    :  NULL
*******************************************************************************/
typedef struct
{
	AP_I2C_TypeDef 		*addr;									// get by software
	MODULE_e 					module;									// get by software
	uint8_t 					INTR_ID;								// get by software
	GPIO_Pin_e 				SDA_PIN;								// I2C Pin 
	GPIO_Pin_e 				SCL_PIN;
	Fmux_Type_e				Fmux_SCL;								// get by software
	Fmux_Type_e				Fmux_SDA;								// get by software
	IIC_Slave_status 	state;
}IIC_Slave_cfg;

/*******************************************************************************
*@ Module    	  	:  Global Variable
*@ Description    :  NULL
*******************************************************************************/
 IIC_Slave_cfg Slave_cfg_g[IIC_COUNT];

/*******************************************************************************
*@ Module    	  	:  Internal Function statement
*@ Description    :  NULL
*******************************************************************************/
void I2C_Slave_Handler(I2C_Evt_t *pev);

/*******************************************************************************
*@ Module    	  	:  Function Statement
*@ Description    :  NULL
*******************************************************************************/
uint8_t Hal_I2C_Slave_Init(I2C_Slave_Parameter *para,uint8_t *handle)
{
	uint8_t ret;
	LOG("Hal_I2C_Slave_Init wakeup id:%02X \r\n",para->id);
	ret = Hal_IIC_Valid_Check(para->id);
	if( ret == TRUE )
	{
		return PPLUS_ERR_IIC_BUSY;
	}
	else if( ret == PPlus_ERR_IIC_ID)
	{
		return PPlus_ERR_IIC_ID;
	}
	else
	{
		if( Hal_IIC_Addr_Valid(para->Slave_Address) !=  PPlus_IIC_SUCCESS)
			return PPlus_ERR_IIC_ADDRESS;
		
		osal_memset(&Slave_cfg_g[para->id],0,sizeof(IIC_Slave_cfg));
		
		// pre-para get
		Slave_cfg_g[para->id].addr = Hal_Get_IIC_Instance(para->id);
		Slave_cfg_g[para->id].module = Hal_GetIIC_ModuleID(para->id);
		Slave_cfg_g[para->id].INTR_ID = Hal_GetIIC_IRQID(para->id);
		Hal_GetIIC_PIN_Fmux(para->id,&(Slave_cfg_g[para->id].Fmux_SCL),&(Slave_cfg_g[para->id].Fmux_SDA));
		Slave_cfg_g[para->id].state = Slave_Closed;
		
		// io init
		Slave_cfg_g[para->id].SCL_PIN = para->SCL_PIN;
		Slave_cfg_g[para->id].SDA_PIN = para->SDA_PIN;
		hal_gpio_fmux_set(para->SCL_PIN, Slave_cfg_g[para->id].Fmux_SCL);
		hal_gpio_fmux_set(para->SDA_PIN, Slave_cfg_g[para->id].Fmux_SDA);
		hal_gpio_pull_set(para->SCL_PIN,STRONG_PULL_UP);
		hal_gpio_pull_set(para->SDA_PIN,STRONG_PULL_UP);
		
		// init
		clk_gate_enable(Slave_cfg_g[para->id].module);
		
		Slave_cfg_g[para->id].addr->IC_ENABLE = FALSE;
		Slave_cfg_g[para->id].addr->IC_CON = para->workmode | para->AddressMode; 
		Slave_cfg_g[para->id].addr->IC_SAR = para->Slave_Address;
		Slave_cfg_g[para->id].addr->IC_RX_TL = para->RX_FIFO_Len - 1;
		Slave_cfg_g[para->id].addr->IC_TX_TL = para->Tx_FIFO_Len - 1;
		Slave_cfg_g[para->id].addr->IC_INTR_MASK = para->IRQ_Source;
	
		Hal_IIC_Register_CallBack(para->id,para->evt_handler);
		
		Hal_I2c_Slave_Open(para->id);
		// NVIC Config
		NVIC_EnableIRQ((IRQn_Type)(Slave_cfg_g[para->id].INTR_ID));
		
		NVIC_SetPriority((IRQn_Type)(Slave_cfg_g[para->id].INTR_ID), IRQ_PRIO_HAL);

		*handle = para->id;
		return PPlus_IIC_SUCCESS;
	}
}

/*******************************************************************************
*@ Module    	  	:  Open I2C Slave
*@ Description    :  NULL
*******************************************************************************/
uint8_t Hal_I2c_Slave_Open(uint8_t handle)
{
	if( Slave_cfg_g[handle].state == Slave_Closed )
	{
		Slave_cfg_g[handle].state = Slave_Opened;
		Slave_cfg_g[handle].addr->IC_ENABLE = TRUE;
		return PPlus_IIC_SUCCESS;
	}
	else
		return PPlus_ERR_IIC_FAILURE;
}

/*******************************************************************************
*@ Module    	  	:  Close I2C Slave
*@ Description    :  NULL
*******************************************************************************/
void Hal_I2c_Slave_Close(uint8_t handle)
{
	Slave_cfg_g[handle].addr->IC_ENABLE = FALSE;
}

/*******************************************************************************
*@ Module    	  	:  Close I2C Slave
*@ Description    :  Should check close states,if closed completed
										 a delay occurs when IC_ENABLE is set to 0, because close iic module depends
										 on the iic bus activity
*******************************************************************************/
uint8_t Hal_Check_I2C_Slave_Closed(uint8_t handle)
{
	if( Hal_Check_IIC_Closed(Slave_cfg_g[handle].addr) == PPlus_IIC_SUCCESS )
	{
		Slave_cfg_g[handle].state = Slave_Closed;
		return PPlus_IIC_SUCCESS;
	}
	else
		return PPLUS_ERR_IIC_ENABLE;
}

/*******************************************************************************
*@ Module    	  	:  Close I2C Slave
*@ Description    :  NULL
*******************************************************************************/
uint8_t Hal_I2C_Slave_Deinit(uint8_t *handle)
{
	LOG("HAL I2C SLAVE DINIT handle Value %d \n",*handle);
	if( Hal_IIC_unRegister_CallBack(*handle) != PPlus_IIC_SUCCESS )
		return PPlus_ERR_IIC_FAILURE;
	
//	// NVIC Config -- Disable
//	NVIC_DisableIRQ((IRQn_Type)(Slave_cfg_g[*handle].INTR_ID));
//	
//	// clock disable
//	clk_gate_disable( Slave_cfg_g[*handle].module);
//	
//	// io deinit
//	hal_gpio_fmux(Slave_cfg_g[*handle].SCL_PIN,Bit_DISABLE);
//	hal_gpio_fmux(Slave_cfg_g[*handle].SDA_PIN,Bit_DISABLE);
//	hal_gpio_pull_set(Slave_cfg_g[*handle].SCL_PIN,PULL_DOWN);
//	hal_gpio_pull_set(Slave_cfg_g[*handle].SDA_PIN,PULL_DOWN);
//	
//	osal_memset(&Slave_cfg_g[*handle],0,sizeof(IIC_Slave_cfg));
	
	*handle = PPlus_INVALID_HANDLE;
	return PPlus_IIC_SUCCESS;
}

/*******************************************************************************
*@ Module    	  	:  Read data from rx fifo
*@ Description    :  NULL
*******************************************************************************/
void Hal_I2C_Slave_ReadRX_FIFO(uint8_t handle,uint8_t *p,uint8_t len)
{
	for(uint8_t i =0;i<len;i++)
		*p++ = Hal_IIC_Read_RXFIFO(Slave_cfg_g[handle].addr);
}

/*******************************************************************************
*@ Module    	  	:  Clear interrupt status 
*@ Description    :  NULL
*******************************************************************************/
void Hal_I2C_Slave_CLR_IRQs(uint8_t handle,uint32_t irqs)
{
	Hal_INTR_SOURCE_Clear(Slave_cfg_g[handle].addr,irqs);
}

/*******************************************************************************
*@ Module    	  	:  Hal_I2C_Slave_WriteTX_FIFO 
*@ Description    :  NULL
*******************************************************************************/
void Hal_I2C_Slave_WriteTX_FIFO(uint8_t handle,uint8_t *p,uint8_t len)
{
	for(uint8_t i=0;i<len;i++)
		Hal_IIC_Write_TXFIFO(Slave_cfg_g[handle].addr,*p++);
}

/*******************************************************************************
*@ Module    	  	:  I2C Slave Init steps:
*@ Description    :  NULL
*******************************************************************************/
/*
	1、IC_ENABLE=0;
	2、IC_SAR = ? （Master和Slave不能设置相同的地址类型？）
	3、IC_CON(配置地址模式和SLAVE模式)
	4、IC_ENABLE=1;
*/
/*******************************************************************************
*@ Module			:  I2C Slave 单字节发送
*@ Description	  :  NULL
*******************************************************************************/
/*
	1、IIC MASTER 寻址
	2、IIC SLAVE ACK
	3、IIC SLAVE 触发 RD_REQ 中断， 同时 HOLD THE SCL LOW , WAIT UNTIL THE SOFTWARE RESPONDS(数据响应？)
		SOFTWARE RESPONDS的时间间隔，10倍SCL周期 （400K--->25us）（一个byte传输完成的时间）
	4、读请求触发前，如果TXFIFO,总线触发TX_ABRT中断，清除旧数据	（通过软件读取 IC_CLR_TX_ABRT register，释放总线）
	5、写数据到 IC_DATA_CMD
	6、清除RD_REQ,TX_ABRT中断标志位（如果中断被屏蔽，则需要清除IC_RAW_INTR_STAT）
	7、总线释放SCL并传输数据（与2对应）
	8、MASTER 通过RESTART 或 STOP 继续操作总线
*/
/*******************************************************************************
*@ Module			:  I2C Slave 单字节接收
*@ Description	  :  NULL
*******************************************************************************/
/*
	1、IIC MASTER 寻址
	2、IIC SLAVE ACK
	3、receives the transmitted byte and places it in the receive buffer
		如果RX FIFO已经满了，再来数据的时候，将会触发R_RX_OVER中断，于此同时，
		IIC数据将会继续传输（没有NACK信号），并无法保证后续数据的完整
	4、RX_FULL 中断
	5、从IC_DATA_CMD 中读取数据
	6、MASTER 通过RESTART 或 STOP 继续操作总线
*/

