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
* @file			bleIIC_Slave Protocol.c
* @brief		Contains the bleIIC Slave Protocol definitions and prototypes
* @version	1.0
* @date			2018/7/5  
* @author		Rugooo
* 
* Copyright(C) 2016, PhyPlus Semiconductor All rights reserved.
*******************************************************************************/

/*******************************************************************************
*@ Module    			:  Pre-Compiler
*@ Description    :  NULL
*******************************************************************************/
#define _BLE_IIC_SLAVE_PROTOCOL_CMD_
#define BLE_IIC_Slave_State_TimeOut				5

/*******************************************************************************
*@ Module    			:  Includes
*@ Description    :  None
*******************************************************************************/
#include <string.h>
#include "types.h"
#include "att.h"
#include "bleIIC_Slave_protocol.h"
#include "bleIIC_Slave_service.h"
#include "i2c_Slave.h"
#include "pwrmgr.h"
#include "clock.h"
#include "error.h"
#include "bleIIC_Slave.h"
#include "peripheral.h"
#include "linkdb.h"
#include "log.h"
#include "osal.h"

/*******************************************************************************
*@ Module    			:  Structure
*@ Description    :  None
*******************************************************************************/
typedef enum{
	IDLE = 0,
	BUSY,
}bleiic_state;

typedef struct{
	// common
	bleiic_state state;
	
	uint8_t ble_rx_buf[I2C_RX_TL_CNT];
	uint8_t ble_tx_buf[I2C_TX_TL_CNT];
	// IIC Receiver
	uint8_t iic_rx_buf[I2C_RX_TL_CNT];

	// IIC Transmitter
	struct{
	 uint8_t iic_tx_buf[I2C_TX_TL_CNT];
	}tx[I2C_FIFO_DEPTH];
}BUP_ctx_t;

/*******************************************************************************
*@ Module    			:  Variable
*@ Description    :  None
*******************************************************************************/
static BUP_ctx_t mBUP_Ctx;
static uint8_t I2C_Slave_Handle = PPlus_INVALID_HANDLE;
const uint8_t I2C_Slave_Module = IIC_Module0;
static MODULE_e i2c_mod;

/*******************************************************************************
*@ Module    			:  Local Function
*@ Description    :  None
*******************************************************************************/
void hal_i2c_evt_hdl(I2C_Evt_t *pev);
void hal_i2c_slave_wakeupconfig(void);
void hal_i2c_slave_sleepconfig(void);
void hal_i2c_slave_rx_full_handler(void);
void hal_i2c_slave_read_request_handler(void);
void hal_i2c_slave_rx_done_handler(void);
void hal_i2c_slave_tx_abort_handler(void);

/*******************************************************************************
*@ Module    			:  LOCAL Function
*@ Description    :  GPIO WakeUP Host to start IIC Communication
*******************************************************************************/
void IIC_Slave_gpio_wakeup_handle(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	unsigned short int conn_hdl;
	LOG(" IIC_Slave_gpio_wakeup_handle \n");
  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
  if(conn_hdl == INVALID_CONNHANDLE){
		LOG("BLE Unconnected \n");
    hal_pwrmgr_unlock(MOD_USR1);
    return;
  }
  if(hal_pwrmgr_is_lock(MOD_USR1) == FALSE){
		hal_pwrmgr_lock(MOD_USR1);
		hal_i2c_slave_wakeupconfig();
		
		LOG("hal_pwrmgr_lock MOD_USR1\n");
  }
}

/*******************************************************************************
*@ Module    			:  BUP_disconnect_handler
*@ Description    :  precess, when ble disconnect from the client
*******************************************************************************/
int BUP_disconnect_handler(void)
{
	memset(&mBUP_Ctx, 0, sizeof(mBUP_Ctx));
  hal_gpio_write(FLOW_CTRL_IO_MODUAL_WAKEUP, 0);
  hal_gpio_write(FLOW_CTRL_IO_NOTICE_HOST, 0);
  hal_pwrmgr_unlock(MOD_USR1);
	hal_pwrmgr_unlock(i2c_mod);
	return  0;
}

/*******************************************************************************
*@ Module    			:  BUP_data_BLE_Received
*@ Description    :  upload data to IIC ,when ble receive data
*******************************************************************************/
int BUP_data_BLE_Received(unsigned char* pdata, unsigned char size)
{
	// copy data to rx buffer ( ble client to ble server)
	if(size > I2C_RX_TL_CNT)
		osal_memcpy(mBUP_Ctx.ble_rx_buf,pdata,I2C_RX_TL_CNT);
	else
	{
		osal_memset(mBUP_Ctx.ble_rx_buf,0,I2C_RX_TL_CNT);
		osal_memcpy(mBUP_Ctx.ble_rx_buf,pdata,size);
	}
	LOG("BUP_data_BLE_Received :");
	for(uint8_t i=0;i<size;i++)
		LOG("%02X,",mBUP_Ctx.ble_rx_buf[i]);
	LOG("\n");
	
	// copy data to iic transmit buffer
	if( mBUP_Ctx.ble_rx_buf[0] > (I2C_RX_TL_CNT-1))
	{
		osal_memset(mBUP_Ctx.tx[0].iic_tx_buf,0,I2C_TX_TL_CNT);
		osal_memcpy(mBUP_Ctx.tx[0].iic_tx_buf,mBUP_Ctx.ble_rx_buf,size);	
	}
	else
	{
		osal_memset(mBUP_Ctx.tx[mBUP_Ctx.ble_rx_buf[0]].iic_tx_buf,0,I2C_TX_TL_CNT);
		osal_memcpy(mBUP_Ctx.tx[mBUP_Ctx.ble_rx_buf[0]].iic_tx_buf,mBUP_Ctx.ble_rx_buf,size);
	}
	
	// Notice IIC Master that there's new data come from BLE Client
	hal_pwrmgr_lock(MOD_USR2);
	hal_gpio_write(FLOW_CTRL_IO_NOTICE_HOST,TRUE);
	osal_start_timerEx(BLE_IIC_TaskID, BUP_OSAL_EVT_IIC_Notice_Master, 10);
	
	return  0;
}

/*******************************************************************************
*@ Module    			:  BUP_data_Notify_IIC_Rx_Data
*@ Description    :  notify data ,when iic receive data from IIC MASTER
*******************************************************************************/
void BUP_data_Notify_IIC_Rx_Data(void)
{
	unsigned char ret;	
	attHandleValueNoti_t notify_data={0};
	if(bleIIC_Slave_NotifyIsReady() != FALSE){
		osal_memcpy(notify_data.value,mBUP_Ctx.ble_tx_buf,I2C_TX_TL_CNT);
		notify_data.len = I2C_TX_TL_CNT;
			
		ret = bleIIC_Slave_Notify(gapConnHandle, &notify_data, BLE_IIC_TaskID);
		if(ret == SUCCESS){
			LOG("Notify success \n");
		}else{
			LOG("Notify failure \n");
		}
	}
	else
		LOG("Notify is not ready \n");
	hal_pwrmgr_unlock(i2c_mod);
}

/*******************************************************************************
*@ Module    			:  iic init 
*@ Description    :  as Slave Mode
*******************************************************************************/
int BUP_IIC_Slave_init(void)
{
	osal_memset(&mBUP_Ctx,0,sizeof(BUP_ctx_t));
	mBUP_Ctx.state = IDLE;

	// register iic slave power management
	i2c_mod = Hal_GetIIC_ModuleID( I2C_Slave_Module );

//	hal_pwrmgr_register(i2c_mod, hal_i2c_slave_sleepconfig, NULL);
hal_pwrmgr_register(i2c_mod, NULL, NULL);
	hal_gpioin_register(FLOW_CTRL_IO_MODUAL_WAKEUP, IIC_Slave_gpio_wakeup_handle, NULL);
  hal_pwrmgr_register(MOD_USR1, NULL, NULL);
	hal_pwrmgr_register(MOD_USR2, NULL, NULL);
//	
	return PPlus_SUCCESS;
}

/*******************************************************************************
*@ Module    			:  hal_i2c_evt_hdl 
*@ Description    :  None
*******************************************************************************/
void hal_i2c_evt_hdl(I2C_Evt_t *pev)
{
	uint16_t conn_hdl;
  GAPRole_GetParameter( GAPROLE_CONNHANDLE, &conn_hdl);
  if( (conn_hdl == INVALID_CONNHANDLE) || (mBUP_Ctx.state == IDLE))
	{
		LOG("hal_i2c_evt_hdl :ble unconnected or IIC IDLE\n");
    BUP_disconnect_handler();
    return;
  }
	
	switch(pev->type){
		case I2C_RX_UNDER_Evt:
		break;
		case I2C_RX_OVER_Evt:
		break;
		case I2C_RX_FULL_Evt:
			hal_i2c_slave_rx_full_handler();
		break;
		case I2C_TX_OVER_Evt:
		break;
		case I2C_TX_EMPTY_Evt:
		break;
		case I2C_RD_REQ_Evt:
			hal_i2c_slave_read_request_handler();
			Hal_I2C_Slave_CLR_IRQs(I2C_Slave_Handle,I2C_MASK_RD_REQ);
		break;
		case I2C_TX_ABRT_Evt:
			hal_i2c_slave_tx_abort_handler();
			Hal_I2C_Slave_CLR_IRQs(I2C_Slave_Handle,I2C_MASK_TX_ABRT);
		break;
		case I2C_RX_DONE_Evt:
			hal_i2c_slave_rx_done_handler();
			Hal_I2C_Slave_CLR_IRQs(I2C_Slave_Handle,I2C_MASK_RX_DONE);
		break;
		case I2C_ACTIVITY_Evt:
		break;
		case I2C_STOP_DET_Evt:
		break;
		case I2C_START_DET_Evt:
		break;
		case I2C_GEN_CALL_Evt:
		break;
		case I2C_DINIT_SUCCESS:
		break;
  default:
    break;
  }
}

/*******************************************************************************
*@ Module    			:  hal_i2c_slave_wakeupconfig 
*@ Description    :  None
*******************************************************************************/
void hal_i2c_slave_wakeupconfig(void)
{
	LOG("WakeUp:\n");
	uint8_t ret = PPlus_ERR_IIC_FAILURE;
	
	I2C_Slave_Parameter *I2C_Slave_cfg = (I2C_Slave_Parameter *)osal_mem_alloc(sizeof(I2C_Slave_Parameter));
	osal_memset(I2C_Slave_cfg,0,sizeof(I2C_Slave_Parameter));
	
	// config i2c slave parameter
	I2C_Slave_cfg->id = I2C_Slave_Module;
	I2C_Slave_cfg->workmode = Slave;
	I2C_Slave_cfg->AddressMode = I2C_ADDR_7bit;
	I2C_Slave_cfg->IRQ_Source = (I2C_MASK_TX_ABRT | I2C_MASK_RD_REQ | I2C_MASK_RX_FULL | I2C_MASK_RX_DONE);
	I2C_Slave_cfg->RX_FIFO_Len = I2C_RX_TL_CNT;
	I2C_Slave_cfg->Tx_FIFO_Len = I2C_TX_TL_CNT;
	I2C_Slave_cfg->Slave_Address = I2C_IC_DEFAULT_ADDR;
	I2C_Slave_cfg->SCL_PIN = GPIO_P31;
	I2C_Slave_cfg->SDA_PIN = GPIO_P32;
	I2C_Slave_cfg->evt_handler = hal_i2c_evt_hdl;
	
	ret = Hal_I2C_Slave_Init(I2C_Slave_cfg,&I2C_Slave_Handle);
	if( ret != PPlus_IIC_SUCCESS )
	{
		I2C_Slave_Handle = PPlus_INVALID_HANDLE;
		LOG("I2C Slave Init Failure %d \n",ret);
	}
	else
	{
		mBUP_Ctx.state = BUSY;
		LOG("I2C Slave Init Success , handle:%d \n",I2C_Slave_Handle);
	}
	osal_mem_free(I2C_Slave_cfg);
}

/*******************************************************************************
*@ Module    			:  hal_i2c_slave_sleepconfig 
*@ Description    :  None
*******************************************************************************/
void hal_i2c_slave_sleepconfig(void)
{
	LOG("Sleep:\n");
	uint8_t ret;
	if(mBUP_Ctx.state == BUSY)
	{
		if( I2C_Slave_Handle != PPlus_INVALID_HANDLE )
		{
			while( Hal_Check_I2C_Slave_Closed(I2C_Slave_Handle) != PPlus_IIC_SUCCESS)
			{
				Hal_I2c_Slave_Close(I2C_Slave_Handle);
			}
			do
			{
				LOG("-----hal_i2c_slave_sleepconfig Handle Value %d\n",I2C_Slave_Handle);
				ret = Hal_I2C_Slave_Deinit(&I2C_Slave_Handle);
			}while( ret != PPlus_IIC_SUCCESS);
		}
	}
}

/*******************************************************************************
*@ Module    			:  hal_i2c_slave_rx_full_handler 
*@ Description    :  read data from rx fifo,
*******************************************************************************/
void hal_i2c_slave_rx_full_handler(void)
{
	LOG("hal_i2c_slave_rx_full_handler :");
	
	Hal_I2C_Slave_ReadRX_FIFO( I2C_Slave_Handle ,mBUP_Ctx.iic_rx_buf,I2C_RX_TL_CNT);

	// decode data type
	if( mBUP_Ctx.iic_rx_buf[0] != 0)
	{
		// request register 1-7 data
		osal_memcpy(mBUP_Ctx.tx[0].iic_tx_buf,mBUP_Ctx.tx[mBUP_Ctx.iic_rx_buf[0]].iic_tx_buf,I2C_TX_TL_CNT);
	}
	else
	{
		// notify data to BLE Client
		osal_memcpy(mBUP_Ctx.ble_tx_buf,mBUP_Ctx.iic_rx_buf,I2C_RX_TL_CNT);
		hal_pwrmgr_lock(i2c_mod);
		hal_pwrmgr_unlock(MOD_USR1);
		osal_start_timerEx(BLE_IIC_TaskID,BUP_OSAL_EVT_NOTIFY_IIC_RX_DATA,1);
	}
}

/*******************************************************************************
*@ Module    			:  hal_i2c_slave_read_request_handler 
*@ Description    :  send data to iic master
*******************************************************************************/
void hal_i2c_slave_read_request_handler(void)
{
	Hal_I2C_Slave_WriteTX_FIFO(I2C_Slave_Handle,mBUP_Ctx.tx[0].iic_tx_buf,I2C_TX_TL_CNT);
}

/*******************************************************************************
*@ Module    			:  hal_i2c_slave_rx_done_handler 
*@ Description    :  none
*******************************************************************************/
void hal_i2c_slave_rx_done_handler(void)
{
	LOG("I2C Slave rx done event \n");
	hal_pwrmgr_unlock(MOD_USR1);
}

/*******************************************************************************
*@ Module    			:  hal_i2c_slave_tx_abort_handler 
*@ Description    :  none
*******************************************************************************/
void hal_i2c_slave_tx_abort_handler(void)
{
	Hal_I2c_Slave_Close(I2C_Slave_Handle);
	osal_start_timerEx(BLE_IIC_TaskID, BUP_OSAL_EVT_IIC_TX_ABRT, BLE_IIC_Slave_State_TimeOut);
}

/*******************************************************************************
*@ Module    			:  BUP_IIC_Slave_status_Check 
*@ Description    :  none
*******************************************************************************/
void BUP_IIC_Slave_status_Check(void)
{
	if( Hal_Check_I2C_Slave_Closed( I2C_Slave_Handle ) == PPlus_IIC_SUCCESS )
	{
		mBUP_Ctx.state = IDLE;
		Hal_I2c_Slave_Open(I2C_Slave_Handle);
	}
	else
	{
		osal_start_timerEx(BLE_IIC_TaskID, BUP_OSAL_EVT_IIC_TX_ABRT, BLE_IIC_Slave_State_TimeOut);
	}
}

