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

/*!
*date 2018-03-20
*yufeng.lao
*version V1.0
* @brief this file is used for acceleration sensor drive and interference*/
#include <stdio.h>
#include <stdlib.h>
#include "osal.h"
#include "KX023.h"
#include "types.h"
#include "hal_mcu.h"
#include "gpio.h"
#include "error.h"
#include "i2c.h"
#include "log.h"
#include "app_wrist.h"

#define KX023_SLAVE_ADDR 0x1E

typedef struct _kx023_ctx_t{
	bool			module_valid;
	uint32_t		evt_cnt;
	kx023_evt_hdl_t evt_hdl;
}kx023_ctx_t;

// store come from pedometer parm config
static kx023_ctx_t s_kx023_ctx;


// store ACC data;
uint8_t  acc_data[41*6];

void kx023_timer_stop(void)
{
}
void kx023_timer_start(int ms)
{
  osal_start_timerEx(AppWrist_TaskID, ACC_DATA_EVT, ms);
}


void kx023_delay_ms(int ms)
{	
	volatile int i = 4500;
	volatile int loop = ms;
	
  while(loop) 
  { 
		loop--; 
		for(; i; i--);
	} 
}


static void* kxi2c_init(void)
{
  void* pi2c;
	hal_i2c_pin_init(I2C_0, P26, P27);
  pi2c = hal_i2c_init(I2C_0,I2C_CLOCK_400K);
  return pi2c;
}
static int kxi2c_deinit(void* pi2c)
{
  int ret;
  ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(P26,IE);
  hal_gpio_pin_init(P27,IE);
	return ret;
}


static int kxi2c_read(void* pi2c, uint8_t reg, uint8_t* data, uint8_t size)
{
 	return hal_i2c_read(pi2c, KX023_SLAVE_ADDR, reg, data, size);
}

static int kxi2c_write(void* pi2c, uint8_t reg, uint8_t val)
{
  uint8_t data[2];
  data[0] = reg;
  data[1] = val;
  hal_i2c_addr_update(pi2c, KX023_SLAVE_ADDR);
  {
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data, 2);
    HAL_EXIT_CRITICAL_SECTION();
  }
  return hal_i2c_wait_tx_completed(pi2c);
}


//return value: 1 is raise, 0 is drop
static uint8_t kx023_figure_gesture(uint8_t current, uint8_t prev)
{
	if(current == TPR_FD && (prev == TPR_LE ||prev == TPR_RI||prev == TPR_DO))
		return 1;

	return 0;

}

uint16_t kx023_fetch_acc_data(void)
{
  int ret;
	kx023_ev_t ev;
	uint8_t val;
	//uint8_t acc_count;
	//int16_t new_x, new_y, new_z;
	void* pi2c = kxi2c_init();
	ret = kxi2c_read(pi2c, ACC_REG_ADDR_BUF_STATUS_REG1,&val,1);
	if(ret != PPlus_SUCCESS){
    kxi2c_deinit(pi2c);
    return 0;
  }
	if(val>0)
	{
		//int16_t* acc_data_xyz = (int16_t*)acc_data;
		kxi2c_read(pi2c, ACC_REG_ADDR_BUF_READ,acc_data,val);
		
		//LOG("ACC:[%x][%x][%x][%x][%x][%x]",acc_data[0],acc_data[1],acc_data[2],acc_data[3],acc_data[4],acc_data[5]);
		//LOG("--[%x][%x][%x]\n",acc_data_xyz[0],acc_data_xyz[1],acc_data_xyz[2]);
	}
	kxi2c_deinit(pi2c);
	ev.ev = wmi_event;
	ev.size = val;
	ev.data = acc_data;
	s_kx023_ctx.evt_hdl(&ev);
	return val;
}


uint8_t drv_kx023_event_handle()
{
	uint8_t val;
	uint8_t tmp;
	kx023_ev_t ev;
	void* pi2c;
  kx023_timer_start(400);
  pi2c = kxi2c_init();
	kxi2c_read(pi2c,ACC_REG_ADDR_INT_SRC_REG2 ,&val,1);
	kxi2c_read(pi2c,ACC_REG_ADDR_INT_REL,&tmp,1);
	kxi2c_deinit(pi2c);
  //LOG("K\n");
	if(val & INS2_BFI)	//fifo full
	{
		//LOG("Fifo full\n");
    kx023_fetch_acc_data();
		
		LOG("fifo is full\r\n");
	}
	if(val & INS2_TDTS1)	//Double tap
	{
		//LOG("Double tap\n");
		ev.ev = dtap_event;
		ev.data = NULL;
		s_kx023_ctx.evt_hdl(&ev);
	}
	if(val & INS2_TDTS0)	//tap
	{
		//LOG("Single tap\n");
		ev.data = NULL;
		ev.ev = stap_event;
		s_kx023_ctx.evt_hdl(&ev);
	}
	if(val & INS2_TPS)	//tilt
	{
		uint8_t t1,t2;
		uint8_t flg;
		pi2c = kxi2c_init();
		kxi2c_read(pi2c, ACC_REG_ADDR_TILT_POS_CUR,&t1,1);
		kxi2c_read(pi2c, ACC_REG_ADDR_TILT_POS_PRE,&t2,1);
		kxi2c_deinit(pi2c);
		flg = kx023_figure_gesture(t1,t2);
		//LOG("tilt, c:%x, p:%x\n",t1,t2);
		ev.flg = flg;
		ev.ev = tilt_event;
		s_kx023_ctx.evt_hdl(&ev);
	}
	if(val & INS2_WMI)// WMI
	{
		//LOG("WMI\n");
		kx023_timer_stop();
		kx023_timer_start(1000);

		kx023_fetch_acc_data();
	}

		
		
	return 0;
}


	
void kx023_int_hdl(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
	if(type == POSEDGE)
  {
		s_kx023_ctx.evt_cnt ++;
    osal_set_event(AppWrist_TaskID, ACC_DATA_EVT);
	}
}



void kx023_timeout_hdl(void *parm)
{
  kx023_fetch_acc_data();
}			
	

/*!
*	@brief this funtion is used for initialize KX023 register,and initialize GPIOE，and regeister event handler
*
*/
int kx023_config(void* pi2c,  bool tilt_en, uint8_t fifo_thresh)
{
	uint8_t  val;
	/***************verify proper integraterd circuit******************************/
	kxi2c_read(pi2c, ACC_REG_ADDR_DCST_RESP,&val,1);
	if(val!=0x55)
		return PPlus_ERR_IO_FAIL;
	kx023_delay_ms(50);
		
	//val = ACC_COTC;
	//kxi2c_write(ACC_REG_ADDR_CTRL_REG2,val);
	//kxi2c_read(ACC_REG_ADDR_DCST_RESP,&val,1);
	//if(val != 0xAA)
	//	return PPlus_ERR_IO_FAIL;
		
	//kx023_delay_ms(50);
	
	// verifiy chip ID default 0x15
	//kxi2c_read(ACC_REG_ADDR_WHO_AM_I,&val,1);
	//if(val != REG_VAL_WHO_AM_I)
	//	return PPlus_ERR_IO_FAIL;

	
	/***************verify proper integraterd circuit******************************/

	val = 0x00;    				// disable sensor 
	kxi2c_write(pi2c, ACC_REG_ADDR_CTRL_REG1,val);

	val = 0x10;    				// disable sensor  range  8g  low power mode
	kxi2c_write(pi2c, ACC_REG_ADDR_CTRL_REG1,val); 
  
	val = 0x02;			//Configure ODR, default 0x02:50Hz, 0x01:25Hz
	kxi2c_write(pi2c, ACC_REG_ADDR_ODR_CNTL,val);
	

	/****************configure physical PIN************************/
	val = 0x30;   		//0x28 Active low, pulse ,0x38 Active high pluse //0x20 Active low, latch ,0x30 Active high, latch
	kxi2c_write(pi2c, ACC_REG_ADDR_INT_CTRL_REG1,val);

	/*******************bond physical PIN interrupt to event********/
	kx023_delay_ms(50);
	
	if(tilt_en){
		val = TPI1|TDTI1|WMI1|BFI1;   //0x10 for DRDY, 0x02 for Wake up, 0x20 for WMI, 0x04 for TAP,0x01 for tilt
	}
	else{
		val = TDTI1|WMI1|BFI1;   //0x10 for DRDY, 0x02 for Wake up, 0x20 for WMI, 0x04 for TAP,0x01 for tilt
	}
	kxi2c_write(pi2c, ACC_REG_ADDR_INT_CTRL_REG4,val);

	kx023_delay_ms(50);

	//set fifo control
	val = 0x0B; 		//0B:no samples average
	kxi2c_write(pi2c, ACC_REG_ADDR_LP_CNTL,val);
	val = fifo_thresh;			//Buffer threshold
	kxi2c_write(pi2c, ACC_REG_ADDR_BUF_CTRL1,val);
	val = BUFE|BRES|BFIE;//0xE0;
	kxi2c_write(pi2c, ACC_REG_ADDR_BUF_CTRL2,val);


	//set Tilt position
	//val = 6
	kx023_delay_ms(50);

	//Turn on Sensor  
	kxi2c_read(pi2c, ACC_REG_ADDR_CTRL_REG1,&val, 1); 
	if(tilt_en){
		val |= ACC_REG_CTRL_PC | ACC_REG_CTRL_GSEL_TPE;   //enable sensor
	}
	else{
		val |= ACC_REG_CTRL_PC;   //enable sensor
	}
	kxi2c_write(pi2c, ACC_REG_ADDR_CTRL_REG1,val);
	
	kx023_delay_ms(50);
	//Clear the interrupt
 	kxi2c_read(pi2c, ACC_REG_ADDR_INT_REL,&val, 1);
	
	kx023_delay_ms(50);
  kx023_fetch_acc_data();
	//kxi2c_read(ACC_REG_ADDR_INT_REL,&val,1);

	return PPlus_SUCCESS;

}


int kx023_enable_tilt(bool en)
{
	void* pi2c  = kxi2c_init();
	kx023_config(pi2c, en, DEFAULT_BUF_THRES);
	kxi2c_deinit(pi2c);
	return PPlus_SUCCESS;
}
int kx023_enable(void)
{
	int ret = 0;
	void* pi2c = kxi2c_init();
	ret = kx023_config(pi2c, FALSE, DEFAULT_BUF_THRES);
	LOG("kx023_enable is %d\n",ret);
	kxi2c_deinit(pi2c);
	return ret;
}

int kx023_disable(void)
{
	void* pi2c = kxi2c_init();
	kxi2c_write(pi2c, ACC_REG_ADDR_CTRL_REG1,0);
	kxi2c_deinit(pi2c);
	return PPlus_SUCCESS;
}


int kx023_init(kx023_evt_hdl_t evt_hdl)
{
	int ret = PPlus_SUCCESS;
	//copy pcfg
	s_kx023_ctx.evt_hdl = evt_hdl;
	
									
	ret = hal_gpioin_register(P14, kx023_int_hdl, NULL );//pin_event_handler);
	
	ret = kx023_enable();
  kx023_timer_start(400);

	if(ret != PPlus_SUCCESS){
		s_kx023_ctx.module_valid = false;
		return ret;
	}
	s_kx023_ctx.module_valid = true;

  
	return PPlus_SUCCESS;
}































