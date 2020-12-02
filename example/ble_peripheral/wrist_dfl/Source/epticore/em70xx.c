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

#include <stdlib.h>
#include "types.h"
#include "app_wrist.h"
#include "hal_mcu.h"
#include "app_err.h"
#include "em70xx.h"
#include "OSAL_Timers.h"
#include "i2c.h"
#include "gpio.h"
#include "log.h"
#include "error.h"

extern int em70xx_bpm_dynamic(int in,int qsensor_x,int qsensor_y,int qsensor_z);
extern void em70xx_reset(void);

uint8 EM70xx_SLAVE_ADDR = (0x24);
static int heartrate_value;
static em70xxCB_t sEM70xxCB = NULL;
static uint8_t hr_started = 0;
static uint16_t hr_rawdata[8];
static uint32 hr_rawdata_cnt = 0;


static int hr_timer_start(uint32 intval_ms)
{
    osal_start_timerEx(AppWrist_TaskID, TIMER_HR_EVT, intval_ms);
    return 0;
}


static void* em_init(void)
{
	hal_i2c_pin_init(I2C_0, P26, P27);
  return hal_i2c_init(I2C_0, I2C_CLOCK_400K);
}
static int em_deinit(void* pi2c)
{
  int ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(P26,IE);
  hal_gpio_pin_init(P27,IE);
	return ret;
}


static int em_read(void* pi2c, uint8 reg, uint8* data)
{
	return hal_i2c_read(pi2c, EM70xx_SLAVE_ADDR, reg, data, 1);
}
static int em_write(void* pi2c, uint8 reg, uint8 val)
{
  uint8 data[2];
  data[0] = reg;
  data[1] = val;
  hal_i2c_addr_update(pi2c, EM70xx_SLAVE_ADDR);
  {
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data, 2);
    HAL_EXIT_CRITICAL_SECTION();
  }
  return hal_i2c_wait_tx_completed(pi2c);
}


int em70xx_register(em70xxCB_t cb)
{
  em70xx_stop();
  sEM70xxCB = cb;
	return PPlus_SUCCESS;
}

void em70xx_timer_handler(void)
{
  uint8 reg30,reg31;
  uint32 value = 0;
  int hr;
  if(!hr_started)
    return;
  void* pi2c = em_init();
  em_read(pi2c, 0x30,&reg30);
  em_read(pi2c, 0x31,&reg31);
  em_deinit(pi2c);
  value = ((uint32)reg30)|(((uint32)reg31)<<8);
  
  hr_rawdata[hr_rawdata_cnt%8] = value;
  if(hr_rawdata_cnt%8 == 7){
    hr_ev_t ev;
    ev.ev = HR_EV_RAW_DATA;
    ev.value = 8;
    ev.data = hr_rawdata;
    sEM70xxCB(&ev);
  }
  hr_rawdata_cnt++;
  
  hr = em70xx_bpm_dynamic((int)value, 0,0,0);
	//LOG1("%d, %d\n",value,hr);
  if(hr != heartrate_value){
    LOG("em70xx %d\n", hr);
    heartrate_value = hr;
    if(sEM70xxCB){
      hr_ev_t ev;
      ev.ev = HR_EV_HR_VALUE;
      ev.value = hr;
      ev.data = NULL;
      sEM70xxCB(&ev);
    }
  }
  hr_timer_start(30);
}






int em70xx_start(void)
{
  void* pi2c = em_init();
  //em_read(0, &pid);
  LOG("em70xx_start\n");
  em_write(pi2c, 0x01, 0x08);
  em_write(pi2c, 0x08, 0x00);
  em_write(pi2c, 0x0A, 0x7F);
  em_write(pi2c, 0x0D, 0xC7);

  em_deinit(pi2c);
  em70xx_reset();
  heartrate_value = 0;
  hr_started = 1;
  hr_timer_start(30);
	return 0;
}


int em70xx_stop(void)
{
  heartrate_value = 0;
  hr_started = 0;
  void* pi2c = em_init();
  LOG("em70xx_stop\n");
  em_write(pi2c, 0x01, 0x08);
  em_write(pi2c, 0x08, 0x00);
  em_write(pi2c, 0x0A, 0x7F);
  em_write(pi2c, 0x0D, 0xC7);
  em_write(pi2c, 0x01, 0x00);

  em_deinit(pi2c);
	return 0;
}

bool em70xx_state(void)
{
  return hr_started ? TRUE:FALSE;
}



