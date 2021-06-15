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
  Filename:       CHT8305_sensor.h
  Revised:        $Date $
  Revision:       $Revision $
  Author:         qing
  PHY+
**************************************************************************************************/

#include "gpio.h"
#include "log.h"
#include "OSAL.h"
#include "pwrmgr.h"
#include "common.h"
#include "error.h"
#include "CHT8305_sensor.h"
#include "i2c.h"
#include "clock.h"
#include "EM_platform.h"

#define CHT8305_I2C_OP_TIMEOUT  100
#define CHT8305_I2CDEV  I2C_0
#define CONSOLE_OUT(...)    printf(__VA_ARGS__)


AP_I2C_TypeDef *pCht_i2cx;


uint8_t configReg; //higher 8 bits of the configuration register

static int  CHT8305_read_reg(void* pi2c, uint8_t regAddr, uint8_t* data, uint8_t numOfBytes);

 void CHT8305_I2CInit(void)
{
	hal_i2c_pin_init(CHT8305_I2CDEV,P32,P31);
	pCht_i2cx=hal_i2c_init(CHT8305_I2CDEV,I2C_CLOCK_100K);
	hal_i2c_addr_update(pCht_i2cx,CHT8305_DEV_ADDR);
}

void CHT8305_I2CDeinit(void)
{
	hal_i2c_deinit(pCht_i2cx);
	hal_gpio_pin_init(P31,IE);
	hal_gpio_pin_init(P32,IE);
}


#if 0
static int CHT8305_I2C_WriteData(void* pi2c,  uint8_t val)
{
  uint8_t data[1];
  data[0] = val;
  hal_i2c_addr_update(pCht_i2cx, CHT8305_DEV_ADDR);
  {
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data, 1);
    HAL_EXIT_CRITICAL_SECTION();
  }
	  
  return hal_i2c_wait_tx_completed(pi2c);
}
#endif


static int CHT8305_i2c_read_s(uint8_t slave_addr,uint8_t reg, uint8_t* data, uint8_t size)
{
	uint8_t i;

	  I2C_INIT_TOUT(to);

  hal_i2c_addr_update(pCht_i2cx, slave_addr);
  
  HAL_ENTER_CRITICAL_SECTION();
  hal_i2c_tx_start(pCht_i2cx);
  hal_i2c_send(pCht_i2cx, &reg,1);
	
  if(reg==0x00||reg==0x001)
  	WaitMs(10);
   	for(i=0;i<size;i++){
		I2C_READ_CMD(pCht_i2cx);
   		}
  HAL_EXIT_CRITICAL_SECTION();
  
  while(1){
    if(I2C_RX_FIFO_NOT_EMPTY(pCht_i2cx)){
      *data = (pCht_i2cx->IC_DATA_CMD&0xff);
      data++;
      size --;
      if(size == 0)
        break;
    }
    I2C_CHECK_TOUT(to, CHT8305_I2C_OP_TIMEOUT*size, "I2C RD TO\n");
  }
	return PPlus_SUCCESS;


}



void CHT8305_init(void)
{
	configReg = 0x10; //POR default
}

bool CHT8305_is_connected(void)
{
	uint32_t id = 0;

	id = CHT8305_get_manufacturerID();

	CONSOLE_OUT("id=%d\n\r",id);
	if ( id== 0x5959)
		return TRUE;
	return FALSE;
}

static int  CHT8305_read_reg(void* pi2c, uint8_t regAddr, uint8_t* data, uint8_t numOfBytes)
{
#if 1
  	return CHT8305_i2c_read_s(CHT8305_DEV_ADDR, regAddr, data, numOfBytes);
#else
    
    return hal_i2c_read(pi2c, CHT8305_DEV_ADDR, regAddr, data, numOfBytes);
#endif


}

unsigned int CHT8305_get_manufacturerID(void)
{
    uint8_t id[2]={0};
	CHT8305_read_reg(pCht_i2cx,CHT8305_REG_MID,id, 2);
	return id[0] << 8 | id[1];
}

unsigned int CHT8305_get_deviceID(void)
{
     uint8_t id[2]={0};
	CHT8305_read_reg(pCht_i2cx,CHT8305_REG_DID,id, 2);
	return id[0] << 8 | id[1];
}

void CHT8305_set_temperature_res(uint8_t res)
{
	if (res > 1) res = T_RES_14;

	configReg &= ~(1 << BIT_T_RES);
	configReg |= res << BIT_T_RES;
}

void CHT8305_set_humidity_res(uint8_t res)
{
	if (res > 2) res = H_RES_14;

	configReg &= ~(0x3 << BIT_H_RES);
	configReg |= res << BIT_H_RES;
}

void CHT8305_turnon_heater(bool heaterOn)
{
	if (heaterOn)
		configReg |= 1 << BIT_HEATER;
	else
		configReg &= ~(1 << BIT_HEATER);
}

bool CHT8305_battery_OK()
{
	uint8_t bat_val[2]={0};
	CHT8305_read_reg(pCht_i2cx,CHT8305_REG_DID,bat_val, 2);
	
	configReg = bat_val[0];

	return (configReg & (1 << BIT_BATTERY_OK)) == 0;
}



uint8_t CHT8305_get_temp_humi(uint16_t *t, uint16_t *h)
{
	unsigned int th, tl, hh, hl;



	uint8_t temp_val[4]={0};
    CHT8305_read_reg(pCht_i2cx,CHT8305_REG_TEMP,temp_val, 2);
	WaitMs(10);
	CHT8305_read_reg(pCht_i2cx,CHT8305_REG_TEMP,&temp_val[2], 2);


	th = temp_val[0];
	tl = temp_val[1];
	hh = temp_val[2];
	hl = temp_val[3];

	(*t) = ((th << 8 | tl)* 165.0 / 65535.0 - 40.0)*10;;
	(*h) = ((hh << 8 | hl)* 100.0 / 65535.0)*10;

	return PPlus_SUCCESS;
}


