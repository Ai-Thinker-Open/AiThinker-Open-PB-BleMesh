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


/************************************************************** 
 * COPYRIGHT(c)2018, Phy+.
 * All rights reserved. 
 *
 *
 * Module Name:	battery
 * File name:	battery.c 
 * Brief description:
 *    battery module
 * Author:	Eagle.Lao
 * Data:	2018-02-27
 * Revision:V0.01

****************************************************************/
#include "types.h"
#include "adc.h"
#include "battery.h"
#include "Sensor_Broadcast.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "string.h"
#include "gpio.h"
#include "gpio.h"
#include "error.h"
#include "log.h"

extern uint8 Sensor_Broadcast_TaskID;

enum{
	BATT_ST_NORMAL = 0,
	BATT_ST_CHARGING,
	BATT_ST_CHARGING_FULL,
};
  
#define PIN_BATT_CHARGE         P12
#define BATT_MEASURE_INTERVAL		60*1000 //1 minute interval for normal

//typedef struct{
//	uint8_t		percent;
//	uint16_t	volt;
//}batt_ref_t;

typedef struct _batt_ctx_t{
	bool            is_charging;
	uint16_t           vbat_value;
}batt_ctx_t;

static int batt_adc_init(void);

static batt_ctx_t s_batt_ctx;
static   adc_Cfg_t cfg = {
      .channel = ADC_BIT(ADC_CH3P_P20),
      .is_continue_mode = FALSE,
      .is_differential_mode = 0x00,
      .is_high_resolution = 0x00,
};

void batt_measure(void)
{
//	LOG("batt_measure\n");
  //Event handler is called immediately after conversion is finished.
  batt_adc_init();
}

uint16_t get_batt_value(uint16_t* buf, uint8_t size)
{
  uint8_t i;
  int adc_sum = 0;
  for (i = 0; i < size; i++) {
    adc_sum += buf[i];
  } 

  return (adc_sum/size)&0x0fff;
}


static void batt_adc_evt(adc_Evt_t* pev)
{
	bool is_high_resolution = FALSE;
	bool is_differential_mode = FALSE;	
	batt_ctx_t* p_ctx = &s_batt_ctx;
	
  if(pev->type == HAL_ADC_EVT_DATA){
//    p_ctx->vbat_value = get_batt_value(pev->data, pev->size);
		
		is_high_resolution = (cfg.is_high_resolution & ADC_BIT(ADC_CH3P_P20))?TRUE:FALSE;
		is_differential_mode = (cfg.is_differential_mode & ADC_BIT(ADC_CH3P_P20))?TRUE:FALSE;
		p_ctx->vbat_value = hal_adc_value_cal((adc_CH_t)(ADC_CH3P_P20),pev->data, pev->size, is_high_resolution,is_differential_mode);				
  }
//  LOG("batt:%d",p_ctx->vbat_value);
}


uint16_t batt_voltage(void)
{
  LOG("batt:%d",s_batt_ctx.vbat_value);
	return s_batt_ctx.vbat_value;
}

static int batt_adc_init(void)
{
	int ret;      
	ret = hal_adc_config_channel(cfg, batt_adc_evt);
  if(ret)
    return ret;

  hal_adc_start();
	return PPlus_SUCCESS;
}

//uint16_t batt_value_get(void){
//  uint16_t adc_data[MAX_ADC_SAMPLE_SIZE];
//  int n;
//  for (n = 0; n < MAX_ADC_SAMPLE_SIZE; n++) {
//    adc_data[n] = (uint16_t)(read_reg(ADC_CH_BASE + (ADC_CH3P * 0x80) + (n * 4))&0xfff);
//  }
//  uint16_t vbat_value_2=get_batt_value(adc_data, MAX_ADC_SAMPLE_SIZE);
//  
//  return vbat_value_2;
//}

int batt_init(void)
{
//	LOG("batt_init\n");
	memset(&s_batt_ctx, 0, sizeof(s_batt_ctx));

	batt_measure();
	return PPlus_SUCCESS;
}


