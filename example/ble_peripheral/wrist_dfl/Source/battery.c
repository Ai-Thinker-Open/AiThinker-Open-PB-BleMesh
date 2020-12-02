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
#include "app_wrist.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "string.h"
#include "gpio.h"
#include "gpio.h"
#include "error.h"
#include "log.h"


enum{
	BATT_ST_NORMAL = 0,
	BATT_ST_CHARGING,
	BATT_ST_CHARGING_FULL,
};
  
#define PIN_BATT_CHARGE         P12
#define BATT_MEASURE_INTERVAL		30*1000 //1 minute interval for normal
#define BATT_MEASURE_INTERVAL_FIRST		500 //500ms wait for first time sampling

typedef struct{
	uint8_t		percent;
	uint16_t	volt;
}batt_ref_t;

typedef struct _batt_ctx_t{
	bool            is_charging;
	int             vbat_value_int;
	float           vbat_value;
}batt_ctx_t;

static int batt_adc_init(void);

static batt_ctx_t s_batt_ctx;

static adc_Cfg_t cfg = {
      .channel = ADC_BIT(ADC_CH1N_P11),
      .is_continue_mode = FALSE,
      .is_differential_mode = 0x00,
      .is_high_resolution = 0xff,
};
	
static int batt_timer_start(uint32_t intval_ms)
{
    osal_start_timerEx(AppWrist_TaskID, TIMER_BATT_EVT, intval_ms);
    return 0;
}

void batt_measure(void)
{
	LOG("batt_measure\n");
  //Event handler is called immediately after conversion is finished.
  batt_adc_init();
}

void batt_meas_timeout_handler(void)
{
	batt_measure();
  batt_timer_start(BATT_MEASURE_INTERVAL);
}

static void batt_adc_evt(adc_Evt_t* pev)
{
	bool is_high_resolution = FALSE;
	bool is_differential_mode = FALSE;	
	batt_ctx_t* p_ctx = &s_batt_ctx;
	
  if(pev->type == HAL_ADC_EVT_DATA){
    //p_ctx->vbat_value = hal_adc_value_cal(pev->ch,pev->data, pev->size, TRUE, FALSE);
		is_high_resolution = (cfg.is_high_resolution & ADC_BIT(ADC_CH1N_P11))?TRUE:FALSE;
		is_differential_mode = (cfg.is_differential_mode & ADC_BIT(ADC_CH1N_P11))?TRUE:FALSE;
		p_ctx->vbat_value = hal_adc_value_cal((adc_CH_t)(ADC_CH1N_P11),pev->data, pev->size, is_high_resolution,is_differential_mode);		
    p_ctx->vbat_value = p_ctx->vbat_value*4.7;//11.0;//4.3/0.9;
    p_ctx->vbat_value_int = (int)p_ctx->vbat_value*1000;
  	LOG("batt_measure_evt %d\n",(int)(p_ctx->vbat_value*1000));
    osal_set_event(AppWrist_TaskID, BATT_VALUE_EVT);
  }
}


float batt_voltage(void)
{
	return s_batt_ctx.vbat_value;
}

int batt_voltage_int(void)
{
	return s_batt_ctx.vbat_value_int;
}
bool batt_charge_detect(void)
{
	return s_batt_ctx.is_charging;
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



static void batt_charge_evt_hdl(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{
  LOG("batt_charge_evt_hdl %d\n", type);
	if(type == POSEDGE){
    if(s_batt_ctx.is_charging == FALSE){
      s_batt_ctx.is_charging = TRUE;
      osal_set_event(AppWrist_TaskID, BATT_CHARGE_EVT);
    }
	}
  else
	{
    if(s_batt_ctx.is_charging){
      s_batt_ctx.is_charging = FALSE;
      osal_set_event(AppWrist_TaskID, BATT_CHARGE_EVT);
    }
	}
}


static int batt_charge_io_init(void)
{
	int ret;

  hal_gpio_pull_set(P11, FLOATING);
  
	ret = hal_gpioin_register(PIN_BATT_CHARGE, batt_charge_evt_hdl, batt_charge_evt_hdl);

	if(ret != PPlus_SUCCESS){
		return ret;
	}
	return PPlus_SUCCESS;
}

int batt_init(void)
{
	LOG("batt_init\n");
	memset(&s_batt_ctx, 0, sizeof(s_batt_ctx));

	batt_charge_io_init();

  batt_timer_start(BATT_MEASURE_INTERVAL_FIRST);

	return PPlus_SUCCESS;
}


