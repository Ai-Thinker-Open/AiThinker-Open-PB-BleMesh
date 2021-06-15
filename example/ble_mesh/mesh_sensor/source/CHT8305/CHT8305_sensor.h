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
#ifndef __CHT8305_SENSOR_H
#define __CHT8305_SENSOR_H
#include "types.h"

//#define byte unsigned char
//#define uint unsigned int
//#define bool unsigned char
//#define true  0x1
//#define false 0x0

#define CHT8305_DEV_ADDR  0x40 //0x40
#define CHT8305_REG_TEMP  0x00
#define CHT8305_REG_HUMI  0x01
#define CHT8305_REG_CONF  0x02
#define CHT8305_REG_MID   0xFE //0x5959
#define CHT8305_REG_DID   0xFF //0x8305

static const uint8_t BIT_T_RES       = 2;
static const uint8_t BIT_H_RES       = 0;
static const uint8_t BIT_BATTERY_OK  = 3;
static const uint8_t BIT_ACQ_MODE    = 4;
static const uint8_t BIT_HEATER      = 5;
static const uint8_t BIT_RST         = 7;

static const uint8_t T_RES_14 = 0;
static const uint8_t T_RES_11 = 1;

static const uint8_t H_RES_14 = 0;
static const uint8_t H_RES_11 = 1;
static const uint8_t H_RES_8 = 2;

void CHT8305_init(void);
bool CHT8305_is_connected(void);
uint32_t CHT8305_get_manufacturerID(void);
uint32_t CHT8305_get_deviceID(void);

void CHT8305_set_temperature_res(uint8_t res);
void CHT8305_set_humidity_res(uint8_t res);
void CHT8305_turnon_heater(bool heaterOn);
bool CHT8305_battery_OK(void);


void CHT8305_I2CInit(void);
void CHT8305_I2CDeinit(void);


uint8_t CHT8305_get_temp_humi(uint16_t *t, uint16_t *h);

#endif

