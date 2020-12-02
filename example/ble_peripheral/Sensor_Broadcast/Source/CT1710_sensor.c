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
  Filename:       CT1710_sensor.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

#include "gpio.h"
#include "log.h"
#include "OSAL.h"
#include "Sensor_Broadcast.h"
#include "pwrmgr.h"
#include "common.h"
#include "error.h"
#include "CT1710_sensor.h"

//extern void pad_ds_control(GPIO_Pin_e pin, BitAction_e value);

static uint8_t CT1710_init(void){
  uint8_t i;
//  hal_gpio_pull_set(P18,PULL_DOWN);
  hal_gpio_write(P18,0);                 //pull_down line
  WaitUs(450);                             //delay 450us to 650us
  hal_gpio_write(P18,1);
  hal_gpio_pin_init(P18, IE);
//  hal_gpio_pull_set(P18,STRONG_PULL_UP);        //waiting for CT1710 pull-low line,once CT1710 give response
  i=0;
  
  if(~hal_gpio_read(P18)){
    WaitUs(200);
    return PPlus_SUCCESS;
  }else{
    while(hal_gpio_read(P18))
  {
    WaitUs(10);
    i++;
    if(i>50){                    //if waiting time > 5ms
      return PPlus_ERR_TIMEOUT; //return fail,init fail
    }
  }   
  }
  
  
  return PPlus_SUCCESS;         //return success
}

static void CT1710_write_byte(uint8_t dat){
  uint8_t j;
  for(j=0;j<8;j++){
    hal_gpio_write(P18,0);              //pull-low line with 1us
    WaitUs(7);
    hal_gpio_write(P18,dat & 0x01);     //write one-bit data with LSB in first
    WaitUs(50);
    hal_gpio_write(P18,1);              //release single-wire line??to be ready for next byte
    dat >>=1;
    WaitUs(33);
    
  }
  
}

static uint8_t CT1710_read_byte(void){
  uint8_t byte=0,bi;
  uint32_t j;
  for(j=8;j>0;j--){
    hal_gpio_write(P18,0);              //pull-low line with 1us
    hal_gpio_write(P18,1);              //then release line
    hal_gpio_pin_init(P18, IE);         //then set gpio to ie...
    bi = hal_gpio_read(P18);            //Read Data from line, LSB in first
    /*move byte 1-bit to right, move bi 7-bit to left*/
    byte = (byte>>1)|(bi<<7);
    WaitUs(48);
  }
  return byte;
}

static void CT1710_temp_conv(void){
  CT1710_init();
  WaitUs(200);
  CT1710_write_byte(0xcc);              // Read register command
  WaitUs(50);
  CT1710_write_byte(0x44);              // Temp converter command
  WaitMs(45);
}

static void CT1710_read_temp_com(void){
  CT1710_init();
  WaitUs(200);
  CT1710_write_byte(0xcc);              // Read register command
  WaitUs(200);
  CT1710_write_byte(0xbe);              // Read register command
}

uint32_t CT1710_read_temp_degree(void){
  hal_pwrmgr_lock(MOD_USR2);
  uint32_t temp=0;
  uint8_t tmh,tml;
  CT1710_temp_conv();                   //Send Temp converter command, 0x44
  CT1710_read_temp_com();               //Read Temp
  WaitUs(200);
  tml = CT1710_read_byte();             //Read LSB for Temperature in first, Reg0_T_LSB
  WaitUs(200);
  tmh = CT1710_read_byte();             //Then read MSB, Reg1_T_MSB
  temp = tmh;
  temp <<= 8;
  temp |= tml;
  hal_pwrmgr_unlock(MOD_USR2);
  return temp;
}

void CT1710_sensor_init(void){
    hal_pwrmgr_register(MOD_USR2, NULL, NULL);        //power manage register 
}


