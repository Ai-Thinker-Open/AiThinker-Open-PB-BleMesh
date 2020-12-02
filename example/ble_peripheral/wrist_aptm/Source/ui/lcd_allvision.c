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
 * Module Name:	lcd driver
 * File name:	lcd_allvision.c 
 * Brief description:
 *	  lcd driver
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/

#if(CFG_DISP==DISP_OLED)

#include <stdint.h>
#include "lcd_allvision.h"
#include "i2c.h"
#include "types.h"
#include "hal_mcu.h"
#include "log.h"

#define STATE_MAX 0xFF
#define STATE_MIN 0x00
#define STATE_55 0x55
#define STATE_AA 0xAA
#define START_PAGE 0xB0
#define PAGE_TOTAL 4
#define START_HIGH_BIT 0x10
#define START_LOW_BIT 0x00
#define FRAME_HIGH_ROW 0x01
#define FRAME_LOW_ROW 0x80


static void* I2CInit(void)
{
	hal_i2c_pin_init(I2C_1, P28, P29);
  return hal_i2c_init(I2C_1, I2C_CLOCK_400K);
}
int I2CDeinit(void* pi2c)
{
  int ret = hal_i2c_deinit(pi2c);
	hal_gpio_pin_init(P28,IE);
  hal_gpio_pin_init(P29,IE);
	return ret;
}
static int I2CWrite(void* pi2c, uint8 reg, uint8 val)
{
  uint8 data[2];
  data[0] = reg;
  data[1] = val;
  hal_i2c_addr_update(pi2c, 0x78/2);
  {
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_tx_start(pi2c);
    hal_i2c_send(pi2c, data, 2);
    HAL_EXIT_CRITICAL_SECTION();
  }
  return hal_i2c_wait_tx_completed(pi2c);
}

static void WriteCmd(void* pi2c, uint8_t cmd)
{
	I2CWrite(pi2c, 0x00, cmd);
}

static void WriteData(void* pi2c, uint8_t data)
{
	I2CWrite(pi2c, 0x40, data);
}


int lcd_bus_init(void)
{
  return 0;
}

void lcd_init(void)
{
  //hal_gpio_pull_set(P6, PULL_DOWN);
  //WaitMs(20);
  //hal_gpio_pull_set(P4, STRONG_PULL_UP);
  void* pi2c = I2CInit();
	WriteCmd(pi2c, 0xAE); //display off
	WriteCmd(pi2c, 0x20);	//Set Memory Addressing Mode	
	WriteCmd(pi2c, 0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(pi2c, 0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(pi2c, 0xc8);	//Set COM Output Scan Direction
	WriteCmd(pi2c, 0x00); //---set low column address
	WriteCmd(pi2c, 0x10); //---set high column address
	WriteCmd(pi2c, 0x40); //--set start line address
	WriteCmd(pi2c, 0x81); //--set contrast control register
	WriteCmd(pi2c, 0xff); //���ȵ��� 0x00~0xff
	WriteCmd(pi2c, 0xa1); //--set segment re-map 0 to 127
	WriteCmd(pi2c, 0xa6); //--set normal display
	WriteCmd(pi2c, 0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(pi2c, 0x3F); //
	WriteCmd(pi2c, 0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(pi2c, 0xd3); //-set display offset
	WriteCmd(pi2c, 0x00); //-not offset
	WriteCmd(pi2c, 0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(pi2c, 0xf0); //--set divide ratio
	WriteCmd(pi2c, 0xd9); //--set pre-charge period
	WriteCmd(pi2c, 0x22); //
	WriteCmd(pi2c, 0xda); //--set com pins hardware configuration
	WriteCmd(pi2c, 0x12);
	WriteCmd(pi2c, 0xdb); //--set vcomh
	WriteCmd(pi2c, 0x20); //0x20,0.77xVcc
	WriteCmd(pi2c, 0x8d); //--set DC-DC enable
	WriteCmd(pi2c, 0x14); //
	WriteCmd(pi2c, 0xae); //--turn on oled panel
	I2CDeinit(pi2c);

}

void lcd_on(void)
{
    
  void* pi2c = I2CInit();
  WriteCmd(pi2c, 0X8D);
  WriteCmd(pi2c, 0X14);
  WriteCmd(pi2c, 0XAF);
  I2CDeinit(pi2c);
}

void lcd_off(void)
{
  
  //hal_gpio_pull_set(P6, STRONG_PULL_UP);
  //hal_gpio_write(P6, 1);
  //hal_gpio_write(P6, 1);
  //hal_gpio_pull_set(P4, PULL_DOWN);
  void* pi2c = I2CInit();
  WriteCmd(pi2c, 0X8D);
  WriteCmd(pi2c, 0X10);
  WriteCmd(pi2c, 0XAE);
  I2CDeinit(pi2c);
}


int lcd_draw(uint8_t page_s, uint8_t x, uint8_t page_e, uint8_t width, const uint8_t* data)
{

	 uint8_t page_number,column_number;
	 
	 void* pi2c = I2CInit();
	 for(page_number=page_s;page_number<=page_e;page_number++)
	 {
		WriteCmd(pi2c, START_PAGE+page_number);

		WriteCmd(pi2c, START_HIGH_BIT + (x>>4));
		WriteCmd(pi2c, (x&0xf)|0x01); 
	 
		for(column_number=0;column_number<width;column_number++)
		   WriteData(pi2c, data[page_number*SCN_WIDTH + x+column_number ]);
	}
	I2CDeinit(pi2c);
	return 0;
}

#endif/*CFG_DISP==DISP_OLED*/


