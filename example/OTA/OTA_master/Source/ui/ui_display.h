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
 *
 * Module Name:	display
 * File name:	ui_display.h 
 * Brief description:
 *    UI display drvier
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/
#if(CFG_DISP==DISP_OLED)

#ifndef _UI_DISPLAY_HEAD_
#define _UI_DISPLAY_HEAD_
#include <stdint.h>
#include "res_tab.h"
#include "lcd_allvision.h"

#define FRAME_BUF_SIZE		(SCN_WIDTH*SCN_HEIGHT/8)

#define FONT_BITMAP_SIZE (32+4)

#define DISP_POINT(p, px, py) {p.x = px; p.y = py;}

#define DISP_RECT(rect, px, py, pw, ph) 	{	rect.x = px;\
											rect.y = py;\
											rect.w = pw;\
											rect.h = ph;}

#define DISP_RECT_FULL(rect) 	{	rect.x = 0;\
									rect.y = 0;\
									rect.w = SCN_WIDTH;\
									rect.h = SCN_HEIGHT;}

#define DISP_RECT_EMPTY(rect) 	{	rect.x = 0;\
									rect.y = 0;\
									rect.w = 0;\
									rect.h = 0;}




#define RECT_V2H(rect)	{uint16_t tmp = rect.x; rect.x = rect.y; rect.y = SCN_HEIGHT-tmp + 1 - rect.w; tmp = rect.w; rect.w = rect.h;rect.h = tmp;}
#define RECT_H2V(rect)
#define POINT_V2H(rect)
#define POINT_H2V(rect)



enum{
	DIGI_TM = 0,
	DIGI_1,
	DIGI_2,
	DIGI_3
};


enum{
	UI_DISP_OFF = 0,
	UI_DISP_ON = 1
};

typedef struct{
	int16_t x;
	int16_t y;
	int16_t w;
	int16_t h;
}disp_rect_t;

typedef struct{
	int16_t x;
	int16_t y;
}disp_point_t;


void disp_init(void);
void disp_off(void);
void disp_on(void);
void disp_reflash(void);
void disp_reflash_all(void);
void disp_bri_style(uint8_t br,uint8_t style);

disp_rect_t disp_draw_rect(disp_rect_t rect);
disp_rect_t disp_clrscn(disp_rect_t rect);
disp_rect_t disp_character(disp_point_t point,  char ch);
disp_rect_t disp_unicode(disp_point_t point, uint16_t unicode);
disp_rect_t disp_string_u(disp_point_t point, const uint16_t *unicode);
disp_rect_t disp_string(disp_point_t point, const char* utf8);
disp_rect_t disp_digi(disp_point_t point, uint16_t type, char* str);
disp_rect_t disp_idb(disp_point_t point, uint16_t idb);

disp_rect_t disp_character_f(disp_point_t point,  char ch);
disp_rect_t disp_unicode_f(disp_point_t point, uint16_t unicode);
disp_rect_t disp_string_u_f(disp_point_t point, const uint16_t *unicode);
disp_rect_t disp_string_f(disp_point_t point, const char* utf8);
disp_rect_t disp_digi_f(disp_point_t point, uint16_t type, char* str);
disp_rect_t disp_idb_f(disp_point_t point, uint16_t idb);

#ifdef TEST_ENTRY

void disp_test(void);

#endif

#endif

#endif/*CFG_DISP==DISP_OLED*/

