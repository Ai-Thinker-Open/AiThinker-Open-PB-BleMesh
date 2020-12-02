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
 * File name:	ui_display.c 
 * Brief description:
 *    UI display drvier
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/

#if(CFG_DISP==DISP_OLED)

#include <stdint.h>
#include <string.h>
#include "ui_display.h"
#include "ui_font.h"
#include "res_tab.h"
#include "app_err.h"
#include "ap_cp.h"
#include "log.h"



typedef struct{
	uint16_t status;		//screen status 0 is off 1, is on
	void* font;
	uint16_t layer_cnt;		//pending layer count that not flushed to screen
	disp_rect_t rect;		//the layer rectangle area that need to flush to screen, if the blocks is more than 2, flush full sceen 
	uint8_t fb[FRAME_BUF_SIZE];	//frame buffer
}ui_disp_t;

static ui_disp_t s_disp;

extern const unsigned char*  bmp_table[NUM_BITMAP];
static void lcd_update_rect(disp_rect_t rect, const uint8_t* data)
{
	uint16_t page_s = rect.y/8, page_e = (rect.y+rect.h-1)/8;

	lcd_draw(page_s, rect.x, page_e, rect.w, data);
}

static __INLINE uint8_t GET_PIXEL_X(int16_t x, int16_t y, uint16_t img_w, const uint8_t* img) {
//static uint8_t GET_PIXEL_X(int16_t x, int16_t y, uint16_t img_w, const uint8_t* img) {
	uint8_t pix = img[x/8 + y*((img_w+7)/8)];
	return (uint8_t)((pix>>(7-(x%8))) &1);
	
}

static __INLINE uint8_t GET_PIXEL(int16_t x, int16_t y, uint16_t img_w, const uint8_t* img) {
	uint8_t pix = img[x + (y/8)*img_w];
	return (uint8_t)((pix>>(y%8)) &1);
	
}
static __INLINE void SET_PIXEL(int16_t x, int16_t y, uint8_t pixel) {
//static void SET_PIXEL(int16_t x, int16_t y, uint8_t pixel) {
	int16_t offset = x + (y/8)*SCN_WIDTH;
	if(x >= SCN_WIDTH || y >= SCN_HEIGHT || x <0 || y <0)
		return;
	uint8_t* ppix = &s_disp.fb[offset];
	uint8_t mask = 1 << (y%8);
	//LOG("sp:%d, %d, %d\n", offset, x,y);
	*ppix = (pixel == 1) ? ((*ppix )| mask) : ((*ppix )& (~mask)) ;
}

static __INLINE void SET_PIXEL_f(int16_t x, int16_t y, uint8_t pixel) {
	int16_t x_o = y, y_o = SCN_HEIGHT-x;
	int16_t offset = x_o + (y_o/8)*SCN_WIDTH;
	if(x_o >= SCN_WIDTH || y_o >= SCN_HEIGHT || x_o <0 || y_o <0)
		return;
	uint8_t* ppix = &s_disp.fb[offset];
	uint8_t mask = 1 << (y_o % 8);
	//LOG("sp:%d, %d, %d\n", offset, x,y);
	*ppix = (pixel == 1) ? ((*ppix )| mask) : ((*ppix )& (~mask)) ;
}
static disp_rect_t disp_img_x(disp_point_t point, const uint8_t* img)
{
	//ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	int16_t i,j;
	uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1],bmp_w = (uint16_t)img[3];
	int16_t x = point.x, y = point.y;
  if(bmp_w == 0)
    bmp_w = img_w;
	DISP_RECT_EMPTY(rect);

		//case not aligned to 8 pixel
		for(j = 0; j <img_h; j++){
			for(i = 0; i<bmp_w; i++ ){
				SET_PIXEL(i+x, j+y, (GET_PIXEL_X(i,j,img_w,img+4)));
			}
		}
		
	rect.x = point.x;
	rect.y = point.y - (point.y % 8);
	rect.w = bmp_w;
	rect.h = ((img_h + 7)/8) * 8;
	return rect;
}

static disp_rect_t disp_img(disp_point_t point, const uint8_t* img)
{
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	int16_t i,j;
	uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1];
	int16_t x = point.x, y = point.y;
	
	DISP_RECT_EMPTY(rect);

	if((point.y%8) == 0 && (img_h % 8) == 0){
		//case y-axis and height image is aligned to 8 pixel
		for(j = 0; j <img_h/8; j++){
			for(i = 0; i<img_w; i++ )
				if(((j + point.y/8) >=0) && 
					((i+point.x)>=0) &&
					((i+point.x) <= SCN_WIDTH)&&
					((j + point.y/8) <=SCN_WIDTH/8)){
					pdisp->fb[SCN_WIDTH * (j+point.y/8) + i+point.x] = img[4 + j*img_w + i];
				}
		}
	}
	else 
	{
		//case not aligned to 8 pixel
		for(j = 0; j <img_h; j++){
			for(i = 0; i<img_w; i++ ){
				SET_PIXEL(i+x, j+y, (GET_PIXEL(i,j,img_w,img+4)));
			}
		}
	}
		
	rect.x = point.x;
	rect.y = point.y - (point.y % 8);
	rect.w = img_w;
	rect.h = ((img_h + 7)/8) * 8;
	return rect;
}




static disp_rect_t disp_img_f(disp_point_t point, const uint8_t* img)
{
	//ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	int16_t i,j;
	uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1];
	int16_t x = point.x, y = point.y;
	
	DISP_RECT_EMPTY(rect);

	//case not aligned to 8 pixel
	for(j = 0; j <img_h; j++){
		for(i = 0; i<img_w; i++ ){
			SET_PIXEL_f(i+x, j+y, (GET_PIXEL(i,j,img_w,img+4)));
		}
	}
		
	rect.x = point.x;
	rect.y = point.y - (point.y % 8);
	rect.w = img_w;
	rect.h = ((img_h + 7)/8) * 8;
	return rect;
}

static disp_rect_t disp_img_fx(disp_point_t point, const uint8_t* img)
{
	//ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	int16_t i,j;
	uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1],bmp_w = (uint16_t)img[3];
	int16_t x = point.x, y = point.y;
	
  if(bmp_w == 0)
    bmp_w = img_w;
	DISP_RECT_EMPTY(rect);

	//case not aligned to 8 pixel
	for(j = 0; j <img_h; j++){
		for(i = 0; i<bmp_w; i++ ){
			SET_PIXEL_f(i+x, j+y, (GET_PIXEL_X(i,j,img_w,img+4)));
		}
	}
		
	rect.x = point.x;
	rect.y = point.y - (point.y % 8);
	rect.w = bmp_w;
	rect.h = ((img_h + 7)/8) * 8;
	return rect;
}


void disp_reflash(void){
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t  rect;
	DISP_RECT_FULL(rect);
	
	if(pdisp->layer_cnt == 1)
		rect = pdisp->rect;
	
	lcd_update_rect(rect, (const uint8_t *)pdisp->fb);
	
	pdisp->layer_cnt = 0;
	DISP_RECT_EMPTY(pdisp->rect);
}

void disp_reflash_all(void){
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	DISP_RECT_FULL(rect);
	
	lcd_update_rect(rect, (const uint8_t *)pdisp->fb);
	pdisp->layer_cnt = 0;
	DISP_RECT_EMPTY(pdisp->rect);
}

void disp_off(void)
{
	ui_disp_t* pdisp = &s_disp;
	lcd_off();
	pdisp->status = UI_DISP_OFF;
}

void disp_on(void)
{
	ui_disp_t* pdisp = &s_disp;
	lcd_on();
	pdisp->status = UI_DISP_ON;
}


void disp_init(void) 
{
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	//uint16_t unicode[2] = {0x4e00,0};

	
	lcd_init();

	memset(pdisp, 0, sizeof(ui_disp_t));
	pdisp->status = UI_DISP_OFF;
	pdisp->layer_cnt = 0;
	pdisp->font = ui_font_load(0x11100000);
	//pdisp->font = ui_font_load(0x11012000);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);
	disp_reflash_all();
	//disp_on();
	//while(1){
  //  disp_point_t point = {0,0};
  //  unicode[0]++;
  	//disp_string_u(point, (const uint16_t*)unicode);
  //	disp_string(point, "，。hello asf");
  //	disp_reflash_all();
	//}

}



disp_rect_t disp_clrscn(disp_rect_t    rect)
{
	ui_disp_t* pdisp = &s_disp;
	uint16_t i,j,x,y;
	x = rect.x;
	y = rect.y;
  if(y%8 == 0 && rect.h %8 == 0){
  	for(j = 0; j <rect.h/8; j++){
  		for(i = 0; i<rect.w; i++ ){
  			//LOG("p is %d\n",SCN_WIDTH * (j+y/8) + i+x);
  			pdisp->fb[SCN_WIDTH * (j+y/8) + i+x] = 0;
  		}
  	}
  }
  else
  {
    uint16_t img_w = rect.w, img_h = rect.h;
    x = rect.x;
    y = rect.y;
    
    
    //case not aligned to 8 pixel
    for(j = 0; j <img_h; j++){
      for(i = 0; i<img_w; i++ ){
        SET_PIXEL(i+x, j+y, 0);
      }
    }

  }
	pdisp->layer_cnt ++;
	pdisp->rect = rect;

	return rect;
}

disp_rect_t disp_draw_rect(disp_rect_t rect)
{
	int16_t i,j;
	uint16_t img_w = rect.w, img_h = rect.h;
	int16_t x = rect.x, y = rect.y;
	

	//case not aligned to 8 pixel
	for(j = 0; j <img_h; j++){
		for(i = 0; i<img_w; i++ ){
			SET_PIXEL(i+x, j+y, 1);
		}
	}
		
	return rect;	
}


disp_rect_t disp_character(disp_point_t point,  char ch)
{
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	char tmp = ch;

	DISP_RECT_EMPTY(rect);
	
	if(ch <' ')
		tmp = ' ';
	rect = disp_img(point, bmp_table[(uint16_t)tmp-0x20]);

	
	pdisp->layer_cnt ++;
	pdisp->rect = rect;
	 
	return rect;
}


disp_rect_t disp_unicode(disp_point_t point, uint16_t unicode)
{
	int ret;
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	DISP_RECT_EMPTY(rect);
	
	uint8_t bitmap[FONT_BITMAP_SIZE];
	ret = ui_font_unicode(pdisp->font, unicode,bitmap);
	if(ret != APP_SUCCESS)
		return rect;
	
	pdisp->layer_cnt ++;
	pdisp->rect = rect;

	return disp_img_x(point, bitmap);

}

disp_rect_t disp_string_u(disp_point_t point, const uint16_t *unicode)
{
	int ret;
	uint8_t bitmap[FONT_BITMAP_SIZE];
	uint16_t offset = 0;
	ui_disp_t* pdisp = &s_disp;

	disp_point_t point_new = point;
	disp_rect_t rect;
	disp_rect_t rect1;
	
	DISP_RECT_EMPTY(rect);
	DISP_RECT_EMPTY(rect1);
	//20 is the max size of the screen area for font
	for(offset = 0; offset < 20; offset ++)
	{
		if(unicode[offset] == 0)
			break;
		//ascII area
		//if(unicode[offset] <128){
		//	rect1 = disp_character(point_new, (char)unicode[offset]);
		//	point_new.x += rect1.w + 1;
		//	continue;
		//}
		
		ret = ui_font_unicode(pdisp->font, unicode[offset],bitmap);
		if(ret != APP_SUCCESS)
			continue;
		rect1 = disp_img_x(point_new, bitmap);
		point_new.x += rect1.w + 1;
				
	}
	rect.x = point.x;
	rect.y = point.y;
	rect.w = point_new.x + 1 - point.x;
	rect.h = rect1.h + rect1.y - rect.y;
	return rect;

}


disp_rect_t disp_string(disp_point_t point, const char *utf8)
{
	int ret;
	const char* utf8_str = utf8;
	uint16_t unic_ch;
	uint8_t bitmap[FONT_BITMAP_SIZE];
	uint16_t offset;
	ui_disp_t* pdisp = &s_disp;
	
	disp_point_t point_new = point;
	disp_rect_t rect;
	disp_rect_t rect1;
	
	DISP_RECT_EMPTY(rect);
	DISP_RECT_EMPTY(rect1);
	while(1)
	{
		offset = utf8_to_unicode(utf8_str, &unic_ch);
		if(offset == 0)
			break;
		//LOG("\n%4x\n",unic_ch);
		utf8_str += offset;
		//ascII area
		//if(unic_ch <128){
		//	rect1 = disp_character(point_new, (char)unic_ch);
		//	point_new.x += rect1.w + 1;
		//	continue;
		//}
		
		ret = ui_font_unicode(pdisp->font, unic_ch,bitmap);
		if(ret != APP_SUCCESS)
			continue;
		rect1 = disp_img_x(point_new, bitmap);
		point_new.x += rect1.w;
				
	}
	rect.x = point.x;
	rect.y = point.y;
	rect.w = point_new.x + 1 - point.x;
	rect.h = rect1.h + rect1.y - rect.y;
	return rect;

}


//display digital big
//flg: 0 is for big font, 1 is for small font
disp_rect_t disp_digi(disp_point_t point, uint16_t type, char* str)
{
	disp_rect_t rect;
	disp_rect_t rect1;
	disp_point_t point_new = point;
	
	uint16_t i = 0;
	uint16_t size = (uint16_t)strlen(str);
	
	uint16_t id_offset;
	char ch;

	switch(type){
	case DIGI_TM:
		id_offset = IDB_TM_0;
		break;
	
	case DIGI_1:
		id_offset = IDB_N1_0;
		break;
	case DIGI_2:
		id_offset = IDB_N2_0;
		break;
	case DIGI_3:
		id_offset = IDB_N3_0;
		break;
	default:
		id_offset = IDB_N1_0;
		break;
	}

	DISP_RECT_EMPTY(rect);
	DISP_RECT_EMPTY(rect1);
	
	for(i = 0; i< size; i++){
		ch = str[i];
		
		if(ch < '0' || ch > '9')
		{
			point_new.x += 8;
			continue;
		}
		
		rect1 = disp_idb(point_new, id_offset + (ch - '0')); 
		point_new.x += rect1.w+1;
	}
	rect.x = point.x;
	rect.y = point.y;
	rect.w = point_new.x + 1 - point.x;
	rect.h = rect1.h + rect1.y - rect.y;
	return rect;
}



disp_rect_t disp_idb(disp_point_t point, uint16_t idb){
	disp_rect_t rect;

	DISP_RECT_EMPTY(rect);
	
	if(idb >NUM_BITMAP)
		return rect;
	return disp_img(point, bmp_table[idb]);
}


disp_rect_t disp_character_f(disp_point_t point,  char ch)
{
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	char tmp = ch;

	DISP_RECT_EMPTY(rect);
	
	if(ch <' ')
		tmp = ' ';
	rect = disp_img_f(point, bmp_table[(uint16_t)tmp-0x20]);

	
	pdisp->layer_cnt ++;
	pdisp->rect = rect;
	 
	return rect;

}

disp_rect_t disp_unicode_f(disp_point_t point, uint16_t unicode)
{
	int ret;
	ui_disp_t* pdisp = &s_disp;
	disp_rect_t rect;
	DISP_RECT_EMPTY(rect);
	
	uint8_t bitmap[FONT_BITMAP_SIZE];
	ret = ui_font_unicode(pdisp->font, unicode,bitmap);
	if(ret != APP_SUCCESS)
		return rect;

	pdisp->layer_cnt ++;
	pdisp->rect = rect;

	return disp_img_fx(point, bitmap);


}
disp_rect_t disp_string_u_f(disp_point_t point, const uint16_t *unicode)
{
	int ret;
	uint8_t bitmap[FONT_BITMAP_SIZE];
	uint16_t offset = 0;
	ui_disp_t* pdisp = &s_disp;
	
	disp_point_t point_new = point;
	disp_rect_t rect;
	disp_rect_t rect1;
	
	DISP_RECT_EMPTY(rect);
	DISP_RECT_EMPTY(rect1);
	//20 is the max size of the screen area for font
	for(offset = 0; offset < 20; offset ++)
	{
		if(unicode[offset] == 0)
			break;
		//ascII area
		//if(unicode[offset] <128){
		//	rect1 = disp_character_f(point_new, (char)unicode[offset]);
		//	point_new.x += rect1.w + 1;
		//	continue;
		//}
		
		ret = ui_font_unicode(pdisp->font, unicode[offset],bitmap);
		if(ret != APP_SUCCESS)
			continue;
		rect1 = disp_img_fx(point_new, bitmap);
		point_new.x += rect1.w + 1;
				
	}
	rect.x = point.x;
	rect.y = point.y;
	rect.w = point_new.x + 1 - point.x;
	rect.h = rect1.h + rect1.y - rect.y;
	return rect;

}

disp_rect_t disp_string_f(disp_point_t point, const char* utf8)
{
	int ret;
	const char* utf8_str = utf8;
	uint16_t unic_ch;
	uint8_t bitmap[FONT_BITMAP_SIZE];
	uint16_t offset;
	ui_disp_t* pdisp = &s_disp;
	
	disp_point_t point_new = point;
	disp_rect_t rect;
	disp_rect_t rect1;
	
	DISP_RECT_EMPTY(rect);
	DISP_RECT_EMPTY(rect1);
	
	while(1)
	{
		offset = utf8_to_unicode(utf8_str, &unic_ch);
		if(offset == 0)
			break;
		//LOG("\n%4x\n",unic_ch);
		utf8_str += offset;
		//ascII area
		//if(unic_ch <128){
		//	rect1 = disp_character_f(point_new, (char)unic_ch);
		//	point_new.x += rect1.w + 1;
		//	continue;
		//}
		
		ret = ui_font_unicode(pdisp->font, unic_ch,bitmap);
		if(ret != APP_SUCCESS)
			continue;
		rect1 = disp_img_fx(point_new, bitmap);
		point_new.x += rect1.w + 1;
				
	}
	
	rect.x = point.x;
	rect.y = point.y;
	rect.w = point_new.x + 1 - point.x;
	rect.h = rect1.h + rect1.y - rect.y;
	
	return rect;

}

disp_rect_t disp_digi_f(disp_point_t point, uint16_t type, char* str)
{
	disp_rect_t rect;
	disp_rect_t rect1;
	disp_point_t point_new = point;
	
	uint16_t i = 0;
	uint16_t size = (uint16_t)strlen(str);
	
	uint16_t id_offset;
	char ch;

	switch(type){
	case DIGI_TM:
		id_offset = IDB_TM_0;
		break;
	
	case DIGI_1:
		id_offset = IDB_N1_0;
		break;
	case DIGI_2:
		id_offset = IDB_N2_0;
		break;
	case DIGI_3:
		id_offset = IDB_N3_0;
		break;
	default:
		id_offset = IDB_N1_0;
		break;
	}

	DISP_RECT_EMPTY(rect);
	DISP_RECT_EMPTY(rect1);
	
	for(i = 0; i< size; i++){
		ch = str[i];
		
		if(ch < '0' || ch > '9')
		{
			point_new.x += 8;
			continue;
		}
		
		rect1 = disp_idb_f(point_new, id_offset + (ch - '0')); 
		point_new.x += rect1.w+1;
	}
	rect.x = point.x;
	rect.y = point.y;
	rect.w = point_new.x + 1 - point.x;
	rect.h = rect1.h + rect1.y - rect.y;
	return rect;

}


disp_rect_t disp_idb_f(disp_point_t point, uint16_t idb)
{
	disp_rect_t rect;

	DISP_RECT_EMPTY(rect);
	
	if(idb >NUM_BITMAP)
		return rect;
	return disp_img_f(point, bmp_table[idb]);

}




#ifdef TEST_ENTRY

void disp_test(void)
{
	//disp_rect_t rect;
	disp_point_t point;
	disp_on();
	point.x = 2, point.y = 1;
	disp_digi_f(point, DIGI_1, "13");

	
	
	//disp_idb(point, IDB_W_HEAD);
	//disp_idb(point, IDB_ICON_alarmclock);
	disp_reflash_all();


}

#endif
#endif/*CFG_DISP==DISP_OLED*/


