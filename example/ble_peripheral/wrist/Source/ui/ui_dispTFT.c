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
 * Module Name: display
 * File name: ui_display.c 
 * Brief description:
 *    UI display drvier
 * Author:  Eagle.Lao
 * Revision:V0.01
****************************************************************/

#if(CFG_DISP==DISP_TFT)

#include <stdint.h>
#include <string.h>
#include "ui_dispTFT.h"
#include "ui_font.h"
#include "res_tab.h"
#include "app_err.h"
#include "ap_cp.h"
#include "error.h"
#include "log.h"



typedef struct{
  uint16_t status;    //screen status 0 is off 1, is on
  void*   font;
  uint8_t fbw;  //frame buffer width
  uint8_t fbh;  //frame buffer height
  uint16_t fb[FRAME_BUF_SIZE]; //frame buffer
}ui_disp_t;

static ui_disp_t s_disp;

extern const unsigned char*  bmp_table[NUM_BITMAP];


static __INLINE int FB_SET(uint16_t img_w, uint16_t img_h) {
  uint16_t fb_size = img_w * img_h;
  s_disp.fbw = 0;
  s_disp.fbh = 0;
  if(fb_size > FRAME_BUF_SIZE)
    return PPlus_ERR_NO_MEM;
  s_disp.fbw = img_w;
  s_disp.fbh = img_h;
  memset(s_disp.fb, 0, fb_size*2);
  return PPlus_SUCCESS;
}

static __INLINE int FB_SET_f(uint16_t img_w, uint16_t img_h) {
  uint16_t fb_size = img_w*img_h;
  s_disp.fbw = 0;
  s_disp.fbh = 0;
  if(fb_size > FRAME_BUF_SIZE)
    return PPlus_ERR_NO_MEM;
  s_disp.fbw = img_h;
  s_disp.fbh = img_w;
  memset(s_disp.fb, 0, fb_size*2);
  return PPlus_SUCCESS;
}

static __INLINE uint8_t GET_PIXEL_X(int16_t x, int16_t y, uint16_t img_w, const uint8_t* img) {
//static uint8_t GET_PIXEL_X(int16_t x, int16_t y, uint16_t img_w, const uint8_t* img) {
	uint8_t pix = img[x/8 + y*((img_w+7)/8)];
	return (uint8_t)((pix>>(7-(x%8))) &1);
	
}
static __INLINE uint8_t GET_PIXEL(uint16_t x, uint16_t y, uint16_t img_w, const uint8_t* img) {
  uint8_t pix = img[x + (y/8)*img_w];
  return (uint8_t)((pix>>(y%8)) &1);
  
}
static __INLINE void SET_PIXEL(uint16_t x, uint16_t y, uint16_t color) {
  if(x >= s_disp.fbw || y >= s_disp.fbh)
    return;
  uint16_t* ppix = &s_disp.fb[s_disp.fbw*y + x];
  //LOG("sp:%d, %d, %d\n", offset, x,y);
  *ppix = color;
}

static __INLINE void SET_PIXEL_f(int16_t x, int16_t y, uint16_t color) {
  int16_t x_o = y, y_o = s_disp.fbh - x - 1;
  if(x_o >= s_disp.fbw || y_o >= s_disp.fbh)
    return;
  uint16_t* ppix = &s_disp.fb[s_disp.fbw*y_o + x_o];
  //LOG("sp:%d, %d, %d\n", offset, x,y);
  *ppix = color;
}

static disp_rect_t disp_img_x(disp_point_t point, const uint8_t* img, uint16_t color)
{
  int ret;
  disp_rect_t rect;
  int16_t i,j;
  uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1];
  
  DISP_RECT_EMPTY(rect);
  
  ret = FB_SET(img_w, img_h);

  for(i = 0; i <img_w; i++){
    for(j = 0; j<img_h; j++ ){
      if(GET_PIXEL_X(i,j,img_w,img+4))
        SET_PIXEL(i, j, color);
    }
  }
  
  rect.x = point.x;
  rect.y = point.y;
  rect.w = img_w;
  rect.h = img_h;
  if(ret == PPlus_SUCCESS)
    //lcd_setscn_TFT(rect.x,rect.y,rect.w,rect.h, RGB_Yellow);
		lcd_draw_TFT(rect.x,rect.y,rect.w, rect.h,(const uint16_t*)s_disp.fb);
  
  return rect;
}



static disp_rect_t disp_img(disp_point_t point, const uint8_t* img, uint16_t color)
{
  int ret;
  disp_rect_t rect;
  int16_t i,j;
  uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1];
  
  DISP_RECT_EMPTY(rect);
  
  ret = FB_SET(img_w, img_h);

  for(i = 0; i <img_w; i++){
    for(j = 0; j<img_h; j++ ){
      if(GET_PIXEL(i,j,img_w,img+4))
        SET_PIXEL(i, j, color);
    }
  }
  
  rect.x = point.x;
  rect.y = point.y;
  rect.w = img_w;
  rect.h = img_h;
  if(ret == PPlus_SUCCESS)
    //lcd_setscn_TFT(rect.x,rect.y,rect.w,rect.h, RGB_Yellow);
		lcd_draw_TFT(rect.x,rect.y,rect.w, rect.h,(const uint16_t*)s_disp.fb);
  
  return rect;
}


static disp_rect_t disp_img_fx(disp_point_t point, const uint8_t* img, uint16_t color)
{
  int ret;
  disp_rect_t rect;
  int16_t i,j;
  uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1];
  
  DISP_RECT_EMPTY(rect);


  ret = FB_SET_f(img_w, img_h);

  //case not aligned to 8 pixel
  for(i = 0; i <img_w; i++){
    for(j = 0; j<img_h; j++ ){
      if(GET_PIXEL_X(i,j,img_w,img+4))
        SET_PIXEL_f(i, j, color);
    }
  }

  rect.x = point.x;
  rect.y = point.y;
  rect.w = img_w;
  rect.h = img_h;
  if(ret == PPlus_SUCCESS){
    disp_rect_t rect_f = rect;
    RECT_V2H(rect_f);
    lcd_draw_TFT(rect_f.x,rect_f.y,rect_f.w,rect_f.h, (const uint16_t*)s_disp.fb);
    //lcd_setscn_TFT(rect_f.x,rect_f.y,rect_f.w,rect_f.h, RGB_Yellow);
  }
  return rect;
}



static disp_rect_t disp_img_f(disp_point_t point, const uint8_t* img, uint16_t color)
{
  int ret;
  disp_rect_t rect;
  int16_t i,j;
  uint16_t img_w = (uint16_t)img[0], img_h = (uint16_t)img[1];
  
  DISP_RECT_EMPTY(rect);


  ret = FB_SET_f(img_w, img_h);

  //case not aligned to 8 pixel
  for(i = 0; i <img_w; i++){
    for(j = 0; j<img_h; j++ ){
      if(GET_PIXEL(i,j,img_w,img+4))
        SET_PIXEL_f(i, j, color);
    }
  }

  rect.x = point.x;
  rect.y = point.y;
  rect.w = img_w;
  rect.h = img_h;
  if(ret == PPlus_SUCCESS){
    disp_rect_t rect_f = rect;
    RECT_V2H(rect_f);
    lcd_draw_TFT(rect_f.x,rect_f.y,rect_f.w,rect_f.h, (const uint16_t*)s_disp.fb);
    //lcd_setscn_TFT(rect_f.x,rect_f.y,rect_f.w,rect_f.h, RGB_Yellow);
  }
  return rect;
}



void disp_off(void)
{
  ui_disp_t* pdisp = &s_disp;
  lcd_off_TFT();
  pdisp->status = UI_DISP_OFF;
}

void disp_on(void)
{
  ui_disp_t* pdisp = &s_disp;
  lcd_on_TFT();
  pdisp->status = UI_DISP_ON;
}


void disp_init(void) 
{
  ui_disp_t* pdisp = &s_disp;
  disp_rect_t rect;

	pdisp->font = ui_font_load(0x11012000);
  
  lcd_init();

  memset(pdisp, 0, sizeof(ui_disp_t));
  pdisp->status = UI_DISP_OFF;

  DISP_RECT_FULL(rect);
  disp_clrscn(rect);

}



disp_rect_t disp_clrscn(disp_rect_t    rect)
{

  lcd_setscn_TFT(rect.x,rect.y,rect.w,rect.h, RGB_Black);
  return rect;
}

disp_rect_t disp_draw_rect(disp_rect_t rect, uint16_t color)
{
  lcd_setscn_TFT(rect.x,rect.y,rect.w,rect.h, color);
  return rect;
}


disp_rect_t disp_character(disp_point_t point,  char ch, uint16_t color)
{
  disp_rect_t rect;
  char tmp = ch;

  DISP_RECT_EMPTY(rect);
  
  if(ch <' ')
    tmp = ' ';
  rect = disp_img(point, bmp_table[(uint16_t)tmp-0x20], color);

  
   
  return rect;
}


disp_rect_t disp_unicode(disp_point_t point, uint16_t unicode, uint16_t color)
{
  int ret;
	ui_disp_t* pdisp = &s_disp;
  disp_rect_t rect;
  DISP_RECT_EMPTY(rect);
  
  uint8_t bitmap[FONT_BITMAP_SIZE];
  ret = ui_font_unicode(pdisp->font,unicode,bitmap);
  if(ret != APP_SUCCESS)
    return rect;
  

  return disp_img_x(point, bitmap, color);

}

disp_rect_t disp_string_u(disp_point_t point, const uint16_t *unicode, uint16_t color)
{
  int ret;
	ui_disp_t* pdisp = &s_disp;
  uint8_t bitmap[FONT_BITMAP_SIZE];
  uint16_t offset = 0;
  
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
    if(unicode[offset] <128){
      rect1 = disp_character(point_new, (char)unicode[offset],color);
      point_new.x += rect1.w + 1;
      continue;
    }
    
    ret = ui_font_unicode(pdisp->font,unicode[offset],bitmap);
    if(ret != APP_SUCCESS)
      continue;
    rect1 = disp_img_x(point_new, bitmap, color);
    point_new.x += rect1.w + 1;
        
  }
  rect.x = point.x;
  rect.y = point.y;
  rect.w = point_new.x + 1 - point.x;
  rect.h = rect1.h + rect1.y - rect.y;
  return rect;

}


disp_rect_t disp_string(disp_point_t point, const char *utf8, uint16_t color)
{
  int ret;
	ui_disp_t* pdisp = &s_disp;
  const char* utf8_str = utf8;
  uint16_t unic_ch;
  uint8_t bitmap[FONT_BITMAP_SIZE];
  uint16_t offset;
  
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
    if(unic_ch <128){
      rect1 = disp_character(point_new, (char)unic_ch, color);
      point_new.x += rect1.w + 1;
      continue;
    }
    
    ret = ui_font_unicode(pdisp->font,unic_ch,bitmap);
    if(ret != APP_SUCCESS)
      continue;
    rect1 = disp_img_x(point_new, bitmap, color);
    point_new.x += rect1.w + 1;
        
  }
  rect.x = point.x;
  rect.y = point.y;
  rect.w = point_new.x + 1 - point.x;
  rect.h = rect1.h + rect1.y - rect.y;
  return rect;

}


//display digital big
//flg: 0 is for big font, 1 is for small font
disp_rect_t disp_digi(disp_point_t point, uint16_t type, char* str, uint16_t color)
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
    
    rect1 = disp_idb(point_new, id_offset + (ch - '0'), color); 
    point_new.x += rect1.w+1;
  }
  rect.x = point.x;
  rect.y = point.y;
  rect.w = point_new.x + 1 - point.x;
  rect.h = rect1.h + rect1.y - rect.y;
  return rect;
}



disp_rect_t disp_idb(disp_point_t point, uint16_t idb, uint16_t color){
  disp_rect_t rect;

  DISP_RECT_EMPTY(rect);
  
  if(idb >NUM_BITMAP)
    return rect;
  return disp_img(point, bmp_table[idb], color);
}


disp_rect_t disp_character_f(disp_point_t point,  char ch, uint16_t color)
{
  disp_rect_t rect;
  char tmp = ch;

  DISP_RECT_EMPTY(rect);
  
  if(ch <' ')
    tmp = ' ';
  rect = disp_img_f(point, bmp_table[(uint16_t)tmp-0x20], color);

  
   
  return rect;

}

disp_rect_t disp_unicode_f(disp_point_t point, uint16_t unicode, uint16_t color)
{
  int ret;
	ui_disp_t* pdisp = &s_disp;
  disp_rect_t rect;
  DISP_RECT_EMPTY(rect);
  
  uint8_t bitmap[FONT_BITMAP_SIZE];
  ret = ui_font_unicode(pdisp->font,unicode,bitmap);
  if(ret != APP_SUCCESS)
    return rect;


  return disp_img_fx(point, bitmap, color);


}
disp_rect_t disp_string_u_f(disp_point_t point, const uint16_t *unicode, uint16_t color)
{
  int ret;
	ui_disp_t* pdisp = &s_disp;
  uint8_t bitmap[FONT_BITMAP_SIZE];
  uint16_t offset = 0;
  
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
    if(unicode[offset] <128){
      rect1 = disp_character_f(point_new, (char)unicode[offset], color);
      point_new.x += rect1.w + 1;
      continue;
    }
    
    ret = ui_font_unicode(pdisp->font,unicode[offset],bitmap);
    if(ret != APP_SUCCESS)
      continue;
    rect1 = disp_img_fx(point_new, bitmap,color);
    point_new.x += rect1.w + 1;
        
  }
  rect.x = point.x;
  rect.y = point.y;
  rect.w = point_new.x + 1 - point.x;
  rect.h = rect1.h + rect1.y - rect.y;
  return rect;

}

disp_rect_t disp_string_f(disp_point_t point, const char* utf8, uint16_t color)
{
  int ret;
	ui_disp_t* pdisp = &s_disp;
  const char* utf8_str = utf8;
  uint16_t unic_ch;
  uint8_t bitmap[FONT_BITMAP_SIZE];
  uint16_t offset;
  
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
    if(unic_ch <128){
      rect1 = disp_character_f(point_new, (char)unic_ch,color);
      point_new.x += rect1.w + 1;
      continue;
    }
    
    ret = ui_font_unicode(pdisp->font,unic_ch,bitmap);
    if(ret != APP_SUCCESS)
      continue;
    rect1 = disp_img_fx(point_new, bitmap,color);
    point_new.x += rect1.w + 1;
        
  }
  
  rect.x = point.x;
  rect.y = point.y;
  rect.w = point_new.x + 1 - point.x;
  rect.h = rect1.h + rect1.y - rect.y;
  
  return rect;

}

disp_rect_t disp_digi_f(disp_point_t point, uint16_t type, char* str, uint16_t color)
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
    
    rect1 = disp_idb_f(point_new, id_offset + (ch - '0'), color); 
    point_new.x += rect1.w+1;
  }
  rect.x = point.x;
  rect.y = point.y;
  rect.w = point_new.x + 1 - point.x;
  rect.h = rect1.h + rect1.y - rect.y;
  return rect;

}


disp_rect_t disp_idb_f(disp_point_t point, uint16_t idb, uint16_t color)
{
  disp_rect_t rect;

  DISP_RECT_EMPTY(rect);
  
  if(idb >NUM_BITMAP)
    return rect;
  return disp_img_f(point, bmp_table[idb], color);

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
#endif /*CFG_DISP=DISP_TFT*/

