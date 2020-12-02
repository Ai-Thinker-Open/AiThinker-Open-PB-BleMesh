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
 * Module Name:	display substance
 * File name:	ui_disp_substance.c 
 * Brief description:
 *    UI display substance.
 * Author:	Eagle.Lao
 * Revision:V0.02
****************************************************************/

#if(CFG_DISP==DISP_TFT)

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ui_dispTFT.h"
#include "ui_disp_substance.h"
#include "res_tab.h"
#include "app_datetime.h"
#include "battery.h"
#include "ui_page.h"
#include "kx023.h"
#include "log.h"

//static disp_rect_t disp_battery_bar(disp_point_t point)
//{
//	
//	uint8_t percent = 0;//batt_percent();
//	disp_rect_t  rect = disp_idb_f(point, IDB_ICON_batt);
//	disp_rect_t  rect_ret = rect;
//	rect.x = point.x+1;
//	rect.y = point.y+1;
//	rect.h = 5;
//	rect.w = 8*percent/100;

//	RECT_V2H(rect);
//	
//	disp_draw_rect(rect);
//	
//	return rect_ret;
//}


void ui_normal_blank_on_style_simple(datetime_t* pdtm, uint8_t* p_addr)
{
	char str[10];
	disp_point_t point = {0,0};
	disp_rect_t rect;
	time_t tmt;
	struct tm * ptm = NULL;
  uint8_t wday = 0;
	struct tm tms;
	DTM2TM(&tms, pdtm)	
	tmt = mktime(&tms);
	ptm = localtime(&tmt);
  wday = ptm->tm_wday;
	DISP_RECT_FULL(rect);
  
  lcd_bus_init();
  //lcd_init();
	disp_clrscn(rect);

	//if(1)//ble_conn_state())
	//{
	//	DISP_POINT(point, 28, 6);
	//	disp_idb_f(point, IDB_ASC_5EH);
	//}


	DISP_POINT(point, 32, 36);
	sprintf(str, "%.2d", pdtm->hour);
	rect = disp_digi_f(point, DIGI_TM, str, RGB_Gold);
	DISP_POINT(point, 32, 64);
	sprintf(str, "%.2d", pdtm->minutes);
	rect = disp_digi_f(point, DIGI_TM, str, RGB_Gold);

	//point.x = 19;
	//point.y = 26;
	//disp_battery_bar(point);

	sprintf(str, "%.2f", batt_voltage());
	DISP_POINT(point, 16, 4);
	disp_string_f(point, str, RGB_Green);
	
	point.x = 33;
	point.y = 90;
	rect = disp_idb_f(point, IDB_W_HEAD, RGB_DoderBlue);
	point.x += rect.w + 2;

  if(wday == 0)
		rect = disp_idb_f(point, IDB_W_7, RGB_DoderBlue);
	else{
		rect = disp_idb_f(point, IDB_W_1 + wday-1, RGB_DoderBlue);
		LOG("%d, %d,% d\n",wday, point.x, point.y);
	}
	if(p_addr){
		char ad_str[10];
		sprintf(ad_str, "%.2x%.2x",p_addr[0], p_addr[1]);
		point.x = 0;
		point.y = 102;
		disp_string_f(point, ad_str, RGB_Gold);
		sprintf(ad_str, "%.2x%.2x",p_addr[2], p_addr[3]);
		point.x = 0;
		point.y = 115;
		disp_string_f(point, ad_str, RGB_Gold);
	}
	
	disp_on();
	
	LOG("ui_normal_blank_on_style_simple exit\n");
}

void ui_normal_pedometer_key_press_long(void)
{
	disp_rect_t rect;
	disp_point_t point;

  lcd_bus_init();
	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46);
	disp_idb_f(point, IDB_ICON_press0, RGB_Gold);

}
void ui_normal_acc_init(void)
{
  uint8_t pos = 10;
	char str[15];
  disp_rect_t rect;
	disp_point_t point;
  lcd_bus_init();
  
  DISP_RECT_FULL(rect);
  disp_clrscn(rect);
  
	sprintf(str, "x:0");
	DISP_POINT(point, 4+pos, 4);
  disp_string_f(point, str, RGB_Gold);
  
	sprintf(str, "y:0");
	DISP_POINT(point, 4+pos, 40);
  disp_string_f(point, str, RGB_Gold);
  
	sprintf(str, "z:0");
	DISP_POINT(point, 4+pos, 76);
  disp_string_f(point, str, RGB_Gold);

  DISP_RECT(rect, 4+pos, 20, 56, 16);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);
  DISP_RECT(rect, 5+pos, 21, 54, 14);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32+pos, 23, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);

  DISP_RECT(rect, 4+pos, 56, 56, 16);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);
  DISP_RECT(rect, 5+pos, 57, 54, 14);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32+pos, 58, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);

  DISP_RECT(rect, 4+pos, 92, 56, 16);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);
  DISP_RECT(rect, 5+pos, 93, 54, 14);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32+pos, 95, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);

  
}

void ui_normal_acc(int x, int y, int z)
{
  uint8_t pos = 10;
	char str[32];
  disp_rect_t rect;
	disp_point_t point;
	float fx,fy,fz;
  //LOG("X%d,Y%d,Z%d\n",x,y,z);
  fx = x/200.0;
  fy = z/200.0;
  fz = y/200.0;

  lcd_bus_init();
  
	sprintf(str, "x: %.2f       ",fx/20);
	DISP_POINT(point, 4+pos, 4);
  disp_string_f(point, str, RGB_Gold);
  
	sprintf(str, "y: %.2f       ",fy/20);
	DISP_POINT(point, 4+pos, 40);
  disp_string_f(point, str, RGB_Gold);
  
	sprintf(str, "z: %.2f       ",fz/20);
	DISP_POINT(point, 4+pos, 76);
  disp_string_f(point, str, RGB_Gold);

  DISP_RECT(rect, 6+pos, 23, 52, 10);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32+pos + (int)fx, 23, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);

  DISP_RECT(rect, 6+pos, 58, 52, 10);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32+pos + (int)fy, 58, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);

  DISP_RECT(rect, 6+pos, 95, 52, 10);
  RECT_V2H(rect);
  disp_clrscn(rect);
  DISP_RECT(rect, 32+pos + (int)fz, 95, 2, 10);
  RECT_V2H(rect);
  disp_draw_rect(rect, RGB_Gold);

}

void ui_normal_pedometer_steps(uint16_t steps, uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;
	char str[10];
	uint8_t length;
	uint16_t type = 0;
	uint16_t offset = 0;
  lcd_bus_init();

	sprintf(str, "%d", steps);
	length = strlen(str);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 7, 30);
	if(frame_id%2)
		disp_idb_f(point, IDB_ICON_steps, RGB_Gold);
	else
		disp_idb_f(point, IDB_ICON_steps1, RGB_Gold);

	DISP_POINT(point, 0, 64);
	//display icon
	switch(length){
	case 1:
		type = DIGI_1;
		offset = 12;
		break;
	case 2:
		type = DIGI_1;
		offset = 6;
		break;
	case 3:
		type = DIGI_1;
		offset = 1;
		break;
	case 4:
		type = DIGI_2;
		offset = 1;
		break;
	case 5:
		type = DIGI_3;
		offset = 0;
		break;
	default:
		break;

	}

	point.x = offset;	
	disp_digi_f(point, type, str, RGB_Gold);

}


void ui_normal_calorie_value(uint16_t calorie)
{
	disp_rect_t rect;
	disp_point_t point;

	char str[10];
	uint8_t length;
	uint16_t type = 0;
	uint16_t offset = 0;
  lcd_bus_init();
	
	calorie = 0;//motion_get_calorie();

	sprintf(str, "%d", calorie);
	length = strlen(str);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);


	//display icon
	DISP_POINT(point, 7, 30);
	
	disp_idb_f(point, IDB_ICON_kalorie, RGB_Gold);

	DISP_POINT(point, 0, 64);
	//display icon
	switch(length){
	case 1:
		type = DIGI_1;
		offset = 12;
		break;
	case 2:
		type = DIGI_1;
		offset = 6;
		break;
	case 3:
		type = DIGI_1;
		offset = 1;
		break;
	case 4:
		type = DIGI_2;
		offset = 1;
		break;
	case 5:
		type = DIGI_3;
		offset = 0;
		break;
	default:
		break;

	}

	point.x = offset;	
	disp_digi_f(point, type, str, RGB_Gold);

}




void ui_normal_hr_blink(uint8_t frame_id)
{
  uint8_t pos = 20;
	disp_rect_t rect;
	disp_point_t point;
  lcd_bus_init();

  if(frame_id == 0){
  	DISP_RECT_FULL(rect);
  	disp_clrscn(rect);
  }
  else
  {
    DISP_RECT(rect, 5+pos, 30, 22, 22);
    RECT_V2H(rect);
  	disp_clrscn(rect);
    
  }

	//display icon
	if(frame_id%2){
		DISP_POINT(point, 5+pos, 30);
		disp_idb_f(point, IDB_ICON_heartrate, RGB_Red);
	}
	else
	{
		DISP_POINT(point, 6+pos, 32);
		disp_idb_f(point, IDB_ICON_heartrate1, RGB_Red);
	}

	DISP_POINT(point, 6+pos, 68);

	disp_idb_f(point, IDB_ICON_heartrate_w, RGB_Gold);


	DISP_POINT(point, 4+pos, 88);
	disp_idb_f(point, IDB_SUFFIX_bpm, RGB_Gold);


	
}

void ui_normal_hr_failure(void)
{
  uint8_t pos = 20;
	disp_rect_t rect;
	disp_point_t point;
  lcd_bus_init();

  DISP_RECT(rect, 5+pos, 67, 30, 16);
  RECT_V2H(rect);
  disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 5+pos, 30);
	disp_idb_f(point, IDB_ICON_heartrate, RGB_Red);

	DISP_POINT(point, 11+pos, 67);
	disp_idb_f(point, IDB_ICON_heartrate_x, RGB_Red);


	//DISP_POINT(point, 4+pos, 88);
	//disp_idb_f(point, IDB_SUFFIX_bpm, RGB_Gold);

}


void ui_normal_hr_heartrate(uint8_t value)
{
  uint8_t pos = 20;
	disp_rect_t rect;
	disp_point_t point;
	char str[10];
	uint8_t length;
	//uint16_t type = 0;
	uint16_t offset = 7;
  lcd_bus_init();

	sprintf(str, "%d", value);
	length = strlen(str);

  DISP_RECT(rect, 5+pos, 67, 30, 16);
  RECT_V2H(rect);
  disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 5+pos, 30);
	disp_idb_f(point, IDB_ICON_heartrate, RGB_Red);
	
	if(length > 2)
		offset = 1;
	DISP_POINT(point, offset+pos, 64);
	disp_digi_f(point, DIGI_1, str, RGB_Gold);

	//DISP_POINT(point, 4+pos, 88);
	//disp_idb_f(point, IDB_SUFFIX_bpm, RGB_Gold);


}

void ui_normal_humiture_icon(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;
  lcd_bus_init();

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 10, (0 - frame_id*8));
	
	disp_idb(point, IDB_ICON_blood_pressure, RGB_Gold);

}

void ui_normal_cup_icon(uint8_t frame_id){
	disp_rect_t rect;
	disp_point_t point;

  lcd_bus_init();
	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	if(frame_id%2){
		DISP_POINT(point, 6, 46);
		disp_idb_f(point, IDB_ICON_press0, RGB_Gold);
	}
	else
	{
		DISP_POINT(point, 6, 48);
		disp_idb_f(point, IDB_ICON_press1, RGB_Gold);
	}

}

void ui_normal_batt_reflash(uint8_t frame_id, uint8_t percent){
  uint8_t pos = 20;
	disp_rect_t rect;
	disp_point_t point;
	char str[10];
	uint8_t len;

  lcd_bus_init();
	sprintf(str, "%d%%",0);//batt_percent());
	len = strlen(str);
	
	if(frame_id == 0){
		DISP_RECT_FULL(rect);
		disp_clrscn(rect);
		DISP_POINT(point, 2+pos, 52);
		disp_idb_f(point, IDB_ICON_batt0, RGB_Gold);
	}
	else
	{
		DISP_RECT(rect, 4+pos, 54, 20, 13);
		RECT_V2H(rect);
		disp_clrscn(rect);
	}
	percent = (percent <100)? percent:100;
	DISP_RECT(rect, 6+pos,57,(16*percent/100), 8);

	RECT_V2H(rect);
	
	disp_draw_rect(rect, RGB_Gold);

	if(len == 2)
	{
		DISP_POINT(point, 8+pos, 78);
	}
	else if(len == 3)
	{
		DISP_POINT(point, 4+pos, 78);
	}
	else
	{
		DISP_POINT(point, 0+pos, 78);
	}
	disp_string_f(point, str, RGB_Green);
	
}

void ui_normal_call_icon(void){
  uint8_t pos = 20;
	disp_rect_t rect;
	disp_point_t point;
  lcd_bus_init();

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6+pos, 46);
	disp_idb_f(point, IDB_ICON_call, RGB_Green);

}

void ui_normal_call_info(uint16_t frame_id, const char* title){
  uint8_t pos = 20;
	disp_rect_t rect;
	disp_point_t point;
	uint16_t idx = frame_id;

	if(idx >= 14)
		idx = 14;
  lcd_bus_init();

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6+pos, 46 - idx*3);
	disp_idb_f(point, IDB_ICON_call, RGB_Gold);

	//display title
	DISP_POINT(point, 68+pos - idx*3, 8);
	disp_string_f(point, title, RGB_Gold);
	
}


void ui_normal_msg_title(uint16_t frame_id, uint16_t ev_id, char* title){
	disp_rect_t rect;
	disp_point_t point;
	uint16_t icon;
  uint16_t color;
  lcd_bus_init();

	switch(ev_id){
	case UI_EV_BLE_SMS:
		icon = IDB_ICON_sms;
    color = RGB_White;
		break;
	case UI_EV_BLE_WECHAT:
		icon = IDB_ICON_wechat;
    color = RGB_Green;
		break;
	case UI_EV_BLE_QQ:
		icon = IDB_ICON_qq;
    color = RGB_Chocolate;
		break;
	case UI_EV_BLE_MAIL:
		icon = IDB_ICON_sms;
    color = RGB_Yellow;
		break;
	case UI_EV_BLE_MSG_NOTIFY:
		icon = IDB_ICON_sms;
    color = RGB_Yellow;
		break;
	default:
		return;
	}

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	if(frame_id % 2){
		//display icon
		DISP_POINT(point, 16, 20);
		disp_idb_f(point, icon, color);

		//display title
		DISP_POINT(point, 4, 60);
		disp_string_f(point, (const char*)title, RGB_White);
	}
	disp_on();
}


void ui_normal_msg_info(uint16_t ev_id, uint16_t** u_str_tbl,bool end){
	disp_rect_t rect;
	disp_point_t point;
	uint16_t icon;
  uint16_t color;
	uint16_t* ustr = NULL;
  int i;
  lcd_bus_init();

	switch(ev_id){
	case UI_EV_BLE_SMS:
		icon = IDB_ICON_sms;
    color = RGB_White;
		break;
	case UI_EV_BLE_WECHAT:
		icon = IDB_ICON_wechat;
    color = RGB_Green;
		break;
	case UI_EV_BLE_QQ:
		icon = IDB_ICON_qq;
    color = RGB_Chocolate;
		break;
	case UI_EV_BLE_MAIL:
		icon = IDB_ICON_sms;
    color = RGB_Yellow;
		break;
	case UI_EV_BLE_MSG_NOTIFY:
		icon = IDB_ICON_sms;
    color = RGB_Yellow;
		break;
	default:
		return;
	}
	
	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 16, 20);
	disp_idb_f(point, icon, color);

  for(i = 0; i<4; i++){
    ustr = u_str_tbl[i];
    if(ustr == NULL)
      break;
		//display title
		DISP_POINT(point, 2, 42 + i*18);
		disp_string_u_f(point, (const uint16_t*)ustr, RGB_White);

	}

	if(end == FALSE){
		//display title
		DISP_POINT(point, 50, 112);
		disp_idb_f(point, IDB_ASC_1EH, RGB_Yellow);
	}
	disp_on();
	
}



void ui_normal_alarm_clk_icon(void){
	disp_rect_t rect;
	disp_point_t point;

  lcd_bus_init();
	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46);
	disp_idb_f(point, IDB_ICON_alarmclock, RGB_Gold);
	disp_on();
}

void ui_firmware_upgrade(void){
	disp_rect_t rect;
	disp_point_t point;

  lcd_bus_init();
	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 6, 46);
	disp_idb_f(point, IDB_ICON_fw_upgrade, RGB_Gold);
	disp_on();
}

#endif /*CFG_DISP=DISP_TFT*/

