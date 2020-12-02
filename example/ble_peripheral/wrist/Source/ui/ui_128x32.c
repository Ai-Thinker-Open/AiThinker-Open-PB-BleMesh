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
 * Module Name:	display substance
 * File name:	ui_disp_substance.c 
 * Brief description:
 *    UI display substance.
 * Author:	Eagle.Lao
 * Revision:V0.01
****************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ui_display.h"
#include "ui_disp_substance.h"
//#include "res_tab.c"
#include "res_tab.h"
#include "app_datetime.h"
#include "battery.h"


static disp_rect_t disp_battery_bar(disp_point_t point)
{
	uint8_t percent = batt_percent();
	disp_rect_t  rect = disp_idb(point, IDB_ICON_batt);
	disp_rect_t  rect_draw = rect;
	rect_draw.x = point.x+1;
	rect_draw.y = point.y+1;
	rect_draw.h = 5;
	rect_draw.w = 8*percent/100;
	
	disp_draw_rect(rect_draw);
}


void ui_subst_blank_on_style_simple(datetime_t* pdtm)
{
	char str[10];
	disp_point_t point = {0,0};
	disp_rect_t rect;
	time_t tmt;
	struct tm * ptm = NULL;
	struct tm tms;
	DTM2TM(&tms, pdtm)	
	tmt = mktime(&tms);
	ptm = localtime(&tmt);
	DISP_RECT_FULL(rect);
	
	disp_clrscn(rect);


	DISP_POINT(point, 8, 4);
	sprintf(str, "%.2d", pdtm->hour);
	rect = disp_digi(point, 2, str);
	point.x += rect.w + 2;
	rect = disp_idb(point, IDB_N3_COLON);
	point.x += rect.w + 2;
	sprintf(str, "%.2d", pdtm->minutes);
	rect = disp_digi(point, 2, str);

	point.x = 102;
	point.y = 4;
	disp_battery_bar(point);

	
	point.x = 88;
	point.y = 14;
	rect = disp_idb(point, IDB_W_HEAD);
	point.x += rect.w + 2;
	rect = disp_idb(point, IDB_W_1 + ptm->tm_wday);
	


	disp_reflash();

	disp_on();
}

void ui_subst_home_reflash_style_simple(uint8_t hour, uint8_t minute, uint8_t fls_cnt)
{
	//do nothing
}



void ui_subst_pedometer_icon(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;
	char str[10];

	uint16_t steps = mp_fit_get_steps();
	sprintf(str, "%d", steps);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 24, (32 - frame_id*6));
	
	disp_idb(point, IDB_ICON_steps);

	//display icon
	DISP_POINT(point, 52, (34 - frame_id*6));
	disp_digi(point, DIGI_0, str);
	disp_reflash();

}

void ui_subst_pedometer_steps(uint16_t steps)
{
	//do nothing
}


void ui_subst_calorie_icon(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;

	char str[10];
	uint16_t calorie  =0;
	calorie = motion_get_calorie();

	sprintf(str, "%d", calorie);

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);


	//display icon
	DISP_POINT(point, 24, (32 - frame_id*6));
	
	disp_idb(point, IDB_ICON_kalorie);

	//display icon
	DISP_POINT(point, 52, (34 - frame_id*6));
	disp_digi(point, DIGI_0, str);

	disp_reflash();

}




void ui_subst_calorie_value(uint16_t calorie)
{
	//do nothing

}


void ui_subst_hr_icon(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;
	
	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 10, (32 - frame_id*8));
	
	disp_idb(point, IDB_ICON_heartrate);

	DISP_POINT(point, 50, (40 - frame_id*8));
	disp_idb(point, IDB_SUFFIX_equal);
	DISP_POINT(point, 62, (40 - frame_id*8));
	disp_idb(point, IDB_SUFFIX_equal);

	DISP_POINT(point, 78, (44 - frame_id*8));
	disp_idb(point, IDB_SUFFIX_bps);
	disp_reflash();

}

void ui_subst_hr_blink(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	if(frame_id%2){
		DISP_POINT(point, 20, 4);
		disp_idb(point, IDB_ICON_heartrate0);
	}
	else
	{
		DISP_POINT(point, 20, 6);
		disp_idb(point, IDB_ICON_heartrate1);
	}
	if(frame_id %2){
		DISP_POINT(point, 50, 12);
		disp_idb(point, IDB_SUFFIX_equal);
		DISP_POINT(point, 62, 12);
		disp_idb(point, IDB_SUFFIX_equal);
	}
	DISP_POINT(point, 78, 16);
	disp_idb(point, IDB_SUFFIX_bps);
	disp_reflash();

}

void ui_subst_hr_failure(void)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);
	disp_clrscn(rect);


	//display icon
	DISP_POINT(point, 20, 4);
	disp_idb(point, IDB_ICON_heartrate0);
	DISP_POINT(point, 58, 12);
	disp_idb(point, IDB_ICON_heartrate_x);
	disp_idb(point, IDB_SUFFIX_bps);
	DISP_POINT(point, 78, 16);
	disp_idb(point, IDB_SUFFIX_bps);
	disp_reflash();
}


void ui_subst_hr_heartrate(uint8_t value)
{
	char str[10];
	disp_rect_t rect;
	disp_point_t point;
	LOG(">>>ui_subst_hr_heartrate, get value %d\n", value);
	DISP_RECT_FULL(rect);
	disp_clrscn(rect);

	//display icon
	sprintf(str, "%d", value);
	DISP_POINT(point, 50, 6);
	rect = disp_digi(point, 2, str);
	point.x += rect.w+2;
	point.y += 4;
	DISP_POINT(point, 20, 4);
	disp_idb(point, IDB_ICON_heartrate0);

	DISP_POINT(point, 78, 16);
	disp_idb(point, IDB_SUFFIX_bps);
	disp_reflash_all();

}

void ui_subst_humiture_icon(uint8_t frame_id)
{
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 10, (0 - frame_id*8));
	
	disp_idb(point, IDB_ICON_blood_pressure);
	disp_reflash();

}

void ui_subst_cup_icon(uint8_t frame_id){
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 60, (32 - frame_id*8));
	
	disp_idb(point, IDB_ICON_press0);
	disp_reflash();

}


void ui_subst_batt_icon(uint8_t frame_id){
	disp_rect_t rect;
	disp_point_t point;

	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	//display icon
	DISP_POINT(point, 60, (32 - frame_id*8));
	
	disp_idb(point, IDB_ICON_fw_upgrade);
	disp_reflash();

}

void ui_subst_batt_reflash(uint8_t frame_id, uint8_t percent){
	disp_rect_t rect;
	disp_point_t point;
	char str[20];

	sprintf(str, "charging:%d%%", percent);
	DISP_RECT_FULL(rect);

	disp_clrscn(rect);

	if(frame_id %2){
		LOG("ui_subst_batt_reflash\n");
		//display icon
		DISP_POINT(point, 10, 8);
		disp_string(point, str);
	}
	disp_reflash();

}


