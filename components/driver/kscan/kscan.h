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

/*******************************************************************************
* @file		kscan.h
* @brief	Contains all functions support for key scan driver
* @version	0.0
* @date		13. Nov. 2017
* @author	Ding
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __KSCAN__H__
#define __KSCAN__H__

#include "types.h"
#include "gpio.h"

#define		MULTI_KEY_READ_ADDR		0x400240CCUL
#define		KSCAN_IRQ_ENABLE      	do{\
										*(volatile unsigned int *) 0x400240C0 |= BIT(1);\
										*(volatile unsigned int *) 0xe000e100 |= BIT(5);\
									}while(0)
#define		KSCAN_IRQ_DISABLE      	do{\
										*(volatile unsigned int *) 0x400240C0 &= ~BIT(1);\
										*(volatile unsigned int *) 0xe000e100 &= ~BIT(5);\
									}while(0)
#define		ENABLE_KSCAN			*(volatile unsigned int *) 0x400240C0 |= BIT(0)
#define		DISABLE_KSCAN			*(volatile unsigned int *) 0x400240C0 &= ~BIT(0)
#define		CONFIG_KEY_ROW(v)		*(volatile unsigned int *) 0x400240F0 |= BIT(v)
#define		CONFIG_KEY_COL(v)		*(volatile unsigned int *) 0x400240C0 |= (BIT(v) << 2)
#define		ENABLE_AUTO_SCAN		*(volatile unsigned int *) 0x400240C0 |= BIT(21)
#define		DISABLE_AUTO_SCAN		*(volatile unsigned int *) 0x400240C0 &= ~BIT(21)
#define		CLEAR_INTERUPT			*(volatile unsigned int *) 0x400240C4 |= BIT(0)
#define		CLEAR_KEY_PRESSED		*(volatile unsigned int *) 0x400240C4 |= BIT(17)
#define		EN_MUX_KSCAN_ROW(v)		*(volatile unsigned int *) 0x4000384C |= BIT(v)
#define		EN_MUX_KSCAN_COL(v)		*(volatile unsigned int *) 0x40003850 |= BIT(v)
#define		DIS_MUX_KSCAN_ROW(v)	*(volatile unsigned int *) 0x4000384C &= ~BIT(v)
#define		DIS_MUX_KSCAN_COL(v)	*(volatile unsigned int *) 0x40003850 &= ~BIT(v)
#define		SET_MULTI_KEY_STATE(v)	subWriteReg(0x400240C0,20,20,v)
#define		SET_POLARITY(v)			subWriteReg(0x400240C0,23,23,v)
#define		SET_INTERVAL(v)			subWriteReg(0x400240C0,31,24,v)
#define  	IS_KEY_PRESSED			(read_reg(0x400240C4) & 0x00020000) >> 17
#define  	KEY_PRESS_STATE			(read_reg(0x400240C8) & 0x00000C00) >> 10
#define		ROW_OF_ONE_KEY			(read_reg(0x400240C8) & 0x000003E0) >> 5
#define		COL_OF_ONE_KEY			(read_reg(0x400240C8) & 0x0000001F)
#define 	POWER_OFF_32K_XOSC		do{\
										*(volatile unsigned int *) 0x4000F01C |= BIT(6);\
										*(volatile unsigned int *) 0x4000F01C &= ~BIT(9);\
									}while(0)
#define		HW_CRTL_MODE			do{\
										*(volatile unsigned int *) 0x4000F01C &= ~BIT(6);\
									}while(0)


#define    IS_ENABLE_KSCAN            (read_reg(0x400240C0) & BIT(0))
#define    IS_ENABEL_KSCANIRQ         ((read_reg(0x400240C0) & BIT(1))>>1)
#define    IS_ENABEL_AUTOKSCAN        ((read_reg(0x400240C0) & BIT(21))>>21)



const static uint8_t KSCAN_ROW_GPIO[16] = {0,2,5,7,10,12,15,18,19,22,23,25,27,29,32,34};
const static uint8_t KSCAN_ROW_MK[16] = {0,1,10,11,4,12,2,5,13,14,6,3,9,15,7,8};
const static uint8_t KSCAN_COL_GPIO[18] = {1,3,4,6,9,11,13,14,16,17,20,21,24,26,28,30,31,33};
const static uint8_t KSCAN_COL_MK[18] = {0,1,9,10,4,11,12,2,16,17,5,13,3,14,8,15,7,6};

#define NUM_KEY_ROWS   4
#define NUM_KEY_COLS   4
#define MAX_KEY_NUM   10	
/*************************************************************
*	@brief		enum variable used for setting rows
*
*/
typedef enum{

	KEY_ROW_P00   =   0,
	KEY_ROW_P02   =   1, 
	KEY_ROW_P05   =   2, 
	KEY_ROW_P07   =   3, 
	KEY_ROW_P10   =   4,
	KEY_ROW_P12   =   5,
	KEY_ROW_P15   =   6, 
	KEY_ROW_P18   =   7, 
	KEY_ROW_P19   =   8, 
	KEY_ROW_P22   =   9,  
	KEY_ROW_P23   =   10, 
	KEY_ROW_P25   =   11, 
	KEY_ROW_P27   =   12, 
	KEY_ROW_P29   =   13,  
	KEY_ROW_P32   =   14, 
	KEY_ROW_P34   =   15
	
}KSCAN_ROWS_e;

/*************************************************************
*	@brief		enum variable used for setting cols
*
*/
typedef enum{

	KEY_COL_P01   =   0,
	KEY_COL_P03   =   1,
	KEY_COL_P04   =   2, 
	KEY_COL_P06   =   3,  
	KEY_COL_P09   =   4,
	KEY_COL_P11   =   5,
	KEY_COL_P13   =   6,
	KEY_COL_P14   =   7,
	KEY_COL_P16   =   8,
	KEY_COL_P17   =   9,
	KEY_COL_P20   =   10,
	KEY_COL_P21   =   11,
	KEY_COL_P24   =   12,
	KEY_COL_P26   =   13,
	KEY_COL_P28   =   14, 
	KEY_COL_P30   =   15,
	KEY_COL_P31   =   16, 
	KEY_COL_P33   =   17
	
}KSCAN_COLS_e;

/*************************************************************
*	@brief		enum variable used for setting multiple key press
*
*/
typedef enum{
	
	NOT_IGNORE_MULTI_KEY = 0,
	IGNORE_MULTI_KEY	= 1
	
}KSCAN_MULTI_KEY_STATE_e;

/*************************************************************
*	@brief		enum variable used for setting whether ignore ghost key
*
*/
typedef enum{
	
	NOT_IGNORE_GHOST_KEY = 0,
	IGNORE_GHOST_KEY	= 1
	
}KSCAN_GHOST_KEY_STATE_e;

/*************************************************************
*	@brief		enum variable used for setting key press sense type
*
*/
typedef enum{
	
	SENCE_HIGH = 0,
	SENCE_LOW = 1
	
}KSCAN_POLARITY_e;

/*************************************************************
*	@brief		enum variable used for setting key press sense type
*
*/
typedef enum{
	
	NO_KEY_PRESS = 0x00,
	ONE_KEY_PRESS = 0x01,
	MULTI_KEY_PRESS = 0x02
	
}KSCAN_KEY_PRESS_STATE_e;

typedef enum{
	KEY_RELEASED = 0,
	KEY_PRESSED,
} kscan_Evt_Type_t;

typedef struct {
	uint8_t				row;
	uint8_t				col;
	kscan_Evt_Type_t	type;
} kscan_Key_t;

typedef struct kscan_Evt_t_{
	uint8_t		 	num;
	kscan_Key_t*	keys; 
} kscan_Evt_t;

typedef void (*kscan_Hdl_t)(kscan_Evt_t* pev);

typedef struct {
	KSCAN_GHOST_KEY_STATE_e  ghost_key_state;
	KSCAN_ROWS_e*			 key_rows;
	KSCAN_COLS_e* 			 key_cols;
	kscan_Hdl_t				 evt_handler;
	uint8_t				 	 interval;
} kscan_Cfg_t;



//PUBLIC FUNCTIONS
int  hal_kscan_init(kscan_Cfg_t cfg, uint8 task_id, uint16 event);
void hal_kscan_timeout_handler(void);
void __attribute__((weak)) hal_KSCAN_IRQHandler(void);

#endif
