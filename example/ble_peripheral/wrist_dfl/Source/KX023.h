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

#ifndef _KX023_H
#define _KX023_H


#include <stdbool.h>
#include <stdint.h>
#define ACC_COTC 0x7F;//set 0x3f,we can get 0x55, set 0x7f we can get 0xAA


// KXTI9 register addresses
#define ACC_REG_ADDR_XOUT_HPF_L        0x00 // R
#define ACC_REG_ADDR_XOUT_HPF_H        0x01 // R
#define ACC_REG_ADDR_YOUT_HPF_L        0x02 // R
#define ACC_REG_ADDR_YOUT_HPF_H        0x03 // R
#define ACC_REG_ADDR_ZOUT_HPF_L        0x04 // R
#define ACC_REG_ADDR_ZOUT_HPF_H        0x05 // R
#define ACC_REG_ADDR_XOUT_L            0x06 // R
#define ACC_REG_ADDR_XOUT_H            0x07 // R
#define ACC_REG_ADDR_YOUT_L            0x08 // R
#define ACC_REG_ADDR_YOUT_H            0x09 // R
#define ACC_REG_ADDR_ZOUT_L            0x0A // R
#define ACC_REG_ADDR_ZOUT_H            0x0B // R
#define ACC_REG_ADDR_DCST_RESP         0x0C // R
#define ACC_REG_ADDR_WHO_AM_I          0x0F // R
#define ACC_REG_ADDR_TILT_POS_CUR      0x10 // R
#define ACC_REG_ADDR_TILT_POS_PRE      0x11 // R

#define ACC_REG_ADDR_INT_SRC_REG1      0x12 // R
#define ACC_REG_ADDR_INT_SRC_REG2      0x13 // R
#define ACC_REG_ADDR_INT_SRC_REG3      0x14 // R
#define ACC_REG_ADDR_STATUS_REG        0x15 // R
#define ACC_REG_ADDR_INT_REL           0x17 // R

#define ACC_REG_ADDR_CTRL_REG1         0x18 // R/W
#define ACC_REG_ADDR_CTRL_REG2         0x19 // R/W
#define ACC_REG_ADDR_CTRL_REG3         0x1A // R/W

#define ACC_REG_ADDR_ODR_CNTL          0x1B // R/W

#define ACC_REG_ADDR_INT_CTRL_REG1     0x1C // R/W
#define ACC_REG_ADDR_INT_CTRL_REG2     0x1D // R/W
#define ACC_REG_ADDR_INT_CTRL_REG3     0x1E // R/W
#define ACC_REG_ADDR_INT_CTRL_REG4     0x1F // R/W
#define ACC_REG_ADDR_INT_CTRL_REG5     0x20 // R/W
#define ACC_REG_ADDR_INT_CTRL_REG6     0x21 // R/W

#define ACC_REG_ADDR_TILT_TIMER        0x22 // R/W
#define ACC_REG_ADDR_WUF_TIMER         0x23 // R/W
#define ACC_REG_ADDR_TDT_EN            0x24 // R/W
#define ACC_REG_ADDR_TDT_TIMER         0x25 // R/W
#define ACC_REG_ADDR_TDT_H_THRESH      0x26 // R/W
#define ACC_REG_ADDR_TDT_L_THRESH      0x27 // R/W
#define ACC_REG_ADDR_TDT_TAP_TIMER     0x28 // R/W
#define ACC_REG_ADDR_TDT_DTAP_TIMER    0x29 // R/W
#define ACC_REG_ADDR_TDT_TAP_TIMER_2   0x2A // R/W
#define ACC_REG_ADDR_TDT_TAP_DTAP_TIMER     0x2B // R/W
#define ACC_REG_ADDR_WUF_THRESH        0x30 // R/W
#define ACC_REG_ADDR_TILT_ANGLE_LL     0x32 // R/W
#define ACC_REG_ADDR_TILT_ANGLE_HL     0x33 // R/W
#define ACC_REG_ADDR_HYST_SET          0x34 // R/W
#define ACC_REG_ADDR_LP_CNTL           0x35 // R/W

#define ACC_REG_ADDR_BUF_CTRL1         0x3A // R/W
#define ACC_REG_ADDR_BUF_CTRL2         0x3B // R/W
#define ACC_REG_ADDR_BUF_STATUS_REG1   0x3C // R
#define ACC_REG_ADDR_BUF_STATUS_REG2   0x3D // R/W
#define ACC_REG_ADDR_BUF_CLEAR         0x3E // W
#define ACC_REG_ADDR_BUF_READ          0x3F // R/W

#define ACC_REG_ADDR_SELF_TEST         0x60 // R/W

// Select register valies
#define REG_VAL_WHO_AM_I               0x15 // (data sheet says 0x14)

// CTRL1 BIT MASKS
#define ACC_REG_CTRL_PC                0x80 // Power control  '1' On    '0' Off
#define ACC_REG_CTRL_RES               0x40 // Resolution     '1' High  '0' Low
#define ACC_REG_CTRL_DRDYE             0x20 // Data Ready     '1' On    '0' Off
#define ACC_REG_CTRL_GSEL_HI           0x10 // Range     '00' +/-2g    '01' +/-4g
#define ACC_REG_CTRL_GSEL_LO           0x08 //           '10' +/-8g    '11' N/A
#define ACC_REG_CTRL_GSEL_TDTE         0x04 // Directional Tap '1' On   '0' Off
#define ACC_REG_CTRL_GSEL_WUFE         0x02 // Wake Up         '1' On   '0' Off
#define ACC_REG_CTRL_GSEL_TPE          0x01 // Tilt Position   '1' On   '0' Off

//ctrl
#define BFI1	0x40// – Buffer full interrupt reported on physical interrupt pin INT1
#define WMI1	0x20// - Watermark interrupt reported on physical interrupt pin INT1
#define DRDYI1	0x10// – Data ready interrupt reported on physical interrupt pin INT1
#define TDTI1	0x04// - Tap/Double Tap interrupt reported on physical interrupt pin INT1
#define WUFI1	0x02// – Wake-Up (motion detect) interrupt reported on physical interrupt pin INT1
#define TPI1	0x01// – Tilt position interrupt reported on physical interrupt pin INT1

//buffer ctrl2
#define BUFE	0x80	//controls activation of the sample buffer.
#define BRES	0x40	// determines the resolution of the acceleration data samples collected by the sample buffer.
#define BFIE	0x20	// buffer full interrupt enable bit



#define INS2_BFI		0x40
#define INS2_WMI		0x20
#define INS2_DRDY		0x10
#define INS2_TDTS1		0x08
#define INS2_TDTS0		0x04
#define INS2_WUFS		0x02
#define INS2_TPS		0x01

//Tilt Position Registers
#define TPR_LE	0x20	//Left State (X-)
#define TPR_RI	0x10	//Right State (X+)
#define TPR_DO	0x08	//Down State (Y-)
#define TPR_UP	0x04	//Up State (Y+)
#define TPR_FD	0x02	//Face-Down State (Z-)
#define TPR_FU	0x01	//Face-Up State (Z+)


//BUF_M1, BUF_M0 selects the operating mode of the sample buffer per Table 23.

#define DEFAULT_BUF_THRES	10

#define 	IO_KX022_INT_1_PIN			16
#define 	KX022_INT1_GPIOTE_USER_ID   1<<(IO_KX022_INT_1_PIN)



enum{
	wufs_event = 0x01,	//
	stap_event = 0x02,	//single tap
	dtap_event = 0x04,	//double tap
	tps_event  = 0x08,
	wmi_event  = 0x10,	//acceleration data
	tilt_event  = 0x20	//tilt event
};



typedef struct _kx023_ev_t{
	uint8_t	ev;
	uint8_t	flg;
	uint8_t size;
	void* 	data;
}kx023_ev_t;

typedef void (*kx023_evt_hdl_t)	(kx023_ev_t* pev);


uint16_t kx023_fetch_acc_data(void);
uint8_t drv_kx023_event_handle(void);
int kx023_enable_tilt(bool en);
int kx023_enable(void);
int kx023_disable(void);
int kx023_init(kx023_evt_hdl_t evt_hdl);


#endif   /* _drv_KX022_H_ */











