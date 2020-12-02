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

#ifndef _BURN_H_
#define _BURN_H_

#include "types.h"


#define M4_BIN_NUM_OFFSET           0x2000
#define M4_BIN_ADDR_OFFSET(n)       (M4_BIN_NUM_OFFSET+4+n*12)
#define M4_BIN_SIZE_OFFSET(n)       (M4_BIN_NUM_OFFSET+8+n*12)
#define M4_RUN_ADDR_OFFSET(n)       (M4_BIN_NUM_OFFSET+12+n*12)

#define M0_BIN_NUM_OFFSET           0x2100
#define M0_BIN_ADDR_OFFSET(n)       (M0_BIN_NUM_OFFSET+4+n*12)
#define M0_BIN_SIZE_OFFSET(n)       (M0_BIN_NUM_OFFSET+8+n*12)
#define M0_RUN_ADDR_OFFSET(n)       (M0_BIN_NUM_OFFSET+12+n*12)


#define USR_DATA_BASE                0x4000

#define IMAGE_BASE_OFFSET           0x5000


#define  M4_BIN          1
#define  M0_BIN          2


#define BURN_BUF_LEN     64

typedef struct{
	uint8_t  valid;
	uint8_t  length;
	uint8_t  data[BURN_BUF_LEN];
	uint8_t  end;

}burn_buf_st;

//void handle_uart_command(void);


#endif
