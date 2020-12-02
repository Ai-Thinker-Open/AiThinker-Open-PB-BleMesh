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


#ifndef __OTA_MAST_PROTO_
#define __OTA_MAST_PROTO_

#include "ota_flash.h"


#define OTA_BURST_SIZE_DEFAULT    16
#define OTA_BURST_SIZE_HISPEED    0xff


enum{
  OTAP_EVT_DISCONNECTED =1,
  OTAP_EVT_CONNECTED,
  OTAP_EVT_NOTIFY,
  OTAP_EVT_DATA_WR_DELAY,
  OTAP_EVT_BLE_TIMEOUT,
};


typedef struct{
  uint8_t   run_mode;
  uint16_t  mtu;
}otam_proto_conn_param_t;

typedef struct{
  uint8_t   ev;
  uint16_t  len;
  void*     data;
}otap_evt_t;

typedef int (*otam_clear_t)(void);
typedef int (*otam_wcmd_op_t)(uint8_t* data, uint16_t len, uint32_t timeout);
typedef int (*otam_wdata_op_t)(uint8_t* data, uint16_t len);
typedef int (*otam_wdata_delay_t)(uint32_t msec_delay);

typedef struct{
  otam_clear_t        clear;
  otam_wcmd_op_t      write_cmd;
  otam_wdata_op_t     write_data;
  otam_wdata_delay_t   write_data_delay;
}otam_proto_meth_t;



void otamProtocol_event(otap_evt_t* pev);
int otamProtocol_start_ota(ota_fw_t* pffw);
int otamProtocol_stop_ota(void);
int otamProtocol_app_start_ota(uint8_t mode);
int otamProtocol_init(otam_proto_meth_t* method);

#endif

