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


#ifndef _OTA_APP_SERVICE_H
#define _OTA_APP_SERVICE_H
#include "bcomdef.h"

enum{
  OTA_MODE_OTA_APPLICATION= 0,
  OTA_MODE_OTA_FCT,//          1
  OTA_MODE_OTA,//              2
  OTA_MODE_RESOURCE,//         3
  OTA_MODE_OTA_NADDR = 6//    6 ota no address plus
};

#define OTA_MODE_SELECT_REG 0x4000f034


#define OTA_APP_SERVICE_VERSION "V2.0.1"

enum{
  OTAAPP_CMD_START_OTA = 1,
  OTAAPP_CMD_INFO,
  OTAAPP_CMD_FORMAT,
  OTAAPP_CMD_VER,
};

typedef struct{
  uint8_t cmd;
  union{
    struct{
      uint8_t reserv[20-1];
    } dummy;
    struct{
      uint8_t mode;
    } start;
  } p;  //parameter
}ota_app_cmd_t;


bStatus_t ota_app_AddService(void);

int ota_vendor_module_StartOTA(uint8_t mode);
int ota_vendor_module_Version(  uint8_t* major, uint8_t* minor, uint8_t* revision, uint8_t *test_build);


#endif

