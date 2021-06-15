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


#ifndef _OTA_PROTOCOL_H
#define _OTA_PROTOCOL_H

#define MAX_OTA_PARAM_SIZE  256
//#define OTA_BLOCK_SIZE    1024




#define OTA_DATA_BURST_SIZE (((uint32_t)s_ota_ctx.mtu_a)*((uint32_t)s_ota_burst_size))


enum{
  OTA_CMD_START_OTA = 1,
  OTA_CMD_PARTITION_INFO,
  OTA_CMD_BLOCK_INFO,
  OTA_CMD_REBOOT,
  OTA_CMD_ERASE,

  OTA_RSP_START_OTA = 0x81,
  OTA_RSP_PARAM,          //82
  OTA_RSP_OTA_COMPLETE,   //83
  OTA_RSP_PARTITION_INFO, //84
  OTA_RSP_PARTITION_COMPLETE, //85
  OTA_RSP_BLOCK_INFO,         //86
  OTA_RSP_BLOCK_BURST,        //87
  OTA_RSP_BLOCK_COMPLETE,     //88
  OTA_RSP_ERASE,              //89
  OTA_RSP_REBOOT,             //8a
  OTA_RSP_ERROR = 0xff,

};

typedef struct{
  uint8_t cmd;
  
  union{
    
    struct{
      uint8_t sector_num;
      uint8_t burst_size;
    } start;
    struct{
      uint8_t index;
      uint8_t flash_addr[4];
      uint8_t run_addr[4];
      uint8_t size[4];
      uint8_t checksum[2];
    } part;

    struct{
      uint8_t size[2];  //max 1024
      uint8_t index;
    } block;
    
    struct{
      uint8_t flash_addr[4];
      uint8_t size[4];
    } erase;

    uint8_t reboot_flag;

  } p;  //parameter
}ota_cmd_t;

extern const char* OTA_CRYPTO_IV;
extern bool aes_ccm_phyplus_dec(const unsigned char* iv, unsigned char* din,int dLen, unsigned char* micIn, unsigned char*dout);

void otaProtocol_mtu(uint16_t mtu);
void otaProtocol_TimerEvt(void);
bool otaProtocol_address_plus(void);
void otaProtocol_BootMode(void);
void otaProtocol_RunApp(void);
int otaProtocol_init(uint8_t task_id, uint16_t tm_evt);


#endif

