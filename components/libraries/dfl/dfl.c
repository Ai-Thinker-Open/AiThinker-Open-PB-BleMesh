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


#include "dfl.h"
#include "hal_mcu.h"
#include "error.h"
#include <crc16.h>
#include <string.h>


//dynamic function loader

typedef struct{
  bool      enable;
  uint32_t  dfp_addr;
  uint32_t  run_addr;
  bool      constructor;
  uint32_t  dfp_version;
  uint8_t   dfp_num;
  uint8_t   dfp_default;  //if dfp_default == 0xff, not load default dfp when dfp_init
  uint8_t   dfp_current;
}dfl_ctx_t;

static dfl_ctx_t s_dfl_ctx;

uint32_t DFL_ENTRY_BASE;

#define _YEAR ((__DATE__ [9] - '0') * 10 + (__DATE__ [10] - '0'))
  
#define _MONTH (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 0 : 5) \
              : __DATE__ [2] == 'b' ? 1 : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) \
              : __DATE__ [2] == 'y' ? 4 : __DATE__ [2] == 'n' ? 5 : __DATE__ [2] == 'l' ? \
              6 : __DATE__ [2] == 'g' ? 7 : __DATE__ [2] == 'p' ? 8 : __DATE__ [2] == 't' ? 9 : __DATE__ [2] == 'v' ? 10 : 11)  
  
#define _DAY ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 + (__DATE__ [5] - '0'))

#define _HOUR ((__TIME__ [0] - '0') * 10 + (__TIME__ [1] - '0'))
#define _MINUTE ((__TIME__ [3] - '0') * 10 + (__TIME__ [4] - '0'))
#define _SECOND ((__TIME__ [6] - '0') * 10 + (__TIME__ [7] - '0'))


static bool validate_symble_datetime(uint32_t faddr)
{
  uint8_t year, month, day, hour, minute, second;
  uint8_t* pflash = (uint8_t*)faddr;

  uint32_t tm, tm1;

  second  = pflash[0];
  minute  = pflash[1];
  hour    = pflash[2];
  day     = pflash[3];
  month   = pflash[4];
  year    = pflash[5];

  if(year != _YEAR)
    return FALSE;
  if(month != _MONTH)
    return FALSE;
  if(day != _DAY)
    return FALSE;

  tm = hour * 3600 + minute * 60 + second;
  tm1 = _HOUR * 3600 + _MINUTE * 60 + _SECOND;

  if((tm - tm1) < 120)
    return TRUE;
	return FALSE;
  
}


/*
dfp package description
index(word)   | desc
0             | flag: 'DFL:'
1             | package num(1~32)--N
2             | run constructor after load(0 | 1)
3             | default dfp
4             | dfp version
5~15          | reserved
16~16+32*4    | package info(flash address|run address|size|crc)
144~~~~       | package binary, each package size should align 4


dfp patition description
index(word)   | desc
0~1           | build datetime
2~3           | reserved
4             | first function entry
5~N           | function entry

*/


int dfl_init(uint32_t dfp_addr, uint32_t run_addr, uint32_t run_size)
{
  uint8_t i;
  uint32_t* pdfp = (uint32_t*)dfp_addr;
  char* flag = (char*)pdfp;
  uint32_t value = 0;

  memset(&s_dfl_ctx, 0, sizeof(s_dfl_ctx));

  DFL_ENTRY_BASE = run_addr;
  
  if(flag[0] != 'D' || flag[1] != 'F' || flag[2] != 'L' || flag[3] != ':' ){
    return PPlus_ERR_INVALID_DATA;
  }

  value = pdfp[1];
  if(value > 32)
    return PPlus_ERR_INVALID_DATA;
  s_dfl_ctx.dfp_num = (uint8_t)value;

  value = pdfp[2];
  if(value > 1)
    return PPlus_ERR_INVALID_DATA;
  s_dfl_ctx.constructor = (value == 1);

  value = pdfp[3];
  if(value == 0xffffffff)
    s_dfl_ctx.dfp_default = 0xff;
  else if(value < s_dfl_ctx.dfp_num)
    s_dfl_ctx.dfp_default = value;
  else
    return PPlus_ERR_INVALID_DATA;

  value = pdfp[4];
  s_dfl_ctx.dfp_version = value;
  
  pdfp = pdfp + 16;

  for(i = 0; i< s_dfl_ctx.dfp_num; i++){
    uint32_t faddr, raddr, size, crc;
    faddr = pdfp[i*4] + dfp_addr;
    raddr = pdfp[i*4+1];
    size = pdfp[i*4+2];
    crc = (uint16_t)pdfp[i*4+3];

    if(run_addr != raddr)
      return PPlus_ERR_INVALID_DATA;
    if(size >= run_size || (size%4) )
      return PPlus_ERR_INVALID_DATA;

    //check build symble date
    if(validate_symble_datetime(faddr) == FALSE){
      return PPlus_ERR_VERSION;
    }
      

    //check crc
    if(crc != crc16(0, (const volatile void *)faddr, size)){
      return PPlus_ERR_INVALID_DATA;
    }
  }

  s_dfl_ctx.enable = TRUE;
  s_dfl_ctx.dfp_addr = dfp_addr;
  s_dfl_ctx.run_addr = run_addr;
  s_dfl_ctx.dfp_current = 0xff;

  if(s_dfl_ctx.dfp_default != 0xff){
    return dfl_load(s_dfl_ctx.dfp_default);
  }

  return PPlus_SUCCESS;

}

uint8_t dfl_num(void)
{
  if(s_dfl_ctx.enable)
    return s_dfl_ctx.dfp_num;
  return 0xff;
}

uint8_t dfl_current(void)
{
  if(s_dfl_ctx.enable)
    return s_dfl_ctx.dfp_current;
  return 0xff;
}

typedef void(*dfp_default_call_t)(void);

int dfl_load(uint8_t dfp_index)
{
  uint32_t faddr, raddr, size;
  uint32_t* pdfp = (uint32_t*)(s_dfl_ctx.dfp_addr);
  pdfp = pdfp + 16;

  faddr = pdfp[dfp_index*4] + s_dfl_ctx.dfp_addr;
  raddr = pdfp[dfp_index*4+1];
  size = pdfp[dfp_index*4+2];
  
  HAL_ENTER_CRITICAL_SECTION();
  memcpy((void*)raddr, (void*)faddr, size);
  HAL_EXIT_CRITICAL_SECTION();

  s_dfl_ctx.dfp_current = dfp_index;

  if(s_dfl_ctx.constructor)
  {
    ((dfp_default_call_t)DFL_FUNC(0))();
  }

  return PPlus_SUCCESS;
}

