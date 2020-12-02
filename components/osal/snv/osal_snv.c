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


#include <stdint.h>
#include "osal.h"
#include "flash.h"
#include "error.h"
#include "osal_snv.h"
#include "log.h"
#ifdef USE_FS
#include "fs.h"
#endif

#ifndef USE_FS
#define USE_FS 0
#endif
static void print_hex (const uint8 *data, uint16 len)
{
    uint16 i;

    for (i = 0; i < len - 1; i++)
    {
        LOG("%x,",data[i]);
        LOG(" ");
    }
    LOG("%x\n",data[i]);
}


#if (USE_FS == 0)
#define NVM_BASE_ADDR           0x11070000  //64K bytes
extern int ProgramWord (unsigned long offset, const unsigned char *buf,uint8_t size);   // size must be 4

static int snvwr(uint32_t addr, const uint8_t*  buf, uint32_t size)
{
  int i;
  addr &= 0x00ffffff;
  if((addr%4)){
    return PPlus_ERR_DATA_ALIGN;
  }
	size = size +3;
  size = size - (size%4);
  for(i = 0; i< size; i+=4){
    if(ProgramWord (addr, buf, 4) ==0 )
      return PPlus_ERR_SPI_FLASH;
    addr += 4;
    buf += 4;
  }
  return PPlus_SUCCESS;
}


static uint32_t snv_calc_addr(osalSnvId_t id)
{
  if(id == 0x70 || id == 0x71)
    return NVM_BASE_ADDR + 0x1000*12 + (id-0x70)*0x1000;
  else if(id >= 0x20 && id < 0x2c)
    return NVM_BASE_ADDR + 0x1000*(id-0x20);
  else
    return 0;
}

uint8 osal_snv_init( void )
{
  return SUCCESS;
}

uint8 osal_snv_read( osalSnvId_t id, osalSnvLen_t len, void *pBuf)
{
  uint32_t* pNv = (uint32_t*)snv_calc_addr(id);

  LOG("osal_snv_read:%x\n",id);

  if(pNv == NULL)
    return NV_OPER_FAILED;

  if(pNv[0] == 0xffffffff)
    return NV_OPER_FAILED;
  osal_memcpy(pBuf, (void*)(pNv+1), (uint32_t)len);
  print_hex(pBuf, len);
  return SUCCESS;
}

uint8 osal_snv_write( osalSnvId_t id, osalSnvLen_t len, void *pBuf)
{
  int ret = PPlus_SUCCESS;
  uint32_t tmp[16];
  uint8_t* pSrc = (uint8_t*)pBuf;
  uint32_t addr = snv_calc_addr(id);
  uint8_t len1 = 0;
  LOG("osal_snv_write:%x,%d\n",id,len);
  print_hex(pBuf, len);

  if(addr == 0)
    return NV_OPER_FAILED;
  
  flash_sector_erase(addr);

  osal_memset(tmp, 0, 16*4);

  tmp[0] = (uint32_t)id;

  len1 = len > 16*4-4 ? 16*4-4 : len;

  osal_memcpy((void*)(tmp+1), pSrc, len1);
  ret = snvwr(addr, (const uint8_t*)tmp, len1+4);
  
  len -= len1;
  pSrc += len1;
  
  if(ret !=0)
    return NV_OPER_FAILED;

  
  while(len){
    
    len1 = len > 16*4 ? 16*4 : len;

    osal_memcpy((void*)tmp, pSrc, len1);
    ret = snvwr(addr, (const uint8_t*)tmp, len1);
    
    len -= len1;
    pSrc += len1;
  }
  if(ret !=0)
    return NV_OPER_FAILED;
  //LOG("Success\n");
  return SUCCESS;
}

uint8 osal_snv_compact( uint8 threshold )
{
  return SUCCESS;

}

#else

uint8 osal_snv_init( void )
{
  if(!hal_fs_initialized())
    return NV_OPER_FAILED;
  return SUCCESS;
}

uint8 osal_snv_read( osalSnvId_t id, osalSnvLen_t len, void *pBuf)
{
  int ret;
  LOG("osal_snv_read:%x\n",id);

  ret = hal_fs_item_read((uint16_t)id,(uint8_t *) pBuf, (uint16_t)len,NULL);
  if(ret != PPlus_SUCCESS){
		LOG("rd_ret:%d\n",ret);
    return NV_OPER_FAILED;
	}
  print_hex(pBuf, len);
  return SUCCESS;
}

uint8 osal_snv_write( osalSnvId_t id, osalSnvLen_t len, void *pBuf)
{
  int ret = PPlus_SUCCESS;
  LOG("osal_snv_write:%x,%d\n",id,len);
  print_hex(pBuf, len);

  if(hal_fs_get_free_size() < len+32){
    if(hal_fs_get_garbage_size(NULL) > len+32){
      hal_fs_garbage_collect();
    }
    else
    {
      return NV_OPER_FAILED;
    }

  }
  
  ret = hal_fs_item_write((uint16_t) id, (uint8_t *) pBuf, (uint16_t) len);
  if(ret !=0){
		LOG("wr_ret:%d\n",ret);
		return NV_OPER_FAILED;
	}
   
  //LOG("Success\n");
  return SUCCESS;
}

uint8 osal_snv_compact( uint8 threshold ){
	return 0;
}

#endif

