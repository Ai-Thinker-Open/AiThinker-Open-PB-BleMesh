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



#include "bcomdef.h"
#include <string.h>
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "peripheral.h"
#include "gattservapp.h"

#include "cam_rx.h"
#include "spi.h"
#include "error.h"
#include "log.h"

#define BLECAM_DONGLE

#ifdef BLECAM_DONGLE

#define BLECAM_IO_SCLK  P34
#define BLECAM_IO_SSN   P31
#define BLECAM_IO_MOSI  P33
#define BLECAM_IO_MISO  P32
#define BLECAM_IO_INT   P14
#define BLECAM_IO_ACK   P15

#else
//case evb board
#define BLECAM_IO_SCLK  P24
#define BLECAM_IO_SSN   P23
#define BLECAM_IO_MOSI  P32
#define BLECAM_IO_MISO  P31
#define BLECAM_IO_INT   P14
#define BLECAM_IO_ACK   P15

#endif


typedef struct{
  uint16_t        offset;
  bool            err;
  uint8_t         frame_idx;
  uint16_t        frame_len;
  //camrx_evt_hdl_t callback;
  
}camrx_ctx_t;

static camrx_ctx_t s_camrx_ctx;
camrx_buf_t s_framebuf[2];


static hal_spi_t s_camrx_spi = {.spi_index = SPI0};

//static uint32_t s_cnt = 0;
static void camrx_spi_int_event(uint8_t* data, uint8_t len)
{
  
  if(len == 4)//case control
  {
    if(data[0] == 0x55 && data[1] == 0xaa)
    {
      s_camrx_ctx.frame_len = (data[2] | (data[3] << 8));
      s_camrx_ctx.offset = 0;
      s_camrx_ctx.err = false;
      s_camrx_ctx.frame_idx ++;
      s_camrx_ctx.frame_idx = s_camrx_ctx.frame_idx % 2;
      s_framebuf[s_camrx_ctx.frame_idx].used = 0;
      s_framebuf[s_camrx_ctx.frame_idx].size = s_camrx_ctx.frame_len;
      LOG(".");
    }
  }
  else if(len == 8)
  {
    uint8_t id  = s_camrx_ctx.frame_idx;
    camrx_buf_t* pbuf = &s_framebuf[id];
		if(s_camrx_ctx.frame_len == 0){
			return;
		}
    if(s_camrx_ctx.frame_len <= s_camrx_ctx.offset){
      return;
    }

    if(s_camrx_ctx.err){
      hal_spis_clear_rx(&s_camrx_spi);
      return;
		}
    memcpy(pbuf->buf + s_camrx_ctx.offset, data, 8);
    s_camrx_ctx.offset += 8;

  }

}

static void camrx_ce_up(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
  int len;
  uint8_t data[8];

  hal_gpio_fast_write(BLECAM_IO_ACK, 1);
  len = hal_spis_rx_len(&s_camrx_spi);
  if(len){
    hal_spis_read_rxn(&s_camrx_spi, data, len);
    camrx_spi_int_event(data, len);
    
  }
  hal_gpio_fast_write(BLECAM_IO_ACK, 0);
}


bool camrx_get_frame(camrx_buf_t* pfbuf)
{
  bool ret = true;
  uint8_t id;
  //disable gpio irq
  LOG("camrx_get_frame\n");

  NVIC_DisableIRQ((IRQn_Type)GPIO_IRQ);
  id = (s_camrx_ctx.frame_idx + 1) &1;
  if(s_framebuf[id].used){
    ret = false;
  }
  else
  {
    pfbuf->size = s_framebuf[id].size;
    memcpy(pfbuf->buf, s_framebuf[id].buf, pfbuf->size);
    s_framebuf[id].used = true;
    LOG("G:%d, size %d\n",id, pfbuf->size);
  }
  NVIC_EnableIRQ((IRQn_Type)GPIO_IRQ);
  return ret;
}

int camrx_init(void)
{
  memset(&s_framebuf[0], 0, sizeof(camrx_buf_t)*2);
  memset(&s_camrx_ctx, 0 ,sizeof(s_camrx_ctx));
  
  spi_Cfg_t cfg = {
      .sclk_pin = BLECAM_IO_SCLK,
      .ssn_pin = BLECAM_IO_SSN,
      .MOSI = BLECAM_IO_MOSI,
      .MISO = BLECAM_IO_MISO,
      .baudrate = 12000000,
      .spi_tmod = SPI_TRXD,
      .spi_scmod = SPI_MODE0,
      .force_cs = false,

      .int_mode = false,
      .evt_handler = NULL,
    };
  
  hal_spis_bus_init(&s_camrx_spi, cfg);

	hal_gpioin_register(BLECAM_IO_INT, camrx_ce_up, NULL);
  hal_gpio_write(BLECAM_IO_ACK, 0);

  return PPlus_SUCCESS;
}

