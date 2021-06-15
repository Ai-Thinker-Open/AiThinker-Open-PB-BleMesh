
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "common.h"
#include "hal_mcu.h"
#include "uart.h"
#include "version.h"
#include "uart_extf_cmd.h"
#include "error.h"
#include "crc16.h"
#include "osal.h"
#include "uart_extf.h"
#include "spi_flash_drv.h"
#include "log.h"


#define OTAU_PART_NUM_MAX     32

#define FLASH_SIZE_MAX        0x200000
#define FLASH_PARTITION_SIZE  65536

#define OTAUC_CMD_SIZE  128
#define OTAUC_PART_SIZE  (64*1024)

#define SET_STATE(st) {HAL_ENTER_CRITICAL_SECTION(); m_otau_ctx.state = st; HAL_EXIT_CRITICAL_SECTION();}

#define RSP_ERR() response("#ER>>:")
#define RSP_OK() response("#OK>>:")

enum{
  OTAUC_ST_UNINIT = 0,
  OTAUC_ST_IDLE,
  OTAUC_ST_RUNCMD,
  OTAUC_ST_DATA,
  OTAUC_ST_WRITEDATA,

};

typedef struct{
  uint32_t  flash_addr;
  uint32_t  run_addr;
  uint32_t  size;
  uint32_t  checksum;
  uint16_t  crc16;
}otau_part_t;


typedef struct{
  uint8_t   state;
  uint8_t   chk_phase;

  int       erased;

  uint8_t   bank_mode;  //single bank or 
  uint32_t  bank_addr;
  
  uint8_t   cmd_offset;
  uint8_t   cmd[OTAUC_CMD_SIZE];
  
  uint32_t   part_num;
  uint32_t   part_offset;
  otau_part_t part;

  uint32_t  app_offset;
  
  uint8_t*  part_buf;
  
}otau_ctx_t;



otau_ctx_t m_otau_ctx = {
  .state = OTAUC_ST_UNINIT,
};

uint32_t UART_TO_INTV =20; //100ms

uint32_t s_uart_int_cnt = 0;

static uint32_t m_part_buf[OTAUC_PART_SIZE/4];

void uartextf_evt_hdl(uart_Evt_t* pev);

void response(const char* str)
{
  //if(strcmp(str, "#ER>>:") == 0){
  //  UART_TO_INTV++;
  //}
  hal_uart_send_buff((uint8_t*)str, strlen((const char*)str));
}

static void print_hex (uint8_t *data, uint16 len)
{
  uint16 i;
  char strbuf[128];
  for (i = 0; i < len; i++)
  {
    if((i%32)==0 && i>0){
      response("\n");
    }
    sprintf(strbuf, "%.2x ",data[i]);
    response(strbuf);
  }
  response("\n\n");
}

static void reset_cmd_buf(void)
{
  memset(m_otau_ctx.cmd, 0, OTAUC_CMD_SIZE);
  m_otau_ctx.cmd_offset = 0;

  //reset part info
  m_otau_ctx.part_offset = 0;
  memset(&(m_otau_ctx.part), 0, sizeof(otau_part_t));

  m_otau_ctx.state = OTAUC_ST_RUNCMD;
}

//if return true, set state to idle, if return false, do nothing
static bool process_cmd(void)
{
  int ret;
  otau_ctx_t* pctx = &m_otau_ctx;
  otau_part_t* ppart = &pctx->part;
  if(pctx->chk_phase)
  {
    char* p = NULL;
    uint32_t checksum = 0;
    pctx->chk_phase = FALSE;
    p = strtok((char*)pctx->cmd, " ");
    checksum = strtoul(p, NULL, 16);
    if(checksum == ppart->checksum){
      RSP_OK();
    }
    else
    {
      RSP_ERR();
    }
  }
  else if(strncmp((const char*)(pctx->cmd), "er512", 5) == 0)
  {
    spiflash_erase_all();
    pctx->erased = TRUE;
    RSP_OK();
  }
  else if(strncmp((const char*)(pctx->cmd), "clock", 5) == 0)
  {
    RSP_OK();
  }
  else if(strncmp((const char*)(pctx->cmd), "uarts", 5) == 0)
  {
    char* p = (char*)(pctx->cmd+5);
    int boardrate = strtoul(p, NULL, 10);
    uart_Cfg_t cfg = {
    .tx_pin = OTAUART_TX,
    .rx_pin = OTAUART_RX,
    .rts_pin = GPIO_DUMMY,
    .cts_pin = GPIO_DUMMY,
    //.baudrate = 115200,
    .baudrate = boardrate,//115200,
    //.baudrate = 1000000,//115200,
    .use_fifo = TRUE,
    .hw_fwctrl = FALSE,
    .use_tx_buf = FALSE,
    .parity     = FALSE,
    .evt_handler = uartextf_evt_hdl,
    };

    RSP_OK();
    hal_uart_deinit();
    hal_uart_init(cfg);
   
  }
  else if(strncmp((const char*)(pctx->cmd), "cpnum ", 6) == 0)
  {
    //cpnum xx
    char* p = NULL;
    int tmp_len = 0;
    int cmd_len = strlen((const char*)pctx->cmd);
    p = strtok((char*)pctx->cmd, " ");
    p = strtok(NULL, " ");
    if(p == NULL)
    { RSP_ERR(); return TRUE;}


    tmp_len = strlen(p) + 1;
    
    pctx->app_offset = 0;
    RSP_OK();

    if(cmd_len > tmp_len+6 + 5){
      memcpy(pctx->cmd , pctx->cmd + 6 + tmp_len, cmd_len +6 + 5 +3);
      if(strncmp((const char*)(pctx->cmd), "cpbin ", 6) == 0)
        {
          //cpbin idx flashaddr len runaddr
          uint32_t faddr, len, raddr;
          char* p = NULL;
          p = strtok((char*)pctx->cmd, " ");

          p = strtok(NULL, " ");
          if(p == NULL)
          {RSP_ERR(); return TRUE;}
          //idx = strtoul(p, NULL, 16);

          p = strtok(NULL, " ");
          if(p == NULL)
          {RSP_ERR(); return TRUE;}
          faddr = strtoul(p, NULL, 16);

          if((faddr & 0x80000000) == 0){
            RSP_ERR(); return TRUE;
          }
          
          p = strtok(NULL, " ");
          if(p == NULL)
          {RSP_ERR(); return TRUE;}
          len = strtoul(p, NULL, 16);
          if(len > FLASH_PARTITION_SIZE)
          {RSP_ERR(); return TRUE;}

          p = strtok(NULL, " ");
          if(p == NULL)
          {RSP_ERR(); return TRUE;}
          raddr = strtoul(p, NULL, 16);

          ppart->checksum = 0;
          ppart->crc16 = 0;
          ppart->flash_addr = faddr;
          ppart->run_addr = raddr;
          ppart->size = len;

          pctx->part_offset = 0;
        
          SET_STATE(OTAUC_ST_DATA);
          UART_TO_INTV = 500;

          if(pctx->erased == FALSE){
            ret = spiflash_erase(faddr, len);
            if(ret != PPlus_SUCCESS)
            {RSP_ERR(); return TRUE;}
          }
          
          response("by hex mode:");
          return FALSE;
          
        }  
    }
    
  }
  else if(strncmp((const char*)(pctx->cmd), "cpbin ", 6) == 0)
  {
    //cpbin idx flashaddr len runaddr
    uint32_t faddr, len, raddr;
    char* p = NULL;
    p = strtok((char*)pctx->cmd, " ");

    p = strtok(NULL, " ");
    if(p == NULL)
    {RSP_ERR(); return TRUE;}
    //idx = strtoul(p, NULL, 16);

    p = strtok(NULL, " ");
    if(p == NULL)
    {RSP_ERR(); return TRUE;}
    faddr = strtoul(p, NULL, 16);
    if((faddr & 0x80000000) == 0){
      RSP_ERR(); return TRUE;
    }
          
    
    p = strtok(NULL, " ");
    if(p == NULL)
    {RSP_ERR(); return TRUE;}
    len = strtoul(p, NULL, 16);
    if(len > FLASH_PARTITION_SIZE)
    {RSP_ERR(); return TRUE;}

    p = strtok(NULL, " ");
    if(p == NULL)
    {RSP_ERR(); return TRUE;}
    raddr = strtoul(p, NULL, 16);

    ppart->checksum = 0;
    ppart->crc16 = 0;
    ppart->flash_addr = faddr;
    ppart->run_addr = raddr;
    ppart->size = len;

    pctx->part_offset = 0;
  
    SET_STATE(OTAUC_ST_DATA);
    UART_TO_INTV = 500;

    if(pctx->erased == FALSE){
      ret = spiflash_erase(faddr, len);
      if(ret != PPlus_SUCCESS)
      {RSP_ERR(); return TRUE;}
    }
    
    response("by hex mode:");
    return FALSE;
    
  }
  else if(strncmp((const char*)(pctx->cmd), "crc16", 5) == 0)
  {
    char tmpstr[11];
    sprintf(tmpstr, "0x%.8lx", ppart->crc16);
    response(tmpstr);
  }
  else if(strncmp((const char*)(pctx->cmd), "read ", 5) == 0)
  {
    uint32_t faddr, len;
    uint8_t* pdata = pctx->part_buf;
    char* p = NULL;
    p = strtok((char*)pctx->cmd, " ");

    p = strtok(NULL, " ");
    if(p == NULL)
    {RSP_ERR(); return TRUE;}
    faddr = strtoul(p, NULL, 16);
          
    
    p = strtok(NULL, " ");
    if(p == NULL)
    {RSP_ERR(); return TRUE;}
    len = strtoul(p, NULL, 16);
    if(len > FLASH_PARTITION_SIZE)
    {RSP_ERR(); return TRUE;}

    spiflash_read(faddr, pdata, len);
		pdata[len] = 0;
    
    print_hex(pdata, len);
		response((const char*)pdata);
    
  }
  else
  {
    RSP_ERR();
  }
  
  return TRUE;
}
uint8_t flash_veirf_buf[1024];
static void process_data(void)
{
  int i;
  uint32_t flash_addr = 0;
  otau_ctx_t* pctx = &m_otau_ctx;
  otau_part_t* ppart = &pctx->part;
  char rsp_str[30];
    uint32_t check = 0;


  if(pctx->part_offset != ppart->size){
    reset_cmd_buf();
    return;
  }

  pctx->state = OTAUC_ST_WRITEDATA;
  ppart->checksum = 0;
  flash_addr = ppart->flash_addr;
  //figure checksum
  for(i = 0; i< ppart->size; i++){
    ppart->checksum += pctx->part_buf[i];
  }
  //figure crc16
  ppart->crc16 = crc16(0, pctx->part_buf, ppart->size);

  hal_gpio_write(P19,1);
  spiflash_erase(flash_addr&0x7fffffff, ppart->size);
  spiflash_write(flash_addr&0x7fffffff, pctx->part_buf, ppart->size);
  hal_gpio_write(P19,0);
  {
    uint32_t size = ppart->size, len = 0;
    uint32_t offset = 0;
    {
			check = 0;
			size = ppart->size;
			len = 0;
			offset = 0;
			
      while(size){
        len = size > 1024? 1024:size;
        spiflash_read((flash_addr+offset)&0x7fffffff, flash_veirf_buf, len);
        size -= len;
        offset += len;
			  for(i = 0; i< len; i++){
					check += flash_veirf_buf[i];
				}

      }
    }
  }
  pctx->chk_phase = TRUE;
  sprintf(rsp_str, "checksum is: 0x%.8lx", check);
  ppart->checksum = check;
  response(rsp_str);
  SET_STATE(OTAUC_ST_IDLE);
  UART_TO_INTV = 100;
}

void process_uart_rx(uint8_t* data,uint8_t len)
{
  otau_ctx_t* pctx = &m_otau_ctx;
  otau_part_t* ppart = &(pctx->part);
  switch(pctx->state){
  case OTAUC_ST_UNINIT:    //device uninit ,just drop data, no response
  case OTAUC_ST_RUNCMD:    //command running, just wait
  case OTAUC_ST_WRITEDATA: //writing data to flash, just wait
    break;
  case OTAUC_ST_IDLE:
    if(pctx->cmd_offset + len < OTAUC_CMD_SIZE) //illegal
    {
      memcpy(pctx->cmd + pctx->cmd_offset, data, len);
      pctx->cmd_offset += len;
    }
    break;
  case OTAUC_ST_DATA:
    if(pctx->part_offset + len <= ppart->size) //illegal
    {
      memcpy(pctx->part_buf + pctx->part_offset, data, len);
      pctx->part_offset += len;
    }
    else{
      s_uart_int_cnt+=0x10000;
    }
    break;
    
  default:
    break;
  }
  
}

void uartextf_timer_hdl(void)
{
  otau_ctx_t* pctx = &m_otau_ctx;
  switch(pctx->state){
  case OTAUC_ST_UNINIT:    //device uninit ,just drop data, no response
  case OTAUC_ST_RUNCMD:    //command running, just wait
  case OTAUC_ST_WRITEDATA: //writing data to flash, just wait
    RSP_ERR();
    SET_STATE(OTAUC_ST_IDLE);
    break;
  case OTAUC_ST_IDLE:
  {
    SET_STATE(OTAUC_ST_RUNCMD);
    if(process_cmd()){
      SET_STATE(OTAUC_ST_IDLE);
    }
    memset(m_otau_ctx.cmd, 0, OTAUC_CMD_SIZE);
    m_otau_ctx.cmd_offset = 0;
    break;
  }
  case OTAUC_ST_DATA:
    process_data();

  }
}


void uartextf_evt_hdl(uart_Evt_t* pev)
{
  s_uart_int_cnt++;
  switch(pev->type)
  {
    case  UART_EVT_TYPE_RX_DATA:
    case  UART_EVT_TYPE_RX_DATA_TO:
      osal_stop_timerEx(otauart_TaskID, OTA_TIMER_EVT);
      osal_clear_event(otauart_TaskID, OTA_TIMER_EVT);
      osal_start_timerEx(otauart_TaskID, OTA_TIMER_EVT, UART_TO_INTV);
      process_uart_rx(pev->data,pev->len);
      break;
    case  UART_EVT_TYPE_TX_COMPLETED:
      break;
    default:
      break;
  }

}





void uartextf_cmdinit(void)
{
  otau_ctx_t* pctx = &m_otau_ctx;

  uart_Cfg_t cfg = {
  .tx_pin = OTAUART_TX,
  .rx_pin = OTAUART_RX,
  .rts_pin = GPIO_DUMMY,
  .cts_pin = GPIO_DUMMY,
  .baudrate = 115200,
  //.baudrate = 500000,//115200,
  //.baudrate = 1000000,//115200,
  .use_fifo = TRUE,
  .hw_fwctrl = FALSE,
  .use_tx_buf = FALSE,
  .parity     = FALSE,
  .evt_handler = uartextf_evt_hdl,
  };

  memset(&m_otau_ctx, 0, sizeof(m_otau_ctx));
  
  hal_uart_init(cfg);

  spiflash_init();

  pctx->part_buf = (uint8_t*)m_part_buf;
	SET_STATE(OTAUC_ST_IDLE);
  

  response("cmd>>:");
}

