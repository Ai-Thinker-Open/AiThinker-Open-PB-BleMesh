

#include "spi_flash_drv.h"
#include "spi.h"
#include "error.h"
#include "hal_mcu.h"
#include "string.h"
#include "log.h"
static void spi_cb(spi_evt_t* evt);
static void send_write_enable_cmd(void);
static uint16_t read_status_register(uint8_t bitsSel);//0~low other~high


#define BUF_LEN 64 + 4
//#define BUF_LEN 32 + 4
static uint8_t s_tx_buf[BUF_LEN];
static uint8_t s_rx_buf[BUF_LEN];
hal_spi_t spi ={
	.spi_index = SPI1,
};

spi_Cfg_t spi_flash_cfg = {
  	.sclk_pin = GPIO_P34,
  	.ssn_pin = GPIO_P31,
  	.MOSI = GPIO_P32,
  	.MISO = GPIO_P33,
  	
  	.baudrate = 12000000,//12M
  	//.baudrate = 1000000,//4M
  	//.baudrate = 8000000,//12M
  	.spi_tmod = SPI_TRXD,
  	.spi_scmod = SPI_MODE3,
  	
  	.int_mode = false,
  	.force_cs = true,
  	.evt_handler = spi_cb,
  };

static void spi_cb(spi_evt_t* evt)
{
	//LOG("(SPI finish:%d %d)	",evt.spi,evt.type);
}

static void read_set_buffer(uint32_t addr, uint16_t len)
{
	
	memset(s_tx_buf,0x00,BUF_LEN);
	memset(s_rx_buf,0x00,BUF_LEN);

	*(s_tx_buf+0) = FLASH_READ;
	*(s_tx_buf+1) = (addr>>16)&0xff;
	*(s_tx_buf+2) = (addr>>8)&0xff;
	*(s_tx_buf+3) = addr & 0xff;
}



static void write_set_buffer(uint32_t addr,uint8_t* tx_data,uint16_t len)
{
	
	memset(s_tx_buf,0x00,len +4);
	memset(s_rx_buf,0x00,len+4);

	s_tx_buf[0] = FLASH_PP;
	s_tx_buf[1] = (addr>>16)&0xff;
	s_tx_buf[2] = (addr>>8)&0xff;
	s_tx_buf[3] = addr & 0xff;
  memcpy(s_tx_buf+4, tx_data, len);
}

static bool spiflash_busy(void)
{
	return (read_status_register(0) & 0x01);
}


static uint16_t read_status_register(uint8_t bitsSel)//0~low other~high
{
	uint8_t buf_send[4] = {0x00,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	if(bitsSel == 0)
	{
		buf_send[0] = FLASH_RDSR_LOW;
	}
	else
	{
		buf_send[0] = FLASH_RDSR_HIGH;
	}
	
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,3))
	{
		return (buf_rece[1] << 8) | (buf_rece[2]);
	}
	else
	{
		return 0xFFFF;
	}
}

static void write_status_register(uint8_t data)
{	
	uint8_t buf_send[2] = {FLASH_WRSR,0x00};
	uint8_t buf_rece[2] = {0x00,0x00};
	
	buf_send[1] = data;
	//while(spiflash_busy());
	send_write_enable_cmd();	
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,2))
	{
		;
	}
}

static void send_write_enable_cmd(void)
{
	uint8_t buf_send[1] = {FLASH_WREN};
	uint8_t buf_rece[1] = {0x00};
	
	while(spiflash_busy());
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,1))
	{
		;
	}
}


void write_enable(void)
{
	//uint16_t ret16;
	
	//ret16 = read_status_register(0);
	//if(ret16 != 0)
	//{
	//	write_status_register(0x00);
	//while(spiflash_busy() == TRUE)	{ ;	}
	///}
	send_write_enable_cmd();
}


void chip_erase(void)
{	
	uint8_t buf_send[1] = {FLASH_CE};
	uint8_t buf_rece[1] = {0x00};
	
	buf_send[0] = FLASH_CE;
	send_write_enable_cmd();
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,1))
	{
		;
	}
}

void sector_erase(uint32_t addr)
{	
	uint8_t buf_send[4] = {FLASH_SE,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	buf_send[1] = (addr>>16) & 0xff;
	buf_send[2] = (addr>>8) & 0xff;
	buf_send[3] = addr & 0xff;

	send_write_enable_cmd();
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,4))
	{
		;
	}
}

static void block_erase_32KB(uint32_t addr)
{
	uint8_t buf_send[4] = {FLASH_BE_32KB,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	buf_send[1] = (addr>>16) & 0xff;
	buf_send[2] = (addr>>8) & 0xff;
	buf_send[3] = addr & 0xff;

	send_write_enable_cmd();
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,4))
	{
		;
	}
}

static void block_erase_64KB(uint32_t addr)
{
	uint8_t buf_send[4] = {FLASH_BE_64KB,0x00,0x00,0x00};
	uint8_t buf_rece[4] = {0x00,0x00,0x00,0x00};
	
	buf_send[1] = (addr>>16) & 0xff;
	buf_send[2] = (addr>>8) & 0xff;
	buf_send[3] = addr & 0xff;

	send_write_enable_cmd();
	if(PPlus_SUCCESS == hal_spi_transmit(&spi,buf_send,buf_rece,4))
	{
		;
	}
}

int spiflash_write(uint32_t addr, uint8* data, uint32_t len)
{
  uint16_t blk_size;
	hal_spi_bus_init(&spi,spi_flash_cfg);
  
  while(len){
    blk_size = (len > BUF_LEN -4)?(BUF_LEN -4):len;
    hal_spi_set_force_cs(&spi,true);
    hal_spi_set_int_mode(&spi,false);
    //hal_spis_clear_rx(&spi);
    write_enable();
    write_set_buffer(addr, data, blk_size);
    hal_spi_transmit(&spi,s_tx_buf,s_rx_buf,blk_size+4);
    len-= blk_size;
    data += blk_size;
    addr += blk_size;
  }
  while(spiflash_busy());
	hal_spi_bus_deinit(&spi);
  return PPlus_SUCCESS;
}
void spi_gd25q16_read_set_buffer(uint32_t addr, uint16_t len)
{
	
	hal_setMem(s_tx_buf,0x00,BUF_LEN);
	hal_setMem(s_rx_buf,0x00,BUF_LEN);

	*(s_tx_buf+0) = FLASH_READ;
	*(s_tx_buf+1) = (addr>>16)&0xff;
	*(s_tx_buf+2) = (addr>>8)&0xff;
	*(s_tx_buf+3) = addr & 0xff;
}

// int spi_read(uint32_t addr, uint8* data, uint16_t len)
// {
//   hal_spi_set_force_cs(&spi,true);
//   hal_spi_set_int_mode(&spi,false);
//   spi_gd25q16_read_set_buffer(addr, len);
//   hal_spi_transmit(&spi,s_tx_buf,s_rx_buf,len+4);
//   memcpy(data, s_rx_buf + 4, len);
// }


int spiflash_read(uint32_t addr, uint8* data, uint32_t len)
{
  uint16_t blk_size;
	hal_spi_bus_init(&spi,spi_flash_cfg);
  while(len){
    blk_size = (len > BUF_LEN -4)?(BUF_LEN -4):len;
    hal_spi_set_force_cs(&spi,true);
    hal_spi_set_int_mode(&spi,false);
    //hal_spis_clear_rx(&spi);
    read_set_buffer(addr, blk_size);
    hal_spi_transmit(&spi,s_tx_buf,s_rx_buf,blk_size+4);
    memcpy(data, s_rx_buf + 4, blk_size);
    len-= blk_size;
    data += blk_size;
    addr += blk_size;
  }
	hal_spi_bus_deinit(&spi);
  return PPlus_SUCCESS;
}


int spiflash_erase(uint32_t addr,uint32_t len)
{
	uint8_t lockinfo = 0;
	uint32_t remainder = 0;

	if((addr >= 0x200000) || (len == 0))
		return PPlus_ERR_INVALID_LENGTH;

	hal_spi_bus_init(&spi,spi_flash_cfg);

	
	lockinfo = read_status_register(0);
	write_status_register(0x00);
	
	if((addr == 0) && (len == 0x200000))
	{
		chip_erase();
	}
	else
	{
		remainder = addr%0x1000;//4KB
		if(remainder){
			addr -= remainder;
			len += remainder;
		}
		remainder = len%0x1000;//4KB
		if(remainder){
			len = len + 0x1000 - remainder;
		}
		
		addr = addr/0x1000;
		len = len/0x1000;
		
		while(len > 0){
			//LOG("addr:%d len:%d\n",addr,len);
			if(((addr %16) == 0) && (len >= 16)){
					while(spiflash_busy());
					block_erase_64KB(addr*0x1000);
					addr += 16;
					len -= 16;
					continue;
				}
			
			if(((addr %8) == 0) && (len >= 8)){
					while(spiflash_busy());
					block_erase_32KB(addr*0x1000);
					addr += 8;
					len -= 8;
					continue;
			}
				
			if(len >= 1){
					while(spiflash_busy());
					sector_erase(addr*0x1000);
					addr += 1;
					len -= 1;
					continue;
			}
		}
	}
	
	write_status_register(lockinfo);
	while(spiflash_busy());
	hal_spi_bus_deinit(&spi);

	return PPlus_SUCCESS;
}

int spiflash_erase_all(void)
{
	hal_spi_bus_init(&spi,spi_flash_cfg);
  chip_erase();
  while(spiflash_busy());

	hal_spi_bus_deinit(&spi);
  return PPlus_SUCCESS;
}


int spiflash_init(void)
{
#if 0
  spi_flash_cfg.sclk_pin = P31,
  spi_flash_cfg.ssn_pin = P24,
  spi_flash_cfg.MOSI = P25,
  spi_flash_cfg.MISO = P16,
  hal_gpio_cfg_analog_io(P16,Bit_DISABLE);

#endif

	hal_spi_bus_init(&spi,spi_flash_cfg);
	hal_spi_bus_deinit(&spi);
	//{
	//	uint8_t tmp[0x20];
	//	spi_read(0, tmp, 0x10);
	//}
	return PPlus_SUCCESS;
}



