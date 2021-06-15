
#ifndef _OTAUART_CMD_H
#define _OTAUART_CMD_H


enum{
  OTA_MODE_OTA_APPLICATION= 0,
  OTA_MODE_OTA_FCT,     //1
  OTA_MODE_OTA,         //2
  OTA_MODE_RESOURCE,    //3
  OTA_MODE_UART         //4
};



void uartextf_timer_hdl(void);
void uartextf_cmdinit(void);


#endif

