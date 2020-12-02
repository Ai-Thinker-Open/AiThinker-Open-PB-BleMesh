
#ifndef __OTAM_1CLK_OTA_H
#define __OTAM_1CLK_OTA_H

enum{
  ONECLK_EVT_CONNECTED,
  ONECLK_EVT_TERMINATED,
  ONECLK_EVT_TIMER,
  ONECLK_EVT_OTA_FINISHED

};

typedef struct{
  uint8_t ev;
  


}oneclk_evt_t;


void otam_oneclick_evt(oneclk_evt_t* pev);

void otam_oneclick_ota(void);

#endif

