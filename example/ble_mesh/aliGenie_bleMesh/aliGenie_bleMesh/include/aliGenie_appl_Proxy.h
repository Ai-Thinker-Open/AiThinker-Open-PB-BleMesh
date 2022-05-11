#ifndef _ALIGENIE_APPL_H_
#define _ALIGENIE_APPL_H_

#include "aliGenie_appl.h"


void UI_proxy_start_adv(MS_SUBNET_HANDLE subnet_handle, UCHAR proxy_adv_mode);
void UI_proxy_callback(NETIF_HANDLE* handle,UCHAR p_evt,UCHAR* data_param,UINT16 data_len);
#ifdef MS_PROXY_SUPPORT
    void UI_register_proxy(void);
#endif

#endif //_ALIGENIE_APPL_H_
