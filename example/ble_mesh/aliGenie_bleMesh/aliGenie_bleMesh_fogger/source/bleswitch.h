#ifndef _BLESWITCH_H
#define _BLESWITCH_H
#include "global_config.h"
#include "gpio.h"

#define  BLESWITCH_2ND     1
#define  BLESWITCH_3ND     1

#define GPIO_KEY1        P20
#define GPIO_KEY2        P18
#define GPIO_KEY3        P2
#define GPIO_STUDY_KEY   P3

#define GPIO_CTRL1       P17
#define GPIO_CTRL2       P16
#define GPIO_CTRL3       P14
#define GPIO_CTRL4       P1
//#define GPIO_CTRL5       P31
//#define GPIO_CTRL6       P25
//#define GPIO_CTRL7       P24
//#define GPIO_CTRL8       P23

#define GPIO_LED         P15

#define GPIO_LED_H()    {hal_gpio_pull_set(GPIO_LED, STRONG_PULL_UP);hal_gpio_write(GPIO_LED,1);}
#define GPIO_LED_L()    {hal_gpio_pull_set(GPIO_LED, PULL_DOWN);hal_gpio_write(GPIO_LED,0);}

#define io_lock(io)     {hal_gpio_pull_set(io, WEAK_PULL_UP);hal_gpio_write(io, 1);}
#define io_unlock(io)   {hal_gpio_pull_set(io, PULL_DOWN);hal_gpio_write(io, 0);}

//#define BLE_MESH_CONNECT_START()     {osal_start_reload_timer( KSCAN_TaskID, BLESWITCH_RESET_LED_EVT,400);}
//#define BLE_MESH_CONNECT_SUCCESS()   {osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT);GPIO_LED_L();}

#define BLESWITCH_START_DEVICE_EVT                           0x0001
#define BLESWITCH_50MS_SCAN_EVT                              0x0002
#define BLESWITCH1_SHORT_PRESS_EVT                           0x0004
#define BLESWITCH2_SHORT_PRESS_EVT                           0x0008
#define BLESWITCH3_SHORT_PRESS_EVT                           0x0010
#define BLESWITCH_3S_PRESS_EVT                               0x0020
#define BLESWITCH_10S_PRESS_EVT                              0x0040
#define BLESWITCH_RESET_EVT                                  0x0080
#define BLESWITCH_RESET_LED_EVT                              0x0100
#define BLESWITCH_LED_BLINK_EVT                              0x0200
#define BLESWITCH_LED_BLINK_TIMEOUT_EVT                      0x0400

extern uint8  ble_mesh_connect_state;

extern bool   BLE_CTRL1_STATE,BLE_CTRL2_STATE,BLE_CTRL3_STATE;

extern void KSCAN_Init( uint8 task_id );
extern uint16 KSCAN_ProcessEvent( uint8 task_id, uint16 events );
extern void ble_mesh_connect_success_led(void);
extern void ble_mesh_connect_start_led(void);

#endif