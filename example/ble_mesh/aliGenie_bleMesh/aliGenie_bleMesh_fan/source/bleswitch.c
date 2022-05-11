#include "bleswitch.h"
#include "global_config.h"
#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "model_state_handler_pl.h"
#include "MS_common.h"
#include "pwrmgr.h"
#include "MS_common.h"
#include "MS_access_api.h"
#include "aliGenie_appl_Generic.h"

//#include "aliGenie_appl_Generic.h"

#if APP_DEBUG  //使能LOG

    #define LOG_OUT       printf
#else
    #define LOG_OUT(...)
#endif


#define APP_DELAY_50MS_TIME    50
#define APP_DELAY_1000MS_TIME  1000
#define APP_1S_TIME            20
#define APP_3S_TIME            60
#define APP_10S_TIME           200

static uint8 KSCAN_TaskID;   // Task ID for internal task/event processing
static void bleswitch_init(void);
static void bleswitch_scan_pro(void);


//static uint8  Key_state=0;
static uint16 Key1_cnt_time=0,Key2_cnt_time=0,Key3_cnt_time=0;
uint8  ble_mesh_connect_state=0;

bool   BLE_CTRL1_STATE=0,BLE_CTRL2_STATE=0,BLE_CTRL3_STATE=0;

/*********************************************************************
    PUBLIC FUNCTIONS
*/
void KSCAN_Init( uint8 task_id)
{
    KSCAN_TaskID = task_id;
    osal_set_event( KSCAN_TaskID, BLESWITCH_START_DEVICE_EVT );
//  osal_start_timerEx( KSCAN_TaskID, BLESWITCH_START_DEVICE_EVT,2000);
}

uint16 KSCAN_ProcessEvent( uint8 task_id, uint16 events )
{
    static  uint8 led_cnt=0,led_blink_cnt=0;

    if ( events & BLESWITCH_START_DEVICE_EVT )
    {
        bleswitch_init();
        ble_mesh_connect_start_led();
        osal_start_reload_timer(KSCAN_TaskID,BLESWITCH_50MS_SCAN_EVT,APP_DELAY_50MS_TIME);
        return (events ^ BLESWITCH_START_DEVICE_EVT);
    }

    if ( events & BLESWITCH_50MS_SCAN_EVT ) //按键扫描事件
    {
        bleswitch_scan_pro();
        return (events ^ BLESWITCH_50MS_SCAN_EVT);
    }

    if ( events & BLESWITCH1_SHORT_PRESS_EVT ) //KEY1
    {
        if(BLE_CTRL1_STATE==1)  //反转状态
            app_report_ctrl_onoff(0,0);
        else
            app_report_ctrl_onoff(0,1);

        LOG_OUT("BLESWITCH1_SHORT_PRESS_EVT\r\n");
        return (events ^ BLESWITCH1_SHORT_PRESS_EVT);
    }

    if ( events & BLESWITCH2_SHORT_PRESS_EVT ) //KEY2
    {
        if(BLE_CTRL2_STATE==1)  //反转状态
            app_report_ctrl_onoff(1,0);
        else
            app_report_ctrl_onoff(1,1);

        LOG_OUT("BLESWITCH2_SHORT_PRESS_EVT\r\n");
        return (events ^ BLESWITCH2_SHORT_PRESS_EVT);
    }

    if ( events & BLESWITCH3_SHORT_PRESS_EVT ) //KEY3
    {
        if(BLE_CTRL3_STATE==1)  //反转状态
            app_report_ctrl_onoff(2,0);
        else
            app_report_ctrl_onoff(2,1);

        LOG_OUT("BLESWITCH3_SHORT_PRESS_EVT\r\n");
        return (events ^ BLESWITCH3_SHORT_PRESS_EVT);
    }

    if ( events & BLESWITCH_3S_PRESS_EVT )
    {
        return (events ^ BLESWITCH_3S_PRESS_EVT);
    }

    if ( events & BLESWITCH_10S_PRESS_EVT )
    {
        uint16 req=0;
        req=MS_access_cm_reset();  //清除配网信息
        osal_start_reload_timer( KSCAN_TaskID, BLESWITCH_RESET_LED_EVT,200);
        led_cnt=0;
        LOG_OUT("BLESWITCH_10S_PRESS_EVT===>%d\r\n",req);
        return (events ^ BLESWITCH_10S_PRESS_EVT);
    }

    if ( events & BLESWITCH_RESET_EVT )
    {
        NVIC_SystemReset();
        return (events ^ BLESWITCH_RESET_EVT);
    }

    if ( events & BLESWITCH_RESET_LED_EVT )
    {
        led_cnt++;

        if(led_cnt==1||led_cnt==3||led_cnt==5)
        {
            GPIO_LED_H();
        }
        else if(led_cnt==2||led_cnt==4||led_cnt==6)
        {
            GPIO_LED_L();
        }

        if(led_cnt>=6)
        {
            led_cnt=0;
            osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_RESET_LED_EVT);
            WaitMs(500);
            NVIC_SystemReset();
        }

        return (events ^ BLESWITCH_RESET_LED_EVT);
    }

    if ( events & BLESWITCH_LED_BLINK_EVT )
    {
        if(ble_mesh_connect_state==0)
        {
            led_blink_cnt++;

            if(led_blink_cnt==1)
            {
                GPIO_LED_L();
            }
            else
            {
                led_blink_cnt=0;
                GPIO_LED_H();
            }
        }
        else
        {
            GPIO_LED_L();
            led_blink_cnt=0;
        }

        return (events ^ BLESWITCH_LED_BLINK_EVT);
    }

    if ( events & BLESWITCH_LED_BLINK_TIMEOUT_EVT )
    {
        osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT);
        GPIO_LED_L();
        return (events ^ BLESWITCH_LED_BLINK_TIMEOUT_EVT);
    }

    // Discard unknown events
    return 0;
}

static void bleswitch_init(void)
{
    hal_gpio_pin_init(GPIO_CTRL1,OEN);
    io_unlock(GPIO_CTRL1);
    hal_gpio_pin_init(GPIO_CTRL2,OEN);
    io_unlock(GPIO_CTRL2);
    hal_gpio_pin_init(GPIO_CTRL3,OEN);
    io_unlock(GPIO_CTRL3);
    hal_gpio_pull_set(GPIO_KEY1, WEAK_PULL_UP);
    hal_gpio_pin_init(GPIO_KEY1,IE);
    hal_gpio_pull_set(GPIO_KEY2, WEAK_PULL_UP);
    hal_gpio_pin_init(GPIO_KEY2,IE);
    hal_gpio_pull_set(GPIO_KEY3, WEAK_PULL_UP);
    hal_gpio_pin_init(GPIO_KEY3,IE);
    hal_gpio_pin_init(GPIO_LED,OEN);
    io_unlock(GPIO_LED);
    Key1_cnt_time=0;
}

static void bleswitch_scan_pro(void)
{
    /*************************************KEY1*********************************************/
    if( hal_gpio_read( GPIO_KEY1 )==1 ) //松开
    {
        if(Key1_cnt_time>=2&&Key1_cnt_time<=APP_3S_TIME)
        {
            osal_set_event( KSCAN_TaskID, BLESWITCH1_SHORT_PRESS_EVT );
        }

        Key1_cnt_time=0;
    }
    else if( hal_gpio_read( GPIO_KEY1 )==0&&hal_gpio_read( GPIO_KEY2 )==1&&hal_gpio_read( GPIO_KEY3 )==1) //按下
    {
        Key1_cnt_time++;

        if(Key1_cnt_time>=0xFFF0)
            Key1_cnt_time=0xFFF0;

        if(Key1_cnt_time==APP_10S_TIME)
        {
            osal_set_event( KSCAN_TaskID, BLESWITCH_10S_PRESS_EVT );
            osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT);
        }
    }

    /**********************************************************************************/
    #if  BLESWITCH_2ND

    /*************************************KEY2*********************************************/
    if( hal_gpio_read( GPIO_KEY2 )==1 ) //松开
    {
        if(Key2_cnt_time>=2&&Key2_cnt_time<=APP_3S_TIME)
        {
            osal_set_event( KSCAN_TaskID, BLESWITCH2_SHORT_PRESS_EVT );
        }

        Key2_cnt_time=0;
    }
    else if( hal_gpio_read( GPIO_KEY2 )==0&&hal_gpio_read( GPIO_KEY1 )==1&&hal_gpio_read( GPIO_KEY3 )==1) //按下
    {
        Key2_cnt_time++;

        if(Key2_cnt_time>=0xFFF0)
            Key2_cnt_time=0xFFF0;

        if(Key2_cnt_time==APP_10S_TIME)
        {
            osal_set_event( KSCAN_TaskID, BLESWITCH_10S_PRESS_EVT );
            osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT);
        }
    }

    /**********************************************************************************/
    #endif
    #if BLESWITCH_3ND

    /*************************************KEY3*********************************************/
    if( hal_gpio_read( GPIO_KEY3 )==1 ) //松开
    {
        if(Key3_cnt_time>=2&&Key3_cnt_time<=APP_3S_TIME)
        {
            osal_set_event( KSCAN_TaskID, BLESWITCH3_SHORT_PRESS_EVT );
        }

        Key3_cnt_time=0;
    }
    else if( hal_gpio_read( GPIO_KEY3 )==0&&hal_gpio_read( GPIO_KEY1 )==1&&hal_gpio_read( GPIO_KEY2 )==1) //按下
    {
        Key3_cnt_time++;

        if(Key3_cnt_time>=0xFFF0)
            Key3_cnt_time=0xFFF0;

        if(Key3_cnt_time==APP_10S_TIME)
        {
            osal_set_event( KSCAN_TaskID, BLESWITCH_10S_PRESS_EVT );
            osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT);
        }
    }

    /**********************************************************************************/
    #endif
}

void ble_mesh_connect_start_led(void)
{
    ble_mesh_connect_state=0;
    osal_start_reload_timer( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT,400);
    osal_start_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_TIMEOUT_EVT,(1000*60));
    LOG_OUT("\r\ble_mesh_connect_start_led!!!!!!!\r\n");
}

void ble_mesh_connect_success_led(void)
{
    osal_stop_timerEx( KSCAN_TaskID, BLESWITCH_LED_BLINK_EVT);
    GPIO_LED_L();
    ble_mesh_connect_state=1;
    LOG_OUT("\r\nble_mesh_connect_success_led!!!!!!!\r\n");
}


