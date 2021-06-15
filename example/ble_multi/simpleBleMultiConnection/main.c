
#include "ap_cp.h"
#include "common.h"
#include "uart.h"

#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "timer.h"
#include "ll.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"

#include "ll_sleep.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"

#include "hal_mcu.h"
#include "burn.h"
#include "gpio.h"
#include "clock.h"
#include "pwrmgr.h"
#include "log.h"

#define WFI()   __WFI();

typedef void (*my_function);


int  main(void);
void  ble_main(void);

extern void osalInitTasks(void);
extern int app_main(void);
extern void init_config(void);
//extern void disableSleep(void);
//extern void enableSleep(void);


extern uint32_t get_timer3_count(void);	


/////////////////////////

extern uint32 sleep_flag;
extern uint32 osal_sys_tick;
extern uint32 ll_remain_time;

extern uint32 llWaitingIrq;
extern uint32 ISR_entry_time;

extern uint32 counter_tracking;

volatile uint8 g_clk32K_config;

//extern unsigned char urx_buf[];

//extern volatile unsigned int uart_rx_wIdx;
/////////////////////////////
// ===================== connection context relate definition

#define   BLE_MAX_ALLOW_CONNECTION              5
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_TX        2
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_RX        2

#define   BLE_PKT_VERSION                       BLE_PKT_VERSION_5_1     

#define   BLE_PKT_BUF_SIZE                  (((BLE_PKT_VERSION == BLE_PKT_VERSION_5_1) ? 1 : 0) *  BLE_PKT51_LEN \
	                                        + ((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN \
	                                        + (sizeof(struct ll_pkt_desc) - 2))

 
#define   BLE_MAX_ALLOW_PER_CONNECTION          ( (BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2) \
                                                 +(BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE)   \
                                                 + BLE_PKT_BUF_SIZE )
                                                 
#define   BLE_CONN_BUF_SIZE                 (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)
                                                                                        

__align(4) uint8            g_pConnectionBuffer[BLE_CONN_BUF_SIZE];
llConnState_t               pConnContext[BLE_MAX_ALLOW_CONNECTION];

uint16 iSamplebuf[2];
uint16 qSamplebuf[2];
//uint16 iSamplebuf[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
//uint16 qSamplebuf[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];


////---------------------------------------------------------------------------------------////
// rom_main_var and defined

/////////////////////////
int   int_state;
uint32 hclk_per_us = 16, hclk_per_us_shift = 4;

uint8_t     soc_mode = COMBO;
uint32_t    TM_pin8;
uint32_t    pin24, pin25;
int         hclk,  pclk;
#define     LARGE_HEAP_SIZE  8*1024
uint8       g_largeHeap[LARGE_HEAP_SIZE];


#define DEFAULT_UART_BAUD   115200
///// following code are for AP only //////////////////////////////////////////

static int hf_cnt = 0;
void HardFault_IRQHandler(void)
{
	unsigned int cur_sp = __current_sp();
  if(hf_cnt == 0){
		uint32_t* stk = (uint32_t*)cur_sp;
  	AT_LOG("Hard Fault SP is %x\n",cur_sp);
		for(int i = 0; i< 0x10; i++)
			AT_LOG("0x%x,", stk[i]);
	}
  hf_cnt++;

  while (*(volatile uint32_t *)0x1fff0e00 != 0x12345678) ;
}

static void rf_wakeup_handler(void){
  NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
  NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);
}


static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    
    //============config RF Frequency Offset
    // g_rfPhyFreqOffSet   = RF_PHY_FREQ_FOFF_N40KHZ;
    int32_t freqPpm = 0;
    freqPpm = *(volatile int32_t *)0x11004008;
    if( freqPpm != 0xFFFFFFFF )
      g_rfPhyFreqOffSet = (int8_t)freqPpm;
    else
      g_rfPhyFreqOffSet = RF_PHY_FREQ_FOFF_00KHZ;


    ble_main();
    
    //Quick Boot setting and 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;       //  3'b1xx: 62.5us.  control bits for main digital part reset wait time after power up charge pump. 

    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
//    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw
		*(volatile uint32_t *) 0x4000f014 = 0xB0C3EDB6;//32K Xtal sleep source, dcdc tune, dig_ldo, pin 30 - pin 34 pull donw
    hal_gpio_pull_set(P10,WEAK_PULL_UP);    

    DCDC_CONFIG_SETTING(0x0d);


    NVIC_SetPriority((IRQn_Type)BB_IRQ, IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)CP_TIMER_IRQ, IRQ_PRIO_HIGH);

    hal_pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);
}


static void hal_init(void)
{
    
    hal_system_init(g_system_clk); //system init

    hal_rtc_clock_config(g_clk32K_config);

		hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);

    hal_gpio_init();
    LL_InitConnectContext(  pConnContext, 
                            g_pConnectionBuffer, 
                            BLE_MAX_ALLOW_CONNECTION, 
                            BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                            BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                            BLE_PKT_VERSION);
    LL_EXT_Init_IQ_pBuff(iSamplebuf,qSamplebuf);
}
static void simple_demo_init(void)
{
    if( DEBUG_INFO !=0 )
        LOG_INIT();
	LOG("SYS_CLK %d %d \n",g_clk32K_config,g_system_clk);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

int  main(void)
{
    g_system_clk = SYS_CLK_DLL_48M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
    g_clk32K_config = CLK_32K_XTAL;//,CLK_32K_RCOSC    

    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);

    init_config();

    hal_pwrmgr_init();

    hal_rfphy_init();

    hal_init();

    simple_demo_init();

    app_main();	

}




