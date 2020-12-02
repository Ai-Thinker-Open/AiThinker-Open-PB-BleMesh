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



#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "ll.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "clock.h"
#include "uart.h"
#include "log.h"
#include "flash.h"
#include "ap_timer.h"
#include "OSAL_Memory.h"

#define DEFAULT_UART_BAUD   115200

#define     LARGE_HEAP_SIZE  6*1024
uint8       g_largeHeap[LARGE_HEAP_SIZE];

extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);
extern int         pclk;

volatile uint8 g_current_advType;
volatile uint8 g_clk32K_config;
/////////////////////////


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
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;

    hal_rom_code_ini();
    
    //Quick Boot setting and 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;       //  3'b1xx: 62.5us.  control bits for main digital part reset wait time after power up charge pump. 

    //========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw

    //========= UART RX Pull up
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

 	hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1 | RET_SRAM2);

    hal_gpio_init();
				
}



static void simple_rfphy_dtm_ext_demo(void)
{

    uint8_t txPower = RF_PHY_TX_POWER_0DBM;
    uint8_t rfChnIdx = 10;
    int8_t  rfFoff = RF_PHY_FREQ_FOFF_00KHZ;
    uint32_t testTimeUs = 100000;
    uint8_t pktType = 0;//0 :prbs9, 1:11110000, 2:10101010
    uint8_t pktLength =31;
    uint32_t numTxPkt = 100;

    uint8_t testMod=0;
    
    uint32_t accCode = RF_PHY_DTM_SYNC_WORD;//

    uint8_t rxEstRssi;
    uint8_t rxEstCarrSens;
    uint16_t rxPktNum;
    uint16_t rxEstFoff;

    rf_phy_dtm_ext_acc_code_set(accCode);

    pwroff_cfg_t pwrOffCfg[1];
    pwrOffCfg[0].pin = P24;        //wakeup pin config
    pwrOffCfg[0].type=NEGEDGE;     //wakeup pin edge

    LOG_INIT();
        
    while(1)
    {
        
        testMod = ((hal_gpio_read(P23)<<2 )|hal_gpio_read(P25)<<1)|(hal_gpio_read(P24));
    
        if(testMod==0)
        {
            LOG("single tone\n") ;       
            rf_phy_dtm_ext_tx_singleTone(txPower,rfChnIdx,rfFoff,testTimeUs);
            WaitMs(3*1000);
        }
        else if(testMod==1)
        {
            LOG("modulation\n") ;       
            rf_phy_dtm_ext_tx_modulation(txPower,rfChnIdx,rfFoff,pktType,testTimeUs);
            WaitMs(3*1000);
        }
        else if(testMod==2)
        {
            LOG("tx burst\n") ;   

            rf_phy_dtm_ext_tx_mod_burst(txPower,rfChnIdx,rfFoff,pktType,pktLength,numTxPkt,20*1000/*pktIntv*/);


        }
        else if(testMod==3)
        {
            LOG("system off\n") ;       

                    
            WaitUs(100);
            hal_pwrmgr_poweroff(pwrOffCfg,1);
        }
        else if(testMod == 4)
        {
            LOG("rx burst\n");
            uint16_t rxTimeOut = 2000;//ms
            rf_phy_dtm_ext_rx_demod_burst(rfChnIdx,rfFoff,pktLength,rxTimeOut,5000/*rxWindow*/,
                                          &rxEstFoff,&rxEstRssi,&rxEstCarrSens,&rxPktNum);

            LOG("%x %x %x [%d]",rxEstFoff,rxEstRssi,rxEstCarrSens,rxPktNum);
        }

        
        
    }
    
}

static void simple_demo_init(void)
{
    if(hal_gpio_read(P23))
        simple_rfphy_dtm_ext_demo();
    
    if(hal_gpio_read(P25))
    {
        //enter RF DTM no return
        rf_phy_direct_test();
    }

    if(hal_gpio_read(P24))
    {
        g_current_advType = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;
        pGlobal_config[ADV_CHANNEL_INTERVAL] = 600;//6250;
    }
    else
    {
        g_current_advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;
        pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
    }

    //only for demo case, 
    hal_gpio_pull_set(P24,FLOATING);
    hal_gpio_pull_set(P25,FLOATING);

    LOG_INIT();
    LOG_CHIP_ID();
    LOG_CHIP_MADDR();
    LOG_PATCH_DATA_TIME();
    LOG("SYS_CLK %d %d \n",g_clk32K_config,g_system_clk);

}

#if (SYS_CLK_TEST_UCDS)
void sys_clock_test_config(void)   
{
    uint32 system_clk_cfg = flash_read_ucds(SYS_CLK_TEST_UCDS);
    if(system_clk_cfg==0xffffffff)
    {
        //config the system clk first;
        g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
        g_clk32K_config = CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
    }
    else
    {
        g_system_clk    = 0xff&system_clk_cfg;
        g_clk32K_config = (0xff00&system_clk_cfg)>>8;
    }
}
#endif

int  main(void)  
{     
    g_system_clk = SYS_CLK_DLL_48M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
    g_clk32K_config = CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
    
#if( SYS_CLK_TEST_UCDS)
    sys_clock_test_config();
#endif

    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
    
    init_config();

    hal_pwrmgr_init();

    hal_rfphy_init();
    
    hal_init();

    simple_demo_init();

    app_main();	
    
}


///////////////////////////////////  end  ///////////////////////////////////////

