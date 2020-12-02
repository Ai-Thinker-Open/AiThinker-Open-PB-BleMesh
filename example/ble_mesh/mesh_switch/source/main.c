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

                                                                                                                                                                                                                
#include "ap_cp.h"
#include "common.h"
#include "hal_mcu.h"

#include "uart.h"
#include "flash.h"
#include "gpio.h"
#include "timer.h"
#include "clock.h"
#include "pwrmgr.h"

#include "ll.h"
#include "rf_phy_driver.h"
#include "global_config.h"

#include "ll_debug.h"
#include "log.h"
#include "EM_os.h"

#define DEFAULT_UART_BAUD   115200

#define     LARGE_HEAP_SIZE  12*1024//12*1024
uint8       g_largeHeap[LARGE_HEAP_SIZE];


/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
void  ble_main(void);

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern void init_config(void);
extern int app_main(void);

extern void disableSleep(void);
extern void uart_hw_config(void);
extern void bleMesh_uart_init(void);
extern void hal_rom_code_ini(void);
/*********************************************************************
 * GLOBAL VARIABLES
 */
//uint32 *pGlobal_config = NULL;
volatile uint8 g_clk32K_config;
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32_t  __initial_sp;


static int hf_cnt = 0;
void HardFault_IRQHandler(void)
{
	unsigned int cur_sp = __current_sp();
  if(hf_cnt == 0){
		uint32_t* stk = (uint32_t*)cur_sp;
  	printf("Hard Fault SP is %x\r\n",cur_sp);
		for(int i = 0; i< 0x20; i++)
			printf("0x%x,", stk[i]);
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
    g_rfPhyTxPower  = RF_PHY_TX_POWER_EXTRA_MAX ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_N60KHZ;

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

 	hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);

    hal_gpio_init();
				
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////

int  main(void)  
{
    //config the txPower
    g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
    g_clk32K_config = CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC
    
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);    
  
    init_config();
 
    hal_pwrmgr_init();
    
    hal_rfphy_init();

    LOG_INIT();
    
    hal_init();	
  
    bleMesh_uart_init();    
    hal_pwrmgr_register(MOD_USR1, app_sleep_process, app_wakeup_process);
    app_main();	
}

void app_sleep_process(void)
{
//    printf("SLEEP");
//   gpio_write(14, 0);
//    gpio_write(15, 0);
}


void app_wakeup_process(void)
{
    
}

#define OSALMEM_BIGBLK_IDX 157
// ===========================================================
// ptr: the header of osal heap
//uint32  osal_memory_statics(void *ptr)
uint32  osal_memory_statics(void)    
{
    osalMemHdr_t *header, *current;
    void *ptr;
    uint32  sum_alloc = 0;
    uint32  sum_free = 0;
    uint32  max_block = 0;
//    halIntState_t intState;    

    ptr = (void *)g_largeHeap;

    header = (osalMemHdr_t *)ptr;
    current = (osalMemHdr_t *)ptr;
    
//    HAL_ENTER_CRITICAL_SECTION1( intState );  // Hold off interrupts.

    do
    {     
        if ((uint32)ptr > (uint32)header + 4096)
        {
            printf("==========error: memory audit failed===============\r\n");
            break;
        }      

        // seek to the last block, return
        if ( current->val == 0 )       /// val = 0, so len = 0
        {
            break;
        }
        
        if (current->hdr.inUse)
            sum_alloc += current->hdr.len;
        else
        {            
            sum_free += current->hdr.len;
            if (current->hdr.len > max_block && (void *)(&current->hdr) > (void *)(header + OSALMEM_BIGBLK_IDX))
                max_block = current->hdr.len;
        }
    
        current = (osalMemHdr_t *)((uint8 *)current + current->hdr.len);    
    } while (1);

//    HAL_EXIT_CRITICAL_SECTION1( intState );  // Re-enable interrupts. 
    
//    printf("sum_alloc = %d, sum_free = %d, max_free_block = %d\r\n", sum_alloc, sum_free, max_block);
    printf("sum_alloc = %d, max_free_block = %d ", sum_alloc, max_block);
  
    return sum_alloc;
}


