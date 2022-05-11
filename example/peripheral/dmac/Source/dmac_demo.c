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

/**************************************************************************************************
    Filename:       dmac_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "string.h"
#include "OSAL.h"
#include "dmac_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "dma.h"
#include "flash.h"

enum
{
    MEM2MEM,
    MEM2FLASH,
    FLASH2MEM,
    MEM2UART0,
    TRAN_COUNT
} ;



uint8_t g_dma_src_buffer_u8[0x400*2];
uint8_t g_dma_dst_buffer_u8[0x400*2];


static uint8 dma_TaskID,dma_demolst = MEM2MEM;
static uint8 dma_testcase;

static DMA_CH_CFG_t cfg;

void dma_cb(DMA_CH_t ch)
{
    LOG("\ndma done!!!\n");

    if(dma_testcase == FLASH2MEM)
    {
        for(int i=0; i<cfg.transf_size; i++)
        {
            if(g_dma_src_buffer_u8[i] != g_dma_dst_buffer_u8[i])
            {
                LOG("g_dma_src_buffer_u8[%d]:0x%x != g_dma_dst_buffer_u8[%d]:0x%x",i,g_dma_src_buffer_u8[i],i,g_dma_dst_buffer_u8[i]);
                LOG("mememory to memory error,check it\n");

                while(1);
            }
        }

        LOG("memory to flash test:ok\n");
        LOG("flash to memory test:ok\n");
    }
}


void DMA_Demo_Init(uint8 task_id )
{
    hal_gpio_init();
    hal_dma_init();
    dma_TaskID = task_id;
    HAL_DMA_t ch_cfg;
    ch_cfg.dma_channel = DMA_CH_0;
    ch_cfg.evt_handler = dma_cb;
    hal_dma_init_channel(ch_cfg);
    LOG("dma demo start...\n");
    osal_start_reload_timer(dma_TaskID, DMA_DEMO_CYCLE_TIMER, 2000);
}

void demo_dma_test(uint8 list)
{
    uint8_t mode;
    uint32_t i;
    uint8_t retval;
    mode = list % TRAN_COUNT;
    dma_testcase = mode;
    hal_dma_stop_channel(DMA_CH_0); //for dma demo test

    switch(mode)
    {
    case MEM2MEM:
        cfg.transf_size = 0x100;
        cfg.sinc = DMA_INC_INC;
        cfg.src_tr_width = DMA_WIDTH_WORD;
        cfg.src_msize = DMA_BSIZE_256;
        cfg.src_addr = (uint32_t)g_dma_src_buffer_u8;
        cfg.dinc = DMA_INC_INC;
        cfg.dst_tr_width = DMA_WIDTH_WORD;
        cfg.dst_msize = DMA_BSIZE_256;
        cfg.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
        cfg.enable_int = false;

        for(int i=0; i<cfg.transf_size; i++)
        {
            g_dma_src_buffer_u8[i] = i;
            g_dma_dst_buffer_u8[i] = 0x00;
        }

        break;

    case MEM2FLASH:
        for(i = 0; i< 0x100; i++)
            g_dma_src_buffer_u8[i] = (uint8_t)(0xff-i);

        spif_erase_sector(0x11004000);
        cfg.transf_size = 0x100;
        cfg.sinc = DMA_INC_INC;
        cfg.src_tr_width = DMA_WIDTH_WORD;
        cfg.src_msize = DMA_BSIZE_1;
        cfg.src_addr = (uint32_t)g_dma_src_buffer_u8;
        cfg.dinc = DMA_INC_INC;
        cfg.dst_tr_width = DMA_WIDTH_WORD;
        cfg.dst_msize = DMA_BSIZE_1;
        cfg.dst_addr = (uint32_t)0x11004000;
        cfg.enable_int = true;
        break;

    case FLASH2MEM:
        memset(g_dma_dst_buffer_u8,0,0x400*2);
        cfg.transf_size = 0x100;
        cfg.sinc = DMA_INC_INC;
        cfg.src_tr_width = DMA_WIDTH_WORD;
        cfg.src_msize = DMA_BSIZE_1;
        cfg.src_addr = (uint32_t)0x11004000;
        cfg.dinc = DMA_INC_INC;
        cfg.dst_tr_width = DMA_WIDTH_WORD;
        cfg.dst_msize = DMA_BSIZE_1;
        cfg.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
        cfg.enable_int = true;
        break;

    case MEM2UART0:
        for(i = 0; i< 100; i++)
            g_dma_src_buffer_u8[i] = (uint8_t)((0x7e)-i);

        cfg.transf_size = 0x20;
        cfg.sinc = DMA_INC_INC;
        cfg.src_tr_width = DMA_WIDTH_BYTE;
        cfg.src_msize = DMA_BSIZE_1;
        cfg.src_addr = (uint32_t)g_dma_src_buffer_u8;
        cfg.dinc = DMA_INC_NCHG;
        cfg.dst_tr_width = DMA_WIDTH_BYTE;
        cfg.dst_msize = DMA_BSIZE_1;
        cfg.dst_addr = (uint32_t)0x40004000;
        cfg.enable_int = true;
        break;

    default:
        break;
    }

    retval = hal_dma_config_channel(DMA_CH_0,&cfg);

    if(retval == PPlus_SUCCESS)
    {
        hal_dma_start_channel(DMA_CH_0);

        if(cfg.enable_int == false)
        {
            hal_dma_wait_channel_complete(DMA_CH_0);

            for(int i=0; i<cfg.transf_size; i++)
            {
                if(g_dma_src_buffer_u8[i] != g_dma_dst_buffer_u8[i])
                {
                    LOG("g_dma_src_buffer_u8[%d]:0x%x != g_dma_dst_buffer_u8[%d]:0x%x",i,g_dma_src_buffer_u8[i],i,g_dma_dst_buffer_u8[i]);
                    LOG("mememory to memory error,check it\n");

                    while(1);
                }
            }

            LOG("memory to memory test:ok\n");
        }
    }
    else
        LOG("[DMAC]Config channel Failed,Error code is %d\n",retval);
}


uint16 DMA_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != dma_TaskID)
    {
        return 0;
    }

    if( events & DMA_DEMO_CYCLE_TIMER)
    {
        demo_dma_test(dma_demolst++);

        if( dma_demolst >= 3*TRAN_COUNT)
        {
            LOG("dma test finish:ok\n");
            LOG("stop\n");

            while(1);
        }

        return (events ^ DMA_DEMO_CYCLE_TIMER);
    }

    return 0;
}

/*********************************************************************
*********************************************************************/
