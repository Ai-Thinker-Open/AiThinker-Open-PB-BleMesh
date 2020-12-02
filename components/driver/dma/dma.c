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


#include <string.h>
#include "ap_cp.h"
#include "clock.h"
#include "pwrmgr.h"
#include "error.h"
#include "log.h"

#include "dma.h"

typedef struct{
  bool init_flg;
  
}dma_ctx_t;


const static AP_DMACH_TypeDef* spDMACh[8] = {
		AP_DMACH0,	// GPDMA Channel 0
		AP_DMACH1,	// GPDMA Channel 1
		AP_DMACH2,	// GPDMA Channel 2
		AP_DMACH3,	// GPDMA Channel 3
		AP_DMACH4,	// GPDMA Channel 4
		AP_DMACH5,	// GPDMA Channel 5
		AP_DMACH6,	// GPDMA Channel 6
		AP_DMACH7,	// GPDMA Channel 7
};


dma_ctx_t s_dma_ctx = {
  .init_flg = FALSE,
};

/**
 * @brief Optimized Peripheral Source and Destination burst size
 */
const uint8_t DMA_LUTPerBurst[] = {
		DMA_BSIZE_1,				// Memory
		DMA_BSIZE_32,				// SD Card
		DMA_BSIZE_4,				// SSP0 Tx
		DMA_BSIZE_4,				// SSP0 Rx
		DMA_BSIZE_4,				// SSP1 Tx
		DMA_BSIZE_4,				// SSP1 Rx
		DMA_BSIZE_4,				// SSP2 Tx
		DMA_BSIZE_4,				// SSP2 Rx
		DMA_BSIZE_4,				// ADC
		DMA_BSIZE_1,				// DAC
		DMA_BSIZE_1,				// UART0 Tx
		DMA_BSIZE_1,				// UART0 Rx
		DMA_BSIZE_1,				// UART1 Tx
		DMA_BSIZE_1,				// UART1 Rx
		DMA_BSIZE_1,				// UART2 Tx
		DMA_BSIZE_1,				// UART2 Rx
		DMA_BSIZE_1,				// MAT0.0
		DMA_BSIZE_1,				// MAT0.1
		DMA_BSIZE_1,				// MAT1.0
		DMA_BSIZE_1,				// MAT1.1
		DMA_BSIZE_1,				// MAT2.0
		DMA_BSIZE_1,				// MAT2.1
		DMA_BSIZE_32, 			// I2S channel 0
		DMA_BSIZE_32, 			// I2S channel 1
		0,							// Reserved
		0, 							// Reserved
		DMA_BSIZE_1,				// UART3 Tx
		DMA_BSIZE_1,				// UART3 Rx
		DMA_BSIZE_1,				// UART4 Tx
		DMA_BSIZE_1,				// UART4 Rx
		DMA_BSIZE_1,				// MAT3.0
		DMA_BSIZE_1,				// MAT3.1
};

/**
 * @brief Optimized Peripheral Source and Destination transfer width
 */
const uint8_t DMA_LUTPerWid[] = {
		DMA_WIDTH_WORD,				// memory
		DMA_WIDTH_WORD,				// SD Card
		DMA_WIDTH_BYTE,				// SSP0 Tx
		DMA_WIDTH_BYTE,				// SSP0 Rx
		DMA_WIDTH_BYTE,				// SSP1 Tx
		DMA_WIDTH_BYTE,				// SSP1 Rx
		DMA_WIDTH_BYTE,				// SSP2 Tx
		DMA_WIDTH_BYTE,				// SSP2 Rx
		DMA_WIDTH_WORD,				// ADC
		DMA_WIDTH_BYTE,				// DAC
		DMA_WIDTH_BYTE,				// UART0 Tx
		DMA_WIDTH_BYTE,				// UART0 Rx
		DMA_WIDTH_BYTE,				// UART1 Tx
		DMA_WIDTH_BYTE,				// UART1 Rx
		DMA_WIDTH_BYTE,				// UART2 Tx
		DMA_WIDTH_BYTE,				// UART2 Rx
		DMA_WIDTH_WORD,				// MAT0.0
		DMA_WIDTH_WORD,				// MAT0.1
		DMA_WIDTH_WORD,				// MAT1.0
		DMA_WIDTH_WORD,				// MAT1.1
		DMA_WIDTH_WORD,				// MAT2.0
		DMA_WIDTH_WORD,				// MAT2.1
		DMA_WIDTH_WORD, 				// I2S channel 0
		DMA_WIDTH_WORD, 				// I2S channel 1
		0,								// Reserved
		0, 								// Reserved
		DMA_WIDTH_BYTE,				// UART3 Tx
		DMA_WIDTH_BYTE,				// UART3 Rx
		DMA_WIDTH_BYTE,				// UART4 Tx
		DMA_WIDTH_BYTE,				// UART4 Rx
		DMA_WIDTH_WORD,				// MAT3.0
		DMA_WIDTH_WORD,				// MAT3.1
};



int dma_config_channel(dma_ch_t ch, dma_ch_cfg_t* cfg)
{
	AP_DMACH_TypeDef *pdma;
	uint32_t cctrl = 0;
	uint32_t transf_type = DMA_TRANSFERTYPE_M2M;
	uint32_t src_bsize = DMA_BSIZE_1;
	uint32_t dst_bsize = DMA_BSIZE_1;
	uint32_t src_wide = DMA_WIDTH_WORD;
	uint32_t dst_wide = DMA_WIDTH_WORD;

  if(!s_dma_ctx.init_flg)
    return PPlus_ERR_NOT_REGISTED;
	clk_gate_enable(MOD_DMA);

	
	if (AP_DMA->EnbldChns & (DMA_DMACEnbldChns_Ch(ch))) {
		// This channel is enabled, return ERROR, need to release this channel first
		return PPlus_ERR_BUSY;
	}
	// Get Channel
	pdma = (AP_DMACH_TypeDef *) spDMACh[ch];

	// Reset the Interrupt status
	AP_DMA->IntTCClear = DMA_DMACIntTCClear_Ch(ch);
	AP_DMA->IntErrClr = DMA_DMACIntErrClr_Ch(ch);

	// Clear DMA configure
	pdma->CControl = 0x00;
	pdma->CConfig = 0x00;

	/* Assign Linker List Item value */
	pdma->CLLI = cfg->lli;

	if(cfg->src_conn && cfg->dst_conn){
	  transf_type = DMA_TRANSFERTYPE_P2P;
	  src_bsize = DMA_LUTPerBurst[cfg->src_conn];
	  dst_bsize = DMA_LUTPerBurst[cfg->dst_conn];
  	src_wide = DMA_LUTPerWid[cfg->src_conn];
  	dst_wide = DMA_LUTPerWid[cfg->dst_conn];
	}
	else if(cfg->src_conn){
	  transf_type = DMA_TRANSFERTYPE_P2M;
	  src_bsize = DMA_LUTPerBurst[cfg->src_conn];
  	src_wide = DMA_LUTPerWid[cfg->src_conn];
	}
	else if(cfg->dst_conn){
	  transf_type = DMA_TRANSFERTYPE_M2P;
	  dst_bsize = DMA_LUTPerBurst[cfg->dst_conn];
  	dst_wide = DMA_LUTPerWid[cfg->dst_conn];
	}

  pdma->CSrcAddr = cfg->src_addr;
	pdma->CDestAddr = cfg->dst_addr;
	
	//pdma->CControl =  0x8c480005;//DMA_DMACCxControl_TransferSize(cfg->transf_size)| 
	cctrl =  DMA_DMACCxControl_TransferSize(cfg->transf_size)| \
           DMA_DMACCxControl_SBSize(src_bsize)| \
           DMA_DMACCxControl_DBSize(dst_bsize)| \
           DMA_DMACCxControl_SWidth(src_wide)| \
           DMA_DMACCxControl_DWidth(dst_wide);
  if(cfg->di)
    cctrl |= DMA_DMACCxControl_DI;
  if(cfg->si)
    cctrl |= DMA_DMACCxControl_SI;
		

	if(cfg->enable_int)
		cctrl |= DMA_DMACCxControl_I;

  pdma->CControl = cctrl;

	/* Enable DMA channels, little endian */
	AP_DMA->Config = DMA_DMACConfig_E;
	while (!(AP_DMA->Config & DMA_DMACConfig_E));

	// Configure DMA Channel, enable Error Counter and Terminate counter
	//pdma->CConfig = DMA_DMACCxConfig_IE | DMA_DMACCxConfig_ITC|DMA_DMACCxConfig_TransferType(transf_type)
	pdma->CConfig = DMA_DMACCxConfig_TransferType(transf_type)| \
                  DMA_DMACCxConfig_SrcPeripheral(cfg->src_conn)| \
                  DMA_DMACCxConfig_DestPeripheral(cfg->dst_conn);	


	if(cfg->enable_int)
  	pdma->CConfig |= DMA_DMACCxConfig_IE | DMA_DMACCxConfig_ITC;

  return PPlus_SUCCESS;
}



int dma_start_channel(dma_ch_t ch, bool enable)
{
	AP_DMACH_TypeDef *pDMAch;

  if(!s_dma_ctx.init_flg)
    return PPlus_ERR_NOT_REGISTED;
	
	// Get Channel pointer
	pDMAch = (AP_DMACH_TypeDef *) spDMACh[ch];
  if(enable){
		pDMAch->CConfig |= DMA_DMACCxConfig_E;
	} else {
		pDMAch->CConfig &= ~DMA_DMACCxConfig_E;
	}
	return PPlus_SUCCESS;
}

int dma_wait_channel_complete(dma_ch_t ch)
{
  if(!s_dma_ctx.init_flg)
    return PPlus_ERR_NOT_REGISTED;
  return PPlus_SUCCESS;
}

int dma_init(void)
{
  s_dma_ctx.init_flg = TRUE;
  return PPlus_SUCCESS;
}







