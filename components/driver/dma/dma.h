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


/* Peripheral group ----------------------------------------------------------- */
/** @defgroup GPDMA	GPDMA (General Purpose Direct Memory Access)
 * @ingroup LPC177x_8xCMSIS_FwLib_Drivers
 * @{
 */

#ifndef __DMA_H_
#define __DMA_H_

/* Includes ------------------------------------------------------------------- */
#include "ap_cp.h"
#include "common.h"
//#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Public Macros -------------------------------------------------------------- */
/** @defgroup GPDMA_Public_Macros GPDMA Public Macros
 * @{
 */

/** DMA Connection number definitions */
typedef enum{
  DMA_CONN_MEM=0,//        ((0))   /*memory*/
  DMA_CONN_MCI,//				((1UL))			/** SD card */
  DMA_CONN_SSP0_Tx,// 			((2UL)) 		/**< SSP0 Tx */
  DMA_CONN_SSP0_Rx,// 			((3UL)) 		/**< SSP0 Rx */
  DMA_CONN_SSP1_Tx,// 			((4UL)) 		/**< SSP1 Tx */
  DMA_CONN_SSP1_Rx,// 			((5UL)) 		/**< SSP1 Rx */
  DMA_CONN_SSP2_Tx,//			((6UL))			/**< SSP2 Tx */
  DMA_CONN_SSP2_Rx,//			((7UL))			/**< SSP2 Rx */
  DMA_CONN_ADC,// 				((8UL)) 		/**< ADC */
  DMA_CONN_DAC,// 				((9UL)) 		/**< DAC */
  DMA_CONN_UART0_Tx,//			((10UL)) 		/**< UART0 Tx */
  DMA_CONN_UART0_Rx,//			((11UL)) 		/**< UART0 Rx */
  DMA_CONN_UART1_Tx,//			((12UL)) 		/**< UART1 Tx */
  DMA_CONN_UART1_Rx,//			((13UL)) 		/**< UART1 Rx */
  DMA_CONN_UART2_Tx,//			((14UL)) 		/**< UART2 Tx */
  DMA_CONN_UART2_Rx,//			((15UL)) 		/**< UART2 Rx */
  DMA_CONN_MAT0_0,// 			((16UL)) 		/**< MAT0.0 */
  DMA_CONN_MAT0_1,// 			((17UL)) 		/**< MAT0.1 */
  DMA_CONN_MAT1_0,// 			((18UL)) 		/**< MAT1.0 */
  DMA_CONN_MAT1_1,//   		((19UL)) 		/**< MAT1.1 */
  DMA_CONN_MAT2_0,//   		((20UL)) 		/**< MAT2.0 */
  DMA_CONN_MAT2_1,//   		((21UL)) 		/**< MAT2.1 */
  DMA_CONN_I2S_Channel_0,// 	((22UL)) 		/**< I2S channel 0 */
  DMA_CONN_I2S_Channel_1,// 	((23UL)) 		/**< I2S channel 1 */
  DMA_CONN_UART3_Tx,//			((26UL)) 		/**< UART3 Tx */
  DMA_CONN_UART3_Rx,//			((27UL)) 		/**< UART3 Rx */
  DMA_CONN_UART4_Tx,//			((28UL)) 		/**< UART3 Tx */
  DMA_CONN_UART4_Rx,//			((29UL)) 		/**< UART3 Rx */
  DMA_CONN_MAT3_0,// 			((30UL)) 		/**< MAT3.0 */
  DMA_CONN_MAT3_1//   		((31UL)) 		/**< MAT3.1 */
} dma_conn_t;


/** Burst size in Source and Destination definitions */
#define DMA_BSIZE_1 	((0UL)) /**< Burst size = 1 */
#define DMA_BSIZE_4 	((1UL)) /**< Burst size = 4 */
#define DMA_BSIZE_8 	((2UL)) /**< Burst size = 8 */
#define DMA_BSIZE_16 	((3UL)) /**< Burst size = 16 */
#define DMA_BSIZE_32 	((4UL)) /**< Burst size = 32 */
#define DMA_BSIZE_64 	((5UL)) /**< Burst size = 64 */
#define DMA_BSIZE_128 ((6UL)) /**< Burst size = 128 */
#define DMA_BSIZE_256 ((7UL)) /**< Burst size = 256 */

/** Width in Source transfer width and Destination transfer width definitions */
#define DMA_WIDTH_BYTE 		((0UL)) /**< Width = 1 byte */
#define DMA_WIDTH_HALFWORD 	((1UL)) /**< Width = 2 bytes */
#define DMA_WIDTH_WORD 		((2UL)) /**< Width = 4 bytes */

/** DMA Request Select Mode definitions */
#define DMA_REQSEL_UART 	((0UL)) /**< UART TX/RX is selected */
#define DMA_REQSEL_TIMER 	((1UL)) /**< Timer match is selected */

/**
 * @}
 */


/* Private Macros ------------------------------------------------------------- */
/** @defgroup GPDMA_Private_Macros GPDMA Private Macros
 * @{
 */

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/*********************************************************************//**
 * Macro defines for DMA Interrupt Status register
 **********************************************************************/
#define DMA_DMACIntStat_Ch(n)			(((1UL<<n)&0xFF))
#define DMA_DMACIntStat_BITMASK		    ((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Interrupt Terminal Count Request Status register
 **********************************************************************/
#define DMA_DMACIntTCStat_Ch(n)		    (((1UL<<n)&0xFF))
#define DMA_DMACIntTCStat_BITMASK		((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Interrupt Terminal Count Request Clear register
 **********************************************************************/
#define DMA_DMACIntTCClear_Ch(n)		(((1UL<<n)&0xFF))
#define DMA_DMACIntTCClear_BITMASK	((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Interrupt Error Status register
 **********************************************************************/
#define DMA_DMACIntErrStat_Ch(n)		(((1UL<<n)&0xFF))
#define DMA_DMACIntErrStat_BITMASK	((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Interrupt Error Clear register
 **********************************************************************/
#define DMA_DMACIntErrClr_Ch(n)		(((1UL<<n)&0xFF))
#define DMA_DMACIntErrClr_BITMASK		((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Raw Interrupt Terminal Count Status register
 **********************************************************************/
#define DMA_DMACRawIntTCStat_Ch(n)	(((1UL<<n)&0xFF))
#define DMA_DMACRawIntTCStat_BITMASK	((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Raw Error Interrupt Status register
 **********************************************************************/
#define DMA_DMACRawIntErrStat_Ch(n)	(((1UL<<n)&0xFF))
#define DMA_DMACRawIntErrStat_BITMASK	((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Enabled Channel register
 **********************************************************************/
#define DMA_DMACEnbldChns_Ch(n)		(((1UL<<n)&0xFF))
#define DMA_DMACEnbldChns_BITMASK		((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Software Burst Request register
 **********************************************************************/
#define	DMA_DMACSoftBReq_Src(n)		(((1UL<<n)&0xFFFF))
#define DMA_DMACSoftBReq_BITMASK		((0xFFFF))

/*********************************************************************//**
 * Macro defines for DMA Software Single Request register
 **********************************************************************/
#define DMA_DMACSoftSReq_Src(n) 		(((1UL<<n)&0xFFFF))
#define DMA_DMACSoftSReq_BITMASK		((0xFFFF))

/*********************************************************************//**
 * Macro defines for DMA Software Last Burst Request register
 **********************************************************************/
#define DMA_DMACSoftLBReq_Src(n)		(((1UL<<n)&0xFFFF))
#define DMA_DMACSoftLBReq_BITMASK		((0xFFFF))

/*********************************************************************//**
 * Macro defines for DMA Software Last Single Request register
 **********************************************************************/
#define DMA_DMACSoftLSReq_Src(n) 		(((1UL<<n)&0xFFFF))
#define DMA_DMACSoftLSReq_BITMASK		((0xFFFF))

/*********************************************************************//**
 * Macro defines for DMA Configuration register
 **********************************************************************/
#define DMA_DMACConfig_E				((0x01))	 /**< DMA Controller enable*/
#define DMA_DMACConfig_M				((0x02))	 /**< AHB Master endianness configuration*/
#define DMA_DMACConfig_BITMASK		((0x03))

/*********************************************************************//**
 * Macro defines for DMA Synchronization register
 **********************************************************************/
#define DMA_DMACSync_Src(n)			(((1UL<<n)&0xFFFF))
#define DMA_DMACSync_BITMASK			((0xFFFF))

/*********************************************************************//**
 * Macro defines for DMA Request Select register
 **********************************************************************/
#define DMA_DMAReqSel_Input(n)		(((1UL<<(n-8))&0xFF))
#define DMA_DMAReqSel_BITMASK			((0xFF))

/*********************************************************************//**
 * Macro defines for DMA Channel Linked List Item registers
 **********************************************************************/
/** DMA Channel Linked List Item registers bit mask*/
#define DMA_DMACCxLLI_BITMASK 		((0xFFFFFFFC))

/*********************************************************************//**
 * Macro defines for DMA channel control registers
 **********************************************************************/
/** Transfer size*/
#define DMA_DMACCxControl_TransferSize(n) (((n&0xFFF)<<0))
/** Source burst size*/
#define DMA_DMACCxControl_SBSize(n)		(((n&0x07)<<12))
/** Destination burst size*/
#define DMA_DMACCxControl_DBSize(n)		(((n&0x07)<<15))
/** Source transfer width*/
#define DMA_DMACCxControl_SWidth(n)		(((n&0x07)<<18))
/** Destination transfer width*/
#define DMA_DMACCxControl_DWidth(n)		(((n&0x07)<<21))
/** Source increment*/
#define DMA_DMACCxControl_SI				((1UL<<26))
/** Destination increment*/
#define DMA_DMACCxControl_DI				((1UL<<27))
/** Indicates that the access is in user mode or privileged mode*/
#define DMA_DMACCxControl_Prot1			((1UL<<28))
/** Indicates that the access is bufferable or not bufferable*/
#define DMA_DMACCxControl_Prot2			((1UL<<29))
/** Indicates that the access is cacheable or not cacheable*/
#define DMA_DMACCxControl_Prot3			((1UL<<30))
/** Terminal count interrupt enable bit */
#define DMA_DMACCxControl_I				((1UL<<31))
/** DMA channel control registers bit mask */
#define DMA_DMACCxControl_BITMASK			((0xFCFFFFFF))

/*********************************************************************//**
 * Macro defines for DMA Channel Configuration registers
 **********************************************************************/
/** DMA control enable*/
#define DMA_DMACCxConfig_E 					    ((1UL<<0))
/** Source peripheral*/
#define DMA_DMACCxConfig_SrcPeripheral(n) 	    (((n&0x1F)<<1))
/** Destination peripheral*/
#define DMA_DMACCxConfig_DestPeripheral(n) 	    (((n&0x1F)<<6))
/** This value indicates the type of transfer*/
#define DMA_DMACCxConfig_TransferType(n) 		(((n&0x7)<<11))
/** Interrupt error mask*/
#define DMA_DMACCxConfig_IE 					((1UL<<14))
/** Terminal count interrupt mask*/
#define DMA_DMACCxConfig_ITC 					((1UL<<15))
/** Lock*/
#define DMA_DMACCxConfig_L 					    ((1UL<<16))
/** Active*/
#define DMA_DMACCxConfig_A 					    ((1UL<<17))
/** Halt*/
#define DMA_DMACCxConfig_H 					    ((1UL<<18))
/** DMA Channel Configuration registers bit mask */
#define DMA_DMACCxConfig_BITMASK				((0x7FFFF))

/**
 * @}
 */
#define DMA_TRANSFERTYPE_M2M 		((0UL))
/** GPDMA Transfer type definitions: Memory to peripheral - DMA control */
#define DMA_TRANSFERTYPE_M2P 		((1UL))
/** GPDMA Transfer type definitions: Peripheral to memory - DMA control */
#define DMA_TRANSFERTYPE_P2M 		((2UL))
/** Source peripheral to destination peripheral - DMA control */
#define DMA_TRANSFERTYPE_P2P 		((3UL))


/* Public Types --------------------------------------------------------------- */
/** @defgroup GPDMA_Public_Types GPDMA Public Types
 * @{
 */


/**
 * @brief GPDMA Interrupt clear status enumeration
 */
typedef enum{
	DMA_STATCLR_INTTC,	/**< GPDMA Interrupt Terminal Count Request Clear */
	DMA_STATCLR_INTERR	/**< GPDMA Interrupt Error Clear */
}DMA_StateClear_Type;

typedef void (*dma_hdl_t)(uint8 ev);


/**
 * @brief DMA Channel configuration structure type definition
 */
typedef struct {
	uint32_t    transf_size;	/**< Length/Size of transfer */
	
	bool        si;
	uint32_t    src_addr;
	dma_conn_t  src_conn;
	
	bool        di;
	uint32_t    dst_addr;
	dma_conn_t  dst_conn;
	
	uint32_t    lli;

	bool        enable_int;
	dma_hdl_t   evt_handler;
	
} dma_ch_cfg_t;


typedef enum{
	DMA_CH_0 = 0,		
	DMA_CH_1,
	DMA_CH_2,
	DMA_CH_3,
	DMA_CH_4,
	DMA_CH_5,
	DMA_CH_6,
	DMA_CH_7,
	DMA_CH_NUM,
} dma_ch_t;

/**
 * @brief GPDMA Linker List Item structure type definition
 */
typedef struct {
	uint32_t  src_addr;	/**< Source Address */
	uint32_t  dst_addr;	/**< Destination address */
	uint32_t  lli;	    /**< Next LLI address, otherwise set to '0' */
	uint32_t  ctrl;	    /**< GPDMA Control of this LLI */
} dma_lli_t;



int dma_config_channel(dma_ch_t ch, dma_ch_cfg_t* cfg);
int dma_start_channel(dma_ch_t ch, bool enable);
int dma_wait_channel_complete(dma_ch_t ch);
int dma_init(void);



#ifdef __cplusplus
}
#endif

#endif /* __LPC177X_8X_GPDMA_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
