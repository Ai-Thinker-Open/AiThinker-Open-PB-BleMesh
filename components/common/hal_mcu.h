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
  Filename:       hal_mcu.h
  Revised:         
  Revision:        

  Description:    Describe the purpose and contents of the file.


 
**************************************************************************************************/

#ifndef _HAL_MCU_H
#define _HAL_MCU_H
 


/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_defs.h"
#include "types.h"
#include <stdint.h>

/* ------------------------------------------------------------------------------------------------
 *                                        Target Defines
 * ------------------------------------------------------------------------------------------------
 */
 
#define MAXMEMHEAP 4096

typedef enum{
  MOD_NONE = 0,           MOD_SOFT_RESET      =0, MOD_CPU   = 0,
  MOD_LOCKUP_RESET_EN =1,
  MOD_WDT_RESET_EN    =2,
  MOD_DMA     =3,
  MOD_AES     =4,
  MOD_TIMER   =5,
  MOD_WDT     =6,
  MOD_COM     =7,
  MOD_UART    =8,
  MOD_I2C0    =9,
  MOD_I2C1    =10,
  MOD_SPI0    =11,
  MOD_SPI1    =12,
  MOD_GPIO    =13,
  MOD_I2S     =14,
  MOD_QDEC    =15,
  MOD_RNG     =16,
  MOD_ADCC    =17,
  MOD_PWM     =18,
  MOD_SPIF    =19,
  MOD_VOC     =20,
  MOD_KSCAN   =31,
  MOD_USR0    =32,
  MOD_USR1    =33,
  MOD_USR2    =34,
  MOD_USR3    =35,
  MOD_USR4    =36,
  MOD_USR5    =37,
  MOD_USR6    =38,
  MOD_USR8    =39,
}MODULE_e;

/* ------------------------------------------------------------------------------------------------
 *                                     Compiler Abstraction
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                        Interrupt Macros
 * ------------------------------------------------------------------------------------------------
 */
//#define HAL_ENABLE_INTERRUPTS()         st( EA = 1; )
//#define HAL_DISABLE_INTERRUPTS()        st( EA = 0; )
//#define HAL_INTERRUPTS_ARE_ENABLED()    (EA)

//typedef unsigned char halIntState_t;
//#define HAL_ENTER_CRITICAL_SECTION(x)   st( x = EA;  HAL_DISABLE_INTERRUPTS(); )
//#define HAL_EXIT_CRITICAL_SECTION(x)    st( EA = x; )
//#define HAL_CRITICAL_STATEMENT(x)       st( halIntState_t _s; HAL_ENTER_CRITICAL_SECTION(_s); x; HAL_EXIT_CRITICAL_SECTION(_s); )

//#define HAL_ISER   *((volatile uint32_t *)(0xe000e100))
//#define HAL_ICER   *((volatile uint32_t *)(0xe000e180))
	
#define HAL_ENABLE_INTERRUPTS()         st( __enable_irq(); )
#define HAL_DISABLE_INTERRUPTS()        st( __disable_irq(); )
#define HAL_INTERRUPTS_ARE_ENABLED()    (HAL_ISER)

typedef uint32_t halIntState_t;


#define HAL_ENTER_CRITICAL_SECTION()  __disable_irq()
#define HAL_EXIT_CRITICAL_SECTION()   __enable_irq()



/**************************************************************************************************
 */
#endif
