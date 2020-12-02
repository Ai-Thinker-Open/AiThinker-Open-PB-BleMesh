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
  Filename:       kscan_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "qdec.h"
#include "OSAL.h"
#include "qdec_demo.h"
#include "log.h"
#include "global_config.h"

extern uint32 *pGlobal_config;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 QDEC_TaskID;   // Task ID for internal task/event processing


/*********************************************************************
 * LOCAL FUNCTIONS
 */
 static void qdec_evt_handler(qdec_Evt_t* evt)
{
	LOG("qdec_evt_handler\n");
	uint16_t degree = evt->count*360/20;
	
	LOG("count: %d, degree: %d\n", evt->count, degree);
}

/*********************************************************************
 * PROFILE CALLBACKS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void QDEC_Init( uint8 task_id )
{
	QDEC_TaskID = task_id;

	qdec_Cfg_t cfg;
	cfg.cha_pin = GPIO_P15;
	cfg.chb_pin = GPIO_P32;
	cfg.qdec_chn = QDEC_CHX;
	cfg.quaMode = QDEC_MODE_1X;
	cfg.intMode = INT_BY_CHANGE;
	cfg.use_inc = FALSE;
	cfg.evt_handler = qdec_evt_handler;
	
	pGlobal_config[MAX_SLEEP_TIME] = 100000000;
	hal_qdec_init(cfg, task_id, QDEC_WAKEUP_TIMEOUT_EVT);
}


uint16 QDEC_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  LOG("QDEC_ProcessEvent: 0x%x\n",events);
 
	if(QDEC_TaskID != task_id)
	{
	
	}
	
  if ( events & QDEC_WAKEUP_TIMEOUT_EVT )
  {
    // Perform kscan timeout task
    hal_qdec_timeout_handler();
	  
    return (events ^ QDEC_WAKEUP_TIMEOUT_EVT);
  }  
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
*********************************************************************/
