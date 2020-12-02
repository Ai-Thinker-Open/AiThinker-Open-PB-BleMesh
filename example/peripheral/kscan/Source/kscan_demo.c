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
#include "kscan.h"
#include "OSAL.h"
#include "kscan_demo.h"
#include "log.h"
#include "pwrmgr.h"
#include "global_config.h"


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
static uint8 KSCAN_TaskID;   // Task ID for internal task/event processing



#if 1 //32pin
 KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P15,KEY_ROW_P18}; 
 KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P31};
#else //48 pin
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P25,KEY_ROW_P18}; 
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P20};
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void kscan_evt_handler(kscan_Evt_t* evt)
{
	LOG("kscan_evt_handler\n");
	LOG("num: ");
	LOG("%d",evt->num);
	LOG("\n");
			
	for(uint8_t i=0;i<evt->num;i++){
		
		LOG("index: ");
		LOG("%d",i);
		LOG(",row: ");
		LOG("%d",evt->keys[i].row);
		LOG(",col: ");
		LOG("%d",evt->keys[i].col);
		LOG(",type: ");
		LOG("%s",evt->keys[i].type == KEY_PRESSED ? "pressed":"released");
		LOG("\n");
	}	
}

/*********************************************************************
 * PROFILE CALLBACKS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void KSCAN_Init( uint8 task_id )
{
	KSCAN_TaskID = task_id;

	kscan_Cfg_t cfg;
	cfg.ghost_key_state = NOT_IGNORE_GHOST_KEY;
	cfg.key_rows = rows;
	cfg.key_cols = cols;
	cfg.interval = 50;
	cfg.evt_handler = kscan_evt_handler;
	
	pGlobal_config[MAX_SLEEP_TIME] = 100000000;
	hal_kscan_init(cfg, task_id, KSCAN_WAKEUP_TIMEOUT_EVT);
}


uint16 KSCAN_ProcessEvent( uint8 task_id, uint16 events )
{
  
  //VOID task_id; // OSAL required parameter that isn't used in this function
	
	if(KSCAN_TaskID != task_id)
	{
		
	}
	
  LOG("KSCAN_ProcessEvent: 0x%x\n",events);
  
  if ( events & KSCAN_WAKEUP_TIMEOUT_EVT )
  {
    // Perform kscan timeout task
    hal_kscan_timeout_handler();
    
    return (events ^ KSCAN_WAKEUP_TIMEOUT_EVT);
  }  
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
*********************************************************************/
