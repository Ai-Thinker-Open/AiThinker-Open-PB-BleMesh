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
  Filename:       OSAL_bleMesh.c
  Revised:
  Revision:

  Description:    This file contains function that allows user setup tasks



**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#if (APP_CFG == 0)
//#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
#include "gapgattserver.h"

/* GATT */
#include "gatt.h"
#include "gattservapp.h"

/* Timer */
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbTimer.h"
#endif

/* Application */
#include "bleMesh.h"

extern uint16 baseTaskID;
#define EVENT_ID( timerId )            ( 0x0001 << ( ( timerId ) % NUM_CBTIMERS_PER_TASK ) )

// Find out task id using timer id
#define TASK_ID( timerId )             ( ( ( timerId ) / NUM_CBTIMERS_PER_TASK ) + baseTaskID )

// Find out bank task id using task id
#define BANK_TASK_ID( taskId )         ( ( baseTaskID - ( taskId ) ) * NUM_CBTIMERS )

/*********************************************************************
 * CONSTANTS
 */
// Number of callback timers supported per task (limited by the number of OSAL event timers)
#define NUM_CBTIMERS_PER_TASK          15

// Total number of callback timers
#define NUM_CBTIMERS                   ( OSAL_CBTIMER_NUM_TASKS * NUM_CBTIMERS_PER_TASK )

uint16 osal_CbTimerProcessEvent_new( uint8 taskId, uint16 events );
typedef struct
{
  pfnCbTimer_t pfnCbTimer; // callback function to be called when timer expires
  uint8 *pData;            // data to be passed in to callback function
} cbTimer_t;

extern cbTimer_t cbTimers[];

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
  HCI_ProcessEvent,                                                 // task 1
#if defined ( OSAL_CBTIMER_NUM_TASKS )
    OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent_new ),
#endif
    L2CAP_ProcessEvent,                                               // task 2
  GAP_ProcessEvent,                                                 // task 3
  SM_ProcessEvent,                                                  // task 4
  GATT_ProcessEvent,                                                // task 5
  GATTServApp_ProcessEvent,                                         // task 6



  bleMesh_ProcessEvent                                              // task 8

};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  /* LL Task */
  LL_Init( taskID++ );

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif
    
  /* L2CAP Task */
  L2CAP_Init( taskID++ );

  /* GAP Task */
  GAP_Init( taskID++ );

  /* SM Task */
  SM_Init( taskID++ );

  /* GATT Task */
  GATT_Init( taskID++ );

  /* Profiles */
  GATTServApp_Init( taskID++ );



  /* Application */
  bleMesh_Init( taskID );
}

/*********************************************************************
 * @fn          osal_CbTimerProcessEvent
 *
 * @brief       Callback Timer task event processing function.
 *
 * @param       taskId - task ID.
 * @param       events - events.
 *
 * @return      events not processed
 */
uint16 osal_CbTimerProcessEvent_new( uint8 taskId, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    // Process OSAL messages

    // return unprocessed events
    return ( events ^ SYS_EVENT_MSG );
  }

  if ( events )
  {
    uint8 i;
    uint16 event;

    // Process event timers
    for ( i = 0; i < NUM_CBTIMERS_PER_TASK; i++ )
    {
      if ( ( events >> i ) & 0x0001 )
      {
        cbTimer_t *pTimer = &cbTimers[BANK_TASK_ID( taskId )+i];

        // Found the first event
        event =  0x0001 << i;

          if (NULL == pTimer->pfnCbTimer)     // if the timer has been deleted, the callback function pointer is NULL
          {
          }
          else
             // Timer expired, call the registered callback function
             pTimer->pfnCbTimer( pTimer->pData );

        // Mark entry as free
        pTimer->pfnCbTimer = NULL;
        
        // Null out data pointer
        pTimer->pData = NULL;
      
        // We only process one event at a time
        break;
      }
    }

    // return unprocessed events
    return ( events ^ event );
  }

  // If reach here, the events are unknown
  // Discard or make more handlers
  return 0;
}
#endif
/*********************************************************************
*********************************************************************/
