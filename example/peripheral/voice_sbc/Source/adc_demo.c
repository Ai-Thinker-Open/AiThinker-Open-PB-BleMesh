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
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "adc.h"
#include "OSAL.h"
#include "adc_demo.h"
#include "log.h"





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
static uint8 adcDemo_TaskID;   // Task ID for internal task/event processing



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void adc_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void adcMeasureTask( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void adc_Init( uint8 task_id )
{
  adcDemo_TaskID = task_id;

  // Setup a delayed profile startup
  //osal_start_timerEx( adcDemo_TaskID, adcMeasureTask_EVT,100);
  //osal_start_timerEx( adcDemo_TaskID, 0x20, 2500 );
	adcMeasureTask();
}


uint16 adc_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  LOG("adc_ProcessEvent: 0x%x\n",events);
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( adcDemo_TaskID )) != NULL )
    {
      adc_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
 
  if ( events & 0x20 )
  {
    // Perform periodic heart rate task
    LOG("20\n");
    osal_start_timerEx( adcDemo_TaskID, 0x20, 2000);
    
    return (events ^ 0x20);
  }  

  if ( events & adcMeasureTask_EVT )
  {
    // Perform periodic heart rate task
    LOG("adcMeasureTask_EVT\n");
    adcMeasureTask();
    
    return (events ^ adcMeasureTask_EVT);
  }  
  
  // Discard unknown events
  return 0;
}


static void adc_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
}


static void adc_evt(adc_Evt_t* pev)
{
  if(pev->type == HAL_ADC_EVT_DATA){
    float value = hal_adc_value_cal(pev->ch,pev->data, pev->size, TRUE, FALSE);
    
  	LOG("batt_measure_evt %d\n",(int)(value*1000));
  }
}


static void adcMeasureTask( void )
{
  int ret;
  adc_Cfg_t cfg = {
      .is_continue_mode = FALSE,
      .is_differential_mode = FALSE,
      .is_high_resolution = TRUE,
      .is_auto_mode = FALSE,
    };
      
  ret = hal_adc_config_channel(ADC_CH1N, cfg, adc_evt);
  if(ret)
    return;

  hal_adc_start();
 // Restart timer
  osal_start_timerEx( adcDemo_TaskID, adcMeasureTask_EVT, 2000 );
}



/*********************************************************************
*********************************************************************/
