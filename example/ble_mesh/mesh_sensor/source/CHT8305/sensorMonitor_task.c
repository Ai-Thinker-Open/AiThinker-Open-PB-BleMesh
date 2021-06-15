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
  Filename:       sensorMonitor_task.c
  Revised:        $20191217 $
  Revision:       $0.1 $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "sensorMonitor_task.h"
#include "log.h"
#include "error.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "gpio.h"
#include "sensor_server.h"

#include "cht8305_sensor.h"
#include "hal_mcu.h"
#include "pwrmgr.h"




#define CONSOLE_OUT(...)    printf(__VA_ARGS__)

#define CHECK_DIFF(a,b)   (a)>(b)?((a)-(b)):((b)-(a))

#define SENSOR_MOD  MOD_USR5


uint8 SensorMonitor_TaskID;
uint16_t cht8305_temp,cht8305_humi;
uint16_t cht8305_temp_pre,cht8305_humi_pre;

extern API_RESULT MS_Sensor_server_model_state_publish(
	/* IN */ MS_ACCESS_SENSOR_MODEL_STATE_PARAMS	   * current_state_params);


static void SensorMonitorTask_ProcessOSALMsg( osal_event_hdr_t *pMsg );






/*
 * Task Initialization for the BLE Application
 */
void SensorMonitor_task_Init( uint8 task_id )
{
  CONSOLE_OUT("SensorMonitor task init \n\r");
  SensorMonitor_TaskID=task_id;
  hal_pwrmgr_register(SENSOR_MOD, NULL, NULL);
  hal_pwrmgr_lock(SENSOR_MOD);

  CHT8305_init();
	CHT8305_I2CInit();

  if(CHT8305_is_connected()==TRUE)
  	CONSOLE_OUT("cht8305 is connect ok\n\r");
#if 0
  else
  {
   CONSOLE_OUT("cht8305 is connect fail\n\r");
  }
#endif 
	
	CHT8305_I2CDeinit();
    hal_pwrmgr_unlock(SENSOR_MOD);


  
 
  osal_start_timerEx(SensorMonitor_TaskID, SENSOR_MEASURE_EVT, 5000);

}





uint16 SensorMonitor_task_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
 // LOG("voice_task_ProcessEvent: 0x%x\n\r",events);

//  static uint16 wait_time=0;
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( SensorMonitor_TaskID )) != NULL )
    {
      SensorMonitorTask_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

   
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if( events & SENSOR_MEASURE_EVT )
  {
	 cht8305_humi_pre=cht8305_humi;
	 cht8305_temp_pre=cht8305_temp;

	 hal_pwrmgr_lock(SENSOR_MOD);
	CHT8305_I2CInit();
	CHT8305_get_temp_humi(&cht8305_temp,&cht8305_humi);
    CONSOLE_OUT("humi=%d,temp=%d\n\r",cht8305_humi,cht8305_temp);
	CHT8305_I2CDeinit();
    hal_pwrmgr_unlock(SENSOR_MOD);
    //update x senconds
	osal_memcpy(humity_raw_value_x0, humity_raw_value_x1, sizeof(humity_raw_value_x1));
	osal_memcpy(humity_raw_value_x1, humity_raw_value_x2, sizeof(humity_raw_value_x2));
	osal_memcpy(humity_raw_value_x2, humity_raw_value_x3, sizeof(humity_raw_value_x3));
	osal_memcpy(humity_raw_value_x3, humity_raw_value_x4, sizeof(humity_raw_value_x4));

    uint16 senconds=humity_raw_value_x4[0]|(humity_raw_value_x4[1]<<8);
	senconds+=1;
    humity_raw_value_x4[0]=(uint8)senconds;
	humity_raw_value_x4[1]=(uint8)(senconds>>8);

	//update column width 
	osal_memcpy(humity_column_w0, humity_column_w1, sizeof(humity_column_w1));
	osal_memcpy(humity_column_w1, humity_column_w2, sizeof(humity_column_w2));
	osal_memcpy(humity_column_w2, humity_column_w3, sizeof(humity_column_w3));
	osal_memcpy(humity_column_w3, humity_column_w4, sizeof(humity_column_w4));

    uint16 width=humity_column_w4[0]|(humity_column_w4[1]<<8);
	width=1;
    humity_column_w4[0]=(uint8)width;
	humity_column_w4[1]=(uint8)(width>>8);

	//update y value
	osal_memcpy(humity_raw_value_y0, humity_raw_value_y1, sizeof(humity_raw_value_y1));
	osal_memcpy(humity_raw_value_y1, humity_raw_value_y2, sizeof(humity_raw_value_y2));
	osal_memcpy(humity_raw_value_y2, humity_raw_value_y3, sizeof(humity_raw_value_y3));
	osal_memcpy(humity_raw_value_y3, humity_raw_value_y4, sizeof(humity_raw_value_y4));

    humity_raw_value_y4[0]=(uint8)cht8305_humi;
	humity_raw_value_y4[1]=(uint8)(cht8305_humi>>8);

      if(CHECK_DIFF(cht8305_humi,cht8305_humi_pre)>=10)
      {
       CONSOLE_OUT("delta=%d\n\r",CHECK_DIFF(cht8305_humi,cht8305_humi_pre));
       osal_set_event(SensorMonitor_TaskID, SENSOR_UPDATE_EVT);//update right now
       
	  }

     
	
	osal_start_timerEx(SensorMonitor_TaskID, SENSOR_MEASURE_EVT, 5000);
   return (events ^ SENSOR_MEASURE_EVT); 
  }

  if( events & SENSOR_UPDATE_EVT )
  {
    CONSOLE_OUT("update now\n\r");
	//format B:
	uint8 data_len=0x02;
	uint8 humi_update_buf[5];
	humi_update_buf[0]      = ((data_len <<1) | 0x01);
	humi_update_buf[1]  = (uint8)HUMIDITY_PID;
	humi_update_buf[2]  = (uint8)(HUMIDITY_PID>>8);
	osal_memcpy(&humi_update_buf[3], &cht8305_humi, 2);



	MS_ACCESS_SENSOR_MODEL_STATE_PARAMS humi_params;
	humi_params.len=5;
	humi_params.state_type =MS_STATE_SENSOR_DATA_T;
	humi_params.state = humi_update_buf;

    MS_NET_ADDR addr;
    API_RESULT  retval = MS_access_cm_get_primary_unicast_address(&addr);
	

    if (API_SUCCESS == retval)
    {
        if (MS_NET_ADDR_UNASSIGNED != addr)
        {
			MS_Sensor_server_model_state_publish(&humi_params);
        }
    }

	


    osal_start_timerEx(SensorMonitor_TaskID, SENSOR_UPDATE_EVT, 15000);
   return (events ^ SENSOR_UPDATE_EVT); 
  }
  
	
	return 0;

}




static void SensorMonitorTask_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
   switch ( pMsg->event )
  {
      
    default:
      break;
  }
}






