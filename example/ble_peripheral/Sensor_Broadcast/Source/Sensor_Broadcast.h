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
  Filename:       sensor_broadcast.h
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

#ifndef __BRIDGE_IBEACON__H__
#define __BRIDGE_IBEACON__H__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
    
#define RAWADV_FIRST_START_REG                      0x3ffc

/*********************************************************************
 * CONSTANTS
 */
//#define RAWADV_SYS_OFF                              0
//#define RAWADV_SYS_SLEEP                            1
//#define RAW_PM_MODE                                 RAWADV_SYS_SLEEP

    
// Task Events
#define START_DEVICE_EVT                               0x0001
#define SBP_RESET_ADV_EVT                              0x0002

    
#define SENSOR_START_ADV                              0x0008
#define TIMER_KEY_EVT                                 0x0020
#define KEY_SHORT_PRESS_EVT                           0x0040
#define KEY_LONG_PRESS_EVT                            0x0080
#define KEY_PRESS_EVT                                 0x0100
#define KEY_RELEASE_EVT                               0x0200
#define BATT_VALUE_EVT                                0x0400
#define TIMER_BATT_EVT                                0x0800


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void Sensor_Broadcast_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 Sensor_Broadcast_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
