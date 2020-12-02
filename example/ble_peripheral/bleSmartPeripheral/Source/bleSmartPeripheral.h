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
  Filename:       bleSmartPeripheral.h
  Revised:         
  Revision:        

  Description:    This file contains the smart BLE Peripheral sample application
                  definitions and prototypes.

 
**************************************************************************************************/

#ifndef BLESMARTPERIPHERAL_H
#define BLESMARTPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Smart BLE Peripheral Task Events
#define SBP_START_DEVICE_EVT                           0x0001
#define SBP_PERIODIC_EVT                               0x0002
#define SBP_ENTER_NOCONN_EVT                           0x0004
#define SBP_RESET_ADV_EVT                              0x0008

//for batt  
#define TIMER_BATT_EVT                                  0x0010  //for battery detect
#define BATT_VALUE_EVT                                  0x0020  //event for battery voltage value update
#define BATT_CHARGE_EVT                                 0x0040  //event for battery charge status change

//for gpio on/off
#define GPIO_ON_EVT                                   0x0080
#define GPIO_OFF_EVT                                  0x0100

//for button press
#define KEY_SHORT_PRESS_EVT                           0x0200
#define KEY_LONG_PRESS_EVT                            0x0400
#define GPIO_TIMER_KEY_EVT                            0x0800

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

extern uint8 bleSmartPeripheral_TaskID;

/*
 * Task Initialization for the BLE Application
 */
extern void bleSmartPeripheral_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 bleSmartPeripheral_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLESMARTPERIPHERAL_H */
