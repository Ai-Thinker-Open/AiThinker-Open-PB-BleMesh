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
    Filename:       heartrate.h
    Revised:        $Date $
    Revision:       $Revision $

    Description:    This file contains the wrist demo application and prototypes.


**************************************************************************************************/

#ifndef __WRIST_H
#define __WRIST_H

#include "types.h"


/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/


/** Wrist Task Events**/
//start
#define START_DEVICE_EVT                        0x0001

//timer
#define TIMER_UI_EVT                            0x0002  //for UI timer event
#define TIMER_DT_EVT                            0x0004  //for datetime sync
#define TOUCH_PRESS_EVT                         0x0008  //for touch key event
#define TIMER_HR_EVT                            0x0010  //for heartrate detect
#define TIMER_BATT_EVT                          0x0020  //for battery detect
#define BATT_VALUE_EVT                          0x0040  //event for battery voltage value update
#define BATT_CHARGE_EVT                         0x0080  //event for battery charge status change
#define ACC_DATA_EVT                            0x0100  //event for accelerator data change
#define TIMER_LIGHT_EVT                         0x0200  //for led light timeout
#define TIMER_KSCAN_DEBOUNCE_EVT                0x0400  //for keyscan debounce
#define RESET_ADV_EVT                           0x0800  //for adv reset



/*********************************************************************
    MACROS
*/

/*********************************************************************
    FUNCTIONS
*/

extern uint8 AppWrist_TaskID;

/*
    Task Initialization for the BLE Application
*/
extern void appWristInit( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 appWristProcEvt( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/


#endif /* __WRIST_H */
