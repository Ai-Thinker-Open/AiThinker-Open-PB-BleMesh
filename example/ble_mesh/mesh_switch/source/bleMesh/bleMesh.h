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
    Filename:       bleMesh.h
    Revised:
    Revision:

    Description:    This file contains the bleMesh sample application
                  definitions and prototypes.


**************************************************************************************************/

#ifndef BLEMESH_H
#define BLEMESH_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/

extern uint8 bleMesh_TaskID;   // Task ID for internal task/event processing



// BLE Mesh Task Events

/*********************************************************************
    MACROS
*/
#define BLEMESH_START_DEVICE_EVT                            0x0001
#define BLEMESH_UART_RX_EVT                                 0x0002
#define BLEMESH_GAP_SCANENABLED                             0x0004
#define BLEMESH_ECDH_PROCESS                                0x0008
#define BLEMESH_KEY_PRESS_PRO_EVT                           0x0010
#define BLEMESH_LIGHT_PRCESS_EVT                            0x0020
#define BLEMESH_KEY_PRESS_EVT                               0x0040
#define BLEMESH_KEY_LONG_PRESS_EVT                          0x0080
#define BLEMESH_HAL_KEY_MATRIX_EVT                          0x0100
#define BLEMESH_PDU_TX_OVERRUN                              0x0200

#define BLEMESH_PROV_COMP_EVT                               0x0400
#define BLEMESH_APPL_IDLE_EVT                               0x0800

#define BLEMESH_GAP_TERMINATE                               0x1000
#define BLEMESH_GAP_MSG_EVT                                 0x2000

/*********************************************************************
    FUNCTIONS
*/

/*
    Task Initialization for the BLE Application
*/
extern void bleMesh_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 bleMesh_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BLEMESH_H */
