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
    Filename:       simpleGATTprofile.h
    Revised:
    Revision:

    Description:    This file contains the Simple GATT profile definitions and
                  prototypes.

 **************************************************************************************************/

#ifndef _BLE_UART_SERVICE_H
#define _BLE_UART_SERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "att.h"

/*********************************************************************
    CONSTANTS
*/

// Profile Parameters
#define PROFILE_RAWPASS_CHAR_RX                   0  // RW uint8 - Profile Characteristic 1 value 
#define PROFILE_RAWPASS_CHAR_TX                   1  // RW uint8 - Profile Characteristic 2 value

// Simple Keys Profile Services bit fields
#define PROFILE_RAWPASS_SERVICE               0x00000001


#define RAWPASS_RX_BUFF_SIZE                  6

enum
{
    bleuart_EVT_TX_NOTI_DISABLED = 1,
    bleuart_EVT_TX_NOTI_ENABLED,
    bleuart_EVT_BLE_DATA_RECIEVED,
};


typedef struct
{
    uint8_t   ev;
    uint16_t  param;
    void*     data;
} bleuart_Evt_t;

typedef void (*bleuart_ProfileChangeCB_t)(bleuart_Evt_t* pev);



/*********************************************************************
    API FUNCTIONS
*/


/*
    bleuart_AddService- Initializes the raw pass GATT Profile service by registering
            GATT attributes with the GATT server.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.
*/

extern bStatus_t bleuart_AddService( bleuart_ProfileChangeCB_t cb);




extern uint8 bleuart_NotifyIsReady(void);

extern bStatus_t bleuart_Notify( uint16 connHandle, attHandleValueNoti_t* pNoti, uint8 taskId );
extern void set_Bleuart_Notify(void);
extern void clear_Bleuart_Notify(void);
extern void update_Bleuart_ProfileAttrTbl( uint16_t suuid, uint16_t ptuuid);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _BLE_UART_SERVICE_H */

