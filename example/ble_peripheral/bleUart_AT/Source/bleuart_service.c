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
    Filename:       simpleGATTprofile_ota.c
    Revised:
    Revision:

    Description:    This file contains the Simple GATT profile sample GATT service
                  profile for use with the BLE sample application.


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "uart.h"
#include "log.h"

#include "bleuart_service.h"
#include "bleuart.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

#define SERVAPP_NUM_ATTR_SUPPORTED        7

#define RAWPASS_VALUE_HANDLE          2
#define AT_SERVICE_UUID_DEF           0xffe0
#define AT_PT_CHAR_UUID_DEF           0xffe1

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
uint8 bleuart_ServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(AT_SERVICE_UUID_DEF), HI_UINT16(AT_SERVICE_UUID_DEF)
};

// Characteristic PT uuid
uint8 bleuart_PTCharUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(AT_PT_CHAR_UUID_DEF), HI_UINT16(AT_PT_CHAR_UUID_DEF)
};


/*********************************************************************
    EXTERNAL VARIABLES
*/

bleuart_Evt_t evt = {0};

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

static bleuart_ProfileChangeCB_t bleuart_AppCBs = NULL;

/*********************************************************************
    Profile Attributes - variables
*/

// Profile Service attribute ATT_BT_UUID_SIZE
static gattAttrType_t bleuart_Service = { ATT_BT_UUID_SIZE, bleuart_ServiceUUID };

// Profile Characteristic 1 Properties
static uint8 bleuart_PTCharProps = GATT_PROP_NOTIFY| GATT_PROP_READ| GATT_PROP_WRITE_NO_RSP;

// Characteristic 1 Value
static uint8 bleuart_PTCharValue[RAWPASS_RX_BUFF_SIZE];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

// Simple Profile Characteristic 2 User Description
static gattCharCfg_t bleuart_TxCCCD;

/*********************************************************************
    Profile Attributes - Table
*/

static gattAttribute_t bleuart_ProfileAttrTbl[] =
{
    // Simple Profile Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)& bleuart_Service            /* pValue */
    },

    // Characteristic 1 Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &bleuart_PTCharProps
    },

    // Characteristic Value 1
    {
        { ATT_BT_UUID_SIZE, bleuart_PTCharUUID },
        GATT_PERMIT_WRITE,
        0,
        &bleuart_PTCharValue[0]
    },

    // Characteristic 2 User Description
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ|GATT_PERMIT_WRITE,
        0,
        (uint8*)& bleuart_TxCCCD
    },

};



/*********************************************************************
    LOCAL FUNCTIONS
*/
static uint8 bleuart_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                 uint8* pValue, uint8* pLen, uint16 offset, uint8 maxLen );
static bStatus_t bleuart_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                      uint8* pValue, uint8 len, uint16 offset );

static void bleuart_HandleConnStatusCB ( uint16 connHandle, uint8 changeType );


/*********************************************************************
    PROFILE CALLBACKS
*/
// Simple Profile Service Callbacks
CONST gattServiceCBs_t bleuart_ProfileCBs =
{
    (pfnGATTReadAttrCB_t)bleuart_ReadAttrCB,  // Read callback function pointer
    (pfnGATTWriteAttrCB_t)bleuart_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      bleuart_AddService

    @brief   Initializes the Simple Profile service by registering
            GATT attributes with the GATT server.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.

    @return  Success or Failure
*/
bStatus_t bleuart_AddService( bleuart_ProfileChangeCB_t cb)
{
    uint8 status = SUCCESS;
    // Initialize Client Characteristic Configuration attributes
    //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileChar4Config );
    // Register with Link DB to receive link status change callback
    VOID linkDB_Register( bleuart_HandleConnStatusCB  );
    bleuart_TxCCCD.connHandle = INVALID_CONNHANDLE;
    bleuart_TxCCCD.value = 0;
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( bleuart_ProfileAttrTbl,
                                          GATT_NUM_ATTRS( bleuart_ProfileAttrTbl ),
                                          &bleuart_ProfileCBs );

    if(status!=SUCCESS)
        LOG("Add rawpass service failed!\n");

    bleuart_AppCBs = cb;
    return ( status );
}



/*********************************************************************
    @fn          bleuart_ReadAttrCB

    @brief       Read an attribute.

    @param       connHandle - connection message was received on
    @param       pAttr - pointer to attribute
    @param       pValue - pointer to data to be read
    @param       pLen - length of data to be read
    @param       offset - offset of the first octet to be read
    @param       maxLen - maximum length of data to be read

    @return      Success or Failure
*/
static uint8 bleuart_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                 uint8* pValue, uint8* pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;

    // If attribute permissions require authorization to read, return error
    if ( gattPermitAuthorRead( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    if ( osal_memcmp(pAttr->type.uuid, clientCharCfgUUID, 2))
    {
        // case:GATT_CLIENT_CHAR_CFG_UUID(0x2902)
        *pLen = 2;
        osal_memcpy(pValue, pAttr->pValue, 2);
    }
    else if(osal_memcmp(pAttr->type.uuid, bleuart_PTCharUUID, 2))
    {
        //*pLen = 1;
        //pValue[0] = bleuart_PTCharValue[0]; // TBD. ??
        //pValue[0] = '1';
        *pLen = RAWPASS_RX_BUFF_SIZE;

        for(int i=0; i<RAWPASS_RX_BUFF_SIZE; i++)
            pValue[i] = bleuart_PTCharValue[i];
    }
    else
    {
        return ( ATT_ERR_ATTR_NOT_FOUND );
    }

    return ( status );
}

/*********************************************************************
    @fn      simpleProfile_WriteAttrCB

    @brief   Validate attribute data prior to a write operation

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/

static bStatus_t bleuart_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                      uint8* pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;

    //AT_LOG("len: %d\n", len);
    // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    if ( osal_memcmp(pAttr->type.uuid, clientCharCfgUUID, 2) )
    {
        // case:GATT_CLIENT_CHAR_CFG_UUID(0x2902)
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );

        if ( status == SUCCESS && bleuart_AppCBs)
        {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
            LOG("CCCD set: [%d]\n", charCfg);
            evt.ev = (charCfg == GATT_CFG_NO_OPERATION)?bleuart_EVT_TX_NOTI_DISABLED:bleuart_EVT_TX_NOTI_ENABLED;
            bleuart_AppCBs(&evt);
        }
    }
    else if(osal_memcmp(pAttr->type.uuid, bleuart_PTCharUUID, 2) && \
            (pAttr->handle == bleuart_ProfileAttrTbl[RAWPASS_VALUE_HANDLE].handle))
    {
        if(bleuart_AppCBs)
        {
            evt.ev = bleuart_EVT_BLE_DATA_RECIEVED;
            evt.param = (uint16_t)len;
            evt.data = pValue;
            bleuart_AppCBs(&evt);
        }
    }
    else
    {
        return ( ATT_ERR_ATTR_NOT_FOUND );
    }

    return ( status );
}

/*********************************************************************
    @fn          simpleProfile_HandleConnStatusCB

    @brief       Simple Profile link status change handler function.

    @param       connHandle - connection handle
    @param       changeType - type of change

    @return      none
*/
static void bleuart_HandleConnStatusCB ( uint16 connHandle, uint8 changeType )
{
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
                ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
                  ( !linkDB_Up( connHandle ) ) ) )
        {
            bleuart_TxCCCD.value = 0;
            LOG("clear client configuration\n");
        }
    }
}

uint8 bleuart_NotifyIsReady(void)
{
    return (bleuart_TxCCCD.value == GATT_CLIENT_CFG_NOTIFY);
}

void set_Bleuart_Notify(void)
{
    bleuart_TxCCCD.value |= GATT_CLIENT_CFG_NOTIFY;
}

void clear_Bleuart_Notify(void)
{
    bleuart_TxCCCD.value = 0;
}

/*********************************************************************
    @fn          BloodPressure_IMeasNotify

    @brief       Send a notification containing a bloodPressure
                measurement.

    @param       connHandle - connection handle
    @param       pNoti - pointer to notification structure

    @return      Success or Failure
*/
bStatus_t bleuart_Notify( uint16 connHandle, attHandleValueNoti_t* pNoti, uint8 taskId )
{
    uint16 value = bleuart_TxCCCD.value;

    // If notifications enabled
    if ( value & GATT_CLIENT_CFG_NOTIFY )
    {
        // Set the handle
        pNoti->handle = bleuart_ProfileAttrTbl[RAWPASS_VALUE_HANDLE].handle;
        //AT_LOG("N_1\n");
        // Send the Indication
        return GATT_Notification( connHandle, pNoti, FALSE);
    }

    return bleIncorrectMode;
}

/*********************************************************************
    @fn      update_Bleuart_ProfileAttrTbl

    @brief   update 2-byte service uuid or pass-through uuid based on
            pre-defined Bleuart_ProfileAttrTbl. These 2 uuids may be
            updated by 'at+suuid'/'at+tuuid' AT cmds later.

    @param   suuid  - 2-byte service uuid.
            ptuuid - 2-byte pass-through uuid.

    @return  NA
*/
void update_Bleuart_ProfileAttrTbl( uint16_t suuid, uint16_t ptuuid)
{
    //if service is already added, we hold/skip this update.
    if(bleuart_TxCCCD.connHandle != INVALID_CONNHANDLE)
    {
        bleuart_ServiceUUID[0] = LO_UINT16(suuid);
        bleuart_ServiceUUID[1] = HI_UINT16(suuid);
        bleuart_PTCharUUID[0]  = LO_UINT16(ptuuid);
        bleuart_PTCharUUID[1]  = HI_UINT16(ptuuid);
    }
}

/*********************************************************************
*********************************************************************/

