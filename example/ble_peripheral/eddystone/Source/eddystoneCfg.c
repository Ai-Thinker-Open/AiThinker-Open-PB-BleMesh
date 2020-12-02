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
  Filename:       eddystoneCfg.c

  Description:    This file contains the Eddystone Configuration service 
                  profile for use with the BLE sample application.


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "types.h"

#include "ll.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "eddystoneCfg.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// URL Configuration Service UUID
static const uint8_t urlCfgSvcUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(URLCFG_SVC_UUID)
};


// URI Data UUID
static const uint8_t edsCfgCharURIDataUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_URI_DATA_UUID)
};
   
// Advertised TX Power Level UUID
static const uint8_t edsCfgCharAdvTXPwrLvlsUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_ADV_TX_PWR_LVLS_UUID)
};

// TX Power Mode UUID
static const uint8_t edsCfgCharTXPowerModeUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_TX_POWER_MODE_UUID)
};
   
// Beacon Period UUID
static const uint8_t edsCfgCharBeaconPeriodUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_BEACON_PERIOD_UUID)
};

// Reset UUID
static const uint8_t edsCfgCharResetUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_RESET_UUID)
};

// UID UUID
static const uint8_t edsCfgCharUIDUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_UID_UUID)
};

// Eddystone adv frame type combination UUID
static const uint8_t edsCfgCharFrameCombUUID[ATT_UUID_SIZE] =
{
  EDDYSTONE_BASE_UUID_128(EDSCFG_FRAME_COMBINE_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

uint8    URIDataLen;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static edsCfgSvcCBs_t *edsCfgSvc_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// URL Configuration Service attribute
static CONST gattAttrType_t urlCfgService = { ATT_UUID_SIZE, urlCfgSvcUUID };

// URI Data Characteristic Properties
static uint8 edsCfgCharURIDataProps = GATT_PROP_READ | GATT_PROP_WRITE;
// URI Data Characteristic Value
static uint8 edsCfgCharURIData[19];     // should be initialized
// URI Data Characteristic User Description
static uint8 edsCfgCharURIDataUserDesc[] = "URI Data";

// Advertised TX Power Levels Characteristic Properties
static uint8 edsCfgCharAdvTXPwrLvlsProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Advertised TX Power Levels Characteristic Value
static int8 edsCfgCharAdvTXPwrLvls[4] = {-20, -10, -2, 0};
// Advertised TX Power Levels Characteristic User Description
static uint8 edsCfgCharAdvTXPwrLvlsUserDesc[] = "Adv TX Pwr Lvls";

// TX Power Mode Characteristic Properties
static uint8 edsCfgCharTXPowerModeProps = GATT_PROP_READ | GATT_PROP_WRITE;
// TX Power Mode Characteristic Value
static uint8 edsCfgCharTXPowerMode = EDSCFG_CHAR_TX_POWER_MODE_DEFAULT;
// TX Power Mode Characteristic User Description
static uint8 edsCfgCharTXPowerModeUserDesc[] = "TX Power Mode";

// Beacon Period Characteristic Properties
static uint8 edsCfgCharBeaconPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Beacon Period Characteristic Value
static uint16 edsCfgCharBeaconPeriod = EDSCFG_CHAR_BEACON_PERIOD_DEFAULT;
// Beacon Period Characteristic User Description
static uint8 edsCfgCharBeaconPeriodUserDesc[] = "Beacon Period";

// Reset Characteristic Properties
static uint8 edsCfgCharResetProps = GATT_PROP_WRITE;
// Reset Characteristic Value
static uint8 edsCfgCharReset = 0;
// Reset Characteristic User Description
static uint8 edsCfgCharResetUserDesc[] = "Reset configration";

// UID Characteristic Properties
static uint8 edsCfgCharUIDProps = GATT_PROP_READ | GATT_PROP_WRITE;
// UID Characteristic Value
static uint8 edsCfgCharUID[EDSCFG_CHAR_UID_DATA_LEN];
// UID Characteristic User Description
static uint8 edsCfgCharUIDUserDesc[] = "UID";

// Eddystone Frame combinition Characteristic Properties 
static uint8 edsCfgCharFrameCombProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Eddystone Frame combinition Characteristic Value
static uint8 edsCfgCharFrameComb = 2;     // 0 - UID, 1 - URL, 2 - UID + URL
// Eddystone Frame combinition Characteristic User Description
static uint8 edsCfgCharFrameCombUserDesc[] = "EddyStone Frame combination";    

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t edsCfgSvcAttrTbl[] = 
{
    // Simple Profile Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&urlCfgService                   /* pValue */
    },
    
    //  -----  URI  ------------
    // URI Data Characteristic Declaration
    { 
          { ATT_BT_UUID_SIZE, characterUUID },
          GATT_PERMIT_READ,
          0,
          &edsCfgCharURIDataProps
    },

    // URI Data Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharURIDataUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        edsCfgCharURIData
    },

    // URI Data Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharURIDataUserDesc
    },      
      
    // -------------------------  UID    ------------------------
    // UID Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &edsCfgCharUIDProps
    },
    
    // UID Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharUIDUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        edsCfgCharUID
    },
    
    // UID Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharUIDUserDesc
    },
    
    // --------------  Frame type selection --------------------------
    // Eddystone frame type Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &edsCfgCharFrameCombProps
    },
    
    // Eddystone frame type Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharFrameCombUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &edsCfgCharFrameComb
    },
    
    // Eddystone frame type Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharFrameCombUserDesc
    },             
      
    // ---------  TX power levels -------------------
    // Advertised TX Power Levels Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &edsCfgCharAdvTXPwrLvlsProps
    },
    
    // Advertised TX Power Levels Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharAdvTXPwrLvlsUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) edsCfgCharAdvTXPwrLvls
    },
    
    // Advertised TX Power Levels Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharAdvTXPwrLvlsUserDesc
    },           
    
    // -------------- Tx power setting  ---------------------
    // TX Power Mode Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &edsCfgCharTXPowerModeProps
    },
    
    // TX Power Mode Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharTXPowerModeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &edsCfgCharTXPowerMode
    },
    
    // TX Power Mode Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharTXPowerModeUserDesc
    },           
    
    // -------------- beacon period -----------------
    // Beacon Period Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &edsCfgCharBeaconPeriodProps
    },
    
    // Beacon Period Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharBeaconPeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *) &edsCfgCharBeaconPeriod
    },
    
    // Beacon Period Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharBeaconPeriodUserDesc
    },           
    
    //  ---------------- reset configuration ----------------
    // Reset Characteristic Declaration
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &edsCfgCharResetProps
    },
    
    // Reset Characteristic Value
    { 
        { ATT_UUID_SIZE, edsCfgCharResetUUID },
        GATT_PERMIT_WRITE,
        0,
        &edsCfgCharReset
    },
    
    // Reset Characteristic User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        edsCfgCharResetUserDesc
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t edsCfgSvc_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr, 
                                          uint8_t *pValue, uint8_t *pLen,
                                          uint16_t offset, uint8_t maxLen);
static bStatus_t edsCfgSvc_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint8_t len,
                                           uint16_t offset);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Eddystone Configuration Service Callbacks
CONST gattServiceCBs_t edsCfgSvcCBs =
{
    edsCfgSvc_ReadAttrCB,      // Read callback function pointer
    edsCfgSvc_WriteAttrCB,     // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      edsCfgSvc_AddService
 *
 * @brief   Initializes the Eddystone Configuration service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t edsCfgSvc_AddService( void )
{
  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( edsCfgSvcAttrTbl, 
                                      GATT_NUM_ATTRS( edsCfgSvcAttrTbl ),
                                      &edsCfgSvcCBs );
}

/*********************************************************************
 * @fn      edsCfgSvc_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t edsCfgSvc_RegisterAppCBs( edsCfgSvcCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    edsCfgSvc_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      edsCfgSvc_SetParameter
 *
 * @brief   Set a Eddystone Configuration Service parameter.
 *
 * @param   param - Characteristic ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t edsCfgSvc_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case EDSCFG_URI_DATA:
      if ( len <= EDSCFG_CHAR_URI_DATA_LEN ) 
      {
        VOID memcpy( edsCfgCharURIData, value, len );
        URIDataLen = len;
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EDSCFG_ADV_TX_PWR_LVLS:
      if ( len == sizeof(edsCfgCharAdvTXPwrLvls) )
      {
        memcpy(edsCfgCharAdvTXPwrLvls, value, sizeof(edsCfgCharAdvTXPwrLvls));
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EDSCFG_TX_POWER_MODE:
      if ( len == sizeof(uint8) &&
          *((uint8*) value) < sizeof(edsCfgCharAdvTXPwrLvls) ) 
      {
        edsCfgCharTXPowerMode = *((uint8*) value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case EDSCFG_BEACON_PERIOD:
      if ( len == sizeof(uint16) ) 
      {
        edsCfgCharBeaconPeriod = *((uint16*) value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

      case EDSCFG_UID_DATA:
        if ( len == EDSCFG_CHAR_UID_DATA_LEN)
        {
          VOID memcpy( edsCfgCharUID, value, len );
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;     
      
      case EDSCFG_FRAME_COMB_DATA:
        if ( len == sizeof( uint8 ) ) 
        {
          edsCfgCharFrameComb = *((uint8*) value);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;   
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t edsCfgSvc_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case EDSCFG_URI_DATA:
      VOID memcpy(value, edsCfgCharURIData, URIDataLen);
      break;

    case EDSCFG_URI_DATA_LEN:
      *((uint8*) value) = URIDataLen;
      break;   

    case EDSCFG_ADV_TX_PWR_LVLS:
      VOID memcpy(value, edsCfgCharAdvTXPwrLvls, 4);
      break;  

    case EDSCFG_TX_POWER_MODE:
      *((uint8*) value) = edsCfgCharTXPowerMode;
      break;

    case EDSCFG_BEACON_PERIOD:
      *((uint16*) value) = edsCfgCharBeaconPeriod;
      break;      
    
    case EDSCFG_UID_DATA:
      VOID memcpy(value, edsCfgCharUID, EDSCFG_CHAR_UID_DATA_LEN);
      break;
    
    case EDSCFG_FRAME_COMB_DATA:
      *((uint8*) value) = edsCfgCharFrameComb;
      break;    
    
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          edsCfgSvc_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t edsCfgSvc_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint8_t *pLen,
                                          uint16_t offset, uint8_t maxLen)
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    // 128-bit UUID
    if (!memcmp(pAttr->type.uuid, edsCfgCharTXPowerModeUUID, ATT_UUID_SIZE) ||
        !memcmp(pAttr->type.uuid, edsCfgCharFrameCombUUID, ATT_UUID_SIZE) )
    {
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
    }
    else if (!memcmp(pAttr->type.uuid, edsCfgCharURIDataUUID, ATT_UUID_SIZE))
    {
      *pLen = URIDataLen;
      memcpy(pValue, pAttr->pValue, URIDataLen);
    }
    else if (!memcmp(pAttr->type.uuid, edsCfgCharAdvTXPwrLvlsUUID, ATT_UUID_SIZE))
    {
      *pLen = 4;
      memcpy(pValue, pAttr->pValue, 4);
    }
    else if (!memcmp(pAttr->type.uuid, edsCfgCharBeaconPeriodUUID, ATT_UUID_SIZE))
    {
      *pLen = 2;
      pValue[0] = LO_UINT16(edsCfgCharBeaconPeriod);
      pValue[1] = HI_UINT16(edsCfgCharBeaconPeriod);
    }
    else if (!memcmp(pAttr->type.uuid, edsCfgCharUIDUUID, ATT_UUID_SIZE))    // ==== new
    {
        *pLen = EDSCFG_CHAR_UID_DATA_LEN;
         memcpy(pValue, pAttr->pValue, *pLen);        
        
    }
    else
    {
      // Should never get here!
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      edsCfgSvc_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t edsCfgSvc_WriteAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint8_t len,
                                        uint16_t offset)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
  }
  else
  {
    // 128-bit UUID
    if (!memcmp(pAttr->type.uuid, edsCfgCharURIDataUUID, ATT_UUID_SIZE))
    {
      if (len > EDSCFG_CHAR_URI_DATA_LEN)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        URIDataLen = len;

        memcpy(edsCfgCharURIData, pValue, len);
      }
    }

    else if (!memcmp(pAttr->type.uuid,
                     edsCfgCharAdvTXPwrLvlsUUID, ATT_UUID_SIZE))
    {
      if (len != 4)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        memcpy(edsCfgCharAdvTXPwrLvls, pValue, 4);
      }
    }
    else if (!memcmp(pAttr->type.uuid,
                     edsCfgCharTXPowerModeUUID, ATT_UUID_SIZE))
    {
      if (len != 1)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else if (*pValue > 3)
      {
        status = ATT_ERR_WRITE_NOT_PERMITTED;
      }
      else
      {
        edsCfgCharTXPowerMode = *pValue;
      }
    }
    else if (!memcmp(pAttr->type.uuid,
                     edsCfgCharBeaconPeriodUUID, ATT_UUID_SIZE))
    {
      if (len != 2)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        uint16 tempPeriod;
        
        edsCfgCharBeaconPeriod = BUILD_UINT16(pValue[0], pValue[1]);

        // convert into multiple of 0.625us
        tempPeriod = (uint16) (edsCfgCharBeaconPeriod * 8L / 5);
        if (0 < tempPeriod && tempPeriod < LL_ADV_NONCONN_INTERVAL_MIN)
        {
          edsCfgCharBeaconPeriod =
            (uint16) (LL_ADV_NONCONN_INTERVAL_MIN * 5 / 8);
        }
      }
    }
    else if (!memcmp(pAttr->type.uuid, edsCfgCharResetUUID, ATT_UUID_SIZE))
    {
      if (len != 1)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        edsCfgCharReset = *pValue;
        if (edsCfgCharReset != 0)
        {
          notifyApp = EDSCFG_RESET;
        }
      }
    }
    else if (!memcmp(pAttr->type.uuid, edsCfgCharUIDUUID, ATT_UUID_SIZE))    
    {
      if (len != EDSCFG_CHAR_UID_DATA_LEN)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        memcpy(edsCfgCharUID, pValue, len);
      }
    }   
    else if (!memcmp(pAttr->type.uuid, edsCfgCharFrameCombUUID, ATT_UUID_SIZE))  
    {
      if (len != 1)
      {
           status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {        
        if (*pValue > 2)
        {
            status = ATT_ERR_INVALID_VALUE;
        }
        else
            edsCfgCharFrameComb = *pValue;
      }
    }    
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && edsCfgSvc_AppCBs && edsCfgSvc_AppCBs->pfnedsCfgSvcChange )
  {
    edsCfgSvc_AppCBs->pfnedsCfgSvcChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/
