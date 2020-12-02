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
  Filename:       simpleGATTprofile.c
  Revised:         
  Revision:       

  Description:    This file contains the Simple GATT profile sample GATT service 
                  profile for use with the BLE sample application.

 
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "battery.h"
#include "switch.h"
#include "pwm_ctrl.h"

#include "simpleGATTprofile.h"

#include "log.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 simpleProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR1_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 simpleProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR2_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 simpleProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR3_UUID), HI_UINT16(SIMPLEPROFILE_CHAR3_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 simpleProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 simpleProfilechar5UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR5_UUID), HI_UINT16(SIMPLEPROFILE_CHAR5_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

//static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE, simpleProfileServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 simpleProfileChar1Props = GATT_PROP_READ;

// Characteristic 1 Value
static uint16 simpleProfileChar1 = 0;

// Simple Profile Characteristic 1 User Description
static uint8 simpleProfileChar1UserDesp[] = "BATT value\0";


// Simple Profile Characteristic 2 Properties
static uint8 simpleProfileChar2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 2 Value
static uint8 simpleProfileChar2 = 0;

// Simple Profile Characteristic 2 User Description
static uint8 simpleProfileChar2UserDesp[] = "LED1 on\0";


// Simple Profile Characteristic 3 Properties
static uint8 simpleProfileChar3Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic 3 Value
static uint8 simpleProfileChar3[] = {0, 0, 0, 0, 0, 0};

// Simple Profile Characteristic 3 User Description
static uint8 simpleProfileChar3UserDesp[] = "System info\0";

// Simple Profile Characteristic 3 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t simpleProfileChar3Config[GATT_MAX_NUM_CONN];

// Simple Profile Characteristic 4 Properties
static uint8 simpleProfileChar4Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 4 Value
static uint8 simpleProfileChar4 = 0;

// Simple Profile Characteristic 4 User Description
static uint8 simpleProfileChar4UserDesp[] = "PWM switch\0";

// Simple Profile Characteristic 5 Properties
static uint8 simpleProfileChar5Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 5 Value
static uint8 simpleProfileChar5 = 0;

// Simple Profile Characteristic 5 User Description
static uint8 simpleProfileChar5UserDesp[] = "PWM control\0";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleProfileAttrTbl[] = 
{
    // =========== Simple Profile Service
    { 
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&simpleProfileService            /* pValue */
    },

    // ----------------------------------------------------------------------
    // Characteristic 1 Declaration, ADC read only
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &simpleProfileChar1Props 
    },

    // Characteristic Value 1
    { 
        { ATT_BT_UUID_SIZE, simpleProfilechar1UUID },
        GATT_PERMIT_READ, 
        0, 
        (uint8 *)&simpleProfileChar1 
    },

    // Characteristic 1 User Description, this field is optional
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar1UserDesp 
    },      

    // ----------------------------------------------------------------------
    // Characteristic 2 Declaration, PWM, read/write
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &simpleProfileChar2Props 
    },

    // Characteristic Value 2
    { 
        { ATT_BT_UUID_SIZE, simpleProfilechar2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar2 
    },

    // Characteristic 2 User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar2UserDesp 
    },           
      
    // ----------------------------------------------------------------------
    // Characteristic 3 Declaration, some system information
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &simpleProfileChar3Props 
    },

    // Characteristic Value 3
    { 
        { ATT_BT_UUID_SIZE, simpleProfilechar3UUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar3 
    },
    
    // Characteristic 3 configuration
    { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)simpleProfileChar3Config 
    },    

    // Characteristic 3 User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar3UserDesp 
    },
    
    // ----------------------------------------------------------------------
    // Characteristic 4 Declaration, PWM, read/write
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &simpleProfileChar4Props 
    },

    // Characteristic Value 4
    { 
        { ATT_BT_UUID_SIZE, simpleProfilechar4UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar4 
    },

    // Characteristic 4 User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar4UserDesp 
    },  

    // ----------------------------------------------------------------------
    // Characteristic 5 Declaration, PWM, read/write
    { 
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ, 
        0,
        &simpleProfileChar5Props 
    },

    // Characteristic Value 5
    { 
        { ATT_BT_UUID_SIZE, simpleProfilechar5UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar5 
    },

    // Characteristic 5 User Description
    { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar5UserDesp 
    },
 

};

extern switch_cfg_t sw[];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t simpleProfileCBs =
{
    simpleProfile_ReadAttrCB,  // Read callback function pointer
    simpleProfile_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService( uint32 services )
{
    uint8 status = SUCCESS;
  
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileChar3Config );
  
    // Register with Link DB to receive link status change callback
    VOID linkDB_Register( simpleProfile_HandleConnStatusCB );  
    
    if ( services & SIMPLEPROFILE_SERVICE )
    {
      // Register GATT attribute list and CBs with GATT Server App
      status = GATTServApp_RegisterService( simpleProfileAttrTbl, 
                                            GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                            &simpleProfileCBs );
    }
  
    return ( status );
}


/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
//bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
//{
//    if ( appCallbacks )
//    {
//      simpleProfile_AppCBs = appCallbacks;
//      
//      return ( SUCCESS );
//    }
//    else
//    {
//      return ( bleAlreadyInRequestedMode );
//    }
//}
  
/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
      case SIMPLEPROFILE_CHAR1:
        if ( len == sizeof ( uint16 ) ) 
        {
          simpleProfileChar1 = *((uint16*)value);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
  
      case SIMPLEPROFILE_CHAR2:
        if ( len == sizeof ( uint8 ) ) 
        {
          simpleProfileChar2 = *((uint8*)value);
        }
        else
        {
          ret = bleInvalidRange;
        }
        break;
  
      case SIMPLEPROFILE_CHAR3:
        if ( len == sizeof ( simpleProfileChar3 ) ) 
        {
          VOID osal_memcpy( simpleProfileChar3, value, len );
            
          GATTServApp_ProcessCharCfg( simpleProfileChar3Config, simpleProfileChar3, FALSE,
                                    simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                    INVALID_TASK_ID );            
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
bStatus_t SimpleProfile_GetParameter( uint8 param, void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {
      case SIMPLEPROFILE_CHAR1:
        *((uint8*)value) = simpleProfileChar1;
        break;
  
      case SIMPLEPROFILE_CHAR2:
        *((uint8*)value) = simpleProfileChar2;
        break;      
  
      case SIMPLEPROFILE_CHAR3:
        VOID osal_memcpy( value, simpleProfileChar3, sizeof(simpleProfileChar3) );
        break;

      case SIMPLEPROFILE_CHAR4:
        *((uint8*)value) = simpleProfileChar4;
        break;
      
      case SIMPLEPROFILE_CHAR5:
        *((uint8*)value) = simpleProfileChar5;
        break;
  
      default:
        ret = INVALIDPARAMETER;
        break;
    }
    
    return ( ret );
}

/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
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
 * @return      Success or Failure
 */
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
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
   
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
      simpleProfile_Read(connHandle,pAttr,pValue,pLen,offset,maxLen);
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
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
//    uint8 notifyApp = 0xFF;
    
    // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
      // Insufficient authorization
      return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }
    
    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
      simpleProfile_Write(connHandle,pAttr,pValue,len,offset);
    }
    else
    {
      // 128-bit UUID
      status = ATT_ERR_INVALID_HANDLE;
    }
  
    // If a charactersitic value changed then callback function to notify application of change
//    if ( (notifyApp != 0xFF ) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange )
//    {
//      simpleProfile_AppCBs->pfnSimpleProfileChange( notifyApp );  
//    }
    
    return ( status );
}

/*********************************************************************
 * @fn          simpleProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
      // Reset Client Char Config if connection has dropped
      if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
           ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
             ( !linkDB_Up( connHandle ) ) ) )
      { 
        GATTServApp_InitCharCfg( connHandle, simpleProfileChar3Config );
      }
    }
}

bStatus_t simpleProfile_Notify( uint8 param, uint8 len, void *value )
{
    bStatus_t ret = SUCCESS;
    switch ( param )
    {    
      case SIMPLEPROFILE_CHAR3:
        if ( len == sizeof ( simpleProfileChar3 ) ) 
        {
          VOID osal_memcpy( simpleProfileChar3, value, len );
            
          GATTServApp_ProcessCharCfg( simpleProfileChar3Config, simpleProfileChar3, FALSE,
                                    simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                    INVALID_TASK_ID );            
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

bStatus_t simpleProfile_Write( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
      bStatus_t status = SUCCESS;
      // 16-bit UUID
      uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
      
      switch(uuid){
        case SIMPLEPROFILE_CHAR2_UUID:
          if ( offset == 0 )
          {
            if ( len != sizeof(simpleProfileChar2) )
            {
              status = ATT_ERR_INVALID_VALUE_SIZE;
            }
          }
          else
          {
            status = ATT_ERR_ATTR_NOT_LONG;
          }
          if ( status == SUCCESS ){
            uint8 *pCurValue = (uint8 *)pAttr->pValue;
            *pCurValue = pValue[0];
            uint8 newValue=simpleProfileChar2;
            if(newValue==0x01){
              switch_on(sw[0]);
            }else if(newValue==0x00){
              switch_off(sw[0]);
            }
          }
          break;
          
          case SIMPLEPROFILE_CHAR4_UUID:
          if ( offset == 0 )
          {
            if ( len != sizeof(simpleProfileChar4) )
            {
              status = ATT_ERR_INVALID_VALUE_SIZE;
            }
          }
          else
          {
            status = ATT_ERR_ATTR_NOT_LONG;
          }
          if ( status == SUCCESS ){
            uint8 *pCurValue = (uint8 *)pAttr->pValue;
            *pCurValue = pValue[0];
            uint8 newValue=simpleProfileChar4;
            if(newValue==0x01){
              pwm_on(0);
            }else if(newValue==0x00){
              pwm_off(0);
            }
          }
          break;
          
          case SIMPLEPROFILE_CHAR5_UUID:
          if ( offset == 0 )
          {
            if ( len != sizeof(simpleProfileChar5) )
            {
              status = ATT_ERR_INVALID_VALUE_SIZE;
            }
          }
          else
          {
            status = ATT_ERR_ATTR_NOT_LONG;
          }
          if ( status == SUCCESS ){
            uint8 *pCurValue = (uint8 *)pAttr->pValue;
            *pCurValue = pValue[0];
            uint8 newValue=simpleProfileChar5;
            pwm_ctrl(0,newValue);
          }
          break;
          
          case GATT_CLIENT_CHAR_CFG_UUID:
            status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
          break;
          default:
            status = ATT_ERR_ATTR_NOT_FOUND;
          break;
      }

  return status;
}

bStatus_t simpleProfile_Read( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
  
  // 16-bit UUID
      uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
      switch ( uuid )
      {
        case SIMPLEPROFILE_CHAR1_UUID:
          *pLen = sizeof(simpleProfileChar1);
        
          static uint16 charValue=0x0000;         
          charValue = (uint16)(batt_voltage()*1000);
          simpleProfileChar1 = ((charValue&0xff)<<8)|((charValue>>8)&0xff);
        
          VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
          break;
        case SIMPLEPROFILE_CHAR2_UUID:
          *pLen = sizeof(simpleProfileChar2);
          VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
          break;
  
        case SIMPLEPROFILE_CHAR3_UUID:
          *pLen = sizeof(simpleProfileChar3);
          VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
          break;
        
        case SIMPLEPROFILE_CHAR4_UUID:
          *pLen = sizeof(simpleProfileChar4);
          VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
          break;
        
        case SIMPLEPROFILE_CHAR5_UUID:
          *pLen = sizeof(simpleProfileChar5);
          VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
          break;
          
        default:
          // Should never get here! (characteristics 3 and 4 do not have read permissions)
          *pLen = 0;
          status = ATT_ERR_ATTR_NOT_FOUND;
          break;
        
        
        }
      
        return status;
}


/*********************************************************************
*********************************************************************/
