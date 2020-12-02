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
#include "common.h"

#include "simpleGATTprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        18

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

// Characteristic 4 UUID: 0xFFF4
CONST uint8 simpleProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 simpleProfilechar5UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR5_UUID), HI_UINT16(SIMPLEPROFILE_CHAR5_UUID)
};

// Characteristic 6 UUID: 0xFFF6
CONST uint8 simpleProfilechar6UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR6_UUID), HI_UINT16(SIMPLEPROFILE_CHAR6_UUID)
};

// Characteristic 7 UUID: 0xFFF7
CONST uint8 simpleProfilechar7UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR7_UUID), HI_UINT16(SIMPLEPROFILE_CHAR7_UUID)
};

// Characteristic 8 UUID: 0xFFF8
CONST uint8 simpleProfilechar8UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR8_UUID), HI_UINT16(SIMPLEPROFILE_CHAR8_UUID)
};

// Characteristic 9 UUID: 0xFFF9
CONST uint8 simpleProfilechar9UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR9_UUID), HI_UINT16(SIMPLEPROFILE_CHAR9_UUID)
};

// Characteristic 10 UUID: 0xFFFA
CONST uint8 simpleProfileChar10UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR10_UUID), HI_UINT16(SIMPLEPROFILE_CHAR10_UUID)
};

// Characteristic 11 UUID: 0xFFFB
CONST uint8 simpleProfilechar11UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR11_UUID), HI_UINT16(SIMPLEPROFILE_CHAR11_UUID)
};

// Characteristic 12 UUID: 0xFFFC
CONST uint8 simpleProfileChar12UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR12_UUID), HI_UINT16(SIMPLEPROFILE_CHAR12_UUID)
};

// Characteristic 13 UUID: 0xFFFD //Manufacture Date
CONST uint8 simpleProfilechar13UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR13_UUID), HI_UINT16(SIMPLEPROFILE_CHAR13_UUID)
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

static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE, simpleProfileServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 simpleProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 simpleProfileChar1[IBEACON_UUID_LEN];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

// Simple Profile Characteristic 1 User Description
static uint8 simpleProfileChar1UserDesp[] = "UUID0\0";


// Simple Profile Characteristic 2 Properties
static uint8 simpleProfileChar2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 2 Value
static uint16 simpleProfileChar2 = 0;

// Simple Profile Characteristic 2 User Description
static uint8 simpleProfileChar2UserDesp[] = "Major0\0";


// Simple Profile Characteristic 3 Properties
static uint8 simpleProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 3 Value
static uint16 simpleProfileChar3 = 0;

// Simple Profile Characteristic 3 User Description
static uint8 simpleProfileChar3UserDesp[] = "Minor0\0";

// Simple Profile Characteristic 4 Properties
static uint8 simpleProfileChar4Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 4 Value
static uint8 simpleProfileChar4[IBEACON_UUID_LEN];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

// Simple Profile Characteristic 4 User Description
static uint8 simpleProfileChar4UserDesp[] = "UUID1\0";

// Simple Profile Characteristic 5 Properties
static uint8 simpleProfileChar5Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 5 Value
static uint16 simpleProfileChar5 = 0;

// Simple Profile Characteristic 5 User Description
static uint8 simpleProfileChar5UserDesp[] = "Major1\0";


// Simple Profile Characteristic 6 Properties
static uint8 simpleProfileChar6Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 6 Value
static uint16 simpleProfileChar6 = 0;

// Simple Profile Characteristic 6 User Description
static uint8 simpleProfileChar6UserDesp[] = "Minor1\0";

// Simple Profile Characteristic 7 Properties
static uint8 simpleProfileChar7Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 7 Value
static uint8 simpleProfileChar7[IBEACON_UUID_LEN];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

// Simple Profile Characteristic 7 User Description
static uint8 simpleProfileChar7UserDesp[] = "UUID2\0";

// Simple Profile Characteristic 8 Properties
static uint8 simpleProfileChar8Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 8 Value
static uint16 simpleProfileChar8 = 0;

// Simple Profile Characteristic 8 User Description
static uint8 simpleProfileChar8UserDesp[] = "Major2\0";


// Simple Profile Characteristic 9 Properties
static uint8 simpleProfileChar9Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 9 Value
static uint16 simpleProfileChar9 = 0;

// Simple Profile Characteristic 9 User Description
static uint8 simpleProfileChar9UserDesp[] = "Minor2\0";


// Simple Profile Characteristic 4 Properties
static uint8 simpleProfileChar10Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 4 Value
static uint8 simpleProfileChar10 = 0;

// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t simpleProfileChar10Config[GATT_MAX_NUM_CONN];
                                        
// Simple Profile Characteristic 4 User Description
static uint8 simpleProfileChar10UserDesp[] = "Power\0";


// Simple Profile Characteristic 5 Properties
static uint8 simpleProfileChar12Props = GATT_PROP_READ | GATT_PROP_WRITE;   // to change to write only, HZF

// Characteristic 5 Value
static uint8 simpleProfileChar12 = 0;

// Simple Profile Characteristic 5 User Description
static uint8 simpleProfileChar12UserDesp[] = "Reset\0";

// Simple Profile Characteristic 12 Properties
static uint8 simpleProfileChar11Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 12 Value
static uint16 simpleProfileChar11 = 0;
                                        
// Simple Profile Characteristic 12 User Description
static uint8 simpleProfileChar11UserDesp[] = "Adv interval\0";

// Manufacture Date String characteristic
static uint8 devInfoManufactureDateProps = GATT_PROP_READ;
static uint8 devInfoManufactureDate[PRODUCTION_MAX_LEN] = "1999-01-01";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleProfileAttrTbl[] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&simpleProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar1[0] 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar1UserDesp 
      },      

    // Characteristic 2 Declaration
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
        (uint8 *)&simpleProfileChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar2UserDesp 
      },           
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&simpleProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar3UserDesp 
      },
      
      // Characteristic 4 Declaration
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
        &simpleProfileChar4[0] 
      },

      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar4UserDesp 
      },      

    // Characteristic 5 Declaration
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
        (uint8 *)&simpleProfileChar5
      },

      // Characteristic 5 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar5UserDesp 
      },           
      
    // Characteristic 6 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar6Props 
    },

      // Characteristic Value 6
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar6UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&simpleProfileChar6 
      },

      // Characteristic 6 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar6UserDesp 
      },
      
      // Characteristic 7 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar7Props 
    },

      // Characteristic Value 7
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar7UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar7[0] 
      },

      // Characteristic 7 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar7UserDesp 
      },      

    // Characteristic 8 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar8Props 
    },

      // Characteristic Value 8
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar8UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&simpleProfileChar8 
      },

      // Characteristic 8 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar8UserDesp 
      },           
      
    // Characteristic 9 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar9Props 
    },

      // Characteristic Value 9
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar9UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&simpleProfileChar9 
      },

      // Characteristic 9 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar9UserDesp 
      },

    // Characteristic 10 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar10Props 
    },

      // Characteristic Value 10
      { 
        { ATT_BT_UUID_SIZE, simpleProfileChar10UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&simpleProfileChar10 
      },

      // Characteristic 10 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar10UserDesp 
      },
      
         // Characteristic 11 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar11Props 
    },
    
    // Characteristic Value 11
      { 
        { ATT_BT_UUID_SIZE, simpleProfilechar11UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&simpleProfileChar11 
      },
      
       // Characteristic 11 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar11UserDesp 
      },
      
    // Characteristic 12 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar12Props 
    },

      // Characteristic Value 12
      { 
        { ATT_BT_UUID_SIZE, simpleProfileChar12UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar12 
      },

      // Characteristic 12 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar12UserDesp 
      },
      
      // Characteristic 13 User Description
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoManufactureDateProps
    },

      // Model Number Value
      {
        { ATT_BT_UUID_SIZE, simpleProfilechar13UUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoManufactureDate
      },
      


};


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
  //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileChar10Config );

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
bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    simpleProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  

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
      if ( len <= IBEACON_UUID_LEN )   
      {
        osal_memcpy(simpleProfileChar1, value, len);				
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR2:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar2 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR3:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar3 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case SIMPLEPROFILE_CHAR4:
      if ( len <= IBEACON_UUID_LEN )   
      {
        osal_memcpy(simpleProfileChar4, value, len);				
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR5:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar5 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR6:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar6 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
   case SIMPLEPROFILE_CHAR7:
      if ( len <= IBEACON_UUID_LEN )   
      {
        osal_memcpy(simpleProfileChar7, value, len);				
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR8:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar8 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR9:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar9 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR10:
      if ( len == sizeof ( uint8 ) ) 
      {
        simpleProfileChar10 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    
    case SIMPLEPROFILE_CHAR11:
      if ( len == sizeof ( uint16 ) ) 
      {
        simpleProfileChar11 = (*(uint8 *)value) << 8 | *((uint8 *)value + 1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR12:
      if ( len == sizeof ( uint8 ) ) 
      {
        simpleProfileChar12 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

	case SIMPLEPROFILE_CHAR13:
      osal_memcpy(devInfoManufactureDate, value, len);
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
      VOID osal_memcpy( value, simpleProfileChar1, IBEACON_UUID_LEN );    
      break;

    case SIMPLEPROFILE_CHAR2:
      *((uint16*)value) = simpleProfileChar2;
      break;      

    case SIMPLEPROFILE_CHAR3:
      *((uint16*)value) = simpleProfileChar3;
      break; 
      
    case SIMPLEPROFILE_CHAR4:
      VOID osal_memcpy( value, simpleProfileChar4, IBEACON_UUID_LEN );    
      break;

    case SIMPLEPROFILE_CHAR5:
      *((uint16*)value) = simpleProfileChar5;
      break;      

    case SIMPLEPROFILE_CHAR6:
      *((uint16*)value) = simpleProfileChar6;
      break;
    
    case SIMPLEPROFILE_CHAR7:
      VOID osal_memcpy( value, simpleProfileChar7, IBEACON_UUID_LEN );    
      break;

    case SIMPLEPROFILE_CHAR8:
      *((uint16*)value) = simpleProfileChar8;
      break;      

    case SIMPLEPROFILE_CHAR9:
      *((uint16*)value) = simpleProfileChar9;
      break;

    case SIMPLEPROFILE_CHAR10:
      *((uint8*)value) = simpleProfileChar10;
      break;
    
    case SIMPLEPROFILE_CHAR11:
      *((uint16*)value) = simpleProfileChar11;
      break;

    case SIMPLEPROFILE_CHAR12:
      *((uint8*)value) = simpleProfileChar12;
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
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      case SIMPLEPROFILE_CHAR1_UUID:
      case SIMPLEPROFILE_CHAR4_UUID:
      case SIMPLEPROFILE_CHAR7_UUID:
	      *pLen = IBEACON_UUID_LEN;
	      VOID osal_memcpy( pValue, pAttr->pValue, IBEACON_UUID_LEN );
	      break;
      case SIMPLEPROFILE_CHAR2_UUID:
      case SIMPLEPROFILE_CHAR3_UUID:
      case SIMPLEPROFILE_CHAR5_UUID:
      case SIMPLEPROFILE_CHAR6_UUID:
      case SIMPLEPROFILE_CHAR8_UUID:
      case SIMPLEPROFILE_CHAR9_UUID:
      case SIMPLEPROFILE_CHAR11_UUID:
	  	//case SIMPLEPROFILE_CHAR10_UUID:
	      *pLen = 2;
	      VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
	      break;
      case SIMPLEPROFILE_CHAR10_UUID:
      case SIMPLEPROFILE_CHAR12_UUID:
      
	      *pLen = 1;
	      pValue[0] = *pAttr->pValue;
	      break;
      
      case SIMPLEPROFILE_CHAR13_UUID:
      // verify offset
      if (offset >= (sizeof(devInfoManufactureDate) - 1))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length (exclude null terminating character)
        *pLen = MIN(maxLen, ((hal_my_sizeof(devInfoManufactureDate)) - offset));

        // copy data
        osal_memcpy(pValue, &devInfoManufactureDate[offset], *pLen);
      }
      break;
        
      default:
	      // Should never get here! (characteristics 3 and 4 do not have read permissions)
	      *pLen = 0;
	      status = ATT_ERR_ATTR_NOT_FOUND;
	      break;
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
 // TODO: test this function
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
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
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SIMPLEPROFILE_CHAR1_UUID:
      case SIMPLEPROFILE_CHAR4_UUID:
      case SIMPLEPROFILE_CHAR7_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != IBEACON_UUID_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          VOID osal_memcpy( pCurValue, pValue, IBEACON_UUID_LEN );
          if( uuid == SIMPLEPROFILE_CHAR1_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR1;        
          }else if( uuid == SIMPLEPROFILE_CHAR4_UUID ){
            notifyApp = SIMPLEPROFILE_CHAR4; 
          }else if( uuid == SIMPLEPROFILE_CHAR7_UUID ){
            notifyApp = SIMPLEPROFILE_CHAR7; 
          }
            
        }
             	  	
	  	break;
      case SIMPLEPROFILE_CHAR2_UUID:	  	
      case SIMPLEPROFILE_CHAR3_UUID:
      case SIMPLEPROFILE_CHAR5_UUID:	  	
      case SIMPLEPROFILE_CHAR6_UUID:
      case SIMPLEPROFILE_CHAR8_UUID:	  	
      case SIMPLEPROFILE_CHAR9_UUID:
      case SIMPLEPROFILE_CHAR11_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 2 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];
          *(pCurValue + 1	) =  pValue[1];   

          if( uuid == SIMPLEPROFILE_CHAR2_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR2;        
          }
          else if( uuid == SIMPLEPROFILE_CHAR3_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR3;           
          }else if( uuid == SIMPLEPROFILE_CHAR5_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR5;           
          }else if( uuid == SIMPLEPROFILE_CHAR6_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR6;           
          }else if( uuid == SIMPLEPROFILE_CHAR8_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR8;           
          }else if( uuid == SIMPLEPROFILE_CHAR9_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR9;           
          }else if( uuid == SIMPLEPROFILE_CHAR11_UUID ){
            notifyApp = SIMPLEPROFILE_CHAR11; 
          }
        }
        
             	  	
	  	break;	  	
      case SIMPLEPROFILE_CHAR10_UUID:
      case SIMPLEPROFILE_CHAR12_UUID:
      

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 1 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];

          if( uuid == SIMPLEPROFILE_CHAR10_UUID )
          {
            notifyApp = SIMPLEPROFILE_CHAR10;        
          }
          else
          {
            notifyApp = SIMPLEPROFILE_CHAR12;           
          }
        }
             
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange )
  {
    simpleProfile_AppCBs->pfnSimpleProfileChange( notifyApp );  
  }
  
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
      GATTServApp_InitCharCfg( connHandle, simpleProfileChar10Config );
    }
  }
}


/*********************************************************************
*********************************************************************/
