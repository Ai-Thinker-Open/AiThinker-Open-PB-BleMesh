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


/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "weixinservice.h"
#include "linkdb.h"
#include "WeChatble.h"
#include "ble_wechat_util.h"
#include "log.h"
/*********************************************************************
 * MACROS
 */

#define WEIXIN_INDICATE_VALUE   8
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
static uint16_t connectionHandle;

uint16 g_step_number=20600;
uint8_t *g_data = NULL;
int g_len = 0;
int g_flag = 0;
bool g_send_last= TRUE;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// weixin  service
CONST uint8 weixinServUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WEIXIN_SERVICE_UUID), HI_UINT16(WEIXIN_SERVICE_UUID)
};

// weixin indicate uuid
CONST uint8 weixinindicateUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WEIXIN_INDICATE_UUID), HI_UINT16(WEIXIN_INDICATE_UUID)
};

// weixin read uuid
CONST uint8 weixinreadUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WEIXIN_READ_UUID), HI_UINT16(WEIXIN_READ_UUID)
};

// weixin write uuid
CONST uint8 weixinwriteUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WEIXIN_WRITE_UUID), HI_UINT16(WEIXIN_WRITE_UUID)
};

// weixin pedometer uuid
CONST uint8 weixinPedometerUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WEIXIN_PEDOMETER_UUID), HI_UINT16(WEIXIN_PEDOMETER_UUID)
};

// weixin target uuid
CONST uint8 weixinTargetUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(WEIXIN_TARGET_UUID), HI_UINT16(WEIXIN_TARGET_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 AppWrist_TaskID;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern void* memcpy(void *dest, const void *src, size_t len);

/*********************************************************************
 * LOCAL VARIABLES
 */

static weixinProfileCBs_t weixinProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// weixin Profile Service attribute
static CONST gattAttrType_t weixinProfileService = { ATT_BT_UUID_SIZE, weixinServUUID };


// weixin Profile Characteristic read Properties
static uint8 weixinreadProfileCharProps = GATT_PROP_READ;

// Characteristic read Value
static uint8 weixinreadProfileChar[20] = {0};

// weixin Profile Characteristic read User Description
static uint8 weixinreadProfileCharUserDesp[17] = "weixin read";


// weixin Profile Characteristic write Properties
static uint8 weixinwriteProfileCharProps = GATT_PROP_WRITE;

// Characteristic write Value
static uint8 weixinwriteProfileChar[20] = {0};

// weixin Profile Characteristic write User Description
static uint8 weixinwriteProfileCharUserDesp[17] = "weixin write";


// weixin Profile Characteristic indicate Properties
static uint8 weixinindicateProfileCharProps = GATT_PROP_INDICATE;

// Characteristic indicate Value
static uint8 weixinindicateProfileChar[20] = {0};

// weixin Profile Characteristic indicate Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
gattCharCfg_t weixinindicateProfileCharConfig[GATT_MAX_NUM_CONN];

// weixin Profile Characteristic indicate User Description
static uint8 weixinindicateProfileCharUserDesp[17] = "weixin indicate";


// weixin Profile Characteristic indicate Properties
static uint8 weixinPedometerCharProps = GATT_PROP_READ | GATT_PROP_INDICATE;

// Characteristic indicate Value
uint8 weixinPedometerChar[4] = {0};

// weixin Profile Characteristic indicate Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
gattCharCfg_t weixinPedometerCharConfig[GATT_MAX_NUM_CONN];

// weixin Profile Characteristic indicate User Description
static uint8 weixinPedometerCharUserDesp[30] = "weixin pedometer measurement";


// weixin Profile Characteristic indicate Properties
static uint8 weixinTargetCharProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_INDICATE;

// Characteristic indicate Value
uint8 weixinTargetChar[4] = {0};

// weixin Profile Characteristic indicate Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
gattCharCfg_t weixinTargetCharConfig[GATT_MAX_NUM_CONN];

// weixin Profile Characteristic indicate User Description
static uint8 weixinTargetCharUserDesp[30] = "weixin pedometer target";

static void wechat_write_cb_consume(uint8 * newValue);

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t weixinProfileAttrTbl[WEIXINSERVAPP_NUM_ATTR_SUPPORTED] =
{
    // weixin Profile Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8 *)&weixinProfileService            /* pValue */
    },
/*---------------------------------------------------------------------------read*/		

		
    // Characteristic read Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &weixinreadProfileCharProps
    },

    // Characteristic Value read
    {
        { ATT_BT_UUID_SIZE, weixinreadUUID },
        GATT_PERMIT_READ,
        0,
        weixinreadProfileChar
    },

    // Characteristic read User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        weixinreadProfileCharUserDesp
    },
/*---------------------------------------------------------------------------write*/		
    // Characteristic write Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &weixinwriteProfileCharProps
    },

    // Characteristic Value write
    {
        { ATT_BT_UUID_SIZE, weixinwriteUUID },
        GATT_PERMIT_WRITE,
        0,
        weixinwriteProfileChar
    },

    // Characteristic write User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        weixinwriteProfileCharUserDesp
    },
/*---------------------------------------------------------------------------indicate*/		
    // Characteristic indicate Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &weixinindicateProfileCharProps
    },

    // Characteristic Value indicate
    {
        { ATT_BT_UUID_SIZE, weixinindicateUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        weixinindicateProfileChar
    },

    // Characteristic indicate configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&weixinindicateProfileCharConfig
    },

    // Characteristic indicate User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        weixinindicateProfileCharUserDesp
    },
		
/*---------------------------------------------------------------------------Pedometer*/		

	// Characteristic pedometer Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &weixinPedometerCharProps
    },

    // Characteristic Value pedometer
    {
        { ATT_BT_UUID_SIZE, weixinPedometerUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        weixinPedometerChar
    },

    // Characteristic pedometer configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&weixinPedometerCharConfig
    },

    // Characteristic pedometer User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        weixinPedometerCharUserDesp
    },
/*---------------------------------------------------------------------------weixinTarget*/		

	
	// Characteristic target Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &weixinTargetCharProps
    },

    // Characteristic Value target
    {
        { ATT_BT_UUID_SIZE, weixinTargetUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        weixinTargetChar
    },

    // Characteristic target configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&weixinTargetCharConfig
    },

    // Characteristic target User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        weixinTargetCharUserDesp
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void WeixinProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

static bStatus_t weixin_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 *pLen, uint16 offset,
                                    uint8 maxLen);

static bStatus_t weixin_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint8 len, uint16 offset);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// weixin Service Callbacks
CONST gattServiceCBs_t weixinserviceCBs =
{
    weixin_ReadAttrCB, // Read callback function pointer
    weixin_WriteAttrCB,               // Write callback function pointer
    NULL                // Authorization callback function pointer
};

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Weixin_AddService
 *
 * @brief   Initializes the Weixin  service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Weixin_AddService(  uint32 services )
{
    uint8 status = SUCCESS;

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, weixinindicateProfileCharConfig );

    // Register with Link DB to receive link status change callback
    VOID linkDB_Register( WeixinProfile_HandleConnStatusCB );

    if ( services & WEIXINPROFILE_SERVICE )
    {
        // Register GATT attribute list and CBs with GATT Server App
        status = GATTServApp_RegisterService( weixinProfileAttrTbl,
                                              GATT_NUM_ATTRS( weixinProfileAttrTbl ),
                                              &weixinserviceCBs );
    }

    return ( status );
}
/*********************************************************************
 * @fn      WeixinProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t WeixinProfile_RegisterAppCBs( weixinProfileCBs_t appCallbacks )
{
    if (appCallbacks)
    {
        weixinProfile_AppCBs = appCallbacks;

        return  SUCCESS;
    }
    else
    {
        return bleAlreadyInRequestedMode;
    }
}

/*********************************************************************
 * @fn          WeixinProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void WeixinProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
             ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
               ( !linkDB_Up( connHandle ) ) ) )
        {
            GATTServApp_InitCharCfg( connHandle, weixinindicateProfileCharConfig );
        }
    }
}


/*********************************************************************
 * @fn      Weixin_SetParameter
 *
 * @brief   Set a Weixin parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Weixin_SetParameter( uint8 param, uint8 len, void *value )
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {
        case WEIXINPROFILE_READ_CHAR:
            if( (len > 0) && ( len <= 20 ))
            {
                VOID memcpy( weixinreadProfileChar, value, len );
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case WEIXINPROFILE_WRITE_CHAR:
            if( (len > 0) && ( len <= 20 ))
            {
                VOID memcpy( weixinwriteProfileChar, value, len );
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

        case WEIXINPROFILE_INDICATE_CHAR:
            if( (len > 0) && ( len <= 20 ))
            {
                // weixinindicateProfileChar = *((uint8*)value);
                VOID memcpy( weixinindicateProfileChar, value, len );

                // See if Notification has been enabled
                GATTServApp_ProcessCharCfg( weixinindicateProfileCharConfig, weixinindicateProfileChar, FALSE,
                                            weixinProfileAttrTbl, GATT_NUM_ATTRS( weixinProfileAttrTbl ),
                                            INVALID_TASK_ID );

            }
            else
            {
                ret = bleInvalidRange;
            }
            break;

		case WEIXIN_SIMPLEPROFILE_PEDOMETER_CHAR:
            if( (len > 0) && ( len <= 20 ))
            {
                // weixinindicateProfileChar = *((uint8*)value);
                VOID memcpy( weixinPedometerChar, value, len );

                // See if Notification has been enabled
                GATTServApp_ProcessCharCfg( weixinPedometerCharConfig, weixinPedometerChar, FALSE,
                                            weixinProfileAttrTbl, GATT_NUM_ATTRS( weixinProfileAttrTbl ),
                                            INVALID_TASK_ID );

            }
            else
            {
                ret = bleInvalidRange;
            }
            break;
			
		case WEIXIN_SIMPLEPROFILE_TARGET_CHAR:
            if( (len > 0) && ( len <= 20 ))
            {
                // weixinindicateProfileChar = *((uint8*)value);
                VOID memcpy( weixinTargetChar, value, len );

                // See if Notification has been enabled
                GATTServApp_ProcessCharCfg( weixinTargetCharConfig, weixinTargetChar, FALSE,
                                            weixinProfileAttrTbl, GATT_NUM_ATTRS( weixinProfileAttrTbl ),
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
 * @fn      Weixin_GetParameter
 *
 * @brief   Get a Weixin  parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Weixin_GetParameter( uint8 param, void *value )
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {
        case WEIXINPROFILE_READ_CHAR:
            memcpy(value, weixinreadProfileChar, sizeof(weixinreadProfileChar));
            break;

        case WEIXINPROFILE_WRITE_CHAR:
            memcpy(value, weixinwriteProfileChar, sizeof(weixinwriteProfileChar));
            break;
			
        case WEIXINPROFILE_INDICATE_CHAR:
            memcpy(value, weixinindicateProfileChar, sizeof(weixinindicateProfileChar));
            break;
		
		case WEIXIN_SIMPLEPROFILE_PEDOMETER_CHAR:
            memcpy(value, weixinPedometerChar, sizeof(weixinPedometerChar));
            break;
		
		case WEIXIN_SIMPLEPROFILE_TARGET_CHAR:
            memcpy(value, weixinTargetChar, sizeof(weixinTargetChar));
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }

    return ( ret );
}

/*********************************************************************
 * @fn          Weixin_pedometertargetIndicata
 *
 * @brief       Send a Indicate containing a RSC measurement.
 *
 * @param       connHandle - connection handle
 * @param       pNoti      - pointer to notification structure
 *
 * @return      Success or Failure
 */
/*
bStatus_t Weixin_pedometertargetIndicata(uint8_t taskId)
{


 GATT_Indication(connectionHandle, &pedometertargetInd, FALSE, ICall_getEntityId()) ;

}
*/

/*********************************************************************
 * @fn          weixin_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
extern void device_senddata_func(void *args, uint8_t **r_data, uint32_t *r_len);
static bStatus_t weixin_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 *pLen, uint16 offset,
                                    uint8 maxLen)
{
    bStatus_t status = SUCCESS;
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    //uint8_t valueToCopy[]={0};

    // If the value offset of the Read Blob Request is greater than the
    // length of the attribute value, an Error Response shall be sent with
    // the error code Invalid Offset.
    switch (uuid)
    {
        case WEIXIN_READ_UUID:
            // determine read length
            *pLen = 20;

            // copy data
            VOID memcpy( pValue, pAttr->pValue, 20 );
						LOG("(R)WEIXIN_READ_UUID\n");
            break;
				
        case WEIXIN_INDICATE_UUID:
            // determine read length
            *pLen = 20;

            // copy data
            VOID memcpy( pValue, pAttr->pValue, 20 );
						LOG("(R)WEIXIN_INDICATE_UUID\n");
            break;
		
			case WEIXIN_PEDOMETER_UUID:            
            *pLen = 4;// determine read length        
            VOID memcpy( pValue, pAttr->pValue, 4);// copy data
						//VOID memcpy( pValue, pAttr->pValue, 20);								
						LOG("(R)WEIXIN_PEDOMETER_UUID\n");
            break;
		
			case WEIXIN_TARGET_UUID:           
            *pLen = 4; // determine read length            
            VOID memcpy( pValue, pAttr->pValue, 4);// copy data
//	VOID memcpy( pValue, pAttr->pValue, 20 );
		
//device_senddata();
//		{
//			uint8_t *data = NULL;
//			uint32_t len = 0;
//			wechat_info m_info = {CMD_NULL, {SEND_HELLO_WECHAT, sizeof(SEND_HELLO_WECHAT)}};

//			device_senddata_func(&m_info,&data, &len);

//			VOID memcpy( pValue, data, 20 );
//		}
						LOG("(R)WEIXIN_TARGET_UUID\n");
            break;


        default:
            *pLen = 0;
            status = ATT_ERR_ATTR_NOT_FOUND;
						LOG("(R)weixin_ReadAttrCB(error)\n");
            break;
    }

    return ( status );
}


/*********************************************************************
 * @fn      weixin_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t weixin_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint8 len, uint16 offset)
{
    bStatus_t status = SUCCESS;
    uint16 charCfg;

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
        switch (uuid)
        {
            case WEIXIN_WRITE_UUID:
							LOG("(W)WEIXIN_WRITE_UUID\n");
                if(offset == 0)
                {
                    if(len > 20)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                //Write the value
                if(status == SUCCESS)
                {
                    uint8 *pCurValue = (uint8 *)pAttr->pValue;

                    VOID memcpy( pCurValue,pValue, len);

                    if(pAttr->pValue == weixinwriteProfileChar)
                    {
											// If a charactersitic value changed then callback function to notify application of change
											if (weixinProfile_AppCBs)
											{
												weixinProfile_AppCBs(WECHAT_GATT_WRITE);
											}
                    }
                }
                break;

            case GATT_CLIENT_CHAR_CFG_UUID:
								LOG("(W)GATT_CLIENT_CHAR_CFG_UUID\n");
                status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                         offset, GATT_CLIENT_CFG_INDICATE );
			
								charCfg = BUILD_UINT16(pValue[0], pValue[1]);
								if ((status == SUCCESS) && weixinProfile_AppCBs && charCfg == GATT_CLIENT_CFG_INDICATE)
								{
									weixinProfile_AppCBs(WECHAT_GATT_INDICATE);
								}
                break;

						case WEIXIN_TARGET_UUID:
								LOG("(W)WEIXIN_TARGET_UUID\n");
                //Validate the value
                // Make sure it's not a blob oper
                if(offset == 0)
                {
                    if (len > 20)
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }

                //Write the value
                if (status == SUCCESS)
                {
                    uint8 *pCurValue = (uint8 *)pAttr->pValue;
                    VOID memcpy( pCurValue,pValue, len);
                }
                break;
				
            default:
                // Should never get here! (characteristics  do not have write permissions)
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    return (status);
}


bStatus_t ble_send_indicate_data_to_wechat(void)
{
    attHandleValueInd_t senddata;
    int i;
    bool send_err= FALSE;
    bStatus_t stat= FAILURE;
	
    // Get the connection handle.
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);
    uint16 value = GATTServApp_ReadCharCfg(connectionHandle,weixinindicateProfileCharConfig);
    	
	  // If indicate enabled
    if (value & GATT_CLIENT_CFG_INDICATE)
    {
        senddata.handle = weixinProfileAttrTbl[WEIXIN_INDICATE_VALUE].handle;
        if((g_len>20)&& (send_err == FALSE))
        {
            senddata.len = 20;
            for(i=0; i<20; i++)
            {
                senddata.value[i] =g_data[g_flag + i];
            }
            stat = GATT_Indication(connectionHandle, &senddata, FALSE, AppWrist_TaskID);
            if(stat==SUCCESS)
            {
                g_flag +=  20;
                g_len -= 20;
								return stat;
            }
            else
            {
                send_err = TRUE;
            }
        }
				
        if((g_len == 20)||((g_len > 0)&&(g_len < 20)) && (send_err == FALSE))
        {
            senddata.len = g_len;
            for(i=0; i<g_len; i++)
            {
                senddata.value[i] =g_data[g_flag + i];
            }
            stat = GATT_Indication(connectionHandle, &senddata, FALSE, AppWrist_TaskID);
            if(stat==SUCCESS)
            {
                g_len = 0;
                g_send_last= FALSE;
                return stat;
            }
            else
            {
                send_err = TRUE;
            }
        }
    }
    return bleIncorrectMode;
}

bStatus_t ble_send_indicate_data_conform()
{
		//LOG("ble_send_indicate_data_conform\n");
    return ble_send_indicate_data_to_wechat();
}

bStatus_t  ble_wechat_indicate_data(uint8_t *data, int len)
{
    g_data = data;
    g_len = len;
    g_flag = 0;

    return ble_send_indicate_data_to_wechat();
}

/*********************************************************************
*********************************************************************/
