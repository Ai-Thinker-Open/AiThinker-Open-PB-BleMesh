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

/*******************************************************************************
* @file			bleIIC_Slave service.c
* @brief		Contains the bleIIC Slave service definitions and prototypes
* @version	1.0
* @date			2018/7/5  
* @author		Rugooo
* 
* Copyright(C) 2016, PhyPlus Semiconductor All rights reserved.
*******************************************************************************/

/*******************************************************************************
*@ Module    			:  Pre-Compiler
*@ Description    :  NULL
*******************************************************************************/
#define _BLE_IIC_SLAVE_SERVICE_CMD_

/*******************************************************************************
*@ Module    			:  Includes
*@ Description    :  None
*******************************************************************************/
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "bleIIC_Slave.h"
#include "bleIIC_Slave_service.h"

#include "log.h"

/*******************************************************************************
*@ Module    			:  Macro Define
*@ Description    :  TX,RX Profile Handler Index
*******************************************************************************/
#define RAWPASS_TX_VALUE_HANDLE     4
#define RAWPASS_RX_VALUE_HANDLE     2

/*******************************************************************************
*@ Module    			:  GLOBAL VARIABLES
*@ Description    :  Rawpass GATT Profile Service UUID
*******************************************************************************/
const unsigned char bleIIC_Slave_ServiceUUID[ATT_UUID_SIZE] =
{0x55, 0xe4,0x05,0xd2,0xaf,0x9f,0xa9,0x8f,0xe5,0x4a,0x7d,0xfe,0x43,0x53,0x53,0x49};

// Characteristic rx uuid
const unsigned char bleIIC_Slave_RxCharUUID[ATT_UUID_SIZE] =
{0xb3,0x9b,0x72,0x34,0xbe,0xec, 0xd4,0xa8,0xf4,0x43,0x41,0x88,0x43,0x53,0x53,0x49};

// Characteristic tx uuid
const unsigned char bleIIC_Slave_TxCharUUID[ATT_UUID_SIZE] =
{0x16,0x96,0x24,0x47,0xc6,0x23, 0x61,0xba,0xd9,0x4b,0x4d,0x1e,0x43,0x53,0x53,0x49};

/*******************************************************************************
*@ Module    			:  LOCAL VARIABLES
*@ Description    :  None
*******************************************************************************/
static bleIIC_Slave_ProfileChangeCB_t bleIIC_Slave_AppCBs = NULL;

/*******************************************************************************
*@ Module    			:  LOCAL VARIABLES
*@ Description    :  Profile Attributes - variables
*******************************************************************************/
// Profile Service attribute
static const gattAttrType_t bleIIC_Slave_Service = { ATT_UUID_SIZE, bleIIC_Slave_ServiceUUID };

// Profile Characteristic 1 Properties
static unsigned char bleIIC_Slave_RxCharProps = GATT_PROP_WRITE_NO_RSP| GATT_PROP_WRITE;

// Characteristic 1 Value
static unsigned char bleIIC_Slave_RxCharValue[RAWPASS_RX_BUFF_SIZE];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

// Profile Characteristic 2 Properties
static unsigned char bleIIC_Slave_TxCharProps = GATT_PROP_NOTIFY| GATT_PROP_INDICATE;

// Characteristic 2 Value
static unsigned char bleIIC_Slave_TxCharValue = 0;

// Simple Profile Characteristic 2 User Description
static gattCharCfg_t bleIIC_Slave_TxCCCD;

/*******************************************************************************
*@ Module    			:  LOCAL VARIABLES
*@ Description    :  Profile Attributes - Table
*******************************************************************************/
static gattAttribute_t bleIIC_Slave_ProfileAttrTbl[] = 
{
  // Simple Profile Service
  { 
	{ ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
	GATT_PERMIT_READ,						  /* permissions */
	0,										  /* handle */
	(uint8 *)&bleIIC_Slave_Service			 /* pValue */
  },

	// Characteristic 1 Declaration
	{ 
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ, 
	  0,
	  &bleIIC_Slave_RxCharProps 
	},

	  // Characteristic Value 1
	  { 
		{ ATT_UUID_SIZE, bleIIC_Slave_RxCharUUID },
		GATT_PERMIT_WRITE, 
		0, 
		&bleIIC_Slave_RxCharValue[0] 
	  },

	// Characteristic 2 Declaration
	{ 
	  { ATT_BT_UUID_SIZE, characterUUID },
	  GATT_PERMIT_READ, 
	  0,
	  &bleIIC_Slave_TxCharProps 
	},

	  // Characteristic Value 2
	  { 
		{ ATT_UUID_SIZE, bleIIC_Slave_TxCharUUID },
		0, 
		0, 
		(uint8 *)&bleIIC_Slave_TxCharValue 
	  },

	  // Characteristic 2 User Description
	  { 
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ|GATT_PERMIT_WRITE, 
		0, 
		(uint8*)&bleIIC_Slave_TxCCCD 
	  },		  
};

/*******************************************************************************
*@ Module    			:  LOCAL FUNCTIONS
*@ Description    :  None
*******************************************************************************/
static uint8 bleIIC_Slave_ReadAttrCB( unsigned short int connHandle, \
												gattAttribute_t *pAttr, 
												unsigned char *pValue,\
												unsigned char *pLen, \
												unsigned short int offset, \
												unsigned char maxLen );
static bStatus_t bleIIC_Slave_WriteAttrCB( unsigned short int connHandle, \
													gattAttribute_t *pAttr,
								 					unsigned char *pValue, \
								 					unsigned char len, \
								 					unsigned short int offset );
static void bleIIC_Slave_HandleConnStatusCB ( unsigned short int connHandle, \
															unsigned char changeType );

const gattServiceCBs_t bleIIC_Slave_ProfileCBs =
{
	bleIIC_Slave_ReadAttrCB,  // Read callback function pointer
	bleIIC_Slave_WriteAttrCB, // Write callback function pointer
	NULL						 // Authorization callback function pointer
};


/*******************************************************************************
*@ Module    			:  
*@ Description    :  
*******************************************************************************/
/*******************************************************************************
*@ Description    :  bleIIC_Slave_AddService
*@ Input          :  cb:service callback
*@ Output         :  None
*@ Return         :  Success or Failure
*******************************************************************************/
bStatus_t bleIIC_Slave_AddService( bleIIC_Slave_ProfileChangeCB_t cb)
{
	unsigned char status = SUCCESS;
	
	// Register with Link DB to receive link status change callback
	VOID linkDB_Register( bleIIC_Slave_HandleConnStatusCB  );  

	bleIIC_Slave_TxCCCD.connHandle = INVALID_CONNHANDLE;
	bleIIC_Slave_TxCCCD.value = 0;
	// Register GATT attribute list and CBs with GATT Server App
	status = GATTServApp_RegisterService( bleIIC_Slave_ProfileAttrTbl, 
										GATT_NUM_ATTRS( bleIIC_Slave_ProfileAttrTbl ),
										&bleIIC_Slave_ProfileCBs );
	if(status!=SUCCESS)
	  LOG("Add rawpass service failed!\n");
	bleIIC_Slave_AppCBs = cb;

	return ( status );
}

/*******************************************************************************
*@ Module    			:  bleIIC_Slave_NotifyIsReady
*@ Description    :  None
*******************************************************************************/
uint8 bleIIC_Slave_NotifyIsReady(void)
{
	return (bleIIC_Slave_TxCCCD.value == GATT_CLIENT_CFG_NOTIFY);
}

/*******************************************************************************
*@ Module    			:  
*@ Description    :  
*******************************************************************************/
bStatus_t bleIIC_Slave_Notify( unsigned short int connHandle, \
													attHandleValueNoti_t *pNoti, 
													unsigned char taskId )
{
	unsigned short int value = bleIIC_Slave_TxCCCD.value;

	// If notifications enabled
	if ( value & GATT_CLIENT_CFG_NOTIFY )
	{
		// Set the handle
		pNoti->handle = bleIIC_Slave_ProfileAttrTbl[RAWPASS_TX_VALUE_HANDLE].handle;

		// Send the Indication
		return GATT_Notification( connHandle, pNoti, FALSE);

	}
	return bleIncorrectMode;

}

/*******************************************************************************
*@ Description    :  bleIIC_Slave_ReadAttrCB
*@ Input          :
					* @param	   connHandle - connection message was received on
					* @param	   pAttr - pointer to attribute
					* @param	   pValue - pointer to data to be read
					* @param	   pLen - length of data to be read
					* @param	   offset - offset of the first octet to be read
					* @param	   maxLen - maximum length of data to be read

*@ Output         : None
*@ Return         : Success or Failure
*******************************************************************************/
static uint8 bleIIC_Slave_ReadAttrCB( unsigned short int connHandle, \
												gattAttribute_t *pAttr, 
												unsigned char *pValue,\
												unsigned char *pLen, \
												unsigned short int offset, \
												unsigned char maxLen )
{
	bStatus_t status = SUCCESS;
	LOG("ReadAttrCB\n");
	// If attribute permissions require authorization to read, return error
	if ( gattPermitAuthorRead( pAttr->permissions ) )
	{
		// Insufficient authorization
		return ( ATT_ERR_INSUFFICIENT_AUTHOR );
	}

	// Make sure it's not a blob operation (no attributes in the profile are long) 
	if ( pAttr->type.len == ATT_BT_UUID_SIZE )
	{
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
		if(uuid == GATT_CLIENT_CHAR_CFG_UUID)
		{
			*pLen = 2;
			osal_memcpy(pValue, pAttr->pValue, 2);
		}
	}
	else
	{
		if(!osal_memcmp(pAttr->type.uuid, bleIIC_Slave_TxCharUUID, 16))
		{
			*pLen = 1;
			pValue[0] = '1';
		}
		else if(!osal_memcmp(pAttr->type.uuid, bleIIC_Slave_RxCharUUID, 16))
		{
			LOG("read tx char\n");
		}
	}
	return ( status );
}
												
/*******************************************************************************
*@ Description    :  bleIIC_Slave_WriteAttrCB
*@ Input          :
					* @param   connHandle - connection message was received on
					* @param   pAttr - pointer to attribute
					* @param   pValue - pointer to data to be written
					* @param   len - length of data
					* @param   offset - offset of the first octet to be written

*@ Output         :  None
*@ Return         :  Success or Failure
*******************************************************************************/
static bStatus_t bleIIC_Slave_WriteAttrCB( unsigned short int connHandle, \
													gattAttribute_t *pAttr,
													unsigned char *pValue, \
													unsigned char len, \
													unsigned short int offset )
{


	bStatus_t status = SUCCESS;
	//uint8 notifyApp = 0xFF;
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
		if(uuid == GATT_CLIENT_CHAR_CFG_UUID)
		{
		  status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
												   offset, GATT_CLIENT_CFG_NOTIFY );
		  if ( status == SUCCESS && bleIIC_Slave_AppCBs)
		  {
			uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
			bleIIC_Slave_Evt_t evt;
		  
			LOG("CCCD set: [%d]\n", charCfg);
			evt.ev = (charCfg == GATT_CFG_NO_OPERATION)?bleIIC_Slave_EVT_TX_NOTI_DISABLED:bleIIC_Slave_EVT_TX_NOTI_ENABLED;
			bleIIC_Slave_AppCBs(&evt);
		  }
		}

	}
	else
	{
		LOG("Come here\n");
		// 128-bit UUID
		if(pAttr->handle == bleIIC_Slave_ProfileAttrTbl[RAWPASS_RX_VALUE_HANDLE].handle)
		{
		  if(bleIIC_Slave_AppCBs){
			bleIIC_Slave_Evt_t evt;
			evt.ev = bleIIC_Slave_EVT_BLE_DATA_RECIEVED;
			evt.param = (uint16_t)len;
			evt.data = pValue;
			bleIIC_Slave_AppCBs(&evt);
		  }
		}
	}
	return ( status );
}
													
/*******************************************************************************
*@ Description    :  bleIIC_Slave_HandleConnStatusCB
*@ Input          :
					* @param	   connHandle - connection handle
					* @param	   changeType - type of change
*@ Output         :  None
*@ Return         :  None
*******************************************************************************/
static void bleIIC_Slave_HandleConnStatusCB ( unsigned short int connHandle, \
															unsigned char changeType )
{
	// Make sure this is not loopback connection
	if ( connHandle != LOOPBACK_CONNHANDLE )
	{
		// Reset Client Char Config if connection has dropped
		if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED ) 	 ||
				( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
				( !linkDB_Up( connHandle ) ) ) )
		{ 
			bleIIC_Slave_TxCCCD.value = 0;
			LOG("clear client configuration\n");
		}
	}

}
