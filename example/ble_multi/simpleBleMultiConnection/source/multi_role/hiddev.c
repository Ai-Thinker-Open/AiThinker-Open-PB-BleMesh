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

#include "OSAL.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "gapbondmgr.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"
#include "hidkbdservice.h"
//#include "touch_key.h"
#include "log.h"
#include "multi.h"
#include "multi_role.h"

/*********************************************************************
 * MACROS
 */

// TRUE to run scan parameters refresh notify test
#define DEFAULT_SCAN_PARAM_NOTIFY_TEST        TRUE

#define reportQEmpty(x)                        ( firstQIdx[x] == lastQIdx[x] )

/*********************************************************************
 * CONSTANTS
 */

#define HID_DEV_DATA_LEN                      8

#ifdef HID_DEV_RPT_QUEUE_LEN
  #define HID_DEV_REPORT_Q_SIZE               (HID_DEV_RPT_QUEUE_LEN+1)
#else
  #define HID_DEV_REPORT_Q_SIZE               (10+1)
#endif

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
 uint8 id;
 uint8 type;
 uint8 len;
 uint8 data[HID_DEV_DATA_LEN];
} hidDevReport_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
 
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
 extern uint8 multiRole_TaskId;

/*********************************************************************
 * LOCAL VARIABLES
 */
 // GAP State
extern MultiRoleApp_Link_t g_MRLink[MAX_CONNECTION_NUM];

static hidRptMap_t *pHidDevRptTbl;

static uint8 hidDevRptTblLen;

static hidDevCB_t *pHidDevCB;

hidDevCfg_t *pHidDevCfg;

// Whether to change to the preferred connection parameters
static uint8 updateConnParams = TRUE;

// Pending reports
static uint8 firstQIdx[MAX_CONNECTION_NUM] = {0};
static uint8 lastQIdx[MAX_CONNECTION_NUM] = {0};
static hidDevReport_t hidDevReportQ[MAX_CONNECTION_NUM][HID_DEV_REPORT_Q_SIZE];

// Last report sent out
static attHandleValueNoti_t lastNoti[MAX_CONNECTION_NUM] = { 0 };

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static hidRptMap_t *hidDevRptByHandle( uint16 handle );
static hidRptMap_t *hidDevRptById( uint8 id, uint8 type );
static hidRptMap_t *hidDevRptByCccdHandle( uint16 handle );
static void hidDevEnqueueReport( uint16 connHandle,uint8 id, uint8 type, uint8 len, uint8 *pData );
//static hidDevReport_t *hidDevDequeueReport( uint16 connHandle );
static void hidDevSendReport(uint16 connHandle,uint8 id, uint8 type, uint8 len, uint8 *pData );
static uint8 hidDevBondCount( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      HidDev_Register
 *
 * @brief   Register a callback function with HID Dev.
 *
 * @param   pCfg - Parameter configuration.
 * @param   pfnServiceCB - Callback function.
 *
 * @return  None.
 */
void HidDev_Register( hidDevCfg_t *pCfg, hidDevCB_t *pCBs )
{
  pHidDevCB = pCBs;
  pHidDevCfg = pCfg;
}

/*********************************************************************
 * @fn      HidDev_RegisterReports
 *
 * @brief   Register the report table with HID Dev.
 *
 * @param   numReports - Length of report table.
 * @param   pRpt - Report table.
 *
 * @return  None.
 */
void HidDev_RegisterReports( uint8 numReports, hidRptMap_t *pRpt )
{
  pHidDevRptTbl = pRpt;
  hidDevRptTblLen = numReports;
}

/*********************************************************************
 * @fn      HidDev_Report
 *
 * @brief   Send a HID report.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
void HidDev_Report( uint16 connHandle,uint8 id, uint8 type, uint8 len, uint8*pData )
{
AT_LOG("%s\n",__func__);
  // if connected
  if ( g_MRLink[connHandle].state == GAPROLE_CONNECTED )
  {
    // if connection is secure
    if ( g_MRLink[connHandle].ConnSecure )
    {
      // Make sure there're no pending reports
      if ( reportQEmpty(connHandle) )
      {
        // send report
        hidDevSendReport( connHandle,id, type, len, pData );

        return; // we're done
      }
    }
  }

  // hidDev task will send report when secure connection is established
  hidDevEnqueueReport(connHandle,id, type, len, pData );
}

/*********************************************************************
 * @fn      HidDev_Close
 *
 * @brief   Close the connection or stop advertising.
 *
 * @return  None.
 */
void HidDev_Close( void )
{
//  uint8 param;

//  // if connected then disconnect
//  if ( hidDevGapState == GAPROLE_CONNECTED )
//  {
//    GAPRole_TerminateConnection();
//  }
//  // else stop advertising
//  else
//  {
//    param = FALSE;
//    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &param );
//  }
}

/*********************************************************************
 * @fn      HidDev_SetParameter
 *
 * @brief   Set a HID Dev parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t HidDev_SetParameter( uint16 connHandle,uint8 param, uint8 len, void *pValue )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case HIDDEV_ERASE_ALLBONDS:
      if ( len == 0 )
      {
        // See if the last report sent out wasn't a release key
        if ( osal_isbufset( lastNoti[connHandle].value, 0x00, lastNoti[connHandle].len ) == FALSE )
        {
          // Send a release report before disconnecting, otherwise
          // the last pressed key would get 'stuck' on the HID Host.
          osal_memset( lastNoti[connHandle].value, 0x00, lastNoti[connHandle].len );

          GATT_Notification( connHandle, &lastNoti[connHandle], FALSE );
        }

        // Drop connection
        if ( g_MRLink[connHandle].state == GAPROLE_CONNECTED )
        {
//          GAPRole_TerminateConnection();
        }

        // Flush report queue
        firstQIdx[connHandle] = lastQIdx[connHandle] = 0;

        // Erase bonding info
        GAPBondMgr_SetParameter( GAPBOND_ERASE_ALLBONDS, 0, NULL );
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
 * @fn          HidDev_ReadAttrCB
 *
 * @brief       HID Dev attribute read callback.
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
uint8 HidDev_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                         uint8 *pValue, uint16 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t   status = SUCCESS;
  hidRptMap_t *pRpt;
  AT_LOG("%s\n",__func__);

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // Only report map is long
  if ( offset > 0 && uuid != REPORT_MAP_UUID )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( uuid == REPORT_UUID ||
       uuid == BOOT_KEY_INPUT_UUID ||
       uuid == BOOT_KEY_OUTPUT_UUID ||
       uuid == BOOT_MOUSE_INPUT_UUID )
  {
    // find report ID in table
    if ( (pRpt = hidDevRptByHandle(pAttr->handle)) != NULL )
    {
      // execute report callback
      status  = (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
                                        HID_DEV_OPER_READ, pLen, pValue );
    }
    else
    {
      *pLen = 0;
    }
  }
  else if ( uuid == REPORT_MAP_UUID )
  {
    // verify offset
    if ( offset >= hidReportMapLen )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // determine read length
      *pLen = MIN( maxLen, (hidReportMapLen - offset) );

      // copy data
      osal_memcpy( pValue, pAttr->pValue + offset, *pLen );
    }
  }
  else if ( uuid == HID_INFORMATION_UUID )
  {
    *pLen = HID_INFORMATION_LEN;
    osal_memcpy( pValue, pAttr->pValue, HID_INFORMATION_LEN );
  }
  else if ( uuid == GATT_REPORT_REF_UUID )
  {
    *pLen = HID_REPORT_REF_LEN;
    osal_memcpy( pValue, pAttr->pValue, HID_REPORT_REF_LEN );
  }
  else if ( uuid == PROTOCOL_MODE_UUID )
  {
    *pLen = HID_PROTOCOL_MODE_LEN;
    pValue[0] = pAttr->pValue[0];
  }
  else if ( uuid == GATT_EXT_REPORT_REF_UUID )
  {
    *pLen = HID_EXT_REPORT_REF_LEN;
    osal_memcpy( pValue, pAttr->pValue, HID_EXT_REPORT_REF_LEN );
  }

  // restart idle timer
  if ( status == SUCCESS )
  {
    hidDevStartIdleTimer();
  }

  return ( status );
}

/*********************************************************************
 * @fn      HidDev_WriteAttrCB
 *
 * @brief   HID Dev attribute read callback.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
bStatus_t HidDev_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                              uint8 *pValue, uint16 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  hidRptMap_t *pRpt;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  AT_LOG("%s,uuid 0x%X\n",__func__,uuid);

  if ( uuid == REPORT_UUID ||
       uuid == BOOT_KEY_OUTPUT_UUID )
  {
    // find report ID in table
    if ((pRpt = hidDevRptByHandle(pAttr->handle)) != NULL)
    {
      // execute report callback
      status  = (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
                                        HID_DEV_OPER_WRITE, &len, pValue );
    }
  }
  else if ( uuid == HID_CTRL_PT_UUID )
  {
    // Validate length and value range
    if ( len == 1 )
    {
      if ( pValue[0] == HID_CMD_SUSPEND ||  pValue[0] == HID_CMD_EXIT_SUSPEND )
      {
        // execute HID app event callback
        (*pHidDevCB->evtCB)( (pValue[0] == HID_CMD_SUSPEND) ?
                             HID_DEV_SUSPEND_EVT : HID_DEV_EXIT_SUSPEND_EVT );
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }
  else if ( uuid == GATT_CLIENT_CHAR_CFG_UUID )
  {
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY );
    if ( status == SUCCESS )
    {
      uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

      // find report ID in table
      if ( (pRpt = hidDevRptByCccdHandle(pAttr->handle)) != NULL )
      {
        // execute report callback
        (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
                                (charCfg == GATT_CLIENT_CFG_NOTIFY) ?
                                  HID_DEV_OPER_ENABLE : HID_DEV_OPER_DISABLE,
                                &len, pValue );
      }
    }
  }
  else if ( uuid == PROTOCOL_MODE_UUID )
  {
    if ( len == HID_PROTOCOL_MODE_LEN )
    {
      if ( pValue[0] == HID_PROTOCOL_MODE_BOOT ||
           pValue[0] == HID_PROTOCOL_MODE_REPORT )
      {
        pAttr->pValue[0] = pValue[0];

        // execute HID app event callback
        (*pHidDevCB->evtCB)( (pValue[0] == HID_PROTOCOL_MODE_BOOT) ?
                             HID_DEV_SET_BOOT_EVT : HID_DEV_SET_REPORT_EVT );
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    }
    else
    {
      status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }

  // restart idle timer
  if (status == SUCCESS)
  {
    hidDevStartIdleTimer();
  }

  return ( status );
}

/*********************************************************************
 * @fn      hidDev_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
//static void hidDev_ProcessOSALMsg( osal_event_hdr_t *pMsg )
//{
//  switch ( pMsg->event )
//  {
//  case GATT_MSG_EVENT:
//      hidDevProcessGattMsg( (gattMsgEvent_t *) pMsg );
//      break;
//
//  default:
//      break;
//  }
//}

/*********************************************************************
 * @fn      hidDevProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
//static void hidDevProcessGattMsg( gattMsgEvent_t *pMsg )
//{
//
//}

/*********************************************************************
 * @fn          hidDevHandleConnStatusCB
 *
 * @brief       Reset client char config.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
//static void hidDevHandleConnStatusCB( uint16 connHandle, uint8 changeType )
//{
//  uint8           i;
//  hidRptMap_t     *p = pHidDevRptTbl;
//  uint16          retHandle;
//  gattAttribute_t *pAttr;
//
//  // Make sure this is not loopback connection
//  if ( connHandle != LOOPBACK_CONNHANDLE )
//  {
//    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
//         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
//           ( !linkDB_Up( connHandle ) ) ) )
//    {
//      for ( i = hidDevRptTblLen; i > 0; i--, p++ )
//      {
//        if ( p->cccdHandle != 0 )
//        {
//          if ( (pAttr = GATT_FindHandle(p->cccdHandle, &retHandle)) != NULL )
//          {
//            GATTServApp_InitCharCfg( connHandle, (gattCharCfg_t *) pAttr->pValue );
//          }
//        }
//      }
//    }
//  }
//}

/*********************************************************************
 * @fn      hidDevDisconnected
 *
 * @brief   Handle disconnect.
 *
 * @return  none
 */
//static void hidDevDisconnected( void )
//{
//  // Stop idle timer
//  hidDevStopIdleTimer();
//
//  // Reset client characteristic configuration descriptors
//  Batt_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );
//  //ScanParam_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );
//  hidDevHandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );
//
//  // Reset state variables
//  hidDevConnSecure = FALSE;
//  hidProtocolMode = HID_PROTOCOL_MODE_REPORT;
//  hidDevPairingStarted = FALSE;
//
//  // Reset last report sent out
//  osal_memset( &lastNoti, 0, sizeof( attHandleValueNoti_t ) );
//
//  // if bonded and normally connectable start advertising
//  if ( ( hidDevBondCount() > 0 ) &&
//       ( pHidDevCfg->hidFlags & HID_FLAGS_NORMALLY_CONNECTABLE ) )
//  {
//    hidDevLowAdvertising();
//  }
//}

/*********************************************************************
 * @fn      hidDevPairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
//void hidDevPairStateCB( uint16 connHandle, uint8 state, uint8 status )
//{
//  if ( state == GAPBOND_PAIRING_STATE_STARTED )
//  {
//    hidDevPairingStarted = TRUE;
//  }
//  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
//  {
//    hidDevPairingStarted = FALSE;
//
//    if ( status == SUCCESS )
//    {
//      hidDevConnSecure = TRUE;
//    }
//
//    pairingStatus = status;
//  }
//  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
//  {
//    if ( status == SUCCESS )
//    {
//      hidDevConnSecure = TRUE;
//    }
//  }
//  //if(hidDevConnSecure){
//  //  osal_start_reload_timer(hidDevTaskId, HID_TEST_EVT, 1000);
//  //}
//  if ( !reportQEmpty() && hidDevConnSecure )
//  {
//    // Notify our task to send out pending reports
//    osal_set_event( hidDevTaskId, HID_SEND_REPORT_EVT );
//  }
//}

/*********************************************************************
 * @fn      hidDevPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - address of device to pair with, and could be either public or random.
 * @param   connectionHandle - connection handle
 * @param   uiInputs - pairing User Interface Inputs - Ask user to input passcode
 * @param   uiOutputs - pairing User Interface Outputs - Display passcode
 *
 * @return  none
 */
//void hidDevPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
//                                        uint8 uiInputs, uint8 uiOutputs )
//{
//  if ( pHidDevCB && pHidDevCB->passcodeCB )
//  {
//    // execute HID app passcode callback
//    (*pHidDevCB->passcodeCB)( deviceAddr, connectionHandle, uiInputs, uiOutputs );
//  }
//  else
//  {
//    // Send passcode response
//    GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, 0 );
//  }
//}


/*********************************************************************
 * @fn      hidDevBattPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
//static void hidDevBattPeriodicTask( void )
//{
//  if ( hidDevGapState == GAPROLE_CONNECTED )
//  {
//    // perform battery level check
//    Batt_MeasLevel( );
//
//    // Restart timer
//    osal_start_timerEx( hidDevTaskId, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
//  }
//}

/*********************************************************************
 * @fn      hidDevRptByHandle
 *
 * @brief   Find the HID report structure for the given handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *hidDevRptByHandle( uint16 handle )
{
  uint8       i;
  hidRptMap_t *p = pHidDevRptTbl;

  for ( i = hidDevRptTblLen; i > 0; i--, p++ )
  {
    if ( p->handle == handle && p->mode == hidProtocolMode)
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      hidDevRptByCccdHandle
 *
 * @brief   Find the HID report structure for the given CCC handle.
 *
 * @param   handle - ATT handle
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *hidDevRptByCccdHandle( uint16 handle )
{
  uint8       i;
  hidRptMap_t *p = pHidDevRptTbl;

  for ( i = hidDevRptTblLen; i > 0; i--, p++ )
  {
    if ( p->cccdHandle == handle)
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      hidDevRptById
 *
 * @brief   Find the HID report structure for the Report ID and type.
 *
 * @param   id - HID report ID
 * @param   type - HID report type
 *
 * @return  Pointer to HID report structure
 */
static hidRptMap_t *hidDevRptById( uint8 id, uint8 type )
{
  uint8       i;
  hidRptMap_t *p = pHidDevRptTbl;

  for ( i = hidDevRptTblLen; i > 0; i--, p++ )
  {
    if ( p->id == id && p->type == type && p->mode == hidProtocolMode )
    {
      return p;
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      hidDevSendReport
 *
 * @brief   Send a HID report.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void hidDevSendReport( uint16 connHandle,uint8 id, uint8 type, uint8 len, uint8 *pData )
{
  hidRptMap_t           *pRpt;
  gattAttribute_t       *pAttr;
  uint16                retHandle;

  AT_LOG("%s\n",__func__);
  // Get ATT handle for report
  if ( (pRpt = hidDevRptById(id, type)) != NULL )
  {
    // if notifications are enabled
    if ( (pAttr = GATT_FindHandle(pRpt->cccdHandle, &retHandle)) != NULL )
    {
      uint16 value;

      value  = GATTServApp_ReadCharCfg( connHandle, (gattCharCfg_t *) pAttr->pValue );
      if ( value & GATT_CLIENT_CFG_NOTIFY )
      {
        // After service discovery and encryption, the HID Device should request to
        // change to the preferred connection parameters that best suit its use case.
        if ( updateConnParams )
        {
//          GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &updateConnParams );
          updateConnParams = FALSE;
        }

        // send notification
        lastNoti[connHandle].handle = pRpt->handle;
        lastNoti[connHandle].len = len;
        osal_memcpy(lastNoti[connHandle].value, pData, len);

        GATT_Notification( connHandle, &lastNoti[connHandle], FALSE );
		AT_LOG("GATT_Notification \n");
        // start idle timer
        hidDevStartIdleTimer();
      }
    }
  }
}

/*********************************************************************
 * @fn      hidDevEnqueueReport
 *
 * @brief   Enqueue a HID report to be sent later.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
static void hidDevEnqueueReport( uint16 connHandle,uint8 id, uint8 type, uint8 len, uint8 *pData )
{
  // Enqueue only if bonded
  if ( hidDevBondCount() > 0 )
  {
    // Update last index
    lastQIdx[connHandle] = ( lastQIdx[connHandle] + 1 ) % HID_DEV_REPORT_Q_SIZE;

    if ( lastQIdx[connHandle] == firstQIdx[connHandle] )
    {
      // Queue overflow; discard oldest report
      firstQIdx[connHandle] = ( firstQIdx[connHandle] + 1 ) % HID_DEV_REPORT_Q_SIZE;
    }

    // Save report
    hidDevReportQ[connHandle][lastQIdx[connHandle]].id = id;
    hidDevReportQ[connHandle][lastQIdx[connHandle]].type = type;
    hidDevReportQ[connHandle][lastQIdx[connHandle]].len = len;
    osal_memcpy( hidDevReportQ[connHandle][lastQIdx[connHandle]].data, pData, len );

    if ( g_MRLink[connHandle].ConnSecure )
    {
      // Notify our task to send out pending reports
      osal_set_event( multiRole_TaskId, MULTIROLE_HID_SEND_REPORT_EVT );
    }
  }
}

/*********************************************************************
 * @fn      hidDevDequeueReport
 *
 * @brief   Dequeue a HID report to be sent out.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  None.
 */
//static hidDevReport_t *hidDevDequeueReport( uint16 connHandle )
//{
//  if ( reportQEmpty(connHandle) )
//  {
//    return NULL;
//  }

//  // Update first index
//  firstQIdx[connHandle] = ( firstQIdx[connHandle] + 1 ) % HID_DEV_REPORT_Q_SIZE;

//  return ( &(hidDevReportQ[connHandle][firstQIdx[connHandle]]) );
//}

/*********************************************************************
 * @fn      hidDevBondCount
 *
 * @brief   Gets the total number of bonded devices.
 *
 * @param   None.
 *
 * @return  number of bonded devices.
 */
static uint8 hidDevBondCount( void )
{
  uint8 bondCnt = 0;

  VOID GAPBondMgr_GetParameter( GAPBOND_BOND_COUNT, &bondCnt );

  return ( bondCnt );
}
/*********************************************************************
*********************************************************************/

