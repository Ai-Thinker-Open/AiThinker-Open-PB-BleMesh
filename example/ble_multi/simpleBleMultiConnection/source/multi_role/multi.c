/******************************************************************************

 @file       multi.c

 @brief multi GAPRole profile code

 Group: CMCU, SCS
 Target Device: PHY6212

 ******************************************************************************/
 
/*********************************************************************
* INCLUDES
*/
#include <string.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "hci_tl.h"
#include "l2cap.h"
#include "gap.h"
#include "linkdb.h"
#include "hal_mcu.h"
#include "gatt.h"
#include "osal_snv.h"
#include "gapbondmgr.h"

/* This Header file contains all BLE API and icall structure definition */
#include "multi.h"
#include "multi_role.h"
#include "log.h"
#include "flash.h"
/*********************************************************************
* MACROS
*/
#define MULTI_ROLE_CENTRAL_HANDLER		0xFE

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES            20

// service discovery process default delay
#define DEFAULT_SDP_DELAY				1500

// default advertising,scanning,linking scheduling
#define DEFAULT_ASL_EVT_INTV			10

// connection interval
#define DEFAULT_MIN_CONN_INTERVAL     0x0006  // 7.5 milliseconds
#define DEFAULT_MAX_CONN_INTERVAL     0x0C80  // 4 seconds

// Length of bd addr as a string
#define B_ADDR_STR_LEN                  15

// default Exchange MTU SIZE
#define DEFAULT_EXCHANGE_MTU_LEN		120

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

/*********************************************************************
* GLOBAL VARIABLES
*/

// Link DB maximum number of connections
uint8_t linkDBNumConns = MAX_NUM_LL_CONN;      // hardcoded, 

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/
extern MultiRoleApp_Link_t g_MRLink[MAX_CONNECTION_NUM];

/*********************************************************************
* LOCAL VARIABLES
*/
static uint8 gapMultiRole_TaskID;   // Task ID for internal task/event processing

//static multiRole_states_t gapMultiRole_state;

/*********************************************************************
* Profile Parameters - reference GAPROLE_PROFILE_PARAMETERS for
* descriptions
*/
static uint8_t  gapMultiRoleMaxScanRes = DEFAULT_MAX_SCAN_RES;

// multi-role common info
static GAPMultiRole_Param_t g_multiRoleParam;

// multi-role as peripheral Info
// max support DEFAULT_SLAVE_CNT slave 
static GAPMultiRole_Peripheral_t g_MultiPeriInfo[MAX_CONNECTION_SLAVE_NUM];

// multi-role as Central Info
static GAPMultiRole_Central_t g_MultiCentrInfo[MAX_CONNECTION_MASTER_NUM];

// multi-role link state 
static GAPMultiRole_LinkCtrl_t g_multiLinkInfo[MAX_CONNECTION_NUM];

// Application callbacks
static gapMultiRolesCBs_t *pGapRoles_AppCGs = NULL;

//multi-Role State-Machine
static GAPMultiRole_StateMachine_t g_MultiRoleSM;

// Number of scan results and scan result index
static uint8 g_MRScanRes;
//static uint8 g_MRScanIdx;

// Scan result list
static MultiRoleADV_ScanData g_MRSDevList[DEFAULT_MAX_SCAN_RES];

// parameter update no success actions
static uint8 paramUpdateNoSuccessOption = MULTIROLE_NO_ACTION;

/*********************************************************************
* Profile Attributes - variables
*/

/*********************************************************************
* Profile Attributes - Table
*/

/*********************************************************************
* LOCAL FUNCTIONS
*/
static uint8 MultiRole_Advertising(void);
static uint8 MultiRole_Scanning(void);
static uint8 MultiRole_Connecting(void);
static uint8 MultiRole_CancelConn(void);
static uint8 MultiRole_SM_ReScheduler(void);
static void  MultiRole_UpdateADVData(void);
static uint8 MultiRole_MSTCheckConnIdx(uint16 connHandle );
static uint8 MultiRole_MstGetUsedConnIdx(void);
static uint8 MultiRole_MSTMatchConnIdx(uint8 connIdx );
static void  MultiRole_ProcessLinkInfo(gapEstLinkReqEvent_t *pPkt);
static void  MultiRole_ProTerminateLinkInfo(gapTerminateLinkEvent_t *pPkt);
static uint8 MultiRole_SLVPrepareParamUpdate(void);
static void  MultiRole_SLVStartParamUpdate(void);
static void  MultiRole_ProcessParamUpdateInfo(gapLinkUpdateEvent_t *pPkt);
static void  MultiRole_HandleParamUpdateNoSuccess( void );

static void  MultiRole_ProcessASLEvent(void);
static void  MultiRole_AddDeviceInfo( gapDeviceInfoEvent_t *pPkt );
static void  MultiRole_AnalysisADVDATA(uint8 Index,gapDeviceInfoEvent_t *pData);
static void  MultiRole_PrepareSDP(void);
static void  MultiRole_PrepareSDPInfo(gattMsgEvent_t *pMsg);
static void MultiRole_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static uint8_t MultiRole_processGAPMsg(gapEventHdr_t *pMsg);
static void multiRoleProcessGATTMsg( gattMsgEvent_t *pMsg );
static void MultiRole_SetupGAP(uint8_t *numConns);
//static void MultiRole_startConnUpdate( uint8 handleFailure );
static void multiRolePasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                               uint8 uiInputs, uint8 uiOutputs );
static void multiRolePairStateCB( uint16 connHandle, uint8 state, uint8 status );

// Bond Manager Callbacks
static const gapBondCBs_t multiRoleBondCB =
{
  multiRolePasscodeCB,
  multiRolePairStateCB
};
/*********************************************************************
* @brief   Set a GAP Role parameter.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPMultiRole_SetParameter(uint16_t param, uint8_t len, void *pValue, uint8_t Handler)
{
	bStatus_t ret = SUCCESS;
	switch (param)
	{
		case GAPROLE_PROFILEROLE:
		{
			if (len == sizeof(uint8_t))
			{
				g_multiRoleParam.profileRole = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;
		case GAPROLE_IRK:
		{
			if (len == KEYLEN)
			{
				VOID memcpy(g_multiRoleParam.IRK, pValue, KEYLEN);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_SRK:
		{
			if (len == KEYLEN)
			{
				VOID memcpy(g_multiRoleParam.SRK, pValue, KEYLEN);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_SIGNCOUNTER:
		{
			if (len == sizeof (uint32_t))
			{
				g_multiRoleParam.signCounter = *((uint32_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADVERT_ENABLED:  //connectable advertising
		{
			if ( (len == sizeof(uint8_t)) && (( Handler < MAX_CONNECTION_SLAVE_NUM ) ))
			{
				g_MultiPeriInfo[Handler].AdvEnabled = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADV_NONCONN_ENABLED:
		{
			if ( (len == sizeof(uint8_t)) && (( Handler < MAX_CONNECTION_SLAVE_NUM ) ))
			{
				g_MultiPeriInfo[Handler].AdvNonConnEnabled = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADVERT_OFF_TIME:
		{
			if ( (len == sizeof (uint16_t)) && ( Handler < MAX_CONNECTION_SLAVE_NUM )  )
			{
				g_MultiPeriInfo[Handler].advOffTime = *((uint16_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADVERT_DATA:
		{
			if ((len <= B_MAX_ADV_LEN) && ( Handler < MAX_CONNECTION_SLAVE_NUM ))
			{
				VOID memset(g_MultiPeriInfo[Handler].pAdvData, 0, B_MAX_ADV_LEN);
				VOID memcpy(g_MultiPeriInfo[Handler].pAdvData, pValue, len);
				g_MultiPeriInfo[Handler].AdvertDataLen = len;
				g_MultiPeriInfo[Handler].isAdvDataUpdate = FALSE;
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_SCAN_RSP_DATA:
		{
			if ((len <= B_MAX_ADV_LEN) && ( Handler < MAX_CONNECTION_SLAVE_NUM ))
			{
				VOID memset(g_MultiPeriInfo[Handler].pScanRspData, 0, B_MAX_ADV_LEN);
				VOID memcpy(g_MultiPeriInfo[Handler].pScanRspData, pValue, len);
				g_MultiPeriInfo[Handler].ScanRspDataLen = len;
				g_MultiPeriInfo[Handler].isSRspDataUpdate = FALSE;
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADV_EVENT_TYPE:
		{
			if ((len == sizeof (uint8_t)) && (*((uint8_t *)pValue) <= GAP_ADTYPE_ADV_LDC_DIRECT_IND) && \
				 ( Handler < MAX_CONNECTION_SLAVE_NUM ))
			{
				g_MultiPeriInfo[Handler].AdvEventType = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADV_DIRECT_TYPE:
		{
			if ((len == sizeof (uint8_t)) && (*((uint8_t *)pValue) <= ADDRTYPE_PRIVATE_RESOLVE) && \
				( Handler < MAX_CONNECTION_SLAVE_NUM ))
			{
				g_MultiPeriInfo[Handler].AdvDirectType = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADV_DIRECT_ADDR:
		{
			if ((len == B_ADDR_LEN) && ( Handler < MAX_CONNECTION_SLAVE_NUM ) )
			{
				VOID memcpy(g_MultiPeriInfo[Handler].AdvDirectAddr, pValue, B_ADDR_LEN);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADV_CHANNEL_MAP:
		{
			if ((len == sizeof (uint8_t)) && (*((uint8_t *)pValue) <= 0x07) && \
				( Handler < MAX_CONNECTION_SLAVE_NUM ) )
			{
				g_MultiPeriInfo[Handler].AdvChanMap = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;

		case GAPROLE_ADV_FILTER_POLICY:
		{
			if ((len == sizeof (uint8_t)) && (*((uint8_t *)pValue) <= GAP_FILTER_POLICY_WHITE) && \
				( Handler < MAX_CONNECTION_SLAVE_NUM ))
			{
				g_MultiPeriInfo[Handler].AdvFilterPolicy = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;
		case GAPROLE_PARAM_UPDATE_ENABLE:
			if ( len == sizeof (uint8_t) )
			{
				g_MultiPeriInfo[Handler].paramUpdateEnable = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}

		break;
		case GAPROLE_MIN_CONN_INTERVAL:
			if ( (len == sizeof (uint16_t)) && ( Handler < MAX_CONNECTION_SLAVE_NUM )  )
			{
				g_MultiPeriInfo[Handler].paramConnIntvMIN = *((uint16_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		break;
		case GAPROLE_MAX_CONN_INTERVAL:
			if ( (len == sizeof (uint16_t)) && ( Handler < MAX_CONNECTION_SLAVE_NUM )  )
			{
				g_MultiPeriInfo[Handler].paramConnIntvMAX = *((uint16_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		break;
		case GAPROLE_SLAVE_LATENCY:
			if ( (len == sizeof (uint16_t)) && ( Handler < MAX_CONNECTION_SLAVE_NUM )  )
			{
				g_MultiPeriInfo[Handler].paramLatency = *((uint16_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		break;
		case GAPROLE_TIMEOUT_MULTIPLIER:
			if ( (len == sizeof (uint16_t)) && ( Handler < MAX_CONNECTION_SLAVE_NUM )  )
			{
				g_MultiPeriInfo[Handler].paramTimeOut = *((uint16_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		break;
		case GAPROLE_MAX_SCAN_RES:
		{
			if (len == sizeof (uint8_t))
			{
				gapMultiRoleMaxScanRes = *((uint8_t *)pValue);
			}
			else
			{
				ret = bleInvalidRange;
			}
		}
		break;	
		default:
			// The param value isn't part of this profile, try the GAP.
			if ((param < TGAP_PARAMID_MAX) && (len == sizeof (uint16_t)))
			{
				ret = GAP_SetParamValue(param, *((uint16_t *)pValue));
			}
			else
			{
				ret = INVALIDPARAMETER;
			}
		break;
	}
	return (ret);
}

/*********************************************************************
* @brief   Get a GAP Role parameter.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPMultiRole_GetParameter(uint16_t param, void *pValue, uint8_t Handler)
{
	bStatus_t ret = SUCCESS;

	switch (param)
	{
		case GAPROLE_PROFILEROLE:
		    *((uint8_t *)pValue) = g_multiRoleParam.profileRole;
		break;

		case GAPROLE_IRK:
			VOID memcpy(pValue, g_multiRoleParam.IRK, KEYLEN);
		break;

		case GAPROLE_SRK:
			VOID memcpy(pValue, g_multiRoleParam.SRK, KEYLEN);
		break;

		case GAPROLE_SIGNCOUNTER:
		    *((uint32_t *)pValue) = g_multiRoleParam.signCounter;
		break;

		case GAPROLE_BD_ADDR:
			VOID memcpy(pValue, g_multiRoleParam.bdAddr, B_ADDR_LEN);
		break;

		case GAPROLE_ADVERT_ENABLED:
			*((uint8_t *)pValue) = g_MultiPeriInfo[Handler].AdvEnabled;
		break;

		case GAPROLE_ADV_NONCONN_ENABLED:
		    *((uint8_t *)pValue) = g_MultiPeriInfo[Handler].AdvNonConnEnabled;
		break;

		case GAPROLE_ADVERT_OFF_TIME:
		    *((uint16_t *)pValue) = g_MultiPeriInfo[Handler].advOffTime;
		break;

		case GAPROLE_ADVERT_DATA:
			VOID memcpy(pValue , g_MultiPeriInfo[Handler].pAdvData , g_MultiPeriInfo[Handler].AdvertDataLen);
		break;

		case GAPROLE_SCAN_RSP_DATA:
			VOID memcpy(pValue, g_MultiPeriInfo[Handler].pScanRspData, g_MultiPeriInfo[Handler].ScanRspDataLen);
		break;

		case GAPROLE_ADV_EVENT_TYPE:
			*((uint8_t *)pValue) = g_MultiPeriInfo[Handler].AdvEventType;
		break;

		case GAPROLE_ADV_DIRECT_TYPE:
			*((uint8_t *)pValue) = g_MultiPeriInfo[Handler].AdvDirectType;
		break;

		case GAPROLE_ADV_DIRECT_ADDR:
			VOID memcpy(pValue, g_MultiPeriInfo[Handler].AdvDirectAddr, B_ADDR_LEN);
		break;

		case GAPROLE_ADV_CHANNEL_MAP:
			*((uint8_t *)pValue) = g_MultiPeriInfo[Handler].AdvChanMap;
		break;

		case GAPROLE_ADV_FILTER_POLICY:
			*((uint8_t *)pValue) = g_MultiPeriInfo[Handler].AdvFilterPolicy;
		break;

		case GAPROLE_PARAM_UPDATE_ENABLE:
			*((uint8_t *)pValue) = g_MultiPeriInfo[Handler].paramUpdateEnable;
		break;
		case GAPROLE_MIN_CONN_INTERVAL:
			*((uint16_t *)pValue) = g_MultiPeriInfo[Handler].paramConnIntvMIN;
		break;
		case GAPROLE_MAX_CONN_INTERVAL:
			*((uint16_t *)pValue) = g_MultiPeriInfo[Handler].paramConnIntvMAX;
		break;
		case GAPROLE_SLAVE_LATENCY:
			*((uint16_t *)pValue) = g_MultiPeriInfo[Handler].paramLatency;
		break;
		case GAPROLE_TIMEOUT_MULTIPLIER:
			*((uint16_t *)pValue) = g_MultiPeriInfo[Handler].paramTimeOut;
		break;
		
		case GAPROLE_MAX_SCAN_RES:
			*((uint8_t *)pValue) = gapMultiRoleMaxScanRes;
		break;

		default:
		// The param value isn't part of this profile, try the GAP.
		if (param < TGAP_PARAMID_MAX)
		{
		  *((uint16_t *)pValue) = GAP_GetParamValue(param);
		}
		else
		{
		  ret = INVALIDPARAMETER;
		}
		break;
		}
	return (ret);
}

/*********************************************************************
* @brief   Does the device initialization.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPMultiRole_StartDevice(gapMultiRolesCBs_t *pAppCallbacks, uint8_t * numConns)
{
  // Clear all of the Application callbacks
  if (pAppCallbacks)
  {
    pGapRoles_AppCGs = pAppCallbacks;
  }

  // Start the GAP
  MultiRole_SetupGAP(numConns);

  return (SUCCESS);
}

/*********************************************************************
* @brief   Terminates the existing connection.
*
* Public function defined in peripheral.h.
*/
bStatus_t GAPMultiRole_TerminateConnection(uint16_t Handler)
{
  return (GAP_TerminateLinkReq(gapMultiRole_TaskID, Handler,
                               HCI_DISCONNECT_REMOTE_USER_TERM));
}



/*********************************************************************
* LOCAL FUNCTION PROTOTYPES
*/
/*********************************************************************
* @fn	   bdAddr2Str
*
* @brief   Convert Bluetooth address to string. Only needed when
*		   LCD display is used.
*
* @return  none
*/
char *bdAddr2Str( uint8 *pAddr )
{
  uint8 	  i;
  char		  hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char		  *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
	*pStr++ = hex[*--pAddr >> 4];
	*pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
* @fn      gapRole_init
*
* @brief   Initialization function for the GAP Role Task.
*
* @param   none
*
* @return  none
*/
void GAPMultiRole_Init(uint8 taskId)
{
	gapMultiRole_TaskID = taskId;
	
	memset(&g_multiRoleParam,0,sizeof(GAPMultiRole_Param_t));
	memset(g_MultiPeriInfo,0,sizeof(g_MultiPeriInfo));
	memset(g_MultiCentrInfo,0,sizeof(g_MultiCentrInfo));
	memset(&g_MultiRoleSM,0,sizeof(GAPMultiRole_StateMachine_t));	 
	
//	// Register to receive incoming ATT Indications/Notifications
//	GATT_RegisterForInd( gapMultiRole_TaskID );

	// Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &multiRoleBondCB );

	GAP_RegisterForHCIMsgs( taskId );
}

/**
* @brief   Establish a link to a peer device.
*
* Public function defined in central.h.
*/
bStatus_t GAPMultiRole_EstablishLink(uint8_t highDutyCycle, uint8_t whiteList,
                                uint8_t addrTypePeer, uint8_t *peerAddr)
{
  gapEstLinkReq_t params;

  params.taskID = gapMultiRole_TaskID;
  params.highDutyCycle = highDutyCycle;
  params.whiteList = whiteList;
  params.addrTypePeer = addrTypePeer;
  VOID memcpy(params.peerAddr, peerAddr, B_ADDR_LEN);

  return GAP_EstablishLinkReq(&params);
}

/**
* @brief   Start a device discovery scan.
*
* Public function defined in central.h.
*/
bStatus_t GAPMultiRole_StartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList)
{
  gapDevDiscReq_t params;

  params.taskID = gapMultiRole_TaskID;
  params.mode = mode;
  params.activeScan = activeScan;
  params.whiteList = whiteList;

  return GAP_DeviceDiscoveryRequest(&params);
}

/**
* @brief   Cancel a device discovery scan.
*
* Public function defined in central.h.
*/
bStatus_t GAPMultiRole_CancelDiscovery(void)
{
  return GAP_DeviceDiscoveryCancel(gapMultiRole_TaskID);
}

/*********************************************************************
* @fn      gapRole_taskFxn
*
* @brief   Task entry point for the GAP Peripheral Role.
*
* @param   a0 - first argument
* @param   a1 - second argument
*
* @return  none
*/
//static void gapRole_taskFxn(UArg a0, UArg a1)
extern uint8_t             llState;
extern uint8_t             llSecondaryState;
extern llGlobalStatistics_t g_pmCounters;  

uint16 GAPMultiRole_ProcessEvent( uint8 task_id, uint16 events )    
{
	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( gapMultiRole_TaskID )) != NULL )
		{
			MultiRole_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
			// Release the OSAL message
			VOID osal_msg_deallocate( pMsg );
		}
		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}
	if( events & UPDATE_ADV_DATA_EVT)
	{
		MultiRole_UpdateADVData();
		return ( events ^ UPDATE_ADV_DATA_EVT );
	}
	if (events & START_ADV_SCAN_INIT_EVT)
	{
		MultiRole_ProcessASLEvent();
		return (events ^ START_ADV_SCAN_INIT_EVT);
	}
	if( events & ADV_OFF_EVT )
	{
		uint8 ret = GAP_EndDiscoverable(gapMultiRole_TaskID);
		LOG("GAP_EndDiscoverable ret %d\n",ret);
		return ( events ^ ADV_OFF_EVT);
	}
	if( events & SLV_START_CONN_UPDATE_EVT )
	{
		MultiRole_SLVStartParamUpdate();
		return ( events ^ SLV_START_CONN_UPDATE_EVT );
	}
	if( events & SLV_CONN_PARAM_TIMEOUT_EVT )
	{
		MultiRole_HandleParamUpdateNoSuccess();
		return ( events ^ SLV_CONN_PARAM_TIMEOUT_EVT );
	}
	if( events & CONN_TIMEOUT_EVT )
	{
		MultiRole_CancelConn();
		return ( events ^ CONN_TIMEOUT_EVT );
	}
	if( events & MST_SDP_EVT )
	{
		if( 0 == osal_get_timeoutEx( gapMultiRole_TaskID , MST_SDP_EVT ) )
		{
			osal_start_timerEx(gapMultiRole_TaskID, MST_SDP_EVT, DEFAULT_SDP_DELAY );
		}
		MultiRole_PrepareSDP();
		return ( events ^ MST_SDP_EVT);
	}

	// Discard unknown events
	return 0;
}

/*********************************************************************
* @fn      gapRole_processStackMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
//static uint8_t gapRole_processStackMsg(ICall_Hdr *pMsg)
static void MultiRole_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
//  uint8_t safeToDealloc = TRUE;
  switch (pMsg->event)
  {
  case GAP_MSG_EVENT:
    MultiRole_processGAPMsg((gapEventHdr_t *)pMsg);
    break;

  case L2CAP_SIGNAL_EVENT:
    {
      
    }
    break;
  case GATT_MSG_EVENT:
		multiRoleProcessGATTMsg( (gattMsgEvent_t *) pMsg );
	break;

  default:
    break;
  }
}

/*********************************************************************
* @fn      gapRole_processGAPMsg
*
* @brief   Process an incoming task message.
*
* @param   pMsg - message to process
*
* @return  none
*/
static uint8_t MultiRole_processGAPMsg(gapEventHdr_t *pMsg)
{
	uint8_t notify = FALSE;   // State changed so notify the app? (default no)
	uint8 i;
	switch (pMsg->opcode)
	{
		// Device initialized
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
			bStatus_t stat = pPkt->hdr.status;
			if (stat == SUCCESS)
			{
				AT_LOG("Device Init Done \n");
				LOG("BLE Multi-Role Address:" );
				LOG( bdAddr2Str( pPkt->devAddr ) );
				LOG("\n");
				LOG("HCI_LE PKT LEN %d\n",pPkt->dataPktLen);
				LOG("HCI_LE NUM TOTAL %d\n",pPkt->numDataPkts);

				for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
				{
					if( (g_MultiPeriInfo[i].isAdvDataUpdate) && ( g_MultiPeriInfo[i].isSRspDataUpdate) )
					{
						// already Update Controller Advertising Data and Scan Response Data
					}
					else
					{
						osal_set_event( gapMultiRole_TaskID, UPDATE_ADV_DATA_EVT );
						break;
					}
				}
			}
			else
			{
				LOG("Device Init Done ERR_Code 0x%02X\n",stat);
			}
		}
		break;

		// Update advertising done
		case GAP_ADV_DATA_UPDATE_DONE_EVENT:
		{
			gapAdvDataUpdateEvent_t *pPkt = (gapAdvDataUpdateEvent_t *)pMsg;
			if (pPkt->hdr.status == SUCCESS)
			{
				if( pPkt->adType )
				{
					LOG("GAP adv Data update Done \n");
					osal_set_event(gapMultiRole_TaskID,UPDATE_ADV_DATA_EVT );

				}
				else
				{
					LOG("GAP adv Scan Response Data update Done \n");
					g_MultiRoleSM.lastState = GAPROLE_INIT;
					g_MultiRoleSM.currentState = GAPROLE_WAITING;
					MultiRole_SM_ReScheduler();
				}

			}
			if (pPkt->hdr.status != SUCCESS)
			{
				notify = TRUE;
			}
		}
		break;

		// Advertising started or ended
		case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
		{
			gapMakeDiscoverableRspEvent_t *pPkt = (gapMakeDiscoverableRspEvent_t *)pMsg;
			if( pPkt->hdr.status == SUCCESS)
			{
			for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
			{
				if( g_MultiPeriInfo[i].PeriState == GAPROLE_STARTED )
				{
					g_MultiPeriInfo[i].PeriState = GAPROLE_ADVERTISING;
//					LOG("GAP Make Discoverable Done idx=%d\n",i);
					break;
					}
				}
				LOG("GAP_MakeDiscoverable Done \n");
			}
			else
			{
				osal_stop_timerEx(gapMultiRole_TaskID, ADV_OFF_EVT);
				LOG("GAP_MakeDiscoverable err 0x%02X \n",pPkt->hdr.status);
				g_MultiRoleSM.lastState = g_MultiRoleSM.currentState;
				g_MultiRoleSM.currentState = GAPROLE_WAITING;
				MultiRole_SM_ReScheduler();
			}
		}
		break;
		case GAP_END_DISCOVERABLE_DONE_EVENT:
        {
//			gapEndDiscoverableRspEvent_t *pPkt = (gapEndDiscoverableRspEvent_t *)pMsg;
			for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
			{
				if( g_MultiPeriInfo[i].PeriState == GAPROLE_ADVERTISING )
				{
					g_MultiPeriInfo[i].PeriState = GAPROLE_INIT;
					break;
				}
			}
			g_MultiRoleSM.lastState = g_MultiRoleSM.currentState;
			g_MultiRoleSM.currentState = GAPROLE_WAITING;
			MultiRole_SM_ReScheduler();
//				LOG("GAP End Discoverable status 0x%02X \n",pPkt->hdr.status );
        }
		break;

		// Connection formed
		case GAP_LINK_ESTABLISHED_EVENT:
		{
			gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;
			osal_stop_timerEx(gapMultiRole_TaskID, CONN_TIMEOUT_EVT );
			g_MultiRoleSM.lastState = g_MultiRoleSM.currentState;
			g_MultiRoleSM.currentState = GAPROLE_WAITING;
			if (pPkt->hdr.status == SUCCESS)
			{
							MultiRole_ProcessLinkInfo(pPkt);
			}
			else
			{
				for(uint8 i=0;i< MAX_CONNECTION_MASTER_NUM ; i++)
				{
					if( g_MultiCentrInfo[i].CentState == GAPROLE_CONNECTING )
					{
						g_MultiCentrInfo[i].CentState = GAPROLE_INIT;
						LOG("CentState CHANGE I %d\n",i);
					}
				}
				LOG("Link Established Error %d\n",pPkt->hdr.status);
			}
			MultiRole_SM_ReScheduler();
		}
		break;

		// Connection terminated
		case GAP_LINK_TERMINATED_EVENT:
		{
			gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;
			if (pPkt->hdr.status == SUCCESS)
			{
				MultiRole_ProTerminateLinkInfo(pPkt);
				if( 0 == osal_get_timeoutEx(gapMultiRole_TaskID, CONN_TIMEOUT_EVT) )
					MultiRole_SM_ReScheduler();
			}
			else
			{
				LOG("Terminate errCode %d\n",pPkt->hdr.status);
			}
		}
		break;

		// Security request received from slave
		case GAP_SLAVE_REQUESTED_SECURITY_EVENT:
		{

		}
		break;

		// Connection parameter update
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
			MultiRole_ProcessParamUpdateInfo(pPkt);
		}
		break;


		case GAP_DEVICE_INFO_EVENT:
		{
			gapDeviceInfoEvent_t *pPkt = (gapDeviceInfoEvent_t *)pMsg;
			uint8 i=0;
			uint8 j=0;

			for( i=0;i < MAX_CONNECTION_MASTER_NUM ; i++ )
			{
				if( g_MultiCentrInfo[i].initFlag == TRUE )
				{
					j++;
				}
			}
			if( j==MAX_CONNECTION_MASTER_NUM)
				GAP_DeviceDiscoveryCancel(gapMultiRole_TaskID);
			
			MultiRole_AddDeviceInfo( pPkt);
		}
		break;

		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			gapDevDiscEvent_t *pPkt = ( gapDevDiscEvent_t *)pMsg;
			if( pPkt->hdr.status == SUCCESS )
				LOG("GAP Discovery success \n");
			else if( pPkt->hdr.status == bleGAPUserCanceled )
				LOG("GAP Discovery user cancel \n ");
			else
				LOG("GAP Discovery ret status 0x%x\n",pPkt->hdr.status);
				
				
			
			g_MultiRoleSM.lastState = g_MultiRoleSM.currentState;
			g_MultiRoleSM.currentState = GAPROLE_WAITING;
			MultiRole_SM_ReScheduler();
			LOG("Devices Foun %d\n",g_MRScanRes);
			pGapRoles_AppCGs->pfnScanDone( g_MRScanRes , g_MRSDevList );

		}
		break;
		default:
		break;
		} 

	if (notify == TRUE) // App needs to take further action
	{

	}
	return TRUE;
}


/*********************************************************************
 * @fn      multiRoleProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void multiRoleProcessGATTMsg( gattMsgEvent_t *pMsg )
{
	uint16 handle = pMsg->connHandle;
	switch( pMsg->method )
	{
		case ATT_EXCHANGE_MTU_RSP:
			if( pMsg->hdr.status == SUCCESS )
			{
				if( g_MultiCentrInfo[handle].MUT_ECState == MTUEC_STATE_ING )
					g_MultiCentrInfo[handle].MUT_ECState = MTUEC_STATE_DONE;
			}
			else
			{
				g_MultiCentrInfo[handle].MUT_ECState = MTUEC_STATE_IDLE;

			}
			LOG("connHand %d,exchangeMTU status %d,rxMTU %d\n",	pMsg->connHandle,\
																	pMsg->hdr.status,\
																	pMsg->msg.exchangeMTURsp.serverRxMTU);
		break;
		case ATT_READ_RSP:
			if( pMsg->hdr.status == SUCCESS )
			{
				LOG("Read Success Handle %d,len %d ",pMsg->connHandle,pMsg->msg.readRsp.len );
				for( uint8 i=0;i < pMsg->msg.readRsp.len;i++)
					LOG("0x%02X,",pMsg->msg.readRsp.value[i]);
				LOG("\n");
			}
		break;
		case ATT_WRITE_REQ:
			if( pMsg->hdr.status == SUCCESS )
			{
				LOG( "Write sent connHandle %d\n",pMsg->connHandle );	  
			}
		break;
		case ATT_HANDLE_VALUE_NOTI:
			if( pMsg->hdr.status == SUCCESS )
			{
				LOG("Notify success connHanle %d,len %d\n",pMsg->connHandle,pMsg->msg.handleValueNoti.len);
				if(pMsg->msg.handleValueNoti.len>20)
				{
					for(unsigned char i = 0;i < 10; i++)
					{
						LOG( "0x%02X,", pMsg->msg.handleValueNoti.value[i]);
					}
					LOG("\n");
				}
			}
		break;
		case ATT_READ_BY_GRP_TYPE_RSP:
		case ATT_READ_BY_TYPE_RSP:
			MultiRole_PrepareSDPInfo(pMsg);
		break;
		case ATT_ERROR_RSP:
			switch( pMsg->msg.errorRsp.reqOpcode )
			{
				case ATT_READ_REQ:
					LOG( "Read Error %d\n", pMsg->msg.errorRsp.errCode );
				break;
				case ATT_WRITE_REQ:
					LOG( "Write Error: %d\n", pMsg->msg.errorRsp.errCode );
				break;
				case ATT_HANDLE_VALUE_NOTI:
					LOG( "Notify Error: %d\n", pMsg->msg.errorRsp.errCode );
				break;
				default:
					LOG( "connHandle %d,ATT_ERROR_RSP %d\n", handle,pMsg->msg.errorRsp.reqOpcode );
				break;
			}
		break;
		default:
		break;
	}
}

/*********************************************************************
* @fn      MultiRole_SetupGAP
*
* @brief   Call the GAP Device Initialization function using the
*          Profile Parameters. Negotiate the maximum number
*          of simultaneous connections with the stack.
*
* @param   numConns - desired number of simultaneous connections
*
* @return  none
*/
static void MultiRole_SetupGAP(uint8_t * numConns)
{
	// Set number of possible simultaneous connections
	*numConns = linkDBNumConns;
	bStatus_t ret= GAP_DeviceInit(	gapMultiRole_TaskID, g_multiRoleParam.profileRole, gapMultiRoleMaxScanRes,
									 g_multiRoleParam.IRK, g_multiRoleParam.SRK,
									(uint32*)&g_multiRoleParam.signCounter);
}

/*********************************************************************
* @fn      gapRole_setEvent
*
* @brief   Set an event
*
* @param   event - event to be set
*
* @return  none
*/
void gapRole_setEvent(uint32_t event)
{
    osal_set_event(gapMultiRole_TaskID, event);
}


/********************************************************************
 * @fn          GAPRole_SendUpdateParam
 *
 * @brief       Update the parameters of an existing connection
 *
 * @param       minConnInterval - the new min connection interval
 * @param       maxConnInterval - the new max connection interval
 * @param       latency - the new slave latency
 * @param       connTimeout - the new timeout value
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again, or take no action
 *
 * @return      SUCCESS, bleNotConnected, or bleInvalidRange
 */
bStatus_t GAPMultiRole_SendUpdateParam( uint16 minConnInterval, uint16 maxConnInterval,
                                   uint16 latency, uint16 connTimeout, uint8 handleFailure )
{    

	return SUCCESS;
}

uint8 GAPMultiRole_addPeerAddr(uint8_t *pAddr,uint8 addrType,uint8 en_connect )
{
	uint8 i=0;

	for( i=0;i < MAX_CONNECTION_MASTER_NUM ; i++ )
	{
		if( g_MultiCentrInfo[i].initFlag == FALSE )
		{
			memcpy(g_MultiCentrInfo[i].peerAddr,pAddr,B_ADDR_LEN );
			g_MultiCentrInfo[i].peerAddrType = addrType;
			g_MultiCentrInfo[i].enConnected = en_connect;

			g_MultiCentrInfo[i].initFlag = TRUE;
			
			LOG(" add addr: %d, 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",i,\
													pAddr[0],pAddr[1],pAddr[2],pAddr[3],\
													pAddr[4],pAddr[5]);
			break;
		}
	}
	if( i == MAX_CONNECTION_MASTER_NUM )
		return FALSE;
	return TRUE;
}

uint8 GAPMultiRole_delPeerAddr(uint8_t *pAddr )
{
	uint8 i=0;
	// check pAddr in device list or not
	for( i=0;i < MAX_CONNECTION_MASTER_NUM ; i++ )
	{
		if( g_MultiCentrInfo[i].initFlag )
		{
			if( 0 == memcmp( pAddr, g_MultiCentrInfo[i].peerAddr , B_ADDR_LEN ))
			{
				g_MultiCentrInfo[i].initFlag = FALSE ;
//				memset( g_MultiCentrInfo[i].peerAddr , 0, B_ADDR_LEN );
				if( g_MultiCentrInfo[i].CentState == GAPROLE_CONNECTED )
				{
					for( uint8 j=0;j<MAX_CONNECTION_NUM;j++)
					{
						// find pAddr's connHandle
						if( g_multiLinkInfo[j].mstConnIdx == i )
						{
							GAPMultiRole_TerminateConnection(j);
							LOG("Delete addr ... %d,disconnect ... ",i);
							LOG( bdAddr2Str( &g_MultiCentrInfo[i].peerAddr[0])); 
							LOG("\n");
							break;
						}
					}
				}
				break;
			}
		}
	}
	if( i == MAX_CONNECTION_MASTER_NUM )
		return FALSE;
	return TRUE;
}

void GAPMultiRole_enConnPeerAddr(uint8_t *pAddr,uint8 en)
{
	uint8 i=0;
	// check pAddr in device list or not
	for( i=0;i < MAX_CONNECTION_MASTER_NUM ; i++ )
	{
		if( g_MultiCentrInfo[i].initFlag )
		{
			if( 0 == memcmp( pAddr, g_MultiCentrInfo[i].peerAddr , B_ADDR_LEN ))
			{
				g_MultiCentrInfo[i].enConnected = en;
			}
		}
	}
}


uint8  MultiRole_Advertising(void)
{
	uint8 i;

	LOG("Advertising ... \n");
	for(i=0;i < MAX_CONNECTION_SLAVE_NUM ; i++)
	{
		LOG("g_MultiPeriInfo[%d].PeriState %d\n",i,g_MultiPeriInfo[i].PeriState);
		if( g_MultiPeriInfo[i].PeriState == GAPROLE_INIT )
		{
			if( !g_MultiPeriInfo[i].isAdvDataUpdate || !g_MultiPeriInfo[i].isSRspDataUpdate ) 
			{
				osal_set_event(gapMultiRole_TaskID,UPDATE_ADV_DATA_EVT );
				return Idle_Role;
			}
			break;
		}
	}

	if( i >= MAX_CONNECTION_SLAVE_NUM )
		return Idle_Role;
	// If any type of advertising is enabled
	if ( g_MultiPeriInfo[i].AdvEnabled || g_MultiPeriInfo[i].AdvNonConnEnabled )
	{
		gapAdvertisingParams_t params;

		// Setup advertisement parameters
		if ( g_MultiPeriInfo[i].AdvNonConnEnabled )
		{
			// Only advertise non-connectable undirected.
			params.eventType = GAP_ADTYPE_ADV_NONCONN_IND;
		}
		else
		{
			// Connectable advertising
			params.eventType = g_MultiPeriInfo[i].AdvEventType ;
			params.initiatorAddrType = g_MultiPeriInfo[i].AdvDirectType;
			VOID memcpy(params.initiatorAddr, g_MultiPeriInfo[i].AdvDirectAddr, B_ADDR_LEN);
		}

		// Set advertising channel map
		params.channelMap = g_MultiPeriInfo[i].AdvChanMap;

		// Set advertising filter policy
		params.filterPolicy = g_MultiPeriInfo[i].AdvFilterPolicy;
		
		// Start advertising
		uint8 ret = GAP_MakeDiscoverable(gapMultiRole_TaskID, &params);
		if( ret == SUCCESS )
		{
			g_MultiPeriInfo[i].PeriState = GAPROLE_STARTED;
			g_MultiRoleSM.currentState = GAPROLE_ADVERTISING;
			LOG("g_MultiPeriInfo[%d].advOffTime %d\n",i,g_MultiPeriInfo[i].advOffTime);
			LOG("GAP_MakeDiscoverable success \n");
		}
		else
		{
			LOG("GAP_MakeDiscoverable Ret = 0x%02X\n",ret);
		}
	}
	return i;
}

void MultiRole_UpdateADVData(void)
{
	bStatus_t ret = SUCCESS;
	LOG("MultiRole_UpdateADVData \n");
	for(unsigned char i=0;i<MAX_CONNECTION_SLAVE_NUM;i++)
	{
		if( !g_MultiPeriInfo[i].isAdvDataUpdate || !g_MultiPeriInfo[i].isSRspDataUpdate ) 
		{
			if( !g_MultiPeriInfo[i].isAdvDataUpdate )
			{
				for(unsigned char j=0;j<g_MultiPeriInfo[i].AdvertDataLen;j++)
					LOG("0x%02X,",g_MultiPeriInfo[i].pAdvData[j]);
				LOG("\n");
				// Update the advertising data
		      	ret = GAP_UpdateAdvertisingData(	gapMultiRole_TaskID,\
		                                      		TRUE, \
		                                      		g_MultiPeriInfo[i].AdvertDataLen, \
		                                      		g_MultiPeriInfo[i].pAdvData);
				if( ret == SUCCESS  )
				{
					g_MultiPeriInfo[i].isAdvDataUpdate = TRUE;
					LOG("GAP Update Advertising Data success i=%d \n",i);
				}
				else
					LOG("GAP Update Advertising Data errorCode = %d\n",ret);
			}
			else if( !g_MultiPeriInfo[i].isSRspDataUpdate )
			{
				for(unsigned char j=0;j<g_MultiPeriInfo[i].ScanRspDataLen;j++)
					LOG("0x%02X,",g_MultiPeriInfo[i].pScanRspData[j]);
				LOG("\n");
				// Update the advertising scan Response data
		      	ret = GAP_UpdateAdvertisingData(	gapMultiRole_TaskID,\
		                                      		FALSE, \
		                                      		g_MultiPeriInfo[i].ScanRspDataLen, \
		                                      		g_MultiPeriInfo[i].pScanRspData);
				if( ret == SUCCESS  )
				{
					g_MultiPeriInfo[i].isSRspDataUpdate = TRUE;
					LOG("GAP Update Advertising ScanRsp Data successi=%d \n",i);
				}
				else
					LOG("GAP Update Advertising ScanRsp Data errorCode = %d\n",ret);
			}
			break;
		}
	}
}

static uint8 MultiRole_MstGetUsedConnIdx(void)
{
	uint8 i;
	for(i=0;i< MAX_CONNECTION_MASTER_NUM ; i++)
	{
		if( g_MultiCentrInfo[i].CentState == GAPROLE_CONNECTING )
			break;
	}
	return i;
}
static uint8 MultiRole_MSTMatchConnIdx(uint8 connIdx )
{
	uint8 i;
	for(i=0;i< MAX_CONNECTION_NUM ; i++)
	{
		if( g_multiLinkInfo[i].mstConnIdx == connIdx )
			break;
	}
	return i;
}

static uint8 MultiRole_MSTCheckConnIdx(uint16 connHandle )
{
	uint8 i = 0;
	for(i=0;i< MAX_CONNECTION_NUM ; i++)
	{
		if( g_multiLinkInfo[i].connectionHandle == connHandle )
			break;
	}
	return (g_multiLinkInfo[i].mstConnIdx) ;
}


static void MultiRole_ProcessLinkInfo(gapEstLinkReqEvent_t *pPkt)
{
	uint8 i = 0;
	uint8 tmpConnIdx = 0;
	GAPMultiRole_State_t RoleState = Master_Role;
	uint16 connHandle = pPkt->connectionHandle;
	g_MultiRoleSM.linkCnt++;
	
	g_multiLinkInfo[connHandle].connectionHandle = connHandle;
	g_multiLinkInfo[connHandle].connInterval = pPkt->connInterval;
	g_multiLinkInfo[connHandle].connLatency = pPkt->connLatency;
	g_multiLinkInfo[connHandle].peerDevAddrType = pPkt->devAddrType;
	memcpy(g_multiLinkInfo[connHandle].peerDevAddr,pPkt->devAddr,B_ADDR_LEN );

	for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
	{
		if( g_MultiPeriInfo[i].PeriState == GAPROLE_ADVERTISING )
		{
			RoleState = Slave_Role;
			break;
		}
	}
	g_multiLinkInfo[connHandle].RoleState = RoleState;
	AT_LOG("ESTABLISH LINK SUCCESS Handle %d,Intv %d,role %d \n",connHandle,pPkt->connInterval,g_multiLinkInfo[connHandle].RoleState);

	if( g_multiLinkInfo[connHandle].RoleState == Master_Role )
	{
		g_MultiRoleSM.masterRole_Cnt++;
		tmpConnIdx = MultiRole_MstGetUsedConnIdx();
		if( tmpConnIdx < MAX_CONNECTION_MASTER_NUM )
		{
			g_multiLinkInfo[connHandle].mstConnIdx = tmpConnIdx;
			g_MultiCentrInfo[tmpConnIdx].CentState = GAPROLE_CONNECTED;
			if( 0 == osal_get_timeoutEx( gapMultiRole_TaskID , MST_SDP_EVT ) )
			{
				osal_start_timerEx(gapMultiRole_TaskID, MST_SDP_EVT, DEFAULT_SDP_DELAY );
			}
			LOG("master \n");
		}
		else
			LOG("Allocate Connection Handle err\n");
		
//		GAPBondMgr_LinkEst( pPkt->devAddrType, pPkt->devAddr, pPkt->connectionHandle, GAP_PROFILE_CENTRAL );
	}
	else
	{
		LOG("slave \n");
		g_MultiRoleSM.slaveRole_Cnt++;
		g_MultiPeriInfo[i].advHandle = connHandle;
		g_MultiPeriInfo[i].PeriState = GAPROLE_CONNECTED;

		if( 0 == osal_get_timeoutEx( gapMultiRole_TaskID , ADV_OFF_EVT ) )
		{
			osal_stop_timerEx(gapMultiRole_TaskID, ADV_OFF_EVT);
		}
        GAPBondMgr_LinkEst( pPkt->devAddrType, pPkt->devAddr, pPkt->connectionHandle, GAP_PROFILE_PERIPHERAL );
	}
	pGapRoles_AppCGs->pfnEstablish(	connHandle,g_multiLinkInfo[connHandle].RoleState,\
									GAPROLE_CONNECTED,pPkt->devAddr);

}


static void MultiRole_ProTerminateLinkInfo(gapTerminateLinkEvent_t *pPkt)
{
	uint8 i = 0;
	uint8 tmpConnIdx = 0;
	uint16 connHandle = pPkt->connectionHandle;
	AT_LOG("TERMINATE LINK Handle %d,reason %d,role %d\n",connHandle,pPkt->reason,g_multiLinkInfo[connHandle].RoleState);
	g_MultiRoleSM.linkCnt--;
	
	if( g_multiLinkInfo[connHandle].RoleState == Slave_Role )
	{
		g_MultiRoleSM.slaveRole_Cnt--;
		for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
		{
			if( g_MultiPeriInfo[i].advHandle == pPkt->connectionHandle )
			{
				g_MultiPeriInfo[i].isAdvDataUpdate = FALSE;
				g_MultiPeriInfo[i].isSRspDataUpdate = FALSE;
//				g_MultiPeriInfo[i].paramUpdateEnable = FALSE;
				if( g_MultiPeriInfo[i].paramUpdateState != STATE_IDLE )
				{
					g_MultiPeriInfo[i].paramUpdateState = STATE_IDLE;
				}
				if( osal_get_timeoutEx( gapMultiRole_TaskID , SLV_START_CONN_UPDATE_EVT ) > 0 )
					osal_stop_timerEx( gapMultiRole_TaskID , SLV_START_CONN_UPDATE_EVT );
				if( osal_get_timeoutEx( gapMultiRole_TaskID , SLV_CONN_PARAM_TIMEOUT_EVT ) > 0 )
					osal_stop_timerEx( gapMultiRole_TaskID , SLV_CONN_PARAM_TIMEOUT_EVT );
				break;
			}
		}
		g_MultiPeriInfo[i].PeriState = GAPROLE_INIT;
	}
	else
	{
		g_MultiRoleSM.masterRole_Cnt--;
		tmpConnIdx = MultiRole_MSTCheckConnIdx( connHandle );
		if( tmpConnIdx < MAX_CONNECTION_MASTER_NUM )
		{
			// reset central state after terminate connection
			g_MultiCentrInfo[tmpConnIdx].CentState =  GAPROLE_INIT;
			g_MultiCentrInfo[tmpConnIdx].SDPState = DISC_STATE_IDLE;
			g_MultiCentrInfo[tmpConnIdx].MUT_ECState = MTUEC_STATE_IDLE;
			// clear service information saved in master
			memset(&g_MultiCentrInfo[tmpConnIdx].service,0,sizeof(GattScanServer));
		}
		else
			LOG("Check Connection Handle err\n");
	}
	pGapRoles_AppCGs->pfnTerminate( connHandle,g_multiLinkInfo[connHandle].RoleState,\
									GAPROLE_TERMINATED,pPkt->reason );
	memset(&g_multiLinkInfo[connHandle],0,sizeof(GAPMultiRole_LinkCtrl_t));
	g_multiLinkInfo[connHandle].RoleState = Idle_Role;
	g_multiLinkInfo[connHandle].peerDevAddrType = 0xFF;
	g_multiLinkInfo[connHandle].connectionHandle = INVALID_CONNHANDLE;


}

void MultiRole_ProcessParamUpdateInfo(gapLinkUpdateEvent_t *pPkt)
{
	uint8 i;
	// Cancel connection param update timeout timer (if active)
	if( osal_get_timeoutEx( gapMultiRole_TaskID , SLV_CONN_PARAM_TIMEOUT_EVT ) > 0 )
	{
		VOID osal_stop_timerEx( gapMultiRole_TaskID, SLV_CONN_PARAM_TIMEOUT_EVT );
	}
	if ( pPkt->hdr.status == SUCCESS )
	{
		LOG("handle %d,Intv %d,latency %d,TO %d\n", 	pPkt->connectionHandle,\
														pPkt->connInterval,\
														pPkt->connLatency,\
														pPkt->connTimeout);
		if( g_multiLinkInfo[pPkt->connectionHandle].RoleState == Master_Role )
		{
//			LOG("Master param update ");
		}
		else
		{
//			LOG("Slave param update ");
			// check which slave role id paramter update procedure
			for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
			{
				if( ( g_MultiPeriInfo[i].advHandle == pPkt->connectionHandle ) && \
					( g_MultiPeriInfo[i].paramUpdateState == STATE_ING ))
				{
					g_MultiPeriInfo[i].paramUpdateState = STATE_DONE;
					break;
				}
			}
			// check there's pending parameter update procedure
			for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
			{
				if(	( g_MultiPeriInfo[i].PeriState == GAPROLE_CONNECTED ) && \
					( g_MultiPeriInfo[i].paramUpdateEnable == TRUE) && \
					( ( g_MultiPeriInfo[i].paramUpdateState == STATE_IDLE) || \
					( g_MultiPeriInfo[i].paramUpdateState == STATE_WAITING) ) )
				{
					if( TRUE == MultiRole_SLVPrepareParamUpdate() )
					{
						LOG("Slave pending param update ... \n");
						if( g_MultiPeriInfo[i].paramUpdateState == STATE_IDLE )
							g_MultiPeriInfo[i].paramUpdateState = STATE_WAITING;
					}
					break;
				}
				else
				{
					LOG("Slave pending param update err \n");
					LOG("g_MultiPeriInfo[%d].PeriState %d \n",i,g_MultiPeriInfo[i].PeriState);
					LOG("g_MultiPeriInfo[%d].paramUpdateEnable %d \n",i,g_MultiPeriInfo[i].paramUpdateEnable);
					LOG("g_MultiPeriInfo[%d].paramUpdateState %d \n",i,g_MultiPeriInfo[i].paramUpdateState);

				}
			}
		}
		g_multiLinkInfo[pPkt->connectionHandle].connInterval = pPkt->connInterval;
		g_multiLinkInfo[pPkt->connectionHandle].connLatency = pPkt->connLatency;
		g_multiLinkInfo[pPkt->connectionHandle].connTimeout = pPkt->connTimeout;
		
		
	}
	else
	{
		LOG("error status %d\n",pPkt->hdr.status);
	}
}

uint8 MultiRole_SLVPrepareParamUpdate(void)
{
	// Get the minimum time upon connection establishment before the 
	// peripheral can start a connection update procedure.
	uint16 timeout ;
	if( 0 == osal_get_timeoutEx( gapMultiRole_TaskID , SLV_START_CONN_UPDATE_EVT ) )
	{
		GAPMultiRole_GetParameter( TGAP_CONN_PAUSE_PERIPHERAL,&timeout,0 );
		uint8 ret = osal_start_timerEx( gapMultiRole_TaskID, SLV_START_CONN_UPDATE_EVT, timeout*100 );
		if(SUCCESS == ret )
		{
			return TRUE;
		}
		else
		{
			LOG("Slave request update parameter timer ERR %d\n",ret );
		}
	}
	return FALSE;
}

void MultiRole_SLVStartParamUpdate(void)
{
	uint8 i;
	for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
	{
		if( ( g_MultiPeriInfo[i].PeriState == GAPROLE_CONNECTED ) && \
			( g_MultiPeriInfo[i].paramUpdateEnable == TRUE) && \
			( g_MultiPeriInfo[i].paramUpdateState == STATE_WAITING))
		{
			break;
		}
	}
	if( i >= MAX_CONNECTION_SLAVE_NUM )
	{
		LOG("MultiRole_SLVStartParamUpdate i>2 \n");
		return;
	}
	g_MultiPeriInfo[i].paramUpdateState = STATE_ING;
	
	uint16 connIntv = g_MultiPeriInfo[i].paramConnIntvMIN;
	// First check the current connection parameters versus the configured parameters
	if ( ( connIntv < DEFAULT_MAX_CONN_INTERVAL) && ( connIntv > DEFAULT_MIN_CONN_INTERVAL )  )
	{
		l2capParamUpdateReq_t updateReq;
		uint16 timeout = GAP_GetParamValue( TGAP_CONN_PARAM_TIMEOUT );

		updateReq.intervalMin = g_MultiPeriInfo[i].paramConnIntvMIN;
		updateReq.intervalMax = g_MultiPeriInfo[i].paramConnIntvMAX;
		updateReq.slaveLatency = g_MultiPeriInfo[i].paramLatency;
		updateReq.timeoutMultiplier = g_MultiPeriInfo[i].paramTimeOut;
		L2CAP_ConnParamUpdateReq( g_MultiPeriInfo[i].advHandle , &updateReq, gapMultiRole_TaskID );

		paramUpdateNoSuccessOption = MULTIROLE_NO_ACTION;

		// Let's wait for L2CAP Connection Parameters Update Response
		VOID osal_start_timerEx( gapMultiRole_TaskID, SLV_CONN_PARAM_TIMEOUT_EVT, timeout );
	}
}

void MultiRole_HandleParamUpdateNoSuccess( void )
{
	uint8 i;
	for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
	{
		if( ( g_MultiPeriInfo[i].PeriState == GAPROLE_CONNECTED ) && \
			( g_MultiPeriInfo[i].paramUpdateEnable == TRUE) && \
			( g_MultiPeriInfo[i].paramUpdateState == STATE_ING))
		{
			break;
		}
	}
	g_MultiPeriInfo[i].paramUpdateState = STATE_IDLE;
	LOG("Slave param update no success handle %d\n",i);
	// See which option was choosen for unsuccessful updates
	switch ( paramUpdateNoSuccessOption )
	{
		case MULTIROLE_RESEND_PARAM_UPDATE:
			
		break;

		case MULTIROLE_TERMINATE_LINK:
			
		break;

		case MULTIROLE_NO_ACTION:
		// fall through
		default:
		//do nothing
		break;
	}
}


// multi-role Process ASL(Advertising + Scanning + Link ) Event
static void MultiRole_ProcessASLEvent(void)
{
	uint8 advIdx = 0xFF;
	if(	( g_MultiRoleSM.currentState == GAPROLE_WAITING ) && \
		( g_MultiRoleSM.lastState == GAPROLE_SCANING ) && \
		( g_MultiRoleSM.masterRole_Cnt < MAX_CONNECTION_MASTER_NUM ))
	{
		uint8 connRet = MultiRole_Connecting();
		if( connRet != SUCCESS )
		{
			g_MultiRoleSM.lastState = GAPROLE_INIT;
			MultiRole_SM_ReScheduler();

		}
	}
	else if(	( g_MultiRoleSM.currentState == GAPROLE_WAITING ) && \
				( (g_MultiRoleSM.lastState == GAPROLE_ADVERTISING)  ) && \
				( g_MultiRoleSM.masterRole_Cnt < MAX_CONNECTION_MASTER_NUM ))
	{
		MultiRole_Scanning();
	}
	else if( ( (g_MultiRoleSM.currentState == GAPROLE_INIT) || g_MultiRoleSM.currentState == GAPROLE_WAITING) && \
		(g_MultiRoleSM.slaveRole_Cnt < MAX_CONNECTION_SLAVE_NUM) )
	{
		advIdx = MultiRole_Advertising();
		if( advIdx < Idle_Role )
		{
			uint8 ret = osal_start_timerEx( gapMultiRole_TaskID, ADV_OFF_EVT, g_MultiPeriInfo[advIdx].advOffTime );	
			LOG("advIdx %d,advOffTime %d,ret=%d\n",advIdx,g_MultiPeriInfo[advIdx].advOffTime,ret);
		}
		else
		{
			// there's no adv role , shall start scanning case
			g_MultiRoleSM.lastState = GAPROLE_ADVERTISING;
			MultiRole_SM_ReScheduler();
		}
	}
	else
	{
		if(  g_MultiRoleSM.lastState == GAPROLE_INIT )
		{
			g_MultiRoleSM.lastState = GAPROLE_ADVERTISING;
		}
		else if( g_MultiRoleSM.lastState == GAPROLE_CONNECTING )
		{
			g_MultiRoleSM.lastState = GAPROLE_INIT ;
		}
		MultiRole_SM_ReScheduler();
	}
	

}

uint8 MultiRole_Scanning(void)
{
	LOG("Scan ... \n");
	uint8 ret = GAPMultiRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
											 DEFAULT_DISCOVERY_ACTIVE_SCAN, 	 // passive scan
											 DEFAULT_DISCOVERY_WHITE_LIST );
	if( ret == SUCCESS )
	{
		g_MRScanRes = 0;
		memset(g_MRSDevList,0,sizeof(g_MRSDevList));
		g_MultiRoleSM.currentState = GAPROLE_SCANING;
	}
	return SUCCESS;
}

uint8 MultiRole_Connecting(void)
{
	uint8 i;
	uint8 ret = FAILURE;
	for (i = 0; i <MAX_CONNECTION_MASTER_NUM ; i ++)
	{
		// if there's any not complete connecting , then return
		if( g_MultiCentrInfo[i].CentState == GAPROLE_CONNECTING )
			return ret;
	}
	{
		LOG("MultiRole_Connecting g_MRScanRes %d\n",g_MRScanRes);
		{
			for (i = 0; i <MAX_CONNECTION_MASTER_NUM ; i ++)
			{
				if( (g_MultiCentrInfo[i].enConnected == TRUE ) && ( g_MultiCentrInfo[i].CentState == GAPROLE_INIT ))
				{
//					if( 0 == memcmp(&g_MRSDevList[j].addr[0],g_MultiCentrInfo[i].peerAddr,B_ADDR_LEN ) )
					{
						ret = GAPMultiRole_EstablishLink(	DEFAULT_LINK_HIGH_DUTY_CYCLE,
															DEFAULT_LINK_WHITE_LIST,
															g_MultiCentrInfo[i].peerAddrType, 
															&g_MultiCentrInfo[i].peerAddr[0] ); 

						if( ret == SUCCESS )
						{
							g_MultiCentrInfo[i].CentState = GAPROLE_CONNECTING;
							g_MultiRoleSM.currentState = GAPROLE_CONNECTING;
							osal_start_timerEx(gapMultiRole_TaskID, CONN_TIMEOUT_EVT,3*1000 );
						}
						LOG( "Connecting : " );
						LOG( bdAddr2Str( &g_MultiCentrInfo[i].peerAddr[0])); 
						LOG("...ret=%d\n",ret);
						// clear g_MRScanRes 
						g_MRScanRes = 0;
						return ret;
					}
				}
				else
				{
					LOG(" Disable connected \n");
				}
			}
		}
	}
	return ret;
}

uint8 MultiRole_CancelConn(void)
{
	uint8 status = GAPMultiRole_TerminateConnection(GAP_CONNHANDLE_INIT);
	LOG("Connection Timeout terminate conn status %d\n",status);
//	if( status == SUCCESS )
//	{
//		g_MultiRoleSM.lastState = g_MultiRoleSM.currentState;
//		g_MultiRoleSM.currentState = GAPROLE_WAITING;
//	}
//	MultiRole_SM_ReScheduler();
	return SUCCESS;
}

uint8 MultiRole_SM_ReScheduler(void)
{
	uint8 ret = FALSE;
	if( g_MultiRoleSM.linkCnt < MAX_CONNECTION_NUM )
	{
		if( g_MultiRoleSM.currentState == GAPROLE_WAITING )
		{
//			osal_set_event(gapMultiRole_TaskID,START_ADV_SCAN_INIT_EVT );
			osal_start_timerEx(gapMultiRole_TaskID,START_ADV_SCAN_INIT_EVT ,DEFAULT_ASL_EVT_INTV);
			ret = TRUE;
		}
		else
		{
			ret = FALSE;
		}
	}
	return ret;
}

void MultiRole_AddDeviceInfo( gapDeviceInfoEvent_t *pPkt )
{
	uint8 i;  
	uint8_t ZeroBuf[B_ADDR_LEN]={0,0,0,0,0,0};
	uint8_t Addr_Index=0;

    if(	( pPkt->opcode != GAP_DEVICE_INFO_EVENT ) || ( g_MRScanRes >= DEFAULT_MAX_SCAN_RES))
		return;
	// retrieve the ADDR Index
	for(i=0;i < DEFAULT_MAX_SCAN_RES;i++)
	{
		if( ( osal_memcmp( g_MRSDevList[i].addr, ZeroBuf , B_ADDR_LEN ) ) || \
			( osal_memcmp( pPkt->addr, g_MRSDevList[i].addr , B_ADDR_LEN )))
		{
			Addr_Index = i;
			break;
		}
	}
	if( Addr_Index < g_MRScanRes)
	{
		// update g_MRSDevList[Addr_Index]
		g_MRSDevList[Addr_Index].rssi = pPkt->rssi;
		if( !g_MRSDevList[Addr_Index].RSP_ReadFlag )
		{
			if( pPkt->eventType == GAP_ADRPT_SCAN_RSP )
			{
				// Analysis Scan RSP data
				LOG("scan rsp data \n");
				MultiRole_AnalysisADVDATA(Addr_Index, pPkt);
				g_MRSDevList[Addr_Index].RSP_ReadFlag = TRUE;
			}
		}
	}
	else
	{
		// first commit g_MRSDevList[Addr_Index]
		g_MRSDevList[Addr_Index].rssi = pPkt->rssi;
		// Mac Address Type
		g_MRSDevList[Addr_Index].AddrType = pPkt->addrType;
		// Mac Address
		osal_memcpy( g_MRSDevList[Addr_Index].addr, pPkt->addr, B_ADDR_LEN );
//		AT_LOG("g_MRSDevList[Addr_Index].AddrType %d\n",g_MRSDevList[Addr_Index].AddrType);
		if( pPkt->eventType == GAP_ADRPT_ADV_IND )
		{
			// Analysis Adv data
			LOG("GAP_ADRPT_ADV_IND \n");
			MultiRole_AnalysisADVDATA(Addr_Index, pPkt);
		}
		// Increment scan result count
        g_MRScanRes++;
	}
	pGapRoles_AppCGs->pfnEachScan( pPkt );
}

void MultiRole_AnalysisADVDATA(uint8 Index,gapDeviceInfoEvent_t *pData)
{
	int8 DataLength;
	int8 New_ADStructIndex = 0;
	int8 AD_Length = 0;
	uint8_t AD_Type;
	
	DataLength = pData->dataLen;
	while(DataLength)
	{
		New_ADStructIndex += AD_Length;
		// DATA FORMAT : Length + AD Type + AD Data
		AD_Length = pData->pEvtData[pData->dataLen - DataLength];
		AD_Type = pData->pEvtData[New_ADStructIndex+1];

		// if the remaining broadcast data length does not meet the broadcase format , 
		// it will not be parsed
		if( DataLength < ( AD_Length + 1 ) )
		{
			// DataLength : remain un-parsed data len
			// if parsed AD_Length plus 1 ( remain total adv data ) does not meet DataLength, 
			// then exit current cycle
			DataLength = 0;
			break;
		}
		else
		{
			if(AD_Length<2 || AD_Length>0x1f)
			{
				LOG("[AD_TYPE] ERR %02x %02x\n",AD_Type,AD_Length);
				break;
			}
			switch(AD_Type)
			{
				case GAP_ADTYPE_FLAGS:
					g_MRSDevList[Index].Flags = pData->pEvtData[New_ADStructIndex+2];
				break;
				case GAP_ADTYPE_16BIT_MORE:
				case GAP_ADTYPE_16BIT_COMPLETE:
				case GAP_ADTYPE_32BIT_MORE:
				case GAP_ADTYPE_32BIT_COMPLETE:
				case GAP_ADTYPE_128BIT_MORE:
				case GAP_ADTYPE_128BIT_COMPLETE: 
					g_MRSDevList[Index].UUID.Type = AD_Type;
					while( AD_Length - 1 - g_MRSDevList[Index].UUID.Len)
					{
						if( ( AD_Type == GAP_ADTYPE_16BIT_MORE) || ( AD_Type == GAP_ADTYPE_16BIT_COMPLETE))
						{
							g_MRSDevList[Index].UUID.Len += ATT_BT_UUID_SIZE;
						}
						else if( ( AD_Type == GAP_ADTYPE_32BIT_MORE) || ( AD_Type == GAP_ADTYPE_32BIT_COMPLETE))
						{
							g_MRSDevList[Index].UUID.Len += ATT_BT_UUID_SIZE<<1;
						}
						else
						{
							g_MRSDevList[Index].UUID.Len += ATT_UUID_SIZE;
						}
			
						if(AD_Length< 1 + g_MRSDevList[Index].UUID.Len)
						{
							LOG("[AD_TYPE] ERR %02x %02x\n",AD_Type,AD_Length);
							break;
						}
					}
					osal_memcpy(	g_MRSDevList[Index].UUID.Value,\
													&(pData->pEvtData[New_ADStructIndex+2]),\
													g_MRSDevList[Index].UUID.Len);
				break;
				case GAP_ADTYPE_LOCAL_NAME_SHORT:
				case GAP_ADTYPE_LOCAL_NAME_COMPLETE:
					g_MRSDevList[Index].LocalName.Type = AD_Type;
					g_MRSDevList[Index].LocalName.Length = AD_Length - 1;
					osal_memcpy(	g_MRSDevList[Index].LocalName.Value,&(pData->pEvtData[New_ADStructIndex+2]),\
												g_MRSDevList[Index].LocalName.Length);
				break;
				case GAP_ADTYPE_POWER_LEVEL:
					g_MRSDevList[Index].TxPower = pData->pEvtData[New_ADStructIndex+2];
				break;
				case GAP_ADTYPE_OOB_CLASS_OF_DEVICE:
				case GAP_ADTYPE_OOB_SIMPLE_PAIRING_HASHC:
				case GAP_ADTYPE_OOB_SIMPLE_PAIRING_RANDR:
					
				break;
				case GAP_ADTYPE_SM_TK:
					
				break;
				case GAP_ADTYPE_SM_OOB_FLAG:
					
				break;
				case GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE:
					g_MRSDevList[Index].ConnIntvRange.ConnMin = BUILD_UINT16(pData->pEvtData[New_ADStructIndex+2],\
																																					 pData->pEvtData[New_ADStructIndex+3]);
					g_MRSDevList[Index].ConnIntvRange.ConnMax = BUILD_UINT16(pData->pEvtData[New_ADStructIndex+4],\
																																					 pData->pEvtData[New_ADStructIndex+5]);
				break;
				case GAP_ADTYPE_SIGNED_DATA:
					
				break;
				case GAP_ADTYPE_SERVICES_LIST_16BIT:
				case GAP_ADTYPE_SERVICES_LIST_128BIT:
					g_MRSDevList[Index].ServiceSolic.Type = AD_Type;
					if( AD_Type ==	GAP_ADTYPE_SERVICES_LIST_16BIT)
						g_MRSDevList[Index].ServiceSolic.Len = ATT_BT_UUID_SIZE;
					else
						g_MRSDevList[Index].ServiceSolic.Len = ATT_UUID_SIZE;
					osal_memcpy(	g_MRSDevList[Index].ServiceSolic.Value,\
												&(pData->pEvtData[New_ADStructIndex+2]),\
												g_MRSDevList[Index].ServiceSolic.Len);
				break;
				case GAP_ADTYPE_SERVICE_DATA:
					
				break;
				case GAP_ADTYPE_APPEARANCE:
					
				break;
				case GAP_ADTYPE_MANUFACTURER_SPECIFIC:
					g_MRSDevList[Index].ManufactData.Length = AD_Length - 1;
					osal_memcpy(g_MRSDevList[Index].ManufactData.Value,&(pData->pEvtData[New_ADStructIndex+2]),\
												g_MRSDevList[Index].ManufactData.Length);
				break;
				default:
					break;
			}
			AD_Length++;
			DataLength -= AD_Length;
		}
	}
}

void MultiRole_PrepareSDP(void)
{
	for(uint8 i=0;i < MAX_CONNECTION_MASTER_NUM; i++)
	{
		if( g_MultiCentrInfo[i].CentState == GAPROLE_CONNECTED )
		{
			if( g_MultiCentrInfo[i].MUT_ECState == MTUEC_STATE_IDLE )
			{
				ATT_SetMTUSizeMax( DEFAULT_EXCHANGE_MTU_LEN );
				attExchangeMTUReq_t pReq;
				pReq.clientRxMTU = DEFAULT_EXCHANGE_MTU_LEN;
				uint8 status =GATT_ExchangeMTU(	MultiRole_MSTMatchConnIdx(i),\
												&pReq, gapMultiRole_TaskID );
				if( status == SUCCESS )
				{
					g_MultiCentrInfo[i].MUT_ECState = MTUEC_STATE_ING;
				}
				AT_LOG( "handle %d ,[MTU Req]%d %d\n",i,status,pReq.clientRxMTU);

			}
			else if( g_MultiCentrInfo[i].dleDone == FALSE )
			{
				HCI_LE_SetDataLengthCmd(i,251, 2120);
				AT_LOG("DLE %d\n",i);
				g_MultiCentrInfo[i].dleDone = TRUE;
			}
			else if( g_MultiCentrInfo[i].SDPState == DISC_STATE_IDLE ) 
			{
				// service discovery not Done and 
				// service discovery not in primservice or characteristic discovery
				memset( &g_MultiCentrInfo[i].service,0,sizeof(GattScanServer));
				bStatus_t ret = GATT_DiscAllPrimaryServices(MultiRole_MSTMatchConnIdx(i), \
															gapMultiRole_TaskID); 
				if( ret == SUCCESS  )
				{
					g_MultiCentrInfo[i].SDPState = DISC_STATE_SVC;
				}
				AT_LOG("GATT Discovery primary service handle %d, ret %d\n",i,ret);
			}
		}
	}

}

static void  MultiRole_PrepareSDPInfo(gattMsgEvent_t *pMsg)
{
	uint8 i=0;
	uint8 primServCnt;
	uint8 primCharIdx;
	uint8 charFindIdx;
	uint16 connHandle = pMsg->connHandle ;
	uint8 tmpMstIdx = MultiRole_MSTCheckConnIdx( connHandle );
	if( g_MultiCentrInfo[ tmpMstIdx ].SDPState == DISC_STATE_SVC )
	{
		switch ( pMsg->method )
		{
			case ATT_READ_BY_GRP_TYPE_RSP:
				if( pMsg->msg.readByGrpTypeRsp.numGrps > 0 )
				{
					for(i = 0 ; i < pMsg->msg.readByGrpTypeRsp.numGrps;i++)
					{
						primServCnt = g_MultiCentrInfo[ tmpMstIdx ].service.PrimSerCnt;
						// Current Attribute LEN
						g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primServCnt].Attr_Data_Len = pMsg->msg.readByGrpTypeRsp.len;
						// Current Attribute Handle
						g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primServCnt].Attr_StHandle = \
							BUILD_UINT16(	pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i], \
							pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 1]);
						// Current Attribute End Group Handle
    					g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primServCnt].Attr_EGHandle = \
    						BUILD_UINT16(	pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i+2], \
    						pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 3]);
						// Caculate Primary Service UUID Length
    					g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primServCnt].UUID_Len = \
    						pMsg->msg.readByGrpTypeRsp.len - 4;
						// Copy UUID
    					osal_memcpy(g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primServCnt].UUID,\
    									&(pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 4]),\
    									g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primServCnt].UUID_Len);
						g_MultiCentrInfo[ tmpMstIdx ].service.PrimSerCnt++;

					}
				}
				else if( pMsg->hdr.status == bleProcedureComplete )
				{
					g_MultiCentrInfo[ tmpMstIdx ].SDPState = DISC_STATE_CHAR;
					if( g_MultiCentrInfo[ tmpMstIdx ].service.PrimSerCnt > 0 )
					{
						// Characteristic Find Index Init
		        		g_MultiCentrInfo[ tmpMstIdx ].service.PrimCharIdx = 0;
		        		g_MultiCentrInfo[ tmpMstIdx ].service.CharFindIdx = 0;
						GATT_DiscAllChars(  connHandle,
						                    g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[0].Attr_StHandle,\
						                    g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[0].Attr_EGHandle, 
						                    gapMultiRole_TaskID );
						AT_LOG("SDP Service handle %d,PrimServCnt %d\n",connHandle,g_MultiCentrInfo[ tmpMstIdx ].service.PrimSerCnt);
					}

				}
				else if( pMsg->method == ATT_ERROR_RSP )
				{
					g_MultiCentrInfo[ tmpMstIdx ].SDPState = DISC_STATE_IDLE;
					AT_LOG("SDP Service error connHandle %d\n",connHandle);
				}
			break;
			default:
			break;
		}
	}
	else if( g_MultiCentrInfo[ tmpMstIdx ].SDPState == DISC_STATE_CHAR  )
	{
		switch ( pMsg->method )
		{
			case ATT_READ_BY_TYPE_RSP:
				if ( pMsg->msg.readByTypeRsp.numPairs > 0 )
				{
					// Iterate through all three pairs found.
					for(i = 0; i < pMsg->msg.readByTypeRsp.numPairs ; i++)
					{
						primCharIdx = g_MultiCentrInfo[ tmpMstIdx ].service.PrimCharIdx;
						charFindIdx = g_MultiCentrInfo[ tmpMstIdx ].service.CharFindIdx;
						// Extract the starting handle, ending handle, and UUID of the current characteristic.
						// characteristic Handle
						g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Charac[charFindIdx].charStartHandle = \
										BUILD_UINT16(	pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i], \
														pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 1]);
    				
	    				// Characteristic Properties
	    				g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Charac[charFindIdx].Properties = \
	    								pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 2];
    				
	    				// Characteristic UUID
	    				g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Charac[charFindIdx].charUuid = \
	    								BUILD_UINT16(	pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 5], \
    													pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 6]);
    				
    					g_MultiCentrInfo[ tmpMstIdx ].service.CharFindIdx++;
					}
				}
				else if( pMsg->hdr.status == bleProcedureComplete )
				{
					primCharIdx = g_MultiCentrInfo[ tmpMstIdx ].service.PrimCharIdx;
					g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].CharacNum = g_MultiCentrInfo[ tmpMstIdx ].service.CharFindIdx;
					g_MultiCentrInfo[ tmpMstIdx ].service.CharFindIdx = 0;
					
					if(++g_MultiCentrInfo[ tmpMstIdx ].service.PrimCharIdx < g_MultiCentrInfo[ tmpMstIdx ].service.PrimSerCnt )
					{
						primCharIdx = g_MultiCentrInfo[ tmpMstIdx ].service.PrimCharIdx;
						GATT_DiscAllChars(  connHandle, 
						                    g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Attr_StHandle,\
						                    g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Attr_EGHandle, 
						                    gapMultiRole_TaskID );
//						LOG("Dis all char 0x%04X,0x%04X\n",g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Attr_StHandle,\
//							g_MultiCentrInfo[ tmpMstIdx ].service.PrimServ[primCharIdx].Attr_EGHandle);
					}
					else
					{
						AT_LOG("All Characteristic Discover Success \n");
						g_MultiCentrInfo[ tmpMstIdx ].SDPState = DISC_STATE_DONE;
						pGapRoles_AppCGs->SDPNotify( connHandle );
					}
				}
				else if( pMsg->method == ATT_ERROR_RSP )
				{
					g_MultiCentrInfo[ tmpMstIdx ].SDPState = DISC_STATE_IDLE;
					AT_LOG("SDP Characteristic error connHandle %d\n",connHandle);
				}
			break;
			default:
			break;
		}
	}
}

GattScanServer* GAPMultiRole_GetSDPIdx( uint16 connHandle )
{
	uint8 idx = MultiRole_MSTCheckConnIdx( connHandle );
	return ( &g_MultiCentrInfo[idx].service );
}
extern uint8 multiRole_TaskId;
void multiRolePasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
	uint32 passcode = 0;
	LOG("pass code CB connHandle 0x%02X,uiInputs:0x%x,uiOutputs:0x%x\n",connectionHandle,uiInputs,uiOutputs);
	GAPBondMgr_GetParameter( GAPBOND_DEFAULT_PASSCODE,&passcode);
	LOG("passcode %d\n",passcode);
//	GAP_PasscodeUpdate(passcode,connectionHandle);
	GAPBondMgr_PasscodeRsp(connectionHandle,SUCCESS,passcode);

}
void multiRolePairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
	uint8 isUpdate;
	uint8 i;
	AT_LOG("PairStateCB handle 0x%02X,status %d,state 0x%X\n",connHandle,status,state);
	isUpdate = FALSE;

	if ( state == GAPBOND_PAIRING_STATE_STARTED )
	{
	    AT_LOG( "Pairing started\n" );
		g_MRLink[connHandle].PairingStarted = TRUE;
	}
	else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
	{
		if ( status == SUCCESS )
		{
		      AT_LOG( "Pairing success\n" );
			  g_MRLink[connHandle].ConnSecure = TRUE;
			  isUpdate = TRUE;
		}
		else
		{
			g_MRLink[connHandle].PairingStarted = FALSE;
			AT_LOG( "Pairing fail\n" );
		}
	}
	else if ( state == GAPBOND_PAIRING_STATE_BONDED )
	{
		if ( status == SUCCESS )
		{
			AT_LOG( "Bonding success\n" );
			g_MRLink[connHandle].ConnSecure = TRUE;
			isUpdate = TRUE;
		}
		else
		{
			g_MRLink[connHandle].ConnSecure = FALSE;
		}
	}

	if( isUpdate == TRUE )
	{osal_start_timerEx(multiRole_TaskId,0x1000,5000);
		for(i=0;i< MAX_CONNECTION_SLAVE_NUM ; i++)
		{
			if( g_MultiPeriInfo[i].advHandle == connHandle )
			{
				break;
			}
		}
		if( i< MAX_CONNECTION_SLAVE_NUM )
		{
			g_MultiPeriInfo[i].paramUpdateEnable = TRUE;
			// slave role parameter update procedure
			if( (	 g_MultiPeriInfo[i].paramUpdateEnable == TRUE ) && \
				(	g_MultiPeriInfo[i].paramUpdateState == STATE_IDLE ) && \
				(	( 0 == osal_get_timeoutEx( gapMultiRole_TaskID , SLV_START_CONN_UPDATE_EVT )) || \
					( 0 == osal_get_timeoutEx( gapMultiRole_TaskID , SLV_CONN_PARAM_TIMEOUT_EVT) )) )
			{
				if( TRUE == MultiRole_SLVPrepareParamUpdate() )
				{
					g_MultiPeriInfo[i].paramUpdateState = STATE_WAITING;
				}
			}
		}
	}

}
