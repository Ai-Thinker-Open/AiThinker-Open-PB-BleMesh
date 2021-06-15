/**
 *  @defgroup Multi GAPRole (Multi)
 *  @brief This module implements the Multi GAP Role
 *  For a detailed usage section describing how to send these commands and receive events,
 *  see the <a href="../ble-stack/gaprole.html">GAPRole Section</a> of the
 *  User's Guide.
 *  @{
 *  @file       multi.h
 *  @brief      Multi layer interface
 */

#ifndef MULTI_H
#define MULTI_H

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------------------------------------------------
 * INCLUDES
 */
#include "gap.h"
#include "att.h"

/*-------------------------------------------------------------------
 * CONSTANTS
 */

/** @defgroup Multi_Constants Multi GAPRole Constants
 * @{
 */
#define   MAX_CONNECTION_NUM          5
#define   MAX_CONNECTION_SLAVE_NUM	  2
#define   MAX_CONNECTION_MASTER_NUM	  MAX_CONNECTION_NUM - MAX_CONNECTION_SLAVE_NUM

// multi-role scan service
#define PRIMARY_SERVICE_RES					5
#define Characteristic_LEN					10
#define Characteristic_ValueIndex		    1
#define Characteristic_NotifyIndex	        2


// Multi-role Event
#define START_ADV_SCAN_INIT_EVT       0x4000
#define UPDATE_ADV_DATA_EVT			  0x0001
#define ADV_OFF_EVT					  0x0004			
#define SLV_START_CONN_UPDATE_EVT     0x0008  // Connection Parameters Update Timeout
#define SLV_CONN_PARAM_TIMEOUT_EVT	  0x0010
#define CONN_TIMEOUT_EVT			  0x0020
#define MST_SDP_EVT					  0x0040


/** @defgroup Multi_Params Multi GAPRole Parameters
 * @{
 * Parameters set via @ref GAPRole_SetParameter
 */

/**
 * @brief This parameter will return GAP Role type (Read-only)
 *
 * size: uint8_t
 *
 * range: @ref GAP_Profile_Roles
 */
#define GAPROLE_PROFILEROLE         0x300

/**
 * @brief Identity Resolving Key (Read/Write) Size is uint8_t[KEYLEN].
 *
 * @note If this is set to all 0x00's, the IRK will be randomly generated
 *
 * size: uint8_t[16]
 *
 * default: 0x00000000000000000000000000000000
 *
 * range: 0x00000000000000000000000000000000 - 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
 */
#define GAPROLE_IRK                 0x301

/**
 * @brief Signature Resolving Key (Read/Write)
 *
 * @note If this is set to all 0x00's, the SRK will be randomly generated
 *
 * size: uint8_t[16]
 *
 * default: 0x00000000000000000000000000000000
 *
 * range: 0x00000000000000000000000000000000 - 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
 */
#define GAPROLE_SRK                 0x302

/**
 * @brief Sign Counter (Read/Write)
 *
 * size: uint32_t
 *
 * default: 0x0000
 *
 * range: 0x0000 - 0xFFFF
 */
#define GAPROLE_SIGNCOUNTER         0x303

/**
 * @brief Device Address read from the controller (Read-only)
 *
 * The BDADDR is read, in increasing order of priortiy, from the info page,
 * secondary address from flash, or set from @ref HCI_ReadBDADDRCmd
 *
 * size: uint8_t[6]
 *
 * default: BDADDR from info page
 *
 * range: 0x000000000000 - 0xFFFFFFFFFFFE
 */
#define GAPROLE_BD_ADDR             0x304

/**
 * @brief Enable/Disable Connectable Advertising (Read/Write)
 *
 * @warning @ref GAPROLE_ADV_NONCONN_ENABLED must be set to FALSE in order to enable this
 *
 * size: uint8_t
 *
 * default: TRUE
 *
 * range: TRUE (enabled) or FALSE (disabled)
 */
#define GAPROLE_ADVERT_ENABLED      0x305

/**
 * @brief How long to remain off (in sec) after advertising stops before starting again (Read/Write)
 *
 * If set to 0, advertising will not start again.
 *
 * size: uint16
 *
 * default: 30
 *
 * range: 0-65535
 */
#define GAPROLE_ADVERT_OFF_TIME     0x306

/**
 * @brief Advertisement data (Read/Write)
 *
 * @note The third byte sets limited / general advertising as defined in Vol 3, Part C, section 11.1.3
 * of the BT 4.2 Core Spec.
 *
 * size: a uint8_t array of up to 31 bytes
 *
 * default: 02:01:01 (general advertising)
 */
#define GAPROLE_ADVERT_DATA         0x307

/**
 * @brief Scan Response Data (Read/Write)
 *
 * @note This should be formatted as define d in Vol 3, Part C, section 11.1.3 of the BT 4.2 Core Spec.
 *
 * size: a uint8_t array of up to 31 bytes
 *
 * default: all 0x00's
 */
#define GAPROLE_SCAN_RSP_DATA       0x308

/**
 * @brief Advertisement Types (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref GAP_ADTYPE_ADV_IND
 *
 * range: @ref GAP_Adv_Types
 */
#define GAPROLE_ADV_EVENT_TYPE      0x309

/**
 * @brief Direct Advertisement Type (Read/Write)
 *
 * size: uint8_t
 *
 * default: @ref ADDRMODE_PUBLIC
 *
 * range: @ref Gap_Addr_Modes
 */
#define GAPROLE_ADV_DIRECT_TYPE     0x30A

/**
 * @brief Direct Advertisement Address (Read/Write)
 *
 * size: uint8_t[6]
 *
 * default: NULL
 *
 * range: 0x000000000000 - 0xFFFFFFFFFFFE
 */
#define GAPROLE_ADV_DIRECT_ADDR     0x30B

/**
 * @brief Which channels to advertise on (Read/Write)
 *
 * Multiple channels can be selected by ORing the bit values below.
 *
 * size: uint8_t
 *
 * default: @ref GAP_ADVCHAN_ALL
 *
 * range: @ref GAP_Adv_Chans
 */
#define GAPROLE_ADV_CHANNEL_MAP     0x30C

/**
 * @brief Policy for filtering advertisements (Read/Write)
 *
 * @note This is ignored for direct advertising.
 *
 * size: uint8_t
 *
 * default: @ref GAP_FILTER_POLICY_ALL
 *
 * range: @ref GAP_Adv_Filter_Policices
 */
#define GAPROLE_ADV_FILTER_POLICY   0x30D


#define GAPROLE_PARAM_UPDATE_ENABLE 0x310

/**
 * @brief  Minimum connection interval (n * 1.25 ms) to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 6
 *
 * range: 6 - @ref GAPROLE_MAX_CONN_INTERVAL
 */
#define GAPROLE_MIN_CONN_INTERVAL   0x311

/**
 * @brief Maximum connection interval (n * 1.25 ms) to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 3200
 *
 * range: @ref GAPROLE_MIN_CONN_INTERVAL - 3200
 */
#define GAPROLE_MAX_CONN_INTERVAL   0x312

/**
 * @brief Slave latency to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 0
 *
 * range: 0 - 499
 */
#define GAPROLE_SLAVE_LATENCY       0x313

/**
 * @brief Supervision timeout (n x 10 ms) to use when performing param update (Read/Write)
 *
 * size: uint16_t
 *
 * default: 1000
 *
 * range: 10-3200
 */
#define GAPROLE_TIMEOUT_MULTIPLIER  0x314

/**
 * @brief Enable / Disable non-connectable advertising (Read/Write)
 *
 * @warning @ref GAPROLE_ADVERT_ENABLED must be set to FALSE in order to enable this
 *
 * size: uint8_t
 *
 * default: FALSE
 *
 * range: TRUE (enable) or FALSE (disable)
 */
#define GAPROLE_ADV_NONCONN_ENABLED 0x31B

/**
 * @brief Maximmum number of scan reports to store from @ref GAPCentralRole_StartDiscovery (Read/Write)
 *
 * size: uint8_t
 *
 * default: 8
 *
 * range: 0-256 but this will be constrained by available RAM
 */
#define GAPROLE_MAX_SCAN_RES        0x404

/**
 * @brief Advertising off time Units:ms
 *
 * size: uint8_t
 *
 * default: 10(ms)
 *
 * range: 0-256 : 
 */
 #define GAPROLE_ADV_OFF_UNITS

/** @} End Multi_Params */

/** @defgroup Multi_Param_Update_Fail_Actions Failed Parameter Update Actions
* @{
*  Possible actions the device may take if an unsuccessful parameter
*  update is received.
*/
#define MULTIROLE_NO_ACTION                    0 //!< Take no action upon unsuccessful parameter updates
#define MULTIROLE_RESEND_PARAM_UPDATE          1 //!< Continue to resend request until successful update
#define MULTIROLE_TERMINATE_LINK               2 //!< Terminate link upon unsuccessful parameter updates
/** @} End Multi_Param_Update_Fail_Actions */


/** @defgroup Multi_Param_Update_Options Parameter Update Options
 * @{
 *  Possible actions the device may take when it receives a
 *  Connection Parameter Update Request.
 */
#define GAPROLE_LINK_PARAM_UPDATE_ACCEPT       0 //!< Accept all parameter update requests
#define GAPROLE_LINK_PARAM_UPDATE_REJECT       1 //!< Reject all parameter update requests
#define GAPROLE_LINK_PARAM_UPDATE_APP_DECIDES  2 //!< Notify app for it to decide
#define GAPROLE_LINK_PARAM_UPDATE_NUM_OPTIONS  3 //!< Number of options. Used for parameter checking.
/** @} End Multi_Param_Update_Options */

/** @} End Multi_Constants */

/*-------------------------------------------------------------------
 * TYPEDEFS
 */

/** @defgroup Multi_Structs Multi GAPRole Structures
 * @{
 */


/// @brief Multi GAPRole Event Structure

/**
 * GAP multi-gap Role States.
 */
typedef enum
{
  GAPROLE_INIT = 0,                       //!< Waiting to be started
  GAPROLE_STARTED,                        //!< Started but not advertising
  GAPROLE_ADVERTISING,                    //!< Currently Advertising
  GAPROLE_SCANING,                        //!< Currently scaning, add by PHY+        
  GAPROLE_CONNECTING,                     //!< Currently scaning, add by PHY+      
  GAPROLE_WAITING,                        //!< Device is started but not advertising, is in waiting period before advertising again
  GAPROLE_WAITING_AFTER_TIMEOUT,          //!< Device just timed out from a connection but is not yet advertising, is in waiting period before advertising again
  GAPROLE_CONNECTED,                      //!< In a connection
  GAPROLE_TERMINATED,
  GAPROLE_CONNECTED_ADV,                  //!< In a connection + advertising
  GAPROLE_CONNECTED_SCAN,                 //!< In a connection + scan, add by PHY+  `  
  GAPROLE_ERROR                           //!< Error occurred - invalid state
} multiRole_states_t;

typedef union
{
  gapEventHdr_t             gap;                //!< @ref GAP_MSG_EVENT and status.
  gapDeviceInitDoneEvent_t  initDone;           //!< GAP initialization done.
  gapDeviceInfoEvent_t      deviceInfo;         //!< Discovery device information event structure.
  gapDevDiscEvent_t         discCmpl;           //!< Discovery complete event structure.
  gapEstLinkReqEvent_t      linkCmpl;           //!< Link complete event structure.
  gapLinkUpdateEvent_t      linkUpdate;         //!< Link update event structure.
  gapTerminateLinkEvent_t   linkTerminate;      //!< Link terminated event structure.
} gapMultiRoleEvent_t;

/// @brief Multi GAPRole Parameter Update Structure
typedef struct
{
  uint8_t paramUpdateEnable;            //!< @ref Multi_Param_Update_Options
  uint16_t connHandle;                  //!< connection handle
  uint16_t minConnInterval;             //!< minimum connection interval
  uint16_t maxConnInterval;             //!< maximum connection interval
  uint16_t slaveLatency;                //!< slave latency
  uint16_t timeoutMultiplier;           //!< supervision timeout
} gapRole_updateConnParams_t;


/*********************************************************************
* TYPEDEFS -- multi-role as master scan/link
*/
typedef struct
{
	uint8_t 	Type;
	uint8_t 	Len;
	uint8_t 	Value[ATT_UUID_SIZE];
}ADVServiceUUIDs;
typedef struct
{
	uint8_t 	Type;
	uint8_t 	Length;
	uint8_t 	Value[31];		// Max PDU = 31
}ADVLoaclName;
typedef struct{
	uint16_t	ConnMin;
	uint16_t	ConnMax;
}ADVSlaveInterval;
typedef struct
{
	uint8_t 	Type;
	uint8_t 	Len;
	uint8_t 	Value[ATT_UUID_SIZE];
}ADVServiceSolicitation;
typedef struct
{
	uint8_t 	Length;
	uint8_t 	Value[31];		// Max PDU = 31
}ADVServiceDATA;
typedef struct
{
	uint8_t 	Length;
	uint8_t 	Value[31];		// Max PDU = 31
}ADVManufactureDATA;
typedef struct
{
	uint8_t 				AddrType;
	uint8_t 				addr[B_ADDR_LEN];
	bool					RSP_ReadFlag;
	int8					rssi;
	int8					TxPower;
	uint8_t 				Flags;
	ADVServiceUUIDs 		UUID;
	ADVLoaclName			LocalName;
	uint8_t 				OOB_Data;
	uint8_t 				SM_TK_Value;
	ADVSlaveInterval		ConnIntvRange;
	ADVServiceSolicitation	ServiceSolic;				//ServiceSolicitation
	ADVServiceDATA			ServiceData;
	ADVManufactureDATA		ManufactData;
}MultiRoleADV_ScanData;
typedef enum
{
	Dev_Name = 0,
	Dev_MAC,
	Dev_ConnIntv
}MultiRole_filter_t;

// Service and Characteristic
typedef struct
{
	uint16_t	charStartHandle;
	uint16_t	charUuid;
	uint8_t 	UUID_Len;
	uint8_t 	UUID[ATT_UUID_SIZE];
	uint8_t 	Properties;
}Characteristic;
typedef struct
{
	// Service Info , User Don't Care
	uint8_t 	Attr_Data_Len;
	uint16_t	Attr_StHandle;		// start handler
	uint16_t	Attr_EGHandle;		// End_Group_Handle
	// User Care  
	// Caculate UUID Len
	uint8_t 	UUID_Len;
	uint8_t 	UUID[ATT_UUID_SIZE];
	// Characteristic
	uint8_t 	CharacNum;
	Characteristic Charac[Characteristic_LEN];
}GATTReadGroupRsp;
typedef struct
{
	// Primary Service Num
	uint8_t 								PrimSerCnt;
	// Primary Service Characteristic Index
	uint8_t 								PrimCharIdx;
	// Characteristic Find Index
	uint8_t 								CharFindIdx;
	GATTReadGroupRsp	PrimServ[PRIMARY_SERVICE_RES];				//ServerGroupService
}GattScanServer;

/*********************************************************************
* TYPEDEFS
*/
typedef struct
{
	uint8_t 	profileRole;
	uint8_t 	IRK[KEYLEN];
	uint8_t 	SRK[KEYLEN];
	uint32_t	signCounter;
	uint8_t 	bdAddr[B_ADDR_LEN];
}GAPMultiRole_Param_t;
typedef enum
{
	STATE_IDLE = 0,
	STATE_WAITING,
	STATE_ING,
	STATE_DONE
}MultiRole_slvParaUpdate_t;
typedef struct
{
	uint8_t		advHandle;
	multiRole_states_t PeriState;
	uint8_t 	AdvEnabled;
	uint8_t 	AdvNonConnEnabled;
	uint8_t 	AdvEventType;
	uint8_t 	AdvDirectType;
	uint8_t 	AdvDirectAddr[B_ADDR_LEN];
	uint8_t 	AdvChanMap;
	uint8_t 	AdvFilterPolicy;
	uint8_t		paramUpdateEnable;
	MultiRole_slvParaUpdate_t		paramUpdateState;
	uint16		paramConnIntvMIN;
	uint16		paramConnIntvMAX;
	uint16		paramLatency;
	uint16		paramTimeOut;
	uint8_t 	AdvertDataLen;
	uint8_t 	ScanRspDataLen;
	uint16_t	advOffTime;
	uint8_t		advOffTimeIntv;
	uint8_t 	pAdvData[B_MAX_ADV_LEN];
	uint8_t 	pScanRspData[B_MAX_ADV_LEN];
	uint8_t		isAdvDataUpdate;
	uint8_t		isSRspDataUpdate;
}GAPMultiRole_Peripheral_t;

// Discovery states
typedef enum
{
  MTUEC_STATE_IDLE = 0,                
  MTUEC_STATE_ING,                 
  MTUEC_STATE_DONE                 
}MultiRole_MTUEC_t;					// MTU Exchange 
  

// Discovery states
typedef enum
{
  DISC_STATE_IDLE = 0,                // Idle
  DISC_STATE_SVC,                 // Service discovery
  DISC_STATE_CHAR,                 // Characteristic discovery
  DISC_STATE_DONE
}MultiRole_DISC_t;
  

typedef struct
{
	uint8_t		initFlag;
	uint8_t		enConnected;
	// addr and addrType : for storage only , not one-to-one correspondence with CentState ...
	uint8_t 	peerAddr[B_ADDR_LEN];
	uint8_t		peerAddrType;
	// record central state
	multiRole_states_t CentState;
	// exchange 
	MultiRole_DISC_t	SDPState;
	MultiRole_MTUEC_t	MUT_ECState;		// MTU Exchange State
	uint8  dleDone;
	// service 
	GattScanServer service;
}GAPMultiRole_Central_t;

typedef enum
{
	Idle_Role = 0xFF,
	Slave_Role = 0x0,
	Master_Role = 0x3
}GAPMultiRole_State_t;

typedef struct
{
	GAPMultiRole_State_t 	RoleState;
	uint8 	encSuccess;
	uint8	mstConnIdx;					// if master role , stote connection handle index in g_MultiCentrInfo
	uint8 	peerDevAddrType;
	uint8 	peerDevAddr[B_ADDR_LEN];
	uint16 	connectionHandle; 
	uint16 	connInterval;     
	uint16 	connLatency;      
	uint16 	connTimeout;  
}GAPMultiRole_LinkCtrl_t;

typedef struct
{
	multiRole_states_t lastState;
	multiRole_states_t currentState;
	uint8_t	slaveRole_Cnt;
	uint8_t	masterRole_Cnt;
	uint8_t linkCnt;
}GAPMultiRole_StateMachine_t;


/** @} End Multi_Structs */

/*-------------------------------------------------------------------
 * MACROS
 */

/*-------------------------------------------------------------------
 * Profile Callbacks
 */

/**
 * Callback when the device has been started.  Callback event to
 * the Notify of a state change.
 */
typedef void (*gapRolesEachScan_t)( gapDeviceInfoEvent_t *pPkt );
typedef void (*gapRolesScanDone_t)(uint8 scanCnt , MultiRoleADV_ScanData *pPkt);
typedef void (*gapRolesEstablish_t)( uint16 connHandle,GAPMultiRole_State_t role,multiRole_states_t newState,uint8 *addr );
typedef void (*gapRolesTerminate_t)( uint16 connHandle,GAPMultiRole_State_t role,multiRole_states_t newState,uint8 reason );

/**
 * Callback when the device has read an new RSSI value during a connection.
 */
typedef void (*gapRolesRssiRead_t)( uint16 connHandle,int8 newRSSI );

/**
 * Callback when SDP done during a connection.
 */
typedef void (*gapRolesSDPNotify_t)(uint16 connHandle); 

typedef struct
{
	gapRolesEachScan_t		pfnEachScan;
	gapRolesScanDone_t		pfnScanDone;
	gapRolesEstablish_t    	pfnEstablish;  //!< Whenever the device changes state
	gapRolesTerminate_t		pfnTerminate;
	gapRolesRssiRead_t      pfnRssiRead;     //!< When a valid RSSI is read from controller
	gapRolesSDPNotify_t		SDPNotify;      //!< Event callback.
} gapMultiRolesCBs_t;

/** @defgroup Multi_CBs Multi GAPRole Callbacks
 * @{
 * These are functions whose pointers are passed from the application
 * to the GAPRole so that the GAPRole can send events to the application
 */

/**
 * @brief Multi Event Callback Function
 *
 * This callback is used by the Multi GAPRole to forward GAP_Events to the
 * application.
 *
 * If the message is successfully queued to the application for later processing,
 * FALSE is returned because the application deallocates it later. Consider the
 * following state change event from multi_role as an example of this:
 *
 * @code{.c}
 * static void multi_role_processAppMsg(mrEvt_t *pMsg)
 * {
 *   switch (pMsg->event)
 *   {
 *   case MR_STATE_CHANGE_EVT:
 *     multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
 *     // Free the stack message
 *     ICall_freeMsg(pMsg->pData);
 *     break;
 * @endcode
 *
 * If the message is not successfully queued to the application, TRUE is returned
 * so that the GAPRole can deallocate the message. If the heap has enough room,
 * the message must always be successfully enqueued.
 *
 * @param pEvent Pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message
 * @return  FALSE otherwise
 */
typedef uint8_t (*passThroughToApp_t)
(
  gapMultiRoleEvent_t *pEvent
);

/**
 * @brief Callback for the app to decide on a parameter update request
 *
 * This callback will be used if the @ref GAP_UPDATE_LINK_PARAM_REQ_EVENT parameter
 * is set to @ref GAPROLE_LINK_PARAM_UPDATE_APP_DECIDES
 *
 * @param pReq Pointer to param update request
 * @param pRsp Pointer to param update response.
 */
typedef void (*paramUpdateAppDecision_t)
(
  gapUpdateLinkParamReq_t *pReq
//  gapUpdateLinkParamReqReply_t *pRsp
);

/**
 * @brief Multi GAPRole Callback structure
 *
 * This must be setup by the application and passed to the GAPRole when
 * @ref GAPRole_StartDevice is called.
 */
typedef struct
{
  passThroughToApp_t        pfnPassThrough;             //!< When the event should be processed by the app instead of the GAP Role
  paramUpdateAppDecision_t  pfnParamUpdateAppDecision;  //!< When the app should decide on a param update request
} gapRolesCBs_t;

/** @} End Multi_Structs */

/*-------------------------------------------------------------------
 * API FUNCTIONS
 */

/**
 * @brief       Set a GAP Role parameter.
 *
 * @note
 *        The "len" field must be set to the size of a "uint16_t" and the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param param     @ref Multi_Params
 * @param len       length of data to write
 * @param pValue    pointer to data to write.  This is dependent on
 *   the parameter ID and will be cast to the appropriate
 *   data type (example: data type of uint16_t will be cast to
 *   uint16_t pointer).
 * @param           connHandle connection handle
 *
 * @return @ref SUCCESS
 * @return @ref INVALIDPARAMETER
 * @return  @ref bleInvalidRange : len is invalid for the given param
 */
extern bStatus_t GAPMultiRole_SetParameter(uint16_t param, uint8_t len, void *pValue, uint8 Handler);

/**
 * @brief       Get a GAP Role parameter.
 *
 * @note
 *        The "pValue" field must point to a "uint16_t".
 *
 * @param param     @ref Multi_Params
 * @param pValue    pointer to location to get the value.  This is dependent on
 *   the parameter ID and will be cast to the appropriate
 *   data type (example: data type of uint16_t will be cast to
 *   uint16_t pointer).
 * @param           connHandle connection handle
 *
 * @return @ref SUCCESS
 * @return @ref INVALIDPARAMETER
 */
extern bStatus_t GAPMultiRole_GetParameter(uint16_t param, void *pValue, uint8 Handler);

/**
 * @brief Initialize the GAP layer.
 *
 * @warning Only call this function once.
 *
 * @param       pAppCallbacks @ref gapRolesCBs_t
 * @param       numConns a pointer to the desired number of connections that the
 *              application wants is passed in with this parameter. the GAPRole
 *              will use this value to negotiate with the amount of connections
 *              that the stack supports and place the negotiated value in this
 *              memory location for return to the app.
 *
 * @return      @ref SUCCESS
 * @return  @ref bleAlreadyInRequestedMode : Device already started.
 */
bStatus_t GAPMultiRole_StartDevice(gapMultiRolesCBs_t *pAppCallbacks, uint8_t* numConns);

/**
 * @brief       Terminates the existing connection.
 *
 * @param       connHandle handle of connection to terminate
 *
 * @return      @ref SUCCESS
 * @return      @ref bleIncorrectMode
 * @return      @ref HCI_ERROR_CODE_CONTROLLER_BUSY : terminate procedure has already started
 */
extern bStatus_t GAPMultiRole_TerminateConnection(uint16_t connHandle);

/**
 * @brief   Start a device discovery scan.
 *
 * @param   mode discovery mode: @ref GAP_Discovery
 * @param   activeScan TRUE to perform active scan
 * @param   whiteList TRUE to only scan for devices in the white list
 *
 * @return  @ref SUCCESS : Discovery discovery has started.
 * @return  @ref bleIncorrectMode : Invalid profile role.
 * @return  @ref bleAlreadyInRequestedMode : Device discovery already started
 * @return  @ref HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS : bad parameter
 */
extern bStatus_t GAPMultiRole_StartDiscovery(uint8_t mode, uint8_t activeScan, uint8_t whiteList);

/**
 * @brief   Cancel a device discovery scan.
 *
 * @return  @ref SUCCESS : Cancel started.
 * @return  @ref bleInvalidTaskID : Not the task that started discovery.
 * @return  @ref bleIncorrectMode : Not in discovery mode.
 */
extern bStatus_t GAPMultiRole_CancelDiscovery(void);

/**
 * @brief   Establish a link to a peer device.
 *
 * @param   highDutyCycle  TRUE to high duty cycle scan, FALSE if not
 * @param   whiteList determines use of the white list: @ref GAP_Whitelist
 * @param   addrTypePeer @ref Addr_type
 * @param   peerAddr peer device address
 *
 * @return  @ref SUCCESS : started establish link process
 * @return  @ref bleIncorrectMode : invalid profile role
 * @return  @ref bleNotReady : a scan is in progress
 * @return  @ref bleAlreadyInRequestedMode : can�t process now
 * @return  @ref bleNoResources : too many links
 */
extern bStatus_t GAPMultiRole_EstablishLink(uint8_t highDutyCycle, uint8_t whiteList,
                                              uint8_t addrTypePeer, uint8_t *peerAddr);


/**
 * @brief   Send a connection parameter update to a connected device
 *
 * @param   handleFailure @ref Multi_Param_Update_Fail_Actions
 * @param   pConnParams pointer to connection parameters
 *
 * @return  @ref SUCCESS : operation was successful.
 * @return  @ref INVALIDPARAMETER : Data can not fit into one packet.
 * @return  @ref MSG_BUFFER_NOT_AVAIL : No HCI buffer is available.
 * @return  @ref bleInvalidRange : params do not satisfy spec
 * @return  @ref bleIncorrectMode : invalid profile role.
 * @return  @ref bleAlreadyInRequestedMode : already updating link parameters.
 * @return  @ref bleNotConnected : Connection is down
 * @return  @ref bleMemAllocError : Memory allocation error occurred.
 * @return  @ref bleNoResources : No available resource
 */
extern bStatus_t GAPMultiRole_connUpdate(uint8_t handleFailure,
                                       gapRole_updateConnParams_t *pConnParams);


// === from peripheral.c
/**
 * @brief       Update the parameters of an existing connection
 *
 * @param       connInterval - the new connection interval
 * @param       latency - the new slave latency
 * @param       connTimeout - the new timeout value
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again, or take no action
 *
 * @return      SUCCESS, bleNotConnected or bleInvalidRange
 */
extern bStatus_t GAPMultiRole_SendUpdateParam( uint16 minConnInterval, uint16 maxConnInterval,
                                          uint16 latency, uint16 connTimeout, uint8 handleFailure );
/// @cond NODOC

/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */

/**
 * @internal
 *
 * @brief       Initialization function for the GAP Role Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param       the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return      void
 */
extern void GAPMultiRole_Init( uint8 task_id );

/**
 * @internal
 *
 * @brief       GAP Role Task event processor.
 *          This function is called to process all events for the task.
 *          Events include timers, messages and any other user defined
 *          events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return      events not processed
 */
extern uint16 GAPMultiRole_ProcessEvent( uint8 task_id, uint16 events );

/// @endcond // NODOC

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

/**
 *
 * @brief       GAP MultiRole Init peer address to link
 *
 * @param   	idx - Index of peer address
 *				pAddr-peer device Address
 *				addrType - peer device address type (Public address ...)
 *
 * @return     	None
 */
extern char *bdAddr2Str( uint8 *pAddr );
extern uint8 GAPMultiRole_addPeerAddr(uint8_t *pAddr,uint8 addrType,uint8 en_connect );
extern uint8 GAPMultiRole_delPeerAddr(uint8_t *pAddr );
extern void  GAPMultiRole_enConnPeerAddr(uint8_t *pAddr,uint8 en);



extern GattScanServer* GAPMultiRole_GetSDPIdx( uint16 connHandle );

//extern void gapRole_setEvent(uint32_t event);


#ifdef __cplusplus
}
#endif

#endif /* MULTI_H */

/** @} End Multi */
