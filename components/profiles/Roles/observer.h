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
#ifndef OBSERVER_H
#define OBSERVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "gap.h"

/*********************************************************************
    CONSTANTS
*/

/** @defgroup GAPOBSERVERROLE_PROFILE_PARAMETERS GAP Observer Role Parameters
    @{
*/
#define GAPOBSERVERROLE_BD_ADDR        0x400  //!< Device's Address. Read Only. Size is uint8[B_ADDR_LEN]. This item is read from the controller.
#define GAPOBSERVERROLE_MAX_SCAN_RES   0x401  //!< Maximum number of discover scan results to receive. Default is 0 = unlimited.
/** @} End GAPOBSERVERROLE_PROFILE_PARAMETERS */

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/

/**
    Observer Event Structure
*/
typedef union
{
    gapEventHdr_t             gap;                //!< GAP_MSG_EVENT and status.
    gapDeviceInitDoneEvent_t  initDone;           //!< GAP initialization done.
    gapDeviceInfoEvent_t      deviceInfo;         //!< Discovery device information event structure.
    gapDevDiscEvent_t         discCmpl;           //!< Discovery complete event structure.
} gapObserverRoleEvent_t;

/**
    RSSI Read Callback Function
*/
typedef void (*pfnGapObserverRoleRssiCB_t)
(
    uint16 connHandle,                    //!< Connection handle.
    int8  rssi                            //!< New RSSI value.
);

/**
    Observer Event Callback Function
*/
typedef void (*pfnGapObserverRoleEventCB_t)
(
    gapObserverRoleEvent_t* pEvent         //!< Pointer to event structure.
);

/**
    Observer Callback Structure
*/
typedef struct
{
    pfnGapObserverRoleRssiCB_t   rssiCB;   //!< RSSI callback.
    pfnGapObserverRoleEventCB_t  eventCB;  //!< Event callback.
} gapObserverRoleCB_t;

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    API FUNCTIONS
*/

/*  -------------------------------------------------------------------
    Observer Profile Public APIs
*/

/**
    @defgroup OBSERVER_PROFILE_API Observer Profile API Functions

    @{
*/

/**
    @brief   Start the device in Observer role.  This function is typically
            called once during system startup.

    @param   pAppCallbacks - pointer to application callbacks

    @return  SUCCESS: Operation successful.<BR>
            bleAlreadyInRequestedMode: Device already started.<BR>
*/
extern bStatus_t GAPObserverRole_StartDevice( gapObserverRoleCB_t* pAppCallbacks );

/**
    @brief   Set a parameter in the Observer Profile.

    @param   param - profile parameter ID: @ref GAPOBSERVERROLE_PROFILE_PARAMETERS
    @param   len - length of data to write
    @param   pValue - pointer to data to write.  This is dependent on
            the parameter ID and WILL be cast to the appropriate
            data type.

    @return  SUCCESS: Operation successful.<BR>
            INVALIDPARAMETER: Invalid parameter ID.<BR>
*/
extern bStatus_t GAPObserverRole_SetParameter( uint16 param, uint8 len, void* pValue );

/**
    @brief   Get a parameter in the Observer Profile.

    @param   param - profile parameter ID: @ref GAPOBSERVERROLE_PROFILE_PARAMETERS
    @param   pValue - pointer to buffer to contain the read data

    @return  SUCCESS: Operation successful.<BR>
            INVALIDPARAMETER: Invalid parameter ID.<BR>
*/
extern bStatus_t GAPObserverRole_GetParameter( uint16 param, void* pValue );

/**
    @brief   Start a device discovery scan.

    @param   mode - discovery mode: @ref GAP_DEVDISC_MODE_DEFINES
    @param   activeScan - TRUE to perform active scan
    @param   whiteList - TRUE to only scan for devices in the white list

    @return  SUCCESS: Discovery scan started.<BR>
            bleIncorrectMode: Invalid profile role.<BR>
            bleAlreadyInRequestedMode: Not available.<BR>
*/
extern bStatus_t GAPObserverRole_StartDiscovery( uint8 mode, uint8 activeScan, uint8 whiteList );

/**
    @brief   Cancel a device discovery scan.

    @return  SUCCESS: Cancel started.<BR>
            bleInvalidTaskID: Not the task that started discovery.<BR>
            bleIncorrectMode: Not in discovery mode.<BR>
*/
extern bStatus_t GAPObserverRole_CancelDiscovery( void );

/**
    @}
*/

/*  -------------------------------------------------------------------
    TASK API - These functions must only be called by OSAL.
*/

/**
    @internal

    @brief   Observer Profile Task initialization function.

    @param   taskId - Task ID.

    @return  void
*/
extern void GAPObserverRole_Init( uint8 taskId );

/**
    @internal

    @brief   Observer Profile Task event processing function.

    @param   taskId - Task ID
    @param   events - Events.

    @return  events not processed
*/
extern uint16 GAPObserverRole_ProcessEvent( uint8 taskId, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_H */
