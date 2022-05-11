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

/*************************************************************************************************
    Filename:       gap.c
    Revised:
    Revision:

    Description:    This file contains the GAP Configuration API.



**************************************************************************************************/

#include "bcomdef.h"
#include "gap.h"
#include "sm.h"
#include "log.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

/*********************************************************************
    LOCAL FUNCTION PROTOTYPES
*/

/*********************************************************************
    API FUNCTIONS
*/

/*********************************************************************
    Called to setup the device.  Call just once.

    Public function defined in gap.h.
*/
bStatus_t GAP_DeviceInit(  uint8 taskID,
                           uint8 profileRole,
                           uint8 maxScanResponses,
                           uint8* pIRK,
                           uint8* pSRK,
                           uint32* pSignCounter )
{
    bStatus_t stat = INVALIDPARAMETER;   // Return status

    // Valid profile roles and supported combinations
    switch ( profileRole )
    {
    case GAP_PROFILE_BROADCASTER:
        #if ( HOST_CONFIG & ( BROADCASTER_CFG | PERIPHERAL_CFG ) )
        {
            stat = SUCCESS;
        }

        #endif
        break;

    case GAP_PROFILE_OBSERVER:
        #if ( HOST_CONFIG & ( OBSERVER_CFG | CENTRAL_CFG ) )
        {
            stat = SUCCESS;
        }

        #endif
        break;

    case GAP_PROFILE_PERIPHERAL:
        #if ( HOST_CONFIG & PERIPHERAL_CFG )
        {
            stat = SUCCESS;
        }

        #endif
        break;

    case GAP_PROFILE_CENTRAL:
        #if ( HOST_CONFIG & CENTRAL_CFG )
        {
            stat = SUCCESS;
        }

        #endif
        break;

    case (GAP_PROFILE_BROADCASTER | GAP_PROFILE_OBSERVER):
        #if ( ( HOST_CONFIG & ( BROADCASTER_CFG | PERIPHERAL_CFG ) ) && \
        ( HOST_CONFIG & ( OBSERVER_CFG | CENTRAL_CFG ) ) )
        {
            stat = SUCCESS;
        }

        #endif
        break;

    case (GAP_PROFILE_PERIPHERAL | GAP_PROFILE_OBSERVER):
            #if ( ( HOST_CONFIG & PERIPHERAL_CFG ) && \
            ( HOST_CONFIG & ( OBSERVER_CFG | CENTRAL_CFG ) ) )
        {
            stat = SUCCESS;
        }

            #endif
        break;

    case (GAP_PROFILE_CENTRAL | GAP_PROFILE_BROADCASTER):
            #if ( ( HOST_CONFIG & CENTRAL_CFG ) && \
            ( HOST_CONFIG & ( BROADCASTER_CFG | PERIPHERAL_CFG ) ) )
        {
            stat = SUCCESS;
        }

            #endif
        break;

    case (GAP_PROFILE_CENTRAL | GAP_PROFILE_PERIPHERAL):
    {
        LOG("GAP_DeviceInit: GAP_PROFILE_CENTRAL | GAP_PROFILE_PERIPHERAL \n");
        stat = SUCCESS;
    }
    break;

    // Invalid profile roles
    default:
            stat = INVALIDPARAMETER;
            break;
        }

    if ( stat == SUCCESS )
    {
        // Setup the device configuration parameters
        stat = GAP_ParamsInit( taskID, profileRole );

        if ( stat == SUCCESS )
        {
            #if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
            {
                GAP_SecParamsInit( pIRK, pSRK, pSignCounter );
            }
            #endif
            #if ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
            {
//        if ( (profileRole == GAP_PROFILE_BROADCASTER) ||
//             (profileRole == GAP_PROFILE_PERIPHERAL) )
//        {
//          maxScanResponses = 0; // Can't scan, so no need for responses. Force 0.
//        }

                // Initialize GAP Central Device Manager
                GAP_CentDevMgrInit( maxScanResponses );

                #if ( HOST_CONFIG & CENTRAL_CFG )
                {
                    // Register GAP Central Connection processing functions
                    GAP_CentConnRegister();

                    // Initialize SM Initiator
                    SM_InitiatorInit();
                }
                #endif
            }
            #endif
            #if ( HOST_CONFIG & ( PERIPHERAL_CFG | BROADCASTER_CFG ) )
            {
                // Initialize GAP Peripheral Device Manager
                GAP_PeriDevMgrInit();

                #if ( HOST_CONFIG & PERIPHERAL_CFG )
                {
                    // Initialize SM Responder
                    SM_ResponderInit();
                }
                #endif
            }
            #endif
        }
    }

    return ( stat );
}

/*********************************************************************
*********************************************************************/
