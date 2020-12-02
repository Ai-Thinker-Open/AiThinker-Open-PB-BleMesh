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
  Filename:       eddystoneCfg.h

  Description:    This file contains the Eddystone URL Configuration Service
                  definitions and prototypes.


**************************************************************************************************/

#ifndef EDDYSTONECFG_H
#define EDDYSTONECFG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// TX Power Mode
#define TX_POWER_MODE_LOWEST      0
#define TX_POWER_MODE_LOW         1
#define TX_POWER_MODE_MEDIUM      2
#define TX_POWER_MODE_HIGH        3


// Parameter index
#define EDSCFG_LOCK_STATE              0   // boolean 
#define EDSCFG_LOCK                    1   // uint128
#define EDSCFG_UNLOCK                  2   // uint128
#define EDSCFG_URI_DATA                3   // uint8[18]
#define EDSCFG_FLAGS                   4   // int8
#define EDSCFG_ADV_TX_PWR_LVLS         5   // int8[4]
#define EDSCFG_TX_POWER_MODE           6   // uint8
#define EDSCFG_BEACON_PERIOD           7   // uint16
#define EDSCFG_RESET                   8   // boolean
#define EDSCFG_URI_DATA_LEN            9   // uint8

#define EDSCFG_UID_DATA                10   // uint8[16]
#define EDSCFG_FRAME_COMB_DATA         11   // uint8

// URL Configuration Service UUID
#define URLCFG_SVC_UUID                     0x2080
    
// Characteristic UUID
#define EDSCFG_LOCK_STATE_UUID              0x2081
#define EDSCFG_LOCK_UUID                    0x2082
#define EDSCFG_UNLOCK_UUID                  0x2083
#define EDSCFG_URI_DATA_UUID                0x2084
#define EDSCFG_FLAGS_UUID                   0x2085
#define EDSCFG_ADV_TX_PWR_LVLS_UUID         0x2086
#define EDSCFG_TX_POWER_MODE_UUID           0x2087
#define EDSCFG_BEACON_PERIOD_UUID           0x2088
#define EDSCFG_RESET_UUID                   0x2089

#define EDSCFG_UID_UUID                     0xfff1
#define EDSCFG_FRAME_COMBINE_UUID           0xfff2
  
// Simple Keys Profile Services bit fields
#define EDSCFG_SERVICE               0x00000001

// Length of URI Data in bytes
#define EDSCFG_CHAR_URI_DATA_LEN           18

// Length of UID Data in bytes
#define EDSCFG_CHAR_UID_DATA_LEN           18

// Characteristic default values
#define EDSCFG_CHAR_URI_DATA_DEFAULT          "http://www.phyplusinc.com/"
#define EDSCFG_CHAR_FLAGS_DEFAULT             0
#define EDSCFG_CHAR_TX_POWER_MODE_DEFAULT     TX_POWER_MODE_LOW
#define EDSCFG_CHAR_BEACON_PERIOD_DEFAULT     1000
#define EDSCFG_CHAR_LOCK_DEFAULT              {0}

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8    URIDataLen;

/*********************************************************************
 * MACROS
 */

// Eddystone Base 128-bit UUID: EE0CXXXX-8786-40BA-AB96-99B91AC981D8
#define EDDYSTONE_BASE_UUID_128( uuid )  0xD8, 0x81, 0xC9, 0x1A, 0xB9, 0x99, \
                                         0x96, 0xAB, 0xBA, 0x40, 0x86, 0x87, \
                           LO_UINT16( uuid ), HI_UINT16( uuid ), 0x0C, 0xEE

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*edsCfgSvcChange_t)( uint8 paramID );

typedef struct
{
  edsCfgSvcChange_t        pfnedsCfgSvcChange;  // Called when characteristic value changes
} edsCfgSvcCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * edsCfgSvc_AddService- Initializes the Eddystone URL Configuration
 *          service by registering GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t edsCfgSvc_AddService( void );

/*
 * edsCfgSvc_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t edsCfgSvc_RegisterAppCBs( edsCfgSvcCBs_t *appCallbacks );

/*
 * edsCfgSvc_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t edsCfgSvc_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * edsCfgSvc_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t edsCfgSvc_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* EDDYSTONECFG_H */
