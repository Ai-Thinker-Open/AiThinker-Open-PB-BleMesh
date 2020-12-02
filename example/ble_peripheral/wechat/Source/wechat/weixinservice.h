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


#ifndef WEIXINSERVICE_H
#define WEIXINSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "types.h"
/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters

#define WEIXINPROFILE_READ_CHAR                   0  // RW uint8 - Weinxin Profile Characteristic read value 
#define WEIXINPROFILE_WRITE_CHAR                  1  // RW uint8 - Weinxin Profile Characteristic write value
#define WEIXINPROFILE_INDICATE_CHAR               2  // RW uint8 - Weinxin Profile Characteristic indicate value
#define WEIXIN_SIMPLEPROFILE_PEDOMETER_CHAR       3  // RW uint8 - Weinxin Profile Characteristic pedometer value
#define WEIXIN_SIMPLEPROFILE_TARGET_CHAR          4  // RW uint8 - Weinxin Profile Characteristic pedometer value


  
// Weixin Service Parameters
#define WEIXIN_SERVICE_UUID          	0xFEE7

#define WEIXIN_INDICATE_UUID         	0xFEC8
#define WEIXIN_READ_UUID				0xFEC9
#define WEIXIN_WRITE_UUID				0xFEC7
#define WEIXIN_PEDOMETER_UUID			0xFEA1
#define WEIXIN_TARGET_UUID				0xFEA2

#define WEIXINSERVAPP_NUM_ATTR_SUPPORTED        20
   
   
// Weixin Keys Profile Services bit fields
#define WEIXINPROFILE_SERVICE               0x00000001   

#define WECHAT_GATT_WRITE      		1
#define WECHAT_GATT_INDICATE     	2
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


// Callback when a characteristic value has changed
typedef void (*weixinProfileChange_t)( uint8 paramID );
typedef void (*weixinProfileCBs_t)( uint8 notifyApp );
    
/*********************************************************************
 * API FUNCTIONS
 */

/*
 * Weixin_AddService- Initializes the weixin Information service by registering
 *          GATT attributes with the GATT server.
 *
 */

extern bStatus_t Weixin_AddService( uint32 services );

/*********************************************************************
 * @fn      Weixin_SetParameter
 *
 * @brief   Set a weixin Information parameter.
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
bStatus_t Weixin_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Weixin_GetParameter - Get a weixin Information parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Weixin_GetParameter( uint8 param, void *value );
extern bStatus_t WeixinProfile_RegisterAppCBs( weixinProfileCBs_t appCallbacks );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* WEIXINSERVICE_H */
