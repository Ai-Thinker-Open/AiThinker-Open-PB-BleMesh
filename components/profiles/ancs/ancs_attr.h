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


#ifndef __ANCS_ATTR_H
#define __ANCS_ATTR_H

#include "ble_ancs.h"

#define GATTC_OPCODE_SIZE                1      /**< Size of the GATTC OPCODE. */
#define GATTC_ATTR_HANDLE_SIZE           4      /**< Size of the Attribute handle Size. */


#define ANCS_GATTC_WRITE_PAYLOAD_LEN_MAX (23 - GATTC_OPCODE_SIZE - GATTC_ATTR_HANDLE_SIZE)  /**< Maximum Length of the data we can send in one write. */

typedef enum
{
    APP_ATTR_COMMAND_ID, /**< Currently encoding the Command ID. */
    APP_ATTR_APP_ID,     /**< Currently encoding the App ID. */
    APP_ATTR_ATTR_ID,    /**< Currently encoding the Attribute ID. */
    APP_ATTR_DONE        /**< Encoding done. */
}encode_app_attr_t;

extern void ancs_parse_get_attrs_response(ancs_ctx_t  * p_ancs, const uint8_t * p_data_src, uint8_t  hvx_data_len);
extern bStatus_t app_attrs_get(ancs_ctx_t  * p_ancs, const uint8_t * p_app_id, uint8_t app_id_len);
extern bStatus_t notif_attrs_get(ancs_ctx_t  * p_ancs,const uint8_t * pNotificationUID);

#endif //__ANCS_ATTR_H

