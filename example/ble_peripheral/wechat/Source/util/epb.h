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


#ifndef __EPB_H__
#define __EPB_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t *data;
    int len;
} Bytes;

typedef struct
{
    const uint8_t *data;
    int len;
} CBytes;

typedef struct
{
    char *str;
    int len;
} eString;

typedef struct
{
    const char *str;
    int len;
} EString;

typedef uint8_t Message;

typedef struct 
{
    const uint8_t *unpack_buf;
    uint8_t *pack_buf;
    int buf_len;
    int buf_offset;
} Epb;

/*
 * embeded protobuf unpack functions
 */

void epb_unpack_init(Epb *e, const uint8_t *buf, int len);
bool epb_has_tag(Epb *e, uint16_t tag);

//Varint
int32_t epb_get_int32(Epb *e, uint16_t tag);
uint32_t epb_get_uint32(Epb *e, uint16_t tag);
int32_t epb_get_sint32(Epb *e, uint16_t tag);
bool epb_get_bool(Epb *e, uint16_t tag);
int epb_get_enum(Epb *e, uint16_t tag);

//Length Delimited
const char *epb_get_string(Epb *e, uint16_t tag, int *len);
const uint8_t *epb_get_bytes(Epb *e, uint16_t tag, int *len);
const Message *epb_get_message(Epb *e, uint16_t tag, int *len);

//Length Delimited Packed Repeadted Field
//TODO

//Fixed32
uint32_t epb_get_fixed32(Epb *e, uint16_t tag);
int32_t epb_get_sfixed32(Epb *e, uint16_t tag);
float epb_get_float(Epb *e, uint16_t tag);

/*
 * embeded protobuf pack functions
 */

void epb_pack_init(Epb *e, uint8_t *buf, int len);
int epb_get_packed_size(Epb *e);

//Varint
int epb_set_int32(Epb *e, uint16_t tag, int32_t value);
int epb_set_uint32(Epb *e, uint16_t tag, uint32_t value);
int epb_set_sint32(Epb *e, uint16_t tag, int32_t value);
int epb_set_bool(Epb *e, uint16_t tag, bool value);
int epb_set_enum(Epb *e, uint16_t tag, int value);

//Length Delimited
int epb_set_string(Epb *e, uint16_t tag, const char *data, int len);
int epb_set_bytes(Epb *e, uint16_t tag, const uint8_t *data, int len);
int epb_set_message(Epb *e, uint16_t tag, const Message *data, int len);

//Length Delimited Packed Repeadted Field
//TODO

//Fixed32
int epb_set_fixed32(Epb *e, uint16_t tag, uint32_t value);
int epb_set_sfixed32(Epb *e, uint16_t tag, int32_t value);
int epb_set_float(Epb *e, uint16_t tag, float value);

//Pack size
int epb_varint32_pack_size(uint16_t tag, uint32_t value, bool is_signed);
int epb_fixed32_pack_size(uint16_t tag);
int epb_length_delimited_pack_size(uint16_t tag, int len);

#endif
