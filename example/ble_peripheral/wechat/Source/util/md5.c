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


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "md5.h"
const uint32_t X[4][2] = {{0, 1}, {1, 5}, {5, 3}, {0, 7}};
const uint32_t S[4][4] = {{ 7, 12, 17, 22 },{ 5, 9 , 14, 20 },{ 4, 11, 16, 23 },{ 6, 10, 15, 21 }};
uint32_t F( uint32_t X, uint32_t Y, uint32_t Z )
{
	return ( X & Y ) | ( ~X & Z );
}
uint32_t G( uint32_t X, uint32_t Y, uint32_t Z )
 {
	return ( X & Z ) | ( Y & ~Z );
 }
uint32_t H( uint32_t X, uint32_t Y, uint32_t Z )
{
	return X ^ Y ^ Z;
}
uint32_t I( uint32_t X, uint32_t Y, uint32_t Z )
{
	return Y ^ ( X | ~Z );
}
  // rotates x left s bits.
uint32_t rotate_left( uint32_t x, uint32_t s )
{
	return ( x << s ) | ( x >> ( 32 - s ) );
}
  // Pre-processin
uint32_t count_padding_bits ( uint32_t length )
{
//	uint32_t div = length * BITS / BLOCK_SIZE;
	uint32_t mod = length * BITS % BLOCK_SIZE;
	uint32_t c_bits;
	if ( mod == 0 )
	{
			c_bits = MOD_SIZE;
	}
	else
	{
		c_bits = ( MOD_SIZE + BLOCK_SIZE - mod ) % BLOCK_SIZE;
	}
	return c_bits / BITS;
}
MD5String append_padding_bits ( char * argv )
{
	uint32_t msg_length = strlen ( argv );
	uint32_t bit_length = count_padding_bits ( msg_length );
	uint64_t app_length = msg_length * BITS;
	MD5String string;
	string.str = (char *)malloc(msg_length + bit_length + APP_SIZE / BITS);
	strncpy ( string.str, argv, msg_length );
	memset ( string.str + msg_length, 0, bit_length );
	string.str [ msg_length ] = SINGLE_ONE_BIT;
	memmove ( string.str + msg_length + bit_length, (char *)&app_length, sizeof( uint64_t ) );
	string.len = msg_length + bit_length + sizeof( uint64_t );
	return string;
}
int32_t md5 (char *argv, uint8_t *md5_32)
{
	MD5String string;
	uint32_t w[16];
	uint32_t chain[4];
	uint32_t state[4];
//	uint8_t r[16];
	uint32_t ( *auxi[ 4 ])( uint32_t, uint32_t, uint32_t ) = { F, G, H, I };
	int sIdx;
	int wIdx;
	string = append_padding_bits ( argv );
	chain[0] = A;
	chain[1] = B;
	chain[2] = C;
	chain[3] = D;
	for (uint32_t j = 0; j < string.len; j += BLOCK_SIZE / BITS)
	{
		memmove ( (char *)w, string.str + j, BLOCK_SIZE / BITS );
		memmove ( state, chain, sizeof(chain) );
		for ( uint8_t roundIdx = 0; roundIdx < 4; roundIdx++ )
		{
			wIdx = X[ roundIdx ][ 0 ];
			sIdx = 0;
			for (uint8_t i = 0; i < 16; i++ )
			{	
				state[sIdx] = state [(sIdx + 1)%4] + rotate_left( state[sIdx] +(*auxi[ roundIdx])( state[(sIdx+1) % 4],
				state[(sIdx+2) % 4], 
				state[(sIdx+3) % 4]) + w[ wIdx ] + (uint32_t)floor((1ULL << 32) * fabs(sin( roundIdx * 16 + i + 1 )) ),
				S[ roundIdx ][ i % 4 ]);
				sIdx = ( sIdx + 3 ) % 4;
				wIdx = ( wIdx + X[ roundIdx ][ 1 ] ) & 0xF;
			}
		}
		chain[ 0 ] += state[ 0 ];
		chain[ 1 ] += state[ 1 ];
		chain[ 2 ] += state[ 2 ];
		chain[ 3 ] += state[ 3 ];
	}
	memmove ( md5_32 + 0, (char *)&chain[0], sizeof(uint32_t) );
	memmove ( md5_32 + 4, (char *)&chain[1], sizeof(uint32_t) );
	memmove ( md5_32 + 8, (char *)&chain[2], sizeof(uint32_t) );
	memmove ( md5_32 + 12, (char *)&chain[3], sizeof(uint32_t) );
	free(string.str);
	string.str = NULL;
  return EXIT_SUCCESS;
}

