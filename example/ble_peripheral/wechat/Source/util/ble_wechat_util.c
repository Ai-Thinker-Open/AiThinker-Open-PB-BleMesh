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


#include "bcomdef.h"
#include "peripheral.h"
#include "ble_wechat_util.h"

//function for getting handler by type
//find the type in the list structure of data handler then return the node pointer
#define BigLittleSwap16(A)  ((((uint16_t)(A) & 0xff00) >> 8) | \
                            (((uint16_t)(A) & 0x00ff) << 8))

 
#define BigLittleSwap32(A)  ((((uint32_t)(A) & 0xff000000) >> 24) | \
                            (((uint32_t)(A) & 0x00ff0000) >> 8) | \
                            (((uint32_t)(A) & 0x0000ff00) << 8) | \
                            (((uint32_t)(A) & 0x000000ff) << 24))

int checkCPUendian()
{
       union{
              unsigned long i;
              uint8_t s[4];
       }c;
 
       c.i = 0x12345678;
       return (0x12 == c.s[0]);
}

unsigned long t_htonl(unsigned long h)
{
       return checkCPUendian() ? h : BigLittleSwap32(h);
}
 
unsigned long t_ntohl(unsigned long n)
{

       return checkCPUendian() ? n : BigLittleSwap32(n);
}

unsigned short htons(unsigned short h)
{
       return checkCPUendian() ? h : BigLittleSwap16(h);
}
 
unsigned short ntohs(unsigned short n)
{
       return checkCPUendian() ? n : BigLittleSwap16(n);
}

/*turn an unsigned short value to big-endian value					*/
/*for example 0x1234 in the memory of X86 is 0x34 and 0x12	*/
/*then turn it to Network Byte Order is 0x12 and 0x34				*/

void get_mac_addr(uint8_t *ownAddress)
{
    uint8_t btAddress[B_ADDR_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, btAddress);
    ownAddress[0]=btAddress[5];
    ownAddress[1]=btAddress[4];
    ownAddress[2]=btAddress[3];
    ownAddress[3]=btAddress[2];
    ownAddress[4]=btAddress[1];
    ownAddress[5]=btAddress[0];
}

char *convertBdAddr2Str(uint8_t *pAddr, uint8_t *pStr)
{
    uint8_t     charCnt;
    char        hex[] = "0123456789ABCDEF";
    static char str[2*B_ADDR_LEN];

    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for (charCnt = B_ADDR_LEN; charCnt > 0; charCnt--)
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }

    return str;
}
