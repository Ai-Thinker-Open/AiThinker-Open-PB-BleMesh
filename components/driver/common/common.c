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

/*******************************************************************************
* @file		common.c
* @brief	Contains all functions support for common function driver£¬such as string function,you can use this driver for spi,adc,uart and so on
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include "types.h"
#include "common.h"
#include "ap_cp.h"

//#include "uart.h"
extern uint32_t hclk,pclk;


/**************************************************************************************
 * @fn          hal_setMem
 *
 * @brief       This function process for set some memory addr with a value
 *
 * input parameters
 *
 * @param       uint8_t *buf: set memory buffer
 *              uint8_t value: memory value
 *              uint32_t length: set memory length        
 *
 * output parameters
 *
 * @param       uint8_t *buf: set memory buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_setMem(uint8_t *buf, uint8_t value, uint32_t length)
{
   while ( length-- )
  {
    *buf++ = value;
  }

}	


/**************************************************************************************
 * @fn          hal_cpyMem
 *
 * @brief       This function process for copying data from source addr to dest addr,once copy one byte
 *
 * input parameters
 *
 * @param       uint8_t *dst: copy destnation buffer
 *              uint8_t *src: copy source buffer
 *              uint32_t length: copy length        
 *
 * output parameters
 *
 * @param       uint8_t *dst: copy destnation buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_cpyMem(uint8_t *dst, uint8_t *src, uint32_t length)
{
	uint8_t *p_dst= (uint8_t *) dst;
	uint8_t *p_src= (uint8_t *) src;
		
	while ( length-- )
  {
      *p_dst++ = *p_src++;
  }

}

/**************************************************************************************
 * @fn          hal_cpyMem32
 *
 * @brief       This function process for copying data from source addr to dest addr,once copy 4 bytes
 *
 * input parameters
 *
 * @param       uint32_t *dst: copy destnation buffer
 *              uint32_t *src: copy source buffer
 *              uint32_t length: copy length        
 *
 * output parameters
 *
 * @param       uint32_t *dst: copy destnation buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_cpyMem32(uint32_t *dst, uint32_t *src, uint32_t length)
{
	 uint32_t *p1=(uint32_t *)dst;
	 uint32_t *p2=(uint32_t *)src;
	
	
	while ( length )
  {
    *p1++ = *p2++;
		if(length>=4)
		  length-=4;
		else
			length=0;
  }
		
}


/**************************************************************************************
 * @fn          hal_my_strcmp
 *
 * @brief       This function process for compare two strings, return  1 means same, 0 means different
 *
 * input parameters
 *
 * @param       const uint8_t *str: the first string
 *              const uint8_t *ptr: the second string      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      1:the same
 *              0:the different
 **************************************************************************************/
int hal_my_strcmp( const uint8_t *str,  const uint8_t *ptr)  
{  
     
    while ( *ptr!='\0')  
    {  
			 if(*str!= *ptr) return 0;
        ++str;  
        ++ptr;  
    }  
   if(*str=='\0') 
      return 1;
	 else
		  return 0;
		
}

uint8 char_array_cmp(uint8 *s1, uint8 *s2, uint8 len)
{
    uint8 i;
     
    for(i = 0; i < len; i ++)
        if(s1[i] > s2[i]) return 1;
        else if(s1[i] < s2[i]) return 2;
    
     
    return 0;//equal
}


/**************************************************************************************
 * @fn          hal_copy_bin_from_flash_to_sram
 *
 * @brief       This function process for copy bin from flash to sram
 *
 * input parameters
 *
 * @param       int toAddr: destnation address
 *              int fromAddr: source address
 *              int length: copy length
 *
 * output parameters
 *
 * @param       (uint8_t *) toAddr: destnation buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_copy_bin_from_flash_to_sram( int toAddr, int fromAddr, int length)
{

	hal_cpyMem((uint8_t *) toAddr, (uint8_t *) fromAddr, length);
	
}

/**************************************************************************************
 * @fn          hal_my_sizeof
 *
 * @brief       This function process for calculate the string length
 *
 * input parameters
 *
 * @param       const uint8_t *str: the source string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      the string length(int)
 **************************************************************************************/
int hal_my_sizeof(const uint8_t *str)
{
	 int size=0;
	
	while(*str != '\0')
	{
		  size++;
		  str++;
	}
  
	return size;
}

/**************************************************************************************
 * @fn          hal_my_strlen
 *
 * @brief       This function process for calculate the string length,PS:the char[] must give the '\0'
 *
 * input parameters
 *
 * @param       const uint8_t *str: the source string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      the string length(int)
 **************************************************************************************/
int hal_my_strlen(const uint8_t *str)
{
	 int len = 0;
   while(str[len] != '\0') ++len;
   return len;
}


/**************************************************************************************
 * @fn          hal_is_an_valid_number
 *
 * @brief       This function process for judge if a char is hex number or not, return  1 means yes, 0 means no
 *
 * input parameters
 *
 * @param       uint8_t ch: the source data     
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      1: yes
 *              0: no
 **************************************************************************************/
int hal_is_an_valid_number(uint8_t ch)
{

if((ch>='0' && ch<='9') || (ch>='a' && ch<='f') || (ch>='A' && ch<='F'))
	 return 1;
else
	 return 0;
	
}



/**************************************************************************************
 * @fn          hal_convert_hex_to_char
 *
 * @brief       This function process for convert a hex data to ASCII code type,include the 0x symbol
 *
 * input parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *              unsigned int data: the source hex data
 *
 * output parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_convert_hex_to_char(unsigned char *ch,  unsigned int data)
{
	 int i;
	
	ch[0]='0';
	ch[1]='x';
	
	for(i=7; i>=0; i--)
	{
		 switch((data & (0xF<<i*4))>>i*4)
		 {
			 case 0:
				   ch[2+7-i]='0';
						break;
			 case 1:
				   ch[2+7-i]='1';
			 			break;
			 case 2:
				   ch[2+7-i]='2';
			 		break;
			 case 3:
				   ch[2+7-i]='3';
			 		break;
			 case 4:
				   ch[2+7-i]='4';
			  		break;
			 case 5:
				   ch[2+7-i]='5';
			 		break;
			 case 6:
				   ch[2+7-i]='6';
			 		break;
			 case 7:
				   ch[2+7-i]='7';
			 		break;
			 case 8:
				   ch[2+7-i]='8';
			 		break;
			 case 9:
				   ch[2+7-i]='9';
			 		break;
			 case 10:
					 ch[2+7-i]='a';
			 		break;
			 case 11:
				   ch[2+7-i]='b';
			 		break;
			 case 12:
				   ch[2+7-i]='c';
			 		break;
			 case 13:
				   ch[2+7-i]='d';
			 		break;
			 case 14:
				   ch[2+7-i]='e';
			 		break;
			 case 15:
				   ch[2+7-i]='f';
			 		break;
			 
			 default:
				  break;
		 }
			 
	}
  ch[10]='\0';	

}


/**************************************************************************************
 * @fn          hal_convert_hex_to_char_wo_x
 *
 * @brief       This function process for convert a hex data to ASCII code type,without the 0x symbol
 *
 * input parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *              unsigned int data: the source hex data
 *
 * output parameters
 *
 * @param       unsigned char *ch: the char string buffer
 *
 * @return      None.
 **************************************************************************************/
void hal_convert_hex_to_char_wo_x(unsigned char *ch,  unsigned int data)
{
	 int i;
	
	for(i=7; i>=0; i--)
	{
		 switch((data & (0xF<<i*4))>>i*4)
		 {
			 case 0:
				   ch[0+7-i]='0';
						break;
			 case 1:
				   ch[0+7-i]='1';
			 			break;
			 case 2:
				   ch[0+7-i]='2';
			 		break;
			 case 3:
				   ch[0+7-i]='3';
			 		break;
			 case 4:
				   ch[0+7-i]='4';
			  		break;
			 case 5:
				   ch[0+7-i]='5';
			 		break;
			 case 6:
				   ch[0+7-i]='6';
			 		break;
			 case 7:
				   ch[0+7-i]='7';
			 		break;
			 case 8:
				   ch[0+7-i]='8';
			 		break;
			 case 9:
				   ch[0+7-i]='9';
			 		break;
			 case 10:
					 ch[0+7-i]='a';
			 		break;
			 case 11:
				   ch[0+7-i]='b';
			 		break;
			 case 12:
				   ch[0+7-i]='c';
			 		break;
			 case 13:
				   ch[0+7-i]='d';
			 		break;
			 case 14:
				   ch[0+7-i]='e';
			 		break;
			 case 15:
				   ch[0+7-i]='f';
			 		break;
			 
			 default:
				  break;
		 }
			 
	}
  ch[8]='\0';	

}



/**************************************************************************************
 * @fn          hal_convert_char_to_hex
 *
 * @brief       This function process for convert a data from ASCII code to hex type
 *
 * input parameters
 *
 * @param       const unsigned char *ch: the source char string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hex data
 **************************************************************************************/
uint32_t hal_convert_char_to_hex(const unsigned char *ch)
{
	 uint32_t  HexData=0;
	 const uint8_t   *p=ch;
	 int       i=0, size;

	 size=hal_my_sizeof (ch);
	 if(size>8)    return 0;
	

  while(i<size && hal_is_an_valid_number(*p))
  {
	   switch(*p++)
	   {
		  case 'f':
			case 'F':
				HexData += 15 << (size-i-1)*4;
				break;
			case 'e':
			case 'E':
				HexData += 14 << (size-i-1)*4;
				break;
			case 'd':
			case 'D':
				HexData += 13<< (size-i-1)*4;
				break;
			case 'c':
			case 'C':
				HexData += 12<< (size-i-1)*4;
				break;
			case 'b':
			case 'B':
				HexData += 11<< (size-i-1)*4;
				break;
			case 'a':
			case 'A':
				HexData += 10<< (size-i-1)*4;
				break;
			case '9':
				HexData += 9<< (size-i-1)*4;
				break;
			case '8':
				HexData += 8<< (size-i-1)*4;
				break;
			case '7':
				HexData += 7<< (size-i-1)*4;
				break;
			case '6':
				HexData += 6<< (size-i-1)*4;
				break;
			case '5':
				HexData += 5<< (size-i-1)*4;
				break;
			case '4':
				HexData += 4<< (size-i-1)*4;
				break;
			case '3':
				HexData += 3<< (size-i-1)*4;
				break;
			case '2':
				HexData += 2<< (size-i-1)*4;
				break;
			case '1':
				HexData += 1<< (size-i-1)*4;
				break;
			case '0':
				HexData += 0<< (size-i-1)*4;
				break;
	   }
		 
		 i++;
		  
 }  
	 
	 return HexData;
   
}


/**************************************************************************************
 * @fn          hal_convert_char_to_dec
 *
 * @brief       This function process for convert a data from ASCII code to decimal type
 *
 * input parameters
 *
 * @param       const unsigned char *ch: the source char string
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      decimal data
 **************************************************************************************/
uint32_t hal_convert_char_to_dec(const unsigned char *ch)
{
	 unsigned int num = 0;
	 while(*ch != '\0'){
		 if(*ch >= '0' && *ch <='9'){
			 num = num*10+(*ch - '0');
			 ch++;
		 }
		 else{
			 num=0;
			 break;
		 }
	 }
   return num;
}

/**************************************************************************************
 * @fn          WaitMs
 *
 * @brief       This function process for wait program msecond,use RTC
 *
 * input parameters
 *
 * @param       uint32_t msecond: the msecond value 
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void WaitMs(uint32_t msecond){
//	uint8_t tick_per_ms = 32;
//	//CLAER_RTC_COUNT;
//	RUN_RTC;
//	uint32_t now_clock_tick = clock_time_rtc();
//	while((clock_time_rtc()-now_clock_tick) < (tick_per_ms * msecond)){
//		if(clock_time_rtc() == 0xffffff){
//			break;
//		}
//	}
//	
    WaitRTCCount((msecond<<15)/1000);

}

/**************************************************************************************
 * @fn          subWriteReg
 *
 * @brief       This function process for write register with sub bit
 *
 * input parameters
 *
 * @param       uint32_t addr: register address 
 *              uint8_t hOff: high bit offset
 *              uint8_t lOff: low bit offset
 *              uint32_t value: write value
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void subWriteReg(uint32_t addr,uint8_t hOff,uint8_t lOff,uint32_t value){
	uint32_t temp=read_reg(addr);
	uint32_t temp2=0xffffffff;
	uint32_t temp3=value << lOff;
	for(uint8_t i=lOff;i<=hOff;i++){
		temp2 &= ~BIT(i);
	}
	temp = temp & temp2;
	temp = temp |temp3;
	write_reg(addr,temp);
}




/**************************************************************************************
 * @fn          hal_system_init
 *
 * @brief       This function process for system initial,you can select diff source,such as RC_32M XTAL_16M and so on
 *
 * input parameters
 *
 * @param       enum H_SYSCLK_SEL h_system_clk_sel: system clock select  SYS_CLK_RC_32M rc32M
 * 																																			 SYS_CLK_DLL_32M dll32M
 *																																			 SYS_CLK_XTAL_16M xtal16M
 *																																			 SYS_CLK_DLL_48M dll48M
 *																																		   SYS_CLK_DLL_64M dll64M
 *																																	     SYS_CLK_DLL_96M dll96M
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_system_init(uint8_t h_system_clk_sel){
    uint16_t delay=300;
	if(h_system_clk_sel == SYS_CLK_RC_32M){
		hclk = 32000000;
	}else if(h_system_clk_sel == SYS_CLK_XTAL_16M){
//		ENABLE_XTAL_TRIGGER;
		ENABLE_XTAL_OUTPUT;
		hclk = 16000000;
	}else if(h_system_clk_sel == SYS_CLK_DLL_32M){
//		ENABLE_XTAL_TRIGGER;
		ENABLE_XTAL_OUTPUT;
		DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>
		DLL32M_CLOCK_ENABLE;
		ENABLE_DLL;
		hclk = 32000000;
	}else{
//		ENABLE_XTAL_TRIGGER;
		ENABLE_XTAL_OUTPUT;
		DLLn_CLOCK_ENABLE(h_system_clk_sel);
		ENABLE_DLL;
		if(h_system_clk_sel == SYS_CLK_DLL_48M){
			hclk = 48000000;
		}else if(h_system_clk_sel == SYS_CLK_DLL_64M){
			hclk = 64000000;
		}else{
			hclk = 96000000;
		}
		
	}

	while (delay > 0)
        delay --;
        
	subWriteReg(0x4000f03c,3,0,h_system_clk_sel);
	if(PCLK_DIV_ENABLE){
		uint32_t divider=(read_reg(0x40000018)>>4)&0xff;
		pclk = hclk/(divider+1);
	}else{
		pclk = hclk;
	}
  
	
}

uint32_t hal_read_current_time(void)
{
	 // return ((4000000-get_timer3_count())/4+2);
     //return (TIME_BASE - (get_timer3_count() >> 2) ) ;
    return(TIME_BASE - ((CP_TIM3->CurrentCount)>>2) );
}

/**************************************************************************************
 * @fn          WaitUs
 *
 * @brief       This function process for wait program usecond,use 4M crystal
 *
 * input parameters
 *
 * @param       uint32_t usecond: the usecond value 
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void WaitUs(uint32_t wtTime){
    
    uint32_t T0,currTick,deltTick;
    T0 = hal_read_current_time();
    
    while(1){
        currTick = hal_read_current_time();
        deltTick =TIME_DELTA(currTick,T0);
        if(deltTick>wtTime)
            break;
    }
}

void WaitRTCCount(uint32_t rtcDelyCnt)
{
    uint32 cnt0,cnt1;
    uint32 delt =0;
    cnt0 = clock_time_rtc();

    while(delt<rtcDelyCnt)
    { 
        cnt1 = clock_time_rtc();

        delt = (cnt1>=cnt0) ? cnt1-cnt0 : (0x00ffffff-cnt0+cnt1);


    }
    
    
}


void hal_system_soft_reset(void)
{
    *(volatile uint32_t *) 0x40000010 = 0x00;
}





