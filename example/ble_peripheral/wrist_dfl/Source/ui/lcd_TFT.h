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


#if(CFG_DISP==DISP_TFT)

#ifndef _LCD_TFT_H
#define _LCD_TFT_H

#include "types.h"

#define SCN_WIDTH   240
#define SCN_HEIGHT  240


#define	RESET_PIN	P19
#define	DC_PIN		P18

#define	LIGHT_LEDK		P24
#define	LIGHT_LEDA		P25



#define RGB_LightPink   0xfdb8  //浅粉红[255,182,193] #FFB6C1
#define RGB_Pink        0xfe19  //粉红[255,192,203] #FFC0CB
#define RGB_Crimson     0xd8a7  //猩红[220,20,60] #DC143C
#define RGB_LavenderBlush       0xff9e  //脸红的淡紫色[255,240,245] #FFF0F5
#define RGB_PaleVioletRed       0xdb92  //苍白的紫罗兰红色[219,112,147] #DB7093
#define RGB_HotPink     0xfb56  //热情的粉红[255,105,180] #FF69B4
#define RGB_DeepPink    0xf8b2  //深粉色[255,20,147] #FF1493
#define RGB_MediumVioletRed     0xc0b0  //适中的紫罗兰红色[199,21,133] #C71585
#define RGB_Orchid      0xdb9a  //兰花的紫色[218,112,214] #DA70D6
#define RGB_Thistle     0xddfb  //蓟[216,191,216] #D8BFD8
#define RGB_plum        0xdd1b  //李子[221,160,221] #DDA0DD
#define RGB_Violet      0xec1d  //紫罗兰[238,130,238] #EE82EE
#define RGB_Magenta     0xf81f  //洋红[255,0,255] #FF00FF
#define RGB_Fuchsia     0xf81f  //灯笼海棠(紫红�?[255,0,255] #FF00FF
#define RGB_DarkMagenta 0x8811  //深洋红色[139,0,139] #8B008B
#define RGB_Purple      0x8010  //紫色[128,0,128] #800080
#define RGB_MediumOrchid        0xbaba  //适中的兰花紫[186,85,211] #BA55D3
#define RGB_DarkVoilet  0x901a  //深紫罗兰色[148,0,211] #9400D3
#define RGB_DarkOrchid  0x9999  //深兰花紫[153,50,204] #9932CC
#define RGB_Indigo      0x4810  //靛青[75,0,130] #4B0082
#define RGB_BlueViolet  0x895c  //深紫罗兰的蓝色[138,43,226] #8A2BE2
#define RGB_MediumPurple        0x939b  //适中的紫色[147,112,219] #9370DB
#define RGB_MediumSlateBlue     0x7b5d  //适中的板岩暗蓝灰色[123,104,238] #7B68EE
#define RGB_SlateBlue   0x6ad9  //板岩暗蓝灰色[106,90,205] #6A5ACD
#define RGB_DarkSlateBlue       0x49f1  //深岩暗蓝灰色[72,61,139] #483D8B
#define RGB_Lavender    0xe73f  //熏衣草花的淡紫色[230,230,250] #E6E6FA
#define RGB_GhostWhite  0xffdf  //幽灵的白色[248,248,255] #F8F8FF
#define RGB_Blue        0x001f  //纯蓝[0,0,255] #0000FF
#define RGB_MediumBlue  0x0019  //适中的蓝色[0,0,205] #0000CD
#define RGB_MidnightBlue        0x18ce  //午夜的蓝色[25,25,112] #191970
#define RGB_DarkBlue    0x0011  //深蓝色[0,0,139] #00008B
#define RGB_Navy        0x0010  //海军蓝[0,0,128] #000080
#define RGB_RoyalBlue   0x435c  //皇军蓝[65,105,225] #4169E1
#define RGB_CornflowerBlue      0x64bd  //矢车菊的蓝色[100,149,237] #6495ED
#define RGB_LightSteelBlue      0xb63b  //淡钢蓝[176,196,222] #B0C4DE
#define RGB_LightSlateGray      0x7453  //浅石板灰[119,136,153] #778899
#define RGB_SlateGray   0x7412  //石板灰[112,128,144] #708090
#define RGB_DoderBlue   0x1c9f  //道奇蓝[30,144,255] #1E90FF
#define RGB_AliceBlue   0xf7df  //爱丽丝蓝[240,248,255] #F0F8FF
#define RGB_SteelBlue   0x4416  //钢蓝[70,130,180] #4682B4
#define RGB_LightSkyBlue        0x867f  //淡蓝色[135,206,250] #87CEFA
#define RGB_SkyBlue     0x867d  //天蓝色[135,206,235] #87CEEB
#define RGB_DeepSkyBlue 0x05ff  //深天蓝[0,191,255] #00BFFF
#define RGB_LightBLue   0xaedc  //淡蓝[173,216,230] #ADD8E6
#define RGB_PowDerBlue  0xb71c  //火药蓝[176,224,230] #B0E0E6
#define RGB_CadetBlue   0x5cf4  //军校蓝[95,158,160] #5F9EA0
#define RGB_Azure       0xf7ff  //蔚蓝色[240,255,255] #F0FFFF
#define RGB_LightCyan   0xe7ff  //淡青色[225,255,255] #E1FFFF
#define RGB_PaleTurquoise       0xaf7d  //苍白的绿宝石[175,238,238] #AFEEEE
#define RGB_Cyan        0x07ff  //青色[0,255,255] #00FFFF
#define RGB_Aqua        0x07ff  //水绿色[0,255,255] #00FFFF
#define RGB_DarkTurquoise       0x067a  //深绿宝石[0,206,209] #00CED1
#define RGB_DarkSlateGray       0x2a69  //深石板灰[47,79,79] #2F4F4F
#define RGB_DarkCyan    0x0451  //深青色[0,139,139] #008B8B
#define RGB_Teal        0x0410  //水鸭色[0,128,128] #008080
#define RGB_MediumTurquoise     0x4e99  //适中的绿宝石[72,209,204] #48D1CC
#define RGB_LightSeaGreen       0x2595  //浅海洋绿[32,178,170] #20B2AA
#define RGB_Turquoise   0x471a  //绿宝石[64,224,208] #40E0D0
#define RGB_Auqamarin   0x7ff5  //绿玉\碧绿色[127,255,170] #7FFFAA
#define RGB_MediumAquamarine    0x07d3  //适中的碧绿色[0,250,154] #00FA9A
#define RGB_MediumSpringGreen   0xf7ff  //适中的春天的绿色[245,255,250] #F5FFFA
#define RGB_MintCream   0x07ef  //薄荷奶油[0,255,127] #00FF7F
#define RGB_SpringGreen 0x3d8e  //春天的绿色[60,179,113] #3CB371
#define RGB_SeaGreen    0x2c4a  //海洋绿[46,139,87] #2E8B57
#define RGB_Honeydew    0xf7fe  //蜂蜜[240,255,240] #F0FFF0
#define RGB_LightGreen  0x9772  //淡绿色[144,238,144] #90EE90
#define RGB_PaleGreen   0x9fd3  //苍白的绿色[152,251,152] #98FB98
#define RGB_DarkSeaGreen        0x8df1  //深海洋绿[143,188,143] #8FBC8F
#define RGB_LimeGreen   0x3666  //酸橙绿[50,205,50] #32CD32
#define RGB_Lime        0x07e0  //酸橙色[0,255,0] #00FF00
#define RGB_ForestGreen 0x2444  //森林绿[34,139,34] #228B22
#define RGB_Green       0x0400  //纯绿[0,128,0] #008000
#define RGB_DarkGreen   0x0320  //深绿色[0,100,0] #006400
#define RGB_Chartreuse  0x7fe0  //查特酒绿[127,255,0] #7FFF00
#define RGB_LawnGreen   0x7fe0  //草坪绿[124,252,0] #7CFC00
#define RGB_GreenYellow 0xafe5  //绿黄色[173,255,47] #ADFF2F
#define RGB_OliveDrab   0x5345  //橄榄土褐色[85,107,47] #556B2F
#define RGB_Beige       0x6c64  //米色(浅褐�?[107,142,35] #6B8E23
#define RGB_LightGoldenrodYellow        0xffda  //浅秋麒麟黄[250,250,210] #FAFAD2
#define RGB_Ivory       0xfffe  //象牙[255,255,240] #FFFFF0
#define RGB_LightYellow 0xfffc  //浅黄色[255,255,224] #FFFFE0
#define RGB_Yellow      0xffe0  //纯黄[255,255,0] #FFFF00
#define RGB_Olive       0x8400  //橄榄[128,128,0] #808000
#define RGB_DarkKhaki   0xbdad  //深卡其布[189,183,107] #BDB76B
#define RGB_LemonChiffon        0xffd9  //柠檬薄纱[255,250,205] #FFFACD
#define RGB_PaleGodenrod        0xef55  //灰秋麒麟[238,232,170] #EEE8AA
#define RGB_Khaki       0xf731  //卡其布[240,230,140] #F0E68C
#define RGB_Gold        0xfea0  //金[255,215,0] #FFD700
#define RGB_Cornislk    0xffdb  //玉米色[255,248,220] #FFF8DC
#define RGB_GoldEnrod   0xdd24  //秋麒麟[218,165,32] #DAA520
#define RGB_FloralWhite 0xffde  //花的白色[255,250,240] #FFFAF0
#define RGB_OldLace     0xffbc  //老饰带[253,245,230] #FDF5E6
#define RGB_Wheat       0xf6f6  //小麦色[245,222,179] #F5DEB3
#define RGB_Moccasin    0xff36  //鹿皮鞋[255,228,181] #FFE4B5
#define RGB_Orange      0xfd20  //橙色[255,165,0] #FFA500
#define RGB_PapayaWhip  0xff7a  //番木瓜[255,239,213] #FFEFD5
#define RGB_BlanchedAlmond      0xff59  //漂白的杏仁[255,235,205] #FFEBCD
#define RGB_NavajoWhite 0xfef5  //Navajo白[255,222,173] #FFDEAD
#define RGB_AntiqueWhite        0xff5a  //古代的白色[250,235,215] #FAEBD7
#define RGB_Tan 0xd5b1  //晒黑[210,180,140] #D2B48C
#define RGB_BrulyWood   0xddd0  //结实的树[222,184,135] #DEB887
#define RGB_Bisque      0xff38  //(浓汤)乳脂,番茄等[255,228,196] #FFE4C4
#define RGB_DarkOrange  0xfc60  //深橙色[255,140,0] #FF8C00
#define RGB_Linen       0xff9c  //亚麻布[250,240,230] #FAF0E6
#define RGB_Peru        0xcc27  //秘鲁[205,133,63] #CD853F
#define RGB_PeachPuff   0xfed7  //桃色[255,218,185] #FFDAB9
#define RGB_SandyBrown  0xf52c  //沙棕色[244,164,96] #F4A460
#define RGB_Chocolate   0xd343  //巧克力[210,105,30] #D2691E
#define RGB_SaddleBrown 0x8a22  //马鞍棕色[139,69,19] #8B4513
#define RGB_SeaShell    0xffbd  //海贝壳[255,245,238] #FFF5EE
#define RGB_Sienna      0xa285  //黄土赭色[160,82,45] #A0522D
#define RGB_LightSalmon 0xfd0f  //浅鲜�?鲑鱼)色[255,160,122] #FFA07A
#define RGB_Coral       0xfbea  //珊瑚[255,127,80] #FF7F50
#define RGB_OrangeRed   0xfa20  //橙红色[255,69,0] #FF4500
#define RGB_DarkSalmon  0xecaf  //深鲜�?鲑鱼)色[233,150,122] #E9967A
#define RGB_Tomato      0xfb08  //番茄[255,99,71] #FF6347
#define RGB_MistyRose   0xff3c  //薄雾玫瑰[255,228,225] #FFE4E1
#define RGB_Salmon      0xfc0e  //鲜肉(鲑鱼)色[250,128,114] #FA8072
#define RGB_Snow        0xffdf  //雪[255,250,250] #FFFAFA
#define RGB_LightCoral  0xf410  //淡珊瑚色[240,128,128] #F08080
#define RGB_RosyBrown   0xbc71  //玫瑰棕色[188,143,143] #BC8F8F
#define RGB_IndianRed   0xcaeb  //印度红[205,92,92] #CD5C5C
#define RGB_Red 0xf800  //纯红[255,0,0] #FF0000
#define RGB_Brown       0xa145  //棕色[165,42,42] #A52A2A
#define RGB_FireBrick   0xb104  //耐火砖[178,34,34] #B22222
#define RGB_DarkRed     0x8800  //深红色[139,0,0] #8B0000
#define RGB_Maroon      0x8000  //栗色[128,0,0] #800000
#define RGB_White       0xffff  //纯白[255,255,255] #FFFFFF
#define RGB_WhiteSmoke  0xf7be  //白烟[245,245,245] #F5F5F5
#define RGB_Gainsboro   0xdefb  //Gainsboro[220,220,220] #DCDCDC
#define RGB_LightGrey   0xd69a  //浅灰色[211,211,211] #D3D3D3
#define RGB_Silver      0xc618  //银白色[192,192,192] #C0C0C0
#define RGB_DarkGray    0xad55  //深灰色[169,169,169] #A9A9A9
#define RGB_Gray        0x8410  //灰色[128,128,128] #808080
#define RGB_DimGray     0x6b4d  //暗淡的灰色[105,105,105] #696969
#define RGB_Black       0x0000  //纯黑[0,0,0] #000000




int lcd_setscn_TFT(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
int lcd_draw_TFT(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint16_t* data);
void lcd_on_TFT(void);
void lcd_off_TFT(void);
int lcd_init(void);
int lcd_bus_init(void);
int lcd_bus_deinit(void);


#endif
#endif /*CFG_DISP=DISP_TFT*/

