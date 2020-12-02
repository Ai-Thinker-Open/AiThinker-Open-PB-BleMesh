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

#ifndef RES_TAB_H
#define RES_TAB_H


#define NUM_BITMAP	178

/* start bitmap define */
#define IDB_ASC_00H	0
#define IDB_ASC_01H	1
#define IDB_ASC_02H	2
#define IDB_ASC_03H	3
#define IDB_ASC_04H	4
#define IDB_ASC_05H	5
#define IDB_ASC_06H	6
#define IDB_ASC_07H	7
#define IDB_ASC_08H	8
#define IDB_ASC_09H	9
#define IDB_ASC_0AH	10
#define IDB_ASC_0BH	11
#define IDB_ASC_0CH	12
#define IDB_ASC_0DH	13
#define IDB_ASC_0EH	14
#define IDB_ASC_0FH	15
#define IDB_ASC_10H	16
#define IDB_ASC_11H	17
#define IDB_ASC_12H	18
#define IDB_ASC_13H	19
#define IDB_ASC_14H	20
#define IDB_ASC_15H	21
#define IDB_ASC_16H	22
#define IDB_ASC_17H	23
#define IDB_ASC_18H	24
#define IDB_ASC_19H	25
#define IDB_ASC_1AH	26
#define IDB_ASC_1BH	27
#define IDB_ASC_1CH	28
#define IDB_ASC_1DH	29
#define IDB_ASC_1EH	30
#define IDB_ASC_1FH	31
#define IDB_ASC_20H	32
#define IDB_ASC_21H	33
#define IDB_ASC_22H	34
#define IDB_ASC_23H	35
#define IDB_ASC_24H	36
#define IDB_ASC_25H	37
#define IDB_ASC_26H	38
#define IDB_ASC_27H	39
#define IDB_ASC_28H	40
#define IDB_ASC_29H	41
#define IDB_ASC_2AH	42
#define IDB_ASC_2BH	43
#define IDB_ASC_2CH	44
#define IDB_ASC_2DH	45
#define IDB_ASC_2EH	46
#define IDB_ASC_2FH	47
#define IDB_ASC_30H	48
#define IDB_ASC_31H	49
#define IDB_ASC_32H	50
#define IDB_ASC_33H	51
#define IDB_ASC_34H	52
#define IDB_ASC_35H	53
#define IDB_ASC_36H	54
#define IDB_ASC_37H	55
#define IDB_ASC_38H	56
#define IDB_ASC_39H	57
#define IDB_ASC_3AH	58
#define IDB_ASC_3BH	59
#define IDB_ASC_3CH	60
#define IDB_ASC_3DH	61
#define IDB_ASC_3EH	62
#define IDB_ASC_3FH	63
#define IDB_ASC_40H	64
#define IDB_ASC_41H	65
#define IDB_ASC_42H	66
#define IDB_ASC_43H	67
#define IDB_ASC_44H	68
#define IDB_ASC_45H	69
#define IDB_ASC_46H	70
#define IDB_ASC_47H	71
#define IDB_ASC_48H	72
#define IDB_ASC_49H	73
#define IDB_ASC_4AH	74
#define IDB_ASC_4BH	75
#define IDB_ASC_4CH	76
#define IDB_ASC_4DH	77
#define IDB_ASC_4EH	78
#define IDB_ASC_4FH	79
#define IDB_ASC_50H	80
#define IDB_ASC_51H	81
#define IDB_ASC_52H	82
#define IDB_ASC_53H	83
#define IDB_ASC_54H	84
#define IDB_ASC_55H	85
#define IDB_ASC_56H	86
#define IDB_ASC_57H	87
#define IDB_ASC_58H	88
#define IDB_ASC_59H	89
#define IDB_ASC_5AH	90
#define IDB_ASC_5BH	91
#define IDB_ASC_5CH	92
#define IDB_ASC_5DH	93
#define IDB_ASC_5EH	94
#define IDB_ASC_5FH	95
#define IDB_ICON_distance	96
#define IDB_ICON_sms	97
#define IDB_ICON_call	98
#define IDB_ICON_qq	99
#define IDB_ICON_sedentary	100
#define IDB_ICON_sleep	101
#define IDB_ICON_sleep1	102
#define IDB_ICON_sleep2	103
#define IDB_ICON_steps	104
#define IDB_ICON_steps1	105
#define IDB_ICON_wechat	106
#define IDB_ICON_batt	107
#define IDB_ICON_batt0	108
#define IDB_ICON_batt1	109
#define IDB_ICON_batt2	110
#define IDB_ICON_batt3	111
#define IDB_ICON_batt4	112
#define IDB_ICON_batt5	113
#define IDB_ICON_blood_pressure	114
#define IDB_ICON_fw_upgrade	115
#define IDB_ICON_kalorie	116
#define IDB_ICON_heartrate	117
#define IDB_ICON_heartrate1	118
#define IDB_ICON_heartrate_w	119
#define IDB_ICON_heartrate_x	120
#define IDB_ICON_alarmclock	121
#define IDB_ICON_press0	122
#define IDB_ICON_press1	123
#define IDB_N1_0	124
#define IDB_N1_1	125
#define IDB_N1_2	126
#define IDB_N1_3	127
#define IDB_N1_4	128
#define IDB_N1_5	129
#define IDB_N1_6	130
#define IDB_N1_7	131
#define IDB_N1_8	132
#define IDB_N1_9	133
#define IDB_N1_DOT	134
#define IDB_N2_0	135
#define IDB_N2_1	136
#define IDB_N2_2	137
#define IDB_N2_3	138
#define IDB_N2_4	139
#define IDB_N2_5	140
#define IDB_N2_6	141
#define IDB_N2_7	142
#define IDB_N2_8	143
#define IDB_N2_9	144
#define IDB_N3_0	145
#define IDB_N3_1	146
#define IDB_N3_2	147
#define IDB_N3_3	148
#define IDB_N3_4	149
#define IDB_N3_5	150
#define IDB_N3_6	151
#define IDB_N3_7	152
#define IDB_N3_8	153
#define IDB_N3_9	154
#define IDB_TM_0	155
#define IDB_TM_1	156
#define IDB_TM_2	157
#define IDB_TM_3	158
#define IDB_TM_4	159
#define IDB_TM_5	160
#define IDB_TM_6	161
#define IDB_TM_7	162
#define IDB_TM_8	163
#define IDB_TM_9	164
#define IDB_TM_COLON	165
#define IDB_SUFFIX_bpm	166
#define IDB_SUFFIX_km	167
#define IDB_SUFFIX_mmHg	168
#define IDB_SUFFIX_h	169
#define IDB_W_HEAD	170
#define IDB_W_1	171
#define IDB_W_2	172
#define IDB_W_3	173
#define IDB_W_4	174
#define IDB_W_5	175
#define IDB_W_6	176
#define IDB_W_7	177
/* end bitmap define */
extern const unsigned char*  bmp_table[NUM_BITMAP];

#endif
