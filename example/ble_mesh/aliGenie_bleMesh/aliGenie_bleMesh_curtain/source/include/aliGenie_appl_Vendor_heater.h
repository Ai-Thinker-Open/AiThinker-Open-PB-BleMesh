
#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"

#ifndef _ALIGENIE_APPL_VENDOR_HEATER_H_
#define _ALIGENIE_APPL_VENDOR_HEATER_H_

/************  HEATER Properties  ***********/

/******  ALIGENIE_ATTRIBUTE_CODE  *****/
#define  ALIGENIE_ATTRCODE_HUMIDITY         0x010F
#define  ALIGENIE_ATTRCODE_ERRORCODE        0x0000
#define  ALIGENIE_ATTRCODE_TEMPERATURE      0x010D
#define  ALIGENIE_ATTRCODE_POWERSTATE       0x0100
#define  ALIGENIE_ATTRCODE_HEATERPOWER      0x0174
#define  ALIGENIE_ATTRCODE_RATEOFWORK       0x0107
#define  ALIGENIE_ATTRCODE_HEATSTATUS       0x057C
#define  ALIGENIE_ATTRCODE_CHILDLOCKONOFF   0x050C
#define  ALIGENIE_ATTRCODE_TARGETTEMPERATURE        0x010C
#define  ALIGENIE_ATTRCODE_HUMIDIFICATIONSWITCH     0x0536

/******  ALIGENIE_PROPERTY  *****/
/******  Spec of humidity( 湿度 )  *****/
#define HEATER_HUMIDITY_DEFAULT     100.0
#define HEATER_HUMIDITY_MIN     0.0
#define HEATER_HUMIDITY_MAX     100.0
#define HEATER_HUMIDITY_STEP    10.0

/******  Spec of errorCode( 错误码 )  *****/
enum HEATER_ERRORCODE_T
{
    //错误码
    HEATER_ERRORCODE_128 = 128, //设备未准备好
    HEATER_ERRORCODE_129 = 129, //不支持的属性
    HEATER_ERRORCODE_130 = 130, //不支持的操作
    HEATER_ERRORCODE_131 = 131, //参数错误
    HEATER_ERRORCODE_132 = 132, //设备状态错误
    HEATER_ERRORCODE_133 = 133, //高温提醒
    HEATER_ERRORCODE_134 = 134, //低温提醒
    HEATER_ERRORCODE_135 = 135, //传感器开路
    HEATER_ERRORCODE_136 = 136, //传感器短路
    HEATER_ERRORCODE_137 = 137, //进水温度过低
    HEATER_ERRORCODE_138 = 138, //一氧化碳超标
    HEATER_ERRORCODE_139 = 139, //点火失败
    HEATER_ERRORCODE_140 = 140, //意外熄火
    HEATER_ERRORCODE_141 = 141, //残火
    HEATER_ERRORCODE_142 = 142, //漏电故障
    HEATER_ERRORCODE_143 = 143, //干烧故障
    HEATER_ERRORCODE_144 = 144, //超温故障
    HEATER_ERRORCODE_145 = 145, //传感器故障
    HEATER_ERRORCODE_146 = 146, //感应异常故障
    HEATER_ERRORCODE_147 = 147, //变频微波故障
    HEATER_ERRORCODE_148 = 148, //炉腔过热
    HEATER_ERRORCODE_149 = 149, //正常
    HEATER_ERRORCODE_150 = 150, //内部感温元件短路
    HEATER_ERRORCODE_151 = 151, //内部感温元件开路
    HEATER_ERRORCODE_152 = 152, //内部温度过高
    HEATER_ERRORCODE_153 = 153, //外部感温元件短路
    HEATER_ERRORCODE_154 = 154, //外部感温元件开路
    HEATER_ERRORCODE_155 = 155, //发热丝断开
    HEATER_ERRORCODE_156 = 156, //无线信号弱或断开
    HEATER_ERRORCODE_157 = 157, //监测器信号弱或断开
    HEATER_ERRORCODE_158 = 158, //监测器故障
    HEATER_ERRORCODE_159 = 159, //进水异常
    HEATER_ERRORCODE_160 = 160, //加热异常
    HEATER_ERRORCODE_161 = 161, //溢流报警
    HEATER_ERRORCODE_162 = 162, //通讯故障下控收不到上控数据
    HEATER_ERRORCODE_163 = 163, //失速保护
    HEATER_ERRORCODE_164 = 164, //速度感应失败
    HEATER_ERRORCODE_165 = 165, //扬升学习失败
    HEATER_ERRORCODE_166 = 166, //过载保护
    HEATER_ERRORCODE_167 = 167, //安全开关脱落
    HEATER_ERRORCODE_168 = 168, //通讯故障上控收不到下控数据
    HEATER_ERRORCODE_169 = 169, //电机开路
    HEATER_ERRORCODE_170 = 170, //漏水故障
    HEATER_ERRORCODE_171 = 171, //制水保护
    HEATER_ERRORCODE_172 = 172, //缺水或低压报警
    HEATER_ERRORCODE_173 = 173, //原水箱缺水报警
    HEATER_ERRORCODE_174 = 174, //加热传感故障
    HEATER_ERRORCODE_175 = 175, //制冷传感故障
    HEATER_ERRORCODE_176 = 176, //净水箱高位浮子逻辑故障
    HEATER_ERRORCODE_177 = 177, //净水箱低位浮子逻辑故障
    HEATER_ERRORCODE_178 = 178, //净水箱浮子逻辑故障
    HEATER_ERRORCODE_179 = 179, //板卡通信故障
    HEATER_ERRORCODE_180 = 180, //上报频次异常
    HEATER_ERRORCODE_181 = 181, //传感器过热
    HEATER_ERRORCODE_182 = 182, //工作电压过低
    HEATER_ERRORCODE_183 = 183, //工作电压过高
    HEATER_ERRORCODE_184 = 184, //锅具干烧
    HEATER_ERRORCODE_185 = 185, //主传感器失效
    HEATER_ERRORCODE_186 = 186, //错误一
    HEATER_ERRORCODE_187 = 187, //错误二
    HEATER_ERRORCODE_188 = 188, //错误三
    HEATER_ERRORCODE_189 = 189, //错误四
    HEATER_ERRORCODE_190 = 190, //错误五
    HEATER_ERRORCODE_191 = 191, //错误六
    HEATER_ERRORCODE_192 = 192, //错误七
    HEATER_ERRORCODE_193 = 193, //错误八
    HEATER_ERRORCODE_194 = 194, //错误九
    HEATER_ERRORCODE_195 = 195, //错误十
    HEATER_ERRORCODE_196 = 196, //错误十一
    HEATER_ERRORCODE_197 = 197, //错误十二
    HEATER_ERRORCODE_198 = 198, //错误十三
    HEATER_ERRORCODE_199 = 199, //错误十四
    HEATER_ERRORCODE_200 = 200, //错误十五
    HEATER_ERRORCODE_201 = 201, //错误十六
    HEATER_ERRORCODE_202 = 202, //错误十七
    HEATER_ERRORCODE_203 = 203, //错误十八
    HEATER_ERRORCODE_204 = 204, //错误十九
    HEATER_ERRORCODE_205 = 205, //错误二十
    HEATER_ERRORCODE_206 = 206, //错误二十一
    HEATER_ERRORCODE_207 = 207, //错误二十二
    HEATER_ERRORCODE_208 = 208, //错误二十三
    HEATER_ERRORCODE_209 = 209, //错误二十四
    HEATER_ERRORCODE_210 = 210, //错误二十五
    HEATER_ERRORCODE_211 = 211, //错误二十六
    HEATER_ERRORCODE_212 = 212, //错误二十七
    HEATER_ERRORCODE_213 = 213, //错误二十八
    HEATER_ERRORCODE_214 = 214, //错误二十九
    HEATER_ERRORCODE_215 = 215, //错误三十
    HEATER_ERRORCODE_216 = 216, //错误三十一
    HEATER_ERRORCODE_217 = 217, //错误三十二
    HEATER_ERRORCODE_218 = 218, //错误三十三
    HEATER_ERRORCODE_219 = 219, //错误三十四
    HEATER_ERRORCODE_220 = 220, //错误三十五
    HEATER_ERRORCODE_221 = 221, //错误三十六
    HEATER_ERRORCODE_222 = 222, //错误三十七
    HEATER_ERRORCODE_223 = 223  //错误三十八
};

/******  Spec of temperature( 温度 )  *****/
#define HEATER_TEMPERATURE_DEFAULT  60
#define HEATER_TEMPERATURE_MIN  1
#define HEATER_TEMPERATURE_MAX  100
#define HEATER_TEMPERATURE_STEP     1

/******  Spec of powerstate( 开关 )  *****/
#define  HEATER_POWERSTATE_OFF   0  //关闭
#define  HEATER_POWERSTATE_ON    1  //打开

/******  Spec of heaterPower( 加热档位 )  *****/
#define HEATER_HEATERPOWER_DEFAULT      1
#define HEATER_HEATERPOWER_UNIT         "gear"
#define HEATER_HEATERPOWER_UNITNAME     "档"
#define HEATER_HEATERPOWER_MIN          0
#define HEATER_HEATERPOWER_MAX          10
#define HEATER_HEATERPOWER_STEP         1

/******  Spec of rateofwork( 功率 )  *****/
#define HEATER_RATEOFWORK_DEFAULT   0.0
#define HEATER_RATEOFWORK_MIN       0.0
#define HEATER_RATEOFWORK_MAX       6553.5
#define HEATER_RATEOFWORK_STEP      0.1

/******  Spec of heatStatus( 加热状态 )  *****/
#define HEATER_HEATSTATUS_DEFAULT   1
#define HEATER_HEATSTATUS_UNIT      "gear"
#define HEATER_HEATSTATUS_UNITNAME  "档"
#define HEATER_HEATSTATUS_MIN       1
#define HEATER_HEATSTATUS_MAX       6
#define HEATER_HEATSTATUS_STEP      1

/******  Spec of childLockOnOff( 童锁功能 )  *****/
#define  HEATER_CHILDLOCKONOFF_OFF   0  //关闭
#define  HEATER_CHILDLOCKONOFF_ON    1  //开启

/******  Spec of targetTemperature( 目标温度 )  *****/
#define HEATER_TARGETTEMPERATURE_DEFAULT    0.0
#define HEATER_TARGETTEMPERATURE_MIN    0.0
#define HEATER_TARGETTEMPERATURE_MAX    655.35
#define HEATER_TARGETTEMPERATURE_STEP   1.0

/******  Spec of humidificationSwitch( 加湿开关 )  *****/
#define  HEATER_HUMIDIFICATIONSWITCH_OFF     0  //关闭
#define  HEATER_HUMIDIFICATIONSWITCH_ON      1  //打开

API_RESULT heater_powerstate_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT UI_HARD_generic_onoff_set(UI_DATA_GENERIC_ONOFF_MODEL_T* me, UINT8 state);
API_RESULT heater_humidity_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_errorCode_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_temperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);

API_RESULT heater_heaterPower_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_rateofwork_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_heatStatus_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_childLockOnOff_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_targetTemperature_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT heater_humidificationSwitch_set(UI_DATA_ALIGENIE_MODEL_T* me, UINT32 data);
API_RESULT init_attr_tbl_heater(UI_DATA_ALIGENIE_MODEL_T* me);
#endif //_ALIGENIE_APPL_VENDOR_HEATER_H_
