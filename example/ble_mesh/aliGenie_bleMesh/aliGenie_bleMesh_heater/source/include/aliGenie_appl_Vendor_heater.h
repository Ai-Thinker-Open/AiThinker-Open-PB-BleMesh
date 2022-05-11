
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
/******  Spec of humidity( ʪ�� )  *****/
#define HEATER_HUMIDITY_DEFAULT     100.0
#define HEATER_HUMIDITY_MIN     0.0
#define HEATER_HUMIDITY_MAX     100.0
#define HEATER_HUMIDITY_STEP    10.0

/******  Spec of errorCode( ������ )  *****/
enum HEATER_ERRORCODE_T
{
    //������
    HEATER_ERRORCODE_128 = 128, //�豸δ׼����
    HEATER_ERRORCODE_129 = 129, //��֧�ֵ�����
    HEATER_ERRORCODE_130 = 130, //��֧�ֵĲ���
    HEATER_ERRORCODE_131 = 131, //��������
    HEATER_ERRORCODE_132 = 132, //�豸״̬����
    HEATER_ERRORCODE_133 = 133, //��������
    HEATER_ERRORCODE_134 = 134, //��������
    HEATER_ERRORCODE_135 = 135, //��������·
    HEATER_ERRORCODE_136 = 136, //��������·
    HEATER_ERRORCODE_137 = 137, //��ˮ�¶ȹ���
    HEATER_ERRORCODE_138 = 138, //һ����̼����
    HEATER_ERRORCODE_139 = 139, //���ʧ��
    HEATER_ERRORCODE_140 = 140, //����Ϩ��
    HEATER_ERRORCODE_141 = 141, //�л�
    HEATER_ERRORCODE_142 = 142, //©�����
    HEATER_ERRORCODE_143 = 143, //���չ���
    HEATER_ERRORCODE_144 = 144, //���¹���
    HEATER_ERRORCODE_145 = 145, //����������
    HEATER_ERRORCODE_146 = 146, //��Ӧ�쳣����
    HEATER_ERRORCODE_147 = 147, //��Ƶ΢������
    HEATER_ERRORCODE_148 = 148, //¯ǻ����
    HEATER_ERRORCODE_149 = 149, //����
    HEATER_ERRORCODE_150 = 150, //�ڲ�����Ԫ����·
    HEATER_ERRORCODE_151 = 151, //�ڲ�����Ԫ����·
    HEATER_ERRORCODE_152 = 152, //�ڲ��¶ȹ���
    HEATER_ERRORCODE_153 = 153, //�ⲿ����Ԫ����·
    HEATER_ERRORCODE_154 = 154, //�ⲿ����Ԫ����·
    HEATER_ERRORCODE_155 = 155, //����˿�Ͽ�
    HEATER_ERRORCODE_156 = 156, //�����ź�����Ͽ�
    HEATER_ERRORCODE_157 = 157, //������ź�����Ͽ�
    HEATER_ERRORCODE_158 = 158, //���������
    HEATER_ERRORCODE_159 = 159, //��ˮ�쳣
    HEATER_ERRORCODE_160 = 160, //�����쳣
    HEATER_ERRORCODE_161 = 161, //��������
    HEATER_ERRORCODE_162 = 162, //ͨѶ�����¿��ղ����Ͽ�����
    HEATER_ERRORCODE_163 = 163, //ʧ�ٱ���
    HEATER_ERRORCODE_164 = 164, //�ٶȸ�Ӧʧ��
    HEATER_ERRORCODE_165 = 165, //����ѧϰʧ��
    HEATER_ERRORCODE_166 = 166, //���ر���
    HEATER_ERRORCODE_167 = 167, //��ȫ��������
    HEATER_ERRORCODE_168 = 168, //ͨѶ�����Ͽ��ղ����¿�����
    HEATER_ERRORCODE_169 = 169, //�����·
    HEATER_ERRORCODE_170 = 170, //©ˮ����
    HEATER_ERRORCODE_171 = 171, //��ˮ����
    HEATER_ERRORCODE_172 = 172, //ȱˮ���ѹ����
    HEATER_ERRORCODE_173 = 173, //ԭˮ��ȱˮ����
    HEATER_ERRORCODE_174 = 174, //���ȴ��й���
    HEATER_ERRORCODE_175 = 175, //���䴫�й���
    HEATER_ERRORCODE_176 = 176, //��ˮ���λ�����߼�����
    HEATER_ERRORCODE_177 = 177, //��ˮ���λ�����߼�����
    HEATER_ERRORCODE_178 = 178, //��ˮ�両���߼�����
    HEATER_ERRORCODE_179 = 179, //�忨ͨ�Ź���
    HEATER_ERRORCODE_180 = 180, //�ϱ�Ƶ���쳣
    HEATER_ERRORCODE_181 = 181, //����������
    HEATER_ERRORCODE_182 = 182, //������ѹ����
    HEATER_ERRORCODE_183 = 183, //������ѹ����
    HEATER_ERRORCODE_184 = 184, //���߸���
    HEATER_ERRORCODE_185 = 185, //��������ʧЧ
    HEATER_ERRORCODE_186 = 186, //����һ
    HEATER_ERRORCODE_187 = 187, //�����
    HEATER_ERRORCODE_188 = 188, //������
    HEATER_ERRORCODE_189 = 189, //������
    HEATER_ERRORCODE_190 = 190, //������
    HEATER_ERRORCODE_191 = 191, //������
    HEATER_ERRORCODE_192 = 192, //������
    HEATER_ERRORCODE_193 = 193, //�����
    HEATER_ERRORCODE_194 = 194, //�����
    HEATER_ERRORCODE_195 = 195, //����ʮ
    HEATER_ERRORCODE_196 = 196, //����ʮһ
    HEATER_ERRORCODE_197 = 197, //����ʮ��
    HEATER_ERRORCODE_198 = 198, //����ʮ��
    HEATER_ERRORCODE_199 = 199, //����ʮ��
    HEATER_ERRORCODE_200 = 200, //����ʮ��
    HEATER_ERRORCODE_201 = 201, //����ʮ��
    HEATER_ERRORCODE_202 = 202, //����ʮ��
    HEATER_ERRORCODE_203 = 203, //����ʮ��
    HEATER_ERRORCODE_204 = 204, //����ʮ��
    HEATER_ERRORCODE_205 = 205, //�����ʮ
    HEATER_ERRORCODE_206 = 206, //�����ʮһ
    HEATER_ERRORCODE_207 = 207, //�����ʮ��
    HEATER_ERRORCODE_208 = 208, //�����ʮ��
    HEATER_ERRORCODE_209 = 209, //�����ʮ��
    HEATER_ERRORCODE_210 = 210, //�����ʮ��
    HEATER_ERRORCODE_211 = 211, //�����ʮ��
    HEATER_ERRORCODE_212 = 212, //�����ʮ��
    HEATER_ERRORCODE_213 = 213, //�����ʮ��
    HEATER_ERRORCODE_214 = 214, //�����ʮ��
    HEATER_ERRORCODE_215 = 215, //������ʮ
    HEATER_ERRORCODE_216 = 216, //������ʮһ
    HEATER_ERRORCODE_217 = 217, //������ʮ��
    HEATER_ERRORCODE_218 = 218, //������ʮ��
    HEATER_ERRORCODE_219 = 219, //������ʮ��
    HEATER_ERRORCODE_220 = 220, //������ʮ��
    HEATER_ERRORCODE_221 = 221, //������ʮ��
    HEATER_ERRORCODE_222 = 222, //������ʮ��
    HEATER_ERRORCODE_223 = 223  //������ʮ��
};

/******  Spec of temperature( �¶� )  *****/
#define HEATER_TEMPERATURE_DEFAULT  60
#define HEATER_TEMPERATURE_MIN  0
#define HEATER_TEMPERATURE_MAX  100
#define HEATER_TEMPERATURE_STEP     1

/******  Spec of powerstate( ���� )  *****/
#define  HEATER_POWERSTATE_OFF   0  //�ر�
#define  HEATER_POWERSTATE_ON    1  //��

/******  Spec of heaterPower( ���ȵ�λ )  *****/
#define HEATER_HEATERPOWER_DEFAULT      1
#define HEATER_HEATERPOWER_UNIT         "gear"
#define HEATER_HEATERPOWER_UNITNAME     "��"
#define HEATER_HEATERPOWER_MIN          0
#define HEATER_HEATERPOWER_MAX          10
#define HEATER_HEATERPOWER_STEP         1

/******  Spec of rateofwork( ���� )  *****/
#define HEATER_RATEOFWORK_DEFAULT   0.0
#define HEATER_RATEOFWORK_MIN       0.0
#define HEATER_RATEOFWORK_MAX       6553.5
#define HEATER_RATEOFWORK_STEP      0.1

/******  Spec of heatStatus( ����״̬ )  *****/
#define HEATER_HEATSTATUS_DEFAULT   1
#define HEATER_HEATSTATUS_UNIT      "gear"
#define HEATER_HEATSTATUS_UNITNAME  "��"
#define HEATER_HEATSTATUS_MIN       1
#define HEATER_HEATSTATUS_MAX       6
#define HEATER_HEATSTATUS_STEP      1

/******  Spec of childLockOnOff( ͯ������ )  *****/
#define  HEATER_CHILDLOCKONOFF_OFF   0  //�ر�
#define  HEATER_CHILDLOCKONOFF_ON    1  //����

/******  Spec of targetTemperature( Ŀ���¶� )  *****/
#define HEATER_TARGETTEMPERATURE_DEFAULT    0.0
#define HEATER_TARGETTEMPERATURE_MIN    0.0
#define HEATER_TARGETTEMPERATURE_MAX    655.35
#define HEATER_TARGETTEMPERATURE_STEP   1.0

/******  Spec of humidificationSwitch( ��ʪ���� )  *****/
#define  HEATER_HUMIDIFICATIONSWITCH_OFF     0  //�ر�
#define  HEATER_HUMIDIFICATIONSWITCH_ON      1  //��

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
