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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "epb_MmBp.h"

#define TAG_BaseResponse_ErrCode												0x08
#define TAG_BaseResponse_ErrMsg													0x12

#define TAG_AuthRequest_BaseRequest												0x0a
#define TAG_AuthRequest_Md5DeviceTypeAndDeviceId								0x12
#define TAG_AuthRequest_ProtoVersion											0x18
#define TAG_AuthRequest_AuthProto												0x20
#define TAG_AuthRequest_AuthMethod												0x28
#define TAG_AuthRequest_AesSign													0x32
#define TAG_AuthRequest_MacAddress												0x3a
#define TAG_AuthRequest_TimeZone												0x52
#define TAG_AuthRequest_Language												0x5a
#define TAG_AuthRequest_DeviceName												0x62

#define TAG_AuthResponse_BaseResponse											0x0a
#define TAG_AuthResponse_AesSessionKey											0x12

#define TAG_InitRequest_BaseRequest												0x0a
#define TAG_InitRequest_RespFieldFilter											0x12
#define TAG_InitRequest_Challenge												0x1a

#define TAG_InitResponse_BaseResponse											0x0a
#define TAG_InitResponse_UserIdHigh												0x10
#define TAG_InitResponse_UserIdLow												0x18
#define TAG_InitResponse_ChalleangeAnswer										0x20
#define TAG_InitResponse_InitScence												0x28
#define TAG_InitResponse_AutoSyncMaxDurationSecond								0x30
#define TAG_InitResponse_UserNickName											0x5a
#define TAG_InitResponse_PlatformType											0x60
#define TAG_InitResponse_Model													0x6a
#define TAG_InitResponse_Os														0x72
#define TAG_InitResponse_Time													0x78
#define TAG_InitResponse_TimeZone												0x8001
#define TAG_InitResponse_TimeString												0x8a01

#define TAG_SendDataRequest_BaseRequest											0x0a
#define TAG_SendDataRequest_Data												0x12
#define TAG_SendDataRequest_Type												0x18

#define TAG_SendDataResponse_BaseResponse										0x0a
#define TAG_SendDataResponse_Data												0x12

#define TAG_RecvDataPush_BasePush												0x0a
#define TAG_RecvDataPush_Data													0x12
#define TAG_RecvDataPush_Type													0x18

#define TAG_SwitchViewPush_BasePush												0x0a
#define TAG_SwitchViewPush_SwitchViewOp											0x10
#define TAG_SwitchViewPush_ViewId												0x18

#define TAG_SwitchBackgroudPush_BasePush										0x0a
#define TAG_SwitchBackgroudPush_SwitchBackgroundOp								0x10


int epb_base_request_pack_size(BaseRequest *request)
{
	int pack_size = 0;

	return pack_size;
}

int epb_pack_base_request(BaseRequest *request, uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_pack_init(&epb, buf, buf_len);

	return epb_get_packed_size(&epb);
}

BaseResponse *epb_unpack_base_response(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	if (!epb_has_tag(&epb, TAG_BaseResponse_ErrCode)) {
		return NULL;
	}

	BaseResponse *response = (BaseResponse *)malloc(sizeof(BaseResponse));
	memset(response, 0, sizeof(BaseResponse));
	response->err_code = epb_get_int32(&epb, TAG_BaseResponse_ErrCode);
	if (epb_has_tag(&epb, TAG_BaseResponse_ErrMsg)) {
		response->err_msg.str = epb_get_string(&epb, TAG_BaseResponse_ErrMsg, &response->err_msg.len);
		response->has_err_msg = true;
	}

	return response;
}

void epb_unpack_base_response_free(BaseResponse *response)
{
	free(response);
}

BasePush *epb_unpack_base_push(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	BasePush *push = (BasePush *)malloc(sizeof(BasePush));
	memset(push, 0, sizeof(BasePush));

	return push;
}

void epb_unpack_base_push_free(BasePush *push)
{
	free(push);
}

int epb_auth_request_pack_size(AuthRequest *request)
{
	int pack_size = 0;
	int len = 0;

	len = epb_base_request_pack_size(request->base_request);
	pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_BaseRequest, len);
	if (request->has_md5_device_type_and_device_id) {
		pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_Md5DeviceTypeAndDeviceId, request->md5_device_type_and_device_id.len);
	}
	pack_size += epb_varint32_pack_size(TAG_AuthRequest_ProtoVersion, request->proto_version, false);
	pack_size += epb_varint32_pack_size(TAG_AuthRequest_AuthProto, request->auth_proto, false);
	pack_size += epb_varint32_pack_size(TAG_AuthRequest_AuthMethod, request->auth_method, false);
	if (request->has_aes_sign) {
		pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_AesSign, request->aes_sign.len);
	}
	if (request->has_mac_address) {
		pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_MacAddress, request->mac_address.len);
	}
	if (request->has_time_zone) {
		pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_TimeZone, request->time_zone.len);
	}
	if (request->has_language) {
		pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_Language, request->language.len);
	}
	if (request->has_device_name) {
		pack_size += epb_length_delimited_pack_size(TAG_AuthRequest_DeviceName, request->device_name.len);
	}

	return pack_size;
}

int epb_pack_auth_request(AuthRequest *request, uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_pack_init(&epb, buf, buf_len);

	int ret;
	int tmp_len;
	uint8_t *tmp;

	tmp_len = epb_base_request_pack_size(request->base_request);
	tmp = (uint8_t *)malloc(tmp_len);
	ret = epb_pack_base_request(request->base_request, tmp, tmp_len);
	if (ret < 0) {
		free(tmp);
		return ret;
	}
	ret = epb_set_message(&epb, TAG_AuthRequest_BaseRequest, tmp, tmp_len);
	free(tmp);
	if (ret < 0) return ret;
	if (request->has_md5_device_type_and_device_id) {
		ret = epb_set_bytes(&epb, TAG_AuthRequest_Md5DeviceTypeAndDeviceId, request->md5_device_type_and_device_id.data, request->md5_device_type_and_device_id.len);
		if (ret < 0) return ret;
	}
	ret = epb_set_int32(&epb, TAG_AuthRequest_ProtoVersion, request->proto_version);
	if (ret < 0) return ret;
	ret = epb_set_int32(&epb, TAG_AuthRequest_AuthProto, request->auth_proto);
	if (ret < 0) return ret;
	ret = epb_set_enum(&epb, TAG_AuthRequest_AuthMethod, request->auth_method);
	if (ret < 0) return ret;
	if (request->has_aes_sign) {
		ret = epb_set_bytes(&epb, TAG_AuthRequest_AesSign, request->aes_sign.data, request->aes_sign.len);
		if (ret < 0) return ret;
	}
	if (request->has_mac_address) {
		ret = epb_set_bytes(&epb, TAG_AuthRequest_MacAddress, request->mac_address.data, request->mac_address.len);
		if (ret < 0) return ret;
	}
	if (request->has_time_zone) {
		ret = epb_set_string(&epb, TAG_AuthRequest_TimeZone, request->time_zone.str, request->time_zone.len);
		if (ret < 0) return ret;
	}
	if (request->has_language) {
		ret = epb_set_string(&epb, TAG_AuthRequest_Language, request->language.str, request->language.len);
		if (ret < 0) return ret;
	}
	if (request->has_device_name) {
		ret = epb_set_string(&epb, TAG_AuthRequest_DeviceName, request->device_name.str, request->device_name.len);
		if (ret < 0) return ret;
	}

	return epb_get_packed_size(&epb);
}

AuthResponse *epb_unpack_auth_response(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	const uint8_t *tmp;
	int tmp_len;

	if (!epb_has_tag(&epb, TAG_AuthResponse_BaseResponse)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_AuthResponse_AesSessionKey)) {
		return NULL;
	}

	AuthResponse *response = (AuthResponse *)malloc(sizeof(AuthResponse));
	memset(response, 0, sizeof(AuthResponse));
	tmp = epb_get_message(&epb, TAG_AuthResponse_BaseResponse, &tmp_len);
	response->base_response = epb_unpack_base_response(tmp, tmp_len);
	if (response->base_response == NULL) {
		free(response);
		return NULL;
	}
	response->aes_session_key.data = epb_get_bytes(&epb, TAG_AuthResponse_AesSessionKey, &response->aes_session_key.len);

	return response;
}

void epb_unpack_auth_response_free(AuthResponse *response)
{
	epb_unpack_base_response_free(response->base_response);
	free(response);
}

int epb_init_request_pack_size(InitRequest *request)
{
	int pack_size = 0;
	int len = 0;

	len = epb_base_request_pack_size(request->base_request);
	pack_size += epb_length_delimited_pack_size(TAG_InitRequest_BaseRequest, len);
	if (request->has_resp_field_filter) {
		pack_size += epb_length_delimited_pack_size(TAG_InitRequest_RespFieldFilter, request->resp_field_filter.len);
	}
	if (request->has_challenge) {
		pack_size += epb_length_delimited_pack_size(TAG_InitRequest_Challenge, request->challenge.len);
	}

	return pack_size;
}

int epb_pack_init_request(InitRequest *request, uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_pack_init(&epb, buf, buf_len);

	int ret;
	int tmp_len;
	uint8_t *tmp;

	tmp_len = epb_base_request_pack_size(request->base_request);
	tmp = (uint8_t *)malloc(tmp_len);
	ret = epb_pack_base_request(request->base_request, tmp, tmp_len);
	if (ret < 0) {
		free(tmp);
		return ret;
	}
	ret = epb_set_message(&epb, TAG_InitRequest_BaseRequest, tmp, tmp_len);
	free(tmp);
	if (ret < 0) return ret;
	if (request->has_resp_field_filter) {
		ret = epb_set_bytes(&epb, TAG_InitRequest_RespFieldFilter, request->resp_field_filter.data, request->resp_field_filter.len);
		if (ret < 0) return ret;
	}
	if (request->has_challenge) {
		ret = epb_set_bytes(&epb, TAG_InitRequest_Challenge, request->challenge.data, request->challenge.len);
		if (ret < 0) return ret;
	}

	return epb_get_packed_size(&epb);
}

InitResponse *epb_unpack_init_response(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	const uint8_t *tmp;
	int tmp_len;

	if (!epb_has_tag(&epb, TAG_InitResponse_BaseResponse)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_InitResponse_UserIdHigh)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_InitResponse_UserIdLow)) {
		return NULL;
	}

	InitResponse *response = (InitResponse *)malloc(sizeof(InitResponse));
	memset(response, 0, sizeof(InitResponse));
	tmp = epb_get_message(&epb, TAG_InitResponse_BaseResponse, &tmp_len);
	response->base_response = epb_unpack_base_response(tmp, tmp_len);
	if (response->base_response == NULL) {
		free(response);
		return NULL;
	}
	response->user_id_high = epb_get_uint32(&epb, TAG_InitResponse_UserIdHigh);
	response->user_id_low = epb_get_uint32(&epb, TAG_InitResponse_UserIdLow);
	if (epb_has_tag(&epb, TAG_InitResponse_ChalleangeAnswer)) {
		response->challeange_answer = epb_get_uint32(&epb, TAG_InitResponse_ChalleangeAnswer);
		response->has_challeange_answer = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_InitScence)) {
		response->init_scence = (EmInitScence)epb_get_enum(&epb, TAG_InitResponse_InitScence);
		response->has_init_scence = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_AutoSyncMaxDurationSecond)) {
		response->auto_sync_max_duration_second = epb_get_uint32(&epb, TAG_InitResponse_AutoSyncMaxDurationSecond);
		response->has_auto_sync_max_duration_second = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_UserNickName)) {
		response->user_nick_name.str = epb_get_string(&epb, TAG_InitResponse_UserNickName, &response->user_nick_name.len);
		response->has_user_nick_name = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_PlatformType)) {
		response->platform_type = (EmPlatformType)epb_get_enum(&epb, TAG_InitResponse_PlatformType);
		response->has_platform_type = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_Model)) {
		response->model.str = epb_get_string(&epb, TAG_InitResponse_Model, &response->model.len);
		response->has_model = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_Os)) {
		response->os.str = epb_get_string(&epb, TAG_InitResponse_Os, &response->os.len);
		response->has_os = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_Time)) {
		response->time = epb_get_int32(&epb, TAG_InitResponse_Time);
		response->has_time = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_TimeZone)) {
		response->time_zone = epb_get_int32(&epb, TAG_InitResponse_TimeZone);
		response->has_time_zone = true;
	}
	if (epb_has_tag(&epb, TAG_InitResponse_TimeString)) {
		response->time_string.str = epb_get_string(&epb, TAG_InitResponse_TimeString, &response->time_string.len);
		response->has_time_string = true;
	}

	return response;
}

void epb_unpack_init_response_free(InitResponse *response)
{
	epb_unpack_base_response_free(response->base_response);
	free(response);
}

int epb_send_data_request_pack_size(SendDataRequest *request)
{
	int pack_size = 0;
	int len = 0;

	len = epb_base_request_pack_size(request->base_request);
	pack_size += epb_length_delimited_pack_size(TAG_SendDataRequest_BaseRequest, len);
	pack_size += epb_length_delimited_pack_size(TAG_SendDataRequest_Data, request->data.len);
	if (request->has_type) {
		pack_size += epb_varint32_pack_size(TAG_SendDataRequest_Type, request->type, false);
	}

	return pack_size;
}

int epb_pack_send_data_request(SendDataRequest *request, uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_pack_init(&epb, buf, buf_len);

	int ret;
	int tmp_len;
	uint8_t *tmp;

	tmp_len = epb_base_request_pack_size(request->base_request);
	tmp = (uint8_t *)malloc(tmp_len);
	ret = epb_pack_base_request(request->base_request, tmp, tmp_len);
	if (ret < 0) {
		free(tmp);
		return ret;
	}
	ret = epb_set_message(&epb, TAG_SendDataRequest_BaseRequest, tmp, tmp_len);
	free(tmp);
	if (ret < 0) return ret;
	ret = epb_set_bytes(&epb, TAG_SendDataRequest_Data, request->data.data, request->data.len);
	if (ret < 0) return ret;
	if (request->has_type) {
		ret = epb_set_enum(&epb, TAG_SendDataRequest_Type, request->type);
		if (ret < 0) return ret;
	}

	return epb_get_packed_size(&epb);
}

SendDataResponse *epb_unpack_send_data_response(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	const uint8_t *tmp;
	int tmp_len;

	if (!epb_has_tag(&epb, TAG_SendDataResponse_BaseResponse)) {
		return NULL;
	}

	SendDataResponse *response = (SendDataResponse *)malloc(sizeof(SendDataResponse));
	memset(response, 0, sizeof(SendDataResponse));
	tmp = epb_get_message(&epb, TAG_SendDataResponse_BaseResponse, &tmp_len);
	response->base_response = epb_unpack_base_response(tmp, tmp_len);
	if (response->base_response == NULL) {
		free(response);
		return NULL;
	}
	if (epb_has_tag(&epb, TAG_SendDataResponse_Data)) {
		response->data.data = epb_get_bytes(&epb, TAG_SendDataResponse_Data, &response->data.len);
		response->has_data = true;
	}

	return response;
}

void epb_unpack_send_data_response_free(SendDataResponse *response)
{
	epb_unpack_base_response_free(response->base_response);
	free(response);
}

RecvDataPush *epb_unpack_recv_data_push(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	const uint8_t *tmp;
	int tmp_len;

	if (!epb_has_tag(&epb, TAG_RecvDataPush_BasePush)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_RecvDataPush_Data)) {
		return NULL;
	}

	RecvDataPush *push = (RecvDataPush *)malloc(sizeof(RecvDataPush));
	memset(push, 0, sizeof(RecvDataPush));
	tmp = epb_get_message(&epb, TAG_RecvDataPush_BasePush, &tmp_len);
	push->base_push = epb_unpack_base_push(tmp, tmp_len);
	if (push->base_push == NULL) {
		free(push);
		return NULL;
	}
	push->data.data = epb_get_bytes(&epb, TAG_RecvDataPush_Data, &push->data.len);
	if (epb_has_tag(&epb, TAG_RecvDataPush_Type)) {
		push->type = (EmDeviceDataType)epb_get_enum(&epb, TAG_RecvDataPush_Type);
		push->has_type = true;
	}

	return push;
}

void epb_unpack_recv_data_push_free(RecvDataPush *push)
{
	epb_unpack_base_push_free(push->base_push);
	free(push);
}

SwitchViewPush *epb_unpack_switch_view_push(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	const uint8_t *tmp;
	int tmp_len;

	if (!epb_has_tag(&epb, TAG_SwitchViewPush_BasePush)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_SwitchViewPush_SwitchViewOp)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_SwitchViewPush_ViewId)) {
		return NULL;
	}

	SwitchViewPush *push = (SwitchViewPush *)malloc(sizeof(SwitchViewPush));
	memset(push, 0, sizeof(SwitchViewPush));
	tmp = epb_get_message(&epb, TAG_SwitchViewPush_BasePush, &tmp_len);
	push->base_push = epb_unpack_base_push(tmp, tmp_len);
	if (push->base_push == NULL) {
		free(push);
		return NULL;
	}
	push->switch_view_op = (EmSwitchViewOp)epb_get_enum(&epb, TAG_SwitchViewPush_SwitchViewOp);
	push->view_id = (EmViewId)epb_get_enum(&epb, TAG_SwitchViewPush_ViewId);

	return push;
}

void epb_unpack_switch_view_push_free(SwitchViewPush *push)
{
	epb_unpack_base_push_free(push->base_push);
	free(push);
}

SwitchBackgroudPush *epb_unpack_switch_backgroud_push(const uint8_t *buf, int buf_len)
{
	Epb epb;
	epb_unpack_init(&epb, buf, buf_len);

	const uint8_t *tmp;
	int tmp_len;

	if (!epb_has_tag(&epb, TAG_SwitchBackgroudPush_BasePush)) {
		return NULL;
	}
	if (!epb_has_tag(&epb, TAG_SwitchBackgroudPush_SwitchBackgroundOp)) {
		return NULL;
	}

	SwitchBackgroudPush *push = (SwitchBackgroudPush *)malloc(sizeof(SwitchBackgroudPush));
	memset(push, 0, sizeof(SwitchBackgroudPush));
	tmp = epb_get_message(&epb, TAG_SwitchBackgroudPush_BasePush, &tmp_len);
	push->base_push = epb_unpack_base_push(tmp, tmp_len);
	if (push->base_push == NULL) {
		free(push);
		return NULL;
	}
	push->switch_background_op = (EmSwitchBackgroundOp)epb_get_enum(&epb, TAG_SwitchBackgroudPush_SwitchBackgroundOp);

	return push;
}

void epb_unpack_switch_backgroud_push_free(SwitchBackgroudPush *push)
{
	epb_unpack_base_push_free(push->base_push);
	free(push);
}
