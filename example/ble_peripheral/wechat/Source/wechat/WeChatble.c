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


#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "log.h"
#include "ble_wechat_util.h"

#include "epb_MmBp.h"
#include "md5.h"
#include "crc32.h"
#include "AES.h"
#include "WeChatble.h"
#include "gatt.h"
#include "gattservapp.h"
#include "peripheral.h"
#include "gapgattserver.h"

wechat_info m_info = {CMD_NULL, {NULL, 0}};

#if defined EAM_md5AndNoEnrypt || EAM_md5AndAesEnrypt
uint8_t md5_type_and_id[16];
#endif

uint8_t challeange[CHALLENAGE_LENGTH] = {0};

wechat_state wechatSta = {false, false, false, false, false, false,0,0,0};
const uint8_t key[16] = DEVICE_KEY;
uint8_t session_key[16] = {0};
extern uint16 GATTServApp_ReadCharCfg( uint16 connHandle, gattCharCfg_t *charCfgTbl);
extern gattCharCfg_t weixinindicateProfileCharConfig[];
extern bStatus_t ble_wechat_indicate_data(uint8_t *data, int len);
extern uint16 g_step_number;

//use for receiving data
uint8_t g_w_value[20]= {0};
uint8_t g_w_len=0;
uint8_t g_w_offset=0;

/**@brief   Function for the light initialization.
 *
 * @details Initializes all lights used by this application.
 */
static uint8_t get_challeange_values()
{
    uint8_t i;
    for(i=0; i<4; i++)
    {
        challeange[i]= (uint8_t)rand()%255 ;
    }
    return 0;
}

static int32_t wechat_get_md5(void)
{
    int32_t error_code = 0;
	
#if defined EAM_md5AndNoEnrypt || EAM_md5AndAesEnrypt
		char device_type[] = DEVICE_TYPE;
		char device_id[] = DEVICE_ID;
		char argv[sizeof(DEVICE_TYPE) + sizeof(DEVICE_ID) - 1];
		memcpy(argv,device_type,sizeof(DEVICE_TYPE));
		/*when add the DEVICE_ID to DEVICE_TYPE, the offset shuld -1 to overwrite '\0'  at the end of DEVICE_TYPE */
		memcpy(argv + sizeof(DEVICE_TYPE)-1,device_id,sizeof(DEVICE_ID));
	
    error_code = md5(argv,md5_type_and_id);
#endif
    return error_code;
}

static void wechat_reset()
{
    wechatSta.auth_send = false;
    wechatSta.auth_state = false;
    wechatSta.indication_state = false;
    wechatSta.init_send = false;
    wechatSta.init_state = false;
    wechatSta.send_data_seq = 0;
    wechatSta.push_data_seq = 0;
    wechatSta.seq = 0;
}

int32_t wechat_init(void)
{
    wechat_reset();
    return (wechat_get_md5());
}

/**@brief Function for handling button pull events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void wechat_data_free_func(uint8_t *data, uint32_t len)
{
    if(data != NULL)
        free(data);
    data = NULL;
}

/*******************************************************************************************************/
 static void device_auth_data_func(uint8_t **r_data, uint32_t *r_len)
{
	#if defined EAM_macNoEncrypt || EAM_md5AndNoEnrypt
    BaseRequest basReq = {NULL};
	#endif
	
    static uint8_t fix_head_len = sizeof(BpFixHead);
    BpFixHead fix_head = {0xFE, 1, 0, 0, 0};
    wechatSta.seq++;

#if defined EAM_md5AndAesEnrypt
    //AES(Ran | Seq | CRC32 ( DeviceId | Ran | Seq ))
    uint8_t deviceid[] = DEVICE_ID;
    uint32_t seq = wechatSta.seq;
    uint32_t ran = 0x11223344;
    ran = t_htonl(ran);
    seq = t_htonl(seq);
    uint8_t id_len = strlen(DEVICE_ID);
    uint8_t* data = malloc(id_len+8);
    if(!data)
    {
        LOG("\nNot enough memory!");
        return;
    }
    memcpy(data,deviceid,id_len);
    memcpy(data+id_len,(uint8_t*)&ran,4);
    memcpy(data+id_len+4,(uint8_t*)&seq,4);
    uint32_t crc = crc32(0, data, id_len+8);
    crc = t_htonl(crc);
    if(data)
    {
        free(data);
        data = NULL;
    }

    uint8_t aesData[12];
    memcpy(aesData,  (uint8_t*)&ran,4);
    memcpy(aesData+4,(uint8_t*)&seq,4);
    memcpy(aesData+8,(uint8_t*)&crc,4);

    uint8_t CipherText[16];
    AES_Init(key);
    AES_Encrypt_PKCS7 (aesData, CipherText, 12, key);
    AuthRequest authReq = {NULL, true,{md5_type_and_id, MD5_TYPE_AND_ID_LENGTH}, PROTO_VERSION, AUTH_PROTO, (EmAuthMethod)AUTH_METHOD, true,{CipherText, CIPHER_TEXT_LENGTH}, false, {NULL, 0}, false, {NULL, 0}, false, {NULL, 0},true,{DEVICE_ID,sizeof(DEVICE_ID)}};
    seq++;
#endif

#if defined EAM_macNoEncrypt
    static uint8_t mac_address[MAC_ADDRESS_LENGTH];
    get_mac_addr(mac_address); 
    AuthRequest authReq = {&basReq, false,{NULL, 0},                                PROTO_VERSION, AUTH_PROTO,(EmAuthMethod)AUTH_METHOD, false,{NULL, 0}, true, {mac_address, MAC_ADDRESS_LENGTH}, false, {NULL, 0}, false, {NULL, 0},true,{DEVICE_ID,sizeof(DEVICE_ID)}};
#endif

#if defined EAM_md5AndNoEnrypt	
		AuthRequest authReq = {&basReq, true,{md5_type_and_id, MD5_TYPE_AND_ID_LENGTH}, PROTO_VERSION, (EmAuthMethod)AUTH_PROTO, (EmAuthMethod)AUTH_METHOD, false ,{NULL, 0}, false, {NULL, 0}, false, {NULL, 0}, false, {NULL, 0},true,{DEVICE_ID,sizeof(DEVICE_ID)}};
#endif

    *r_len = epb_auth_request_pack_size(&authReq) + fix_head_len;
    *r_data = (uint8_t *)malloc(*r_len);
    if(!(*r_data))
    {
        LOG("\nNot enough memory!");
        return;
    }
    if(epb_pack_auth_request(&authReq, *r_data+fix_head_len, *r_len-fix_head_len)<0)
    {
        *r_data = NULL;
        return;
    }
    fix_head.nCmdId = htons(ECI_req_auth);
    fix_head.nLength = htons(*r_len);
    fix_head.nSeq = htons(wechatSta.seq);
    memcpy(*r_data, &fix_head, fix_head_len);
}

void device_auth(void)
{
    uint8_t *data = NULL;
    uint32_t len = 0;

    device_auth_data_func(&data, &len);

    if(data == NULL)
    {
        return;
    }
		LOG(">>device_auth,len:%x\n",len);
		
    //sent data
    ble_wechat_indicate_data(data, len);
    wechat_data_free_func(data,len);
}

static void device_init_data_func(uint8_t **r_data, uint32_t *r_len)
{
    BaseRequest basReq = {NULL};
    static uint8_t fix_head_len = sizeof(BpFixHead);
    BpFixHead fix_head = {0xFE, 1, 0, 0, 0};
    wechatSta.seq++;

    //sent data
    //has challeange
    get_challeange_values();
    InitRequest initReq = {&basReq,false, {NULL, 0},true, {challeange, CHALLENAGE_LENGTH}};
    *r_len = epb_init_request_pack_size(&initReq) + fix_head_len;

#if defined EAM_md5AndAesEnrypt
    uint8_t length = *r_len;
    uint8_t *p = malloc(AES_get_length( *r_len-fix_head_len));
    if(!p)
    {
        LOG("\nNot enough memory!");
        return;
    }
    *r_len = AES_get_length( *r_len-fix_head_len)+fix_head_len;
#endif
    //pack data
    *r_data = (uint8_t *)malloc(*r_len);
    if(!(*r_data))
    {
        LOG("\nNot enough memory!");
        return;
    }
    if(epb_pack_init_request(&initReq, *r_data+fix_head_len, *r_len-fix_head_len)<0)
    {
        *r_data = NULL;
        return;
    }
    //encrypt body
#if defined EAM_md5AndAesEnrypt
    AES_Init(session_key);
    AES_Encrypt_PKCS7(*r_data+fix_head_len,p,length-fix_head_len,session_key);
    memcpy(*r_data + fix_head_len, p, *r_len-fix_head_len);
    if(p)free(p);
#endif
    fix_head.nCmdId = htons(ECI_req_init);
    fix_head.nLength = htons(*r_len);
    fix_head.nSeq = htons(wechatSta.seq);
    memcpy(*r_data, &fix_head, fix_head_len);
}

void device_init()
{
    uint8_t *data = NULL;//,i;
    uint32_t len = 0;

    device_init_data_func(&data, &len);
    if(data == NULL)
    {
        return ;
    }
		
		LOG(">>device_init,len:%x \n",len);
//		for(i=0;i<len;i++)
//			LOG(" %x ",data[i]);
//		LOG("\n");
		
    //sent data
    ble_wechat_indicate_data(data, len);
    wechat_data_free_func(data,len);
}

/*static*/ void device_senddata_func(void *args, uint8_t **r_data, uint32_t *r_len)
{
    wechat_info *info = (wechat_info *)args;
    BaseRequest basReq = {NULL};
    static uint8_t fix_head_len = sizeof(BpFixHead);
    BpFixHead fix_head = {0xFE, 1, 0, 0, 0};
    wechatSta.seq++;

    LOG("\n msg to send : %s",(uint8_t*)info->send_msg.str);

    static uint16_t bleDemoHeadLen = sizeof(BlueDemoHead);
    BlueDemoHead  *bleDemoHead = (BlueDemoHead*)malloc(bleDemoHeadLen + info->send_msg.len);
    if(!bleDemoHead)
    {
        LOG("\nNot enough memory!");
        return;
    }

    bleDemoHead->m_magicCode[0] = MPBLEDEMO2_MAGICCODE_H;
    bleDemoHead->m_magicCode[1] = MPBLEDEMO2_MAGICCODE_L;
    bleDemoHead->m_version = htons( MPBLEDEMO2_VERSION);
    bleDemoHead->m_totalLength = htons(bleDemoHeadLen + info->send_msg.len);
    bleDemoHead->m_cmdid = htons(sendTextReq);
    bleDemoHead->m_seq = htons(wechatSta.seq);
    bleDemoHead->m_errorCode = 0;
    /*connect body and head.*/
    /*turn to uint8_t* befort offset.*/
    memcpy((uint8_t*)bleDemoHead+bleDemoHeadLen, info->send_msg.str, info->send_msg.len);
    SendDataRequest sendDatReq = {&basReq, {(uint8_t*) bleDemoHead, (bleDemoHeadLen + info->send_msg.len)}, false, (EmDeviceDataType)NULL};
    *r_len = epb_send_data_request_pack_size(&sendDatReq) + fix_head_len;
#if defined EAM_md5AndAesEnrypt
    uint16_t length = *r_len;
    uint8_t *p = malloc(AES_get_length( *r_len-fix_head_len));
    if(!p)
    {
        LOG("\nNot enough memory!");
        return;
    }
    *r_len = AES_get_length( *r_len-fix_head_len)+fix_head_len;
#endif
    *r_data = (uint8_t *)malloc(*r_len);
    if(!(*r_data))
    {
        LOG("\nNot enough memory!");
        return;
    }
    if(epb_pack_send_data_request(&sendDatReq, *r_data+fix_head_len, *r_len-fix_head_len)<0)
    {
        *r_data = NULL;
#if defined EAM_md5AndAesEnrypt
        if(p)
        {
            free(p);
            p = NULL;
        }
#endif
        LOG("\nepb_pack_send_data_request error!");
        return;
    }
#if defined EAM_md5AndAesEnrypt
    //encrypt body
    AES_Init(session_key);
    AES_Encrypt_PKCS7(*r_data+fix_head_len,p,length-fix_head_len,session_key);
    memcpy(*r_data + fix_head_len, p, *r_len-fix_head_len);
    if(p)
    {
        free(p);
        p = NULL;
    }
#endif
    fix_head.nCmdId = htons(ECI_req_sendData);
    fix_head.nLength = htons(*r_len);
    fix_head.nSeq = htons(wechatSta.seq);
    memcpy(*r_data, &fix_head, fix_head_len);
    if(bleDemoHead)
    {
        free(bleDemoHead);
        bleDemoHead = NULL;
    }
#ifdef CATCH_LOG
    LOG("\n##send data: ");
    uint8_t *d = *r_data;
    for(uint8_t i=0; i<*r_len; ++i)
    {
        LOG(" %x",d[i]);
    }
    BpFixHead *fix_head1 = (BpFixHead *)*r_data;
    LOG("\n CMDID: %d",ntohs(fix_head1->nCmdId));
    LOG("\n len: %d", ntohs(fix_head1->nLength ));
    LOG("\n Seq: %d", ntohs(fix_head1->nSeq));
#endif
    wechatSta.send_data_seq++;
}

void device_senddata()
{
    uint8_t *data = NULL;
    uint32_t len = 0;
    wechat_info m_info = {CMD_NULL, {SEND_HELLO_WECHAT, sizeof(SEND_HELLO_WECHAT)}};

    device_senddata_func(&m_info,&data, &len);

    if(data == NULL)
    {
        return ;
    }

    ble_wechat_indicate_data(data, len);
    wechat_data_free_func(data,len);
}

static void device_send_data_package_func(uint8_t **r_data, uint32_t *r_len, char *datapackage, uint16 datapackageLen)
{
    static uint16_t bleDemoHeadLen = sizeof(BlueDemoHead);
    BaseRequest basReq = {NULL};
    static uint8_t fix_head_len = sizeof(BpFixHead);
    BpFixHead fix_head = {0xFE, 1, 0, 0, 0};
    wechatSta.seq++;

    BlueDemoHead *bleDemoHead = (BlueDemoHead*)malloc(bleDemoHeadLen+datapackageLen);

    if(!bleDemoHead)
    {
        LOG("\nNot enough memory!");
        return;
    }

    bleDemoHead->m_magicCode[0] = MPBLEDEMO2_MAGICCODE_H;
    bleDemoHead->m_magicCode[1] = MPBLEDEMO2_MAGICCODE_L;
    bleDemoHead->m_version = htons( MPBLEDEMO2_VERSION);
    bleDemoHead->m_totalLength = htons(bleDemoHeadLen + datapackageLen);
    bleDemoHead->m_cmdid = htons(sendTextReq);
    bleDemoHead->m_seq = htons(wechatSta.seq);
    bleDemoHead->m_errorCode = 0;
    /*connect body and head.*/
    /*turn to uint8_t* befort offset.*/
    memcpy((uint8_t*)bleDemoHead+bleDemoHeadLen, datapackage, datapackageLen);
    SendDataRequest sendDatReq = {&basReq, {(uint8_t*) bleDemoHead, (bleDemoHeadLen + datapackageLen)}, false, (EmDeviceDataType)NULL};

    *r_len = epb_send_data_request_pack_size(&sendDatReq) + fix_head_len;
#if defined EAM_md5AndAesEnrypt
    uint16_t length = *r_len;
    uint8_t *p = malloc(AES_get_length( *r_len-fix_head_len));
    if(!p)
    {
        LOG("\nNot enough memory!");
        return;
    }
    *r_len = AES_get_length( *r_len-fix_head_len)+fix_head_len;
#endif
    *r_data = (uint8_t *)malloc(*r_len);
    if(!(*r_data))
    {
        LOG("\nNot enough memory!");
        return;
    }
    if(epb_pack_send_data_request(&sendDatReq, *r_data+fix_head_len, *r_len-fix_head_len)<0)
    {
        *r_data = NULL;
#if defined EAM_md5AndAesEnrypt
        if(p)
        {
            free(p);
            p = NULL;
        }
#endif
        LOG("\nepb_pack_send_data_request error!");
        return;
    }
#if defined EAM_md5AndAesEnrypt
    //encrypt body
    AES_Init(session_key);
    AES_Encrypt_PKCS7(*r_data+fix_head_len,p,length-fix_head_len,session_key);
    memcpy(*r_data + fix_head_len, p, *r_len-fix_head_len);
    if(p)
    {
        free(p);
        p = NULL;
    }
#endif
    fix_head.nCmdId = htons(ECI_req_sendData);
    fix_head.nLength = htons(*r_len);
    fix_head.nSeq = htons(wechatSta.seq);
    memcpy(*r_data, &fix_head, fix_head_len);
    if(bleDemoHead)
    {
        free(bleDemoHead);
        bleDemoHead = NULL;
    }
#ifdef CATCH_LOG
    LOG("\n##send data: ");
    uint8_t *d = *r_data;
    for(uint8_t i=0; i<*r_len; ++i)
    {
        LOG(" %x",d[i]);
    }
    BpFixHead *fix_head1 = (BpFixHead *)*r_data;
    LOG("\n CMDID: %d",ntohs(fix_head1->nCmdId));
    LOG("\n len: %d", ntohs(fix_head1->nLength ));
    LOG("\n Seq: %d", ntohs(fix_head1->nSeq));
#endif
    wechatSta.send_data_seq++;
}

void device_send_step_data()
{
    uint8_t *data = NULL, device_name[10]= {0};
    uint32_t len = 0, StepdataLen=0, step_number = 0;
    char stepdata[50]= {0};

    GGS_GetParameter(GGS_DEVICE_NAME_ATT, device_name);
    step_number = 1;
    StepdataLen = sprintf(stepdata,"%s:bs%d",device_name, step_number);
    device_send_data_package_func(&data, &len, stepdata, StepdataLen);

    if(data == NULL)
    {
        return ;
    }
    ble_wechat_indicate_data(data, len);
    wechat_data_free_func(data,len);
}

void device_send_name_mac_data()
{
    uint8_t *data = NULL,ownAddress[B_ADDR_LEN],device_name[10]= {0};
    uint32_t name_mac_Len=0, len = 0;
    char name_mac[20]= {0}, str[12]= {0};
    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
    convertBdAddr2Str(ownAddress, (uint8 *)str);

    GGS_GetParameter(GGS_DEVICE_NAME_ATT, device_name);
    name_mac_Len =sprintf(name_mac,"%s:%s",device_name,str);
	
    device_send_data_package_func(&data, &len, name_mac, name_mac_Len);

    if(data == NULL)
    {
        return ;
    }

    ble_wechat_indicate_data(data, len);
    wechat_data_free_func(data,len);
}

void device_send_name_pong_data()
{
    uint8_t *data = NULL, device_name[GAP_DEVICE_NAME_LEN]= {0};
    char name_pong[20]= {0};
    uint32_t name_pong_Len=0, len = 0;

    /*----------------------------------------------------------------*/
    /* Code Body                                                      */
    /*----------------------------------------------------------------*/
    GGS_GetParameter(GGS_DEVICE_NAME_ATT, device_name);
    name_pong_Len =sprintf(name_pong,"%s:pong",device_name);

    device_send_data_package_func(&data, &len, name_pong, name_pong_Len);

    if(data == NULL)
    {
        return ;
    }

    ble_wechat_indicate_data(data, len);
    wechat_data_free_func(data,len);
}
/*******************************************************************************************************/

void wechat_on_disconnect()
{
    wechat_reset();
}

static int wechat_data_consume_func(uint8_t *data, uint32_t len)
{
    BpFixHead *fix_head = (BpFixHead *)data;
    uint8_t fix_head_len = sizeof(BpFixHead);
	
#ifdef CATCH_LOG
    LOG("\n##Received data: ");
    uint8_t *d = data;
    for(uint8_t i=0; i<len; ++i)
    {
        LOG(" %x",d[i]);
    }
    LOG("\n CMDID: %d", ntohs(fix_head->nCmdId));
    LOG("\n len: %d", ntohs(fix_head->nLength));
    LOG("\n Seq: %d",ntohs(fix_head->nSeq));
#endif
		
		LOG("from_wechat:%d\n\n",ntohs(fix_head->nCmdId));
		
    switch(ntohs(fix_head->nCmdId))
    {
        case ECI_none:
        {
        }
        break;
				
        case ECI_resp_auth:
        {
            AuthResponse* authResp;
            authResp = epb_unpack_auth_response(data+fix_head_len,len-fix_head_len);
#ifdef CATCH_LOG
            LOG("\n@@Received 'authResp'\n");
#endif
            if(!authResp)
            {
                return errorCodeUnpackAuthResp;
            }
#ifdef CATCH_LOG
            LOG("\n unpack 'authResp' success!\n");
#endif
            if(authResp->base_response)
            {
                if(authResp->base_response->err_code == 0)
                {
                    wechatSta.auth_state = true;
										device_init();
                }
                else
                {
#ifdef CATCH_LOG
                    LOG("\n error code:%d %d",authResp->base_response->err_code,-5);
#endif
#ifdef CATCH_LOG
                    if(authResp->base_response->has_err_msg)
                    {
                        LOG("\n base_response error msg:%s",authResp->base_response->err_msg.str);
                    }
#endif
                    epb_unpack_auth_response_free(authResp);
                    return authResp->base_response->err_code;
                }
            }
#if defined EAM_md5AndAesEnrypt// get sessionkey
            if(authResp->aes_session_key.len)
            {
#ifdef CATCH_LOG
                LOG("\nsession_key:");
#endif
                AES_Init(key);
                AES_Decrypt(session_key,authResp->aes_session_key.data,authResp->aes_session_key.len,key);
#ifdef CATCH_LOG
                for(uint8_t i = 0; i<16; i++)
                {
                    LOG(" 0x%02x",session_key[i]);
                }
#endif
            }
#endif
            epb_unpack_auth_response_free(authResp);
        }
        break;
				
				
        case ECI_resp_sendData:
        {

#ifdef CATCH_LOG
            LOG("\n@@Received 'sendDataResp'\n");
#endif
#if defined EAM_md5AndAesEnrypt
            uint32_t length = len- fix_head_len;
            uint8_t *p = malloc (length);
            if(!p)
            {
                LOG("\nNot enough memory!");
                if(data)free(data);
                data = NULL;
                return 0;
            }
            AES_Init(session_key);
            AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);

            uint8_t temp;
            temp = p[length - 1];
            len = len - temp;
            memcpy(data + fix_head_len, p,length -temp);
            if(p)
            {
                free(p);
                p = NULL;
            }
#endif
            SendDataResponse *sendDataResp;
            sendDataResp = epb_unpack_send_data_response(data+fix_head_len,len-fix_head_len);
            if(!sendDataResp)
            {
                return errorCodeUnpackSendDataResp;
            }
#ifdef CATCH_LOG
            BlueDemoHead *bledemohead = (BlueDemoHead*)sendDataResp->data.data;
            if(ntohs(bledemohead->m_cmdid) == sendTextResp)
            {
                LOG("\n received msg: %s\n",sendDataResp->data.data+sizeof(BlueDemoHead));
            }
#endif
            if(sendDataResp->base_response->err_code)
            {
                epb_unpack_send_data_response_free(sendDataResp);
                return sendDataResp->base_response->err_code;
            }
            epb_unpack_send_data_response_free(sendDataResp);
        }
        break;
				
				
        case ECI_resp_init:
        {
#ifdef CATCH_LOG
            LOG("\n@@Received 'initResp'\n");
#endif
#if defined EAM_md5AndAesEnrypt
            uint32_t length = len- fix_head_len;
            uint8_t *p = malloc (length);
            if(!p)
            {
                LOG("\nNot enough memory!");
                if(data)free(data);
                data = NULL;
                return 0;
            }
            AES_Init(session_key);
            AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);

            uint8_t temp;
            temp = p[length - 1];
            len = len - temp;
            memcpy(data + fix_head_len, p,length -temp);
            if(p)
            {
                free(p);
                p = NULL;
            }
#endif
            InitResponse *initResp = epb_unpack_init_response(data+fix_head_len, len-fix_head_len);
            if(!initResp)
            {
                return errorCodeUnpackInitResp;
            }
#ifdef CATCH_LOG
            LOG("\n unpack 'initResp' success!");
#endif
            if(initResp->base_response)
            {
                if(initResp->base_response->err_code == 0)
                {
                    if(initResp->has_challeange_answer)
                    {
                        if(crc32(0,challeange,CHALLENAGE_LENGTH) == initResp->challeange_answer)
                        {
                            wechatSta.init_state = true;
                        }
                    }
                    else wechatSta.init_state = true;
                    wechatSta.wechats_switch_state = true;
                    device_send_name_mac_data();
                }
                else
                {
#ifdef CATCH_LOG
                    LOG("\n error code:%d %d ",initResp->base_response->err_code,-6);
#endif
                    if(initResp->base_response->has_err_msg)
                    {
#ifdef CATCH_LOG
                        LOG("\n base_response error msg:%s",initResp->base_response->err_msg.str);
#endif
                    }
                    epb_unpack_init_response_free(initResp);
                    return initResp->base_response->err_code;
                }
            }

            epb_unpack_init_response_free(initResp);
        }
        break;
				
				
        case ECI_push_recvData:
        {
#if defined EAM_md5AndAesEnrypt
            uint32_t length = len- fix_head_len;
            uint8_t *p = malloc (length);
            if(!p)
            {
                LOG("\nNot enough memory!");
                if(data)free(data);
                data =NULL;
                return 0;
            }
            AES_Init(session_key);
            AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);

            uint8_t temp;
            temp = p[length - 1];
            len = len - temp;
            memcpy(data + fix_head_len, p,length -temp);
            if(p)
            {
                free(p);
                p = NULL;
            }
#endif
            RecvDataPush *recvDatPush;
            recvDatPush = epb_unpack_recv_data_push(data+fix_head_len, len-fix_head_len);
#ifdef CATCH_LOG
            LOG("\n@@Received 'recvDataPush'\n");
#endif
            if(!recvDatPush)
            {
                return errorCodeUnpackRecvDataPush;
            }
#ifdef CATCH_LOG
            LOG("\n unpack the 'recvDataPush' successfully! \n");
            if(recvDatPush->base_push == NULL)
            {
                LOG("recvDatPush->base_push is NULL! \n");
            }
            else
            {
                LOG(" recvDatPush->base_push is not NULL! \n");
            }
            LOG(" recvDatPush->data.len: %x \n",recvDatPush->data.len);
            LOG(" recvDatPush->data.data:  \n");
            const uint8_t *d = recvDatPush->data.data;
            for(uint8_t i=0; i<recvDatPush->data.len; ++i)
            {
                LOG(" %x",d[i]);
            }
            if(recvDatPush->has_type)
            {
                LOG(" recvDatPush has type! \n");
                LOG(" type: %d\n",recvDatPush->type);
            }
#endif
            BlueDemoHead *bledemohead = (BlueDemoHead*)recvDatPush->data.data;
#ifdef CATCH_LOG
            LOG("\n magicCode: %x",bledemohead->m_magicCode[0]);
            LOG(" %x",bledemohead->m_magicCode[1]);
            LOG("\n version: %x",ntohs(bledemohead->m_version));
            LOG("\n totalLength: %x",ntohs(bledemohead->m_totalLength));
            LOG("\n cmdid: %x",ntohs(bledemohead->m_cmdid ));
            LOG("\n errorCode: %x\n",ntohs(bledemohead->m_errorCode));
#endif
            if((recvDatPush->data.data[0]==0x62)&&(recvDatPush->data.data[1]==0x73))   //for test
            {
                device_send_step_data();
            }
            else
            {
                device_send_name_pong_data();
            }
            /*else if((recvDatPush->data.data[0]==0x42)&&(recvDatPush->data.data[1]==0x73))   //for test
            {
            device_send_step_data();
            } */
            if(ntohs(bledemohead->m_cmdid ) == openLightPush)
            {
#ifdef CATCH_LOG
                LOG("\n light on!! ");
#endif

              
            }
            else if(ntohs(bledemohead->m_cmdid )  == closeLightPush)
            {
#ifdef CATCH_LOG
                LOG("\n light off!! ");
#endif

              
            }
            epb_unpack_recv_data_push_free(recvDatPush);
            wechatSta.push_data_seq++;
        }
        break;
				
        case ECI_push_switchView:
        {

            wechatSta.wechats_switch_state = !wechatSta.wechats_switch_state;
#ifdef CATCH_LOG
            LOG("\n@@Received 'switchViewPush'\n");
#endif
#if defined EAM_md5AndAesEnrypt
            uint32_t length = len- fix_head_len;
            uint8_t *p = malloc (length);
            if(!p)
            {
                LOG("\nNot enough memory!");
                if(data)free(data);
                data = NULL;
                return 0;
            }
            AES_Init(session_key);
            AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);

            uint8_t temp;
            temp = p[length - 1];
            len = len - temp;
            memcpy(data + fix_head_len, p,length -temp);
            if(p)
            {
                free(p);
                p = NULL;
            }
#endif
            SwitchViewPush *swichViewPush;
            swichViewPush = epb_unpack_switch_view_push(data+fix_head_len,len-fix_head_len);
            if(!swichViewPush)
            {
                return errorCodeUnpackSwitchViewPush;
            }
            epb_unpack_switch_view_push_free(swichViewPush);
        }
        break;
				
        case ECI_push_switchBackgroud:
        {
#ifdef CATCH_LOG
            LOG("\n@@Received 'switchBackgroudPush'\n");
#endif
#if defined EAM_md5AndAesEnrypt
            uint32_t length = len- fix_head_len;
            uint8_t *p = malloc (length);
            if(!p)
            {
                LOG("\nNot enough memory!");
                if(data)free(data);
                data = NULL;
                return 0;
            }
            AES_Init(session_key);
            AES_Decrypt(p,data+fix_head_len,len- fix_head_len,session_key);
            uint8_t temp;
            temp = p[length - 1];
            len = len - temp;
            memcpy(data + fix_head_len, p,length -temp);
            if(data)
            {
                free(p);
                p = NULL;
            }
#endif
            SwitchBackgroudPush *switchBackgroundPush = epb_unpack_switch_backgroud_push(data+fix_head_len,len-fix_head_len);
            if(! switchBackgroundPush)
            {
                return errorCodeUnpackSwitchBackgroundPush;
            }
            epb_unpack_switch_backgroud_push_free(switchBackgroundPush);
        }
        break;
				
        case ECI_err_decode:
            break;
        default:
        {
#ifdef CATCH_LOG
            LOG("\n !!ERROR CMDID:%d",ntohs(fix_head->nCmdId));
#endif
        }
        break;
    }
    return 0;
}

void wechat_write_cb_consume(uint8 * newValue)
{
    if(g_w_len == 0)
    {
        BpFixHead *fix_head = (BpFixHead *) newValue;
        g_w_len = ntohs(fix_head->nLength);
    }

    if(g_w_len <= 20)
    {
        memcpy(g_w_value+g_w_offset, newValue, g_w_len);
        wechat_data_consume_func(g_w_value, g_w_len+g_w_offset);
		
        g_w_len = 0;
				g_w_offset = 0;
				memset(g_w_value, 0, 20);
    }
    else
    {
        memcpy(g_w_value+g_w_offset, newValue, 20);
        g_w_offset += 20;
        g_w_len -= 20;
        if(g_w_len == 0)
        {
            wechat_data_consume_func(g_w_value, g_w_offset);
						g_w_offset = 0;
						memset(g_w_value, 0, 20);
        }
    }
}
