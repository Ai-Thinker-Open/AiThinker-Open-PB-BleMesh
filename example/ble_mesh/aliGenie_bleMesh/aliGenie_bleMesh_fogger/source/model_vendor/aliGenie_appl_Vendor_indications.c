#include "aliGenie_appl.h"
#include "aliGenie_appl_vendor.h"


API_RESULT vendor_Aligenie_indication(MS_NET_ADDR saddr,UINT16 attr_code, UINT32 attr_val,int attr_len)
{
    MS_SUBNET_HANDLE    subnet_handle;
    MS_APPKEY_HANDLE    appkey_handle;
    UINT8               ttl;
    UINT32              opcode;
    UCHAR               pdu[15];
    UINT16              data_len = 1;
    subnet_handle = 0;
    appkey_handle = 0;
    ttl = 0x08;
    opcode = MS_ACCESS_VENDOR_ALIGENIE_INDICATION;
    pdu[0]=++vendor_msg_tid;
    make_attr_pdu(pdu+data_len, attr_code, attr_val, attr_len);

    if (attr_len > 0)
    {
        data_len += attr_len+2;
    }

    for(int i = 0; i < 2 ; i++)
    {
        MS_access_send_pdu
        (
            saddr,
            aligenie_addr,
            subnet_handle,
            appkey_handle,
            ttl,
            opcode,
            pdu,
            data_len,
            MS_TRUE
        );
    }

    INFO_PRINT("Vendor Sending Indication, saddr = 0x%02X,data_len = %d \r\n", saddr,data_len);
    return API_SUCCESS;
}


