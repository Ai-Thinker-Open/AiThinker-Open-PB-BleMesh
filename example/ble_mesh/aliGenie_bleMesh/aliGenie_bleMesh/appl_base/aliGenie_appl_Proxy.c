#include "AliGenie_appl_Proxy.h"
#include "aliGenie_appl_Proxy.h"

void UI_proxy_start_adv(MS_SUBNET_HANDLE subnet_handle, UCHAR proxy_adv_mode)
{
    #ifdef MS_PROXY_SERVER
    API_RESULT retval;
    DECL_STATIC UINT8 first_time = 0;

    if (0 == first_time)
    {
        /**
            Register with Proxy Module as Device is going to be a Proxy.
            This is typically a one-time-event, and hence registering the
            PROXY when Proxy ADV is being initiated!
        */
        UI_register_proxy();
        first_time = 1;
    }

    /* Set the role to Proxy with bearer */
    blebrr_gatt_mode_set(BLEBRR_GATT_PROXY_MODE);
    CONSOLE_OUT("Start Proxy Advertisements with %s for Subnet 0x%04X\n",
                (proxy_adv_mode == MS_PROXY_NET_ID_ADV_MODE) ? "Network ID" : "Node Identity",
                subnet_handle);
    retval = MS_proxy_server_adv_start
             (
                 subnet_handle,
                 proxy_adv_mode
             );
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
    #else /* MS_PROXY_SERVER */
    CONSOLE_OUT("\n [** ERR **] MS_PROXY_SERVER feature is DISABLED!\n");
    return;
    #endif /* MS_PROXY_SERVER */
}

void UI_proxy_callback
(
    NETIF_HANDLE*        handle,
    UCHAR                p_evt,
    UCHAR*               data_param,
    UINT16               data_len
)
{
    UCHAR             role;
    MS_IGNORE_UNUSED_PARAM(data_len);

    switch(p_evt)
    {
    case MS_PROXY_UP_EVENT:
        CONSOLE_OUT(
            "\n\n[PROXY APPL]: MS_PROXY_UP_EVENT Received for NETIF Handle 0x%02X\n\n", *handle);

        if (NULL != data_param)
        {
            /* Catch the current role into a local */
            role = data_param[0];

            if (BRR_SERVER_ROLE == role)
            {
                /* Send Secure Network Beacons */
                /* MS_net_broadcast_secure_beacon(0x0000); */
            }
        }

        break;

    case MS_PROXY_DOWN_EVENT:
        CONSOLE_OUT(
            "\n\n[PROXY APPL]: MS_PROXY_DOWN_EVENT Received for NETIF Handle 0x%02X\n\n", *handle);
        break;

    default:
        CONSOLE_OUT(
            "\n\n[PROXY APPL ERR]: Unknown Event Received for NETIF Handle 0x%02X!!\n\n", *handle);
        break;
    }
}

#ifdef MS_PROXY_SUPPORT
void UI_register_proxy(void)
{
    API_RESULT retval;
    CONSOLE_OUT("Registering with Proxy layer...\n");
    retval =  MS_proxy_register(UI_proxy_callback);
    CONSOLE_OUT("Retval - 0x%04X\n", retval);
}
#endif

