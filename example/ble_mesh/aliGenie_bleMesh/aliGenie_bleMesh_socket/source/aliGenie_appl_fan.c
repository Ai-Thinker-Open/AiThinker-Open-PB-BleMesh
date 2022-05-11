#include "aliGenie_appl.h"
#include "aliGenie_appl_Generic.h"
#include "aliGenie_appl_Vendor_fan.h"

UINT16 g_subscription_addr = 0xC007;
extern API_RESULT UI_vendor_aliGenie_model_server_create(MS_ACCESS_NODE_ID  node_id,MS_ACCESS_ELEMENT_HANDLE element_handle,UI_DATA_GENERIC_ONOFF_MODEL_T* generic_onoff_me);
extern API_RESULT init_attr_tbl_fan(UI_DATA_ALIGENIE_MODEL_T* me);
API_RESULT aliGenie_appl_create_elements_and_models(MS_ACCESS_NODE_ID node_id,MS_ACCESS_ELEMENT_HANDLE element_handle1)
{
//  MS_ACCESS_ELEMENT_DESC   element;
    UINT8 ele_cnt;
    API_RESULT retval;
    UI_DATA_GENERIC_ONOFF_MODEL_T* generic_onoff_me;

    if (API_SUCCESS == retval)
    {
        /* !!! generic scene model MUST set at the begging of it's element */
        retval = UI_generic_scene_model_server_create(element_handle1, FAN_MODE_NUM, fan_mode_list);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle1,LIGHT_GREEN,&generic_onoff_me);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        pinit_attr_tbl = init_attr_tbl_fan;
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle1,generic_onoff_me);
    }

    #ifdef ELEMENT_2nd
    MS_ACCESS_ELEMENT_HANDLE element_handle2;
    element.loc = 0x0107;
    retval = MS_access_register_element
             (
                 node_id,
                 &element,
                 &element_handle2
             );

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle2,LIGHT_BLUE,&generic_onoff_me);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle2,generic_onoff_me);
    }

    #endif
    #ifdef ELEMENT_3rd
    MS_ACCESS_ELEMENT_HANDLE element_handle3;
    element.loc = 0x0108;
    retval = MS_access_register_element
             (
                 node_id,
                 &element,
                 &element_handle3
             );

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle3,LIGHT_RED,&generic_onoff_me);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle3,generic_onoff_me);
    }

    #endif
    MS_access_cm_get_element_count(&ele_cnt);
    DEBUG_PRINT("element_cnt = %d\n",ele_cnt);
    return retval;
}
