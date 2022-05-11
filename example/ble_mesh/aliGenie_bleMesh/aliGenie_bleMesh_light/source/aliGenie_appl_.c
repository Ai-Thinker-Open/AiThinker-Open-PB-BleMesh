#include "aliGenie_appl.h"
#include "aliGenie_appl_Generic.h"
#include "aliGenie_appl_Light.h"

API_RESULT aliGenie_appl_create_elements_and_models(MS_ACCESS_NODE_ID node_id,MS_ACCESS_ELEMENT_HANDLE element_handle1)
{
    MS_ACCESS_ELEMENT_DESC   element;
    UINT8 ele_cnt;
    API_RESULT retval;

    if (API_SUCCESS == retval)
    {
        /* Register Generic OnOff model server */
        retval = UI_generic_onoff_model_server_create(element_handle1,LIGHT_GREEN);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle1,LIGHT_GREEN);
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
        retval = UI_generic_onoff_model_server_create(element_handle2,LIGHT_BLUE);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle2,LIGHT_BLUE);
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
        retval = UI_generic_onoff_model_server_create(element_handle3,LIGHT_RED);
    }

    if (API_SUCCESS == retval)
    {
        /* Register Vendor Defined model server */
        retval = UI_vendor_aliGenie_model_server_create(node_id,element_handle3,LIGHT_RED);
    }

    #endif

    if (API_SUCCESS == retval)
    {
        /* Initialize model states */
        UI_model_states_initialization();
    }

    MS_access_cm_get_element_count(&ele_cnt);
    DEBUG_PRINT("element_cnt = %d\n",ele_cnt);
    return retval;
}
