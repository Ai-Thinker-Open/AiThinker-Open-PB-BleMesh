
#ifndef _ALIGENIE_APPL_NODE_MNG_H_
#define _ALIGENIE_APPL_NODE_MNG_H_

#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_net_api.h"

#define MODEL_TREE_MAXNUM   10
bool find_element_by_model( MS_ACCESS_MODEL_HANDLE model_handle, MS_ACCESS_ELEMENT_HANDLE* element_handle);
bool appl_model_tree_add(MS_ACCESS_ELEMENT_HANDLE element_handle,MS_ACCESS_MODEL_HANDLE model_handle,void*  p_private_data);
void* find_model_private_data( MS_ACCESS_MODEL_HANDLE model_handle);
UINT16 find_model_handles(MS_ACCESS_MODEL_HANDLE model_handle_list[]);
bool set_element_addr_by_model_handle( MS_ACCESS_MODEL_HANDLE model_handle, MS_NET_ADDR element_addr);
bool copyget_generic_scene_modele_handle( MS_ACCESS_MODEL_HANDLE model_handle,MS_ACCESS_MODEL_HANDLE* generic_scene_model_handle);
bool set_generic_scene_modele_handle( MS_ACCESS_MODEL_HANDLE model_handle);
bool set_all_element_addr_by_element_handle( MS_ACCESS_ELEMENT_HANDLE element_handle, MS_NET_ADDR element_addr);
bool get_model_handles_of_element( MS_ACCESS_MODEL_HANDLE* model_handle, MS_ACCESS_ELEMENT_HANDLE element_handle);

#endif //_ALIGENIE_APPL_NODE_MNG_H_

