#include "aliGenie_appl_node_mng.h"
#include "aliGenie_appl_Generic.h"

typedef struct _model_tree
{
    UINT8                       using_flg;

    MS_NET_ADDR                 element_addr;
    UINT32                      model_id;
    MS_ACCESS_ELEMENT_HANDLE    element_handle;
    MS_ACCESS_MODEL_HANDLE      model_handle;
    MS_ACCESS_MODEL_HANDLE      generic_scene_model_handle;
    void*                       p_private_data;
} MODEL_TREE_T;

MODEL_TREE_T g_appl_model_tree[MODEL_TREE_MAXNUM] = {0};

void appl_model_tree_reset()
{
    /* !!! YOU MUST FREE p_private_data BEFORE THIS !!!  */
    EM_mem_set(g_appl_model_tree,0,sizeof(MODEL_TREE_T)*MODEL_TREE_MAXNUM);
}

bool appl_model_tree_add(
    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    MS_ACCESS_MODEL_HANDLE      model_handle,
    void*                       p_private_data
)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if((g_appl_model_tree[i].element_handle == element_handle)
                && (g_appl_model_tree[i].model_handle == model_handle)
                &&  g_appl_model_tree[i].using_flg != 0)
            return FALSE;
    }

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].using_flg == 0)
            break;
    }

    if (i >= MODEL_TREE_MAXNUM)
        return FALSE;

    g_appl_model_tree[i].using_flg = 1;
    g_appl_model_tree[i].element_handle = element_handle;
    g_appl_model_tree[i].model_handle   = model_handle;
    g_appl_model_tree[i].p_private_data = p_private_data;
    return TRUE;
}


bool appl_model_tree_del(
    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    MS_ACCESS_MODEL_HANDLE      model_handle
)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if((g_appl_model_tree[i].element_handle == element_handle) && (g_appl_model_tree[i].model_handle == model_handle))
        {
            EM_mem_set(&g_appl_model_tree[i],0,sizeof(MODEL_TREE_T));
            return true;
        }
    }

    return FALSE;
}

bool find_element_by_model( MS_ACCESS_MODEL_HANDLE model_handle, MS_ACCESS_ELEMENT_HANDLE* element_handle)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].model_handle == model_handle)
        {
            EM_mem_copy(element_handle,&g_appl_model_tree[i].element_handle,sizeof(MS_ACCESS_ELEMENT_HANDLE));
            return TRUE;
        }
    }

    return FALSE;
}
void* find_model_private_data( MS_ACCESS_MODEL_HANDLE model_handle)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].model_handle == model_handle)
        {
            return g_appl_model_tree[i].p_private_data;
        }
    }

    return NULL;
}

void* find_model_private_data_ch( UINT16 ch)
{
    int i;
    UI_DATA_GENERIC_ONOFF_MODEL_T* me;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        me= (UI_DATA_GENERIC_ONOFF_MODEL_T*)g_appl_model_tree[i].p_private_data;

        if(me->io_num==ch)
        {
            return g_appl_model_tree[i].p_private_data;
        }
    }

    return NULL;
}

UINT16 find_model_handles(MS_ACCESS_MODEL_HANDLE model_handle_list[])
{
    int i = 0;
    int cnt = 0;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].using_flg > 0)
        {
            model_handle_list[i] = g_appl_model_tree[i].model_handle;
            cnt++;
        }
    }

    return cnt;
}

bool set_element_addr_by_model_handle( MS_ACCESS_MODEL_HANDLE model_handle, MS_NET_ADDR element_addr)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].model_handle == model_handle)
        {
            g_appl_model_tree[i].element_addr = element_addr;
            return true;
        }
    }

    return false;
}



bool set_all_element_addr_by_element_handle( MS_ACCESS_ELEMENT_HANDLE element_handle, MS_NET_ADDR element_addr)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].element_handle == element_handle)
        {
            g_appl_model_tree[i].element_addr = element_addr;
        }
    }

    return true;
}

bool get_model_handles_of_element( MS_ACCESS_MODEL_HANDLE* model_handle, MS_ACCESS_ELEMENT_HANDLE element_handle)
{
    int i;

    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].element_handle == element_handle)
        {
            model_handle[i] = g_appl_model_tree[i].model_handle;
        }
    }

    return TRUE;
}

bool set_generic_scene_modele_handle( MS_ACCESS_MODEL_HANDLE model_handle)
{
    int i;

    //find my element_handle
    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].model_handle == model_handle)
        {
            g_appl_model_tree[i].generic_scene_model_handle = model_handle;
            return true;
        }
    }

    return false;
}


bool copyget_generic_scene_modele_handle( MS_ACCESS_MODEL_HANDLE model_handle,MS_ACCESS_MODEL_HANDLE* generic_scene_model_handle)
{
    int i,my_index;
    MS_ACCESS_ELEMENT_HANDLE element_handle;

    //find my element_handle
    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if(g_appl_model_tree[i].model_handle == model_handle)
        {
            element_handle = g_appl_model_tree[i].element_handle;
            my_index = i;
            break;
        }
    }

    if(i >= MODEL_TREE_MAXNUM)
    {
        return false;
    }

    //find available generic_scene_model_handle and copy
    for (i=0; i<MODEL_TREE_MAXNUM; i++)
    {
        if((g_appl_model_tree[i].element_handle == element_handle) && (g_appl_model_tree[i].generic_scene_model_handle != 0))
        {
            g_appl_model_tree[my_index].generic_scene_model_handle = g_appl_model_tree[i].generic_scene_model_handle;
            *generic_scene_model_handle = g_appl_model_tree[i].generic_scene_model_handle;
            return true;
        }
    }

    return false;
}

