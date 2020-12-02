
/**
 *  \file access_internal.h
 *
 *  Module Internal Header File contains structure definitions including tables
 *  maintained by the module
 */

/*
 *  Copyright (C) 2016. Mindtree Ltd.
 *  All rights reserved.
 */

#ifndef _H_ACCESS_INTERNAL_
#define _H_ACCESS_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_trn_api.h"
#include "MS_net_api.h"

#include "bitarray.h"

/* --------------------------------------------- Global Definitions */

#ifdef ACCESS_NO_DEBUG
#define ACCESS_ERR          EM_debug_null
#else /* ACCESS_NO_DEBUG */
#ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
#define ACCESS_ERR
#else
#define ACCESS_ERR(...)     EM_debug_error(MS_MODULE_ID_ACCESS, __VA_ARGS__)
#endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* ACCESS_NO_DEBUG */

#ifdef ACCESS_DEBUG
#ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
#define ACCESS_TRC
#define ACCESS_INF

#define ACCESS_debug_dump_bytes(data, datalen)

#else
#define ACCESS_TRC(...)     EM_debug_trace(MS_MODULE_ID_ACCESS,__VA_ARGS__)
#define ACCESS_INF(...)     EM_debug_info(MS_MODULE_ID_ACCESS,__VA_ARGS__)

#define ACCESS_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_ACCESS, (data), (datalen))

#endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* ACCESS_DEBUG */
#define ACCESS_TRC          EM_debug_null
#define ACCESS_INF          EM_debug_null

#define ACCESS_debug_dump_bytes(data, datalen)

#endif /* ACCESS_DEBUG */


/** Helper size macro */
#define MS_NETKEY_ENTRY_SIZE                       (sizeof(MS_NETKEY_ENTRY))
#define MS_ACCESS_MODEL_TYPE_SIZE                  (sizeof(MS_ACCESS_MODEL_TYPE))
/** Size */
#define MS_PS_RECORD_ELEMENTS_SIZE                 (sizeof(UINT16) + (sizeof(MS_ACCESS_ELEMENT_TYPE) * MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT)) + sizeof(composition_data_hdr))
#define MS_PS_RECORD_MODELS_SIZE                   (sizeof(UINT16) + (MS_ACCESS_MODEL_TYPE_SIZE * (MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT))))
#define MS_PS_RECORD_SUBNETS_SIZE                  (sizeof(UINT16) + sizeof(ms_iv_index)  + sizeof(ms_netkey_count) + (MS_NETKEY_ENTRY_SIZE * (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS))))
#define MS_PS_RECORD_DEV_KEYS_SIZE                 (sizeof(UINT16) + sizeof(UINT16) + (sizeof(MS_DEV_KEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS)))
#define MS_PS_RECORD_APP_KEYS_SIZE                 (sizeof(UINT16) + (sizeof(MS_APPKEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_APPS)))
#define MS_PS_RECORD_ELEMENT_ADDRS_SIZE            (sizeof(UINT16) + (sizeof(MS_ELEMENT_ADDR_ENTRY) * (1 + MS_CONFIG_LIMITS(MS_MAX_LPNS))))
#define MS_PS_RECORD_VIRTUAL_ADDRS_SIZE            (sizeof(UINT16) + (sizeof(MS_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS)))
#define MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE        (sizeof(UINT16) + (sizeof(MS_NON_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS)))
#define MS_PS_RECORD_SEQ_NUMBER_SIZE               (sizeof(UINT32 /* net_seq_number_state.block_seq_num_max */))
#define MS_PS_RECORD_TX_STATES_FEATURES_SIZE       (sizeof(UINT16) + sizeof(ms_tx_state) + sizeof(ms_features))

/** Offset */
#define MS_PS_RECORD_ELEMENTS_OFFSET               0
#define MS_PS_RECORD_MODELS_OFFSET                 (MS_PS_RECORD_ELEMENTS_OFFSET          + MS_PS_RECORD_ELEMENTS_SIZE)
#define MS_PS_RECORD_SUBNETS_OFFSET                (MS_PS_RECORD_MODELS_OFFSET            + MS_PS_RECORD_MODELS_SIZE)
#define MS_PS_RECORD_DEV_KEYS_OFFSET               (MS_PS_RECORD_SUBNETS_OFFSET           + MS_PS_RECORD_SUBNETS_SIZE)
#define MS_PS_RECORD_APP_KEYS_OFFSET               (MS_PS_RECORD_DEV_KEYS_OFFSET          + MS_PS_RECORD_DEV_KEYS_SIZE)
#define MS_PS_RECORD_ELEMENT_ADDRS_OFFSET          (MS_PS_RECORD_APP_KEYS_OFFSET          + MS_PS_RECORD_APP_KEYS_SIZE)
#define MS_PS_RECORD_VIRTUAL_ADDRS_OFFSET          (MS_PS_RECORD_ELEMENT_ADDRS_OFFSET     + MS_PS_RECORD_ELEMENT_ADDRS_SIZE)
#define MS_PS_RECORD_NON_VIRTUAL_ADDRS_OFFSET      (MS_PS_RECORD_VIRTUAL_ADDRS_OFFSET     + MS_PS_RECORD_VIRTUAL_ADDRS_SIZE)
#define MS_PS_RECORD_SEQ_NUMBER_OFFSET             (MS_PS_RECORD_NON_VIRTUAL_ADDRS_OFFSET + MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE)
#define MS_PS_RECORD_TX_STATES_FEATURES_OFFSET     (MS_PS_RECORD_SEQ_NUMBER_OFFSET        + MS_PS_RECORD_SEQ_NUMBER_SIZE)

/* Macro definition for the total size of the Access Layer Persistent Storage. */
#define MS_PS_RECORD_CORE_MODULES_OFFSET           (MS_PS_RECORD_TX_STATES_FEATURES_OFFSET + MS_PS_RECORD_TX_STATES_FEATURES_SIZE)


/**
 *  Locks the ACCESS Mutex which prevents any global variable being
 *  overwritten by any function. It returns an error if mutex lock fails.
 */
#define ACCESS_LOCK()\
        EM_MUTEX_LOCK(access_mutex, ACCESS)

/**
 *  Locks the ACCESS_mutex which prevents any global variable being
 *  overwritten by any function. To be used in void function as it
 *  returns no error.
 */
#define ACCESS_LOCK_VOID()\
        EM_MUTEX_LOCK_VOID(access_mutex, ACCESS)

/**
 *  Unlocks the ACCESS_mutex which realeses the global variables
 *  to be written into. It returns an error if mutex unlock fails.
 */
#define ACCESS_UNLOCK()\
        EM_MUTEX_UNLOCK(access_mutex, ACCESS)

/**
 *  Unlocks the ACCESS_mutex which realeses the global variables
 *  to be written into. To be used in void functions as it returns
 *  no error.
 */
#define ACCESS_UNLOCK_VOID()\
        EM_MUTEX_UNLOCK_VOID(access_mutex, ACCESS)


#define access_alloc_mem(size)\
        EM_alloc_mem(size)

#define access_free_mem(ptr)\
        EM_free_mem(ptr)

#ifndef ACCESS_NO_NULL_PARAM_CHECK
/** Null Check of Transport API Parameters */
#define ACCESS_NULL_CHECK(x) \
        if (NULL == (x)) \
        { \
            ACCESS_ERR(  \
            "[ACCESS] NULL Pointer detected. Referrence Impossible\n"); \
            return ACCESS_NULL_PARAMETER_NOT_ALLOWED; \
        }
#else
#define ACCESS_NULL_CHECK(x)
#endif /* ACCESS_NO_NULL_PARAM_CHECK */

#ifndef MS_ACCESS_NO_RANGE_CHECK
/** Range Check for Transport API Parameters */
#define ACCESS_RANGE_CHECK_START(param, start) \
        if ( ! ((param) >= (start)) ) \
        { \
            ACCESS_ERR( \
            "[ACCESS] ACCESS Range Check FAILED\n"); \
            return ACCESS_PARAMETER_OUTSIDE_RANGE; \
        }

#define ACCESS_RANGE_CHECK_END(param, end) \
        if ( ! ((param) <= (end)) ) \
        { \
            ACCESS_ERR( \
            "[ACCESS] ACCESS Range Check FAILED\n"); \
            return ACCESS_PARAMETER_OUTSIDE_RANGE; \
        }

#define ACCESS_RANGE_CHECK(param, start, end) \
        if ( ! ( ((param) >= (start)) && ((param) <= (end)) ) ) \
        { \
            ACCESS_ERR( \
            "[ACCESS] ACCESS Range Check FAILED\n"); \
            return ACCESS_PARAMETER_OUTSIDE_RANGE; \
        }

#else
#define ACCESS_RANGE_CHECK_START(param, start)
#define ACCESS_RANGE_CHECK_END(param, end)
#define ACCESS_RANGE_CHECK(param, start, end)
#endif /* ACCESS_NO_RANGE_CHECK */

/* Network IVI Mask */
#define ACCESS_NET_IVI_MASK            0x00000001

extern UINT8 access_default_ttl;
extern UINT8      rx_test_ttl;

/* Macro to get default TTL primary unicast address */
#define ACCESS_CM_GET_DEFAULT_TTL(ttl) \
        (ttl) = access_default_ttl

/* Macro to get rx TTL primary unicast address */
#define ACCESS_CM_GET_RX_TTL(ttl) \
        (ttl) = rx_test_ttl


/** Maximum number of Records in Persistent Storage */
#define MS_PS_ACCESS_MAX_RECORDS       10

/** Bitmask for all PS Access Record types */
#define MS_PS_ACCESS_ALL_RECORDS       ((1 << MS_PS_ACCESS_MAX_RECORDS) - 1)

/** Persistet Storage Records (bitfield values) */
#define MS_PS_RECORD_ELEMENTS           0x00000001
#define MS_PS_RECORD_MODELS             0x00000002
#define MS_PS_RECORD_SUBNETS            0x00000004
#define MS_PS_RECORD_DEV_KEYS           0x00000008
#define MS_PS_RECORD_APP_KEYS           0x00000010
#define MS_PS_RECORD_ELEMENT_ADDRS      0x00000020
#define MS_PS_RECORD_VIRTUAL_ADDRS      0x00000040
#define MS_PS_RECORD_NON_VIRTUAL_ADDRS  0x00000080
#define MS_PS_RECORD_SEQ_NUMBER         0x00000100
#define MS_PS_RECORD_TX_STATES_FEATURES 0x00000200

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
#define MS_ACCESS_SET_PUBLICATION_CALLBACK(model_list_entity, model_ptr) \
        (model_list_entity).fixed.pub_cb = (model_ptr)->pub_cb
#else
#define MS_ACCESS_SET_PUBLICATION_CALLBACK(model_list_entity, model_ptr)
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

/* --------------------------------------------- Data Types/ Structures */
/** Current IV Index and Update State */
typedef struct _MS_ACCESS_IV_INDEX
{
    /** Current IV Index */
    UINT32 iv_index;

    /** Current IV Update State */
    UINT8  iv_update_state;

    /** IV Update timestamp */
    EM_time_type iv_time;

} MS_ACCESS_IV_INDEX;

/**
 * Composition Data Page 0 Header
 */
typedef struct _MS_ACCESS_COMPOSITION_DATA_PAGE_0_HDR
{
    /** CID: 16-bit Company Identifier assigned by BT SIG */
    UINT16    ms_access_cid;

    /** PID: 16-bit Product Identifier */
    UINT16    ms_access_pid;

    /** VID: 16-bit Product Version Identifier */
    UINT16    ms_access_vid;

    /**
     *  CRPL: 16-bit value representing the minimum number
     *  of replay protection list entries
     */
    UINT16    ms_access_crpl;

    /** Features: bit field indicating the device features */
    UINT16    ms_access_features;

} MS_ACCESS_COMPOSITION_DATA_PAGE_0_HDR;

/**
 * Access Element data structure.
 */
typedef struct _MS_ACCESS_ELEMENT_TYPE
{
    /**
     * Location descriptor.
     * Defined in Bluetooth SIG GATT Namespace Descriptors section
     * https://www.bluetooth.com/specifications/assigned-numbers/gatt-namespace-descriptors
     */
    UINT16 loc;

    /**
     *  Unicast Address assigned to the element
     *  during the provisioning procedure.
     *
     *  Maintained in a separate MS_ELEMENT_ADDR_ENTRY
     *  data structure.
     */

    /** Number of SIG Model IDs */
    UINT8  num_s;

    /** Number of Vendor Model IDs */
    UINT8  num_v;

    /** Valid ? */
    UINT8  valid;

} MS_ACCESS_ELEMENT_TYPE;

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
/* Periodic Step Timer data structure */
typedef struct _MS_ACCESS_PERIODIC_STEP_TIMER_TYPE
{
    /* Steps in 100ms resolution */
    UINT32 step_count;

    /* Associated Model Handle */
    MS_ACCESS_MODEL_HANDLE  handle;

    /* Expiry Counter */
    UINT32                  expiry_counter;

    /* Next Element in the Periodic Timer */
    struct _MS_ACCESS_PERIODIC_STEP_TIMER_TYPE *next;

}MS_ACCESS_PERIODIC_STEP_TIMER_TYPE;

typedef MS_ACCESS_PERIODIC_STEP_TIMER_TYPE *  MS_ACCESS_PERIODIC_STEP_TIMER_HANDLE;
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

/**
 * Access Publication related meta information
 * Many of the fields are similat to MS_ACCESS_PUBLISH_INFO
 */
typedef struct _MS_ACCESS_PUBLISH_META_INFO
{
    /** PublishAddress Handle */
    MS_ACCESS_ADDRESS_HANDLE addr_handle;

    /** AppKey Handle */
    UINT16          appkey_handle;

    /** CredentialFlag */
    UINT8           crden_flag;

    /** PublishTTL */
    UINT8           ttl;

    /** PublishPeriod */
    UINT8           period;

    /** Fast Period Divisor */
    UINT8           period_divisor;

    /** PublishRetransmitCount */
    UINT8           rtx_count;

    /** PublishRetransmitIntervalSteps */
    UINT8           rtx_interval_steps;

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
    /** Perioding Timer Handle */
    MS_ACCESS_PERIODIC_STEP_TIMER_HANDLE timer_handle;
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

    /** Valid ? */
    UINT8  valid;

} MS_ACCESS_PUBLISH_META_INFO;

/**
 * Access Model data structure.
 */
typedef struct _MS_ACCESS_MODEL_TYPE
{
    struct
    {
        /** Model ID */
        MS_ACCESS_MODEL_ID model_id;

        /** Associated Element Handle */
        MS_ACCESS_ELEMENT_HANDLE elem_handle;

        /**
         * Callback function pointer to receive packets from the underlying
         * protocol layers
         */
        MS_ACCESS_MODEL_CB cb;

        /** Number of Opcodes */
        UINT16             num_opcodes;

        /** List of Opcodes */
        DECL_CONST UINT32  * opcodes;

        /** Publish Meta Information */
        MS_ACCESS_PUBLISH_META_INFO publish;

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
        /**
         * Callback function called when Publication Timer expires.
         * Set to NULL if model does not support periodic publication.
         */
        MS_ACCESS_MODEL_PUBLISH_TIMEOUT_CB   pub_cb;
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

        /** TBD: Subscription List */

        /** Valid ? */
        UINT8              valid;
    } fixed;

    struct
    {
        /** Bitarry to maintain AppKey mapping */
        MS_DEFINE_GLOBAL_ARRAY(UINT32, appkey_bitarray, BITARRAY_NUM_BLOCKS(MS_CONFIG_LIMITS(MS_MAX_APPS)));

        /** Bitarry to maintain Subscription List mapping */
        MS_DEFINE_GLOBAL_ARRAY(UINT32, subscription_bitarray, BITARRAY_NUM_BLOCKS(MS_CONFIG_LIMITS(MS_MAX_ADDRS)));
    } config;

} MS_ACCESS_MODEL_TYPE;

/**
 * Element Address Data Structure.
 * Used to maintain element address of local device
 * and also of friend devices (LPNs).
 */
typedef struct _MS_ELEMENT_ADDR_ENTRY
{
    /** Unicast Address of the first element */
    MS_NET_ADDR    uaddr;

    /** Number of Elements */
    UINT8         element_count;

} MS_ELEMENT_ADDR_ENTRY;

/**
 * AppKey List
 *
 * - Config AppKey Add
 * - Config AppKey Update
 * - Config AppKey Delete
 * - Config AppKey Get
 * - Config AppKey List
 */
typedef struct _MS_APPKEY_ENTRY
{
    /** Associated Subnet Handle */
    MS_SUBNET_HANDLE    subnet_handle;

    /**
     * AppKey
     * [0]-th index: Current Key
     * [1]-st index: Updated Key
     */
    UINT8   app_key[2][16];

    /**
     * AppKey Index.
     * - 12 bit value
     * - Ranging from 0x000 to 0xFFF inclusive.
     */
    UINT16  appkey_index;

    /**
     * Application Key Identifier
     * [0]-th index: Current Key AID
     * [1]-st index: Updated Key AID
     */
    UCHAR   aid[2];

    /** Key Refresh Phase State */
    UCHAR  key_refresh_phase;

} MS_APPKEY_ENTRY;

/**
 * NetKey List
 */
typedef struct _MS_NETKEY_ENTRY
{
    struct
    {
        /**
         * Net Key Index.
         * - 12 bit value
         * - Ranging from 0x000 to 0xFFF inclusive.
         *
         * Net Key Index will be same in Key Refresh.
         */
        UINT16  netkey_index;

        /**
         * Network Key
         * [0]-th index: Current Key
         * [1]-st index: Updated Key
         */
        UINT8   net_key[2][16];

        /** Privacy Key */
        UINT8   privacy_key[16];

        /** Encryption Key */
        UINT8   encrypt_key[16];

        /**
         * NID
         * [0]-th index: Current Key NID
         * [1]-st index: Updated Key NID
         */
        UINT8   nid[2];

        /* TODO: Define below fields */
        /** Node Identity State */

        /** Beacon Key */
        UCHAR beacon_key[16];

        /** Network ID */
        UCHAR network_id[8];

        /** Identity Key */
        /* TODO: Do we need if Proxy Feature is not supported */
        UCHAR identity_key[16];

#ifdef MS_PROXY_SUPPORT
        /** Node Identity State */
        UCHAR  node_id_state;
#endif /* MS_PROXY_SUPPORT */

        /** Key Refresh Phase State */
        UCHAR  key_refresh_phase;
    } fixed;

    struct
    {
        /** Bitarry to maintain AppKey mapping */
        MS_DEFINE_GLOBAL_ARRAY(UINT32, appkey_bitarray, BITARRAY_NUM_BLOCKS(MS_CONFIG_LIMITS(MS_MAX_APPS)));

        /** Bitarry to maintain Associated Friend/LPN Credentials mapping */
        /** 0-th location of Friend will contain associated master subnet handle */
        MS_DEFINE_GLOBAL_ARRAY(UINT32, friend_bitarray, BITARRAY_NUM_BLOCKS(MS_CONFIG_LIMITS(MS_MAX_LPNS)));
    } config;

} MS_NETKEY_ENTRY;

/**
 * DeviceKey Data Structure, containing Primary Element Address and
 * number of elements.
 */
typedef struct _MS_DEV_KEY_ENTRY
{
    /** Device Key */
    UINT8   dev_key[16];

    /** Unicast Address of the first element */
    MS_NET_ADDR    uaddr;

    /** Number of Elements */
    UCHAR num_elements;

} MS_DEV_KEY_ENTRY;

/**
 * Virtual Address Data Structure
 */
typedef struct _MS_VIRTUAL_ADDR_ENTRY
{
    /** Label UUID */
    UINT8          label[MS_LABEL_UUID_LENGTH];

    /**
     * Virtual Address calculated from Label.
     *
     * Use this also as valid/free flag.
     * When set to ZERO, the entry is free.
     */
    MS_NET_ADDR    vaddr;

    /**
     *  Index 0 - Publication count.
     *  Index 1 - Subscription count.
     *
     *  When both publish and subscribe counts reach ZERO,
     *  the virtual address will be deleted.
     */
    UINT8          count[2];

} MS_VIRTUAL_ADDR_ENTRY;

/**
 * Non-Virtual Address Data Structure
 */
typedef struct _MS_NON_VIRTUAL_ADDR_ENTRY
{
    /**
     * Non-Virtual Address.
     *
     * Use this also as valid/free flag.
     * When set to ZERO, the entry is free.
     */
    MS_NET_ADDR    vaddr;

    /**
     *  Index 0 - Publication count.
     *  Index 1 - Subscription count.
     *
     *  When both publish and subscribe counts reach ZERO,
     *  the non-virtual address will be deleted.
     */
    UINT8          count[2];

} MS_NON_VIRTUAL_ADDR_ENTRY;

/**
 *  Network/Relay Transmit
 *
 *  - Config Network/Relay Transmit Get
 *  - Config Network/Relay Transmit Set
 */
typedef struct _MS_CONFIG_TRANSMIT
{
    /** Transmit Count */
    UINT8    tx_count;

    /** Transmit Interval Steps */
    UINT8   tx_steps;

} MS_CONFIG_TRANSMIT;

/**
 * Address Specific Operation Type.
 * Publish or Subscribe.
 */
typedef UINT8 MS_ADDR_OP_TYPE;
extern MS_ACCESS_COMPOSITION_DATA_PAGE_0_HDR composition_data_hdr;
extern MS_ACCESS_IV_INDEX ms_iv_index;
extern UINT16          ms_netkey_count;
extern MS_CONFIG_TRANSMIT ms_tx_state[2];
extern UINT8 ms_features;


/** Address Operation Type - Publish - also used as index */
#define MS_ADDR_OP_TYPE_PUBLISH      0x00
/** Address Operation Type - Subscribe - also used as index */
#define MS_ADDR_OP_TYPE_SUBSCRIBE    0x01

/* --------------------------------------------- Functions */
/**
 *  \brief Initializes Supported Features.
 *
 *  \par Description
 *  This routine initializes supported feautures, which can be one or more
 *  of the following
 *  - Relay, Proxy, Friend, Low Power
 */
void ms_access_init_supported_features(void);

/**
 *  \par Description
 *  This function handles the incoming data received over upper transport layer.
 *
 *  \param [in] net_hdr
 *         Received Network Packet Header
 *  \param [in] subnet_handle
 *         Handle identifying associated subnet on which the packet is received
 *  \param [in] appkey_handle
 *         Handle identifying application key associated with the received packet
 *  \param [in] pdata
 *         The incoming Data Packet
 *  \param [in] pdatalen
 *         Size of the incoming Data Packet
 */
void access_pkt_in
     (
         /* IN */ MS_NET_HEADER     * net_hdr,
         /* IN */ MS_SUBNET_HANDLE    subnet_handle,
         /* IN */ MS_APPKEY_HANDLE    appkey_handle,
         /* IN */ UCHAR             * pdata,
         /* IN */ UINT16              pdatalen
     );

API_RESULT ms_access_allocate_element(/* OUT */ MS_ACCESS_ELEMENT_HANDLE   * handle);

/* Function to search and allocate a free model */
API_RESULT ms_access_search_and_allocate_model
           (
               /* IN */  MS_ACCESS_MODEL   * model,
               /* OUT */ UINT16            * id
           );
void ms_access_handle_rx_opcode
     (
         /* IN */ MS_NET_HEADER     * net_hdr,
         /* IN */ MS_SUBNET_HANDLE    subnet_handle,
         /* IN */ MS_APPKEY_HANDLE    appkey_handle,
         /* IN */ UINT32              opcode,
         /* IN */ UCHAR             * udata,
         /* IN */ UINT16              udatalen
     );
API_RESULT ms_access_is_opcode_in_model
           (
               /* IN */ UINT32  opcode,
               /* IN */ UINT32  model_index
           );
void ms_access_get_element_models
     (
         /* IN */    UINT32   element_index,
         /* OUT */   UINT32 * model_indices,
         /* INOUT */ UINT32 * model_count
     );

/* Configuration Manager related internal functions */
API_RESULT ms_access_cm_init(void);

/** Init Subnet/Net Key Table */
void ms_access_cm_init_subnet_table(void);

/** This routines adds/updates netkeys */
void ms_access_cm_save_netkey_at_offset
     (
         /* IN */ MS_SUBNET_HANDLE    handle,
         /* IN */ UINT16              netkey_index,
         /* IN */ UINT8             * net_key,
         /* IN */ UINT8               offset
     );

/* This function creates keys associated with Netkey */
void ms_access_cm_create_keys_from_netkey
     (
         /* IN */ MS_SUBNET_HANDLE    handle
     );

/** Update Key Refresh Phase for Netkey */
UINT8 ms_access_cm_update_key_refresh_phase
      (
          /* IN */ MS_SUBNET_HANDLE    handle,
          /* IN */ UINT8               key_refresh_phase
      );

/** Update Key Refresh Phase for all associated Appkey */
void ms_access_cm_refresh_all_appkeys
     (
         /* IN */ MS_SUBNET_HANDLE    subnet_handle,
         /* IN */ UINT8               key_refresh_phase
     );

/** Update all associated Friend Credentials */
void ms_access_cm_refresh_all_friend_credentials
     (
         /* IN */ MS_SUBNET_HANDLE    subnet_handle,
         /* IN */ UINT8               key_refresh_phase
     );

/* Friend Credentials refresh */
void ms_access_cm_friend_credentials_refresh
     (
          /* IN */ MS_SUBNET_HANDLE    friend_subnet_handle,
          /* IN */ MS_SUBNET_HANDLE    master_subnet_handle,
          /* IN */ UINT8               key_refresh_phase
     );

/** Update Key Refresh Phase for Appkey */
UINT8 ms_access_cm_appkey_refresh
      (
          /* IN */ MS_APPKEY_HANDLE    handle,
          /* IN */ UINT8               key_refresh_phase
      );

/** Init App Key Table */
void ms_access_cm_init_appkey_table(void);

/** Init Device Key Table */
void ms_access_cm_init_dev_key_table(void);

/** Search and add an entry to Virtual Address Table */
API_RESULT ms_search_and_add_virtual_address
           (
               /* IN */  UCHAR                    * label,
               /* IN */  MS_ADDR_OP_TYPE            type,
               /* OUT */ UINT16                   * addr,
               /* OUT */ MS_ACCESS_ADDRESS_HANDLE * handle
           );

/** Search an entry to Virtual Address Table */
API_RESULT ms_search_virtual_address_on_label
           (
               /* IN */  UCHAR                    * label,
               /* IN */  MS_ADDR_OP_TYPE            type,
               /* OUT */ UINT16                   * addr,
               /* OUT */ MS_ACCESS_ADDRESS_HANDLE * handle
           );

API_RESULT ms_search_virtual_address
           (
               /* IN */  MS_NET_ADDR              * addr,
               /* IN */  MS_ADDR_OP_TYPE            type,
               /* OUT */ MS_ACCESS_ADDRESS_HANDLE * handle
           );

/** Search and add an entry to Non-Virtual Address Table */
API_RESULT ms_search_and_add_address
           (
               /* IN */  MS_NET_ADDR              * addr,
               /* IN */  MS_ADDR_OP_TYPE            type,
               /* OUT */ MS_ACCESS_ADDRESS_HANDLE * handle
           );

/** Search an entry to Non-Virtual Address Table */
API_RESULT ms_search_address
           (
               /* IN */  MS_NET_ADDR              * addr,
               /* IN */  MS_ADDR_OP_TYPE            type,
               /* OUT */ MS_ACCESS_ADDRESS_HANDLE * handle
           );

/** Delete an entry from Non-Virtual/Virtual Address Table */
API_RESULT ms_delete_address
           (
               /* IN */ MS_ACCESS_ADDRESS_HANDLE addr_handle,
               /* IN */ MS_ADDR_OP_TYPE            type
           );

/** Get an entry from Non-Virtual/Virtual Address Table */
API_RESULT ms_get_address
           (
               /* IN */  MS_ACCESS_ADDRESS_HANDLE   addr_handle,
               /* IN */  MS_ADDR_OP_TYPE            type,
               /* OUT */ MS_NET_ADDR              * addr
           );

/** Delete appkey */
void ms_delete_appkey(/* IN */ UINT32 key_index);

/** Get Publish Address */
API_RESULT ms_access_get_publish_addr
           (
               /* IN */ MS_ACCESS_MODEL_HANDLE  * handle,
               /* IN */ MS_NET_ADDR             * publish_addr
           );

/** Publish */
API_RESULT ms_access_publish_ex
           (
               /* IN */ MS_ACCESS_MODEL_HANDLE  * handle,
               /* IN */ UINT32                    opcode,
               /* IN */ MS_NET_ADDR               dst_addr,
               /* IN */ UCHAR                   * data_param,
               /* IN */ UINT16                    data_len,
               /* IN */ UINT8                     reliable
           );

#ifdef MS_STORAGE
void access_ps_init (void);

/**
 *  \brief Store Access Layer information in persistent storage.
 *
 *  \par Description
 *  This function saves in RAM data structures/configurations in persistent storage.
 *  This function will be called whenever there is change in any of the data
 *  structures/configurations which need to be stored.
 *
 *  \param [in] records    This is a bitmask identifying the records to be stored.
 */
void ms_access_ps_store (/* IN */ UINT32 records);

/**
 *  \brief Load Access Layer information from persistent storage.
 *
 *  \par Description
 *  This function loads data structures/configurations which are already
 *  saved in persistent storage to in RAM data structures.
 *  This function will generally be called only once during the power of
 *  initialization.
 *
 *  \param [in] records    This is a bitmask identifying the records to be loaded.
 */
void ms_access_ps_load (/* IN */ UINT32 records);

#else

#define access_ps_init()
#define ms_access_ps_store(records)
#define ms_access_ps_load(records)

#endif /* MS_STORAGE */

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
/* Periodic Step Timer - Initialization Routine */
API_RESULT ms_access_init_periodic_step_timer(void);

/* Periodic Step Timer - Start Routine */
API_RESULT ms_access_start_periodic_step_timer
           (
               /* IN */  UINT8                                  period,
               /* IN */  UINT8                                  period_divisor,
               /* IN */  MS_ACCESS_MODEL_HANDLE               * handle,
               /* OUT */ MS_ACCESS_PERIODIC_STEP_TIMER_HANDLE * timer_handle
           );

/* Periodic Step Timer - Stop Routine */
API_RESULT ms_access_stop_periodic_step_timer
           (
               /* IN */ MS_ACCESS_PERIODIC_STEP_TIMER_HANDLE timer_handle
           );

#define MS_ACCESS_INIT_PERIODIC_STEP_TIMER() \
        ms_access_init_periodic_step_timer()

#define MS_ACCESS_START_PERIODIC_STEP_TIMER(period, period_divisor, handle, timer_handle) \
        ms_access_start_periodic_step_timer((period), (period_divisor), (handle), (timer_handle))

#define MS_ACCESS_STOP_PERIODIC_STEP_TIMER(publish) \
        if (NULL != (publish).timer_handle) \
        { \
            ms_access_stop_periodic_step_timer((publish).timer_handle); \
          \
            /* TODO: Check the return value */ \
            (publish).timer_handle = NULL; \
        }
#else
#define MS_ACCESS_INIT_PERIODIC_STEP_TIMER()

#define MS_ACCESS_START_PERIODIC_STEP_TIMER(period, period_divisor, handle, timer_handle) \
        API_FAILURE

#define MS_ACCESS_STOP_PERIODIC_STEP_TIMER(publish)
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

#endif /* _H_ACCESS_INTERNAL_ */

