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


/**
 *  \file blebrr.c
 *
 *  This File contains the BLE Bearer interface for the
 *  Mindtree Mesh stack.
 */

/*
 *  Copyright (C) 2016. Mindtree Ltd.
 *  All rights reserved.
 */

/* ------------------------------- Header File Inclusion */
#include "MS_brr_api.h"
#include "MS_prov_api.h"
#include "blebrr.h"
#include "clock.h"
#include "ll.h"
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Global Definitions */
#define BLEBRR_MAX_ADV_FILTER_LIST_COUNT    100
#define BLEBRR_MAX_ADV_DATA_SIZE            31

#define BLEBRR_BCON_ELEMENTS                2
#define BLEBRR_QUEUE_SIZE                   64
#define BLEBRR_ACTIVEADV_TIMEOUT            1 /* Second */
#define BLEBRR_ADV_TIMEOUT                  (EM_TIMEOUT_MILLISEC | 10) /* Millisecond */
#define BLEBRR_SCAN_TIMEOUT                 (EM_TIMEOUT_MILLISEC | 100) /* Millisecond */
#define BLEBRR_NCON_ADVTYPE_OFFSET          2
#define BLEBRR_PROADVREPEAT_COUNT           6
#define BLEBRR_ADVREPEAT_COUNT              2
#define BLEBRR_SCAN_ADJ_STEP                15
#define BLEBRR_SCAN_ADJ_THD_MAX             45
#define BLEBRR_SCAN_ADJ_THD_MIN             15   
#define BLEBRR_TURNOFF_RELAY_THD            20
#define BLEBRR_SKIP_BEACON_QUEUE_DEPTH      0

/** Bearer state defines */
#define BLEBRR_STATE_IDLE                   0x00
#define BLEBRR_STATE_IN_SCAN_ENABLE         0x01
#define BLEBRR_STATE_IN_SCAN_DISABLE        0x02
#define BLEBRR_STATE_SCAN_ENABLED           0x04
#define BLEBRR_STATE_IN_ADV_ENABLE          0x10
#define BLEBRR_STATE_IN_ADV_DISABLE         0x20
#define BLEBRR_STATE_ADV_ENABLED            0x40

/** Bearer Queue defines */
#define BLEBRR_QTYPE_DATA                   0x00
#define BLEBRR_QTYPE_BEACON                 0x01
#define BLEBRR_NUM_QUEUES                   0x02

/** Beacon type defines */
#define BLEBRR_UPROV_ADV_BCON               0x00
#define BLEBRR_UPROV_ADV_URI                0x01
#define BLEBRR_UPROV_GATT_BCON              0x02
#define BLEBRR_UPROV_GATT_URI               0x03
#define BLEBRR_SECNET_BCON                  0x04
#define BLEBRR_NUM_BCONS                    0x05

/** GATT Mode GAP Connectable Advertising Service data offset */
#define BLEBRR_GATT_ADV_SERV_DATA_OFFSET    11
#define BLEBRR_GATT_ADV_SERV_DATALEN_OFFSET 7

/** Advertising data maximum length */
#define BLEBRR_GAP_ADVDATA_LEN              31

/** Advertising data sets MAX */
#define BLEBRR_GAP_MAX_ADVDATA_SETS         2

/* --------------------------------------------- Macros */
#define BLEBRR_MUTEX_INIT()                 MS_MUTEX_INIT(blebrr_mutex, BRR);
#define BLEBRR_MUTEX_INIT_VOID()            MS_MUTEX_INIT_VOID(blebrr_mutex, BRR);
#define BLEBRR_LOCK()                       MS_MUTEX_LOCK(blebrr_mutex, BRR);
#define BLEBRR_LOCK_VOID()                  MS_MUTEX_LOCK_VOID(blebrr_mutex, BRR);
#define BLEBRR_UNLOCK()                     MS_MUTEX_UNLOCK(blebrr_mutex, BRR);
#define BLEBRR_UNLOCK_VOID()                MS_MUTEX_UNLOCK_VOID(blebrr_mutex, BRR);

#define BLEBRR_SET_STATE(x)                 blebrr_state = (x)
#define BLEBRR_GET_STATE()                  blebrr_state


extern uint8 UI_prov_state;
/* --------------------------------------------- Structures/Data Types */
/** BLEBRR Data Queue Element */
typedef struct _BLEBRR_Q_ELEMENT
{
    /* "Allocated" Data Pointer */
    UCHAR *pdata;

    /*
     *  Data Length. If data length is zero, the element is considered
     *  invalid.
     */
    UINT16 pdatalen;

    /* Type of data element */
    UCHAR type;

} BLEBRR_Q_ELEMENT;

/** BLEBRR Data Queue */
typedef struct _BLEBRR_Q
{
    /* List of Bearer Queue elements */
    BLEBRR_Q_ELEMENT element[BLEBRR_QUEUE_SIZE];

    /* Queue start index */
    UINT16 start;

    /* Queue end index */
    UINT16 end;

} BLEBRR_Q;

/** Advertising Data type */
typedef struct _BLEBRR_GAP_ADV_DATA
{
    /** Data */
    UCHAR data[BLEBRR_GAP_ADVDATA_LEN];

    /** Data Length */
    UCHAR datalen;

} BLEBRR_GAP_ADV_DATA;


/* --------------------------------------------- Global Variables */
#ifdef BLEBRR_FILTER_DUPLICATE_PACKETS
DECL_STATIC UCHAR blebrr_adv_list[BLEBRR_MAX_ADV_FILTER_LIST_COUNT][BLEBRR_MAX_ADV_DATA_SIZE];
DECL_STATIC UCHAR blebrr_adv_list_inser_index = 0;
#endif /* BLEBRR_FILTER_DUPLICATE_PACKETS */

 BRR_BEARER_INFO blebrr_adv;  //HZF
//DECL_STATIC BRR_BEARER_INFO blebrr_adv;

DECL_STATIC BRR_HANDLE blebrr_advhandle;

DECL_STATIC UCHAR blebrr_bconidx;
DECL_STATIC UCHAR blebrr_beacon;
DECL_STATIC BLEBRR_Q_ELEMENT blebrr_bcon[BRR_BCON_COUNT];
DECL_STATIC BLEBRR_Q blebrr_queue;
/* DECL_STATIC UCHAR blebrr_adv_bcon_type; */

MS_DEFINE_MUTEX_TYPE(static, blebrr_mutex)
DECL_STATIC EM_timer_handle blebrr_timer_handle;
 UCHAR blebrr_state;    // HZF
//DECL_STATIC UCHAR blebrr_state;

UINT32 blebrr_scanTimeOut = BLEBRR_SCAN_ADJ_THD_MAX;

DECL_STATIC UCHAR blebrr_datacount;

/* DECL_STATIC UCHAR blebrr_scan_type; */
DECL_STATIC UCHAR blebrr_advrepeat_count;
DECL_STATIC UCHAR blebrr_scan_interleave;

BLEBRR_GAP_ADV_DATA blebrr_gap_adv_data[BLEBRR_GAP_MAX_ADVDATA_SETS] =
{
    /* Index 0x00: Mesh Provisioning Service ADV Data */
    {
        {
            /**
             *  Flags:
             *      0x01: LE Limited Discoverable Mode
             *      0x02: LE General Discoverable Mode
             *      0x04: BR/EDR Not Supported
             *      0x08: Simultaneous LE and BR/EDR to Same Device
             *            Capable (Controller)
             *      0x10: Simultaneous LE and BR/EDR to Same Device
             *            Capable (Host)
             */
            0x02, 0x01, 0x06,

            /**
             *  Service UUID List:
             *      Mesh Provisioning Service (0x1827)
             */
            0x03, 0x03, 0x27, 0x18,

            /**
             *  Service Data List:
             *      Mesh Provisioning Service (0x1827)
             *      Mesh UUID (16 Bytes)
             *      Mesh OOB Info (2 Bytes)
             */
             0x15, 0x16,
             0x27, 0x18,
             0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
             0x00, 0x00
        },

        /** Advertising Data length */
        29
    },
    /* Index 0x01: Mesh Proxy Service ADV Data */
    {
        {
            /**
             *  Flags:
             *      0x01: LE Limited Discoverable Mode
             *      0x02: LE General Discoverable Mode
             *      0x04: BR/EDR Not Supported
             *      0x08: Simultaneous LE and BR/EDR to Same Device
             *            Capable (Controller)
             *      0x10: Simultaneous LE and BR/EDR to Same Device
             *            Capable (Host)
             */
            0x02, 0x01, 0x06,

            /**
             *  Service UUID List:
             *      Mesh Proxy Service (0x1828)
             */
            0x03, 0x03, 0x28, 0x18,

            /**
             *  Service Data List:
             *      Mesh Provisioning Service (0x1828)
             *      Type (1 Byte) "0x00 - Network ID; 0x01 - Node Identity"
             *      NetWork ID (8 Bytes)
             */
             0x0C, 0x16,
             0x28, 0x18,
             0x00,
             0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88
        },

        /** Advertising Data length */
        20
    }
};

DECL_STATIC UCHAR pl_advdata_offset;
DECL_STATIC UCHAR blebrr_sleep;


// ------------ add by HZF
uint32 blebrr_advscan_timeout_count = 0;

/* ------------------------------- Functions */
void blebrr_handle_evt_adv_complete (UINT8 enable);

API_RESULT blebrr_queue_depth_check(void);

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \return void
 */
void blebrr_scan_enable(void)
{
    BLEBRR_LOCK_VOID();

    if ((BLEBRR_STATE_IDLE == BLEBRR_GET_STATE()) &&
        (MS_TRUE != blebrr_sleep))
    {
        blebrr_scan_pl(MS_TRUE);

        /* Update state */
        BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_ENABLE);
    }

    BLEBRR_UNLOCK_VOID();
}

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param type
 *  \param bcon
 *
 *  \return void
*/
DECL_STATIC UCHAR blebrr_get_beacon_type (UCHAR type, UCHAR bcon)
{
    return (BRR_BCON_PASSIVE == type)?
           ((BRR_BCON_TYPE_UNPROV_DEVICE == bcon)? BLEBRR_UPROV_ADV_BCON: BLEBRR_SECNET_BCON):
           ((BRR_BCON_TYPE_UNPROV_DEVICE == bcon)? BLEBRR_UPROV_GATT_BCON: BLEBRR_NUM_BCONS);
}

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param type
 *
 *  \return void
 */
DECL_STATIC BLEBRR_Q_ELEMENT * blebrr_enqueue_alloc (void)
{
    BLEBRR_Q_ELEMENT * elt;
    UINT16 ei;

    /* Get reference to the requested Queue block members */
    elt = blebrr_queue.element;
    ei = blebrr_queue.end;

    /* Check if queue end element is free */
    if (0 != (elt + ei)->pdatalen)
    {
        /* Not free */
        elt = NULL;
    }
    else
    {
        /* Set the element to be returned */
        elt = (elt + ei);

        /* Update the data availability */
        blebrr_datacount++;

        /* EM_debug_trace (0, "[BLEBRR] Enqueue at Q Index: %d\n", ei); */
        printf ("[BLEBRR] Enqueue at Q Index: %d\r\n", ei);		

        /* Update queue end */
        ei++;
        ei &= (BLEBRR_QUEUE_SIZE - 1);
        blebrr_queue.end = ei;
    }

    return elt;
}

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param type
 *
 *  \return void
 */
DECL_STATIC BLEBRR_Q_ELEMENT * blebrr_dequeue (void)
{
    BLEBRR_Q_ELEMENT * elt;
    UINT16 si;

    /* Get reference to the requested Queue block members */
    elt = blebrr_queue.element;
    si = blebrr_queue.start;

    /* Check if queue start element is valid */
    if (0 == (elt + si)->pdatalen)
    {
        /* Not valid */
        elt = NULL;
    }
    else
    {
        /* Set the element to be returned */
        elt = (elt + si);

        /* EM_debug_trace (0, "[BLEBRR] Dequeue at Q Index: %d\n", si); */
        printf ("[BLEBRR] DeQ Index = %d\r\n", si);

        /* Is Adv data type in element? */
        if (BRR_BCON_COUNT == elt->type)
        {
            /* Update the data availability */
            blebrr_datacount--;
        }

        /* Update queue start */
        si++;
        si &= (BLEBRR_QUEUE_SIZE - 1);
        blebrr_queue.start = si;
    }

    return elt;
}

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param bcon
 *
 *  \return void
 */
DECL_STATIC void blebrr_clear_bcon (UCHAR bconidx)
{
    BLEBRR_Q_ELEMENT * elt;

    /* Get reference to the beacon queue element */
    elt = &blebrr_bcon[bconidx];

    /* Clear the element and the next one for the given type of beacon */
    if (NULL != elt->pdata)
    {
        EM_free_mem (elt->pdata);
        elt->pdata = NULL;
        elt->pdatalen = 0;

        elt->type = BRR_BCON_COUNT;

        if ((BRR_BCON_TYPE_UNPROV_DEVICE == bconidx) &&
            (NULL != (elt + 1)->pdata) &&
            (0 != (elt + 1)->pdatalen))
        {
            EM_free_mem((elt + 1)->pdata);
            (elt + 1)->pdata = NULL;
            (elt + 1)->pdatalen = 0;

            (elt + 1)->type = BRR_BCON_COUNT;
        }

        blebrr_datacount--;
    }
}

UCHAR blebrr_get_queue_depth(void)
{
    UCHAR depth;

    if(blebrr_queue.end>=blebrr_queue.start)
    {
        depth = blebrr_queue.end-blebrr_queue.start;
    }
    else
    {
        depth = BLEBRR_QUEUE_SIZE-(blebrr_queue.start-blebrr_queue.end);
    }
    return depth;
}

API_RESULT blebrr_queue_depth_check(void)
{   
    API_RESULT retval =API_SUCCESS;
    uint8_t randData;
   
    UCHAR depth =blebrr_get_queue_depth();
    
    if(depth>BLEBRR_TURNOFF_RELAY_THD)
    {
        LL_Rand(&randData, 1);
        randData=randData>>1;
        if( depth > randData)
        {
            retval= API_FAILURE;
        }
        BLEBRR_LOG("[Queue DATA CNT] = %d %d %4X\n", depth,randData,retval);
    }

    return retval;
    
}

extern uint8             llState, llSecondaryState;
/**
 *  \brief
 *
 *  \par Description
 *
 * *
 * *  \param void
 *
 *  \return void
 */
DECL_STATIC API_RESULT blebrr_update_advdata(void)
{
    BLEBRR_Q_ELEMENT * elt;
    UCHAR type;
    elt = NULL;

    //ZQY skip bcon adv when queue is not empty
    if(blebrr_get_queue_depth()>BLEBRR_SKIP_BEACON_QUEUE_DEPTH)
        blebrr_beacon=0;

    if (blebrr_beacon)
    {
        UCHAR bconidx;

        bconidx = blebrr_bconidx;

        do
        {
            if (0 != blebrr_bcon[blebrr_bconidx].pdatalen)
            {
                elt = &blebrr_bcon[blebrr_bconidx];
            }

            if (BRR_BCON_COUNT == ++blebrr_bconidx)
            {
                blebrr_bconidx = 0;
            }
        } while ((blebrr_bconidx != bconidx) && (NULL == elt));

        blebrr_beacon = 0;
    }

    if (!blebrr_beacon && NULL == elt)
    {
        elt = blebrr_dequeue();
        blebrr_beacon = 1;
    }

    if (NULL == elt)
    {
        return API_FAILURE;
    }

    /* Set the type */
    type = (BRR_BCON_COUNT == elt->type) ? BRR_BCON_PASSIVE : elt->type;

    /* Update global adv bcon type */
    /* blebrr_adv_bcon_type = type; */

    /* Set the advertising data */
    blebrr_advrepeat_count = 1;
    blebrr_advertise_data_pl(type, elt->pdata, elt->pdatalen);
    /* BLEBRR_LOG("\n ### [ADV-Tx]: blebrr adv send of type 0x%02X:\n", type);
    appl_dump_bytes(elt->pdata, elt->pdatalen); */

    /* Is Adv data type in element? */
    if (BRR_BCON_COUNT == elt->type)
    {
        /* Yes, Free the element */
        EM_free_mem(elt->pdata);
        elt->pdatalen = 0;
    }

    return API_SUCCESS;
}


/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param type
 *  \param pdata
 *  \param datalen
 *  \param elt
 *
 *  \return void
 */
DECL_STATIC void blebrr_send
                 (
                     UCHAR type,
                     void * pdata,
                     UINT16 datalen,
                     BLEBRR_Q_ELEMENT * elt
                 )
{
    API_RESULT retval;
    UCHAR * data;
    UINT16 packet_len;
    UCHAR offset;

    data = (UCHAR *)pdata;

    /* BLEBRR_LOG("[ADV-Tx >]: ");
    BLEBRR_dump_bytes(data, datalen); */

    /* Get the offset based on the type */
    offset = (0 != type)? BLEBRR_NCON_ADVTYPE_OFFSET: 0;

    /* Calculate the total length, including Adv Data Type headers */
    packet_len = datalen + offset;

    /* Allocate and save the data */
    elt->pdata = EM_alloc_mem(packet_len);
    if (NULL == elt->pdata)
    {
        BLEBRR_LOG("Failed to allocate memory!\n");
        return;
    }

    if (offset >= 1)
    {
        /* Add the Length and Adv type headers */
        elt->pdata[0] = (UCHAR)(datalen + (offset - 1));

        if (offset - 1)
        {
            elt->pdata[1] = type;
        }
    }

    /* Update the data and datalen */
    EM_mem_copy((elt->pdata + offset), data, datalen);
    elt->pdatalen = packet_len;
if(UI_prov_state==1) 
{
//printf ("=========\n ");
//    for(int i=0;i<datalen;i++)
//        printf ("%02x ",data[i]);
//printf ("=========\n ");

WaitMs(15);

}

    /* Is the Adv/Scan timer running? */
    if (EM_TIMER_HANDLE_INIT_VAL != blebrr_timer_handle)
    {
        /* Yes. Do nothing */
    }
    else
    {
        /*
         * No. Scan must be enabled. Disable it to trigger alternating
         * Adv/Scan Procedure
         */
        if (BLEBRR_STATE_SCAN_ENABLED == BLEBRR_GET_STATE())
        {


            /* printf ("Trigger Tx...\r\n"); */
            blebrr_scan_pl (MS_FALSE);

            /* Update state */
            BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_DISABLE);
        }
        else if (BLEBRR_STATE_IDLE == BLEBRR_GET_STATE())
        {
            /* No, Enable Advertising with Data */
            retval = blebrr_update_advdata();

            if (API_SUCCESS != retval)
            {
                /* Enable Scan */
                blebrr_scan_pl(MS_TRUE);

                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_ENABLE);
            }
            else
            {
                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IN_ADV_ENABLE);
            }
        }
    }

    return;
}


/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param handle
 *  \param pdata
 *  \param datalen
 *
 *  \return void
 */
DECL_STATIC API_RESULT blebrr_bcon_send(BRR_HANDLE * handle, void * pdata, UINT16 datalen)
{
    BRR_BEACON_INFO * info;
    BLEBRR_Q_ELEMENT * elt;
    UCHAR op, action, type, bcon, bcontype;
    UCHAR bconidx;

    /* Get the beacon information */
    info = (BRR_BEACON_INFO *)pdata;

    /* Get the Operation and Action */
    op = (info->action & 0x0F);
    action = ((info->action & 0xF0) >> 4);

    /* Get the Broadcast/Observe type and Beacon type */
    type = (info->type & 0x0F);
    bcon = ((info->type & 0xF0) >> 4);

    /* Lock */
    BLEBRR_LOCK();

    /* Check the operations */
    switch (op)
    {
        case BRR_OBSERVE:
            /* blebrr_scan_type = type; */
            break;

        case BRR_BROADCAST:
            /* Get the Beacon mapping at the BLEBRR */
            bcontype = blebrr_get_beacon_type (type, bcon);

            /* Set the bcon index */
            bconidx = bcon;

            if (BRR_ENABLE == action)
            {
                /* Update the connectable beacon packet */
                if ((BRR_BCON_ACTIVE == type) && ((NULL != info->bcon_data)))
                {
                    /* Active Beacon (advdata) Source Index */
                    UCHAR abs_index;

                    abs_index = blebrr_gatt_mode_get();

                    if (BLEBRR_GATT_PROV_MODE == abs_index)
                    {
                        /* Copy the incoming UUID and OOB info to global connectable ADV data for PB GATT */
                        /* TODO have a state to decide about provisioned and unprovisioned state */

                        EM_mem_copy
                        (
                            blebrr_gap_adv_data[abs_index].data + BLEBRR_GATT_ADV_SERV_DATA_OFFSET,
                            info->bcon_data + 1,
                            16 + 2
                        );

                        /**
                         * NOTE: It is not need to calculate assign the Service Data Length as
                         *       Service Data length is Fixed for Connectable Provisioning ADV.
                         * This data length is : 1 + 2 + 16 + 2 = 0x15 Bytes, already updated
                         * in the global data strucutre blebrr_gap_adv_data[0].
                         */

                        /* Disable Interleaving */
                        blebrr_scan_interleave = MS_FALSE;
                    }
                    /* Assuming that this Active Beacon is for GATT Proxy*/
                    else
                    {
                        /* Copy the incoming UUID and OOB info to global connectable ADV data for PB GATT */
                        /* TODO have a state to decide about provisioned and unprovisioned state */

                        abs_index = BLEBRR_GATT_PROXY_MODE;

                        /* Copy the incoming Proxy ADV data */
                        EM_mem_copy
                        (
                            blebrr_gap_adv_data[abs_index].data + BLEBRR_GATT_ADV_SERV_DATA_OFFSET,
                            info->bcon_data,
                            info->bcon_datalen
                        );

                        /* Copy the incoming Proxy ADV datalen + the BLEBRR_GATT_ADV_SERV_DATA_OFFSET */
                        blebrr_gap_adv_data[abs_index].datalen = BLEBRR_GATT_ADV_SERV_DATA_OFFSET + info->bcon_datalen;

                        /**
                         * Assign the service data length correctly for Proxy ADVs
                         * Total incoming data + 1 Byte of AD Flags + 2 Bytes of Service UUID
                         */
                        blebrr_gap_adv_data[abs_index].data[BLEBRR_GATT_ADV_SERV_DATALEN_OFFSET] =
                            info->bcon_datalen + 1 + 2;
                    }

                    /* Re-assign updated ADV data to Info Structure */
                    info->bcon_data    = blebrr_gap_adv_data[abs_index].data + pl_advdata_offset;
                    info->bcon_datalen = blebrr_gap_adv_data[abs_index].datalen - pl_advdata_offset;
                }

                /* Check if beacon element is free */
                if (0 != blebrr_bcon[bconidx].pdatalen)
                {
                    /* Unlock */
                    BLEBRR_UNLOCK();

                    BLEBRR_LOG("Beacon Not Free!\n");
                    return API_FAILURE;
                }

                elt = &blebrr_bcon[bconidx];

                blebrr_datacount++;

                /* Update element type */
                elt->type = type;

                /* Schedule to send */
                blebrr_send
                (
                    ((BRR_BCON_TYPE_UNPROV_DEVICE == bcon) &&
                     (BRR_BCON_ACTIVE != type))? MESH_AD_TYPE_BCON : 0,
                    info->bcon_data,
                    info->bcon_datalen,
                    elt
                );

                /* Check if URI data is present for Unprovisioned device */
                if ((BRR_BCON_TYPE_UNPROV_DEVICE == bconidx) &&
                    (NULL != info->uri) &&
                    (NULL != info->uri->payload) &&
                    (0 != info->uri->length))
                {
                    elt = &blebrr_bcon[bconidx + 1];

                    /* Update element type */
                    elt->type = bcontype + 1;

                    /* Schedule to send */
                    blebrr_send
                    (
                        0,
                        info->uri->payload,
                        info->uri->length,
                        elt
                    );
                }
            }
            else
            {
                /* Remove the beacon with type from the queue */
                blebrr_clear_bcon (bconidx);
            }
            break;

        default:
            break;
    }

    /* Unlock */
    BLEBRR_UNLOCK();

    return API_SUCCESS;
}


/**
*  \brief
*
*  \par Description
*
*
*  \param handle
*  \param pdata
*  \param datalen
*
*  \return void
*/
DECL_STATIC API_RESULT blebrr_adv_send(BRR_HANDLE * handle, UCHAR type, void * pdata, UINT16 datalen)
{
    BLEBRR_Q_ELEMENT * elt;

    /* Validate handle */
    if (*handle != blebrr_advhandle)
    {
        return API_FAILURE;
    }

    if ((NULL == pdata) ||
        (0 == datalen))
    {
        return API_FAILURE;
    }

    /* Enable interleaving */
    blebrr_scan_interleave = MS_TRUE;

    /* If beacon type, pass to the handler */
    if (MESH_AD_TYPE_BCON == type)
    {
        BRR_BEACON_INFO * info;

        /* Get reference to the beacon info */
        info = (BRR_BEACON_INFO *)pdata;

        if (BRR_BCON_TYPE_SECURE_NET != (info->type >> 4))
        {
            return blebrr_bcon_send(handle, pdata, datalen);
        }
        else
        {
            /* Update the data and length reference */
            pdata = info->bcon_data;
            datalen = info->bcon_datalen;
        }
    }

    /* Lock */
    BLEBRR_LOCK();

    /* Allocate the next free element in the data queue */
    elt = blebrr_enqueue_alloc();

    /* Is any element free? */
    if (NULL == elt)
    {
        /* Unlock */
        BLEBRR_UNLOCK();

        BLEBRR_LOG("Queue Full! blebrr_advscan_timeout_count = %d, ble state = %d\r\n", blebrr_advscan_timeout_count, BLEBRR_GET_STATE());

        blebrr_scan_pl(FALSE);    // HZF
        return API_FAILURE;
    }

    /* Update element type */
    elt->type = BRR_BCON_COUNT;

    /* Schedule to send */
    blebrr_send
    (
        type,
        pdata,
        datalen,
        elt
    );

    /* Unlock */
    BLEBRR_UNLOCK();

    return API_SUCCESS;
}

#ifdef BLEBRR_LP_SUPPORT
DECL_STATIC void blebrr_adv_sleep(BRR_HANDLE * handle)
{
    BLEBRR_LOCK_VOID();

    /* Set bearer sleep state */
    blebrr_sleep = MS_TRUE;

    if (BLEBRR_STATE_SCAN_ENABLED == BLEBRR_GET_STATE())
    {
        /* Disable Scan */
        blebrr_scan_pl(MS_FALSE);

        /* Update state */
        BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_DISABLE);
    }

    /* Enter platform sleep */
    EM_enter_sleep_pl();

    BLEBRR_UNLOCK_VOID();
}

DECL_STATIC void blebrr_adv_wakeup(BRR_HANDLE * handle, UINT8 mode)
{
    BLEBRR_LOCK_VOID();

    /* Exit platform sleep */
    EM_exit_sleep_pl();

    if (BRR_RX & mode)
    {
        /* Reset bearer sleep state */
        blebrr_sleep = MS_FALSE;

    if (BLEBRR_STATE_IDLE == BLEBRR_GET_STATE())
    {
        /* Enable Scan */
        blebrr_scan_pl(MS_TRUE);

            /* Update state */
            BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_ENABLE);
        }
    }

    BLEBRR_UNLOCK_VOID();
}
#endif /* BLEBRR_LP_SUPPORT */

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param args
 *  \param size
 *
 *  \return void
 */
DECL_STATIC void blebrr_advscan_timeout_handler (void * args, UINT16 size)
{
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);

    BLEBRR_LOCK_VOID();

	blebrr_advscan_timeout_count ++;

    /* Reset Timer Handler */
    blebrr_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

    /* Check the state of AdvScan procedure */
    switch (BLEBRR_GET_STATE())
    {
        case BLEBRR_STATE_ADV_ENABLED:
            /* Disable Adv */
            if (!blebrr_advertise_pl (MS_FALSE))        // HZF
//            blebrr_advertise_pl (MS_FALSE);
                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IN_ADV_DISABLE);

            break;

        case BLEBRR_STATE_SCAN_ENABLED:
            /* Disable Scan */
            blebrr_scan_pl (MS_FALSE);

            /* Update state */
            BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_DISABLE);
            break;

        default:
            /* Should not reach here */
		printf("=======blebrr_advscan_timeout_handler error: state = %2X, state will not change\r\n", BLEBRR_GET_STATE());
            break;
    }

    BLEBRR_UNLOCK_VOID();
}


/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param timeout
 *
 *  \return void
 */
DECL_STATIC void blebrr_timer_start (UINT32 timeout)
{
    EM_RESULT retval;
//printf("<T>");
    retval = EM_start_timer
             (
                 &blebrr_timer_handle,
                 timeout,
                 blebrr_advscan_timeout_handler,
                 NULL,
                 0
             );

    if (EM_SUCCESS != retval)
    {
        /* TODO: Log */
    }
}


/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param enable
 *
 *  \return void
 */
void blebrr_pl_scan_setup (UCHAR enable)
{
    API_RESULT retval;

    BLEBRR_LOCK_VOID();

#ifdef BLEBRR_ENABLE_SCAN_TRACE
    BLEBRR_LOG ("Scan Setup - %d", enable);
#endif /* BLEBRR_ENABLE_SCAN_TRACE */

    /* Is scan enabled? */
    if (MS_TRUE == enable)
    {
        /* Yes, Update state */
        BLEBRR_SET_STATE(BLEBRR_STATE_SCAN_ENABLED);

        /* Is data available in queue to be sent? */
        if (0 != blebrr_datacount)
        {
            /* printf ("DATA: %d\n", blebrr_datacount); */
            UCHAR depth=blebrr_get_queue_depth();

          
            if(depth>8)
            {
                printf("add\n");
                blebrr_scanTimeOut -= BLEBRR_SCAN_ADJ_STEP;

            }
            else
            {
                //printf("~~~add\n");
                blebrr_scanTimeOut += BLEBRR_SCAN_ADJ_STEP; 
            }

            //if(depth>0)
            //    blebrr_scanTimeOut = BLEBRR_SCAN_ADJ_THD_MIN;
            //else
            //    blebrr_scanTimeOut = BLEBRR_SCAN_ADJ_THD_MAX;

            if(blebrr_scanTimeOut > BLEBRR_SCAN_ADJ_THD_MAX)
                blebrr_scanTimeOut = BLEBRR_SCAN_ADJ_THD_MAX;

            if(blebrr_scanTimeOut < BLEBRR_SCAN_ADJ_THD_MIN)
                blebrr_scanTimeOut = BLEBRR_SCAN_ADJ_THD_MIN;    


             /* Yes, Start bearer timer for Scan Timeout */
            blebrr_timer_start ((EM_TIMEOUT_MILLISEC | blebrr_scanTimeOut));
        }
    }
    else
    {
        /* No, Enable Advertising with Data */
        retval = blebrr_update_advdata();

        if (API_SUCCESS != retval)
        {
            if (MS_TRUE != blebrr_sleep)
            {
                /* Enale Scan */
                blebrr_scan_pl(MS_TRUE);

                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_ENABLE);
            }
            else
            {
                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IDLE);
            }
        }
        else
        {
            /* Update state */
            BLEBRR_SET_STATE(BLEBRR_STATE_IN_ADV_ENABLE);
        }
    }

    BLEBRR_UNLOCK_VOID();
}

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param type
 *  \param enable
 *
 *  \return void
 */
void blebrr_pl_advertise_setup (UCHAR enable)
{
    UINT8   blebrr_advrepeat_count_max;
    BLEBRR_LOCK_VOID();

#ifdef BLEBRR_ENABLE_ADV_TRACE
    BLEBRR_LOG ("Adv Setup - %d", enable);
#endif /* BLEBRR_ENABLE_ADV_TRACE */

    /* Is advertise enabled? */
    if (MS_TRUE == enable)
    {
        /* Yes, Update state */
        BLEBRR_SET_STATE(BLEBRR_STATE_ADV_ENABLED);

        /* Start bearer timer for Adv Timeout */
        if (blebrr_scan_interleave == MS_TRUE)
        {
            blebrr_timer_start (BLEBRR_ADV_TIMEOUT);
        }
    }
    else
    {
        blebrr_advrepeat_count_max = (UI_prov_state == 3)?BLEBRR_ADVREPEAT_COUNT:BLEBRR_PROADVREPEAT_COUNT;
        if (blebrr_advrepeat_count_max > blebrr_advrepeat_count)
        {
            blebrr_advrepeat_count++;
            blebrr_advertise_pl(MS_TRUE);

            /* Update state */
            BLEBRR_SET_STATE(BLEBRR_STATE_IN_ADV_ENABLE);
        }
        else
        {
            if (MS_TRUE != blebrr_sleep)
            {
                /* No, Enable Scanning */
                blebrr_scan_pl(MS_TRUE);

                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_ENABLE);
            }
            else
            {
                /* Update state */
                BLEBRR_SET_STATE(BLEBRR_STATE_IDLE);
            }
        }
        
    }

    BLEBRR_UNLOCK_VOID();
}

/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param None
 *
 *  \return void
 */
void blebrr_pl_advertise_end (void)
{
    BLEBRR_LOCK_VOID();

    BLEBRR_SET_STATE(BLEBRR_STATE_IDLE);
    blebrr_advrepeat_count = 0;

    BLEBRR_UNLOCK_VOID();
}

#ifdef BLEBRR_FILTER_DUPLICATE_PACKETS
/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param p_adv_data_with_bd_addr
 *
 *  \return void
 */
DECL_STATIC API_RESULT blebrr_adv_duplicate_filter(/* IN */ UCHAR * p_adv_data_with_bd_addr)
{
    UCHAR length, index;

    /* Get the ADV data packet length */
    length = p_adv_data_with_bd_addr[1 + BT_BD_ADDR_SIZE];

    for (index = 0; index < BLEBRR_MAX_ADV_FILTER_LIST_COUNT; index++)
    {
        /* First Match BD Addr */
        if (0 == EM_mem_cmp
                 (
                     &blebrr_adv_list[index][0],
                     p_adv_data_with_bd_addr,
                     1 + BT_BD_ADDR_SIZE
                 ))
        {
            /* Check Data Length */
            if (blebrr_adv_list[index][1 + BT_BD_ADDR_SIZE] == p_adv_data_with_bd_addr[1 + BT_BD_ADDR_SIZE])
            {
                if (0 == EM_mem_cmp
                         (
                             &blebrr_adv_list[index][1 + BT_BD_ADDR_SIZE + 1],
                             &p_adv_data_with_bd_addr[1 + BT_BD_ADDR_SIZE + 1],
                             length
                         ))
                {
                    return API_SUCCESS;
                }
            }

            /* Update Adv data */
            EM_mem_copy
            (
                &blebrr_adv_list[index][1 + BT_BD_ADDR_SIZE],
                &p_adv_data_with_bd_addr[1 + BT_BD_ADDR_SIZE],
                length + 1
            );

            return API_FAILURE;
        }
    }

    /* Find out the suitable location to save the most recent ADV packet */
    /* New peer device. Add */
    EM_mem_copy
    (
        &blebrr_adv_list[blebrr_adv_list_inser_index][0],
        p_adv_data_with_bd_addr,
        length + 1 + BT_BD_ADDR_SIZE + 1
    );

    /* Increment and Wrap (if required) */
    blebrr_adv_list_inser_index++;

    if (BLEBRR_MAX_ADV_FILTER_LIST_COUNT <= blebrr_adv_list_inser_index)
    {
        blebrr_adv_list_inser_index = 0;
    }

    return API_FAILURE;
}
#endif /* BLEBRR_FILTER_DUPLICATE_PACKETS */


extern uint8  osal_memory_audit(void *ptr);
/**
 *  \brief
 *
 *  \par Description
 *
 *
 *  \param type
 *  \param pdata
 *  \param pdatalen
 *  \param rssi
 *
 *  \return void
 */
void blebrr_pl_recv_advpacket (UCHAR type, UCHAR * pdata, UINT16 pdatalen, UCHAR rssi)
{
    MS_BUFFER info;

#ifdef BLEBRR_FILTER_DUPLICATE_PACKETS
    /* Duplicate Filtering */
    retval = blebrr_adv_duplicate_filter(p_adv_data_with_bd_addr);

    /* If found the ADV packet as duplicate, drop the ADV packet */
    if (API_SUCCESS == retval)
    {
        return API_FAILURE;
    }
#endif /* BLEBRR_FILTER_DUPLICATE_PACKETS */

    /* Handle only if Non-Connectable (Passive) Advertising */
    if (BRR_BCON_PASSIVE != type)
    {
        return;
    }

    /* Pack the RSSI as metadata */
    info.payload = &rssi;
    info.length = sizeof(UCHAR);

    /* Deliver the packet to the bearer */
    if (NULL != blebrr_adv.bearer_recv)
    {
        /* BLEBRR_LOG("[ADV-Rx <]: ");
        BLEBRR_dump_bytes(pdata, pdatalen); */

        blebrr_adv.bearer_recv(&blebrr_advhandle, pdata, pdatalen, &info);
    }
    else
    {
        BLEBRR_LOG("BEARER RECV Callback Currently NULL !!\n");
    }
}

/**
 *  \brief
 *
 *  \par Description
 *
 * *
 * *  \param void
 *
 *  \return void
 */
void blebrr_register(void)
{
    /* Initialize locals */
    BLEBRR_MUTEX_INIT_VOID();

    /* Initialize Timer Handler */
    blebrr_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

    /* Initialize the transport */
    blebrr_init_pl();

    /* Get the platform Advdata initial offset if any */
    pl_advdata_offset = blebrr_get_advdata_offset_pl ();

    /* Reset the bearer sleep */
    blebrr_sleep = MS_FALSE;

    /* Add the Adv Bearer */
    blebrr_adv.bearer_send = blebrr_adv_send;

#ifdef BLEBRR_LP_SUPPORT
    blebrr_adv.bearer_sleep = blebrr_adv_sleep;
    blebrr_adv.bearer_wakeup = blebrr_adv_wakeup;
#endif /* BLEBRR_LP_SUPPORT */

    MS_brr_add_bearer(BRR_TYPE_ADV, &blebrr_adv, &blebrr_advhandle);

    /* Enable all Mesh related Module Debugging */
    EM_enable_module_debug_flag
    (
        MS_MODULE_ID_APP    |
        MS_MODULE_ID_ACCESS |
        MS_MODULE_ID_TRN    |
        MS_MODULE_ID_LTRN   |
        MS_MODULE_ID_NET    |
        MS_MODULE_ID_COMMON |
        MS_MODULE_ID_BRR    |
        MS_MODULE_ID_STBX   |
        MS_MODULE_ID_PROV
    );

    /* Set Debug Level Trace as default. Enable Information only if required */
    EM_set_debug_level(EM_DEBUG_LEVEL_TRC);

    /* Allow the tasks to start and be ready */
    EM_sleep (1);

#if 0
    /* Start Observing */
    BLEBRR_LOG ("Start Observing...\n");
    blebrr_scan_pl (MS_TRUE);

    /* Update state */
    BLEBRR_SET_STATE(BLEBRR_STATE_IN_SCAN_ENABLE);
#else /* 0 */
    BLEBRR_SET_STATE(BLEBRR_STATE_IDLE);
#endif /* 0 */
}

