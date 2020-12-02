
/**
 *  \file nvs.c
 *
 *
 */

/*
 *  Copyright (C) 2013. Mindtree Limited.
 *  All rights reserved.
 */

/* --------------------------------------------- Header File Inclusion */
#include "nvs.h"
#include "flash.h"
#include "hal_mcu.h"
#include "EM_platform.h"
#include "MS_net_api.h"
#include "access_internal.h"

#define NVS_ENABLE

#ifdef printf
#undef printf
#define printf(...)
#endif /* printf */

#define NVS_FLASH_BASE        0x05000
#define NVS_FLASH_SIZE        2288

extern NET_SEQ_NUMBER_STATE net_seq_number_state;


/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
#ifdef NVS_ENABLE
/** Flash Base */
DECL_STATIC UINT8 *nvs_base[NVS_NUM_BANKS];

/* NVS bank size */
DECL_STATIC UINT16 nvs_size[NVS_NUM_BANKS];

/* NVS bank current offset */
DECL_STATIC UINT16 nvs_offset[NVS_NUM_BANKS];

/* NVS Bank current state */
DECL_STATIC UCHAR nvs_state[NVS_NUM_BANKS];

/* NVS Signature */
DECL_STATIC UCHAR NVS_SIGNATURE[NVS_NUM_BANKS][NVS_SIGNATURE_SIZE] =
        { {'E', 'T', 'H', 'E', 'R', 'M', 'I', 'N', 'D', 'P', 'S'} };

/* Non-volatile write buffer for sector write */
__attribute((aligned))
DECL_STATIC UCHAR mem[NVS_FLASH_SIZE];
#endif /* NVS_ENABLE */
				
/* --------------------------------------------- Functions */
#ifdef NVS_ENABLE
static void NV_Erase (void)
{
    flash_sector_erase(NVS_FLASH_BASE);
}

static unsigned int NV_Read (unsigned int addr, uint8_t * buffer, unsigned int size)
{
    UINT32 i;
	
    for (i = 0; i < size; i ++)
    {
        *(buffer + i) = ReadFlash ((unsigned int)(addr + i));
    }
    
    return i;
}

static unsigned int NV_Write
                    (
                        unsigned int start_addr,
                        unsigned int length,
                        unsigned int* data
                    )
{
    unsigned int ret;
    uint32_t i,j;

    //printf("[NV_Write]MS_PS_RECORD_CORE_MODULES_OFFSET = 0x%08X\n",MS_PS_RECORD_CORE_MODULES_OFFSET);
    j = (MS_PS_RECORD_CORE_MODULES_OFFSET & 0x03) ? 1 : 0;
    j += (MS_PS_RECORD_CORE_MODULES_OFFSET >> 2);

    /* Read to local static */
    NV_Read((unsigned int)(nvs_base[0]), mem, (j+2)<<2);

    /* Update local static */
    memcpy(&mem[nvs_offset[0]], data, length);

    /* Erase the flash */
    NV_Erase();
    

    for (i = 0; i < (j+2); i++)
    {
        //HAL_DISABLE_INTERRUPTS();
        
        ret = WriteFlash
              (
                  (((unsigned int)start_addr) + (i * sizeof(uint32_t))),
                  (uint32_t)(*(((uint32_t *)(mem)) + i))
              );
        //HAL_ENABLE_INTERRUPTS();

        if (0 == ret)
        {
            printf ("WriteFlash Failed! - %d", i);
            break;
        }
    }
		
    return !ret;
}
#endif /* NVS_ENABLE */

UINT16 nvs_init (UINT8 bank)
{
#ifdef NVS_ENABLE
    /* Assign the base */
    nvs_base[bank] = (UINT8 *)NVS_FLASH_BASE;

    /* Get the maximum allowed size limit */
    nvs_size[bank] = (UINT16)NVS_FLASH_SIZE;

    printf ("Initializing Storage... %d bytes\r\n", nvs_size[bank]);

    /* Set initial state to closed */
    nvs_state[bank] = NVS_CLOSE;

    return nvs_size[bank];
#else /* NVS_ENABLE */

    return 0;
    
#endif    
}

void nvs_shutdown (UINT8 bank)
{
}

void nvs_reset (UINT8 bank)
{
#ifdef NVS_ENABLE
    NV_Erase();
    nvs_offset[bank] = 0;
#endif /* NVS_ENABLE */
}

INT8 nvs_open (UINT8 bank, UINT8 mode, UINT16 offset)
{
    UINT32 ret;
#ifdef NVS_ENABLE
    UCHAR sign[NVS_SIGNATURE_SIZE];

    /* Check if state is closed. Only then open */
    if (NVS_CLOSE != nvs_state[bank])
    {
        return (INT8)-1;
    }

    ret = (INT8)-1;

    /* Initialize access offset */
    nvs_offset[bank] = 0;

    /* Read the signature from the partition start */
    NV_Read((unsigned int)(nvs_base[bank]), sign, sizeof(sign));

    /* Check for validity of signature */
    if (0 != EM_mem_cmp (sign, NVS_SIGNATURE[bank], sizeof (sign)))
    {
        printf ("Signature Mismatch");
        if (NVS_ACCESS_READ == mode)
        {
            net_seq_number_state.seq_num = 0;
            net_seq_number_state.block_seq_num_max = MS_CONFIG_LIMITS(MS_NET_SEQ_NUMBER_BLOCK_SIZE);
            /* Signature mismatch for read. Return failure */
            return (INT8)-1;
        }
        else
        {
            printf ("Write Signature");

            /* Signature mismatch for write. Write the Signature */
            ret = NV_Write
                  (
                      (unsigned int)nvs_base[bank],
                      NVS_SIGNATURE_SIZE,
                      (unsigned int *)&(NVS_SIGNATURE[bank][0])
                  );
        }
    }
    else
    {
        ret = 0;
    }

    /* Update the access offset */
    nvs_offset[bank] += NVS_SIGNATURE_SIZE;
    nvs_offset[bank] += offset;

    /* Set persistant state to open */
    nvs_state[bank] = (NVS_ACCESS_READ == mode)? NVS_RDOPEN: NVS_WROPEN;

    printf ("Storage opened for %s\r\n", (NVS_RDOPEN == nvs_state[bank])? "Reading": "Writing");

#endif /* NVS_ENABLE */
    return ret;
}

INT8 nvs_close (UINT8 bank)
{
#ifdef NVS_ENABLE
    /* Seek to the start of the bank */
    nvs_seek(bank, 0);

    nvs_state[bank] = NVS_CLOSE;

    printf ("Storage Closed");
#endif /* NVS_ENABLE */
    return 0;
}

INT16 nvs_write (UINT8 bank, void * buffer, UINT16 size)
{
#ifdef NVS_ENABLE
//    UINT16 ret;

    if (NVS_WROPEN != nvs_state[bank])
    {
        return -1;
    }

    /* Write to flash */
    NV_Write
          (
              (unsigned int)nvs_base[bank],
              size,
              buffer
          );

    /* Update access offset */
    nvs_offset[bank] += size;

    printf ("Written %d bytes. Offset at %d", size, nvs_offset[bank]);
#endif /* NVS_ENABLE */
    return size;
}

INT16 nvs_read (UINT8 bank, void * buffer, UINT16 size)
{
#ifdef NVS_ENABLE
    if (NVS_RDOPEN != nvs_state[bank])
    {
        return -1;
    }

    /* Read from flash */
    NV_Read((unsigned int)((unsigned int)(nvs_base[bank]) + nvs_offset[bank]), buffer, size);

    /* Update access offset */
    nvs_offset[bank] += size;

    printf ("Read %d bytes. Offset at %d", size, nvs_offset[bank]);
#endif /* NVS_ENABLE */
    return size;
}

INT16 nvs_seek (UINT8 bank, UINT32 offset)
{
#ifdef NVS_ENABLE
    /* Check if flash is not closed */
    if (NVS_CLOSE == nvs_state[bank])
    {
        return -1;
    }

    /* Update the access offset */
    nvs_offset[bank] = (sizeof(NVS_SIGNATURE[bank]) + offset);

    printf ("Seek %d offset. Offset at %d", offset, nvs_offset[bank]);
#endif /* NVS_ENABLE */

    return offset;
}

#include "nvsto.h"

void nvs_test (void)
{
    NVSTO_HANDLE shdl;
    volatile INT32 nbytes;
    char wtest[] = "EtherMind Mesh Storage Test";
    char rtest[sizeof(wtest)];

//    UCHAR test[8] = {0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef};
//    static UCHAR tt[8] = {0};

    printf ("Storage Testing...\r\n");

    printf ("Initializing Storage...\r\n")
    nvsto_init();
    nvsto_register_ps(sizeof(wtest), &shdl);

    printf ("Opening Storage for Read...\r\n")
    nbytes = nvsto_open_psread(shdl);
    printf ("Retval - 0x%04X\r\n", nbytes);

    if (0 == nbytes)
    {
        printf ("Read...\r\n")
        nbytes = nvsto_read_ps(shdl, rtest, sizeof(rtest));
        printf ("Retval - 0x%04X\r\n", nbytes);

        if (0 < nbytes)
        {
            printf ("Read from Flash - %s\r\n", rtest);
        }

        printf ("Closing Storage...\r\n");
        nvsto_close_ps (shdl);
    }
    else
    {
        printf ("Opening Storage for Write...\r\n")
        nbytes = nvsto_open_pswrite(shdl);
        printf ("Retval - 0x%04X\r\n", nbytes);

        if (0 == nbytes)
        {
            printf ("Write...\r\n")
            nbytes = nvsto_write_ps(shdl, wtest, sizeof(wtest));
            printf ("Retval - 0x%04X\r\n", nbytes);

            if (0 < nbytes)
            {
                printf ("Write to Flash - %s\r\n", wtest);
            }

            printf ("Closing Storage...\r\n");
            nvsto_close_ps (shdl);
        }
    }
}

