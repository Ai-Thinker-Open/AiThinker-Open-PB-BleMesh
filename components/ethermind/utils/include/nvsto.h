
/**
 *  \file nvsto.h
 *
 *
 */

/*
 *  Copyright (C) 2013. Mindtree Limited.
 *  All rights reserved.
 */

#ifndef _H_NVSTO_
#define _H_NVSTO_

/* --------------------------------------------- Header File Inclusion */
#include "EM_os.h"
#include "nvs.h"

/* --------------------------------------------- Global Definitions */
/** Number of partitions per bank in platform */
#define NVSTO_NUM_PARTITIONS                5

/* --------------------------------------------- Structures/Data Types */
/** NVSTO Handle type */
typedef UINT8   NVSTO_HANDLE;

/* --------------------------------------------- Macros */
/** Persistent storage access wrappers */
#define nvsto_register_ps(size, handle) \
    nvsto_register(NVS_BANK_PERSISTENT, (size), (handle))

#define nvsto_open_pswrite(handle) \
    nvsto_open(NVS_BANK_PERSISTENT, (handle), NVS_ACCESS_WRITE)

#define nvsto_open_psread(handle) \
    nvsto_open(NVS_BANK_PERSISTENT, (handle), NVS_ACCESS_READ)

#define nvsto_close_ps(handle) \
    nvsto_close(NVS_BANK_PERSISTENT, (handle))

#define nvsto_write_ps(handle, buffer, length) \
    nvsto_write(NVS_BANK_PERSISTENT, (handle), (buffer), (length))

#define nvsto_read_ps(handle, buffer, length) \
    nvsto_read(NVS_BANK_PERSISTENT, (handle), (buffer), (length))

#define nvsto_seek_ps(handle, offset) \
    nvsto_seek(NVS_BANK_PERSISTENT, (handle), (offset))

/* --------------------------------------------- Internal Functions */

/* --------------------------------------------- API Declarations */
/**
 *  \brief
 *
 *  \Description
 *
 *
 *  \param void
 *
 *  \return void
 */
void nvsto_init (void);

/**
 *  \brief
 *
 *  \Description
 *
 *
 *  \param void
 *
 *  \return void
 */
void nvsto_shutdown (void);

/**
 *  \fn nvsto_register
 *
 *  \brief
 *
 *  \Description
 *
 *
 *  \param storage
 *  \param size
 *  \param handle
 *
 *  \return void
 */
INT8 nvsto_register
     (
         /* IN */  UINT8     storage,
         /* IN */  UINT16    size,
         /* OUT */ UINT8   * handle
     );

/**
 *  \fn nvsto_open
 *
 *  \brief
 *
 *  \Description
 *
 *
 *  \param storage
 *  \param handle
 *  \param access
 *
 *  \return void
 */
INT16 nvsto_open
      (
          /* IN */ UINT8    storage,
          /* IN */ UINT8    handle,
          /* IN */ UINT8    access
      );

/**
 *  \fn nvsto_close
 *
 *  \brief
 *
 *  \Description
 *
 *
 *  \param storage
 *  \param handle
 *
 *  \return void
 */
INT16 nvsto_close
      (
          /* IN */ UINT8    storage,
          /* IN */ UINT8    handle
      );

/**
 *  \fn nvsto_write
 *
 *  \brief
 *
 *  \Description
 *
 *
 *  \param storage
 *  \param handle
 *  \param buffer
 *  \param length
 *
 *  \return Number of bytes written
 */
INT16 nvsto_write
      (
          /* IN */ UINT8    storage,
          /* IN */ UINT8    handle,
          /* IN */ void   * buffer,
          /* IN */ UINT16   length
      );

/**
 *  \fn nvsto_read
 *
 *  \brief
 *
 *  \Description
 *
 *
 *  \param storage
 *  \param handle
 *  \param buffer
 *  \param length
 *
 *  \return Number of bytes read
 */
INT16 nvsto_read
      (
          /* IN */ UINT8    storage,
          /* IN */ UINT8    handle,
          /* IN */ void   * buffer,
          /* IN */ UINT16   length
      );

/**
 *  \fn nvsto_seek
 *
 *  \brief
 *
 *  \Description
 *
 *
 *  \param storage
 *  \param handle
 *  \param offset
 *
 *  \return void
 */
INT16 nvsto_seek
      (
          /* IN */ UINT8    storage,
          /* IN */ UINT8    handle,
          /* IN */ UINT32   offset
      );

#endif /* _H_NVSTO_ */

