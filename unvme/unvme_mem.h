/**
 * Copyright (c) 2015-2016, Micron Technology, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * @brief Memory function support header files.
 */

#ifndef _UNVME_MEM_H
#define _UNVME_MEM_H

#include <stdlib.h>
#include <pthread.h>
#include "xil_types.h"
#include "../params.h"

typedef s8 __s8;
typedef s16 __s16;
typedef s32 __s32;
typedef s64 __s64;
typedef u8 __u8;
typedef u16 __u16;
typedef u32 __u32;
typedef u64 __u64;


/// memory allocation entry
typedef struct _mem {
	int static_mem;
	int valid;
    struct _mem_device*    dev;        ///< device owner
    int                     mmap;       ///< mmap indication flag
    size_t                  size;       ///< size
    void*                   dma_buf;        ///< memory buffer
    size_t                  dma_size;       ///< allocated size
    __u64                   dma_addr;       ///< I/O DMA address
    struct _mem*       prev;       ///< previous entry
    struct _mem*       next;       ///< next entry
} mem_t;

/// device structure
typedef struct _mem_device {
    int                     pagesize;   ///< system page size
    __u64                   iovabase;   ///< IO virtual address base
    __u64                   iovanext;   ///< next IO virtual address to use
    __u64                   iovamask;   ///< max IO virtual address mask
    mem_t*                  memlist;    ///< memory allocated list
    void*                   membuf;     ///< Memory buffer pointer
    off_t                   memoff;     ///< Memory buffer offset
    size_t                  memsize;    ///< Memory buffer size
} mem_device_t;

// Export functions
int mem_create(mem_device_t* dev);
void mem_delete(mem_device_t* dev);
mem_t* mem_alloc(mem_device_t* dev, mem_t* mem, size_t size, int clear);
int mem_free(mem_t* mem);

#endif // _UNVME_MEM_H

