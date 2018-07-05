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
 * @brief UNVMe client header file.
 */

#ifndef _UNVME_H
#define _UNVME_H

#include <stdint.h>
#include "unvme_mem.h"
#include "unvme_nvme.h"
#include "../params.h"

#ifndef _U_TYPE
#define _U_TYPE                     ///< bit size data types
typedef int8_t          s8;         ///< 8-bit signed
typedef int16_t         s16;        ///< 16-bit signed
typedef int32_t         s32;        ///< 32-bit signed
typedef int64_t         s64;        ///< 64-bit signed
typedef uint8_t         u8;         ///< 8-bit unsigned
typedef uint16_t        u16;        ///< 16-bit unsigned
typedef uint32_t        u32;        ///< 32-bit unsigned
typedef uint64_t        u64;        ///< 64-bit unsigned
#endif // _U_TYPE

/// Namespace attributes structure
typedef struct _unvme_ns {
    u16                 id;         ///< namespace id
    u16                 vid;        ///< vendor id
    char                mn[40];     ///< model number
    char                sn[20];     ///< serial number
    char                fr[8];      ///< firmware revision
    u64                 blockcount; ///< total number of available blocks
    u64                 pagecount;  ///< total number of available pages
    u16                 blocksize;  ///< logical block size
    u16                 pagesize;   ///< page size
    u16                 blockshift; ///< block size shift value
    u16                 pageshift;  ///< page size shift value
    u16                 bpshift;    ///< block to page shift
    u16                 nbpp;       ///< number of blocks per page
    u16                 maxbpio;    ///< max number of blocks per I/O
    u16                 maxppio;    ///< max number of pages per I/O
    u16                 maxiopq;    ///< max number of I/O submissions per queue
    u16                 nscount;    ///< number of namespaces available
    u32                 qcount;     ///< number of I/O queues
    u32                 maxqcount;  ///< max number of queues supported
    u32                 qsize;      ///< I/O queue size
    u32                 maxqsize;   ///< max queue size supported
//    unvme_session_t     ses;        ///< associated session
} unvme_ns_t;

/// I/O descriptor (not to be copied and is cleared upon apoll completion)
typedef struct _unvme_iod {
    void*               buf;        ///< data buffer (as submitted)
    u64                 slba;       ///< starting lba (as submitted)
    u32                 nlb;        ///< number of blocks (as submitted)
    u32                 qid;        ///< queue id (as submitted)
    u32                 opc;        ///< op code
    u32                 id;         ///< descriptor id
} *unvme_iod_t;

/// IO full descriptor
typedef struct _unvme_desc {
    void*                   buf;        ///< buffer
    u64                     slba;       ///< starting lba
    u32                     nlb;        ///< number of blocks
    u32                     qid;        ///< queue id
    u32                     opc;        ///< op code
    u32                     id;         ///< descriptor id
    void*                   sentinel;   ///< sentinel check
    struct _unvme_queue*    q;          ///< queue context owner
    struct _unvme_desc*     prev;       ///< previous descriptor node
    struct _unvme_desc*     next;       ///< next descriptor node
    int                     error;      ///< error status
    int                     cidcount;   ///< number of pending cids
    u64                     cidmask[UNVME_QSIZE/8];  ///< cid pending bit mask
} unvme_desc_t;

/// IO queue entry
typedef struct _unvme_queue {
	int                     isadmin;
	nvme_queue_t*           adminq;
    nvme_queue_t            nvmeq;      ///< NVMe associated queue
    mem_t             sqdma;      ///< submission queue mem
    mem_t             cqdma;      ///< completion queue mem
    mem_t             prplist;    ///< PRP list
    u32                     size;       ///< queue depth
    u16                     cid;        ///< next cid to check and use
    int                     cidcount;   ///< number of pending cids
    int                     desccount;  ///< number of pending descriptors
    int                     masksize;   ///< bit mask size to allocate
    u64                     cidmask[UNVME_QSIZE/8];    ///< cid pending bit mask
    unvme_desc_t*           desclist;   ///< used descriptor list
    unvme_desc_t*           descfree;   ///< free descriptor list
    unvme_desc_t*           descpend;   ///< pending descriptor list
    unvme_desc_t            descs[UNVME_DESCS];
} unvme_queue_t;

/// Device context
typedef struct _unvme_device {
    mem_device_t            memdev;     ///< memory device
    nvme_device_t           nvmedev;    ///< NVMe device
    unvme_queue_t           adminq;     ///< adminq queue
    int                     refcount;   ///< reference count
    mem_t                   iomem[UNVME_IOMEMS]; ///< array of allocated memory
    int                     iomem_count;      ///< array count
    unvme_ns_t              nscnt;      ///< controller namespace (id=0)
    unvme_ns_t              nsio;       ///< io namespace (id=<0)
    unvme_queue_t           ioqs[UNVME_QCOUNT]; ///< IO queues
} unvme_device_t;

// Export functions
int unvme_open(unvme_device_t* dev);
int unvme_openq(unvme_device_t* dev);
int unvme_close(unvme_device_t* dev);

void* unvme_alloc(unvme_device_t* dev, u64 size);
int unvme_free(unvme_device_t* dev, void* buf);

int unvme_write(unvme_device_t* dev, int qid, const void* buf, u64 slba, u32 nlb);
int unvme_read(unvme_device_t* dev, int qid, void* buf, u64 slba, u32 nlb);
int unvme_cmd(unvme_device_t* dev, int qid, int opc, int nsid, void* buf, u64 bufsz, u32 cdw10_15[6], u32* cqe_cs);

unvme_iod_t unvme_awrite(unvme_device_t* dev, int qid, const void* buf, u64 slba, u32 nlb);
unvme_iod_t unvme_aread(unvme_device_t* dev, int qid, void* buf, u64 slba, u32 nlb);
unvme_iod_t unvme_acmd(unvme_device_t* dev, int qid, int opc, int nsid, void* buf, u64 bufsz, u32 cdw10_15[6]);

int unvme_apoll(unvme_iod_t iod, int timeout);
int unvme_apoll_cs(unvme_iod_t iod, int timeout, u32* cqe_cs);

#endif // _UNVME_H

