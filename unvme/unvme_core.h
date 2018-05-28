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
 * @brief uNVMe core header file.
 */

#ifndef _UNVME_CORE_H
#define _UNVME_CORE_H

#include <sys/types.h>

#include "unvme.h"
#include "unvme_log.h"
#include "unvme_mem.h"
#include "unvme_nvme.h"

/// Doubly linked list add node
#define LIST_ADD(head, node)                                    \
            if ((head) != NULL) {                               \
                (node)->next = (head);                          \
                (node)->prev = (head)->prev;                    \
                (head)->prev->next = (node);                    \
                (head)->prev = (node);                          \
            } else {                                            \
                (node)->next = (node)->prev = (node);           \
                (head) = (node);                                \
            }

/// Doubly linked list remove node
#define LIST_DEL(head, node)                                    \
            if ((node->next) != (node)) {                       \
                (node)->next->prev = (node)->prev;              \
                (node)->prev->next = (node)->next;              \
                if ((head) == (node)) (head) = (node)->next;    \
            } else {                                            \
                (head) = NULL;                                  \
            }

/// Log and print an unrecoverable error message and exit
#define FATAL(fmt, arg...)  do { ERROR(fmt, ##arg); abort(); } while (0)

/// Page size
typedef char unvme_page_t[4096];

/// IO memory allocation tracking info
typedef struct _unvme_iomem {
    mem_dma_t**            map;        ///< dynamic array of allocated memory
    int                     size;       ///< array size
    int                     count;      ///< array count
} unvme_iomem_t;

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
    u64                     cidmask[];  ///< cid pending bit mask
} unvme_desc_t;

/// IO queue entry
typedef struct _unvme_queue {
    nvme_queue_t*           nvmeq;      ///< NVMe associated queue
    mem_dma_t*             sqdma;      ///< submission queue mem
    mem_dma_t*             cqdma;      ///< completion queue mem
    mem_dma_t*             prplist;    ///< PRP list
    u32                     size;       ///< queue depth
    u16                     cid;        ///< next cid to check and use
    int                     cidcount;   ///< number of pending cids
    int                     desccount;  ///< number of pending descriptors
    int                     masksize;   ///< bit mask size to allocate
    u64*                    cidmask;    ///< cid pending bit mask
    unvme_desc_t*           desclist;   ///< used descriptor list
    unvme_desc_t*           descfree;   ///< free descriptor list
    unvme_desc_t*           descpend;   ///< pending descriptor list
} unvme_queue_t;

/// Device context
typedef struct _unvme_device {
    mem_device_t            memdev;     ///< memory device
    nvme_device_t           nvmedev;    ///< NVMe device
    unvme_queue_t           adminq;     ///< adminq queue
    int                     refcount;   ///< reference count
    unvme_iomem_t           iomem;      ///< IO memory tracker
    unvme_ns_t              ns;         ///< controller namespace (id=0)
    unvme_queue_t*          ioqs;       ///< pointer to IO queues
} unvme_device_t;

/// Session context
typedef struct _unvme_session {
    struct _unvme_session*  prev;       ///< previous session node
    struct _unvme_session*  next;       ///< next session node
    unvme_device_t*         dev;        ///< device context
    unvme_ns_t              ns;         ///< namespace
} unvme_session_t;

unvme_ns_t* unvme_do_open(int pci, int nsid, int qcount, int qsize);
int unvme_do_close(const unvme_ns_t* ns);
void* unvme_do_alloc(const unvme_ns_t* ns, u64 size);
int unvme_do_free(const unvme_ns_t* ses, void* buf);
int unvme_do_poll(unvme_desc_t* desc, int sec, u32* cqe_cs);
unvme_desc_t* unvme_do_cmd(const unvme_ns_t* ns, int qid, int opc, int nsid, void* buf, u64 bufsz, u32 cdw10_15[6]);
unvme_desc_t* unvme_do_rw(const unvme_ns_t* ns, int qid, int opc, void* buf, u64 slba, u32 nlb);

#endif  // _UNVME_CORE_H

