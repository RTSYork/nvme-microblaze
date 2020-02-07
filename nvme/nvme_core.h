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

#ifndef _NVME_CORE_H
#define _NVME_CORE_H

#include <sys/types.h>

#include "nvme.h"
#include "nvme_log.h"
#include "nvme_mem.h"
#include "nvme_nvme.h"

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


int unvme_do_open(unvme_device_t* dev);
int unvme_do_close(unvme_device_t* dev);
void* unvme_do_alloc(unvme_device_t* dev, u64 size);
int unvme_do_free(unvme_device_t* dev, void* buf);
int unvme_do_poll(unvme_desc_t* desc, int sec, u32* cqe_cs);
unvme_desc_t* unvme_do_cmd(unvme_device_t* dev, int qid, int opc, int nsid, void* buf, u64 bufsz, u32 cdw10_15[6]);
unvme_desc_t* unvme_do_rw(unvme_device_t* dev, int qid, int opc, void* buf, u64 slba, u32 nlb);

#endif  // _NVME_CORE_H

