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
 * @brief UNVMe client library interface functions.
 */

#include "unvme_core.h"

/**
 * Open a client session with specified number of IO queues and queue size.
 * @param   pci         PCI device (as 0xbbddff format)
 * @param   nsid        namespace ID
 * @param   qcount      number of io queues
 * @param   qsize       io queue size
 * @return  namespace pointer or NULL if error.
 */
const unvme_ns_t* unvme_openq(int pci, int nsid, int qcount, int qsize)
{
    if (qcount < 0 || qsize < 0 || qsize == 1) {
        ERROR("invalid qcount %d or qsize %d", qcount, qsize);
        return NULL;
    }

    return unvme_do_open(pci, nsid, qcount, qsize);
}

/**
 * Open a client session.
 * @param   pciname     PCI device name (as %x:%x.%x[/NSID] format)
 * @return  namespace pointer or NULL if error.
 */
const unvme_ns_t* unvme_open(int pci, int nsid)
{
    return unvme_openq(pci, nsid, 0, 0);
}

/**
 * Close a client session and delete its contained io queues.
 * @param   ns          namespace handle
 * @return  0 if ok else error code.
 */
int unvme_close(const unvme_ns_t* ns)
{
    return unvme_do_close(ns);
}

/**
 * Allocate an I/O buffer associated with a session.
 * @param   ns          namespace handle
 * @param   size        buffer size
 * @return  the allocated buffer or NULL if failure.
 */
void* unvme_alloc(const unvme_ns_t* ns, u64 size)
{
    return unvme_do_alloc(ns, size);
}

/**
 * Free an I/O buffer associated with a session.
 * @param   ns          namespace handle
 * @param   buf         buffer pointer
 * @return  0 if ok else -1.
 */
int unvme_free(const unvme_ns_t* ns, void* buf)
{
    return unvme_do_free(ns, buf);
}

/**
 * Submit a generic or vendor specific command.
 * @param   ns          namespace handle
 * @param   qid         client queue index (-1 for admin queue)
 * @param   opc         command op code
 * @param   nsid        namespace id
 * @param   buf         data buffer (from unvme_alloc)
 * @param   bufsz       data buffer size
 * @param   cdw10_15    NVMe command word 10 through 15
 * @return  descriptor or NULL if failed.
 */
inline unvme_iod_t unvme_acmd(const unvme_ns_t* ns, int qid, int opc, int nsid,
                              void* buf, u64 bufsz, u32 cdw10_15[6])
{
    return (unvme_iod_t)unvme_do_cmd(ns, qid, opc, nsid, buf, bufsz, cdw10_15);
}

/**
 * Read data from specified logical blocks on device.
 * @param   ns          namespace handle
 * @param   qid         client queue index
 * @param   buf         data buffer (from unvme_alloc)
 * @param   slba        starting logical block
 * @param   nlb         number of logical blocks
 * @return  I/O descriptor or NULL if failed.
 */
inline unvme_iod_t unvme_aread(const unvme_ns_t* ns, int qid, void* buf, u64 slba, u32 nlb)
{
    return (unvme_iod_t)unvme_do_rw(ns, qid, NVME_CMD_READ, buf, slba, nlb);
}

/**
 * Write data to specified logical blocks on device.
 * @param   ns          namespace handle
 * @param   qid         client queue index
 * @param   buf         data buffer (from unvme_alloc)
 * @param   slba        starting logical block
 * @param   nlb         number of logical blocks
 * @return  I/O descriptor or NULL if failed.
 */
inline unvme_iod_t unvme_awrite(const unvme_ns_t* ns, int qid,
                         const void* buf, u64 slba, u32 nlb)
{
    return (unvme_iod_t)unvme_do_rw(ns, qid, NVME_CMD_WRITE, (void*)buf, slba, nlb);
}

/**
 * Poll for completion status of a previous IO submission.
 * If there's no error, the descriptor will be freed.
 * @param   iod         IO descriptor
 * @param   timeout     in seconds
 * @return  0 if ok else error status (-1 for timeout).
 */
inline int unvme_apoll(unvme_iod_t iod, int timeout)
{
    return unvme_do_poll((unvme_desc_t*)iod, timeout, NULL);
}

/**
 * Poll for completion status of a previous IO submission.
 * If there's no error, the descriptor will be freed.
 * @param   iod         IO descriptor
 * @param   timeout     in seconds
 * @param   cqe_cs      CQE command specific DW0 returned
 * @return  0 if ok else error status (-1 for timeout).
 */
inline int unvme_apoll_cs(unvme_iod_t iod, int timeout, u32* cqe_cs)
{
    return unvme_do_poll((unvme_desc_t*)iod, timeout, cqe_cs);
}

/**
 * Submit a generic or vendor specific command and then poll for completion.
 * @param   ns          namespace handle
 * @param   qid         client queue index (-1 for admin queue)
 * @param   opc         command op code
 * @param   nsid        namespace id
 * @param   buf         data buffer (from unvme_alloc)
 * @param   bufsz       data buffer size
 * @param   cdw10_15    NVMe command word 10 through 15
 * @param   cqe_cs      CQE command specific DW0 returned
 * @return  descriptor or NULL if failed.
 */
int unvme_cmd(const unvme_ns_t* ns, int qid, int opc, int nsid,
              void* buf, u64 bufsz, u32 cdw10_15[6], u32* cqe_cs)
{
    unvme_iod_t iod = unvme_acmd(ns, qid, opc, nsid, buf, bufsz, cdw10_15);
    if (iod) {
//        sched_yield();
        return unvme_apoll_cs(iod, UNVME_TIMEOUT, cqe_cs);
    }
    return -1;
}

/**
 * Read data from specified logical blocks on device.
 * @param   ns          namespace handle
 * @param   qid         client queue index
 * @param   buf         data buffer (from unvme_alloc)
 * @param   slba        starting logical block
 * @param   nlb         number of logical blocks
 * @return  0 if ok else error status.
 */
int unvme_read(const unvme_ns_t* ns, int qid, void* buf, u64 slba, u32 nlb)
{
    unvme_iod_t iod = unvme_aread(ns, qid, buf, slba, nlb);
    if (iod) {
//        sched_yield();
        return unvme_apoll(iod, UNVME_TIMEOUT);
    }
    return -1;
}

/**
 * Write data to specified logical blocks on device.
 * @param   ns          namespace handle
 * @param   qid         client queue index
 * @param   buf         data buffer (from unvme_alloc)
 * @param   slba        starting logical block
 * @param   nlb         number of logical blocks
 * @return  0 if ok else error status.
 */
int unvme_write(const unvme_ns_t* ns, int qid,
                const void* buf, u64 slba, u32 nlb)
{
    unvme_iod_t iod = unvme_awrite(ns, qid, buf, slba, nlb);
    if (iod) {
//        sched_yield();
        return unvme_apoll(iod, UNVME_TIMEOUT);
    }
    return -1;
}

