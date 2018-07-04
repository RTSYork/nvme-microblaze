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
 * @brief UNVMe driver module.
 */

#include "unvme_core.h"

#include <string.h>
#include <signal.h>
#include <sched.h>
#include <fcntl.h>

#include "rdtsc.h"

/// IO descriptor debug print
#define PDEBUG(fmt, arg...) //fprintf(stderr, fmt "\n", ##arg)

int descs = 0;
int descs_max = 0;

/**
 * Get a descriptor entry by moving from the free to the use list.
 * @param   q       queue
 * @return  the descriptor added to the use list.
 */
static unvme_desc_t* unvme_desc_get(unvme_queue_t* q)
{
    static u32 id = 0;
    unvme_desc_t* desc;

    if (q->descfree) {
        desc = q->descfree;
        LIST_DEL(q->descfree, desc);

        desc->error = 0;
        desc->cidcount = 0;
        u64* cidmask = desc->cidmask;
        int i = q->masksize >> 3;
        while (i--) *cidmask++ = 0;
    } else {
        desc = zalloc(sizeof(unvme_desc_t) + q->masksize);
        if (descs_max < ++descs) descs_max = descs;
        INFO_FN("descs: %d / %d", descs, descs_max);
        desc->id = ++id;
        desc->q = q;
    }
    LIST_ADD(q->desclist, desc);
    if (desc == desc->next) q->descpend = desc; // head of pending list
    q->desccount++;
    return desc;
}

/**
 * Put a descriptor entry back by moving it from the use to the free list.
 * @param   desc    descriptor
 */
static void unvme_desc_put(unvme_desc_t* desc)
{
    unvme_queue_t* q = desc->q;

    // check to change the pending head or clear the list
    if (desc == q->descpend) {
        if (desc != desc->next) q->descpend = desc->next;
        else q->descpend = NULL;
    }

    LIST_DEL(q->desclist, desc);
    LIST_ADD(q->descfree, desc);
    q->desccount--;
}

/**
 * Process an I/O completion.
 * @param   q           queue
 * @param   timeout     timeout in seconds
 * @param   cqe_cs      CQE command specific DW0 returned
 * @return  0 if ok else NVMe error code (-1 means timeout).
 */
static int unvme_check_completion(unvme_queue_t* q, int timeout, u32* cqe_cs)
{
    // wait for completion
    int err, cid;
    u64 endtsc = 0;
    nvme_queue_t *nvmeq = (q->isadmin) ? q->adminq : &q->nvmeq;
    do {
        cid = nvme_check_completion(nvmeq, &err, cqe_cs);
        if (timeout == 0 || cid >= 0) break;
        else endtsc = rdtsc() + (u64)timeout * nvmeq->dev->rdtsec;
    } while (rdtsc() < endtsc);

    if (cid < 0) return cid;

    // find the pending cid in the descriptor list to clear it
    unvme_desc_t* desc = q->descpend;
    int b = cid >> 6;
    u64 mask = (u64)1 << (cid & 63);
    while ((desc->cidmask[b] & mask) == 0) {
        desc = desc->next;
        if (desc == q->descpend)
            FATAL("pending cid %d not found", cid);
    }
    if (err) desc->error = err;

    // clear cid bit used
    desc->cidmask[b] &= ~mask;
    desc->cidcount--;
    q->cidmask[b] &= ~mask;
    q->cidcount--;
    q->cid = cid;

    // check to advance next pending descriptor
    if (q->cidcount) {
        while (q->descpend->cidcount == 0) q->descpend = q->descpend->next;
    }
    PDEBUG("# c q%d={%d %d %#lx} d={%d %d %#lx} @%d",
           q->nvmeq->id, cid, q->cidcount, *q->cidmask,
           desc->id, desc->cidcount, *desc->cidmask, q->descpend->id);
    return err;
}

/**
 * Get a free cid.  If queue is full then process currently pending submissions.
 * @param   desc        descriptor
 * @return  cid.
 */
static u16 unvme_get_cid(unvme_desc_t* desc)
{
    u16 cid;
    unvme_queue_t* q = desc->q;
    int qsize = q->size;

    // if submission queue is full then process completion first
    if ((q->cidcount + 1) == qsize) {
        int err = unvme_check_completion(q, UNVME_TIMEOUT, NULL);
        if (err) {
            if (err == -1) FATAL("q%d timeout", q->nvmeq.id);
            else ERROR("q%d error %#x", q->nvmeq.id, err);
        }
    }

    // get a free cid
    cid = q->cid;
    while (q->cidmask[cid >> 6] & ((u64)1L << (cid & 63))) {
        if (++cid >= qsize) cid = 0;
    }

    // set cid bit used
    int b = cid >> 6;
    u64 mask = (u64)1 << (cid & 63);
    desc->cidmask[b] |= mask;
    desc->cidcount++;
    q->cidmask[b] |= mask;
    q->cidcount++;
    q->cid = cid;
    if (++q->cid >= qsize) q->cid = 0;

    return cid;
}

/**
 * Lookup DMA address associated with the user buffer.
 * @param   ns          namespace handle
 * @param   buf         user data buffer
 * @param   bufsz       buffer size
 * @return  DMA address or -1L if error.
 */
static u64 unvme_map_dma(const unvme_device_t* dev, void* buf, u64 bufsz)
{
    mem_t* mem = NULL;
    int i;
    for (i = 0; i < dev->iomem.count; i++) {
        mem = dev->iomem.map[i];
        if (mem->dma_buf <= buf && buf < (mem->dma_buf + mem->dma_size)) break;
    }
    if (i == dev->iomem.count)
        FATAL("invalid I/O buffer address");
    u64 addr = mem->dma_addr + (u64)(buf - mem->dma_buf);
    if ((addr + bufsz) > (mem->dma_addr + mem->dma_size))
        FATAL("buffer overrun");
    return addr;
}

/**
 * Map the user buffer to PRP addresses (compose PRP list as necessary).
 * @param   ns          namespace handle
 * @param   q           queue
 * @param   cid         queue entry index
 * @param   buf         user buffer
 * @param   bufsz       buffer size
 * @param   prp1        returned prp1 value
 * @param   prp2        returned prp2 value
 * @return  0 if ok else -1 if buffer address error.
 */
static int unvme_map_prps(const unvme_device_t* dev, unvme_queue_t* q, int cid,
                          void* buf, u64 bufsz, u64* prp1, u64* prp2)
{
    u64 addr = unvme_map_dma(dev, buf, bufsz);
    if (addr == -1L) return -1;

    *prp1 = addr;
    *prp2 = 0;
    int numpages = (bufsz + dev->nsio.pagesize - 1) >> dev->nsio.pageshift;
    if (numpages == 2) {
        *prp2 = addr + dev->nsio.pagesize;
    } else if (numpages > 2) {
        int prpoff = cid << dev->nsio.pageshift;
        u64* prplist = q->prplist.dma_buf + prpoff;
        *prp2 = q->prplist.dma_addr + prpoff;
        int i;
        for (i = 1; i < numpages; i++) {
            addr += dev->nsio.pagesize;
            *prplist++ = addr;
        }
    }
    return 0;
}

/**
 * Submit a generic (vendor specific) NVMe command.
 * @param   ns          namespace handle
 * @param   desc        descriptor
 * @param   buf         data buffer
 * @param   slba        starting lba
 * @param   nlb         number of logical blocks
 * @return  cid if ok else -1.
 */
static int unvme_submit_io(const unvme_device_t* dev, unvme_desc_t* desc,
                           void* buf, u64 slba, u32 nlb)
{
    u64 prp1, prp2;
    unvme_queue_t* ioq = desc->q;
    u16 cid = unvme_get_cid(desc);
    u64 bufsz = (u64)nlb << dev->nsio.blockshift;
    if (unvme_map_prps(dev, ioq, cid, buf, bufsz, &prp1, &prp2)) return -1;

    // submit I/O command
    if (nvme_cmd_rw(&ioq->nvmeq, desc->opc, cid,
                    dev->nsio.id, slba, nlb, prp1, prp2)) return -1;
    PDEBUG("# %c %#lx %#x q%d={%d %d %#lx} d={%d %d %#lx}",
           desc->opc == NVME_CMD_READ ? 'r' : 'w', slba, nlb,
           ioq->nvmeq->id, cid, ioq->cidcount, *ioq->cidmask,
           desc->id, desc->cidcount, *desc->cidmask);
    return cid;
}

/**
 * Initialize a queue allocating descriptors and PRP list pages.
 * @param   dev         device context
 * @param   q           queue
 * @param   qsize       queue depth
 */
static void unvme_queue_init(unvme_device_t* dev, unvme_queue_t* q, int qsize)
{
    memset(q, 0, sizeof(*q));
    q->size = qsize;

    // allocate queue entries and PRP list
    mem_alloc(&dev->memdev, &q->sqdma, qsize * sizeof(nvme_sq_entry_t), 1);
    mem_alloc(&dev->memdev, &q->cqdma, qsize * sizeof(nvme_cq_entry_t), 1);
    mem_alloc(&dev->memdev, &q->prplist, qsize << dev->nscnt.pageshift, 1);
    if (!q->sqdma.valid || !q->cqdma.valid || !q->prplist.valid)
        FATAL("vfio_dma_alloc");

    // setup descriptors and pending masks
    q->masksize = ((qsize + 63) >> 6) << 3; // (qsize + 63) / 64) * sizeof(u64)
    int i;
    for (i = 0; i < 16; i++) unvme_desc_get(q);
    q->descfree = q->desclist;
    q->desclist = NULL;
    q->desccount = 0;
}

/**
 * Clean up a queue freeing its memory allocation.
 * @param   q           queue
 */
static void unvme_queue_cleanup(unvme_queue_t* q)
{
    // free all descriptors
    unvme_desc_t* desc;
    while ((desc = q->desclist) != NULL) {
        LIST_DEL(q->desclist, desc);
        free(desc);
        INFO_FN("descs: %d / %d", --descs, descs_max);
    }
    while ((desc = q->descfree) != NULL) {
        LIST_DEL(q->descfree, desc);
        free(desc);
        INFO_FN("descs: %d / %d", --descs, descs_max);
    }

    if (q->prplist.valid) mem_free(&q->prplist);
    if (q->cqdma.valid) mem_free(&q->cqdma);
    if (q->sqdma.valid) mem_free(&q->sqdma);
}

/**
 * Setup admin queue.
 * @param   dev         device context
 * @param   qsize       admin queue depth
 */
static void unvme_adminq_create(unvme_device_t* dev)
{
    DEBUG_FN("%x", PCI_DEV);
    unvme_queue_t* adminq = &dev->adminq;
    unvme_queue_init(dev, adminq, UNVME_AQSIZE);
    adminq->isadmin = 1;
    if (!nvme_adminq_setup(&dev->nvmedev, UNVME_AQSIZE,
                           adminq->sqdma.dma_buf, adminq->sqdma.dma_addr,
                           adminq->cqdma.dma_buf, adminq->cqdma.dma_addr))
        FATAL("nvme_setup_adminq failed");
    adminq->adminq = &dev->nvmedev.adminq;
}

/**
 * Delete admin queue.
 * @param   dev         device context
 */
static void unvme_adminq_delete(unvme_device_t* dev)
{
    DEBUG_FN("%x", PCI_DEV);
    unvme_queue_cleanup(&dev->adminq);
}

/**
 * Create an I/O queue.
 * @param   dev         device context
 * @param   q           queue index starting at 0
 */
static void unvme_ioq_create(unvme_device_t* dev, int q)
{
    DEBUG_FN("%x q=%d", PCI_DEV, q+1);
    unvme_queue_t* ioq = &dev->ioqs[q];
    unvme_queue_init(dev, ioq, dev->nscnt.qsize);
    if (nvme_ioq_create(&dev->nvmedev, &ioq->nvmeq, q+1, ioq->size,
                                       ioq->sqdma.dma_buf, ioq->sqdma.dma_addr,
                                       ioq->cqdma.dma_buf, ioq->cqdma.dma_addr))
        FATAL("nvme_ioq_create %d failed", q+1);
    DEBUG_FN("%x q=%d qd=%d db=%#04lx", PCI_DEV, ioq->nvmeq.id,
             ioq->size, (u32)ioq->nvmeq.sq_doorbell - (u32)dev->nvmedev.reg);
}

/**
 * Delete an I/O queue.
 * @param   dev         device context
 * @param   q           queue index starting at 0
 */
static void unvme_ioq_delete(unvme_device_t* dev, int q)
{
    DEBUG_FN("%x %d", PCI_DEV, q+1);
    unvme_queue_t* ioq = &dev->ioqs[q];
    (void)nvme_ioq_delete(&ioq->nvmeq);
    unvme_queue_cleanup(ioq);
}

/**
 * Initialize a namespace instance.
 * @param   ns          namespace context
 * @param   nsid        namespace id
 */
static void unvme_ns_init(unvme_device_t* dev)
{
	unvme_ns_t *ns = &dev->nsio;
    ns->id = NSID;
    ns->maxiopq = ns->qsize - 1;

    mem_t* mem = mem_alloc(&dev->memdev, NULL, ns->pagesize, 1);

    if (nvme_acmd_identify(&dev->nvmedev, NSID, mem->dma_addr, 0))
        FATAL("nvme_acmd_identify %d failed", NSID);
    nvme_identify_ns_t* idns = (nvme_identify_ns_t*)mem->dma_buf;
    ns->blockcount = idns->ncap;
    ns->blockshift = idns->lbaf[idns->flbas & 0xF].lbads;
    ns->blocksize = 1 << ns->blockshift;
    ns->bpshift = ns->pageshift - ns->blockshift;
    ns->nbpp = 1 << ns->bpshift;
    ns->pagecount = ns->blockcount >> ns->bpshift;
    ns->maxbpio = ns->maxppio << ns->bpshift;
    mem_free(mem);

    DEBUG_FN("%s qc=%d qd=%d bs=%d bc=%#lx mbio=%d", PCI_DEV_NAME, dev->nscnt.qcount,
    		dev->nscnt.qsize, dev->nscnt.blocksize, dev->nscnt.blockcount, dev->nscnt.maxbpio);
}

/**
 * Clean up.
 */
static void unvme_cleanup(unvme_device_t* dev)
{
    if (--dev->refcount == 0) {
        DEBUG_FN("%s", PCI_DEV_NAME);
        int q;
        for (q = 0; q < dev->nscnt.qcount; q++) unvme_ioq_delete(dev, q);
        unvme_adminq_delete(dev);
        mem_delete(&dev->memdev);
        free(dev->iomem.map);
    }
}


/**
 * Open and attach to a UNVMe driver.
 * @param   pci         PCI device id
 * @param   nsid        namespace id
 * @param   qcount      number of queues (0 for max number of queues support)
 * @param   qsize       size of each queue (0 default to 65)
 * @return  namespace pointer or NULL if error.
 */
int unvme_do_open(unvme_device_t* dev)
{
	// setup controller namespace
	memset(dev, 0, sizeof(*dev));
	mem_create(&dev->memdev);
	nvme_create(&dev->nvmedev);
	unvme_adminq_create(dev);

	DEBUG_FN("Created aqueue");

	// get controller info
	mem_t* mem = mem_alloc(&dev->memdev, NULL, 4096, 1);
	if (nvme_acmd_identify(&dev->nvmedev, 0, mem->dma_addr, 0))
		FATAL("nvme_acmd_identify controller failed");
	static nvme_identify_ctlr_t idc;
	memcpy(&idc, mem->dma_buf, sizeof(nvme_identify_ctlr_t));

	if (NSID > idc.nn) {
		ERROR("invalid %06x nsid %d (max %d)", PCI_DEV, NSID, idc.nn);
		return 1;
	}

	unvme_ns_t* ns = &dev->nscnt;
	ns->id = 0;
	ns->nscount = idc.nn;
	ns->maxqsize = dev->nvmedev.maxqsize;
	ns->pageshift = dev->nvmedev.pageshift;
	ns->pagesize = 1 << ns->pageshift;
	int i;
	ns->vid = idc.vid;
	memcpy(ns->mn, idc.mn, sizeof (ns->mn));
	for (i = sizeof (ns->mn) - 1; i > 0 && ns->mn[i] == ' '; i--) ns->mn[i] = 0;
	memcpy(ns->sn, idc.sn, sizeof (ns->sn));
	for (i = sizeof (ns->sn) - 1; i > 0 && ns->sn[i] == ' '; i--) ns->sn[i] = 0;
	memcpy(ns->fr, idc.fr, sizeof (ns->fr));
	for (i = sizeof (ns->fr) - 1; i > 0 && ns->fr[i] == ' '; i--) ns->fr[i] = 0;

	// set limit to 1 PRP list page per IO submission
	ns->maxppio = ns->pagesize / sizeof(u64);
	if (idc.mdts) {
		int mp = 2 << (idc.mdts - 1);
		if (ns->maxppio > mp) ns->maxppio = mp;
	}
	mem_free(mem);

	// get max number of queues supported
	nvme_feature_num_queues_t nq;
	if (nvme_acmd_get_features(&dev->nvmedev, 0,
							   NVME_FEATURE_NUM_QUEUES, 0, 0, (u32*)&nq))
		FATAL("nvme_acmd_get_features number of queues failed");
	u32 maxqcount = (nq.nsq < nq.ncq ? nq.nsq : nq.ncq) + 1;
	u32 qcount = (maxqcount > UNVME_QCOUNT ? UNVME_QCOUNT : maxqcount);
	u32 qsize = (dev->nvmedev.maxqsize > UNVME_QSIZE ? UNVME_QSIZE : dev->nvmedev.maxqsize);

	ns->maxqcount = maxqcount;
	ns->qcount = qcount;
	ns->qsize = qsize;

	// setup IO queues
	DEBUG_FN("Creating %d IO queues (of max %d), queue size %d", qcount, maxqcount, qsize);
	for (i = 0; i < qcount; i++) unvme_ioq_create(dev, i);

    dev->refcount++;
    memcpy(&dev->nsio, ns, sizeof(unvme_ns_t));
    unvme_ns_init(dev);

    INFO_FN("%s/%d (%.40s) is ready", PCI_DEV_NAME, NSID, dev->nsio.mn);
    return 0;
}

/**
 * Close and detach from a UNVMe driver.
 * @param   ns          namespace handle
 * @return  0 if ok else -1.
 */
int unvme_do_close(unvme_device_t* dev)
{
    DEBUG_FN("%s", PCI_DEV_NAME);
    unvme_cleanup(dev);
    return 0;
}

/**
 * Allocate an I/O buffer.
 * @param   ns          namespace handle
 * @param   size        buffer size
 * @return  the allocated buffer or NULL if failure.
 */
void* unvme_do_alloc(unvme_device_t* dev, u64 size)
{
    DEBUG_FN("%s %#lx", PCI_DEV_NAME, size);
    unvme_iomem_t* iomem = &dev->iomem;
    void* buf = NULL;

    mem_t* mem = mem_alloc(&dev->memdev, NULL, size, 0);
    if (mem) {
        if (iomem->count == iomem->size) {
            iomem->size += 256;
            iomem->map = realloc(iomem->map, iomem->size * sizeof(void*));
        }
        iomem->map[iomem->count++] = mem;
        buf = mem->dma_buf;
    }
    return buf;
}

/**
 * Free an I/O buffer.
 * @param   ns          namespace handle
 * @param   buf         buffer pointer
 * @return  0 if ok else -1.
 */
int unvme_do_free(unvme_device_t* dev, void* buf)
{
    DEBUG_FN("%s %p", PCI_DEV_NAME, buf);
    unvme_iomem_t* iomem = &dev->iomem;

    int i;
    for (i = 0; i < iomem->count; i++) {
        if (buf == iomem->map[i]->dma_buf) {
            mem_free(iomem->map[i]);
            iomem->count--;
            if (i != iomem->count)
                iomem->map[i] = iomem->map[iomem->count];
            return 0;
        }
    }
    return -1;
}

/**
 * Poll for completion status of a previous IO submission.
 * If there's no error, the descriptor will be released.
 * @param   desc        IO descriptor
 * @param   timeout     in seconds
 * @param   cqe_cs      CQE command specific DW0 returned
 * @return  0 if ok else error status (-1 means timeout).
 */
int unvme_do_poll(unvme_desc_t* desc, int timeout, u32* cqe_cs)
{
    if (desc->sentinel != desc)
        FATAL("bad IO descriptor");

    PDEBUG("# POLL d={%d %d %#lx}", desc->id, desc->cidcount, *desc->cidmask);
    int err = 0;
    while (desc->cidcount) {
        if ((err = unvme_check_completion(desc->q, timeout, cqe_cs)) != 0) break;
    }
    if (desc->cidcount == 0) unvme_desc_put(desc);
    PDEBUG("# q%d +%d", desc->q->nvmeq->id, desc->q->desccount);

    return err;
}

/**
 * Submit a read/write command that may require multiple I/O submissions
 * and processing some completions.
 * @param   ns          namespace handle
 * @param   qid         queue id
 * @param   opc         op code
 * @param   buf         data buffer
 * @param   slba        starting lba
 * @param   nlb         number of logical blocks
 * @return  I/O descriptor or NULL if error.
 */
unvme_desc_t* unvme_do_rw(unvme_device_t* dev, int qid, int opc,
                          void* buf, u64 slba, u32 nlb)
{
    unvme_queue_t* q = &dev->ioqs[qid];
    unvme_desc_t* desc = unvme_desc_get(q);
    desc->opc = opc;
    desc->buf = buf;
    desc->qid = qid;
    desc->slba = slba;
    desc->nlb = nlb;
    desc->sentinel = desc;

    PDEBUG("# %s %#lx %#x @%d +%d", opc == NVME_CMD_READ ? "READ" : "WRITE",
           slba, nlb, desc->id, q->desccount);
    while (nlb) {
        int n = dev->nsio.maxbpio;
        if (n > nlb) n = nlb;
        int cid = unvme_submit_io(dev, desc, buf, slba, n);
        if (cid < 0) {
            // poll currently pending descriptor
            int err = unvme_do_poll(desc, UNVME_TIMEOUT, NULL);
            if (err) {
                if (err == -1) FATAL("q%d timeout", q->nvmeq.id);
                else ERROR("q%d error %#x", q->nvmeq.id, err);
            }
        }

        buf += n << dev->nsio.blockshift;
        slba += n;
        nlb -= n;
    }

    return desc;
}

/**
 * Submit a generic or vendor specific command.
 * @param   ns          namespace handle
 * @param   qid         client queue index (-1 for admin queue)
 * @param   opc         command op code
 * @param   nsid        namespace id
 * @param   cdw10_15    NVMe command word 10 through 15
 * @param   buf         data buffer (from unvme_alloc)
 * @param   bufsz       data buffer size
 * @return  command descriptor or NULL if error.
 */
unvme_desc_t* unvme_do_cmd(unvme_device_t* dev, int qid, int opc, int nsid,
                           void* buf, u64 bufsz, u32 cdw10_15[6])
{
    unvme_queue_t* q = (qid == -1) ? &dev->adminq : &dev->ioqs[qid];
    nvme_queue_t *nvmeq = (q->isadmin) ? q->adminq : &q->nvmeq;
    unvme_desc_t* desc = unvme_desc_get(q);
    desc->opc = opc;
    desc->buf = buf;
    desc->qid = qid;
    desc->sentinel = desc;

    u64 prp1, prp2;
    u16 cid = unvme_get_cid(desc);
    if (unvme_map_prps(dev, q, cid, buf, bufsz, &prp1, &prp2) ||
        nvme_cmd_vs(nvmeq, opc, cid, nsid, prp1, prp2, cdw10_15)) {
        unvme_desc_put(desc);
        return NULL;
    }

    PDEBUG("# CMD=%#x %d q%d={%d %d %#lx} d={%d %d %#lx}",
           opc, nsid, q->nvmeq->id, cid, q->cidcount, *q->cidmask,
           desc->id, desc->cidcount, *desc->cidmask);
    return desc;
}

