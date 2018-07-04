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
 * @brief Invoke NVMe identify command.
 */

#include <string.h>
#include <stdio.h>
#include "nvme_tests.h"
#include "../unvme/unvme_mem.h"
#include "../unvme/unvme_nvme.h"
#include "../unvme/unvme_log.h"

static mem_device_t memdev;
static nvme_device_t nvmedev;
static mem_t* adminsq;
static mem_t* admincq;

/**
 * NVMe setup.
 */
static void nvme_setup(int aqsize)
{
    int ret = mem_create(&memdev);
    if (ret) errx(1, "vfio_create");

    ret = nvme_create(&nvmedev);
    if (ret) errx(1, "nvme_create");

    adminsq = mem_alloc(&memdev, NULL, aqsize * sizeof(nvme_sq_entry_t), 1);
    if (!adminsq) errx(1, "vfio_dma_alloc");
    admincq = mem_alloc(&memdev, NULL, aqsize * sizeof(nvme_cq_entry_t), 1);
    if (!admincq) errx(1, "vfio_dma_alloc");

    if (!nvme_adminq_setup(&nvmedev, aqsize, adminsq->dma_buf, adminsq->dma_addr,
                                            admincq->dma_buf, admincq->dma_addr)) {
        errx(1, "nvme_setup_adminq");
    }
}

/**
 * NVMe cleanup.
 */
static void nvme_cleanup()
{
    mem_free(adminsq);
    mem_free(admincq);
    mem_delete(&memdev);
}


/**
 * Print controller info.
 */
void print_controller(void* buf)
{
    nvme_identify_ctlr_t* ctlr = buf;

    printf("Identify Controller 0x0\n");
    printf("=======================\n");
    printf("vid      : %#x\n", ctlr->vid);
    printf("ssvid    : %#x\n", ctlr->ssvid);
    printf("sn       : %.20s\n", ctlr->sn);
    printf("mn       : %.40s\n", ctlr->mn);
    printf("fr       : %.8s\n", ctlr->fr);
    printf("rab      : %#x\n", ctlr->rab);
    printf("ieee     : %02x%02x%02x\n", ctlr->ieee[0], ctlr->ieee[1], ctlr->ieee[2]);
    printf("mic      : %#x\n", ctlr->mic);
    printf("mdts     : %#x\n", ctlr->mdts);
    printf("oacs     : %#x\n", ctlr->oacs);
    printf("acl      : %#x\n", ctlr->acl);
    printf("aerl     : %#x\n", ctlr->aerl);
    printf("frmw     : %#x\n", ctlr->frmw);
    printf("lpa      : %#x\n", ctlr->lpa);
    printf("elpe     : %#x\n", ctlr->elpe);
    printf("npss     : %#x\n", ctlr->npss);
    printf("avscc    : %#x\n", ctlr->avscc);
    printf("sqes     : %#x\n", ctlr->sqes);
    printf("cqes     : %#x\n", ctlr->cqes);
    printf("nn       : %#lx\n", ctlr->nn);
    printf("oncs     : %#x\n", ctlr->oncs);
    printf("fuses    : %#x\n", ctlr->fuses);
    printf("fna      : %#x\n", ctlr->fna);
    printf("vwc      : %#x\n", ctlr->vwc);
    printf("awun     : %#x\n", ctlr->awun);
    printf("awupf    : %#x\n", ctlr->awupf);
    printf("nvscc    : %#x\n", ctlr->nvscc);
}

/**
 * Print namespace info.
 */
void print_namespace(void* buf, int nsid)
{
    nvme_identify_ns_t* ns = buf;

    printf("\nIdentify Namespace %#x\n", nsid);
    printf("======================\n");
    printf("nsze     : %#llx\n", ns->nsze);
    printf("ncap     : %#llx\n", ns->ncap);
    printf("nuse     : %#llx\n", ns->nuse);
    printf("nsfeat   : %#x\n", ns->nsfeat);
    printf("nlbaf    : %#x\n", ns->nlbaf);
    printf("flbas    : %#x\n", ns->flbas);
    printf("mc       : %#x\n", ns->mc);
    printf("dpc      : %#x\n", ns->dpc);
    printf("dps      : %#x\n", ns->dps);

    int i;
    for (i = 0; i <= ns->nlbaf; i++) {
        printf("lbaf.%-3d : ms=%-5d lbads=%-3d rp=%-2d %s\n",
               i, ns->lbaf[i].ms, ns->lbaf[i].lbads, ns->lbaf[i].rp,
               (ns->flbas & 0xf) == i ? "(formatted)" : "");
    }
}

/**
 * Main program.
 */
int nvme_identify()
{
	printf("\r\n%s test starting...\r\n\n", __func__);

    int nsid = 0;

    nvme_setup(8);
    mem_t* mem = mem_alloc(&memdev, NULL, 16384, 0);
    if (!mem) errx(1, "mem_alloc");

    if (nvme_acmd_identify(&nvmedev, 0, mem->dma_addr, mem->dma_addr + 4096))
        errx(1, "nvme_acmd_identify 0");
    static nvme_identify_ctlr_t ctlr;
    memcpy(&ctlr, mem->dma_buf, sizeof(nvme_identify_ctlr_t));
    print_controller(&ctlr);

    u64 nsaddr = mem->dma_addr + 8192;
    void* nsbuf = mem->dma_buf + 8192;

    if (nsid) {
        if (nsid > ctlr.nn)
            errx(1, "invalid nsid %d", nsid);
        if (nvme_acmd_identify(&nvmedev, nsid, nsaddr, nsaddr + 4096))
            errx(1, "nvme_acmd_identify %d", nsid);
        print_namespace(nsbuf + 8192, nsid);
    } else {
        for (nsid = 1; nsid <= ctlr.nn; nsid++) {
            if (nvme_acmd_identify(&nvmedev, nsid, nsaddr, nsaddr + 4096))
                errx(1, "nvme_acmd_identify %d", nsid);
            print_namespace(nsbuf + 8192, nsid);
        }
    }

    mem_free(mem);
    nvme_cleanup();

    printf("\r\n%s test complete\r\n\n", __func__);
    return 0;
}

