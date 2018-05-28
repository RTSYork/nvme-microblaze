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
 * @brief Common NVMe setup functions.
 */

#include <stdio.h>
//#include <err.h>

#include "nvme_tests.h"

#include "../unvme/unvme_mem.h"
#include "../unvme/unvme_nvme.h"
#include "../unvme/unvme_log.h"

static mem_device_t* memdev;
static nvme_device_t* nvmedev;
static mem_dma_t* adminsq;
static mem_dma_t* admincq;

#define errx(code, fmt, arg...) do { printf("error: " fmt "\n\r", ##arg); exit(code); } while (0)

/**
 * NVMe setup.
 */
static void nvme_setup(int pci, int aqsize)
{
    if (log_open_stdout()) exit(1);
    memdev = mem_create(NULL, pci);
    if (!memdev) errx(1, "vfio_create");

    nvmedev = nvme_create(NULL);
    if (!nvmedev) errx(1, "nvme_create");

    adminsq = mem_dma_alloc(memdev, aqsize * sizeof(nvme_sq_entry_t), 1);
    if (!adminsq) errx(1, "vfio_dma_alloc");
    admincq = mem_dma_alloc(memdev, aqsize * sizeof(nvme_cq_entry_t), 1);
    if (!admincq) errx(1, "vfio_dma_alloc");

    if (!nvme_adminq_setup(nvmedev, aqsize, adminsq->buf, adminsq->addr,
                                            admincq->buf, admincq->addr)) {
        errx(1, "nvme_setup_adminq");
    }
}

/**
 * NVMe cleanup.
 */
static void nvme_cleanup()
{
    mem_dma_free(adminsq);
    mem_dma_free(admincq);
    nvme_delete(nvmedev);
    mem_delete(memdev);
}

