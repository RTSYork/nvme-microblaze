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
 * @brief UNVMe simple write-read-verify test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include "unvme_tests.h"

#include "../unvme/unvme.h"

#include "../timer.h"


int unvme_sim_test(u64 lba, u64 size, int pci, int nsid)
{
	printf("\r\nunvme_sim_test test starting...\r\n\n");

    u64 datasize = (size > 0) ? size : 100 * 1024 * 1024;
    u64 slba = (lba > 0) ? lba : -1L;

    printf("SIMPLE WRITE-READ-VERIFY TEST BEGIN\n\r");
    const unvme_ns_t* ns = unvme_open(pci, nsid);
    if (!ns) exit(1);
    printf("%s qc=%ld/%ld qs=%ld/%ld bc=%#llx bs=%d mbio=%d ds=%#llx\n\r",
            ns->device, ns->qcount, ns->maxqcount, ns->qsize, ns->maxqsize,
            ns->blockcount, ns->blocksize, ns->maxbpio, datasize);

    void* buf = unvme_alloc(ns, datasize);
    if (!buf) {
    	printf("error: unvme_alloc %lld failed\r\n", datasize);
    	return 1;
    }
    u64 tstart = timer_get_value();
    u64 nlb = datasize / ns->blocksize;
    if (!nlb) nlb = 1;

    if (slba == -1L) {
//        srandom(timer_get_value());
        srandom(0);
        slba = (random() % ns->blockcount) - (ns->qcount * nlb);
        slba &= ~(ns->nbpp - 1);
        if (slba >= ns->blockcount) slba = 0;
    }

    u64* p = buf;
    u64 wsize = datasize / sizeof(u64);
    u64 w;
    int q, stat;

    for (q = 0; q < ns->qcount; q++) {
        printf("Test q=%-2d lba=%#llx nlb=%#llx\n\r", q, slba, nlb);
        for (w = 0; w < wsize; w++) {
            u64 pat = (q << 24) + w;
            p[w] = (pat << 32) | (~pat & 0xffffffff);
        }
        stat = unvme_write(ns, q, p, slba, nlb);
        if (stat) {
        	printf("error: unvme_write failed: slba=%#llx nlb=%#llx stat=%#x\r\n", slba, nlb, stat);
            return 1;
        }
        for (int i = 0; i < (nlb * ns->blocksize) / sizeof(u64); i++) p[i] = 0;
        stat = unvme_read(ns, q, p, slba, nlb);
        if (stat) {
        	printf("error: unvme_read failed: slba=%#llx nlb=%#llx stat=%#x\r\n", slba, nlb, stat);
			return 1;
		}
        for (w = 0; w < wsize; w++) {
            u64 pat = (q << 24) + w;
            pat = (pat << 32) | (~pat & 0xffffffff);
            if (p[w] != pat) {
                w *= sizeof(w);
                slba += w / ns->blocksize;
                w %= ns->blocksize;
                printf("error: miscompare at lba %#llx offset %#llx\r\n", slba, w);
                return 1;
            }
        }
        slba += nlb;
        if (slba >= ns->blockcount) slba = 0;
    }

    unvme_free(ns, buf);
    unvme_close(ns);

    printf("SIMPLE WRITE-READ-VERIFY TEST COMPLETE (%lld secs)\n\r", (timer_get_value() - tstart) / TIMER_TICKS_PER_SECOND);
//    printf("SIMPLE WRITE-READ-VERIFY TEST COMPLETE\n\r");

    printf("\r\nunvme_sim_test test complete\r\n\n");
    return 0;
}

