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
 * @brief UNVMe API test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include "../unvme/unvme.h"

#include "../timer.h"

/// Print if verbose flag is set
#define VERBOSE(fmt, arg...) if (verbose) printf(fmt, ##arg)


/**
 * Main.
 */
int unvme_api_test(int verbose, int ratio, char* pciname)
{
	printf("\r\nunvme_api_test test starting...\r\n\n");

    if (ratio < 1) ratio = 4;

    printf("API TEST BEGIN\n\r");
    const unvme_ns_t* ns = unvme_open(pciname);
    if (!ns) exit(1);

    // set large number of I/O and size
    int maxnlb = ratio * ns->maxbpio;
    int iocount = ratio * (ns->qsize - 1);

    printf("%s qc=%ld/%ld qs=%ld/%ld bc=%#llx bs=%d maxnlb=%d/%d\n\r",
            ns->device, ns->qcount, ns->maxqcount, ns->qsize, ns->maxqsize,
            ns->blockcount, ns->blocksize, maxnlb, ns->maxbpio);

    int q, i, nlb;
    u64 slba, size, w, *p;
    unvme_iod_t* iod = malloc(iocount * sizeof(unvme_iod_t));
    void** buf = malloc(iocount * sizeof(void*));

    u64 tstart = timer_get_value();
    for (q = 0; q < ns->qcount; q++) {
        printf("> Test q=%d ioc=%d\n\r", q, iocount);
        u64 t = 0;//timer_get_value();

        printf("Test alloc\n\r");
        srandom(t);
        for (i = 0; i < iocount; i++) {
            nlb = random() % maxnlb + 1;
            size = nlb * ns->blocksize;
            VERBOSE("  alloc.%-2d  %#8x %#llx\n\r", i, nlb, size);
            if (!(buf[i] = unvme_alloc(ns, size))) {
            	printf("error: alloc.%d failed\r\n", i);
            	return 1;
            }
        }

        printf("Test awrite\n\r");
        srandom(t);
        slba = 0;
        for (i = 0; i < iocount; i++) {
            nlb = random() % maxnlb + 1;
            size = nlb * ns->blocksize / sizeof(u64);
            p = buf[i];
            for (w = 0; w < size; w++) p[w] = (w << 32) + i;
            VERBOSE("  awrite.%-2d %#8x %p %#llx\n\r", i, nlb, p, slba);
            if (!(iod[i] = unvme_awrite(ns, q, p, slba, nlb))) {
            	printf("error: awrite.%d failed\r\n", i);
            	return 1;
            }
            slba += nlb;
        }

        printf("Test apoll.awrite\n\r");
        for (i = iocount-1; i >= 0; i--) {
            VERBOSE("  apoll.awrite.%-2d\n\r", i);
            if (unvme_apoll(iod[i], UNVME_TIMEOUT)) {
            	printf("error: apoll_awrite.%d failed\r\n", i);
                return 1;
            }
        }

        printf("Test aread\n\r");
        srandom(t);
        slba = 0;
        for (i = 0; i < iocount; i++) {
            nlb = random() % maxnlb + 1;
            size = nlb * ns->blocksize;
            p = buf[i];
            for (int i = 0; i < size/sizeof(u64); i++) p[i] = 0;
            VERBOSE("  aread.%-2d  %#8x %p %#llx\n\r", i, nlb, p, slba);
            if (!(iod[i] = unvme_aread(ns, q, p, slba, nlb))) {
            	printf("error: aread.%d failed\r\n", i);
            	return 1;
            }
            slba += nlb;
        }

        printf("Test apoll.aread\n\r");
        for (i = iocount-1; i >= 0; i--) {
            VERBOSE("  apoll.aread.%-2d\n\r", i);
            if (unvme_apoll(iod[i], UNVME_TIMEOUT)) {
            	printf("error: apoll_aread.%d failed\r\n", i);
            	return 1;
            }
        }

        printf("Test verify\n\r");
        srandom(t);
        slba = 0;
        for (i = 0; i < iocount; i++) {
            nlb = random() % maxnlb + 1;
            size = nlb * ns->blocksize / sizeof(u64);
            p = buf[i];
            VERBOSE("  verify.%-2d %#8x %p %#llx\n\r", i, nlb, p, slba);
            for (w = 0; w < size; w++) {
                if (p[w] != ((w << 32) + i)) {
                    w *= sizeof(w);
                    slba += w / ns->blocksize;
                    w %= ns->blocksize;
                    printf("error: miscompare at lba %#llx offset %#llx\r\n", slba, w);
                    return 1;
                }
            }
            slba += nlb;
        }

        printf("Test free\n\r");
        for (i = 0; i < iocount; i++) {
            VERBOSE("  free.%-2d\n\r", i);
            if (unvme_free(ns, buf[i])) {
            	printf("error: free.%d failed\r\n", i);
            	return 1;
            }
        }
    }

    free(buf);
    free(iod);
    unvme_close(ns);

    printf("API TEST COMPLETE (%lld secs)\n\r", (timer_get_value() - tstart) / TIMER_TICKS_PER_SECOND);


	printf("\r\nunvme_api_test test complete\r\n\n");
    return 0;
}
