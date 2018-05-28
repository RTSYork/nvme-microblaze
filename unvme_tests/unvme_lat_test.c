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
 * @brief UNVMe I/O latency test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>

#include "../unvme/unvme.h"
#include "../unvme/rdtsc.h"

/// macro to print an io related error message
#define IOERROR(s, p)   printf("ERROR: " s " lba=%#llx\r\n", (p)->lba)

/// page structure
typedef struct {
    void*           buf;        ///< IO buffer
    u64             lba;        ///< lba
    unvme_iod_t     iod;        ///< returned IO descriptor
    u64             tsc;        ///< tsc time
} lat_page_t;

// Global variables
static const unvme_ns_t* ns;    ///< unvme namespace pointer
static int qcount = 1;          ///< queue count
static int qsize = 8;           ///< queue size
static int runtime = 15;        ///< run time in seconds
static u64 endtsc;              ///< end run tsc
static u64 timeout;             ///< tsc elapsed timeout
static u64 last_lba;            ///< last page boundary lba
static u64 ioc;                 ///< total number of io count
static u64 avg_slat;            ///< total submission time
static u64 avg_clat;            ///< total completion time
static u64 min_slat;            ///< minimum submission time
static u64 max_slat;            ///< maximum submission time
static u64 min_clat;            ///< minimum completimesn time
static u64 max_clat;            ///< maximum completimesn time

/**
 * Submit an io and record the submission latency time.
 */
static void io_submit(int q, int rw, lat_page_t* p)
{
    p->lba = random() & ~(ns->nbpp - 1);
    if (p->lba > last_lba) p->lba &= last_lba;

    p->tsc = rdtsc();
    if (rw) {
        p->iod = unvme_awrite(ns, q, p->buf, p->lba, ns->nbpp);
        if (!p->iod) {
        	IOERROR("awrite", p);
           	return;
        }
    } else {
        p ->iod = unvme_aread(ns, q, p->buf, p->lba, ns->nbpp);
        if (!p->iod) {
        	IOERROR("aread", p);
        	return;
        }
    }
    ioc++;

    u64 ts = rdtsc_elapse((u64)(p->tsc));
    if (min_slat > ts) min_slat = ts;
    if (max_slat < ts) max_slat = ts;
    avg_slat += ts;
}

/**
 * Queue thread test.
 */
static void* run_thread(void* arg)
{
    int rw = (long)arg >> 16;
    int q = (long)arg & 0xffff;
    int qdepth = qsize - 1;

    u64 lba = (q * qcount * qdepth * ns->nbpp) << 1;
    lat_page_t* pages = calloc(ns->maxiopq, sizeof(lat_page_t));
    lat_page_t* p = pages;
    int i;
    for (i = 0; i < ns->maxiopq; i++) {
        p->buf = unvme_alloc(ns, ns->pagesize);
        lba += (ns->nbpp << 1);
        if (lba > last_lba) lba = i * ns->nbpp;
        p->lba = lba;
        p++;
    }

    for (i = 0; i < qdepth; i++) io_submit(q, rw, pages + i);

    i = 0;
    int pending = qdepth;
    do {
        p = pages + i;
        if (p->iod) {
            if (unvme_apoll(p->iod, 0) == 0) {
                u64 tc = rdtsc_elapse(p->tsc);
                if (min_clat > tc) min_clat = tc;
                if (max_clat < tc) max_clat = tc;
                avg_clat += tc;
        
                if ((tc + p->tsc) < endtsc) {
                    io_submit(q, rw, p);
                } else {
                    p->iod = 0;
                    pending--;
                }
            } else if ((rdtsc_elapse(p->tsc)) > timeout) {
                IOERROR("apoll timeout", p);
            }
        }
        if (++i == ns->maxiopq) i = 0;
    } while (pending > 0);

    p = pages;
    for (i = 0; i < ns->maxiopq; i++) {
        unvme_free(ns, p->buf);
        p++;
    }
    free(pages);
    return 0;
}


/**
 * Run test to spawn one thread for each queue.
 */
void run_test(const char* name, int rw)
{
    ioc = 0;
    avg_slat = 0;
    avg_clat = 0;
    min_slat = -1;
    max_slat = 0;
    min_clat = -1;
    max_clat = 0;

    u64 tsec = rdtsc_second();
    int q;
    for (q = 0; q < qcount; q++) {
        long arg = (rw << 16) + q;
        run_thread((void*)arg);
    }

    sleep(1);

    printf("%s: run test for %d seconds\n",
           name, runtime);
    endtsc = rdtsc() + ((u64)runtime * tsec);
    timeout = (u64)UNVME_TIMEOUT * tsec;

    u64 utsc = tsec / 1000000;
    printf("%s: slat=(%.2f-%.2f %.2f) lat=(%.2f-%.2f %.2f) usecs ioc=%llu\n",
            name, (double)min_slat/utsc, (double)max_slat/utsc,
            (double)avg_slat/ioc/utsc, (double)min_clat/utsc,
            (double)max_clat/utsc, (double)avg_clat/ioc/utsc, ioc);
    /*
    printf("%s: slat=(%lu-%lu %lu) lat=(%lu-%lu %lu) tscs ioc=%lu\n",
            name, min_slat, max_slat, avg_slat/ioc,
            min_clat, max_clat, avg_clat/ioc, ioc);
    */
}

/**
 * Main program.
 */
int unvme_lat_test(int runtime_in, int qcount_in, int qsize_in, int pci, int nsid)
{
	printf("\r\nunvme_lat_test test starting...\r\n\n");

    if (runtime_in > 0) runtime = runtime_in;
    if (qcount_in > 0) qcount = qcount_in;
    if (qsize_in > 1) qsize = qsize_in;


    printf("LATENCY TEST BEGIN\n");
    time_t tstart = timer_get_value();
    if (!(ns = unvme_open(pci, nsid))) exit(1);
    if (qcount <= 0 || qcount > ns->qcount) {
    	printf("error: qcount limit %ld\r\n", ns->qcount);
    	return 1;
    }
    if (qsize <= 1 || qsize > ns->qsize) {
    	printf("error: qsize limit %ld\r\n", ns->qsize);
    	return 1;
    }

    last_lba = (ns->blockcount - ns->nbpp) & ~(u64)(ns->nbpp - 1);
    if (!qcount) qcount = ns->qcount;
    if (!qsize) qsize = ns->qsize;

    printf("%s qc=%d/%ld qs=%d/%ld bc=%#llx bs=%d mbio=%d\n",
            ns->device, qcount, ns->qcount, qsize, ns->qsize,
            ns->blockcount, ns->blocksize, ns->maxbpio);

    run_test("read", 0);
    run_test("write", 1);

    unvme_close(ns);

    printf("LATENCY TEST COMPLETE (%lld secs)\n", (timer_get_value() - tstart) / TIMER_TICKS_PER_SECOND);

    printf("\r\nunvme_lat_test test complete\r\n\n");
    return 0;
}

