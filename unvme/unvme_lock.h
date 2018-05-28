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
 * @brief UNVMe fast read lock with occasional writes.
 */
#if 0

#include <sched.h>

/// Lock write bit
#define UNVME_LOCKWBIT      0x80000000

/// Simple read write lock
typedef unsigned unvme_lock_t;


/**
 * Increment read lock and wait if pending write.
 * @param   lock    lock variable
 */
static inline void unvme_lockr(unvme_lock_t* lock)
{
    for (;;) {
        if (!(__sync_fetch_and_add(lock, 1) & UNVME_LOCKWBIT)) return;
        __sync_fetch_and_sub(lock, 1);
        sched_yield();
    }
}

/**
 * Decrement read lock.
 * @param   lock    lock variable
 */
static inline void unvme_unlockr(unvme_lock_t* lock)
{
    __sync_fetch_and_sub(lock, 1);
}

/**
 * Acquire write lock and wait for all pending read/write.
 * @param   lock    lock variable
 */
static inline void unvme_lockw(unvme_lock_t* lock)
{
    for (;;) {
        int val = __sync_fetch_and_or(lock, UNVME_LOCKWBIT);
        if (val == 0) return;
        sched_yield();

        // if not pending write then just wait for all reads to clear
        if (!(val & UNVME_LOCKWBIT)) {
            while (*lock != UNVME_LOCKWBIT) sched_yield();
            return;
        }
    }
}

/**
 * Clear the write lock.
 * @param   lock    lock variable
 */
static inline void unvme_unlockw(unvme_lock_t* lock)
{
    __sync_fetch_and_and(lock, ~UNVME_LOCKWBIT);
}

#endif
