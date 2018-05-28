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
 * @brief memory support routines.
 */

#include "unvme_mem.h"

#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "pci_regs.h"
#include "../pcie.h"

#include "unvme_log.h"

/// Print fatal error and exit
#define FATAL(fmt, arg...)  do { ERROR(fmt, ##arg); abort(); } while (0)


/**
 * Allocate memory memory.  The size will be rounded to page aligned size.
 * If pmb is set, it indicates memory has been premapped.
 * @param   dev         device context
 * @param   size        size
 * @param   pmb         premapped buffer
 * @return  memory structure pointer or NULL if error.
 */
static mem_t* mem_alloc(mem_device_t* dev, size_t size, void* pmb, int clear)
{
    mem_t* mem = zalloc(sizeof(*mem));
    mem->size = size;
    size_t mask = dev->pagesize - 1;
    size = (size + mask) & ~mask;

    if (pmb) {
        mem->dma.buf = pmb;
    } else {
        if (dev->memoff + size > dev->memsize) {
            ERROR("Out of memory space (next allocation would use %#lx of %#lx)", dev->memoff + size, dev->memsize);
            free(mem);
            return NULL;
        }
        mem->dma.buf = dev->membuf + dev->memoff;
        dev->memoff += size;
    }

    mem->dma.size = size;
    mem->dma.addr = dev->iovanext;
    mem->dma.mem = mem;
    mem->dev = dev;

    // add node to the memory list
    if (!dev->memlist) {
        mem->prev = mem;
        mem->next = mem;
        dev->memlist = mem;
    } else {
        mem->prev = dev->memlist->prev;
        mem->next = dev->memlist;
        dev->memlist->prev->next = mem;
        dev->memlist->prev = mem;
    }
    dev->iovanext += size;
    DEBUG_FN("%x %#lx %#lx %#lx %#lx", dev->pci, mem->dma.addr, size, dev->iovanext, dev->memoff);

    if (clear) {
    	DEBUG_FN("clearing %#x bytes of memory at %#lx", size, (u32)mem->dma.addr);
		memset((void *)(u32)mem->dma.addr, 0, size);
    }

    return mem;
}

/**
 * Free up memory.
 * @param   mem         memory pointer
 * @return  0 if ok else -1.
 */
int mem_free(mem_t* mem)
{
    mem_device_t* dev = mem->dev;

    // remove node from memory list
    if (mem->next == dev->memlist) { // If removing last item in list
        dev->iovanext -= mem->dma.size;
        dev->memoff -= mem->dma.size;
    }
    if (mem->next == mem) { // If removing only item in list
        dev->memlist = NULL;
        dev->iovanext = dev->iovabase;
        dev->memoff = 0;
    } else { // If there are other items in list
        mem->next->prev = mem->prev;
        mem->prev->next = mem->next;
        if (dev->memlist == mem) { // If first item in list
            dev->memlist = mem->next;
        }
        dev->iovanext = dev->memlist->prev->dma.addr + dev->memlist->prev->dma.size; // IOVA next is after last item in list
        dev->memoff = dev->iovanext - dev->iovabase; // Buffer offset is same as IOVA offset
    }
    DEBUG_FN("%x %#lx %#lx %#lx %#lx", dev->pci, mem->dma.addr, mem->dma.size, dev->iovanext, dev->memoff);

    free(mem);
    return 0;
}

/**
 * Map a premapped buffer and return a DMA buffer.
 * @param   dev         device context
 * @param   size        allocation size
 * @param   pmb         premapped buffer
 * @return  0 if ok else -1.
 */
mem_dma_t* mem_dma_map(mem_device_t* dev, size_t size, void* pmb)
{
    mem_t* mem = mem_alloc(dev, size, pmb, 0);
    return mem ? &mem->dma : NULL;
}

/**
 * Free a DMA buffer (without unmapping dma->buf).
 * @param   dma         memory pointer
 * @return  0 if ok else -1.
 */
int mem_dma_unmap(mem_dma_t* dma)
{
    return mem_free(dma->mem);
}

/**
 * Allocate and return a DMA buffer.
 * @param   dev         device context
 * @param   size        allocation size
 * @return  0 if ok else -1.
 */
mem_dma_t* mem_dma_alloc(mem_device_t* dev, size_t size, int clear)
{
    mem_t* mem = mem_alloc(dev, size, 0, clear);
    return mem ? &mem->dma : NULL;
}

/**
 * Free a DMA buffer.
 * @param   dma         memory pointer
 * @return  0 if ok else -1.
 */
int mem_dma_free(mem_dma_t* dma)
{
    return mem_free(dma->mem);
}

/**
 * Create a memory device context.
 * @param   dev         if NULL then allocate context
 * @param   pci         PCI device id (as %x:%x.%x format)
 * @return  device context or NULL if failure.
 */
mem_device_t* mem_create(mem_device_t* dev, int pci, u64 base_pci, void *base_mb, size_t size)
{
    DEBUG_FN("%x started", pci);

    // allocate and initialize device context
    if (!dev) dev = zalloc(sizeof(*dev));
    else dev->ext = 1;
    dev->pci = pci;
    dev->pagesize = 4096;
    dev->iovabase = base_pci;
    dev->iovanext = dev->iovabase;

	pcie_ecam_enable();

	__u8 config[256];
	pcie_ecam_read_pci(dev->pci, 0, config, sizeof(config));
	HEX_DUMP(config, sizeof(config));

	__u16* vendor = (__u16*)(config + PCI_VENDOR_ID);
	__u16* cmd = (__u16*)(config + PCI_COMMAND);

	if (*vendor == 0xffff)
		FATAL("device in bad state");

	*cmd |= PCI_COMMAND_MASTER|PCI_COMMAND_MEMORY|PCI_COMMAND_INTX_DISABLE;
	pcie_ecam_write_pci(dev->pci, PCI_COMMAND, cmd, sizeof(*cmd));
	pcie_ecam_read_pci(dev->pci, PCI_COMMAND, cmd, sizeof(*cmd));

#ifdef UNVME_DEBUG
	pcie_print_ecam_pci(dev->pci, 0x20);
#endif

	pcie_ecam_disable();

	DEBUG_FN("%x vendor=%#x cmd=%#x device=%04x rev=%d",
			 pci, *vendor, *cmd,
			 *(__u16*)(config + PCI_DEVICE_ID), config[PCI_REVISION_ID]);

    dev->membuf = base_mb;
    dev->memoff = 0;
    dev->memsize = size;

    DEBUG_FN("%x base_pci=%#x base_mb=%#x size=%#x",
			 pci, base_pci, base_mb, size);

    return (mem_device_t*)dev;
}

/**
 * Delete a memory device context.
 * @param   dev         device context
 */
void mem_delete(mem_device_t* dev)
{
    if (!dev) return;
    DEBUG_FN("%x", dev->pci);

    // free all memory associated with the device
    while (dev->memlist) mem_free(dev->memlist);

    if (!dev->ext) free(dev);
}
