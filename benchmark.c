#include <stdio.h>
#include <string.h>
#include "unvme/unvme.h"
#include "timer.h"

int read_benchmark(int pci, int nsid, u64 mem_base_pci, void *mem_base_mb, size_t mem_size) {
	printf("\r\nRead benchmark running\r\n");

	void *buf;
	u64 start, end;
	double time, bps;
	u64 blocks = 0x5d27215;
	u64 blocks_per_io;
	u64 size, size_per_io;

    unvme_device_t dev;
	if (unvme_openq(&dev, pci, nsid, mem_base_pci, mem_base_mb, mem_size)) {
		printf("\r\nError opening device\r\n");
		return 1;
	};

	blocks_per_io = dev.nsio.maxbpio * dev.nsio.maxiopq;
	size = dev.nsio.blocksize * blocks;
	size_per_io = dev.nsio.blocksize * blocks_per_io;

	buf = unvme_alloc(&dev, size_per_io);
	if (!buf) {
		printf("\r\nError allocating memory\r\n");
		unvme_close(&dev);
		return 1;
	};

	start = timer_get_value();
	for (int i = 0; i < blocks; i += blocks_per_io) {
		if (i + blocks_per_io <= blocks)
			unvme_read(&dev, 0, buf, i, blocks_per_io);
		else
			unvme_read(&dev, 0, buf, i, blocks - i);
	}
	end = timer_get_value();

	time = (double)(end - start) / (double)TIMER_TICKS_PER_SECOND;

	bps = size / time / (1024.0 * 1024.0);

	printf("Reading %llu bytes took %.3lf seconds - %.3lf MiB/s\r\n", size, time, bps);

	unvme_free(&dev, buf);

	unvme_close(&dev);

	printf("\r\nRead benchmark done.\r\n");

	return 0;
}

int write_benchmark(int pci, int nsid, u64 mem_base_pci, void *mem_base_mb, size_t mem_size) {
	printf("\r\nWrite benchmark running\r\n");

	void *buf;
	u64 start, end;
	double time, bps;
	u64 blocks = 0x5d27215;
	u64 blocks_per_io;
	u64 size, size_per_io;

    unvme_device_t dev;
	if (unvme_openq(&dev, pci, nsid, mem_base_pci, mem_base_mb, mem_size)) {
		printf("\r\nError opening device\r\n");
		return 1;
	};

	blocks_per_io = dev.nsio.maxbpio * dev.nsio.maxiopq;
	size = dev.nsio.blocksize * blocks;
	size_per_io = dev.nsio.blocksize * blocks_per_io;

	buf = unvme_alloc(&dev, size_per_io);
	if (!buf) {
		printf("\r\nError allocating memory\r\n");
		unvme_close(&dev);
		return 1;
	};

	start = timer_get_value();
	for (int i = 0; i < blocks; i += blocks_per_io) {
		if (i + blocks_per_io <= blocks)
			unvme_write(&dev, 0, buf, i, blocks_per_io);
		else
			unvme_write(&dev, 0, buf, i, blocks - i);
	}
	end = timer_get_value();

	time = (double)(end - start) / (double)TIMER_TICKS_PER_SECOND;

	bps = size / time / (1024.0 * 1024.0);

	printf("Writing %llu bytes took %.3lf seconds - %.3lf MiB/s\r\n", size, time, bps);

	unvme_free(&dev, buf);

	unvme_close(&dev);

	printf("\r\nWrite benchmark done.\r\n");
	return 0;
}
