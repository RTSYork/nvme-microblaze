#include <stdio.h>
#include <string.h>
#include "unvme/unvme.h"
#include "timer.h"

void read_benchmark() {
	printf("\r\nRead benchmark running\r\n");

	void *buf;
	const unvme_ns_t *ns;
	u64 start, end;
	double time, bps;
	u64 blocks = 0x5d27215;
	u64 blocks_per_io;
	u64 size, size_per_io;

	ns = unvme_openq(0x010000, 1, 1, 1024);

	blocks_per_io = ns->maxbpio * ns->maxiopq;
	size = ns->blocksize * blocks;
	size_per_io = ns->blocksize * blocks_per_io;

	buf = unvme_alloc(ns, size_per_io);

	start = timer_get_value();
	for (int i = 0; i < blocks; i += blocks_per_io) {
		if (i + blocks_per_io <= blocks)
			unvme_read(ns, 0, buf, i, blocks_per_io);
		else
			unvme_read(ns, 0, buf, i, blocks - i);
	}
	end = timer_get_value();

	time = (double)(end - start) / (double)TIMER_TICKS_PER_SECOND;

	bps = size / time / (1024.0 * 1024.0);

	printf("Reading %llu bytes took %.3lf seconds - %.3lf MiB/s\r\n", size, time, bps);

	unvme_free(ns, buf);

	unvme_close(ns);

	printf("\r\nRead benchmark done.\r\n");
}

void write_benchmark() {
	printf("\r\nWrite benchmark running\r\n");

	void *buf;
	const unvme_ns_t *ns;
	u64 start, end;
	double time, bps;
	u64 blocks = 0x5d27215;
	u64 blocks_per_io;
	u64 size, size_per_io;

	ns = unvme_openq(0x010000, 1, 1, 1024);

	blocks_per_io = ns->maxbpio * ns->maxiopq;
	size = ns->blocksize * blocks;
	size_per_io = ns->blocksize * blocks_per_io;

	buf = unvme_alloc(ns, size_per_io);

	start = timer_get_value();
	for (int i = 0; i < blocks; i += blocks_per_io) {
		if (i + blocks_per_io <= blocks)
			unvme_write(ns, 0, buf, i, blocks_per_io);
		else
			unvme_write(ns, 0, buf, i, blocks - i);
	}
	end = timer_get_value();

	time = (double)(end - start) / (double)TIMER_TICKS_PER_SECOND;

	bps = size / time / (1024.0 * 1024.0);

	printf("Writing %llu bytes took %.3lf seconds - %.3lf MiB/s\r\n", size, time, bps);

	unvme_free(ns, buf);

	unvme_close(ns);

	printf("\r\nWrite benchmark done.\r\n");
}
