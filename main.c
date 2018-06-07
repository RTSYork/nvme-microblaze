#include <stdio.h>
#include <sleep.h>
#include "platform.h"
#include "timer.h"
#include "pcie.h"
#include "unvme_tests/unvme_tests.h"
#include "nvme_tests/nvme_tests.h"
#include "benchmark.h"

#define MEM_BASE_MB (void *)0x00000000
#define MEM_BASE_PCI 0x500000000
#define MEM_SIZE 0x80000000

XTmrCtr timer;

int main()
{
	init_platform();

	printf("\n\r***********************************************************\n\r");
	printf("NVMe test\n\r");

	pcie_setup();

	printf("\n\rPCIe setup done\n\r");

	pcie_set_ecam_reg(1, 0, 0, 0x10, 0xe0010004); // MLBAR[BA] = 0xe0010000
//	pcie_set_ecam_reg(1, 0, 0, 0x04, 0x00100002); // CMD[MSE] = 1

//	pcie_print_ecam(0, 0, 0, 0x100);
//	pcie_print_ecam(1, 0, 0, 0x100);

	sleep(1);

	printf("\r\nSetting up timer...\r\n");

	timer_setup();

//	while(1) {
//		u64 time = timer_get_value();
//		printf("Time: %llu / %llu\r\n", time, time / TIMER_TICKS_PER_SECOND);
//		sleep(1);
//	}

	printf("\r\nRunning UNVMe tests...\r\n");
	if (unvme_info(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	if (unvme_get_features(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	if (unvme_sim_test(0, 1024 * 1024, 0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	if (unvme_api_test(0, 1, 0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	if (unvme_lat_test(0, 0, 0, 0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	printf("\r\nDone\n\r");

	printf("\r\nRunning NVMe tests...\r\n");
	if (nvme_identify(0x010000, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	if (nvme_get_features(0x010000, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error;
	if (nvme_get_log_page(0x010000, 1, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error; // error information
	if (nvme_get_log_page(0x010000, 2, -1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error; // SMART / Health information (device doesn't respond)
	if (nvme_get_log_page(0x010000, 3, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE)) goto error; // firmware slot information
	printf("\r\nDone\n\r");

//	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'w', 0, 0, 0, 0x1000000, 0, 0, 0, 0)) goto error;
//	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'r', 0, 0, 0, 0x1000, 0, 0, 0, 0)) goto error;
//	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'r', 0, 0, 0, 0x10000, 0, 0, 0, 0)) goto error;
//	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'r', 0, 0, 0, 0x100000, 0, 0, 0, 0)) goto error;
//	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'r', 0, 0, 0, 0x1000000, 0, 0, 0, 10)) goto error;
//	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'r', 0, 1, 0, 0x100000, 0, 0, 0, 10)) goto error;

	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'w', 0, 1, 0, 0x10000, 0, 0, 0, 2)) goto error;
	if (unvme_wrc(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE, 'r', 0, 0, 0, 0x10000, 0, 0, 0, 2)) goto error;

	read_benchmark(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE);
	write_benchmark(0x010000, 1, MEM_BASE_PCI, MEM_BASE_MB, MEM_SIZE);

	cleanup_platform();
	return 0;

error:
	printf("\r\n\n****** ERROR ******\r\n\n");
	return 1;
}
