#include <stdio.h>
#include <sleep.h>
#include "platform.h"
#include "timer.h"
#include "pcie.h"
#include "unvme_tests/unvme_tests.h"
#include "nvme_tests/nvme_tests.h"
#include "benchmark.h"

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

//	printf("\r\nRunning UNVMe tests...\r\n");
//	unvme_info("01:00.0");
//	unvme_get_features("01:00.0");
//	unvme_sim_test(0, 1024 * 1024, "01:00.0");
//	unvme_api_test(0, 1, "01:00.0");
//	unvme_lat_test(0, 0, 0, "01:00.0");
//	printf("\r\nDone\n\r");
//
//	printf("\r\nRunning NVMe tests...\r\n");
//	nvme_identify("01:00.0");
//	nvme_get_features("01:00.0");
//	nvme_get_log_page("01:00.0", 1, 1); // error information
//	nvme_get_log_page("01:00.0", 2, -1); // SMART / Health information (device doesn't respond)
//	nvme_get_log_page("01:00.0", 3, 1); // firmware slot information
//	printf("\r\nDone\n\r");

//	unvme_wrc("01:00.0", 'w', 0, 0, 0, 0x1000000, 0, 0, 0, 0);
//	unvme_wrc("01:00.0", 'r', 0, 0, 0, 0x1000, 0, 0, 0, 0);
//	unvme_wrc("01:00.0", 'r', 0, 0, 0, 0x10000, 0, 0, 0, 0);
//	unvme_wrc("01:00.0", 'r', 0, 0, 0, 0x100000, 0, 0, 0, 0);
//	unvme_wrc("01:00.0", 'r', 0, 0, 0, 0x1000000, 0, 0, 0, 10);
//	unvme_wrc("01:00.0", 'r', 0, 1, 0, 0x100000, 0, 0, 0, 10);

	read_benchmark();
	write_benchmark();

	cleanup_platform();
	return 0;
}
