#include <stdio.h>
#include <sleep.h>
#include "platform.h"
#include "mb_interface.h"
#include "pvr.h"
#include "timer.h"
#include "pcie.h"
#include "unvme_tests/unvme_tests.h"
#include "nvme_tests/nvme_tests.h"
#include "benchmark.h"
#include "params.h"

XTmrCtr timer;

int main()
{
	init_platform();

	printf("\n\r***********************************************************\n\r");
	printf("Simple NVMe test\n\r");

//	pvr_t pvr;
//	microblaze_get_pvr(&pvr);
//	u32 use_mmu = MICROBLAZE_PVR_USE_MMU(pvr);
//	u32 mmu_type = MICROBLAZE_PVR_MMU_TYPE(pvr);
//	u32 msr = mfmsr();
//	printf("use_mmu = 0x%08lx, mmu_type = 0x%08lx, msr = 0x%08lx\r\n", use_mmu, mmu_type, msr);
//	return 0;

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
	if (unvme_info()) goto error;
	if (unvme_get_features()) goto error;
	if (unvme_sim_test(0, 1024 * 1024)) goto error;
//	if (unvme_api_test(0, 1)) goto error;
//	if (unvme_lat_test(0, 0, 0)) goto error;
	printf("\r\nDone\n\r");

	printf("\r\nRunning NVMe tests...\r\n");
	if (nvme_identify()) goto error;
	if (nvme_get_features()) goto error;
	if (nvme_get_log_page(1, 1)) goto error; // error information
	if (nvme_get_log_page(2, -1)) goto error; // SMART / Health information
	if (nvme_get_log_page(3, 1)) goto error; // firmware slot information
	printf("\r\nDone\n\r");

////	if (unvme_wrc('w', 0, 0, 0, 0x1000000, 0, 0, 0, 0)) goto error;
////	if (unvme_wrc('r', 0, 0, 0, 0x1000, 0, 0, 0, 0)) goto error;
////	if (unvme_wrc('r', 0, 0, 0, 0x10000, 0, 0, 0, 0)) goto error;
////	if (unvme_wrc('r', 0, 0, 0, 0x100000, 0, 0, 0, 0)) goto error;
////	if (unvme_wrc('r', 0, 0, 0, 0x1000000, 0, 0, 0, 10)) goto error;
////	if (unvme_wrc('r', 0, 1, 0, 0x100000, 0, 0, 0, 10)) goto error;

//	if (unvme_wrc('w', 0, 1, 0, 0x10000, 0, 0, 0, 2)) goto error;
//	if (unvme_wrc('r', 0, 1, 0, 0x10000, 0, 0, 0, 2)) goto error;
//	if (unvme_wrc('w', 0, 1, 0, 0x10000, 1, 16, 0, 2)) goto error;
//	if (unvme_wrc('r', 0, 1, 0, 0x10000, 1, 16, 0, 2)) goto error;

	if (read_benchmark()) goto error;
	if (write_benchmark()) goto error;

	cleanup_platform();
	return 0;

error:
	printf("\r\n\n****** ERROR ******\r\n\n");
	return 1;
}
