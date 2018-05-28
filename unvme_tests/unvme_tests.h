#ifndef SRC_UNVME_TESTS_H_
#define SRC_UNVME_TESTS_H_

#include "xil_types.h"

int unvme_info(char* dev);
int unvme_get_features(char* dev);
int unvme_sim_test(u64 lba, u64 size, char* pciname);
int unvme_api_test(int verbose, int ratio, char* pciname);
int unvme_lat_test(int runtime_in, int qcount_in, int qsize_in, char* pciname);
int unvme_wrc(const char* pciname, u32 rw_in, u64 pattern_in, u64 patinc_in, u64 startlba_in, u64 lbacount_in, u32 qcount_in, u32 qdepth_in, u32 nbpio_in, u64 dumptime_in);

#endif /* SRC_UNVME_TESTS_H_ */
