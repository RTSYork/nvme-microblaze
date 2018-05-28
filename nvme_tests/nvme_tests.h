#ifndef SRC_NVME_TESTS_H_
#define SRC_NVME_TESTS_H_

#include "xil_types.h"

#define errx(code, fmt, arg...) do { printf("error: " fmt "\n\r", ##arg); exit(code); } while (0)

int nvme_identify(int pci, u64 mem_base_pci, void *mem_base_mb, size_t mem_size);
int nvme_get_features(int pci, u64 mem_base_pci, void *mem_base_mb, size_t mem_size);
int nvme_get_log_page(int pci, int lid, int nsid, u64 mem_base_pci, void *mem_base_mb, size_t mem_size);

#endif /* SRC_NVME_TESTS_H_ */

