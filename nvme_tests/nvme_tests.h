#ifndef SRC_NVME_TESTS_H_
#define SRC_NVME_TESTS_H_

#include "xil_types.h"

int nvme_identify(int pci);
int nvme_get_features(int pci);
int nvme_get_log_page(int pci, int lid, int nsid);

#endif /* SRC_NVME_TESTS_H_ */

