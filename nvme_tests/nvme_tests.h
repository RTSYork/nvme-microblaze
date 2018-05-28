#ifndef SRC_NVME_TESTS_H_
#define SRC_NVME_TESTS_H_

#include "xil_types.h"

int nvme_identify(char* dev);
int nvme_get_features(char* dev);
int nvme_get_log_page(char* dev, int lid, int nsid);

#endif /* SRC_NVME_TESTS_H_ */

