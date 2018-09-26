#ifndef SRC_NVME_TESTS_H_
#define SRC_NVME_TESTS_H_

#include "xil_types.h"

#define errx(code, fmt, arg...) do { printf("error: " fmt "\n\r", ##arg); exit(code); } while (0)

int nvme_identify(void);
int nvme_get_features(void);
int nvme_get_log_page(int lid, int nsid);

#endif /* SRC_NVME_TESTS_H_ */

