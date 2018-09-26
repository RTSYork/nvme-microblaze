#ifndef SRC_PARAMS_H_
#define SRC_PARAMS_H_

//#define MEM_BASE_MB  0x000000000
//#define MEM_BASE_PCI 0x500000000
//#define MEM_SIZE     0x080000000
#define MEM_BASE_MB  0xb0000000
#define MEM_BASE_PCI 0xb0000000
#define MEM_SIZE     0x10000000

#define NSID 1
#define PCI_DEV 0x010000
#define PCI_DEV_NAME "01:00.0"

#define UNVME_TIMEOUT   60          ///< default timeout in seconds
#define UNVME_AQSIZE    8           ///< admin queue size
#define UNVME_QSIZE     256         ///< default I/O queue size
#define UNVME_QCOUNT    1
#define UNVME_DESCS     16
#define UNVME_IOMEMS    256

#endif // SRC_PARAMS_H_
