#ifndef SRC_PCIE_H_
#define SRC_PCIE_H_

#include "xil_types.h"

#define CRF_APB				0xFD1A0000
#define RST_FPD_TOP			0x00000100

#define AXIPCIE_MAIN			0xFD0E0000
#define BRIDGE_CORE_CFG_PCIE_RX0	0x00000000
#define E_BREG_CAPABILITIES		0x00000200
#define E_BREG_STATUS			0x00000204
#define E_BREG_CONTROL			0x00000208
#define E_BREG_BASE_LO			0x00000210
#define E_BREG_BASE_HI			0x00000214
#define E_ECAM_CAPABILITIES		0x00000220
#define E_ECAM_STATUS			0x00000224
#define E_ECAM_CONTROL			0x00000228
#define E_ECAM_BASE_LO			0x00000230
#define E_ECAM_BASE_HI			0x00000234
#define E_DREG_CAPABILITIES		0x00000280
#define E_DREG_STATUS			0x00000284
#define E_DREG_CONTROL			0x00000288
#define E_DREG_BASE_LO			0x00000290
#define E_DREG_BASE_HI			0x00000294
#define I_MSII_CAPABILITIES		0x00000300
#define I_MSII_CONTROL			0x00000308
#define I_MSII_BASE_LO			0x00000310
#define I_MSII_BASE_HI			0x00000314
#define I_ISUB_CAPABILITIES		0x000003E0
#define I_ISUB_STATUS			0x000003E4
#define I_ISUB_CONTROL			0x000003E8

#define PCIE_ATTRIB			0xFD480000
#define ATTR_0				0x00000000
#define ATTR_1				0x00000004
#define ATTR_2				0x00000008
#define ATTR_3				0x0000000C
#define ATTR_4				0x00000010
#define ATTR_5				0x00000014
#define ATTR_6				0x00000018
#define ATTR_7				0x0000001C
#define ATTR_8				0x00000020
#define ATTR_9				0x00000024
#define ATTR_10				0x00000028
#define PCIE_STATUS			0x00000238
#define RP_CTRL				0x00000234
#define ISR				0x00000304

#define GPIO				0xFF0A0000
#define MASK_DATA_1_LSW			0x00000008
#define DATA_1_RO			0x00000064

#define ECAM_BASE			0xE0000000

#define PCIE_DEBUG 0


int pcie_setup();
void pcie_ecam_enable();
void pcie_ecam_disable();
void pcie_hexdump(void* buf, int len);
void pcie_print_attrs();
void pcie_print_ecam(u32 bus, u32 device, u32 function, int length);
void pcie_print_ecam_pci(u32 pci, int length);
void pcie_set_ecam_reg(u32 bus, u32 device, u32 function, u32 offset, u32 value);
void pcie_set_ecam_reg_pci(u32 pci, u32 offset, u32 value);
void pcie_ecam_write(u32 bus, u32 device, u32 function, u32 offset, void *buffer, size_t length);
void pcie_ecam_write_pci(u32 pci, u32 offset, void *buffer, size_t length);
void pcie_ecam_read(u32 bus, u32 device, u32 function, u32 offset, void *buffer, size_t length);
void pcie_ecam_read_pci(u32 pci, u32 offset, void *buffer, size_t length);

#endif /* SRC_PCIE_H_ */
