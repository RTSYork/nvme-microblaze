#include <stdio.h>
#include <string.h>
#include "xil_io.h"
#include "sleep.h"
#include "pcie.h"

int pcie_setup()
{
	u32 val;

	// Assert reset (MIO31)
	Xil_Out32LE(GPIO + MASK_DATA_1_LSW, 0xFFDF0000);

	// Note: set-up for Root Port mode performed by the FSBL
	// Details in Ch.30 of Zynq US+ TRM

	// Map bridge register aperture
	val = Xil_In32LE(AXIPCIE_MAIN + E_BREG_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_BREG_CONTROL, val | 0x00030000); // 32KB of translation (TRM says use 64KB) (breg_size == 3)
	Xil_Out32LE(AXIPCIE_MAIN + E_BREG_BASE_LO, 0xFD0E0000);
	Xil_Out32LE(AXIPCIE_MAIN + E_BREG_BASE_HI, 0x00000000);

	// Map ECAM space
	val = Xil_In32LE(AXIPCIE_MAIN + E_ECAM_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_ECAM_CONTROL, (val & 0xFFE0FFFF) | 0x000C0000); // 16MB of translation (ecam_size == 0xC)
	Xil_Out32LE(AXIPCIE_MAIN + E_ECAM_BASE_LO, 0xE0000000);
	Xil_Out32LE(AXIPCIE_MAIN + E_ECAM_BASE_HI, 0x00000000);
	val = Xil_In32LE(AXIPCIE_MAIN + E_ECAM_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_ECAM_CONTROL, val | 0x00000001); // ecam_enable = 1

	// Map DMA register aperture
	val = Xil_In32LE(AXIPCIE_MAIN + E_DREG_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_DREG_CONTROL, (val & 0xFFFCFFFF)); // 4KB of translation (TRM says use 512B) (dma_size == 0)
	Xil_Out32LE(AXIPCIE_MAIN + E_DREG_BASE_LO, 0xFD0F0000);
	Xil_Out32LE(AXIPCIE_MAIN + E_DREG_BASE_HI, 0x00000000);

	// Setup ingress MSI aperture
	Xil_Out32LE(AXIPCIE_MAIN + I_MSII_BASE_LO, 0xFE440000);
	val = Xil_In32LE(AXIPCIE_MAIN + I_MSII_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + I_MSII_CONTROL, val | 0x00000001); // i_msii_enable = 1

	// Disable DMA register access from Endpoint
	val = Xil_In32LE(AXIPCIE_MAIN + BRIDGE_CORE_CFG_PCIE_RX0);
	Xil_Out32LE(AXIPCIE_MAIN + BRIDGE_CORE_CFG_PCIE_RX0, val | 0x00020007); // cfg_disable_pcie_dma_reg_access = 1, cfg_dma_reg_bar = 7

	// Allow all upstream transactions to access the AXI interface without translation
	Xil_Out32LE(AXIPCIE_MAIN + I_ISUB_CONTROL, 0x00000001);

	// Enable translation apertures in the bridge for access by the AXI processor
	val = Xil_In32LE(AXIPCIE_MAIN + E_BREG_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_BREG_CONTROL, (val & 0xFFFFFFFD) | 0x00000001); // breg_enable_force = 0, breg_enable = 1

	// Set-up Xilinx PCI/AXI bridge ECAM (copied from Linux - would be better to look these up properly)
	pcie_set_ecam_reg(0, 0, 0, 0x04, 0x00100006);
	pcie_set_ecam_reg(0, 0, 0, 0x04, 0x00100006);
	pcie_set_ecam_reg(0, 0, 0, 0x18, 0x000c0100);
//	pcie_set_ecam_reg(0, 0, 0, 0x1C, 0x20000000);
	pcie_set_ecam_reg(0, 0, 0, 0x20, 0xe000e000);
	pcie_set_ecam_reg(0, 0, 0, 0x24, 0x0001fff1);
	pcie_set_ecam_reg(0, 0, 0, 0x7c, 0x00010010);

	// Release reset (MIO31)
	Xil_Out32LE(GPIO + MASK_DATA_1_LSW, 0xFFDF0020);

	// Delay for a short while after reset
	sleep(1);

#if PCIE_DEBUG
	printf("\r\nPCIE_ATTRIB:\r\n");
	print_attrs();

	val = Xil_In32LE(PCIE_ATTRIB + PCIE_STATUS);
	printf("\r\nPCIE_STATUS: 0x%08lx\r\n", val);

	val = Xil_In32LE(PCIE_ATTRIB + ISR);
	printf("\r\nISR: 0x%08lx\r\n", val);

	val = Xil_In32LE(CRF_APB + RST_FPD_TOP);
	printf("\r\nRST_FPD_TOP: 0x%08lx\r\n", val);

	printf("\r\nE_BREG_CAPABILITIES:\r\n");
	pcie_hexdump((void *)(AXIPCIE_MAIN + E_BREG_CAPABILITIES), 32);

	printf("\r\nE_ECAM_CAPABILITIES:\r\n");
	pcie_hexdump((void *)(AXIPCIE_MAIN + E_ECAM_CAPABILITIES), 32);

	printf("\r\nE_DREG_CAPABILITIES:\r\n");
	pcie_hexdump((void *)(AXIPCIE_MAIN + E_DREG_CAPABILITIES), 32);
#endif

	return 0;
}

void pcie_ecam_enable()
{
	u32 val = Xil_In32LE(AXIPCIE_MAIN + E_ECAM_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_ECAM_CONTROL, val | 0x00000001); // ecam_enable = 1
}

void pcie_ecam_disable()
{
	u32 val = Xil_In32LE(AXIPCIE_MAIN + E_ECAM_CONTROL);
	Xil_Out32LE(AXIPCIE_MAIN + E_ECAM_CONTROL, val & ~0x00000001); // ecam_enable = 0
}

void pcie_hexdump(void* buf, int len)
{
	u32* data = (u32*)buf;
	u32 line[8];
	u8 c;
	for (int j = 0; j < len/4; j+=8) {
		printf("\r\n%08lx: ", (u32)data + j*4);
		for (int i = 0; i < 8; i++) {
			line[i] = data[j + i];
			printf("%08lx ", line[i]);
		}
		for (int i = 0; i < 8; i++) {
			for (int off = 0; off < 32; off+=8) {
				c = line[i] >> off;
				if (c >= ' ' && c <= '~')
					printf("%c", c);
				else
					printf(".");
			}
		}
	}
	printf("\r\n");
}

void pcie_print_attrs() {
	for (int i = 0; i < 110; i++) {
		u32 val = Xil_In32LE(PCIE_ATTRIB + i*4);
		printf("ATTR%-3d 0x%08lx\r\n", i, val);
	}
}

void pcie_print_ecam(u32 bus, u32 device, u32 function, int length)
{
	void *ecam = (void *)(ECAM_BASE + (bus << 20) + (device << 15) + (function << 12));
	printf("\r\nECAM %02lu:%02lu.%01lu\r\n", bus, device, function);
	pcie_hexdump(ecam, length);
}

void pcie_print_ecam_pci(u32 pci, int length)
{
	pcie_print_ecam(pci >> 16, (pci >> 8) & 0xff, pci & 0xff, length);
}

void pcie_set_ecam_reg(u32 bus, u32 device, u32 function, u32 offset, u32 value)
{
	u32 addr = ECAM_BASE + (bus << 20) + (device << 15) + (function << 12) + offset;
	Xil_Out32LE(addr, value);
}

void pcie_set_ecam_reg_pci(u32 pci, u32 offset, u32 value)
{
	pcie_set_ecam_reg(pci >> 16, (pci >> 8) & 0xff, pci & 0xff, offset, value);
}

void pcie_ecam_write(u32 bus, u32 device, u32 function, u32 offset, void *buffer, size_t length)
{
	void *ecam = (void *)(ECAM_BASE + (bus << 20) + (device << 15) + (function << 12) + offset);
	memcpy(ecam, buffer, length);
}

void pcie_ecam_write_pci(u32 pci, u32 offset, void *buffer, size_t length)
{
	pcie_ecam_write(pci >> 16, (pci >> 8) & 0xff, pci & 0xff, offset, buffer, length);
}

void pcie_ecam_read(u32 bus, u32 device, u32 function, u32 offset, void *buffer, size_t length)
{
	void *ecam = (void *)(ECAM_BASE + (bus << 20) + (device << 15) + (function << 12) + offset);
	memcpy(buffer, ecam, length);
}

void pcie_ecam_read_pci(u32 pci, u32 offset, void *buffer, size_t length)
{
	pcie_ecam_read(pci >> 16, (pci >> 8) & 0xff, pci & 0xff, offset, buffer, length);
}
