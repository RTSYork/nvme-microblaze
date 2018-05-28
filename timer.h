#ifndef SRC_UNVME_TIMER_H_
#define SRC_UNVME_TIMER_H_

#include "xparameters.h"
#include "xil_types.h"
#include "xtmrctr.h"

#define TIMER_TICKS_PER_SECOND XPAR_MB_AXI_TIMER_0_CLOCK_FREQ_HZ

extern XTmrCtr timer;

static inline int timer_setup()
{
	int status = XTmrCtr_Initialize(&timer, XPAR_MB_AXI_TIMER_0_DEVICE_ID);
	if (status == XST_SUCCESS) {
		XTmrCtr_SetOptions(&timer, 0, XTC_CASCADE_MODE_OPTION);
		XTmrCtr_SetResetValue(&timer, 0, 0);
		XTmrCtr_Reset(&timer, 0);
		XTmrCtr_Start(&timer, 0);
	}
	return status;
}

static inline u64 timer_get_value() {
	u64 time1 = XTmrCtr_GetValue(&timer, 0);
	u64 time2 = XTmrCtr_GetValue(&timer, 1);
	return (time2 << 32) | time1;
}

#endif /* SRC_UNVME_TIMER_H_ */
