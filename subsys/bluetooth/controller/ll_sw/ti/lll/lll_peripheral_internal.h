#ifndef LLL_PERIPHERAL_INTERNAL_H
#define LLL_PERIPHERAL_INTERNAL_H

#include <driverlib/rfc.h>

void lll_isr_peripheral(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);

#endif /* LLL_PERIPHERAL_INTERNAL_H */