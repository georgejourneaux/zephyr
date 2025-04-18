#ifndef BT_CTLR_HAL_SWI_H_
#define BT_CTLR_HAL_SWI_H_

#include <inc/hw_ints.h>

#define IRQ_OFFSET (16)

#define HAL_RADIO_IRQ (INT_RFC_HW_COMB - IRQ_OFFSET)

#define HAL_SWI_RADIO_IRQ  (INT_I2C_IRQ - IRQ_OFFSET)
#define HAL_SWI_WORKER_IRQ (INT_AON_RTC_COMB - IRQ_OFFSET)
#define HAL_SWI_JOB_IRQ    (HAL_SWI_WORKER_IRQ)

void hal_swi_init(void);
void hal_swi_deinit(void);
void hal_swi_lll_pend(void);
void hal_swi_worker_pend(void);
void hal_swi_job_pend(void);

#endif /* BT_CTLR_HAL_SWI_H_ */