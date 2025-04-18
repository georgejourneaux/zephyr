#ifndef BT_CTLR_HAL_CPU_H_
#define BT_CTLR_HAL_CPU_H_

#include <zephyr/kernel.h>

static inline void cpu_sleep(void)
{
	__WFE();
	/* __SEV(); */
	__WFE();
}

static inline void cpu_dmb(void)
{
	__asm__ volatile("" : : : "memory");
}

#endif /* BT_CTLR_HAL_CPU_H_ */