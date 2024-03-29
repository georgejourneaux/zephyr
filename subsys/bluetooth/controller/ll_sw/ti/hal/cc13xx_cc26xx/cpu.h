/*
 * Copyright (c) 2016-2021 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

static inline void cpu_sleep(void)
{
	__WFE();
	/* __SEV(); */
	__WFE();
}

static inline void cpu_dmb(void)
{
	__asm__ volatile ("" : : : "memory");
}