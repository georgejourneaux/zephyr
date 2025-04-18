#include <stddef.h>

#include "hal/debug.h"
#include "hal/swi.h"

#include <zephyr/irq.h>

#include <driverlib/interrupt.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/aon_event.h>

#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "lll.h"

static void SetPendingIRQ(unsigned int irq);
// static void ClearPendingIRQ(unsigned int irq);

extern void rtc_isr_event_ch1(const void *arg);
extern void rtc_isr_event(const void *arg);
static void hal_swi_lll_isr(const void *arg);
// static void hal_swi_ull_low_isr(const void *arg);

void hal_swi_init(void)
{
	/* RTC IRQ already connected in /zephyr/drivers/timer/cc13xx_cc26xx_rtc_timer.c */
	// IRQ_CONNECT(HAL_SWI_WORKER_IRQ, CONFIG_BT_CTLR_ULL_HIGH_PRIO, rtc_isr_event_ch1, NULL,
	// 0);
	IRQ_CONNECT(HAL_SWI_RADIO_IRQ, CONFIG_BT_CTLR_LLL_PRIO, hal_swi_lll_isr, NULL, 0);
	// IRQ_CONNECT(HAL_SWI_JOB_IRQ, CONFIG_BT_CTLR_ULL_LOW_PRIO, hal_swi_ull_low_isr, NULL, 0);

	// irq_enable(HAL_SWI_WORKER_IRQ);
	irq_enable(HAL_SWI_RADIO_IRQ);
	// irq_enable(HAL_SWI_JOB_IRQ);
}

void hal_swi_deinit(void)
{
	// irq_disable(HAL_SWI_WORKER_IRQ);
	irq_disable(HAL_SWI_RADIO_IRQ);
	// irq_disable(HAL_SWI_JOB_IRQ);
}

void hal_swi_lll_pend(void)
{
	SetPendingIRQ(HAL_SWI_RADIO_IRQ);
}

void hal_swi_worker_pend(void)
{
	SetPendingIRQ(HAL_SWI_WORKER_IRQ);
}

void hal_swi_job_pend(void)
{
	SetPendingIRQ(HAL_SWI_JOB_IRQ);
}

static void SetPendingIRQ(unsigned int irq)
{
	IntPendSet(irq + IRQ_OFFSET);
}

// static void ClearPendingIRQ(unsigned int irq)
// {
// 	IntPendClear(irq + IRQ_OFFSET);
// }

void rtc_isr_event_ch1(const void *arg)
{
	DEBUG_TICKER_ISR(1);

	ticker_trigger(0);

	DEBUG_TICKER_ISR(0);
}

void rtc_isr_event(const void *arg)
{
	mayfly_run(TICKER_USER_ID_ULL_HIGH);
}

static void hal_swi_lll_isr(const void *arg)
{
	DEBUG_RADIO_ISR(1);

	mayfly_run(TICKER_USER_ID_LLL);

	DEBUG_RADIO_ISR(0);
}

// void hal_swi_ull_low_isr(const void *arg)
// {
// 	DEBUG_TICKER_JOB(1);

// 	mayfly_run(TICKER_USER_ID_ULL_LOW);

// 	DEBUG_TICKER_JOB(0);
// }