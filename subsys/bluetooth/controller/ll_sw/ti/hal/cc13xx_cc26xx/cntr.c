#include <stdint.h>

#include "hal/cntr.h"
#include "hal/debug.h"
#include "hal/swi.h"

#include <zephyr/irq.h>

#include <ti/drivers/rf/RF.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/aon_event.h>

#define RF_SCALE_RTC_TO_4MHZ 4000000

static uint8_t refcount;

static uint32_t RF_convertRatToRtc(uint32_t rat);

void cntr_init(void)
{
	irq_disable(HAL_SWI_WORKER_IRQ);

	AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC0);
	AONRTCCombinedEventConfig(AON_RTC_CH0 | AON_RTC_CH1);

	irq_enable(HAL_SWI_WORKER_IRQ);
}

uint32_t cntr_start(void)
{
	if (refcount++) {
		return 1;
	}

	AONRTCChannelEnable(AON_RTC_CH1);

	return 0;
}

uint32_t cntr_stop(void)
{
	LL_ASSERT(refcount);

	if (--refcount) {
		return 1;
	}

	AONRTCChannelDisable(AON_RTC_CH1);

	return 0;
}

uint32_t cntr_cnt_get(void)
{
	return RF_getCurrentTime();
}

void cntr_cmp_set(uint8_t cmp, uint32_t value)
{
	ARG_UNUSED(cmp);

	uint32_t now_rat = RF_getCurrentTime();
	uint32_t then_rat = value;
	uint32_t dt_rat = then_rat - now_rat;

	uint32_t now_rtc = AONRTCCurrentCompareValueGet();
	uint32_t then_rtc = now_rtc + RF_convertRatToRtc(dt_rat);

	AONRTCChannelDisable(AON_RTC_CH1);
	AONRTCEventClear(AON_RTC_CH1);
	AONRTCModeCh1Set(AON_RTC_MODE_CH1_COMPARE);
	AONRTCCompareValueSet(AON_RTC_CH1, then_rtc);
	AONRTCChannelEnable(AON_RTC_CH1);
}

static uint32_t RF_convertRatToRtc(uint32_t rat)
{
	uint64_t nCurrentTime = rat;
	uint8_t carry = 0;

	nCurrentTime <<= 32;
	nCurrentTime /= RF_SCALE_RTC_TO_4MHZ;

	/* round up */
	if ((nCurrentTime & 0xffff) >= 0x8000) {
		carry = 1;
	}

	/* convert to RTC comparator format */
	nCurrentTime >>= 16;

	return (uint32_t)nCurrentTime + carry;
}
