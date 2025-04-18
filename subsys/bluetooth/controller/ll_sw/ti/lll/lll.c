#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/toolchain.h>

#include <soc.h>
#include <zephyr/device.h>

#include <zephyr/drivers/entropy.h>

#include "hal/radio.h"
#include "hal/swi.h"

#include "util/mem.h"
#include "util/memq.h"

#include "lll.h"

static const struct device *const device_entropy = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

int lll_init(void)
{
	if (!device_is_ready(device_entropy)) {
		return -ENODEV;
	}

	hal_swi_init();

	return hal_radio_init();
}

int lll_deinit(void)
{
#warning "TODO"
	return -1;
}

int lll_csrand_get(void *buf, size_t len)
{
#warning "TODO"
	return -1;
}

int lll_csrand_isr_get(void *buf, size_t len)
{
#warning "TODO"
	return -1;
}

int lll_rand_get(void *buf, size_t len)
{
#warning "TODO"
	return -1;
}

int lll_rand_isr_get(void *buf, size_t len)
{
#warning "TODO"
	return -1;
}

int lll_reset(void)
{
#warning "TODO"
	return -1;
}

void lll_disable(void *param)
{
#warning "TODO"
}

uint32_t lll_radio_is_idle(void)
{
#warning "TODO"
	return 1;
}

uint32_t lll_radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
#warning "TODO"
	return 0;
}

uint32_t lll_radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
#warning "TODO"
	return 0;
}

int8_t lll_radio_tx_pwr_min_get(void)
{
#warning "TODO"
	return RADIO_TXP_DEFAULT;
}

int8_t lll_radio_tx_pwr_max_get(void)
{
#warning "TODO"
	return RADIO_TXP_DEFAULT;
}

int8_t lll_radio_tx_pwr_floor(int8_t tx_pwr_lvl)
{
#warning "TODO"
	return RADIO_TXP_DEFAULT;
}

int lll_prepare_resolve(lll_is_abort_cb_t is_abort_cb, lll_abort_cb_t abort_cb,
			lll_prepare_cb_t prepare_cb, struct lll_prepare_param *prepare_param,
			uint8_t is_resume, uint8_t is_dequeue)
{
#warning "TODO"
	return -1;
}