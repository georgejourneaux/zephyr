#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "hal/cntr.h"

#include "util/memq.h"
#include "util/mayfly.h"

#include "ticker/ticker.h"

#include "ll_sw/lll.h"

uint8_t hal_ticker_instance0_caller_id_get(uint8_t user_id)
{
#warning "TODO"
	return 0;
}

void hal_ticker_instance0_sched(uint8_t caller_id, uint8_t callee_id, uint8_t chain, void *instance)
{
#warning "TODO"
}

void hal_ticker_instance0_trigger_set(uint32_t value)
{
#warning "TODO"
}
