#include <soc.h>

#include "util/memq.h"
#include "util/mayfly.h"

#include "ll_sw/lll.h"

#include "hal/cc13xx_cc26xx/swi.h"
#include "hal/debug.h"

void mayfly_enable_cb(uint8_t caller_id, uint8_t callee_id, uint8_t enable)
{
#warning "TODO"
}

uint32_t mayfly_is_enabled(uint8_t caller_id, uint8_t callee_id)
{
#warning "TODO"
	return 0;
}

uint32_t mayfly_prio_is_equal(uint8_t caller_id, uint8_t callee_id)
{
#warning "TODO"
	return 0;
}

void mayfly_pend(uint8_t caller_id, uint8_t callee_id)
{
#warning "TODO"
}

uint32_t mayfly_is_running(void)
{
#warning "TODO"
	return 0;
}
