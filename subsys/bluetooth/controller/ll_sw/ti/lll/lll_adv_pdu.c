#include <stdbool.h>
#include <stdint.h>

#include "hal/ccm.h"
#include "hal/cpu.h"
#include "hal/debug.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mfifo.h"
#include "util/util.h"

#include "lll/pdu_vendor.h"
#include "ll_sw/pdu.h"

#include "ll_sw/lll.h"
#include "lll_adv_types.h"
#include "ll_sw/lll_adv.h"
#include "lll/lll_adv_pdu.h"

int lll_adv_data_init(struct lll_adv_pdu *pdu)
{
#warning "TODO"
	return -1;
}

int lll_adv_data_reset(struct lll_adv_pdu *pdu)
{
#warning "TODO"
	return -1;
}

int lll_adv_data_dequeue(struct lll_adv_pdu *pdu)
{
#warning "TODO"
	return -1;
}

int lll_adv_data_release(struct lll_adv_pdu *pdu)
{
#warning "TODO"
	return -1;
}

void lll_adv_data_enqueue(struct lll_adv *lll, uint8_t idx)
{
#warning "TODO"
}

struct pdu_adv *lll_adv_data_alloc(struct lll_adv *lll, uint8_t *idx)
{
#warning "TODO"
	return NULL;
}

struct pdu_adv *lll_adv_data_peek(struct lll_adv *lll)
{
#warning "TODO"
	return NULL;
}

struct pdu_adv *lll_adv_data_latest_peek(const struct lll_adv *const lll)
{
#warning "TODO"
	return NULL;
}

struct pdu_adv *lll_adv_pdu_alloc_pdu_adv(void)
{
#warning "TODO"
	return NULL;
}

void lll_adv_scan_rsp_enqueue(struct lll_adv *lll, uint8_t idx)
{
#warning "TODO"
}

struct pdu_adv *lll_adv_scan_rsp_alloc(struct lll_adv *lll, uint8_t *idx)
{
#warning "TODO"
	return NULL;
}

struct pdu_adv *lll_adv_scan_rsp_peek(const struct lll_adv *lll)
{
#warning "TODO"
	return NULL;
}
