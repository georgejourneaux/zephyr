#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/util.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_conn.h"

int lll_conn_init(void)
{
#warning "TODO"
	return -1;
}

int lll_conn_reset(void)
{
#warning "TODO"
	return -1;
}

void lll_conn_flush(uint16_t handle, struct lll_conn *lll)
{
#warning "TODO"
}

void lll_conn_prepare_reset(void)
{
#warning "TODO"
}

void lll_conn_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
#warning "TODO"
}

void lll_conn_isr_rx(void *param)
{
#warning "TODO"
}

void lll_conn_isr_tx(void *param)
{
#warning "TODO"
}

void lll_conn_rx_pkt_set(struct lll_conn *lll)
{
#warning "TODO"
}

void lll_conn_tx_pkt_set(struct lll_conn *lll, struct pdu_data *pdu_data_tx)
{
#warning "TODO"
}

void lll_conn_pdu_tx_prep(struct lll_conn *lll, struct pdu_data **pdu_data_tx)
{
#warning "TODO"
}

uint8_t lll_conn_force_md_cnt_set(uint8_t force_md_cnt)
{
#warning "TODO"
	return 0;
}
