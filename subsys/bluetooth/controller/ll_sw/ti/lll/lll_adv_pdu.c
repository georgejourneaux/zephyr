/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/byteorder.h>

#include "hal/ccm.h"

#include "util/util.h"
#include "util/memq.h"

#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_aux.h"
#include "lll_adv_sync.h"

int lll_adv_data_init(struct lll_adv_pdu *pdu)
{
	return 0;
}

int lll_adv_data_reset(struct lll_adv_pdu *pdu)
{
	return 0;
}

int lll_adv_data_dequeue(struct lll_adv_pdu *pdu)
{
	return 0;
}

int lll_adv_data_release(struct lll_adv_pdu *pdu)
{
	return 0;
}

void lll_adv_data_enqueue(struct lll_adv *lll, uint8_t idx)
{
}

struct pdu_adv *lll_adv_data_alloc(struct lll_adv *lll, uint8_t *idx)
{
	return NULL;
}

struct pdu_adv *lll_adv_data_curr_get(struct lll_adv *lll)
{
	return NULL;
}

struct pdu_adv *lll_adv_data_peek(struct lll_adv *lll)
{
	return NULL;
}

struct pdu_adv *lll_adv_data_latest_peek(const struct lll_adv *const lll)
{
	return NULL;
}

struct pdu_adv *lll_adv_pdu_alloc(struct lll_adv_pdu *pdu, uint8_t *idx)
{
	return NULL;
}

struct pdu_adv *lll_adv_pdu_alloc_pdu_adv(void)
{
	return NULL;
}

void lll_adv_scan_rsp_enqueue(struct lll_adv *lll, uint8_t idx)
{
}

struct pdu_adv *lll_adv_scan_rsp_alloc(struct lll_adv *lll, uint8_t *idx)
{
	return NULL;
}

struct pdu_adv *lll_adv_scan_rsp_peek(const struct lll_adv *lll)
{
	return NULL;
}

#if defined(CONFIG_BT_CTLR_ADV_EXT)
int lll_adv_aux_data_init(struct lll_adv_pdu *pdu)
{
}

struct pdu_adv *lll_adv_aux_data_alloc(struct lll_adv_aux *lll, uint8_t *idx)
{
}

void lll_adv_aux_data_enqueue(struct lll_adv_aux *lll, uint8_t idx)
{
}

struct pdu_adv *lll_adv_aux_data_peek(const struct lll_adv_aux *const lll)
{
}

struct pdu_adv *lll_adv_aux_data_latest_peek(const struct lll_adv_aux *const lll)
{
}

struct pdu_adv *lll_adv_aux_data_curr_get(struct lll_adv_aux *lll)
{
}

struct pdu_adv *lll_adv_aux_scan_rsp_alloc(struct lll_adv *lll, uint8_t *idx)
{
}

#if defined(CONFIG_BT_CTLR_ADV_PERIODIC)
int lll_adv_and_extra_data_release(struct lll_adv_pdu *pdu)
{
}

#if defined(CONFIG_BT_CTLR_ADV_SYNC_PDU_BACK2BACK)
void lll_adv_sync_pdu_b2b_update(struct lll_adv_sync *lll, uint8_t idx)
{
}
#endif

int lll_adv_sync_data_init(struct lll_adv_pdu *pdu)
{
}

struct pdu_adv *lll_adv_pdu_and_extra_data_alloc(struct lll_adv_pdu *pdu, void **extra_data,
						 uint8_t *idx)
{
}

struct pdu_adv *lll_adv_sync_data_alloc(struct lll_adv_sync *lll, void **extra_data, uint8_t *idx)
{
}

void lll_adv_sync_data_release(struct lll_adv_sync *lll)
{
}

void lll_adv_sync_data_enqueue(struct lll_adv_sync *lll, uint8_t idx)
{
}

struct pdu_adv *lll_adv_sync_data_peek(const struct lll_adv_sync *lll, void **extra_data)
{
}

struct pdu_adv *lll_adv_sync_data_latest_peek(const struct lll_adv_sync *const lll)
{
}

#if defined(CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY)
void *lll_adv_sync_extra_data_peek(struct lll_adv_sync *lll)
{
}

void *lll_adv_sync_extra_data_curr_get(struct lll_adv_sync *lll)
{
}

#endif /* CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY */
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC */
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
/* Release PDU and all linked PDUs, shall only be called from ULL */
void lll_adv_pdu_linked_release_all(struct pdu_adv *pdu_first)
{
}

struct pdu_adv *lll_adv_pdu_linked_next_get(struct pdu_adv *pdu)
{
}

struct pdu_adv *lll_adv_pdu_linked_last_get(struct pdu_adv *pdu)
{
}

void lll_adv_pdu_linked_append(struct pdu_adv *pdu, struct pdu_adv *prev)
{
}

void lll_adv_pdu_linked_append_end(struct pdu_adv *pdu, struct pdu_adv *first)
{
}

#endif
