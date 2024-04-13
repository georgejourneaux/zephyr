/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/errno_private.h>

#include "hal/ccm.h"
#include "hal/cpu.h"
#include "hal/debug.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mfifo.h"
#include "util/util.h"

#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_aux.h"
#include "lll_adv_sync.h"

#define PDU_FREE_TIMEOUT K_SECONDS(5)

#if defined(CONFIG_BT_CTLR_ADV_EXT)
#define PAYLOAD_BASED_FRAG_COUNT                                                                   \
	DIV_ROUND_UP(CONFIG_BT_CTLR_ADV_DATA_LEN_MAX, PDU_AC_PAYLOAD_SIZE_MAX)
#define PAYLOAD_FRAG_COUNT  MAX(PAYLOAD_BASED_FRAG_COUNT, BT_CTLR_DF_PER_ADV_CTE_NUM_MAX)
#define BT_CTLR_ADV_AUX_SET CONFIG_BT_CTLR_ADV_AUX_SET
#if defined(CONFIG_BT_CTLR_ADV_PERIODIC)
#define BT_CTLR_ADV_SYNC_SET CONFIG_BT_CTLR_ADV_SYNC_SET
#else /* !CONFIG_BT_CTLR_ADV_PERIODIC */
#define BT_CTLR_ADV_SYNC_SET 0
#endif /* !CONFIG_BT_CTLR_ADV_PERIODIC */
#else
#define PAYLOAD_BASED_FRAG_COUNT 1
#define PAYLOAD_FRAG_COUNT       (PAYLOAD_BASED_FRAG_COUNT)
#define BT_CTLR_ADV_AUX_SET      0
#define BT_CTLR_ADV_SYNC_SET     0
#endif

/* AD data and Scan Response Data need 2 PDU buffers each in the double buffer
 * implementation. Allocate 3 PDU buffers plus CONFIG_BT_CTLR_ADV_DATA_BUF_MAX
 * defined buffer count as the minimum number of buffers that meet the legacy
 * advertising needs. Add 1 each for Extended and Periodic Advertising, needed
 * extra for double buffers for these is kept as configurable, by increasing
 * CONFIG_BT_CTLR_ADV_DATA_BUF_MAX.
 */
#define PDU_MEM_COUNT_MIN                                                                          \
	(((BT_CTLR_ADV_SET) * 3) + ((BT_CTLR_ADV_AUX_SET) * PAYLOAD_BASED_FRAG_COUNT) +            \
	 ((BT_CTLR_ADV_SYNC_SET) * PAYLOAD_FRAG_COUNT))

/* Maximum advertising PDU buffers to allocate, which is the sum of minimum
 * plus configured additional count in CONFIG_BT_CTLR_ADV_DATA_BUF_MAX.
 */
#if defined(CONFIG_BT_CTLR_ADV_EXT)
#if defined(CONFIG_BT_CTLR_ADV_PERIODIC)
/* NOTE: When Periodic Advertising is supported then one additional PDU buffer
 *       plus the additional CONFIG_BT_CTLR_ADV_DATA_BUF_MAX amount of buffers
 *       is allocated.
 *       Set CONFIG_BT_CTLR_ADV_DATA_BUF_MAX to (BT_CTLR_ADV_AUX_SET +
 *       BT_CTLR_ADV_SYNC_SET) if
 *       PDU data is updated more frequently compare to the advertising
 *       interval with random delay included.
 */
#define PDU_MEM_COUNT_MAX                                                                          \
	((PDU_MEM_COUNT_MIN) + ((BT_CTLR_ADV_SYNC_SET) * PAYLOAD_FRAG_COUNT) +                     \
	 (CONFIG_BT_CTLR_ADV_DATA_BUF_MAX * PAYLOAD_BASED_FRAG_COUNT))
#else /* !CONFIG_BT_CTLR_ADV_PERIODIC */
/* NOTE: When Extended Advertising is supported but no Periodic Advertising
 *       then additional CONFIG_BT_CTLR_ADV_DATA_BUF_MAX amount of buffers is
 *       allocated.
 *       Set CONFIG_BT_CTLR_ADV_DATA_BUF_MAX to BT_CTLR_ADV_AUX_SET if
 *       PDU data is updated more frequently compare to the advertising
 *       interval with random delay included.
 */
#define PDU_MEM_COUNT_MAX                                                                          \
	(PDU_MEM_COUNT_MIN + (CONFIG_BT_CTLR_ADV_DATA_BUF_MAX * PAYLOAD_BASED_FRAG_COUNT))
#endif /* !CONFIG_BT_CTLR_ADV_PERIODIC */
#else  /* !CONFIG_BT_CTLR_ADV_EXT */
/* NOTE: When Extended Advertising is not supported then
 *       CONFIG_BT_CTLR_ADV_DATA_BUF_MAX is restricted to 1 in Kconfig file.
 */
#define PDU_MEM_COUNT_MAX (PDU_MEM_COUNT_MIN + CONFIG_BT_CTLR_ADV_DATA_BUF_MAX)
#endif /* !CONFIG_BT_CTLR_ADV_EXT */

/* FIFO element count, that returns the consumed advertising PDUs (AD and Scan
 * Response). 1 each for primary channel PDU (AD and Scan Response), plus one
 * each for Extended Advertising and Periodic Advertising times the number of
 * chained fragments that would get returned.
 */
#define PDU_MEM_FIFO_COUNT                                                                         \
	((BT_CTLR_ADV_SET) + 1 + ((BT_CTLR_ADV_AUX_SET) * PAYLOAD_BASED_FRAG_COUNT) +              \
	 ((BT_CTLR_ADV_SYNC_SET) * PAYLOAD_FRAG_COUNT))

#define PDU_ADV_POOL_SIZE (PDU_ADV_MEM_SIZE * PDU_MEM_COUNT_MAX)

/* Free AD data PDU buffer pool */
static struct {
	void *free;
	uint8_t pool[PDU_ADV_POOL_SIZE];
} mem_pdu;

/* FIFO to return stale AD data PDU buffers from LLL to thread context */
static MFIFO_DEFINE(pdu_free, sizeof(void *), PDU_MEM_FIFO_COUNT);

/* Semaphore to wakeup thread waiting for free AD data PDU buffers */
static struct k_sem sem_pdu_free;

#if defined(CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY)
#if defined(CONFIG_BT_CTLR_DF_ADV_CTE_TX)
#define EXTRA_DATA_MEM_SIZE MROUND(sizeof(struct lll_df_adv_cfg))
#else
#define EXTRA_DATA_MEM_SIZE 0
#endif /* CONFIG_BT_CTLR_DF_ADV_CTE_TX */

/* ToDo check if number of fragments is not smaller than number of CTE
 * to be transmitted. Pay attention it would depend on the chain PDU storage
 *
 * Currently we can send only single CTE with AUX_SYNC_IND.
 * Number is equal to allowed adv sync sets * 2 (double buffering).
 */
#define EXTRA_DATA_MEM_COUNT      (BT_CTLR_ADV_SYNC_SET * PAYLOAD_FRAG_COUNT + 1)
#define EXTRA_DATA_MEM_FIFO_COUNT (EXTRA_DATA_MEM_COUNT * 2)
#define EXTRA_DATA_POOL_SIZE      (EXTRA_DATA_MEM_SIZE * EXTRA_DATA_MEM_COUNT * 2)

/* Free extra data buffer pool */
static struct {
	void *free;
	uint8_t pool[EXTRA_DATA_POOL_SIZE];
} mem_extra_data;

/* FIFO to return stale extra data buffers from LLL to thread context. */
static MFIFO_DEFINE(extra_data_free, sizeof(void *), EXTRA_DATA_MEM_FIFO_COUNT);
static struct k_sem sem_extra_data_free;
#endif /* CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY */

static inline void lll_adv_pdu_enqueue(struct lll_adv_pdu *pdu, uint8_t idx);
#if defined(CONFIG_BT_CTLR_ZLI)
static void mfy_pdu_free_sem_give(void *param);
#endif /* !CONFIG_BT_CTLR_ZLI */
static void pdu_free_sem_give(void);

int lll_adv_pdu_init_reset(void)
{
	/* Initialize AC PDU pool */
	mem_init(mem_pdu.pool, PDU_ADV_MEM_SIZE, (sizeof(mem_pdu.pool) / PDU_ADV_MEM_SIZE),
		 &mem_pdu.free);

	/* Initialize AC PDU free buffer return queue */
	MFIFO_INIT(pdu_free);

#if defined(CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY)
	/* Initialize extra data pool */
	mem_init(mem_extra_data.pool, EXTRA_DATA_MEM_SIZE,
		 (sizeof(mem_extra_data.pool) / EXTRA_DATA_MEM_SIZE), &mem_extra_data.free);

	/* Initialize extra data free buffer return queue */
	MFIFO_INIT(extra_data_free);

	k_sem_init(&sem_extra_data_free, 0, EXTRA_DATA_MEM_FIFO_COUNT);
#endif /* CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY */

	/* Initialize semaphore for ticker API blocking wait */
	k_sem_init(&sem_pdu_free, 0, PDU_MEM_FIFO_COUNT);

	return 0;
}

int lll_adv_data_init(struct lll_adv_pdu *pdu)
{
	struct pdu_adv *p;

	p = mem_acquire(&mem_pdu.free);
	if (!p) {
		return -ENOMEM;
	}

#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
	PDU_ADV_NEXT_PTR(p) = NULL;
#endif /* CONFIG_BT_CTLR_ADV_PDU_LINK */

	p->len = 0U;
	pdu->pdu[0] = (void *)p;

	return 0;
}

int lll_adv_data_reset(struct lll_adv_pdu *pdu)
{
	/* NOTE: this function is used on HCI reset to mem-zero the structure
	 *       members that otherwise was zero-ed by the architecture
	 *       startup code that zero-ed the .bss section.
	 *       pdu[0] element in the array is not initialized as subsequent
	 *       call to lll_adv_data_init will allocate a PDU buffer and
	 *       assign that.
	 */

	pdu->first = 0U;
	pdu->last = 0U;
	pdu->pdu[1] = NULL;

#if defined(CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY)
	/* Both slots are NULL because the extra_memory is allocated only
	 * on request. Not every advertising PDU includes extra_data.
	 */
	pdu->extra_data[0] = NULL;
	pdu->extra_data[1] = NULL;
#endif /* CONFIG_BT_CTLR_ADV_EXT_PDU_EXTRA_DATA_MEMORY */
	return 0;
}

int lll_adv_data_dequeue(struct lll_adv_pdu *pdu)
{
	uint8_t first;
	void *p;

	first = pdu->first;
	if (first == pdu->last) {
		return -ENOMEM;
	}

	p = pdu->pdu[first];
	pdu->pdu[first] = NULL;
	mem_release(p, &mem_pdu.free);

	first++;
	if (first == DOUBLE_BUFFER_SIZE) {
		first = 0U;
	}
	pdu->first = first;

	return 0;
}

int lll_adv_data_release(struct lll_adv_pdu *pdu)
{
	uint8_t last;
	void *p;

	last = pdu->last;
	p = pdu->pdu[last];
	if (p) {
		pdu->pdu[last] = NULL;
		mem_release(p, &mem_pdu.free);
	}

	last++;
	if (last == DOUBLE_BUFFER_SIZE) {
		last = 0U;
	}
	p = pdu->pdu[last];
	if (p) {
		pdu->pdu[last] = NULL;
		mem_release(p, &mem_pdu.free);
	}

	return 0;
}

void lll_adv_data_enqueue(struct lll_adv *lll, uint8_t idx)
{
	lll_adv_pdu_enqueue(&lll->adv_data, idx);
}

struct pdu_adv *lll_adv_data_alloc(struct lll_adv *lll, uint8_t *idx)
{
	return lll_adv_pdu_alloc(&lll->adv_data, idx);
}

struct pdu_adv *lll_adv_data_curr_get(struct lll_adv *lll)
{
	return (struct pdu_adv *)lll->adv_data.pdu[lll->adv_data.first];
}

struct pdu_adv *lll_adv_data_peek(struct lll_adv *lll)
{
	return (struct pdu_adv *)lll->adv_data.pdu[lll->adv_data.last];
}

struct pdu_adv *lll_adv_data_latest_peek(const struct lll_adv *const lll)
{
	return lll_adv_pdu_latest_peek(&lll->adv_data);
}

struct pdu_adv *lll_adv_pdu_alloc(struct lll_adv_pdu *pdu, uint8_t *idx)
{
	uint8_t first, last;
	void *p;

	/* TODO: Make this unique mechanism to update last element in double
	 *       buffer a re-usable utility function.
	 */
	first = pdu->first;
	last = pdu->last;
	if (first == last) {
		/* Return the index of next free PDU in the double buffer */
		last++;
		if (last == DOUBLE_BUFFER_SIZE) {
			last = 0U;
		}
	} else {
		uint8_t first_latest;

		/* LLL has not consumed the first PDU. Revert back the `last` so
		 * that LLL still consumes the first PDU while the caller of
		 * this function updates/modifies the latest PDU.
		 *
		 * Under race condition:
		 * 1. LLL runs before `pdu->last` is reverted, then `pdu->first`
		 *    has changed, hence restore `pdu->last` and return index of
		 *    next free PDU in the double buffer.
		 * 2. LLL runs after `pdu->last` is reverted, then `pdu->first`
		 *    will not change, return the saved `last` as the index of
		 *    the next free PDU in the double buffer.
		 */
		pdu->last = first;
		cpu_dmb();
		first_latest = pdu->first;
		if (first_latest != first) {
			pdu->last = last;
			last++;
			if (last == DOUBLE_BUFFER_SIZE) {
				last = 0U;
			}
		}
	}

	*idx = last;

	p = (void *)pdu->pdu[last];
	if (p) {
		return p;
	}

	p = lll_adv_pdu_alloc_pdu_adv();

	pdu->pdu[last] = (void *)p;

	return p;
}

struct pdu_adv *lll_adv_pdu_alloc_pdu_adv(void)
{
	struct pdu_adv *p;
	int err;

	p = MFIFO_DEQUEUE_PEEK(pdu_free);
	if (p) {
		k_sem_reset(&sem_pdu_free);

		MFIFO_DEQUEUE(pdu_free);

#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
		PDU_ADV_NEXT_PTR(p) = NULL;
#endif
		return p;
	}

	p = mem_acquire(&mem_pdu.free);
	if (p) {
#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
		PDU_ADV_NEXT_PTR(p) = NULL;
#endif
		return p;
	}

	err = k_sem_take(&sem_pdu_free, PDU_FREE_TIMEOUT);
	LL_ASSERT(!err);

	k_sem_reset(&sem_pdu_free);

	p = MFIFO_DEQUEUE(pdu_free);
	LL_ASSERT(p);

#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
	PDU_ADV_NEXT_PTR(p) = NULL;
#endif
	return p;
}

struct pdu_adv *lll_adv_pdu_latest_peek(const struct lll_adv_pdu *const pdu)
{
	uint8_t first;

	first = pdu->first;
	if (first != pdu->last) {
		first += 1U;
		if (first == DOUBLE_BUFFER_SIZE) {
			first = 0U;
		}
	}

	return (void *)pdu->pdu[first];
}

void lll_adv_scan_rsp_enqueue(struct lll_adv *lll, uint8_t idx)
{
	lll_adv_pdu_enqueue(&lll->scan_rsp, idx);
}

struct pdu_adv *lll_adv_scan_rsp_alloc(struct lll_adv *lll, uint8_t *idx)
{
	return lll_adv_pdu_alloc(&lll->scan_rsp, idx);
}

struct pdu_adv *lll_adv_scan_rsp_peek(const struct lll_adv *lll)
{
	return (struct pdu_adv *)lll->scan_rsp.pdu[lll->scan_rsp.last];
}

struct pdu_adv *lll_adv_pdu_latest_get(struct lll_adv_pdu *pdu, uint8_t *is_modified)
{
	uint8_t first;

	first = pdu->first;
	if (first != pdu->last) {
		uint8_t free_idx;
		uint8_t pdu_idx;
		void *p;

		pdu_idx = first;
		p = pdu->pdu[pdu_idx];

		do {
			void *next;

			/* Store partial list in current data index if there is
			 * no free slot in mfifo. It can be released on next
			 * switch attempt (on next event).
			 */
			if (!MFIFO_ENQUEUE_IDX_GET(pdu_free, &free_idx)) {
				break;
			}

#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
			next = lll_adv_pdu_linked_next_get(p);
#else
			next = NULL;
#endif

			MFIFO_BY_IDX_ENQUEUE(pdu_free, free_idx, p);
			pdu_free_sem_give();

			p = next;
		} while (p);

		/* If not all PDUs where released into mfifo, keep the list in
		 * current data index, to be released on the next switch
		 * attempt.
		 */
		pdu->pdu[pdu_idx] = p;

		/* Progress to next data index */
		first += 1U;
		if (first == DOUBLE_BUFFER_SIZE) {
			first = 0U;
		}
		pdu->first = first;
		*is_modified = 1U;
	}

	return (void *)pdu->pdu[first];
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

static inline void lll_adv_pdu_enqueue(struct lll_adv_pdu *pdu, uint8_t idx)
{
	pdu->last = idx;
}

#if defined(CONFIG_BT_CTLR_ZLI)
static void mfy_pdu_free_sem_give(void *param)
{
	ARG_UNUSED(param);

	k_sem_give(&sem_pdu_free);
}

static void pdu_free_sem_give(void)
{
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, mfy_pdu_free_sem_give};

	/* Ignore mayfly_enqueue failure on repeated enqueue call */
	(void)mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_ULL_HIGH, 0, &mfy);
}

#else  /* !CONFIG_BT_CTLR_ZLI */
static void pdu_free_sem_give(void)
{
	k_sem_give(&sem_pdu_free);
}
#endif /* !CONFIG_BT_CTLR_ZLI */