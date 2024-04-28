#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/bluetooth/hci_types.h>

#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_ble_mailbox.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/cc13xx_cc26xx/radio/rf_queue.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/util.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_conn.h"
#include "lll_clock.h"

#include "lll_internal.h"
#include "lll_prof_internal.h"
#include "lll_tim_internal.h"

#include "hal/debug.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_conn);

#define RF_RX_BUFFER_NUMBER_OF_ENTRIES (2)
#define RF_RX_BUFFER_LOCAL_DATA_SIZE   (0)
#define RF_RX_BUFFER_DATA_SIZE         (PDU_DC_CTRL_RX_SIZE_MAX)

#define RF_TX_BUFFER_NUMBER_OF_ENTRIES (2)
#define RF_TX_BUFFER_LOCAL_DATA_SIZE   (0)
#define RF_TX_BUFFER_DATA_SIZE         (PDU_DC_CTRL_TX_SIZE_MAX)

struct RF_RX_DATA_CONN {
	rfc_dataEntryPointer_t *head_entry;
	rfc_dataEntryPointer_t *tail_entry;
	dataQueue_t queue;
	uint8_t buffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(
		RF_RX_BUFFER_NUMBER_OF_ENTRIES, RF_QUEUE_DATA_ENTRY_POINTER_HEADER_SIZE,
		RF_RX_BUFFER_LOCAL_DATA_SIZE)] __attribute__((aligned(4)));
} rf_rx_data_conn;

struct RF_TX_DATA_CONN {
	rfc_dataEntryPointer_t *head_entry;
	rfc_dataEntryPointer_t *tail_entry;
	dataQueue_t queue;
	uint8_t buffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(
		RF_TX_BUFFER_NUMBER_OF_ENTRIES, RF_QUEUE_DATA_ENTRY_POINTER_HEADER_SIZE,
		RF_TX_BUFFER_LOCAL_DATA_SIZE)] __attribute__((aligned(4)));
} rf_tx_data_conn;

static uint8_t crc_expire;
static uint8_t crc_valid;
static uint16_t trx_cnt;

#warning "TODO: Remove if not needed"
static uint8_t MALIGN(4) _pkt_empty[PDU_EM_LL_SIZE_MAX];

static void init_reset(void);
static int isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_data_rx, uint8_t *is_rx_enqueue,
		      struct node_tx **tx_release, uint8_t *is_done);
static void lll_conn_isr_rx_exit(struct lll_conn *lll, struct node_rx_pdu *node_rx,
				 struct node_tx *tx_release, uint8_t is_rx_enqueue);

int lll_conn_init(void)
{
	init_reset();
	return 0;
}

int lll_conn_reset(void)
{
	init_reset();
	return 0;
}

void lll_conn_flush(uint16_t handle, struct lll_conn *lll)
{
	/* Nothing to be flushed */
}

void lll_conn_prepare_reset(void)
{
	trx_cnt = 0U;
	crc_valid = 0U;
	crc_expire = 0U;
}

void lll_conn_abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* Perform event abort here.
		 * After event has been cleanly aborted, clean up resources
		 * and dispatch event done.
		 */
		radio_disable(lll_isr_peripheral);
		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	int err = lll_hfclock_off();
	LL_ASSERT(err >= 0);

	/* Accumulate the latency as event is aborted while being in pipeline */
	struct lll_conn *lll = (struct lll_conn *)prepare_param->param;
	lll->latency_prepare += (prepare_param->lazy + 1);

	/* Extra done event, to check supervision timeout */
	struct event_done_extra *e = ull_event_done_extra_get();
	LL_ASSERT(e);

	e->type = EVENT_DONE_EXTRA_TYPE_CONN;
	e->trx_cnt = 0U;
	e->crc_valid = 0U;

	lll_done(param);
}

void lll_conn_isr_rx(void *param)
{
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_latency_capture();
	}

	struct node_rx_pdu *node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

	struct pdu_data *pdu_data_rx = (struct pdu_data *)node_rx->pdu;
	struct lll_conn *lll = (struct lll_conn *)param;

	uint8_t is_done = 0U;
	struct node_tx *tx_release = NULL;
	uint8_t is_rx_enqueue = 0U;

	if (isr_rx_pdu(lll, pdu_data_rx, &is_rx_enqueue, &tx_release, &is_done)) {
		/* Disable radio trx switch on MIC failure for both
		 * central and peripheral, and close the radio event.
		 */
		radio_disable(lll_isr_peripheral);

		lll_conn_isr_rx_exit(lll, node_rx, tx_release, is_rx_enqueue);
		return;
	}

	/* Reset CRC expiry counter */
	crc_expire = 0U;

	/* CRC valid flag used to detect supervision timeout */
	crc_valid = 1U;

#warning "TODO: Handle CRC errors here?"

	/* prepare tx packet */
	struct pdu_data *pdu_data_tx;
	lll_conn_pdu_tx_prep(lll, &pdu_data_tx);

	/* Decide on event continuation and hence Radio Shorts to use */
	is_done = is_done ||
		  ((pdu_data_rx->md == 0) && (pdu_data_tx->md == 0) && (pdu_data_tx->len == 0));

	/* Fill sn and nesn */
	pdu_data_tx->sn = lll->sn;
	pdu_data_tx->nesn = lll->nesn;

	/* setup the radio tx packet buffer */
	lll_conn_tx_pkt_set(lll, pdu_data_tx);

	lll_conn_isr_rx_exit(lll, node_rx, tx_release, is_rx_enqueue);
}

void lll_conn_isr_tx(void *param)
{
	if (rf_tx_data_conn.tail_entry->status == DATA_ENTRY_FINISHED) {
		if (rf_tx_data_conn.tail_entry->config.type == DATA_ENTRY_TYPE_PTR) {
#if defined(CONFIG_BT_TI_LLL_PACKET_DEBUG)
			uint8_t *data = rf_tx_data_conn.tail_entry->pData;
			uint8_t a_d = data[0];
			uint16_t data_size = data[1] + 2;
			LOG_WRN("tx_entry | ad %u | ds %u |", a_d, data_size);
			struct pdu_adv *pdu_adv = (struct pdu_adv *)data;
			LOG_WRN("pdu | type %u | rfu %u | ch %u | tx %u | rx %u | len %u |",
				pdu_adv->type, pdu_adv->rfu, pdu_adv->chan_sel, pdu_adv->tx_addr,
				pdu_adv->rx_addr, pdu_adv->len);
			LOG_HEXDUMP_WRN(data, data_size, "TX");
#endif /* CONFIG_BT_TI_LLL_PACKET_DEBUG */
		}

		rf_tx_data_conn.tail_entry =
			rf_queue_next_entry_pointer(rf_tx_data_conn.tail_entry);
	} else {
		return;
	}

	struct lll_conn *lll = (struct lll_conn *)param;
	lll_conn_rx_pkt_set(lll);

#if defined(CONFIG_BT_CTLR_PROFILE_ISR) || defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
	radio_tmr_end_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */
}

void lll_conn_rx_pkt_set(struct lll_conn *lll)
{
	struct node_rx_pdu *node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	uint16_t max_rx_octets = lll->dle.eff.max_rx_octets;
#else  /* !CONFIG_BT_CTLR_DATA_LENGTH */
	uint16_t max_rx_octets = PDU_DC_PAYLOAD_SIZE_MIN;
#endif /* !CONFIG_BT_CTLR_DATA_LENGTH */

	if ((PDU_DC_CTRL_RX_SIZE_MAX > PDU_DC_PAYLOAD_SIZE_MIN) &&
	    (max_rx_octets < PDU_DC_CTRL_RX_SIZE_MAX)) {
		max_rx_octets = PDU_DC_CTRL_RX_SIZE_MAX;
	}

	rf_rx_data_conn.head_entry = rf_queue_insert_entry_pointer(
		rf_rx_data_conn.head_entry, node_rx->pdu, RF_RX_BUFFER_DATA_SIZE);
}

void lll_conn_tx_pkt_set(struct lll_conn *lll, struct pdu_data *pdu_data_tx)
{
#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	uint16_t max_tx_octets = lll->dle.eff.max_tx_octets;
#else  /* !CONFIG_BT_CTLR_DATA_LENGTH */
	uint16_t max_tx_octets = PDU_DC_PAYLOAD_SIZE_MIN;
#endif /* !CONFIG_BT_CTLR_DATA_LENGTH */

	if ((PDU_DC_CTRL_TX_SIZE_MAX > PDU_DC_PAYLOAD_SIZE_MIN) &&
	    (max_tx_octets < PDU_DC_CTRL_TX_SIZE_MAX)) {
		max_tx_octets = PDU_DC_CTRL_TX_SIZE_MAX;
	}

	rf_tx_data_conn.head_entry = rf_queue_insert_entry_pointer(
		rf_tx_data_conn.head_entry, (uint8_t *)pdu_data_tx, RF_TX_BUFFER_DATA_SIZE);
}

void lll_conn_pdu_tx_prep(struct lll_conn *lll, struct pdu_data **pdu_data_tx)
{
	struct node_tx *tx;
	struct pdu_data *p;

	memq_link_t *link = memq_peek(lll->memq_tx.head, lll->memq_tx.tail, (void **)&tx);
	if (lll->empty || !link) {
		lll->empty = 1U;

#warning "TODO: Remove if not needed"
		p = (void *)_pkt_empty;
		if (link) {
			p->md = 1U;
		} else {
			p->md = 0U;
		}
	} else {
		uint16_t max_tx_octets;

		p = (struct pdu_data *)(tx->pdu + lll->packet_tx_head_offset);

		if (!lll->packet_tx_head_len) {
			lll->packet_tx_head_len = p->len;
		}

		if (lll->packet_tx_head_offset) {
			p->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
		}

		p->len = lll->packet_tx_head_len - lll->packet_tx_head_offset;

		max_tx_octets = ull_conn_lll_max_tx_octets_get(lll);

		if (((PDU_DC_CTRL_TX_SIZE_MAX <= PDU_DC_PAYLOAD_SIZE_MIN) ||
		     (p->ll_id != PDU_DATA_LLID_CTRL)) &&
		    (p->len > max_tx_octets)) {
			p->len = max_tx_octets;
			p->md = 1U;
		} else if (link->next != lll->memq_tx.tail) {
			p->md = 1U;
		} else {
			p->md = 0U;
		}

		p->rfu = 0U;

#if !defined(CONFIG_BT_CTLR_DATA_LENGTH_CLEAR)
		/* Initialize only if vendor PDU octet3 present */
		if (sizeof(p->octet3.resv)) {
			p->octet3.resv[0] = 0U;
		}
#endif /* CONFIG_BT_CTLR_DATA_LENGTH_CLEAR */
	}

	*pdu_data_tx = p;
}

uint8_t lll_conn_force_md_cnt_set(uint8_t force_md_cnt)
{
	return 0;
}

dataQueue_t *lll_conn_get_rf_rx_queue(void)
{
	return &rf_rx_data_conn.queue;
}

dataQueue_t *lll_conn_get_rf_tx_queue(void)
{
	return &rf_tx_data_conn.queue;
}

static void init_reset(void)
{
	rf_rx_data_conn.head_entry = rf_queue_define_queue_pointer(
		&rf_rx_data_conn.queue, rf_rx_data_conn.buffer, sizeof(rf_rx_data_conn.buffer),
		RF_RX_BUFFER_NUMBER_OF_ENTRIES);
	rf_rx_data_conn.tail_entry = rf_rx_data_conn.head_entry;
	LL_ASSERT(rf_rx_data_conn.head_entry);

	rf_tx_data_conn.head_entry = rf_queue_define_queue_pointer(
		&rf_tx_data_conn.queue, rf_tx_data_conn.buffer, sizeof(rf_tx_data_conn.buffer),
		RF_RX_BUFFER_NUMBER_OF_ENTRIES);
	rf_tx_data_conn.tail_entry = rf_tx_data_conn.head_entry;
	LL_ASSERT(rf_tx_data_conn.head_entry);
}

static int isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_data_rx, uint8_t *is_rx_enqueue,
		      struct node_tx **tx_release, uint8_t *is_done)
{
	if (rf_rx_data_conn.tail_entry->status == DATA_ENTRY_FINISHED) {
		if (rf_rx_data_conn.tail_entry->config.type == DATA_ENTRY_TYPE_PTR) {
#if defined(CONFIG_BT_TI_LLL_PACKET_DEBUG)
			uint8_t *data = rf_rx_data_conn.tail_entry->pData;
			uint8_t a_d = data[0];
			uint16_t data_size = data[1] + 2;
			LOG_WRN("rx_entry | ad %u | ds %u |", a_d, data_size);
			struct pdu_adv *pdu_adv = (struct pdu_adv *)data;
			LOG_WRN("pdu | type %u | rfu %u | ch %u | tx %u | rx %u | len %u |",
				pdu_adv->type, pdu_adv->rfu, pdu_adv->chan_sel, pdu_adv->tx_addr,
				pdu_adv->rx_addr, pdu_adv->len);
			LOG_HEXDUMP_WRN(data, data_size, "RX");
#endif /* CONFIG_BT_TI_LLL_PACKET_DEBUG */
		}

		rf_rx_data_conn.tail_entry =
			rf_queue_next_entry_pointer(rf_rx_data_conn.tail_entry);
	} else {
		return 1;
	}

	/* Ack for tx-ed data */
	if (pdu_data_rx->nesn != lll->sn) {
		struct pdu_data *pdu_data_tx;
		struct node_tx *tx;
		memq_link_t *link;

		/* Increment sequence number */
		lll->sn++;

#if defined(CONFIG_BT_PERIPHERAL)
		/* First ack (and redundantly any other ack) enable use of
		 * peripheral latency.
		 */
		if (lll->role) {
			lll->periph.latency_enabled = 1;
		}
#endif /* CONFIG_BT_PERIPHERAL */

		if (!lll->empty) {
			link = memq_peek(lll->memq_tx.head, lll->memq_tx.tail, (void **)&tx);
		} else {
			lll->empty = 0;

#warning "TODO: Remove if not needed"
			pdu_data_tx = (void *)_pkt_empty;
			if (IS_ENABLED(CONFIG_BT_CENTRAL) && !lll->role && !pdu_data_rx->md) {
				*is_done = !pdu_data_tx->md;
			}

			link = NULL;
		}

		if (link) {
			uint8_t pdu_data_tx_len;
			uint8_t offset;

			pdu_data_tx = (void *)(tx->pdu + lll->packet_tx_head_offset);

			pdu_data_tx_len = pdu_data_tx->len;

			offset = lll->packet_tx_head_offset + pdu_data_tx_len;
			if (offset < lll->packet_tx_head_len) {
				lll->packet_tx_head_offset = offset;
			} else if (offset == lll->packet_tx_head_len) {
				lll->packet_tx_head_len = 0;
				lll->packet_tx_head_offset = 0;

				memq_dequeue(lll->memq_tx.tail, &lll->memq_tx.head, NULL);

				/* TX node UPSTREAM, i.e. Tx node ack path */
				link->next = tx->next; /* Indicates ctrl or data
							* pool.
							*/
				tx->next = link;

				*tx_release = tx;
			} else {
				LL_ASSERT(0);
			}

			if (IS_ENABLED(CONFIG_BT_CENTRAL) && !lll->role && !pdu_data_rx->md) {
				*is_done = !pdu_data_tx->md;
			}
		}
	}

	/* process received data */
	if ((pdu_data_rx->sn == lll->nesn) &&
	    /* check so that we will NEVER use the rx buffer reserved for empty
	     * packet and internal control enqueue
	     */
	    (ull_pdu_rx_alloc_peek(3) != 0)) {
		/* Increment next expected serial number */
		lll->nesn++;

		if (pdu_data_rx->len != 0) {
			/* Enqueue non-empty PDU */
			*is_rx_enqueue = 1U;
		}
	}

	return 0;
}

static void lll_conn_isr_rx_exit(struct lll_conn *lll, struct node_rx_pdu *node_rx,
				 struct node_tx *tx_release, uint8_t is_rx_enqueue)
{
#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_cputime_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	if (tx_release) {
		LL_ASSERT(lll->handle != 0xFFFF);

		ull_conn_lll_ack_enqueue(lll->handle, tx_release);
	}

	if (is_rx_enqueue) {
		ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_DC_PDU;
		node_rx->hdr.handle = lll->handle;

		ull_rx_put(node_rx->hdr.link, node_rx);
	}

	if (tx_release || is_rx_enqueue) {
		ull_rx_sched();
	}

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	/* Collect RSSI for connection */
	uint8_t rssi = radio_rssi_get();

	lll->rssi_latest = rssi;

#if defined(CONFIG_BT_CTLR_CONN_RSSI_EVENT)
	if (((lll->rssi_reported - rssi) & 0xFF) > LLL_CONN_RSSI_THRESHOLD) {
		if (lll->rssi_sample_count) {
			lll->rssi_sample_count--;
		}
	} else {
		lll->rssi_sample_count = LLL_CONN_RSSI_SAMPLE_COUNT;
	}
#endif /* CONFIG_BT_CTLR_CONN_RSSI_EVENT */
#endif /* !CONFIG_BT_CTLR_CONN_RSSI */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_send();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */
}