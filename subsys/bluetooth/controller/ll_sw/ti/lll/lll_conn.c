#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/bluetooth/hci_types.h>

#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_ble_mailbox.h>

#include "hal/ccm.h"
#include "hal/radio.h"
#include "hal/cc13xx_cc26xx/radio/RFQueue.h"

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

static uint8_t crc_expire;
static uint8_t crc_valid;
static uint16_t trx_cnt;

static void isr_done(void *param);
static int isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_data_rx, uint8_t *is_rx_enqueue,
		      struct node_tx **tx_release, uint8_t *is_done);
static void lll_conn_isr_rx_exit(struct lll_conn *lll, struct node_rx_pdu *node_rx,
				 struct node_tx *tx_release, uint8_t is_rx_enqueue);

int lll_conn_init(void)
{
	return 0;
}

int lll_conn_reset(void)
{
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
		radio_isr_set(lll_isr_conn, param);
		radio_disable();
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

void lll_isr_conn(void *param, radio_isr_cb_rf_param_t rf_param)
{
	if (rf_param.event_mask & RADIO_RF_EVENT_MASK_TX_DONE) {
		trx_cnt++;
		lll_conn_isr_tx(param);
	}

	if (rf_param.event_mask & RADIO_RF_EVENT_MASK_RX_DONE) {
		lll_conn_isr_rx(param);
	}

	if (rf_param.event_mask & RADIO_RF_EVENT_MASK_CMD_DONE) {
		isr_done(param);
	}
}

void lll_conn_isr_rx(void *param)
{
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_latency_capture();
	}

	/* Clear radio rx status and events */
	lll_isr_rx_status_reset();

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
		radio_isr_set(lll_isr_conn, param);
		radio_disable();

		/* assert if radio started tx before being disabled */
		LL_ASSERT(!radio_is_ready());

		lll_conn_isr_rx_exit(lll, node_rx, tx_release, is_rx_enqueue);
		return;
	}

	/* Reset CRC expiry counter */
	crc_expire = 0U;

	/* CRC valid flag used to detect supervision timeout */
	crc_valid = 1U;

#warning "TODO: Handle CRC errors here?"
	// else {
	// 	/* Start CRC error countdown, if not already started */
	// 	if (crc_expire == 0U) {
	// 		crc_expire = 2U;
	// 	}

	// 	/* CRC error countdown */
	// 	crc_expire--;
	// 	is_done = (crc_expire == 0U);
	// }

#if defined(CONFIG_BT_CENTRAL)
	uint8_t is_empty_pdu_tx_retry = lll->empty;
#endif /* CONFIG_BT_CENTRAL */

	/* prepare tx packet */
	struct pdu_data *pdu_data_tx;
	lll_conn_pdu_tx_prep(lll, &pdu_data_tx);

	/* Decide on event continuation and hence Radio Shorts to use */
	is_done = is_done ||
		  ((pdu_data_rx->md == 0) && (pdu_data_tx->md == 0) && (pdu_data_tx->len == 0));

	if (is_done) {
		radio_isr_set(lll_isr_conn, param);
#if defined(CONFIG_BT_CENTRAL)
		/* Event done for central */
		if (!lll->role) {
			radio_disable();

			/* assert if radio packet ptr is not set and radio
			 * started tx.
			 */
			LL_ASSERT(!radio_is_ready());

			/* Restore state if last transmitted was empty PDU */
			lll->empty = is_empty_pdu_tx_retry;

			lll_conn_isr_rx_exit(lll, node_rx, tx_release, is_rx_enqueue);
			return;
		} else
#endif /* CONFIG_BT_CENTRAL */
		{
#if defined(CONFIG_BT_PERIPHERAL)
			/* Event done for peripheral */
			radio_switch_complete_and_disable();
#endif /* CONFIG_BT_PERIPHERAL */
		}
	} else {
		radio_tmr_tifs_set(EVENT_IFS_US);

#if defined(CONFIG_BT_CTLR_PHY)
		radio_switch_complete_and_rx(lll->phy_rx);
#else  /* !CONFIG_BT_CTLR_PHY */
		radio_switch_complete_and_rx(0);
#endif /* !CONFIG_BT_CTLR_PHY */

		radio_isr_set(lll_isr_conn, param);

		/* capture end of Tx-ed PDU, used to calculate HCTO. */
		radio_tmr_end_capture();
	}

	/* Fill sn and nesn */
	pdu_data_tx->sn = lll->sn;
	pdu_data_tx->nesn = lll->nesn;

	/* setup the radio tx packet buffer */
	lll_conn_tx_pkt_set(lll, pdu_data_tx);

#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	/* PA enable is overwriting packet end used in ISR profiling, hence
	 * back it up for later use.
	 */
	lll_prof_radio_end_backup();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	radio_gpio_pa_setup();

	uint8_t cte_len = 0U;

	uint32_t pa_lna_enable_us =
		radio_tmr_tifs_base_get() + EVENT_IFS_US + cte_len - HAL_RADIO_GPIO_PA_OFFSET;
#if defined(CONFIG_BT_CTLR_PHY)
	pa_lna_enable_us -= radio_rx_chain_delay_get(lll->phy_rx, PHY_FLAGS_S8);
#else  /* !CONFIG_BT_CTLR_PHY */
	pa_lna_enable_us -= radio_rx_chain_delay_get(0, PHY_FLAGS_S2);
#endif /* !CONFIG_BT_CTLR_PHY */
	radio_gpio_pa_lna_enable(pa_lna_enable_us);
#endif /* HAL_RADIO_GPIO_HAVE_PA_PIN */

	/* assert if radio packet ptr is not set and radio started tx */
	LL_ASSERT(!radio_is_ready());

	lll_conn_isr_rx_exit(lll, node_rx, tx_release, is_rx_enqueue);
}

void lll_conn_isr_tx(void *param)
{
	struct lll_conn *lll;
	uint32_t hcto;

	/* Clear radio tx status and events */
	lll_isr_tx_status_reset();

	/* setup tIFS switching */
	radio_tmr_tifs_set(EVENT_IFS_US);

	lll = param;

/* Use regular API for cases when:
 * - CTE RX is not enabled,
 * - SOC does not require compensation for PHYEND event delay.
 */
#if !defined(CONFIG_BT_CTLR_DF_PHYEND_OFFSET_COMPENSATION_ENABLE)
#if defined(CONFIG_BT_CTLR_PHY)
	radio_switch_complete_and_tx(lll->phy_rx, 0, lll->phy_tx, lll->phy_flags);
#else  /* !CONFIG_BT_CTLR_PHY */
	radio_switch_complete_and_tx(0, 0, 0, 0);
#endif /* !CONFIG_BT_CTLR_PHY */
#endif /* !CONFIG_BT_CTLR_DF_PHYEND_OFFSET_COMPENSATION_ENABLE */

	lll_conn_rx_pkt_set(lll);

	/* assert if radio packet ptr is not set and radio started rx */
	LL_ASSERT(!radio_is_ready());

	/* +/- 2us active clock jitter, +1 us hcto compensation */
	hcto = radio_tmr_tifs_base_get() + EVENT_IFS_US + (EVENT_CLOCK_JITTER_US << 1) +
	       RANGE_DELAY_US + HCTO_START_DELAY_US;
#if defined(CONFIG_BT_CTLR_PHY)
	hcto += radio_rx_chain_delay_get(lll->phy_rx, 1);
	hcto += addr_us_get(lll->phy_rx);
	hcto -= radio_tx_chain_delay_get(lll->phy_tx, lll->phy_flags);
#else  /* !CONFIG_BT_CTLR_PHY */
	hcto += radio_rx_chain_delay_get(0, 0);
	hcto += addr_us_get(0);
	hcto -= radio_tx_chain_delay_get(0, 0);
#endif /* !CONFIG_BT_CTLR_PHY */

	radio_tmr_hcto_configure(hcto);

#if defined(CONFIG_BT_CENTRAL) && defined(CONFIG_BT_CTLR_CONN_RSSI)
	if (!trx_cnt && !lll->role) {
		radio_rssi_measure();
	}
#endif /* CONFIG_BT_CTLR_DF_CONN_CTE_RX */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR) || defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
	radio_tmr_end_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

#if defined(HAL_RADIO_GPIO_HAVE_LNA_PIN)
	radio_gpio_lna_setup();
#if defined(CONFIG_BT_CTLR_PHY)
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + EVENT_IFS_US - 4 -
				 radio_tx_chain_delay_get(lll->phy_tx, lll->phy_flags) -
				 HAL_RADIO_GPIO_LNA_OFFSET);
#else  /* !CONFIG_BT_CTLR_PHY */
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + EVENT_IFS_US - 4 -
				 radio_tx_chain_delay_get(0, 0) - HAL_RADIO_GPIO_LNA_OFFSET);
#endif /* !CONFIG_BT_CTLR_PHY */
#endif /* HAL_RADIO_GPIO_HAVE_LNA_PIN */

	radio_isr_set(lll_isr_conn, param);

#if defined(CONFIG_BT_CTLR_LOW_LAT_ULL)
	ull_conn_lll_tx_demux_sched(lll);
#endif /* CONFIG_BT_CTLR_LOW_LAT_ULL */
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

#if defined(CONFIG_BT_CTLR_PHY)
	uint8_t phy = lll->phy_rx;
#else  /* !CONFIG_BT_CTLR_PHY */
	uint8_t phy = 0U;
#endif /* !CONFIG_BT_CTLR_PHY */

	radio_phy_set(phy, 0);

#if defined(CONFIG_BT_CTLR_LE_ENC)
	if (lll->enc_rx) {
		radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT, (max_rx_octets + PDU_MIC_SIZE),
				    RADIO_PKT_CONF_FLAGS(RADIO_PKT_CONF_PDU_TYPE_DC, phy,
							 RADIO_PKT_CONF_CTE_DISABLED));

#if defined(HAL_RADIO_PDU_LEN_MAX) &&                                                              \
	(!defined(CONFIG_BT_CTLR_DATA_LENGTH_MAX) ||                                               \
	 (CONFIG_BT_CTLR_DATA_LENGTH_MAX < (HAL_RADIO_PDU_LEN_MAX - 4)))
		radio_pkt_rx_set(radio_ccm_rx_pkt_set(&lll->ccm_rx, phy, radio_pkt_decrypt_get()));
#elif !defined(HAL_RADIO_PDU_LEN_MAX)
#error "Undefined HAL_RADIO_PDU_LEN_MAX."
#else
		radio_pkt_rx_set(radio_ccm_rx_pkt_set(&lll->ccm_rx, phy, node_rx->pdu));
#endif
	} else
#endif /* CONFIG_BT_CTLR_LE_ENC */
	{
		radio_pkt_rx_set(node_rx->pdu);
	}
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

#if defined(CONFIG_BT_CTLR_PHY)
	uint8_t phy = lll->phy_tx;
	uint8_t flags = lll->phy_flags;
#else  /* !CONFIG_BT_CTLR_PHY */
	uint8_t phy = 0U;
	uint8_t flags = 0U;
#endif /* !CONFIG_BT_CTLR_PHY */

	radio_phy_set(phy, flags);
#if defined(CONFIG_BT_CTLR_DF_CONN_CTE_TX) || defined(CONFIG_BT_CTLR_LE_ENC)
	uint8_t pkt_flags =
		RADIO_PKT_CONF_FLAGS(RADIO_PKT_CONF_PDU_TYPE_DC, phy, RADIO_PKT_CONF_CTE_DISABLED);
#endif
#if defined(CONFIG_BT_CTLR_DF_CONN_CTE_TX)
	if (pdu_data_tx->cp) {
		pkt_flags = RADIO_PKT_CONF_FLAGS(RADIO_PKT_CONF_PDU_TYPE_DC, phy,
						 RADIO_PKT_CONF_CTE_ENABLED);
	}
#endif /* CONFIG_BT_CTLR_DF_CONN_CTE_TX */
#if defined(CONFIG_BT_CTLR_LE_ENC)
	if (lll->enc_tx) {
		radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT, (max_tx_octets + PDU_MIC_SIZE),
				    pkt_flags);

		radio_pkt_tx_set(radio_ccm_tx_pkt_set(&lll->ccm_tx, pdu_data_tx));
	} else
#endif /* CONFIG_BT_CTLR_LE_ENC */
	{
#warning "TODO - send empty packet?"
		// radio_pkt_tx_set(NULL);
	}
}

void lll_conn_pdu_tx_prep(struct lll_conn *lll, struct pdu_data **pdu_data_tx)
{
	struct node_tx *tx;
	struct pdu_data *p;

	memq_link_t *link = memq_peek(lll->memq_tx.head, lll->memq_tx.tail, (void **)&tx);
	if (lll->empty || !link) {
		lll->empty = 1U;

		p = (struct pdu_data *)radio_pkt_empty_get();
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

static void isr_done(void *param)
{
	lll_isr_status_reset();

	struct event_done_extra *e = ull_event_done_extra_get();
	LL_ASSERT(e);

	e->type = EVENT_DONE_EXTRA_TYPE_CONN;
	e->trx_cnt = trx_cnt;
	e->crc_valid = crc_valid;

#if defined(CONFIG_BT_PERIPHERAL)
	if (trx_cnt) {
		struct lll_conn *lll = (struct lll_conn *)param;

		if (lll->role) {
#if defined(CONFIG_BT_CTLR_PHY)
			uint32_t preamble_to_addr_us = addr_us_get(lll->phy_rx);
#else  /* !CONFIG_BT_CTLR_PHY */
			uint32_t preamble_to_addr_us = addr_us_get(0);
#endif /* !CONFIG_BT_CTLR_PHY */

			e->drift.start_to_address_actual_us =
				radio_tmr_aa_restore() - radio_tmr_ready_get();
			e->drift.window_widening_event_us = lll->periph.window_widening_event_us;
			e->drift.preamble_to_addr_us = preamble_to_addr_us;

			/* Reset window widening, as anchor point sync-ed */
			lll->periph.window_widening_event_us = 0;
			lll->periph.window_size_event_us = 0;
		}
	}
#endif /* CONFIG_BT_PERIPHERAL */

	lll_isr_cleanup(param);
}

static int isr_rx_pdu(struct lll_conn *lll, struct pdu_data *pdu_data_rx, uint8_t *is_rx_enqueue,
		      struct node_tx **tx_release, uint8_t *is_done)
{
#if defined(CONFIG_BT_CTLR_LE_ENC) && defined(HAL_RADIO_PDU_LEN_MAX) &&                            \
	(!defined(CONFIG_BT_CTLR_DATA_LENGTH_MAX) ||                                               \
	 (CONFIG_BT_CTLR_DATA_LENGTH_MAX < (HAL_RADIO_PDU_LEN_MAX - 4)))
	if (lll->enc_rx) {
		uint8_t *pkt_decrypt;

		pkt_decrypt = radio_pkt_decrypt_get();
		memcpy((void *)pdu_data_rx, (void *)pkt_decrypt, offsetof(struct pdu_data, lldata));
	}
#elif !defined(HAL_RADIO_PDU_LEN_MAX)
#error "Undefined HAL_RADIO_PDU_LEN_MAX."
#endif

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

			pdu_data_tx = (void *)radio_pkt_empty_get();
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
#if defined(CONFIG_BT_CTLR_LE_ENC)
			if (pdu_data_tx_len != 0U) {
				/* if encrypted increment tx counter */
				if (lll->enc_tx) {
					lll->ccm_tx.counter++;
				}
			}
#endif /* CONFIG_BT_CTLR_LE_ENC */

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
#if defined(CONFIG_BT_CTLR_LE_ENC)
			/* If required, wait for CCM to finish
			 */
			if (lll->enc_rx) {
				uint32_t done;

				done = radio_ccm_is_done();
				LL_ASSERT(done);

				bool mic_failure = !radio_ccm_mic_is_valid();

				if (mic_failure && lll->ccm_rx.counter == 0 &&
				    (pdu_data_rx->ll_id == PDU_DATA_LLID_CTRL)) {
					/* Received an LL control packet in the
					 * middle of the LL encryption procedure
					 * with MIC failure.
					 * This could be an unencrypted packet
					 */
					struct pdu_data *scratch_pkt = radio_pkt_scratch_get();

					if (ctrl_pdu_len_check(scratch_pkt->len)) {
						memcpy(pdu_data_rx, scratch_pkt,
						       scratch_pkt->len +
							       offsetof(struct pdu_data, llctrl));
						mic_failure = false;
						lll->ccm_rx.counter--;
					}
				}

				if (mic_failure) {
					/* Record MIC invalid */
					mic_state = LLL_CONN_MIC_FAIL;

					return -EINVAL;
				}

				/* Increment counter */
				lll->ccm_rx.counter++;

				/* Record MIC valid */
				mic_state = LLL_CONN_MIC_PASS;
			}
#endif /* CONFIG_BT_CTLR_LE_ENC */

			/* Enqueue non-empty PDU */
			*is_rx_enqueue = 1U;
		}
	}

	return 0;
}

static void lll_conn_isr_rx_exit(struct lll_conn *lll, struct node_rx_pdu *node_rx,
				 struct node_tx *tx_release, uint8_t is_rx_enqueue)
{
	/* Save the AA captured for the first Rx in connection event */
	if (!radio_tmr_aa_restore()) {
		radio_tmr_aa_save(radio_tmr_aa_get());
	}

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_cputime_capture();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */

	uint8_t is_ull_rx = 0U;

	if (tx_release) {
		LL_ASSERT(lll->handle != 0xFFFF);

		ull_conn_lll_ack_enqueue(lll->handle, tx_release);

		is_ull_rx = 1U;
	}

	if (is_rx_enqueue) {
#if defined(CONFIG_BT_CTLR_LE_ENC) && defined(HAL_RADIO_PDU_LEN_MAX) &&                            \
	(!defined(CONFIG_BT_CTLR_DATA_LENGTH_MAX) ||                                               \
	 (CONFIG_BT_CTLR_DATA_LENGTH_MAX < (HAL_RADIO_PDU_LEN_MAX - 4)))
		if (lll->enc_rx) {
			uint8_t *pkt_decrypt_data;

			pkt_decrypt_data = (uint8_t *)radio_pkt_decrypt_get() +
					   offsetof(struct pdu_data, lldata);
			memcpy((void *)pdu_data_rx->lldata, (void *)pkt_decrypt_data,
			       pdu_data_rx->len);
		}
#elif !defined(HAL_RADIO_PDU_LEN_MAX)
#error "Undefined HAL_RADIO_PDU_LEN_MAX."
#endif
		ull_pdu_rx_alloc();

		node_rx->hdr.type = NODE_RX_TYPE_DC_PDU;
		node_rx->hdr.handle = lll->handle;

		ull_rx_put(node_rx->hdr.link, node_rx);
		is_ull_rx = 1U;
	}

	if (is_ull_rx) {
		ull_rx_sched();
	}

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	/* Collect RSSI for connection */
	if (radio_rssi_is_ready()) {
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
	}
#endif /* !CONFIG_BT_CTLR_CONN_RSSI */

#if defined(CONFIG_BT_CTLR_PROFILE_ISR)
	lll_prof_send();
#endif /* CONFIG_BT_CTLR_PROFILE_ISR */
}