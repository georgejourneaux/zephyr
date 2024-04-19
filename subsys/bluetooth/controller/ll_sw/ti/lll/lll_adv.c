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
#include <zephyr/sys/util_macro.h>

#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_ble_mailbox.h>
#include "hal/cc13xx_cc26xx/radio/RFQueue.h"

#include "hal/ccm.h"
#include "hal/debug.h"
#include "hal/radio.h"
#include "hal/ticker.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/util.h"

#include "ticker/ticker.h"

#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_adv_types.h"
#include "lll_adv.h"
#include "lll_adv_pdu.h"
#include "lll_adv_aux.h"
#include "lll_adv_sync.h"
#include "lll_clock.h"
#include "lll_conn.h"
#include "lll_filter.h"
#include "lll_vendor.h"

#include "lll_internal.h"
#include "lll_prof_internal.h"
#include "lll_tim_internal.h"

static rfc_bleAdvPar_t ble_adv_param;
static rfc_bleAdvOutput_t ble_adv_output;
static rfc_CMD_BLE5_ADV_t cmd_ble5_adv;

static int init_reset(void);

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static int prepare_cb(struct lll_prepare_param *p);
static struct pdu_adv *chan_prepare(struct lll_adv *lll);

#if defined(CONFIG_BT_PERIPHERAL)
static int resume_prepare_cb(struct lll_prepare_param *p);
static void isr_abort_all(void *param, radio_isr_cb_rf_param_t rf_param);
#endif /* CONFIG_BT_PERIPHERAL */

static void isr_abort(void *param, radio_isr_cb_rf_param_t rf_param);
static void isr_adv(void *param, radio_isr_cb_rf_param_t rf_param);
static void isr_tx(void *param);
static void isr_rx(void *param);
static inline int isr_rx_pdu(struct lll_adv *lll, uint8_t devmatch_ok, uint8_t devmatch_id,
			     uint8_t irkmatch_ok, uint8_t irkmatch_id, uint8_t rssi_ready);
static void isr_done(void *param);

static bool lll_adv_connect_ind_check(struct lll_adv *lll, struct pdu_adv *ci, uint8_t tx_addr,
				      uint8_t *addr, uint8_t rx_addr, uint8_t *tgt_addr,
				      uint8_t devmatch_ok, uint8_t *rl_idx);
static inline bool isr_rx_ci_adva_check(uint8_t tx_addr, uint8_t *addr, struct pdu_adv *ci);
static inline bool isr_rx_ci_tgta_check(struct lll_adv *lll, uint8_t rx_addr, uint8_t *tgt_addr,
					struct pdu_adv *ci, uint8_t rl_idx);

int lll_adv_init(void)
{
	int err;

#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
#if (BT_CTLR_ADV_AUX_SET > 0)
	err = lll_adv_aux_init();
	if (err) {
		return err;
	}
#endif /* BT_CTLR_ADV_AUX_SET > 0 */
#if IS_ENABLED(CONFIG_BT_CTLR_ADV_PERIODIC)
	err = lll_adv_sync_init();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC */
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_adv_reset(void)
{
	int err;

#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
#if (BT_CTLR_ADV_AUX_SET > 0)
	err = lll_adv_aux_reset();
	if (err) {
		return err;
	}
#endif /* BT_CTLR_ADV_AUX_SET > 0 */
#if IS_ENABLED(CONFIG_BT_CTLR_ADV_PERIODIC)
	err = lll_adv_sync_reset();
	if (err) {
		return err;
	}
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC */
#endif /* CONFIG_BT_CTLR_ADV_EXT */

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

void lll_adv_prepare(void *param)
{
	int err = lll_hfclock_on();
	LL_ASSERT(!err || err == -EINPROGRESS);

	err = lll_prepare(is_abort_cb, abort_cb, prepare_cb, 0, (struct lll_prepare_param *)param);
	LL_ASSERT(!err || err == -EINPROGRESS);
}

static int init_reset(void)
{
	lll_adv_pdu_init_reset();

	ble_adv_param.pRxQ = NULL;
	ble_adv_param.rxConfig.bAutoFlushIgnored = RADIO_RX_CONFIG_AUTO_FLUSH_IGNORED;
	ble_adv_param.rxConfig.bAutoFlushCrcErr = RADIO_RX_CONFIG_AUTO_FLUSH_CRC_ERR;
	ble_adv_param.rxConfig.bAutoFlushEmpty = RADIO_RX_CONFIG_AUTO_FLUSH_EMPTY;
	ble_adv_param.rxConfig.bIncludeLenByte = RADIO_RX_CONFIG_INCLUDE_LEN_BYTE;
	ble_adv_param.rxConfig.bIncludeCrc = RADIO_RX_CONFIG_INCLUDE_CRC;
	ble_adv_param.rxConfig.bAppendRssi = RADIO_RX_CONFIG_APPEND_RSSI;
	ble_adv_param.rxConfig.bAppendStatus = RADIO_RX_CONFIG_APPEND_STATUS;
	ble_adv_param.rxConfig.bAppendTimestamp = RADIO_RX_CONFIG_APPEND_TIMESTAMP;
	ble_adv_param.advConfig.advFilterPolicy = 0;
	ble_adv_param.advConfig.deviceAddrType = 1;
	ble_adv_param.advConfig.peerAddrType = 0;
	ble_adv_param.advConfig.bStrictLenFilter = 0;
	ble_adv_param.advConfig.chSel = IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2);
	ble_adv_param.advConfig.privIgnMode = 0;
	ble_adv_param.advConfig.rpaMode = 0;
	ble_adv_param.advLen = 0;
	ble_adv_param.scanRspLen = 0;
	ble_adv_param.pAdvData = NULL;
	ble_adv_param.pScanRspData = NULL;
	ble_adv_param.pDeviceAddress = NULL;
	ble_adv_param.pWhiteList = NULL;
	ble_adv_param.behConfig.scanRspEndType = 1;
	ble_adv_param.__dummy0 = 0;
	ble_adv_param.__dummy1 = 0;
	ble_adv_param.endTrigger.triggerType = TRIG_NEVER;
	ble_adv_param.endTrigger.bEnaCmd = 0;
	ble_adv_param.endTrigger.triggerNo = 0;
	ble_adv_param.endTrigger.pastTrig = 0;
	ble_adv_param.endTime = 0;

	memset(&ble_adv_output, 0, sizeof(ble_adv_output));

	cmd_ble5_adv.commandNo = CMD_BLE5_ADV;
	cmd_ble5_adv.status = IDLE;
	cmd_ble5_adv.pNextOp = NULL;
	cmd_ble5_adv.startTime = 0;
	cmd_ble5_adv.startTrigger.triggerType = TRIG_NOW;
	cmd_ble5_adv.startTrigger.bEnaCmd = 0;
	cmd_ble5_adv.startTrigger.triggerNo = 0;
	cmd_ble5_adv.startTrigger.pastTrig = 0;
	cmd_ble5_adv.condition.rule = COND_NEVER;
	cmd_ble5_adv.condition.nSkip = COND_ALWAYS;
	cmd_ble5_adv.channel = 0;
	cmd_ble5_adv.whitening.init = 0;
	cmd_ble5_adv.whitening.bOverride = 0;
	cmd_ble5_adv.phyMode.mainMode = 0;
	cmd_ble5_adv.phyMode.coding = 0;
	cmd_ble5_adv.rangeDelay = 0;
	cmd_ble5_adv.txPower = 0;
	cmd_ble5_adv.pParams = &ble_adv_param;
	cmd_ble5_adv.pOutput = &ble_adv_output;
	cmd_ble5_adv.tx20Power = 0;

	return 0;
}

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb)
{
#if defined(CONFIG_BT_PERIPHERAL)
	struct lll_adv *lll = curr;

	if (next == curr) {
		struct pdu_adv *pdu = lll_adv_data_curr_get(lll);
		if (pdu->type == PDU_ADV_TYPE_DIRECT_IND) {
			return 0;
		}
	} else if (lll->is_hdcd) {
		int err;

		/* wrap back after the pre-empter */
		*resume_cb = resume_prepare_cb;

		/* Retain HF clk */
		err = lll_hfclock_on();
		LL_ASSERT(err >= 0);

		return -EAGAIN;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	return -ECANCELED;
}

static void abort_cb(struct lll_prepare_param *prepare_param, void *param)
{
	int err;

	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* Perform event abort here.
		 * After event has been cleanly aborted, clean up resources
		 * and dispatch event done.
		 */
		radio_isr_set(isr_abort, param);
		radio_disable();
		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	err = lll_hfclock_off();
	LL_ASSERT(err >= 0);

	lll_done(param);
}

static int prepare_cb(struct lll_prepare_param *p)
{
	DEBUG_RADIO_START_A(1);

	struct lll_adv *lll = p->param;

#if defined(CONFIG_BT_PERIPHERAL)
	/* Check if stopped (on connection establishment- or disabled race
	 * between LLL and ULL.
	 * When connectable advertising is disabled in thread context, cancelled
	 * flag is set, and initiated flag is checked. Here, we avoid
	 * transmitting connectable advertising event if cancelled flag is set.
	 */
	if (unlikely(lll->conn && (lll->conn->periph.initiated || lll->conn->periph.cancelled))) {
		radio_isr_set(lll_isr_early_abort, lll);
		radio_disable();

		return 0;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	radio_reset();

#if defined(CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL)
	radio_tx_power_set(lll->tx_pwr_lvl);
#else
	radio_tx_power_set(RADIO_TXP_DEFAULT);
#endif /* CONFIG_BT_CTLR_TX_PWR_DYNAMIC_CONTROL */

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* TODO: if coded we use S8? */
	radio_phy_set(lll->phy_p, lll->phy_flags);
	radio_pkt_configure(RADIO_PKT_CONF_LENGTH_8BIT, PDU_AC_LEG_PAYLOAD_SIZE_MAX,
			    RADIO_PKT_CONF_PHY(lll->phy_p));
#else  /* !CONFIG_BT_CTLR_ADV_EXT */
	radio_phy_set(0, 0);
#endif /* !CONFIG_BT_CTLR_ADV_EXT */

	uint32_t aa = sys_cpu_to_le32(PDU_AC_ACCESS_ADDR);
	radio_aa_set((uint8_t *)&aa);
	radio_crc_configure(PDU_CRC_POLYNOMIAL, PDU_AC_CRC_IV);

	lll->chan_map_curr = lll->chan_map;
#if defined(CONFIG_BT_CTLR_ADV_EXT) && defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	struct pdu_adv *pdu_adv = chan_prepare(lll);
#else
	chan_prepare(lll);
#endif

#if defined(CONFIG_BT_HCI_MESH_EXT)
	_radio.mesh_adv_end_us = 0;
#endif /* CONFIG_BT_HCI_MESH_EXT */

#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (ull_filter_lll_rl_enabled()) {
		struct lll_filter *filter = ull_filter_lll_get(!!(lll->filter_policy));

		radio_filter_configure(filter->enable_bitmask, filter->addr_type_bitmask,
				       (uint8_t *)filter->bdaddr);
	} else
#endif /* CONFIG_BT_CTLR_PRIVACY */
		if (IS_ENABLED(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST) && lll->filter_policy) {
			/* Setup Radio Filter */
			struct lll_filter *fal = ull_filter_lll_get(true);

			radio_filter_configure(fal->enable_bitmask, fal->addr_type_bitmask,
					       (uint8_t *)fal->bdaddr);
		}

	uint32_t ticks_at_event = p->ticks_at_expire;
	struct ull_hdr *ull = HDR_LLL2ULL(lll);
	ticks_at_event += lll_event_offset_get(ull);

	uint32_t ticks_at_start = ticks_at_event;
	ticks_at_start += HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

	uint32_t remainder = p->remainder;
	uint32_t start_us = radio_tmr_start((RF_Op *)&cmd_ble5_adv, ticks_at_start, remainder);

	/* capture end of Tx-ed PDU, used to calculate HCTO. */
	radio_tmr_end_capture();

#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
	radio_gpio_pa_setup();
	radio_gpio_pa_lna_enable(start_us + radio_tx_ready_delay_get(0, 0) -
				 HAL_RADIO_GPIO_PA_OFFSET);
#else  /* !HAL_RADIO_GPIO_HAVE_PA_PIN */
	ARG_UNUSED(start_us);
#endif /* !HAL_RADIO_GPIO_HAVE_PA_PIN */

#if defined(CONFIG_BT_CTLR_XTAL_ADVANCED) &&                                                       \
	(EVENT_OVERHEAD_PREEMPT_US <= EVENT_OVERHEAD_PREEMPT_MIN_US)

	uint32_t overhead = lll_preempt_calc(
		ull, (TICKER_ID_ADV_BASE + ull_adv_lll_handle_get(lll)), ticks_at_event);
	/* check if preempt to start has changed */
	if (overhead) {
		LL_ASSERT_OVERHEAD(overhead);

		radio_isr_set(isr_abort, lll);
		radio_disable();

		return -ECANCELED;
	}
#endif /* CONFIG_BT_CTLR_XTAL_ADVANCED */

#if defined(CONFIG_BT_CTLR_ADV_EXT) && defined(CONFIG_BT_TICKER_EXT_EXPIRE_INFO)
	if (lll->aux) {
		/* fill in aux ptr in pdu */
		ull_adv_aux_lll_auxptr_fill(pdu_adv, lll);

		/* NOTE: as first primary channel PDU does not use remainder, the packet
		 * timer is started one tick in advance to start the radio with
		 * microsecond precision, hence compensate for the higher start_us value
		 * captured at radio start of the first primary channel PDU.
		 */
		lll->aux->ticks_pri_pdu_offset += 1U;
	}
#endif

	LL_ASSERT(!lll_prepare_done(lll));

	DEBUG_RADIO_START_A(1);

	return 0;
}

static struct pdu_adv *chan_prepare(struct lll_adv *lll)
{
	uint8_t chan = find_lsb_set(lll->chan_map_curr);
	LL_ASSERT(chan);
	lll->chan_map_curr &= (lll->chan_map_curr - 1);
	lll_radio_chan_set(36 + chan);

	/* FIXME: get latest only when primary PDU without Aux PDUs */
	uint8_t is_modified = 0U;
	struct pdu_adv *pdu_adv = lll_adv_pdu_latest_get(&lll->adv_data, &is_modified);
	LL_ASSERT(pdu_adv);

	uint8_t *device_address = pdu_adv->adv_ind.addr;
	cmd_ble5_adv.pParams->pDeviceAddress = (uint16_t *)device_address;
	cmd_ble5_adv.pParams->advLen = pdu_adv->len;
	cmd_ble5_adv.pParams->pAdvData = pdu_adv->adv_ind.data;

	if ((pdu_adv->type != PDU_ADV_TYPE_NONCONN_IND) &&
	    (!IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT) || (pdu_adv->type != PDU_ADV_TYPE_EXT_IND))) {
		struct pdu_adv *pdu_adv_scan_rsp =
			lll_adv_pdu_latest_get(&lll->scan_rsp, &is_modified);
		LL_ASSERT(pdu_adv_scan_rsp);

#if defined(CONFIG_BT_CTLR_PRIVACY)
		if (is_modified) {
			/* Copy the address from the adv packet we will send into the scan response
			 */
			memcpy(&pdu_adv_scan_rsp->scan_rsp.addr[0], &pdu->adv_ind.addr[0],
			       BDADDR_SIZE);
		}
#else
		ARG_UNUSED(is_modified);
#endif /* !CONFIG_BT_CTLR_PRIVACY */

		cmd_ble5_adv.pParams->scanRspLen = pdu_adv_scan_rsp->len;
		cmd_ble5_adv.pParams->pScanRspData = pdu_adv_scan_rsp->scan_rsp.data;

		radio_tmr_tifs_set(EVENT_IFS_US);
		radio_switch_complete_and_rx(0);
	} else {
		radio_switch_complete_and_disable();
	}

	/* setup Rx buffer */
	struct node_rx_pdu *node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

	cmd_ble5_adv.pParams->pRxQ = radio_get_rf_data_queue();
	radio_pkt_rx_set(node_rx->pdu);
	radio_isr_set(isr_adv, lll);

	return pdu_adv;
}

#if defined(CONFIG_BT_PERIPHERAL)
static int resume_prepare_cb(struct lll_prepare_param *p)
{
	struct ull_hdr *ull;

	ull = HDR_LLL2ULL(p->param);
	p->ticks_at_expire = ticker_ticks_now_get() - lll_event_offset_get(ull);
	p->remainder = 0;
	p->lazy = 0;

	return prepare_cb(p);
}

static void isr_abort_all(void *param, radio_isr_cb_rf_param_t rf_param)
{
	if (false == (rf_param.event_mask & RADIO_RF_EVENT_MASK_RX_DONE)) {
		return;
	}

	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, lll_disable};
	uint32_t ret;

	/* Clear radio status and events */
	lll_isr_status_reset();

	/* Disable any filter that was setup */
	radio_filter_disable();

	/* Current LLL radio event is done*/
	lll_isr_cleanup(param);

	/* Abort any LLL prepare/resume enqueued in pipeline */
	mfy.param = param;
	ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
	LL_ASSERT(!ret);
}
#endif /* CONFIG_BT_PERIPHERAL */

static void isr_abort(void *param, radio_isr_cb_rf_param_t rf_param)
{
	if (false == (rf_param.event_mask & RADIO_RF_EVENT_MASK_RX_DONE)) {
		return;
	}

	/* Clear radio status and events */
	lll_isr_status_reset();

	/* Disable any filter that was setup */
	radio_filter_disable();

	/* Current LLL radio event is done*/
	lll_isr_cleanup(param);
}

static void isr_adv(void *param, radio_isr_cb_rf_param_t rf_param)
{
	if (rf_param.event_mask & RADIO_RF_EVENT_MASK_TX_DONE) {
		if (BLE_DONE_SCAN_RSP !=
		    RF_getCmdOp(rf_param.handle, rf_param.command_handle)->status) {
			isr_tx(param);
		}
	}

	if (rf_param.event_mask & RADIO_RF_EVENT_MASK_RX_DONE) {
		isr_rx(param);
	}

	if (rf_param.event_mask & RADIO_RF_EVENT_MASK_CMD_DONE) {
		isr_done(param);
	}
}

static void isr_tx(void *param)
{
	struct node_rx_pdu *node_rx_prof;
#if defined(CONFIG_BT_CTLR_ADV_EXT)
	struct lll_adv *lll = param;
	uint8_t phy_p = lll->phy_p;
	uint8_t phy_flags = lll->phy_flags;
#else
	const uint8_t phy_p = 0U;
	const uint8_t phy_flags = 0U;
#endif

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_latency_capture();
	}

	/* setup tIFS switching */
	radio_tmr_tifs_set(EVENT_IFS_US);
	radio_switch_complete_and_tx(phy_p, 0, phy_p, phy_flags);

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_cputime_capture();
	}

#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (ull_filter_lll_rl_enabled()) {
		uint8_t count, *irks = ull_filter_lll_irks_get(&count);

		radio_ar_configure(count, irks, 0);
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */

	/* +/- 2us active clock jitter, +1 us hcto compensation */
	uint32_t hcto = radio_tmr_tifs_base_get() + EVENT_IFS_US + 4 + 1;
	hcto += radio_rx_chain_delay_get(phy_p, 0);
	hcto += addr_us_get(phy_p);
	hcto -= radio_tx_chain_delay_get(phy_p, 0);
	radio_tmr_hcto_configure(hcto);

	/* capture end of CONNECT_IND PDU, used for calculating first
	 * peripheral event.
	 */
	radio_tmr_end_capture();

	if (IS_ENABLED(CONFIG_BT_CTLR_SCAN_REQ_RSSI) || IS_ENABLED(CONFIG_BT_CTLR_CONN_RSSI)) {
		radio_rssi_measure();
	}

#if defined(HAL_RADIO_GPIO_HAVE_LNA_PIN)
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		/* PA/LNA enable is overwriting packet end used in ISR
		 * profiling, hence back it up for later use.
		 */
		lll_prof_radio_end_backup();
	}

	radio_gpio_lna_setup();
	radio_gpio_pa_lna_enable(radio_tmr_tifs_base_get() + EVENT_IFS_US - 4 -
				 radio_tx_chain_delay_get(phy_p, 0) - HAL_RADIO_GPIO_LNA_OFFSET);
#endif /* HAL_RADIO_GPIO_HAVE_LNA_PIN */

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		/* NOTE: as scratch packet is used to receive, it is safe to
		 * generate profile event using rx nodes.
		 */
		lll_prof_reserve_send(node_rx_prof);
	}
}

static void isr_rx(void *param)
{
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_latency_capture();
	}

	uint8_t irkmatch_ok = 0U;
	uint8_t irkmatch_id = FILTER_IDX_NONE;
	uint8_t devmatch_ok = radio_filter_has_match();
	uint8_t devmatch_id = radio_filter_match_get();
	if (IS_ENABLED(CONFIG_BT_CTLR_PRIVACY)) {
		irkmatch_ok = radio_ar_has_match();
		irkmatch_id = radio_ar_match_get();
	}
	uint8_t rssi_ready = radio_rssi_is_ready();

	/* Clear radio status and events */
	lll_isr_status_reset();

	if (isr_rx_pdu(param, devmatch_ok, devmatch_id, irkmatch_ok, irkmatch_id, rssi_ready)) {
		return;
	}

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_send();
	}
}

static inline int isr_rx_pdu(struct lll_adv *lll, uint8_t devmatch_ok, uint8_t devmatch_id,
			     uint8_t irkmatch_ok, uint8_t irkmatch_id, uint8_t rssi_ready)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	/* An IRK match implies address resolution enabled */
	uint8_t rl_idx = irkmatch_ok ? ull_filter_lll_rl_irk_idx(irkmatch_id) : FILTER_IDX_NONE;
#else
	uint8_t rl_idx = FILTER_IDX_NONE;
#endif /* CONFIG_BT_CTLR_PRIVACY */

	struct node_rx_pdu *node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

	struct pdu_adv *pdu_rx = (void *)node_rx->pdu;
	struct pdu_adv *pdu_adv = lll_adv_data_curr_get(lll);

	uint8_t *addr = pdu_adv->adv_ind.addr;
	uint8_t tx_addr = pdu_adv->tx_addr;
	uint8_t rx_addr = pdu_adv->rx_addr;
	uint8_t *tgt_addr = NULL;

	if (pdu_adv->type == PDU_ADV_TYPE_DIRECT_IND) {
		tgt_addr = pdu_adv->direct_ind.tgt_addr;
	}

#if defined(CONFIG_BT_PERIPHERAL)
	/* NOTE: Do not accept CONNECT_IND if cancelled flag is set in thread
	 *       context when disabling connectable advertising. This is to
	 *       avoid any race in checking the initiated flags in thread mode
	 *       which is set here if accepting a connection establishment.
	 *
	 *       Under this race, peer central would get failed to establish
	 *       connection as the disconnect reason. This is an acceptable
	 *       outcome to keep the thread mode implementation simple when
	 *       disabling connectable advertising.
	 */
	if ((pdu_rx->type == PDU_ADV_TYPE_CONNECT_IND) &&
	    (pdu_rx->len == sizeof(struct pdu_adv_connect_ind)) && lll->conn &&
	    !lll->conn->periph.cancelled &&
	    lll_adv_connect_ind_check(lll, pdu_rx, tx_addr, addr, rx_addr, tgt_addr, devmatch_ok,
				      &rl_idx)) {
		struct node_rx_ftr *ftr;
		struct node_rx_pdu *rx;

		if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
			rx = ull_pdu_rx_alloc_peek(4);
		} else {
			rx = ull_pdu_rx_alloc_peek(3);
		}

		if (!rx) {
			return -ENOBUFS;
		}

		radio_isr_set(isr_abort_all, lll);
		radio_disable();

		if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
			lll_prof_cputime_capture();
		}

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
		if (rssi_ready) {
			lll->conn->rssi_latest = radio_rssi_get();
		}
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

		/* Stop further LLL radio events */
		lll->conn->periph.initiated = 1;

		rx = ull_pdu_rx_alloc();

		rx->hdr.type = NODE_RX_TYPE_CONNECTION;
		rx->hdr.handle = 0xffff;

		ftr = &(rx->hdr.rx_ftr);
		ftr->param = lll;
		ftr->ticks_anchor = radio_tmr_start_get();
		ftr->radio_end_us = radio_tmr_end_get() - radio_rx_chain_delay_get(0, 0);

#if defined(CONFIG_BT_CTLR_PRIVACY)
		ftr->rl_idx = irkmatch_ok ? rl_idx : FILTER_IDX_NONE;
#endif /* CONFIG_BT_CTLR_PRIVACY */

		if (IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2)) {
			ftr->extra = ull_pdu_rx_alloc();
		}

		ull_rx_put_sched(rx->hdr.link, rx);

		return 0;
#endif /* CONFIG_BT_PERIPHERAL */
	}

	return -EINVAL;
}

static void isr_done(void *param)
{
	/* Clear radio status and events */
	lll_isr_status_reset();

#if defined(CONFIG_BT_HCI_MESH_EXT)
	if (_radio.advertiser.is_mesh && !_radio.mesh_adv_end_us) {
		_radio.mesh_adv_end_us = radio_tmr_end_get();
	}
#endif /* CONFIG_BT_HCI_MESH_EXT */

	struct lll_adv *lll = param;

#if defined(CONFIG_BT_PERIPHERAL)
	if (!IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) && lll->is_hdcd && !lll->chan_map_curr) {
		lll->chan_map_curr = lll->chan_map;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	/* NOTE: Do not continue to connectable advertise if advertising is
	 *       being disabled, by checking the cancelled flag.
	 */
	if (lll->chan_map_curr &&
#if defined(CONFIG_BT_PERIPHERAL)
	    (!lll->conn || !lll->conn->periph.cancelled) &&
#endif /* CONFIG_BT_PERIPHERAL */
	    1) {
#if defined(CONFIG_BT_CTLR_ADV_EXT)
		struct pdu_adv *pdu = chan_prepare(lll);
#else
		chan_prepare(lll);
#endif

#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN) || defined(CONFIG_BT_CTLR_ADV_EXT)
		uint32_t start_us = radio_tmr_start_now(RADIO_TRX_TX);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
		struct lll_adv_aux *lll_aux;

		lll_aux = lll->aux;
		if (lll_aux) {
			(void)ull_adv_aux_lll_offset_fill(pdu, lll_aux->ticks_pri_pdu_offset,
							  lll_aux->us_pri_pdu_offset, start_us);
		}
#endif /* !CONFIG_BT_CTLR_ADV_EXT */

#if defined(HAL_RADIO_GPIO_HAVE_PA_PIN)
		radio_gpio_pa_setup();
		radio_gpio_pa_lna_enable(start_us + radio_tx_ready_delay_get(0, 0) -
					 HAL_RADIO_GPIO_PA_OFFSET);
#endif /* HAL_RADIO_GPIO_HAVE_PA_PIN */
#else  /* !(HAL_RADIO_GPIO_HAVE_PA_PIN || defined(CONFIG_BT_CTLR_ADV_EXT)) */
		radio_tmr_start_now((RF_Op *)&cmd_ble5_adv);
#endif /* !(HAL_RADIO_GPIO_HAVE_PA_PIN || defined(CONFIG_BT_CTLR_ADV_EXT)) */

		/* capture end of Tx-ed PDU, used to calculate HCTO. */
		radio_tmr_end_capture();

		return;
	}

	radio_filter_disable();

#if defined(CONFIG_BT_PERIPHERAL)
	if (!lll->is_hdcd)
#endif /* CONFIG_BT_PERIPHERAL */
	{
#if defined(CONFIG_BT_HCI_MESH_EXT)
		if (_radio.advertiser.is_mesh) {
			uint32_t err;

			err = isr_close_adv_mesh();
			if (err) {
				return 0;
			}
		}
#endif /* CONFIG_BT_HCI_MESH_EXT */
	}

#if defined(CONFIG_BT_CTLR_ADV_INDICATION)
	struct node_rx_hdr *node_rx = ull_pdu_rx_alloc_peek(3);

	if (node_rx) {
		ull_pdu_rx_alloc();

		/* TODO: add other info by defining a payload struct */
		node_rx->type = NODE_RX_TYPE_ADV_INDICATION;

		ull_rx_put_sched(node_rx->link, node_rx);
	}
#endif /* CONFIG_BT_CTLR_ADV_INDICATION */

#if defined(CONFIG_BT_CTLR_ADV_EXT) || defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
#if defined(CONFIG_BT_CTLR_ADV_EXT) && !defined(CONFIG_BT_CTLR_JIT_SCHEDULING)
	/* If no auxiliary PDUs scheduled, generate primary radio event done */
	if (!lll->aux)
#endif /* CONFIG_BT_CTLR_ADV_EXT && !CONFIG_BT_CTLR_JIT_SCHEDULING */

	{
		struct event_done_extra *extra;

		extra = ull_done_extra_type_set(EVENT_DONE_EXTRA_TYPE_ADV);
		LL_ASSERT(extra);
	}
#endif /* CONFIG_BT_CTLR_ADV_EXT || CONFIG_BT_CTLR_JIT_SCHEDULING */

	lll_isr_cleanup(param);
}

static bool lll_adv_connect_ind_check(struct lll_adv *lll, struct pdu_adv *ci, uint8_t tx_addr,
				      uint8_t *addr, uint8_t rx_addr, uint8_t *tgt_addr,
				      uint8_t devmatch_ok, uint8_t *rl_idx)
{
	/* LL 4.3.2: filter policy shall be ignored for directed adv */
	if (tgt_addr) {
#if defined(CONFIG_BT_CTLR_PRIVACY)
		return ull_filter_lll_rl_addr_allowed(ci->tx_addr, ci->connect_ind.init_addr,
						      rl_idx) &&
#else
		return (1) &&
#endif
		       isr_rx_ci_adva_check(tx_addr, addr, ci) &&
		       isr_rx_ci_tgta_check(lll, rx_addr, tgt_addr, ci, *rl_idx);
	}

#if defined(CONFIG_BT_CTLR_PRIVACY)
	return ((((lll->filter_policy & BT_LE_ADV_FP_FILTER_CONN_IND) == 0) &&
		 ull_filter_lll_rl_addr_allowed(ci->tx_addr, ci->connect_ind.init_addr, rl_idx)) ||
		(((lll->filter_policy & BT_LE_ADV_FP_FILTER_CONN_IND) != 0) &&
		 (devmatch_ok || ull_filter_lll_irk_in_fal(*rl_idx)))) &&
	       isr_rx_ci_adva_check(tx_addr, addr, ci);
#else
	return (((lll->filter_policy & BT_LE_ADV_FP_FILTER_CONN_IND) == 0) || (devmatch_ok)) &&
	       isr_rx_ci_adva_check(tx_addr, addr, ci);
#endif /* CONFIG_BT_CTLR_PRIVACY */
}

static inline bool isr_rx_ci_adva_check(uint8_t tx_addr, uint8_t *addr, struct pdu_adv *ci)
{
	return (tx_addr == ci->rx_addr) && !memcmp(addr, ci->connect_ind.adv_addr, BDADDR_SIZE);
}

static inline bool isr_rx_ci_tgta_check(struct lll_adv *lll, uint8_t rx_addr, uint8_t *tgt_addr,
					struct pdu_adv *ci, uint8_t rl_idx)
{
#if defined(CONFIG_BT_CTLR_PRIVACY)
	if (rl_idx != FILTER_IDX_NONE && lll->rl_idx != FILTER_IDX_NONE) {
		return rl_idx == lll->rl_idx;
	}
#endif /* CONFIG_BT_CTLR_PRIVACY */
	return (rx_addr == ci->tx_addr) &&
	       !memcmp(tgt_addr, ci->connect_ind.init_addr, BDADDR_SIZE);
}

#if defined(CONFIG_ZTEST)
uint32_t lll_adv_free_pdu_fifo_count_get(void)
{
	return MFIFO_AVAIL_COUNT_GET(pdu_free);
}

uint32_t lll_adv_pdu_mem_free_count_get(void)
{
	return mem_free_count_get(mem_pdu.free);
}
#endif /* CONFIG_ZTEST */