#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/errno_private.h>
#include <zephyr/sys/util_macro.h>

#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_ble_mailbox.h>

#include "hal/ccm.h"
#include "hal/debug.h"
#include "hal/radio.h"
#include "hal/ticker.h"
#include "hal/cc13xx_cc26xx/radio/rf_queue.h"

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

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_adv);

#define RF_RX_ENTRY_BUFFER_SIZE (2)
// #define RF_RX_BUFFER_SIZE       (sizeof(struct pdu_adv_connect_ind) +
// RF_RX_ADDITIONAL_DATA_BYTES)
#define RF_RX_BUFFER_SIZE       (HAL_RADIO_PDU_LEN_MAX + RF_RX_ADDITIONAL_DATA_BYTES)

struct RF_RX_DATA_ADV {
	rfc_dataEntryGeneral_t *entry;
	dataQueue_t queue;
	uint8_t buffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(RF_RX_ENTRY_BUFFER_SIZE, RF_RX_BUFFER_SIZE)]
		__attribute__((aligned(4)));
} rf_rx_data_adv;

static rfc_bleAdvPar_t ble_adv_param;
static rfc_bleAdvOutput_t ble_adv_output;
static rfc_CMD_BLE5_ADV_t cmd_ble5_adv;

static uint8_t *adv_node_rx_pdu = NULL;
static void *adv_param = NULL;
static void *adv_abort_param = NULL;
#if defined(CONFIG_BT_PERIPHERAL)
static void *adv_abort_all_param = NULL;
#endif /* CONFIG_BT_PERIPHERAL */

static int init_reset(void);

static int is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
static void abort_cb(struct lll_prepare_param *prepare_param, void *param);
static void isr_adv_abort(RF_Handle rf_handle, RF_CmdHandle command_handle,
			  RF_EventMask event_mask);
static int prepare_cb(struct lll_prepare_param *p);
static struct pdu_adv *chan_prepare(struct lll_adv *lll);

#if defined(CONFIG_BT_PERIPHERAL)
static int resume_prepare_cb(struct lll_prepare_param *p);
static void isr_adv_abort_all(RF_Handle rf_handle, RF_CmdHandle command_handle,
			      RF_EventMask event_mask);
#endif /* CONFIG_BT_PERIPHERAL */

static void isr_adv(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);
static void isr_tx(void *param);
static void isr_rx(void *param);
static inline int isr_rx_pdu(struct lll_adv *lll);
static void isr_done(void *param);
static void isr_abort(void *param);

static bool lll_adv_connect_ind_check(struct lll_adv *lll, struct pdu_adv *ci, uint8_t tx_addr,
				      uint8_t *addr, uint8_t rx_addr, uint8_t *tgt_addr,
				      uint8_t *rl_idx);
static inline bool isr_rx_ci_adva_check(uint8_t tx_addr, uint8_t *addr, struct pdu_adv *ci);
static inline bool isr_rx_ci_tgta_check(struct lll_adv *lll, uint8_t rx_addr, uint8_t *tgt_addr,
					struct pdu_adv *ci, uint8_t rl_idx);

int lll_adv_init(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_adv_reset(void)
{
	int err;

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

	rf_rx_data_adv.entry = rf_queue_define_queue(&rf_rx_data_adv.queue, rf_rx_data_adv.buffer,
						     sizeof(rf_rx_data_adv.buffer),
						     RF_RX_ENTRY_BUFFER_SIZE, RF_RX_BUFFER_SIZE);

	ble_adv_param.pRxQ = &rf_rx_data_adv.queue;
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
	ble_adv_param.endTrigger.triggerType = TRIG_REL_START;
	ble_adv_param.endTrigger.bEnaCmd = 0;
	ble_adv_param.endTrigger.triggerNo = 0;
	ble_adv_param.endTrigger.pastTrig = 1;
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
	cmd_ble5_adv.channel = 37;
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
	/* NOTE: This is not a prepare being cancelled */
	if (!prepare_param) {
		/* Perform event abort here.
		 * After event has been cleanly aborted, clean up resources
		 * and dispatch event done.
		 */
		adv_abort_param = param;
		radio_disable(isr_adv_abort);
		return;
	}

	/* NOTE: Else clean the top half preparations of the aborted event
	 * currently in preparation pipeline.
	 */
	LL_ASSERT(lll_hfclock_off() >= 0);

	lll_done(param);
}

static void isr_adv_abort(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask)
{
	radio_isr(rf_handle, command_handle, event_mask);
	LL_ASSERT(adv_abort_param);

	if (event_mask & RADIO_RF_EVENT_MASK_CMD_DONE) {
		isr_abort(adv_abort_param);
	}
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
		radio_disable(lll_isr_early_abort);

		return 0;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	radio_reset();

	radio_tx_power_set(RADIO_TXP_DEFAULT);

	lll->chan_map_curr = lll->chan_map;

	chan_prepare(lll);

	uint32_t ticks_at_event = p->ticks_at_expire;
	struct ull_hdr *ull = HDR_LLL2ULL(lll);
	ticks_at_event += lll_event_offset_get(ull);

	uint32_t ticks_at_start = ticks_at_event;
	ticks_at_start += HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

	uint32_t remainder = p->remainder;

	LOG_DBG("tax %u tae %u tas %u rem %u", p->ticks_at_expire, ticks_at_event, ticks_at_start,
		remainder);
	radio_rf_op_start_tick((RF_Op *)&cmd_ble5_adv, ticks_at_start, remainder, isr_adv);

	LL_ASSERT(!lll_prepare_done(lll));

	DEBUG_RADIO_START_A(1);

	return 0;
}

static struct pdu_adv *chan_prepare(struct lll_adv *lll)
{
	uint8_t chan = find_lsb_set(lll->chan_map_curr);
	LL_ASSERT(chan);
	lll->chan_map_curr &= (lll->chan_map_curr - 1);
	cmd_ble5_adv.channel = 36 + chan;
	LOG_DBG("ch %u", cmd_ble5_adv.channel);

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

		cmd_ble5_adv.pParams->scanRspLen = pdu_adv_scan_rsp->len;
		cmd_ble5_adv.pParams->pScanRspData = pdu_adv_scan_rsp->scan_rsp.data;
	}

	/* setup Rx buffer */
	struct node_rx_pdu *node_rx = ull_pdu_rx_alloc_peek(1);
	LL_ASSERT(node_rx);

#warning "TODO: pass directly into rf_queue? (change to pointer queue)"
	adv_node_rx_pdu = node_rx->pdu;
	adv_param = lll;

#warning "TODO: calculate hcto"
	cmd_ble5_adv.pParams->endTime = 50000;
	ble_adv_param.endTrigger.triggerType = TRIG_REL_START;

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

static void isr_adv_abort_all(RF_Handle rf_handle, RF_CmdHandle command_handle,
			      RF_EventMask event_mask)
{
	radio_isr(rf_handle, command_handle, event_mask);
	LL_ASSERT(adv_abort_all_param);

	if ((event_mask & (RADIO_RF_EVENT_MASK_CMD_DONE | RADIO_RF_EVENT_MASK_CMD_STOPPED)) ==
	    false) {
		return;
	}

	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, lll_disable};

	/* Current LLL radio event is done*/
	lll_isr_cleanup(adv_abort_all_param);

	/* Abort any LLL prepare/resume enqueued in pipeline */
	mfy.param = adv_abort_all_param;
	uint32_t ret = mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_LLL, 1U, &mfy);
	LL_ASSERT(!ret);
}
#endif /* CONFIG_BT_PERIPHERAL */

static void isr_adv(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask)
{
	radio_isr(rf_handle, command_handle, event_mask);
	LL_ASSERT(adv_param);

	if (event_mask & RADIO_RF_EVENT_MASK_TX_DONE) {
		if (BLE_DONE_SCAN_RSP != RF_getCmdOp(rf_handle, command_handle)->status) {
			isr_tx(adv_param);
		}
	}

	if (event_mask & RADIO_RF_EVENT_MASK_RX_DONE) {
		isr_rx(adv_param);
	}

	if (event_mask & RADIO_RF_EVENT_MASK_CMD_DONE) {
		isr_done(adv_param);
	}

	if (event_mask & RADIO_RF_EVENT_MASK_CMD_STOPPED) {
		isr_abort(adv_param);
	}
}

static void isr_tx(void *param)
{
	struct node_rx_pdu *node_rx_prof;
	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_latency_capture();
		node_rx_prof = lll_prof_reserve();
	}

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_cputime_capture();
	}

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

	if (rf_rx_data_adv.entry->status == DATA_ENTRY_FINISHED) {
		if (rf_rx_data_adv.entry->config.type == DATA_ENTRY_TYPE_GEN) {
			uint8_t *data = &rf_rx_data_adv.entry->data;

			/* - PDU header
			 * - PDU body length (advert or data)
			 * - PDU body
			 * - CRC
			 * - //RSSI
			 * - //Channel (and bIgnore, bCrcErr)
			 * - //PHY mode (byte not present if ble4_cmd)
			 * - //Timestamp (32 bit)
			 */
			uint16_t data_index = 0;
			uint8_t a_d = data[data_index++];
			uint16_t data_size = data[data_index++] + 2;

			memcpy(adv_node_rx_pdu, data, data_size);
			data_index += data_size - 2;

			uint32_t crc = (data[data_index++] & 0xFF);
			crc |= ((data[data_index++] << 8) & 0xFF00);
			crc |= ((data[data_index++] << 16) & 0xFF0000);

			LOG_WRN("rx_entry | ad %u | ds %u | crc 0x%08X | rssi %i | ts %u |", a_d,
				data_size, crc, cmd_ble5_adv.pOutput->lastRssi,
				cmd_ble5_adv.pOutput->timeStamp);
			struct pdu_adv *pdu_adv = (struct pdu_adv *)data;
			LOG_WRN("pdu | type %u | rfu %u | ch %u | tx %u | rx %u | len %u |",
				pdu_adv->type, pdu_adv->rfu, pdu_adv->chan_sel, pdu_adv->tx_addr,
				pdu_adv->rx_addr, pdu_adv->len);
			LOG_HEXDUMP_WRN(data, data_size, "RX");
		}

		rf_rx_data_adv.entry = rf_queue_next_entry(rf_rx_data_adv.entry);
	} else {
		return;
	}

	if (isr_rx_pdu(param)) {
		return;
	}

	if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
		lll_prof_send();
	}
}

static inline int isr_rx_pdu(struct lll_adv *lll)
{
	uint8_t rl_idx = FILTER_IDX_NONE;

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
	    lll_adv_connect_ind_check(lll, pdu_rx, tx_addr, addr, rx_addr, tgt_addr, &rl_idx)) {
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

		adv_abort_all_param = lll;
		radio_disable(isr_adv_abort_all);

		if (IS_ENABLED(CONFIG_BT_CTLR_PROFILE_ISR)) {
			lll_prof_cputime_capture();
		}

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
		lll->conn->rssi_latest = cmd_ble5_adv.pOutput->lastRssi;
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

		/* Stop further LLL radio events */
		lll->conn->periph.initiated = 1;

		rx = ull_pdu_rx_alloc();

		rx->hdr.type = NODE_RX_TYPE_CONNECTION;
		rx->hdr.handle = 0xffff;

		ftr = &(rx->hdr.rx_ftr);
		ftr->param = lll;
		// ftr->ticks_anchor = radio_tmr_start_get();
		// ftr->radio_end_us = radio_tmr_end_get() - radio_rx_chain_delay_get(0, 0);

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
	struct lll_adv *lll = param;

#if defined(CONFIG_BT_PERIPHERAL)
	if (!IS_ENABLED(CONFIG_BT_CTLR_LOW_LAT) && lll->is_hdcd && !lll->chan_map_curr) {
		lll->chan_map_curr = lll->chan_map;
	}
#endif /* CONFIG_BT_PERIPHERAL */

	/* NOTE: Do not continue to connectable advertise if advertising is
	 *       being disabled, by checking the cancelled flag.
	 */
	if (lll->chan_map_curr
#if defined(CONFIG_BT_PERIPHERAL)
	    && (!lll->conn || !lll->conn->periph.cancelled)
#endif /* CONFIG_BT_PERIPHERAL */
	) {

		chan_prepare(lll);
		radio_rf_op_start_now((RF_Op *)&cmd_ble5_adv, isr_adv);

		return;
	}

	lll_isr_cleanup(param);
}

static void isr_abort(void *param)
{
	/* Current LLL radio event is done*/
	lll_isr_cleanup(param);
}

static bool lll_adv_connect_ind_check(struct lll_adv *lll, struct pdu_adv *ci, uint8_t tx_addr,
				      uint8_t *addr, uint8_t rx_addr, uint8_t *tgt_addr,
				      uint8_t *rl_idx)
{
	if (tgt_addr) {
		return isr_rx_ci_adva_check(tx_addr, addr, ci) &&
		       isr_rx_ci_tgta_check(lll, rx_addr, tgt_addr, ci, *rl_idx);
	}

	return isr_rx_ci_adva_check(tx_addr, addr, ci);
}

static inline bool isr_rx_ci_adva_check(uint8_t tx_addr, uint8_t *addr, struct pdu_adv *ci)
{
	return (tx_addr == ci->rx_addr) && !memcmp(addr, ci->connect_ind.adv_addr, BDADDR_SIZE);
}

static inline bool isr_rx_ci_tgta_check(struct lll_adv *lll, uint8_t rx_addr, uint8_t *tgt_addr,
					struct pdu_adv *ci, uint8_t rl_idx)
{
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