#include <stdint.h>

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/sys/byteorder.h>

#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_ble_mailbox.h>

#include "hal/ccm.h"
#include "hal/cntr.h"
#include "hal/debug.h"
#include "hal/radio.h"
#include "hal/ticker.h"
#include "hal/cc13xx_cc26xx/radio/rf_queue.h"

#include "util/mem.h"
#include "util/memq.h"
#include "util/mayfly.h"
#include "util/util.h"

#include "ticker/ticker.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll_chan.h"
#include "lll_conn.h"
#include "lll_clock.h"
#include "lll_peripheral.h"
#include "lll_vendor.h"

#include "lll_internal.h"
#include "lll_conn_internal.h"
#include "lll_tim_internal.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_peripheral);

#define RF_RX_SLAVE_CONFIG_AUTO_FLUSH_IGNORED (0)
#define RF_RX_SLAVE_CONFIG_AUTO_FLUSH_CRC_ERR (0)
#define RF_RX_SLAVE_CONFIG_AUTO_FLUSH_EMPTY   (0)

static rfc_ble5SlavePar_t ble_slave_param;
static rfc_bleMasterSlaveOutput_t ble_slave_output;
static rfc_CMD_BLE5_SLAVE_t cmd_ble5_slave;

static struct lll_conn *conn_param = NULL;

static void init_reset(void);
static int prepare_cb(struct lll_prepare_param *p);
static void isr_done(void *param);

int lll_periph_init(void)
{
	init_reset();
	return 0;
}

int lll_periph_reset(void)
{
	init_reset();
	return 0;
}

void lll_periph_prepare(void *param)
{
	int err = lll_hfclock_on();
	LL_ASSERT(err >= 0);

	struct lll_prepare_param *p = (struct lll_prepare_param *)param;
	struct lll_conn *lll_conn = (struct lll_conn *)p->param;

	/* Accumulate window widening */
	lll_conn->periph.window_widening_prepare_us +=
		lll_conn->periph.window_widening_periodic_us * (p->lazy + 1);
	if (lll_conn->periph.window_widening_prepare_us > lll_conn->periph.window_widening_max_us) {
		lll_conn->periph.window_widening_prepare_us =
			lll_conn->periph.window_widening_max_us;
	}

	/* Invoke common pipeline handling of prepare */
	err = lll_prepare(lll_is_abort_cb, lll_conn_abort_cb, prepare_cb, 0, p);
	LL_ASSERT(!err || err == -EINPROGRESS);
}

void lll_isr_peripheral(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));
	radio_isr(rf_handle, command_handle, event_mask);
	LL_ASSERT(conn_param);

	if (event_mask & RADIO_RF_EVENT_MASK_RX_DONE) {
#if defined(CONFIG_BT_TI_LLL_PACKET_DEBUG)
		LOG_DBG("rx_entry | rssi %i | ts %u |", cmd_ble5_slave.pOutput->lastRssi,
			cmd_ble5_slave.pOutput->timeStamp);
#endif /* CONFIG_BT_TI_LLL_PACKET_DEBUG */
		lll_conn_isr_rx(conn_param);
	}

	if (event_mask & RADIO_RF_EVENT_MASK_TX_DONE) {
#if defined(CONFIG_BT_TI_LLL_PACKET_DEBUG)
		LOG_DBG("tx_entry | rssi %i | ts %u |", cmd_ble5_slave.pOutput->lastRssi,
			cmd_ble5_slave.pOutput->timeStamp);
#endif /* CONFIG_BT_TI_LLL_PACKET_DEBUG */
		lll_conn_isr_tx(conn_param);
	}

#if defined(CONFIG_BT_TI_LLL_PACKET_DEBUG)
	if (event_mask & (RADIO_RF_EVENT_MASK_TX_DONE | RADIO_RF_EVENT_MASK_RX_DONE)) {
		LOG_DBG("\r\n"
			"nTx %u\r\n"
			"nTxAck %u\r\n"
			"nTxCtrl %u\r\n"
			"nTxCtrlAck %u\r\n"
			"nTxCtrlAckAck %u\r\n"
			"nTxRetrans %u\r\n"
			"nTxEntryDone %u\r\n"
			"nRxOk %u\r\n"
			"nRxCtrl %u\r\n"
			"nRxCtrlAck %u\r\n"
			"nRxNok %u\r\n"
			"nRxIgnored %u\r\n"
			"nRxEmpty %u\r\n"
			"nRxBufFull %u\r\n"
			"bTimeStampValid %u\r\n"
			"bLastCrcErr %u\r\n"
			"bLastIgnored %u\r\n"
			"bLastEmpty %u\r\n"
			"bLastCtrl %u\r\n"
			"bLastMd %u\r\n"
			"bLastAck %u",
			cmd_ble5_slave.pOutput->nTx, cmd_ble5_slave.pOutput->nTxAck,
			cmd_ble5_slave.pOutput->nTxCtrl, cmd_ble5_slave.pOutput->nTxCtrlAck,
			cmd_ble5_slave.pOutput->nTxCtrlAckAck, cmd_ble5_slave.pOutput->nTxRetrans,
			cmd_ble5_slave.pOutput->nTxEntryDone, cmd_ble5_slave.pOutput->nRxOk,
			cmd_ble5_slave.pOutput->nRxCtrl, cmd_ble5_slave.pOutput->nRxCtrlAck,
			cmd_ble5_slave.pOutput->nRxNok, cmd_ble5_slave.pOutput->nRxIgnored,
			cmd_ble5_slave.pOutput->nRxEmpty, cmd_ble5_slave.pOutput->nRxBufFull,
			cmd_ble5_slave.pOutput->pktStatus.bTimeStampValid,
			cmd_ble5_slave.pOutput->pktStatus.bLastCrcErr,
			cmd_ble5_slave.pOutput->pktStatus.bLastIgnored,
			cmd_ble5_slave.pOutput->pktStatus.bLastEmpty,
			cmd_ble5_slave.pOutput->pktStatus.bLastCtrl,
			cmd_ble5_slave.pOutput->pktStatus.bLastMd,
			cmd_ble5_slave.pOutput->pktStatus.bLastAck);
	}
#endif

	if (event_mask & (RADIO_RF_EVENT_MASK_CMD_DONE | RADIO_RF_EVENT_MASK_CMD_STOPPED)) {
        isr_done(conn_param);
	}
}

static void init_reset(void)
{
	ble_slave_param.pRxQ = lll_conn_get_rf_rx_queue();
	ble_slave_param.pTxQ = lll_conn_get_rf_tx_queue();
	ble_slave_param.rxConfig.bAutoFlushIgnored = RF_RX_SLAVE_CONFIG_AUTO_FLUSH_IGNORED;
	ble_slave_param.rxConfig.bAutoFlushCrcErr = RF_RX_SLAVE_CONFIG_AUTO_FLUSH_CRC_ERR;
	ble_slave_param.rxConfig.bAutoFlushEmpty = RF_RX_SLAVE_CONFIG_AUTO_FLUSH_EMPTY;
	ble_slave_param.rxConfig.bIncludeLenByte = RADIO_RX_CONFIG_INCLUDE_LEN_BYTE;
	ble_slave_param.rxConfig.bIncludeCrc = RADIO_RX_CONFIG_INCLUDE_CRC;
	ble_slave_param.rxConfig.bAppendRssi = RADIO_RX_CONFIG_APPEND_RSSI;
	ble_slave_param.rxConfig.bAppendStatus = RADIO_RX_CONFIG_APPEND_STATUS;
	ble_slave_param.rxConfig.bAppendTimestamp = RADIO_RX_CONFIG_APPEND_TIMESTAMP;
	ble_slave_param.seqStat.lastRxSn = 0;
	ble_slave_param.seqStat.lastTxSn = 0;
	ble_slave_param.seqStat.nextTxSn = 0;
	ble_slave_param.seqStat.bFirstPkt = 0;
	ble_slave_param.seqStat.bAutoEmpty = 0;
	ble_slave_param.seqStat.bLlCtrlTx = 0;
	ble_slave_param.seqStat.bLlCtrlAckRx = 0;
	ble_slave_param.seqStat.bLlCtrlAckPending = 0;
	ble_slave_param.maxNack = 0;
	ble_slave_param.maxPkt = 0;
	ble_slave_param.accessAddress = 0;
	ble_slave_param.crcInit0 = 0;
	ble_slave_param.crcInit1 = 0;
	ble_slave_param.crcInit2 = 0;
	ble_slave_param.timeoutTrigger.triggerType = TRIG_REL_START;
	ble_slave_param.timeoutTrigger.bEnaCmd = 0;
	ble_slave_param.timeoutTrigger.triggerNo = 0;
	ble_slave_param.timeoutTrigger.pastTrig = 1;
	ble_slave_param.timeoutTime = 0;
	ble_slave_param.maxRxPktLen = PDU_DC_CTRL_RX_SIZE_MAX;
	ble_slave_param.maxLenLowRate = 0;
	ble_slave_param.__dummy0 = 0;
	ble_slave_param.endTrigger.triggerType = TRIG_REL_START;
	ble_slave_param.endTrigger.bEnaCmd = 0;
	ble_slave_param.endTrigger.triggerNo = 0;
	ble_slave_param.endTrigger.pastTrig = 1;
	ble_slave_param.endTime = 0;

	memset(&ble_slave_output, 0, sizeof(ble_slave_output));

	cmd_ble5_slave.commandNo = CMD_BLE5_SLAVE;
	cmd_ble5_slave.status = IDLE;
	cmd_ble5_slave.pNextOp = NULL;
	cmd_ble5_slave.startTime = 0;
	cmd_ble5_slave.startTrigger.triggerType = TRIG_NOW;
	cmd_ble5_slave.startTrigger.bEnaCmd = 0;
	cmd_ble5_slave.startTrigger.triggerNo = 0;
	cmd_ble5_slave.startTrigger.pastTrig = 0;
	cmd_ble5_slave.condition.rule = COND_NEVER;
	cmd_ble5_slave.condition.nSkip = COND_ALWAYS;
	cmd_ble5_slave.channel = 0;
	cmd_ble5_slave.whitening.init = 0;
	cmd_ble5_slave.whitening.bOverride = 0;
	cmd_ble5_slave.phyMode.mainMode = 0;
	cmd_ble5_slave.phyMode.coding = 0;
	cmd_ble5_slave.rangeDelay = 0;
	cmd_ble5_slave.txPower = 0;
	cmd_ble5_slave.pParams = &ble_slave_param;
	cmd_ble5_slave.pOutput = &ble_slave_output;
	cmd_ble5_slave.tx20Power = 0;
}

static int prepare_cb(struct lll_prepare_param *p)
{
	DEBUG_RADIO_START_S(1);

	conn_param = (struct lll_conn *)p->param;

	/* Check if stopped (on disconnection between prepare and pre-empt)
	 */
	if (unlikely(conn_param->handle == 0xFFFF)) {
		radio_disable(lll_isr_early_abort);
		return 0;
	}

	/* Reset connection event global variables */
	lll_conn_prepare_reset();

	/* Calculate the current event latency */
	conn_param->latency_event = conn_param->latency_prepare + p->lazy;

	/* Calculate the current event counter value */
	uint16_t event_counter = conn_param->event_counter + conn_param->latency_event;

	/* Update event counter to next value */
	conn_param->event_counter = (event_counter + 1);

	/* Reset accumulated latencies */
	conn_param->latency_prepare = 0;

	if (conn_param->data_chan_sel) {
#if defined(CONFIG_BT_CTLR_CHAN_SEL_2)
		cmd_ble5_slave.channel =
			lll_chan_sel_2(event_counter, conn_param->data_chan_id,
				       &conn_param->data_chan_map[0], conn_param->data_chan_count);
#else  /* !CONFIG_BT_CTLR_CHAN_SEL_2 */
		cmd_ble5_slave.channel = 0;
		LL_ASSERT(cmd_ble5_slave.channel);
#endif /* !CONFIG_BT_CTLR_CHAN_SEL_2 */
	} else {
		cmd_ble5_slave.channel =
			lll_chan_sel_1(&conn_param->data_chan_use, conn_param->data_chan_hop,
				       conn_param->latency_event, &conn_param->data_chan_map[0],
				       conn_param->data_chan_count);
	}

	/* current window widening */
	conn_param->periph.window_widening_event_us +=
		conn_param->periph.window_widening_prepare_us;
	conn_param->periph.window_widening_prepare_us = 0;
	if (conn_param->periph.window_widening_event_us >
	    conn_param->periph.window_widening_max_us) {
		conn_param->periph.window_widening_event_us =
			conn_param->periph.window_widening_max_us;
	}

	/* current window size */
	conn_param->periph.window_size_event_us += conn_param->periph.window_size_prepare_us;
	conn_param->periph.window_size_prepare_us = 0;

	/* Ensure that empty flag reflects the state of the Tx queue, as a
	 * peripheral if this is the first connection event and as no prior PDU
	 * is transmitted, an incorrect acknowledgment by peer should not
	 * dequeue a PDU that has not been transmitted on air.
	 */
	if (!conn_param->empty) {
		/* Check for any Tx PDU at the head of the queue */
		if (!memq_peek(conn_param->memq_tx.head, conn_param->memq_tx.tail, NULL)) {
			/* Update empty flag to reflect that no valid non-empty
			 * PDU was transmitted prior to this connection event.
			 */
			conn_param->empty = 1U;
		}
	}

	/* Start setting up Radio h/w */
	radio_reset();
	radio_tx_power_set(RADIO_TXP_DEFAULT);

	cmd_ble5_slave.pParams->accessAddress =
		(((uint32_t)conn_param->access_addr[3] << 24) & 0xFF000000) |
		(((uint32_t)conn_param->access_addr[2] << 16) & 0xFF0000) |
		(((uint32_t)conn_param->access_addr[1] << 8) & 0xFF00) |
		((uint32_t)conn_param->access_addr[0] & 0xFF);

#warning "PDU_CRC_POLYNOMIAL not used?"
	cmd_ble5_slave.pParams->crcInit0 = conn_param->crc_init[0];
	cmd_ble5_slave.pParams->crcInit1 = conn_param->crc_init[1];
	cmd_ble5_slave.pParams->crcInit2 = conn_param->crc_init[2];

	lll_conn_rx_pkt_set(conn_param);

	uint32_t ticks_at_event = p->ticks_at_expire;
	struct ull_hdr *ull = HDR_LLL2ULL(conn_param);
	ticks_at_event += lll_event_offset_get(ull);

	uint32_t ticks_at_start = ticks_at_event;
	ticks_at_start += HAL_TICKER_US_TO_TICKS(EVENT_OVERHEAD_START_US);

#warning "TODO: calculate hcto"
	cmd_ble5_slave.pParams->timeoutTime = 0;
	cmd_ble5_slave.pParams->timeoutTrigger.triggerType = TRIG_NEVER;
	cmd_ble5_slave.pParams->endTime = 0;
	cmd_ble5_slave.pParams->endTrigger.triggerType = TRIG_NEVER;

#warning "TODO: calculate start time"
	radio_rf_op_start_tick((RF_Op *)&cmd_ble5_slave, ticks_at_start, p->remainder,
			       lll_isr_peripheral);
	// radio_rf_op_start_now((RF_Op *)&cmd_ble5_slave, lll_isr_peripheral);

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	radio_rssi_measure();
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

	LL_ASSERT(!lll_prepare_done(conn_param));

	DEBUG_RADIO_START_S(1);

	return 0;
}

static void isr_done(void *param)
{
	struct event_done_extra *e = ull_event_done_extra_get();
	LL_ASSERT(e);

	e->type = EVENT_DONE_EXTRA_TYPE_CONN;
	e->trx_cnt = cmd_ble5_slave.pOutput->nRxOk;
	e->crc_valid = !cmd_ble5_slave.pOutput->pktStatus.bLastCrcErr;

#if defined(CONFIG_BT_PERIPHERAL)
	if (e->trx_cnt) {
		struct lll_conn *lll = (struct lll_conn *)param;

		if (lll->role) {
#if defined(CONFIG_BT_CTLR_PHY)
			uint32_t preamble_to_addr_us = addr_us_get(lll->phy_rx);
#else  /* !CONFIG_BT_CTLR_PHY */
			uint32_t preamble_to_addr_us = addr_us_get(0);
#endif /* !CONFIG_BT_CTLR_PHY */

#warning "TODO: Figure this out"
			e->drift.start_to_address_actual_us = HAL_TICKER_TICKS_TO_US(
				cmd_ble5_slave.pOutput->timeStamp - cmd_ble5_slave.startTime);
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