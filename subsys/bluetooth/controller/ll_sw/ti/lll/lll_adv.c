/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util_macro.h>

#include <driverlib/rf_ble_cmd.h>
#include "hal/cc13xx_cc26xx/radio/RFQueue.h"

#include "hal/ccm.h"
#include "hal/radio.h"

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

typedef struct RF_RX_DATA {
	dataQueue_t queue;
	uint8_t buffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(
		RADIO_RX_ENTRY_BUFFER_SIZE, RADIO_RX_BUFFER_SIZE)] __attribute__((aligned(4)));
} rf_rx_data_t;

#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
uint8_t adv_data[sizeof(((struct pdu_adv_com_ext_adv *)0)->ext_hdr_adi_adv_data)];
uint8_t scan_rsp_data[sizeof(((struct pdu_adv_com_ext_adv *)0)->ext_hdr_adi_adv_data)];
#else
uint8_t adv_data[sizeof((struct pdu_adv_adv_ind *)0)->data] = {0};
uint8_t scan_rsp_data[sizeof((struct pdu_adv_scan_rsp *)0)->data] = {0};
#endif
uint8_t adv_data_len = 0;
uint8_t scan_rsp_data_len = 0;

rf_rx_data_t rf_rx = {.queue = {NULL}, .buffer = {0}};

rfc_bleAdvPar_t ble_adv_param = {
	.pRxQ = &rf_rx.queue,
	.rxConfig.bAutoFlushIgnored = RADIO_RX_CONFIG_AUTO_FLUSH_IGNORED,
	.rxConfig.bAutoFlushCrcErr = RADIO_RX_CONFIG_AUTO_FLUSH_CRC_ERR,
	.rxConfig.bAutoFlushEmpty = RADIO_RX_CONFIG_AUTO_FLUSH_EMPTY,
	.rxConfig.bIncludeLenByte = RADIO_RX_CONFIG_INCLUDE_LEN_BYTE,
	.rxConfig.bIncludeCrc = RADIO_RX_CONFIG_INCLUDE_CRC,
	.rxConfig.bAppendRssi = RADIO_RX_CONFIG_APPEND_RSSI,
	.rxConfig.bAppendStatus = RADIO_RX_CONFIG_APPEND_STATUS,
	.rxConfig.bAppendTimestamp = RADIO_RX_CONFIG_APPEND_TIMESTAMP,
	.advConfig.advFilterPolicy = 0,
	.advConfig.deviceAddrType = 0,
	.advConfig.peerAddrType = 0,
	.advConfig.bStrictLenFilter = 0,
	.advConfig.chSel = IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2),
	.advConfig.privIgnMode = 0,
	.advConfig.rpaMode = 0,
	.advLen = 0,
	.scanRspLen = 0,
	.pAdvData = adv_data,
	.pScanRspData = scan_rsp_data,
	.pDeviceAddress = NULL,
	.pWhiteList = NULL,
	.behConfig.scanRspEndType = 0,
	.__dummy0 = 0,
	.__dummy1 = 0,
	.endTrigger.triggerType = TRIG_NEVER,
	.endTrigger.bEnaCmd = 0,
	.endTrigger.triggerNo = 0,
	.endTrigger.pastTrig = 0,
	.endTime = 0,
};

rfc_bleAdvOutput_t ble_adv_output = {0};

rfc_CMD_BLE5_ADV_t cmd_ble5_adv = {
	.commandNo = CMD_BLE5_ADV,
	.status = IDLE,
	.pNextOp = NULL,
	.startTime = 0,
	.startTrigger.triggerType = TRIG_NOW,
	.startTrigger.bEnaCmd = 0,
	.startTrigger.triggerNo = 0,
	.startTrigger.pastTrig = 0,
	.condition.rule = COND_NEVER,
	.condition.nSkip = COND_ALWAYS,
	.channel = 0,
	.whitening.init = 0,
	.whitening.bOverride = 0,
	.phyMode.mainMode = 0,
	.phyMode.coding = 0,
	.rangeDelay = 0,
	.txPower = 0,
	.pParams = &ble_adv_param,
	.pOutput = &ble_adv_output,
	.tx20Power = 0,
};

static int init_reset(void);

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
}

static int init_reset(void)
{
	RFQueue_defineQueue(&rf_rx.queue, rf_rx.buffer, sizeof(rf_rx.buffer),
			    RADIO_RX_ENTRY_BUFFER_SIZE, RADIO_RX_BUFFER_SIZE);
	return 0;
}

#if IS_ENABLED(CONFIG_ZTEST)
uint32_t lll_adv_free_pdu_fifo_count_get(void)
{
	return 0;
}

uint32_t lll_adv_pdu_mem_free_count_get(void)
{
	return 0;
}
#endif /* CONFIG_ZTEST */