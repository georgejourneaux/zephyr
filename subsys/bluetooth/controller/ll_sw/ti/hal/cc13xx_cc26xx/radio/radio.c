/*
 * Copyright (c) 2016 - 2020 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <soc.h>

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <driverlib/rf_data_entry.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/osc.h>
#include <driverlib/prcm.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>

#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rfc.h>
#include <rf_patches/rf_patch_cpe_multi_protocol.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>

#include "util/mem.h"

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/cntr.h"
#include "hal/debug.h"
#include "hal/radio.h"
#include "hal/radio_df.h"
#include "hal/swi.h"
#include "hal/ticker.h"
#include "hal/cc13xx_cc26xx/ll_irqs.h"

#include "ll_sw/pdu_df.h"
#include "lll/pdu_vendor.h"
#include "ll_sw/pdu.h"

#include "radio_internal.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_radio);

/* CTEINLINE S0_MASK for periodic advertising PUDs. It allows to accept all types of extended
 * advertising PDUs to have CTE included.
 */
#define DF_S0_ALLOW_ALL_PER_ADV_PDU 0x0
/* CTEINLINE S0_MASK for data channel PDUs. It points to CP bit in S0 byte to check if is it set
 * to 0x1. In that is true then S1 byte (CTEInfo) is considered as present in a PDU.
 */
#define DF_S0_MASK_CP_BIT_IN_DATA_CHANNEL_PDU 0x20

/* Modifying these values could open a portal to The Upside-down */
#define CC13XX_CC26XX_INCLUDE_LEN_BYTE 1
#define CC13XX_CC26XX_INCLUDE_CRC 1
#define CC13XX_CC26XX_APPEND_RSSI 1
#define CC13XX_CC26XX_APPEND_TIMESTAMP 4
#define CC13XX_CC26XX_ADDITIONAL_DATA_BYTES (0 + CC13XX_CC26XX_INCLUDE_LEN_BYTE + CC13XX_CC26XX_INCLUDE_CRC + \
	                                         CC13XX_CC26XX_APPEND_RSSI + CC13XX_CC26XX_APPEND_TIMESTAMP)

#define CC13XX_CC26XX_NUM_RX_BUF 2
#define CC13XX_CC26XX_RX_BUF_SIZE (HAL_RADIO_PDU_LEN_MAX + CC13XX_CC26XX_ADDITIONAL_DATA_BYTES)

#define CC13XX_CC26XX_NUM_TX_BUF 2
#define CC13XX_CC26XX_TX_BUF_SIZE HAL_RADIO_PDU_LEN_MAX

#define RF_EVENT_MASK (RF_EventLastCmdDone | RF_EventTxDone | RF_EventRxEntryDone | RF_EventRxEmpty | RF_EventRxOk | RF_EventRxNOk)

#define TX_MARGIN 0
#define RX_MARGIN 8
#define Rx_OVHD 32 /* Rx overhead, depends on PHY type */
#define MIN_CMD_TIME 400 /* Minimum interval for a delayed radio cmd */

typedef struct isr_radio_param {
	RF_Handle h;
	RF_CmdHandle ch;
	RF_EventMask e;
} isr_radio_param_t;

typedef struct ble_cc13xx_cc26xx_data {
	uint8_t device_address[6];
	uint32_t access_address;
	uint32_t polynomial;
	uint32_t iv;
	uint8_t whiten;
	uint16_t chan;
#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
	uint8_t adv_data[sizeof(
		((struct pdu_adv_com_ext_adv *)0)->ext_hdr_adi_adv_data)];
#else
	uint8_t adv_data[sizeof((struct pdu_adv_adv_ind *)0)->data];
#endif
	uint8_t adv_data_len;

#if IS_ENABLED(CONFIG_BT_CTLR_ADV_EXT)
	uint8_t scan_rsp_data[sizeof(
		((struct pdu_adv_com_ext_adv *)0)->ext_hdr_adi_adv_data)];
#else
	uint8_t scan_rsp_data[sizeof((struct pdu_adv_scan_rsp *)0)->data];
#endif
	uint8_t scan_rsp_data_len;

	RF_EventMask rx_mask;

	dataQueue_t rx_queue;
	rfc_dataEntryPointer_t rx_entry[CC13XX_CC26XX_NUM_RX_BUF];
	uint8_t rx_data[CC13XX_CC26XX_NUM_RX_BUF]
		    [CC13XX_CC26XX_RX_BUF_SIZE] __aligned(4);

	dataQueue_t tx_queue;
	rfc_dataEntryPointer_t tx_entry[CC13XX_CC26XX_NUM_TX_BUF];
	uint8_t tx_data[CC13XX_CC26XX_NUM_TX_BUF]
		    [CC13XX_CC26XX_TX_BUF_SIZE] __aligned(4);

	RF_CmdHandle active_command_handle;

	RF_RatConfigCompare rat_hcto_compare;
	RF_RatHandle rat_hcto_handle;

#if defined(CONFIG_BT_CTLR_DEBUG_PINS)
	uint32_t window_begin_ticks;
	uint32_t window_duration_ticks;
	uint32_t window_interval_ticks;
#endif /* defined(CONFIG_BT_CTLR_DEBUG_PINS) */

	bool ignore_next_rx;
	bool ignore_next_tx;

	rfc_CMD_GET_FW_INFO_t cmd_get_fw_info;

	rfc_CMD_BLE5_RADIO_SETUP_t cmd_ble_radio_setup;

	rfc_CMD_FS_t cmd_set_frequency;

	rfc_CMD_BLE_ADV_t cmd_ble_adv;
	rfc_bleAdvPar_t cmd_ble_adv_param;
	rfc_bleAdvOutput_t cmd_ble_adv_output;

	rfc_CMD_BLE_GENERIC_RX_t cmd_ble_generic_rx;
	rfc_bleGenericRxPar_t cmd_ble_generic_rx_param;
	rfc_bleGenericRxOutput_t cmd_ble_generic_rx_output;

	rfc_CMD_NOP_t cmd_nop;
	rfc_CMD_CLEAR_RX_t cmd_clear_rx;
	rfc_CMD_BLE_ADV_PAYLOAD_t cmd_ble_adv_payload;

	rfc_CMD_BLE_SLAVE_t cmd_ble_slave;
	rfc_bleSlavePar_t cmd_ble_slave_param;
	rfc_bleMasterSlaveOutput_t cmd_ble_slave_output;
} ble_cc13xx_cc26xx_data_t;

typedef struct cc13xx_cc26xx_channel_table {
    uint16_t frequency;
    uint8_t whitening;
} cc13xx_cc26xx_channel_table_t;

static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_Mode RF_modeBle = {
	.rfMode = RF_MODE_MULTIPLE,
	.cpePatchFxn = &rf_patch_cpe_multi_protocol,
	.mcePatchFxn = 0,
	.rfePatchFxn = 0,
};

static uint32_t timer_aa = 0; /* AA (Access Address) timestamp saved value */
static uint32_t timer_aa_save = 0; /* save AA timestamp */
static uint32_t timer_ready = 0; /* radio ready for Tx/Rx timestamp */
static uint32_t timer_end = 0; /* Tx/Rx end timestamp saved value */
static uint32_t timer_end_save = 0; /* save Tx/Rx end timestamp */
static uint32_t timer_tifs = 0;

static uint32_t rtc_start;
static uint32_t rtc_diff_start_us;
static uint32_t skip_hcto;

static uint32_t isr_timer_aa = 0;
static uint32_t isr_timer_end = 0;
static uint32_t isr_latency = 0;
static rfc_bleRadioOp_t *next_radio_cmd;
static rfc_bleRadioOp_t *next_tx_radio_cmd;

static int8_t rssi;

static uint32_t rx_warmup = 0;
static uint32_t tx_warmup = 0;
static uint32_t next_warmup;

static uint32_t radio_trx = 0;

static radio_isr_cb_t isr_cb;
static void           *isr_cb_param;

static uint8_t *rx_packet_ptr;
static uint8_t *tx_packet_ptr;
static uint32_t payload_max_size;

static bool crc_valid = true;

static uint8_t MALIGN(4) _pkt_empty[PDU_EM_LL_SIZE_MAX];
static uint8_t MALIGN(4) _pkt_scratch[MAX((HAL_RADIO_PDU_LEN_MAX + 3),
				       PDU_AC_LL_SIZE_MAX)];

/* Overrides for CMD_BLE5_RADIO_SETUP */
static uint32_t pOverridesCommon[] = {
#if defined(CONFIG_BT_CTLR_DEBUG_PINS)
	/* See http://bit.ly/2vydFIa */
	(uint32_t)0x008F88B3,
	HW_REG_OVERRIDE(
		0x1110,
		0
		| RFC_DBELL_SYSGPOCTL_GPOCTL0_RATGPO1 /* RX */
		| RFC_DBELL_SYSGPOCTL_GPOCTL1_CPEGPO1 /* PA (default setting) */
		| RFC_DBELL_SYSGPOCTL_GPOCTL2_RATGPO2 /* BLE TX Window */
		| RFC_DBELL_SYSGPOCTL_GPOCTL3_RATGPO0 /* TX */
	),
#endif /* defined(CONFIG_BT_CTLR_DEBUG_PINS) */
	(uint32_t)0x00F388D3,
	/* Bluetooth 5: Set pilot tone length to 20 us Common */
	HW_REG_OVERRIDE(0x6024, 0x2E20),
	/* Bluetooth 5: Compensate for reduced pilot tone length */
	(uint32_t)0x01280263,
	/* Bluetooth 5: Default to no CTE. */
	HW_REG_OVERRIDE(0x5328, 0x0000), (uint32_t)0xFFFFFFFF
};

/* Overrides for CMD_BLE5_RADIO_SETUP */
static uint32_t pOverrides1Mbps[] = {
	/* Bluetooth 5: Set pilot tone length to 20 us */
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	/* Bluetooth 5: Compensate syncTimeadjust */
	(uint32_t)0x015302A3, (uint32_t)0xFFFFFFFF
};

/* Overrides for CMD_BLE5_RADIO_SETUP */
static uint32_t pOverrides2Mbps[] = {
	/* Bluetooth 5: Set pilot tone length to 20 us */
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	/* Bluetooth 5: Compensate syncTimeAdjust */
	(uint32_t)0x00F102A3, (uint32_t)0xFFFFFFFF
};

/* Overrides for CMD_BLE5_RADIO_SETUP */
static uint32_t pOverridesCoded[] = {
	/* Bluetooth 5: Set pilot tone length to 20 us */
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	/* Bluetooth 5: Compensate syncTimeadjust */
	(uint32_t)0x07A902A3,
	/* Rx: Set AGC reference level to 0x1B (default: 0x2E) */
	HW_REG_OVERRIDE(0x609C, 0x001B), (uint32_t)0xFFFFFFFF
};

static ble_cc13xx_cc26xx_data_t ble_cc13xx_cc26xx_data = {

	/* clang-format off */
	.cmd_set_frequency = {
		.commandNo = CMD_FS,
		.startTrigger = {
			.triggerType = TRIG_NOW,
		},
		.condition = {
			.rule = COND_NEVER,
		},
	},

	.cmd_ble_radio_setup = {
		.commandNo = CMD_BLE5_RADIO_SETUP,
		.startTrigger = {
			.triggerType = TRIG_NOW,
		},
		.condition = {
			.rule = COND_NEVER,
		},
		.config = {
			.biasMode = 0x1,
		},
		.txPower = 0x7217,
		.pRegOverrideCommon = pOverridesCommon,
		.pRegOverride1Mbps = pOverrides1Mbps,
		.pRegOverride2Mbps = pOverrides2Mbps,
		.pRegOverrideCoded = pOverridesCoded,
	},

	.cmd_ble_adv = {
		.pParams = (rfc_bleAdvPar_t *)&ble_cc13xx_cc26xx_data
				   .cmd_ble_adv_param,
		.pOutput = (rfc_bleAdvOutput_t *)&ble_cc13xx_cc26xx_data
				   .cmd_ble_adv_output,
		.condition = {
			.rule = TRIG_NEVER,
		},
	},

	.cmd_ble_adv_param = {
		.pRxQ = &ble_cc13xx_cc26xx_data.rx_queue,
		.rxConfig = {
			.bAutoFlushIgnored = true,
			.bAutoFlushCrcErr = true,
			/* SCAN_REQ will be discarded if true! */
			.bAutoFlushEmpty = false,
			.bIncludeLenByte = !!CC13XX_CC26XX_INCLUDE_LEN_BYTE,
			.bIncludeCrc = !!CC13XX_CC26XX_INCLUDE_CRC,
			.bAppendRssi = !!CC13XX_CC26XX_APPEND_RSSI,
			.bAppendTimestamp = !!CC13XX_CC26XX_APPEND_TIMESTAMP,
		},
		.advConfig = {
			/* Support Channel Selection Algorithm #2 */
			.chSel = IS_ENABLED(CONFIG_BT_CTLR_CHAN_SEL_2),
		},
		.endTrigger = {
			.triggerType = TRIG_NEVER,
		},
	},

	.cmd_ble_generic_rx = {
		.commandNo = CMD_BLE_GENERIC_RX,
		.condition = {
			.rule = COND_NEVER,
		},
		.pParams = (rfc_bleGenericRxPar_t *)&ble_cc13xx_cc26xx_data
			.cmd_ble_generic_rx_param,
		.pOutput = (rfc_bleGenericRxOutput_t *)&ble_cc13xx_cc26xx_data
			.cmd_ble_generic_rx_output,
	},

	.cmd_ble_generic_rx_param = {
		.pRxQ = &ble_cc13xx_cc26xx_data.rx_queue,
		.rxConfig = {
			.bAutoFlushIgnored = true,
			.bAutoFlushCrcErr = true,
			/* SCAN_REQ will be discarded if true! */
			.bAutoFlushEmpty = false,
			.bIncludeLenByte = !!CC13XX_CC26XX_INCLUDE_LEN_BYTE,
			.bIncludeCrc = !!CC13XX_CC26XX_INCLUDE_CRC,
			.bAppendRssi = !!CC13XX_CC26XX_APPEND_RSSI,
			.bAppendTimestamp = !!CC13XX_CC26XX_APPEND_TIMESTAMP,
		},
		.endTrigger = {
			.triggerType = TRIG_NEVER,
		},
	},

	.cmd_nop = {
		.commandNo = CMD_NOP,
		.condition = {
			.rule = COND_NEVER,
		},
	},

	.cmd_clear_rx = {
		.commandNo = CMD_CLEAR_RX,
		.pQueue = &ble_cc13xx_cc26xx_data.rx_queue,
	},
	.cmd_ble_adv_payload = {
		.commandNo = CMD_BLE_ADV_PAYLOAD,
		.pParams = &ble_cc13xx_cc26xx_data.cmd_ble_adv_param,
	},

	.cmd_ble_slave = {
		.commandNo = CMD_BLE_SLAVE,
		.pParams = &ble_cc13xx_cc26xx_data.cmd_ble_slave_param,
		.pOutput = &ble_cc13xx_cc26xx_data.cmd_ble_slave_output,
		.condition = {
			.rule = COND_NEVER,
		},
	},

	.cmd_ble_slave_param = {
		.pRxQ = &ble_cc13xx_cc26xx_data.rx_queue,
		.pTxQ = &ble_cc13xx_cc26xx_data.tx_queue,
		.rxConfig = {
			.bAutoFlushIgnored = true,
			.bAutoFlushCrcErr = true,
			.bAutoFlushEmpty = true,
			.bIncludeLenByte = !!CC13XX_CC26XX_INCLUDE_LEN_BYTE,
			.bIncludeCrc = !!CC13XX_CC26XX_INCLUDE_CRC,
			.bAppendRssi = !!CC13XX_CC26XX_APPEND_RSSI,
			.bAppendTimestamp = !!CC13XX_CC26XX_APPEND_TIMESTAMP,
		},
		.seqStat = {
			.bFirstPkt = true,
		},
		.timeoutTrigger = {
			.triggerType = TRIG_REL_START,
		},
		.endTrigger = {
			.triggerType = TRIG_NEVER,
		},
	},
	/* clang-format on */
};
static ble_cc13xx_cc26xx_data_t* driver_data = &ble_cc13xx_cc26xx_data;

static cc13xx_cc26xx_channel_table_t channel_table[] = {
    {2402, 0xE5}, {2404, 0xC0}, {2406, 0xC1}, {2408, 0xC2},
    {2410, 0xC3}, {2412, 0xC4}, {2414, 0xC5}, {2416, 0xC6},
    {2418, 0xC7}, {2420, 0xC8}, {2422, 0xC9}, {2424, 0xCA},
    {2426, 0xE6}, {2428, 0xCB}, {2430, 0xCC}, {2432, 0xCD},
    {2434, 0xCE}, {2436, 0xCF}, {2438, 0xD0}, {2440, 0xD1},
    {2442, 0xD2}, {2444, 0xD3}, {2446, 0xD4}, {2448, 0xD5},
    {2450, 0xD6}, {2452, 0xD7}, {2454, 0xD8}, {2456, 0xD9},
    {2458, 0xDA}, {2460, 0xDB}, {2462, 0xDC}, {2464, 0xDD},
    {2466, 0xDE}, {2468, 0xDF}, {2470, 0xE0}, {2472, 0xE1},
    {2474, 0xE2}, {2476, 0xE3}, {2478, 0xE4}, {2480, 0xE7}
};

static const RF_TxPowerTable_Entry RF_BLE_txPowerTable[] = {
	{ -20, RF_TxPowerTable_DEFAULT_PA_ENTRY(6, 3, 0, 2) },
	{ -15, RF_TxPowerTable_DEFAULT_PA_ENTRY(10, 3, 0, 3) },
	{ -10, RF_TxPowerTable_DEFAULT_PA_ENTRY(15, 3, 0, 5) },
	{ -5, RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 3, 0, 9) },
	{ 0, RF_TxPowerTable_DEFAULT_PA_ENTRY(19, 1, 0, 20) },
	{ 1, RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 1, 0, 20) },
	{ 2, RF_TxPowerTable_DEFAULT_PA_ENTRY(25, 1, 0, 25) },
	{ 3, RF_TxPowerTable_DEFAULT_PA_ENTRY(29, 1, 0, 28) },
	{ 4, RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 1, 0, 39) },
	{ 5, RF_TxPowerTable_DEFAULT_PA_ENTRY(23, 0, 0, 57) },
	RF_TxPowerTable_TERMINATION_ENTRY
};

static isr_radio_param_t radio_param;

static const char* const command_number_to_string(uint16_t command_number)
{
    switch(command_number) {
        case CMD_BLE_ADV: return "CMD_BLE_ADV";
        case CMD_NOP: return "CMD_NOP";
        case CMD_BLE_GENERIC_RX: return "CMD_BLE_GENERIC_RX";
        case CMD_BLE_SLAVE: return "CMD_BLE_SLAVE";

        default: break;
    }

	return "[unknown]";
}

static const char* const ble_status_to_string(uint16_t status)
{
	switch (status) {
	    case IDLE: return "Operation not started";
	    case PENDING: return "Start of command is pending";
	    case ACTIVE: return "Running";
	    case SKIPPED: return "Operation skipped due to condition in another command";

	    case DONE_OK: return "Operation ended normally";
	    case DONE_COUNTDOWN: return "Counter reached zero";
	    case DONE_RXERR: return "Operation ended with CRC error";
	    case DONE_TIMEOUT: return "Operation ended with timeout";
	    case DONE_STOPPED: return "Operation stopped after CMD_STOP command";
	    case DONE_ABORT: return "Operation aborted by CMD_ABORT command";
	    case DONE_FAILED: return "Scheduled immediate command failed";

	    case ERROR_PAST_START: return "The start trigger occurred in the past";
	    case ERROR_START_TRIG: return "Illegal start trigger parameter";
	    case ERROR_CONDITION: return "Illegal condition for next operation";
	    case ERROR_PAR: return "Error in a command specific parameter";
	    case ERROR_POINTER: return "Invalid pointer to next operation";
	    case ERROR_CMDID: return "Next operation has a command ID that is undefined or not a radio operation command";
	    case ERROR_WRONG_BG: return "FG level command not compatible with running BG level command";
	    case ERROR_NO_SETUP: return "Operation using Rx or Tx attempted without CMD_RADIO_SETUP";
	    case ERROR_NO_FS: return "Operation using Rx or Tx attempted without frequency synth configured";
	    case ERROR_SYNTH_PROG: return "Synthesizer calibration failed";
	    case ERROR_TXUNF: return "Tx underflow observed";
	    case ERROR_RXOVF: return "Rx overflow observed";
	    case ERROR_NO_RX: return "Attempted to access data from Rx when no such data was yet received";
	    case ERROR_PENDING: return "Command submitted in the future with another command at different level pending";

	    case BLE_DONE_OK: return "Operation ended normally";
	    case BLE_DONE_RXTIMEOUT: return "Timeout of first Rx of slave operation or end of scan window";
	    case BLE_DONE_NOSYNC: return "Timeout of subsequent Rx";
	    case BLE_DONE_RXERR: return "Operation ended because of receive error (CRC or other)";
	    case BLE_DONE_CONNECT: return "CONNECT_IND or AUX_CONNECT_RSP received or transmitted";
	    case BLE_DONE_MAXNACK: return "Maximum number of retransmissions exceeded";
	    case BLE_DONE_ENDED: return "Operation stopped after end trigger";
	    case BLE_DONE_ABORT: return "Operation aborted by command";
	    case BLE_DONE_STOPPED: return "Operation stopped after stop command";
	    case BLE_DONE_AUX: return "Operation ended after following aux pointer pointing far ahead";
	    case BLE_DONE_CONNECT_CHSEL0: return "CONNECT_IND received or transmitted; peer does not support channel selection algorithm #2";
	    case BLE_DONE_SCAN_RSP: return "SCAN_RSP or AUX_SCAN_RSP transmitted";
	    case BLE_ERROR_PAR: return "Illegal parameter";
	    case BLE_ERROR_RXBUF: return "No available Rx buffer (Advertiser, Scanner, Initiator)";
	    case BLE_ERROR_NO_SETUP: return "Operation using Rx or Tx attempted when not in BLE mode";
	    case BLE_ERROR_NO_FS: return "Operation using Rx or Tx attempted without frequency synth configured";
	    case BLE_ERROR_SYNTH_PROG: return "Synthesizer programming failed to complete on time";
	    case BLE_ERROR_RXOVF: return "Receiver overflowed during operation";
	    case BLE_ERROR_TXUNF: return "Transmitter underflowed during operation";
	    case BLE_ERROR_AUX: return "Calculated AUX pointer was too far into the future or in the past";

	    default: break;
	}

    return "Unknown status";
}

static void describe_event_mask(RF_EventMask e)
{
	if (e & RF_EventCmdDone)
		LOG_DBG("A radio operation command in a chain finished.");
	if (e & RF_EventLastCmdDone)
		LOG_DBG("Last radio operation command in a chain finished.");
	if (e & RF_EventFGCmdDone)
		LOG_DBG("A IEEE-mode radio operation command in a chain "
				"finished.");
	if (e & RF_EventLastFGCmdDone)
		LOG_DBG("A stand-alone IEEE-mode radio operation command or "
				"the last command in a chain finished.");
	if (e & RF_EventTxDone)
		LOG_DBG("Packet transmitted");
	if (e & RF_EventTXAck)
		LOG_DBG("ACK packet transmitted");
	if (e & RF_EventTxCtrl)
		LOG_DBG("Control packet transmitted");
	if (e & RF_EventTxCtrlAck)
		LOG_DBG("Acknowledgment received on a transmitted control "
				"packet");
	if (e & RF_EventTxCtrlAckAck)
		LOG_DBG("Acknowledgment received on a transmitted control "
				"packet, and acknowledgment transmitted for that "
				"packet");
	if (e & RF_EventTxRetrans)
		LOG_DBG("Packet retransmitted");
	if (e & RF_EventTxEntryDone)
		LOG_DBG("Tx queue data entry state changed to Finished");
	if (e & RF_EventTxBufferChange)
		LOG_DBG("A buffer change is complete");
	if (e & RF_EventPaChanged)
		LOG_DBG("The PA was reconfigured on the fly.");
	if (e & RF_EventRxOk)
		LOG_DBG("Packet received with CRC OK, payload, and not to "
				"be ignored");
	if (e & RF_EventRxNOk)
		LOG_DBG("Packet received with CRC error");
	if (e & RF_EventRxIgnored)
		LOG_DBG("Packet received with CRC OK, but to be ignored");
	if (e & RF_EventRxEmpty)
		LOG_DBG("Packet received with CRC OK, not to be ignored, "
				"no payload");
	if (e & RF_EventRxCtrl)
		LOG_DBG("Control packet received with CRC OK, not to be "
				"ignored");
	if (e & RF_EventRxCtrlAck)
		LOG_DBG("Control packet received with CRC OK, not to be "
				"ignored, then ACK sent");
	if (e & RF_EventRxBufFull)
		LOG_DBG("Packet received that did not fit in the Rx queue");
	if (e & RF_EventRxEntryDone)
		LOG_DBG("Rx queue data entry changing state to Finished");
	if (e & RF_EventDataWritten)
		LOG_DBG("Data written to partial read Rx buffer");
	if (e & RF_EventNDataWritten)
		LOG_DBG("Specified number of bytes written to partial read Rx "
				"buffer");
	if (e & RF_EventRxAborted)
		LOG_DBG("Packet reception stopped before packet was done");
	if (e & RF_EventRxCollisionDetected)
		LOG_DBG("A collision was indicated during packet reception");
	if (e & RF_EventModulesUnlocked)
		LOG_DBG("As part of the boot process, the CM0 has opened access "
				"to RF core modules and memories");
	if (e & RF_EventInternalError)
		LOG_DBG("Internal error observed");
	if (e & RF_EventMdmSoft)
		LOG_DBG("Synchronization word detected (MDMSOFT interrupt "
				"flag)");
	if (e & RF_EventCmdCancelled)
		LOG_DBG("Command canceled before it was started.");
	if (e & RF_EventCmdAborted)
		LOG_DBG("Abrupt command termination caused by RF_cancelCmd() or "
		       "RF_flushCmd().");
	if (e & RF_EventCmdStopped)
		LOG_DBG("Graceful command termination caused by RF_cancelCmd() "
				"or RF_flushCmd().");
	if (e & RF_EventRatCh)
		LOG_DBG("A user-programmable RAT channel triggered an event.");
	if (e & RF_EventError)
		LOG_DBG("Event flag used for error callback functions to "
				"indicate an error. See RF_Params::pErrCb.");
	if (e & RF_EventCmdPreempted)
		LOG_DBG("Command preempted by another command with higher "
				"priority. Applies only to multi-client "
				"applications.");
}

static void dumpBleSlave(const char *calling_func, rfc_CMD_BLE_SLAVE_t *cmd) 
{
	LOG_DBG("CMD_BLE_SLAVE: "
		"commandNo: %04x "
	    "status: %u "
		"pNextOp: %p "
		"startTime: %u "
		"start{ "
		"triggertType: %u "
		"bEnaCmd: %u "
		"triggerNo: %u "
		"pastTrig: %u "
		"} "
		"cond { "
		"rule: %u "
		"nSkip: %u "
		"} "
		"channel: %u "
		"whitening{ "
		"init: %u "
		"bOverride: %u "
		"}"
		,
		cmd->commandNo,
		cmd->status,
		cmd->pNextOp,
		cmd->startTime,
		cmd->startTrigger.triggerType,
		cmd->startTrigger.bEnaCmd,
		cmd->startTrigger.triggerNo,
		cmd->startTrigger.pastTrig,
		cmd->condition.rule,
		cmd->condition.nSkip,
		cmd->channel,
		cmd->whitening.init,
		cmd->whitening.bOverride
	);

	rfc_bleSlavePar_t *p = (rfc_bleSlavePar_t *)cmd->pParams;
	LOG_DBG(
		"params@%p: "
		"pRxQ: %p "
		"pTxQ: %p "
		"rxConfig{ "
		"bAutoFlushIgnored: %u "
		"bAutoFlushCrcErr: %u "
		"bAutoFlushEmpty: %u "
		"bIncludeLenByte: %u "
		"bIncludeCrc: %u "
		"bAppendRssi: %u "
		"bAppendStatus: %u "
		"bAppendTimestamp: %u "
		"} "
		,
		p,
		p->pRxQ,
		p->pTxQ,
		p->rxConfig.bAutoFlushIgnored,
		p->rxConfig.bAutoFlushCrcErr,
		p->rxConfig.bAutoFlushEmpty,
		p->rxConfig.bIncludeLenByte,
		p->rxConfig.bIncludeCrc,
		p->rxConfig.bAppendRssi,
		p->rxConfig.bAppendStatus,
		p->rxConfig.bAppendTimestamp
	);

	LOG_DBG(
		"seqStat{ "
		"lastRxSn: %u "
		"lastTxSn: %u "
		"nextTxSn: %u "
		"bFirstPkt: %u "
		"bAutoEmpty: %u "
		"bLlCtrlTx: %u "
		"bLlCtrlAckRx: %u "
		"bLlCtrlAckPending: %u "
		"} "
		"maxNack: %u"
		"maxPkt: %u"
		"accessAddress: %x"
		"crcInit0: %02x "
		"crcInit1: %02x "
		"crcInit2: %02x "
		,
		p->seqStat.lastRxSn,
		p->seqStat.lastTxSn,
		p->seqStat.nextTxSn,
		p->seqStat.bFirstPkt,
		p->seqStat.bAutoEmpty,
		p->seqStat.bLlCtrlTx,
		p->seqStat.bLlCtrlAckRx,
		p->seqStat.bLlCtrlAckPending,
		p->maxNack,
		p->maxPkt,
		p->accessAddress,
		p->crcInit0,
		p->crcInit1,
		p->crcInit2
	);

	LOG_DBG(
		"timeout{ "
		"triggerType: %u "
		"bEnaCmd: %u "
		"trigggerNo: %u "
		"pastTrig: %u "
		"} "
		"timeoutTime: %u "
		"end{ "
		"triggerType: %u "
		"bEnaCmd: %u "
		"trigggerNo: %u "
		"pastTrig: %u "
		"} "
		"endTime: %u "
		,
		p->timeoutTrigger.triggerType,
		p->timeoutTrigger.bEnaCmd,
		p->timeoutTrigger.triggerNo,
		p->timeoutTrigger.pastTrig,
		p->timeoutTime,
		p->endTrigger.triggerType,
		p->endTrigger.bEnaCmd,
		p->endTrigger.triggerNo,
		p->endTrigger.pastTrig,
		p->endTime
	);

	rfc_bleMasterSlaveOutput_t *o = (rfc_bleMasterSlaveOutput_t *)cmd->pOutput;

	LOG_DBG(
		"pOutput@%p: "
		"nTx: %u "
		"nTxAck: %u "
		"nTxCtrl: %u "
		"nTxCtrlAck: %u "
		"nTxCtrlAckAck: %u "
		"nTxRetrans: %u "
		"nTxEntryDone: %u "
		"nRxOk: %u "
		"nRxCtrl: %u "
		"nRxCtrlAck: %u "
		"nRxNok: %u "
		"nRxIgnored: %u "
		"nRxEmptry: %u "
		,
		o,
		o->nTx,
		o->nTxAck,
		o->nTxCtrl,
		o->nTxCtrlAck,
		o->nTxCtrlAckAck,
		o->nTxRetrans,
		o->nTxEntryDone,
		o->nRxOk,
		o->nRxCtrl,
		o->nRxCtrlAck,
		o->nRxNok,
		o->nRxIgnored,
		o->nRxEmpty
	);

	LOG_DBG(
		"nRxBufFull: %u "
		"lastRssi: %d "
		"pktStatus{ "
		"bTimeStampValid: %u "
		"bLastCrcErr: %u "
		"bLastIgnored: %u "
		"bLastEmpty: %u"
		"bLastCtrl: %u"
		"bLastMd: %u"
		"bLastAck: %u"
		"} "
		"timeStamp: %u"
		,
		o->nRxBufFull,
		o->lastRssi,
		o->pktStatus.bTimeStampValid,
		o->pktStatus.bLastCrcErr,
		o->pktStatus.bLastIgnored,
		o->pktStatus.bLastEmpty,
		o->pktStatus.bLastCtrl,
		o->pktStatus.bLastMd,
		o->pktStatus.bLastAck,
		o->timeStamp
	);
}

#if defined(CONFIG_BT_CTLR_DEBUG_PINS)
static void transmit_window_callback(RF_Handle h, RF_RatHandle rh, RF_EventMask e, uint32_t compareCaptureTime) {
	uint32_t now = RF_getCurrentTime();

	/*
	BT_DBG("now: %u rh: %d compareCaptureTime: %u", now, rh, compareCaptureTime );
	describe_event_mask(e);
	*/

	uint32_t begin = driver_data->window_begin_ticks;
	uint32_t duration = driver_data->window_duration_ticks;
	uint32_t interval = driver_data->window_interval_ticks;

	if ( now >= begin + duration ) {

		/* reschedule the next transmit window after 1 interval */
		transmit_window_debug( begin + interval, duration, interval );

	} else if ( now >= begin ) {

		RF_RatConfigCompare channelConfig = {
			.callback = transmit_window_callback,
			.channel = RF_RatChannel1,
			.timeout = begin + duration,
		};
		RF_RatConfigOutput ioConfig = {
			.mode = RF_RatOutputModeClear,
			.select = RF_RatOutputSelectRatGpo2,
		};
		RF_ratCompare(rfHandle, &channelConfig, &ioConfig);

	}
}

static void transmit_window_debug(uint32_t begin, uint32_t duration, uint32_t interval) {

	BT_DBG( "TX Window: [%u,%u] ticks ([%u,%u] us) every %u ticks (%u us)",
		begin,
		begin + duration,
		HAL_TICKER_TICKS_TO_US(begin),
		HAL_TICKER_TICKS_TO_US(begin + duration),
		interval,
		HAL_TICKER_TICKS_TO_US(interval)
	);

	driver_data->window_begin_ticks = begin;
	driver_data->window_duration_ticks = duration;
	driver_data->window_interval_ticks = interval;

	RF_RatConfigCompare channelConfig = {
		.callback = transmit_window_callback,
		.channel = RF_RatChannel1,
		.timeout = begin,
	};
	RF_RatConfigOutput ioConfig = {
		.mode = RF_RatOutputModeSet,
		.select = RF_RatOutputSelectRatGpo2,
	};
	RF_ratCompare(rfHandle, &channelConfig, &ioConfig);
}
#endif /* defined(CONFIG_BT_CTLR_DEBUG_PINS) */

static void pkt_rx(const struct isr_radio_param *isr_radio_param)
{
	bool once = false;
	rfc_dataEntryPointer_t *it;
//	rfc_bleRadioOp_t *op =
//		(rfc_bleRadioOp_t *)RF_getCmdOp(rfHandle, isr_radio_param->ch);

	crc_valid = false;

	for (size_t i = 0; i < CC13XX_CC26XX_NUM_RX_BUF; it->status = DATA_ENTRY_PENDING, ++i) {
		it = &driver_data->rx_entry[i];

		if ((it->status == DATA_ENTRY_FINISHED) && (!once)) {
            size_t offs = it->pData[0];
            uint8_t *data = &it->pData[1];

            bool pdu_is_adv = driver_data->access_address == PDU_AC_ACCESS_ADDR;
            if ( pdu_is_adv ) {
                struct pdu_adv *pdu_rx = (struct pdu_adv *)data;
                if ( PDU_ADV_TYPE_CONNECT_IND == pdu_rx->type ) {
                    offs++;
                    offs--;
                }
            }

            ratmr_t timestamp = 0;
            timestamp |= data[--offs] << 24;
            timestamp |= data[--offs] << 16;
            timestamp |= data[--offs] << 8;
            timestamp |= data[--offs] << 0;

            uint32_t crc = 0;
            crc |= data[--offs] << 16;
            crc |= data[--offs] << 8;
            crc |= data[--offs] << 0;

            size_t len = offs + 1;

            rssi = (int8_t)data[--offs];
            rtc_start = timestamp;
            crc_valid = true;

            /* Add to AA time, PDU + CRC time */
            isr_timer_end = rtc_start + HAL_TICKER_US_TO_TICKS(len + sizeof(crc - 1));

            pdu_is_adv = driver_data->access_address == PDU_AC_ACCESS_ADDR;
            
            if ( pdu_is_adv ) {
                struct pdu_adv *pdu_rx = (struct pdu_adv *)data;
                if ( PDU_ADV_TYPE_CONNECT_IND == pdu_rx->type ) {
                    uint32_t now = cntr_cnt_get();
                    LOG_DBG( "CONNECT_IND: now: %u timestamp: %u (%u us)",
                        now, timestamp, HAL_TICKER_TICKS_TO_US(timestamp));

#if defined(CONFIG_BT_CTLR_DEBUG_PINS)
                    // 1.25 ms, constant value in the case of CONNECT_IND
                    const uint32_t transmitWindowDelay = HAL_TICKER_US_TO_TICKS( 1250 );
                    const uint32_t transmitWindowOffset = HAL_TICKER_US_TO_TICKS( pdu_rx->connect_ind.win_offset * 1250 );
                    const uint32_t transmitWindowSize = HAL_TICKER_US_TO_TICKS( pdu_rx->connect_ind.win_size * 1250 );
                    
                    uint32_t transmitWindowStart = (isr_timer_end + transmitWindowDelay + transmitWindowOffset);
                    uint32_t transmitWindowEnd = (transmitWindowStart + transmitWindowSize);

                    transmit_window_debug( transmitWindowStart, (transmitWindowEnd - transmitWindowStart),
                        HAL_TICKER_US_TO_TICKS( pdu_rx->connect_ind.interval * 1250 ));
#endif /* defined(CONFIG_BT_CTLR_DEBUG_PINS) */
                }
            } else {
                // data pdu
                const struct pdu_data *pdu_data = (const struct pdu_data *)data;
                LOG_DBG("Data PDU\n\t"
                        "len: %u\n\t"
                        "crc: %06x\n\t"
                        "rssi: %d\n\t"
                        "timestamp: %u\n"
                        "{\n\t"
                        "ll_id: %u\n\t"
                        "nesn: %u\n\t"
                        "sn: %u\n\t"
                        "md: %u\n\t"
                        "rfu: %u\n\t"
                        "len: %u\n\t"
                        "llctrl.opcode: %02x\n"
                        "}",
                        len,
                        crc,
                        rssi,
                        timestamp,
                        pdu_data->ll_id,
                        pdu_data->nesn,
                        pdu_data->sn,
                        pdu_data->md,
                        pdu_data->rfu,
                        pdu_data->len,
                        pdu_data->llctrl.opcode
                );
            }

            LL_ASSERT(rx_packet_ptr != NULL);

            memcpy(rx_packet_ptr, data, MIN(len, payload_max_size));

            radio_trx = 1;
            once = true;
		}
	}
}

static void update_adv_data(uint8_t *data, uint8_t len, bool scan_rsp)
{
	driver_data->cmd_ble_adv_payload.payloadType = scan_rsp;

	if (NULL == data || 0 == len) {
		len = 0;
	}

	driver_data->cmd_ble_adv_payload.newLen =
		MIN(len, scan_rsp ? sizeof(driver_data->scan_rsp_data) 
                          : sizeof(driver_data->adv_data));

	driver_data->cmd_ble_adv_payload.pNewData = data;

	RFCDoorbellSendTo((uint32_t)&driver_data->cmd_ble_adv_payload);
}

static void rf_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
	radio_param.h = h;
	radio_param.ch = ch;
	radio_param.e = e;

	isr_radio();
}

void isr_radio(void)
{
	DEBUG_RADIO_ISR(1);

	const isr_radio_param_t *const isr_radio_param = &radio_param;

	RF_EventMask irq = isr_radio_param->e;
	rfc_bleRadioOp_t *op =
		(rfc_bleRadioOp_t *)RF_getCmdOp(rfHandle, isr_radio_param->ch);

	if ( !( CMD_BLE_ADV == op->commandNo || CMD_NOP == op->commandNo ) ) {
		LOG_DBG("now: %u h: %p ch: %u (%s) e: %" PRIx64, cntr_cnt_get(),
			   isr_radio_param->h, isr_radio_param->ch,
			   command_number_to_string(op->commandNo), isr_radio_param->e);
        describe_event_mask(isr_radio_param->e);
        LOG_DBG("%s", ble_status_to_string(op->status));
	}

	if (CMD_BLE_SLAVE == op->commandNo) {
		// pParams->seqStat.bFirstPkt shall be cleared by the radio CPU.
		LOG_DBG("CMD_BLE_SLAVE timestamp: %u", driver_data->cmd_ble_slave_output.timeStamp);
		dumpBleSlave(__func__, & driver_data->cmd_ble_slave);
	}

	if (irq & RF_EventTxDone) {
		if (timer_end_save) {
			timer_end = isr_timer_end;
		}
		radio_trx = 1;
	}

	if (irq & RF_EventRxNOk) {
		LOG_DBG("Received packet with CRC error");
	}

	if (irq & (RF_EventRxOk | RF_EventRxEmpty | RF_EventRxEntryDone)) {
		/* Disable Rx timeout */
		/* 0b1010..RX Cancel -- Cancels pending RX events but do not
		 *			abort a RX-in-progress
		 */
		RF_ratDisableChannel(rfHandle, driver_data->rat_hcto_handle);

		/* Copy the PDU as it arrives, calculates Rx end */
		pkt_rx(isr_radio_param);
		if (timer_aa_save) {
			timer_aa = isr_timer_aa;
		}
		if (timer_end_save) {
			timer_end = isr_timer_end; /* from pkt_rx() */
		}
	}

	if (irq & RF_EventLastCmdDone) {
		/* Disable both comparators */
		RF_ratDisableChannel(rfHandle, driver_data->rat_hcto_handle);
	}

	if (radio_trx && next_radio_cmd) {
		/* Start Rx/Tx in TIFS */
		next_radio_cmd->startTrigger.triggerType = TRIG_ABSTIME;
		next_radio_cmd->startTrigger.pastTrig = true;
		next_radio_cmd->startTime = cntr_cnt_get();

		driver_data->active_command_handle =
			RF_postCmd(rfHandle, (RF_Op *)next_radio_cmd,
				   RF_PriorityNormal, rf_callback, RF_EVENT_MASK);
		next_radio_cmd = NULL;
	}

	isr_cb(isr_cb_param);

	DEBUG_RADIO_ISR(0);
}

void radio_isr_set(radio_isr_cb_t cb, void *param)
{
    irq_disable(LL_RADIO_IRQn);

	isr_cb_param = param;
	isr_cb = cb;

	/* Clear pending interrupts */
	ClearPendingIRQ(LL_RADIO_IRQn);

	irq_enable(LL_RADIO_IRQn);
}

static void rat_deferred_hcto_callback(RF_Handle h, RF_RatHandle rh,
				       RF_EventMask e, uint32_t compareCaptureTime)
{
	LOG_DBG("now: %u", cntr_cnt_get());
	RF_cancelCmd(rfHandle, driver_data->active_command_handle, RF_ABORT_GRACEFULLY);
	driver_data->active_command_handle = -1;
}

static void ble_cc13xx_cc26xx_data_init(void)
{
    if (IS_ENABLED(CONFIG_HWINFO_CC13XX_CC26XX_ALWAYS_USE_FACTORY_DEFAULT) ||
        sys_read32(CCFG_BASE + CCFG_O_IEEE_BLE_0) == 0xFFFFFFFF ||
        sys_read32(CCFG_BASE + CCFG_O_IEEE_BLE_1) == 0xFFFFFFFF) {
	    sys_memcpy_swap(driver_data->device_address, (uint8_t *)(FCFG1_BASE + FCFG1_O_MAC_BLE_0), 6);
    } else {
	    sys_memcpy_swap(driver_data->device_address, (uint8_t *)(CCFG_BASE + CCFG_O_IEEE_BLE_0), 6);
    }

	LOG_DBG("device address: %02x:%02x:%02x:%02x:%02x:%02x", 
            driver_data->device_address[0], driver_data->device_address[1],
	        driver_data->device_address[2], driver_data->device_address[3], 
            driver_data->device_address[4], driver_data->device_address[5]);
            
	/* Ensure that this address is marked as _random_ */
	driver_data->cmd_ble_adv_param.advConfig.deviceAddrType = 1;

	/* Setup circular RX queue (TRM 25.3.2.7) */
	memset(&driver_data->rx_entry[0], 0, sizeof(driver_data->rx_entry[0]));
	memset(&driver_data->rx_entry[1], 0, sizeof(driver_data->rx_entry[1]));

	driver_data->rx_entry[0].pNextEntry = (uint8_t *)&driver_data->rx_entry[1];
	driver_data->rx_entry[0].config.type = DATA_ENTRY_TYPE_PTR;
	driver_data->rx_entry[0].config.lenSz = 1;
	driver_data->rx_entry[0].status = DATA_ENTRY_PENDING;
	driver_data->rx_entry[0].length = sizeof(driver_data->rx_data[0]);
	driver_data->rx_entry[0].pData = driver_data->rx_data[0];

	driver_data->rx_entry[1].pNextEntry = (uint8_t *)&driver_data->rx_entry[0];
	driver_data->rx_entry[1].config.type = DATA_ENTRY_TYPE_PTR;
	driver_data->rx_entry[1].config.lenSz = 1;
	driver_data->rx_entry[0].status = DATA_ENTRY_PENDING;
	driver_data->rx_entry[1].length = sizeof(driver_data->rx_data[1]);
	driver_data->rx_entry[1].pData = driver_data->rx_data[1];

	driver_data->rx_queue.pCurrEntry = (uint8_t *)&driver_data->rx_entry[0];
	driver_data->rx_queue.pLastEntry = NULL;

	/* Setup circular TX queue (TRM 25.3.2.7) */
	memset(&driver_data->tx_entry[0], 0, sizeof(driver_data->tx_entry[0]));
	memset(&driver_data->tx_entry[1], 0, sizeof(driver_data->tx_entry[1]));

	driver_data->tx_entry[0].pNextEntry = (uint8_t *)&driver_data->tx_entry[1];
	driver_data->tx_entry[0].config.type = DATA_ENTRY_TYPE_PTR;
	driver_data->tx_entry[0].config.lenSz = 0;
	driver_data->tx_entry[0].config.irqIntv = 0;
	driver_data->tx_entry[0].status = DATA_ENTRY_FINISHED;
	driver_data->tx_entry[0].length = 0;
	driver_data->tx_entry[0].pData = driver_data->tx_data[0];

	driver_data->tx_entry[1].pNextEntry = (uint8_t *)&driver_data->tx_entry[0];
	driver_data->tx_entry[1].config.type = DATA_ENTRY_TYPE_PTR;
	driver_data->tx_entry[1].config.lenSz = 1;
	driver_data->tx_entry[0].status = DATA_ENTRY_FINISHED;
	driver_data->tx_entry[1].length = sizeof(driver_data->tx_data[1]);
	driver_data->tx_entry[1].pData = driver_data->tx_data[1];

	driver_data->tx_queue.pCurrEntry = (uint8_t *)&driver_data->tx_entry[0];
	driver_data->tx_queue.pLastEntry = NULL;

	driver_data->active_command_handle = -1;

	RF_RatConfigCompare_init(
		(RF_RatConfigCompare *)&driver_data->rat_hcto_compare);
	driver_data->rat_hcto_compare.callback = rat_deferred_hcto_callback;
}

/*
 * A (high) arbitrarily-chosen 32-bit number that the RAT will (likely) not
 * encounter soon after reset
 */
#define ISR_LATENCY_MAGIC 0xfedcba98
static void tmp_cb(void *param)
{
	uint32_t t1 = *(uint32_t *)param;
	uint32_t t2 = HAL_TICKER_TICKS_TO_US(cntr_cnt_get());

	isr_latency = t2 - t1;
	/* Mark as done */
	*(uint32_t *)param = ISR_LATENCY_MAGIC;
}

static void get_isr_latency(void)
{
	volatile uint32_t tmp;

	radio_isr_set(tmp_cb, (void *)&tmp);

	/* Reset TMR to zero */
	/* (not necessary using the ISR_LATENCY_MAGIC approach) */

	tmp = HAL_TICKER_TICKS_TO_US(cntr_cnt_get());

	radio_disable();
	while (tmp != ISR_LATENCY_MAGIC) {
	}

	irq_disable(LL_RADIO_IRQn);
}

void radio_setup(void)
{
	RF_Params rfParams;

	RF_Params_init(&rfParams);

	ble_cc13xx_cc26xx_data_init();

	rfHandle = RF_open(&rfObject, &RF_modeBle,
			   (RF_RadioSetup *)&driver_data->cmd_ble_radio_setup,
			   &rfParams);
	LL_ASSERT(rfHandle);

	driver_data->cmd_set_frequency.frequency = channel_table[0].frequency;
	driver_data->cmd_set_frequency.fractFreq = 0x0000;
	driver_data->whiten = channel_table[0].whitening;
	driver_data->chan = 37;
	RF_runCmd(rfHandle, (RF_Op *)&driver_data->cmd_set_frequency,
		  RF_PriorityNormal, NULL, RF_EventLastCmdDone);

	/* Get warmpup times. They are used in TIFS calculations */
	tx_warmup = 0; /* us */
	rx_warmup = 0; /* us */

	get_isr_latency();
}

void radio_reset(void)
{
	irq_disable(LL_RADIO_IRQn);

#if defined(CONFIG_BT_CTLR_DF) && !defined(CONFIG_ZTEST)
	radio_df_reset();
#endif /* CONFIG_BT_CTLR_DF && !CONFIG_ZTEST */

	// hal_radio_reset();
}

void radio_stop(void)
{
	// hal_radio_stop();
}

void radio_phy_set(uint8_t phy, uint8_t flags)
{
	ARG_UNUSED(phy);
	ARG_UNUSED(flags);

	/* This function should set one of three modes:
	 * - BLE 1 Mbps
	 * - BLE 2 Mbps
	 * - Coded BLE
	 * We set this on radio_setup() function. There radio is
	 * setup for DataRate of 1 Mbps.
	 * For now this function does nothing. In the future it
	 * may have to reset the radio
	 * to the 2 Mbps (the only other mode supported by Vega radio).
	 */
}

void radio_tx_power_set(int8_t power)
{
	RF_setTxPower(rfHandle,
		      RF_TxPowerTable_findValue(
			      (RF_TxPowerTable_Entry *)RF_BLE_txPowerTable,
			      power));
}

void radio_tx_power_max_set(void)
{
	RF_setTxPower(rfHandle,
		      RF_TxPowerTable_findValue(
			      (RF_TxPowerTable_Entry *)RF_BLE_txPowerTable,
			      RF_TxPowerTable_MAX_DBM));
}

void radio_freq_chan_set(uint32_t chan)
{
	/*
	 * The LLL expects the channel number to be computed as
	 * 2400 + chan [MHz]. Therefore a compensation of -2 MHz
	 * has been provided.
	 */
	LL_ASSERT(2 <= chan && chan <= 80);
	LL_ASSERT(!(chan & 0x1));

	uint8_t index = chan - 2;
	driver_data->whiten = channel_table[index].whitening;

	switch (chan) {
	case 2:
		driver_data->chan = 37;
		break;
	case 4 ... 24:
		driver_data->chan = (chan - 4) / 2;
		break;
	case 26:
		driver_data->chan = 38;
		break;
	case 28 ... 78:
		driver_data->chan = (chan - 6) / 2;
		break;
	case 80:
		driver_data->chan = 39;
		break;
	}
}

void radio_whiten_iv_set(uint32_t iv)
{
	/*
	 * The LLL expects the channel number to be computed as
	 * 2400 + ch_num [MHz]. Therefore a compensation of
	 * -2 MHz has been provided.
	 */

/*
	LL_ASSERT(2 <= iv && iv <= 80);

	uint8_t index = iv - 2;

	driver_data->whiten = channel_table[index].whitening;
*/
}

void radio_aa_set(const uint8_t *aa)
{
	driver_data->access_address = *(uint32_t*)aa;
}

void radio_pkt_configure(uint8_t bits_len, uint8_t max_len, uint8_t flags)
{
	payload_max_size = max_len;
}

void radio_pkt_rx_set(void *rx_packet)
{
	if (!driver_data->ignore_next_rx) {
		LOG_DBG("rx_packet: %p", rx_packet);
	}
	rx_packet_ptr = rx_packet;
}

void radio_pkt_tx_set(void *tx_packet)
{
	char* typespecstr = "(unknown)";
	bool pdu_is_adv = driver_data->access_address == PDU_AC_ACCESS_ADDR;
	int16_t command_no = -1;

	tx_packet_ptr = tx_packet;
	next_tx_radio_cmd = NULL;

	/* There are a number of TX commands that simply cannot be performed
	 * in isolation using this platform. E.g. SCAN_RSP. There is no command
	 * to initiate that particular PDU. It is typically sent automatically
	 * after a SCAN_REQ is received (which is also automatic).
	 */
	driver_data->ignore_next_tx = false;

	if (pdu_is_adv) {
		const struct pdu_adv *pdu_adv =
			(const struct pdu_adv *)tx_packet;
		if ((pdu_adv->type == PDU_ADV_TYPE_ADV_IND) 
        ||  (pdu_adv->type == PDU_ADV_TYPE_NONCONN_IND)
        ||  (pdu_adv->type == PDU_ADV_TYPE_SCAN_IND)) {
			update_adv_data((uint8_t *)pdu_adv->adv_ind.data,
					pdu_adv->len -
						sizeof(pdu_adv->adv_ind.addr),
					false);
		}

		switch (pdu_adv->type) {
		case PDU_ADV_TYPE_ADV_IND:
			command_no = CMD_BLE_ADV;
			typespecstr = "ADV: ADV_IND";
			next_tx_radio_cmd =
				(rfc_bleRadioOp_t *)&driver_data->cmd_ble_adv;
			break;
		case PDU_ADV_TYPE_DIRECT_IND:
			typespecstr = "ADV: DIRECT_IND";
			break;
		case PDU_ADV_TYPE_NONCONN_IND:
			typespecstr = "ADV: NONCONN_IND";
			break;
		case PDU_ADV_TYPE_SCAN_REQ:
			typespecstr = "ADV: SCAN_REQ";
			break;
		case PDU_ADV_TYPE_SCAN_RSP:
			driver_data->ignore_next_tx = true;
			break;
		case PDU_ADV_TYPE_CONNECT_IND:
			typespecstr = "ADV: CONNECT_IND";
			break;
		case PDU_ADV_TYPE_SCAN_IND:
			typespecstr = "SCAN_IND";
			break;
		case PDU_ADV_TYPE_EXT_IND:
			typespecstr = "ADV: EXT_IND";
			break;
		case PDU_ADV_TYPE_AUX_CONNECT_RSP:
			typespecstr = "ADV: AUX_CONNECT_RSP";
			break;
		default:
			typespecstr = "ADV: (unknown)";
			break;
		}

	} else {
		/* PDU is data */
		const struct pdu_data *pdu_data =
			(const struct pdu_data *)tx_packet;
		switch (pdu_data->ll_id) {
		case PDU_DATA_LLID_DATA_CONTINUE:
			typespecstr = "DATA: DATA_CONTINUE";
            driver_data->tx_entry[0].length = MIN((PDU_DC_LL_HEADER_SIZE + pdu_data->len) ,CC13XX_CC26XX_TX_BUF_SIZE);
			memcpy(&driver_data->tx_data[0], pdu_data, driver_data->tx_entry[0].length);
			driver_data->tx_entry[0].status = DATA_ENTRY_PENDING;
			break;
		case PDU_DATA_LLID_DATA_START:
			typespecstr = "DATA: DATA_START";
			break;
		case PDU_DATA_LLID_CTRL:
			switch (((struct pdu_data_llctrl *)pdu_data)->opcode) {
			case PDU_DATA_LLCTRL_TYPE_CONN_UPDATE_IND:
				typespecstr = "DATA: CTRL: CONN_UPDATE_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_CHAN_MAP_IND:
				typespecstr = "DATA: CTRL: CHAN_MAP_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_TERMINATE_IND:
				typespecstr = "DATA: CTRL: TERMINATE_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_ENC_REQ:
				typespecstr = "DATA: CTRL: ENC_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_ENC_RSP:
				typespecstr = "DATA: CTRL: ENC_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_START_ENC_REQ:
				typespecstr = "DATA: CTRL: START_ENC_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_START_ENC_RSP:
				typespecstr = "DATA: CTRL: START_ENC_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
				typespecstr = "DATA: CTRL: UNKNOWN_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_FEATURE_REQ:
				typespecstr = "DATA: CTRL: FEATURE_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_FEATURE_RSP:
				typespecstr = "DATA: CTRL: FEATURE_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_PAUSE_ENC_REQ:
				typespecstr = "DATA: CTRL: PAUSE_ENC_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_PAUSE_ENC_RSP:
				typespecstr = "DATA: CTRL: PAUSE_ENC_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_VERSION_IND:
				typespecstr = "DATA: CTRL: VERSION_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_REJECT_IND:
				typespecstr = "DATA: CTRL: REJECT_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ:
				typespecstr = "DATA: CTRL: CONN_PARAM_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_CONN_PARAM_RSP:
				typespecstr = "DATA: CTRL: CONN_PARAM_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
				typespecstr = "DATA: CTRL: REJECT_EXT_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_PING_REQ:
				typespecstr = "DATA: CTRL: PING_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_PING_RSP:
				typespecstr = "DATA: CTRL: PING_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_LENGTH_REQ:
				typespecstr = "DATA: CTRL: LENGTH_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_LENGTH_RSP:
				typespecstr = "DATA: CTRL: LENGTH_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_PHY_REQ:
				typespecstr = "DATA: CTRL: PHY_REQ";
				break;
			case PDU_DATA_LLCTRL_TYPE_PHY_RSP:
				typespecstr = "DATA: CTRL: PHY_RSP";
				break;
			case PDU_DATA_LLCTRL_TYPE_PHY_UPD_IND:
				typespecstr = "DATA: CTRL: PHY_UPD_IND";
				break;
			case PDU_DATA_LLCTRL_TYPE_MIN_USED_CHAN_IND:
				typespecstr = "DATA: CTRL: MIN_USED_CHAN_IND";
				break;
			default:
				typespecstr = "DATA: CTRL: (unknown)";
				break;
			}
			break;

		default:
			typespecstr = "DATA: (unknown)";
			break;
		}

#if 1
		if ( NULL != typespecstr ) {
			LOG_DBG(
				"PDU %s: {\n\t"
				"ll_id: %u\n\t"
				"nesn: %u\n\t"
				"sn: %u\n\t"
				"md: %u\n\t"
				"rfu: %u\n\t"
				"len: %u\n\t"
				"}"
				,
				typespecstr,
				pdu_data->ll_id,
				pdu_data->nesn,
				pdu_data->sn,
				pdu_data->md,
				pdu_data->rfu,
				pdu_data->len
			);
		}
#endif
	}

	if (driver_data->ignore_next_tx) {
		LOG_DBG("ignoring next TX %s", command_number_to_string(command_no));
		return;
	}

	if ( -1 == command_no || NULL == next_tx_radio_cmd ) {
//		LL_ASSERT(command_no != -1);
//		LL_ASSERT(next_tx_radio_cmd != NULL);
		return;
	}

	next_tx_radio_cmd->commandNo = command_no;

	if ((CMD_BLE_ADV == command_no) 
    ||  (CMD_BLE_ADV_DIR == command_no)
    ||  (CMD_BLE_ADV_NC == command_no)
    ||  (CMD_BLE_ADV_SCAN == command_no)) {
		driver_data->ignore_next_rx = true;
	}
}

uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return tx_warmup;
}

uint32_t radio_tx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 0;
}

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return rx_warmup;
}

uint32_t radio_rx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 16 + 2 * Rx_OVHD + RX_MARGIN + isr_latency + Rx_OVHD;
}

void radio_rx_enable(void)
{
	radio_tmr_start_now(false);
}

void radio_tx_enable(void)
{
	if (!driver_data->ignore_next_tx) {
		LL_ASSERT(next_tx_radio_cmd != NULL);
		next_radio_cmd = next_tx_radio_cmd;
		radio_tmr_start_now(true);
	}
}

void radio_disable(void)
{
	//LOG_DBG("now: %u", cntr_cnt_get());
	/*
	 * 0b1011..Abort All - Cancels all pending events and abort any
	 * sequence-in-progress
	 */
	RFCDoorbellSendTo(CMDR_DIR_CMD(CMD_ABORT));

	/* Set all RX entries to empty */
	RFCDoorbellSendTo((uint32_t)&driver_data->cmd_clear_rx);

	/* generate interrupt to get into isr_radio */
	RF_postCmd(rfHandle, (RF_Op *)&driver_data->cmd_nop, RF_PriorityNormal,
		   rf_callback, RF_EventLastCmdDone);

	next_radio_cmd = NULL;
}

void radio_status_reset(void)
{
	radio_trx = 0;
}

uint32_t radio_is_ready(void)
{
	/* Always false. LLL expects the radio not to be in idle/Tx/Rx state */
	return 0;
}

uint32_t radio_is_done(void)
{
	return radio_trx;
}

uint32_t radio_has_disabled(void)
{
    /* Not used */
	return 0;
}

uint32_t radio_is_idle(void)
{
	return 1;
}

void radio_crc_configure(uint32_t polynomial, uint32_t iv)
{
	//LOG_DBG("polynomial: 0x%06x iv: 0x%06x", polynomial, iv);
	driver_data->polynomial = polynomial;
	driver_data->iv = iv;
}

uint32_t radio_crc_is_valid(void)
{
    bool r = crc_valid;

	/* only valid for first call */
	crc_valid = false;

	return r;
}

void *radio_pkt_empty_get(void)
{
	return _pkt_empty;
}

void *radio_pkt_scratch_get(void)
{
	return _pkt_scratch;
}

#if defined(CONFIG_BT_CTLR_LE_ENC) && \
	defined(HAL_RADIO_PDU_LEN_MAX) && \
	(!defined(CONFIG_BT_CTLR_DATA_LENGTH_MAX) || \
	 (CONFIG_BT_CTLR_DATA_LENGTH_MAX < (HAL_RADIO_PDU_LEN_MAX - 4)))
static uint8_t MALIGN(4) _pkt_decrypt[MAX((HAL_RADIO_PDU_LEN_MAX + 3),
				       PDU_AC_LL_SIZE_MAX)];

void *radio_pkt_decrypt_get(void)
{
	return _pkt_decrypt;
}
#elif !defined(HAL_RADIO_PDU_LEN_MAX)
#error "Undefined HAL_RADIO_PDU_LEN_MAX."
#endif

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_SYNC_ISO)
/* Dedicated Rx PDU Buffer for Control PDU independent of node_rx with BIS Data
 * PDU buffer. Note this buffer will be used to store whole PDUs, not just the BIG control payload.
 */
static uint8_t pkt_big_ctrl[offsetof(struct pdu_bis, payload) + sizeof(struct pdu_big_ctrl)];

void *radio_pkt_big_ctrl_get(void)
{
	return pkt_big_ctrl;
}
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_SYNC_ISO */


void radio_switch_complete_and_rx(uint8_t phy_rx)
{
	/*  0b0110..RX Start @ T1 Timer Compare Match (EVENT_TMR = T1_CMP) */
	if (!driver_data->ignore_next_rx && NULL == next_radio_cmd) {
		next_radio_cmd =
			(rfc_bleRadioOp_t *)&driver_data->cmd_ble_generic_rx;
	}

	/* the margin is used to account for any overhead in radio switching */
	next_warmup = rx_warmup + RX_MARGIN;
}

void radio_switch_complete_and_tx(uint8_t phy_rx, uint8_t flags_rx,
				  uint8_t phy_tx, uint8_t flags_tx)
{
	/*  0b0010..TX Start @ T1 Timer Compare Match (EVENT_TMR = T1_CMP) */
	if (driver_data->ignore_next_tx) {
		next_radio_cmd = next_tx_radio_cmd;
	}

	/* the margin is used to account for any overhead in radio switching */
	next_warmup = tx_warmup + TX_MARGIN;
}

void radio_switch_complete_and_disable(void)
{
	RF_ratDisableChannel(rfHandle, driver_data->rat_hcto_handle);
	next_radio_cmd = NULL;
}

void radio_rssi_measure(void)
{
	rssi = RF_GET_RSSI_ERROR_VAL;

}

uint32_t radio_rssi_get(void)
{
    return (uint32_t)-rssi;
}

void radio_rssi_status_reset(void)
{
	rssi = RF_GET_RSSI_ERROR_VAL;
}

uint32_t radio_rssi_is_ready(void)
{	
    return (rssi != RF_GET_RSSI_ERROR_VAL);
}

void radio_filter_configure(uint8_t bitmask_enable, uint8_t bitmask_addr_type,
			    uint8_t *bdaddr)
{

}

void radio_filter_disable(void)
{

}

void radio_filter_status_reset(void)
{

}

uint32_t radio_filter_has_match(void)
{
	return true;
}

uint32_t radio_filter_match_get(void)
{
	return 0;
}

void radio_bc_configure(uint32_t n)
{

}

void radio_bc_status_reset(void)
{

}

uint32_t radio_bc_has_match(void)
{
	return 0;
}

void radio_tmr_status_reset(void)
{
	timer_aa_save = 0;
	timer_end_save = 0;
}

void radio_tmr_tx_status_reset(void)
{

}

void radio_tmr_rx_status_reset(void)
{

}

void radio_tmr_tx_enable(void)
{

}

void radio_tmr_rx_enable(void)
{

}

void radio_tmr_tx_disable(void)
{

}

void radio_tmr_rx_disable(void)
{

}

void radio_tmr_tifs_set(uint32_t tifs)
{
	timer_tifs = tifs;
}

/* Start the radio after ticks_start (ticks) + remainder (us) time */
static uint32_t radio_tmr_start_hlp(uint8_t trx, uint32_t ticks_start, uint32_t remainder)
{
	rfc_bleRadioOp_t *radio_start_now_cmd = NULL;
	uint32_t now = cntr_cnt_get();

	/* Save it for later */
	/* rtc_start = ticks_start; */

	/* Convert ticks to us and use just EVENT_TMR */
	rtc_diff_start_us = HAL_TICKER_TICKS_TO_US(rtc_start - now);

	skip_hcto = 0;
	if (rtc_diff_start_us > 0x80000000) {
		LOG_DBG("dropped command");
		/* ticks_start already passed. Don't start the radio */
		rtc_diff_start_us = 0;

		/* Ignore time out as well */
		skip_hcto = 1;
		remainder = 0;
		//return remainder;
	}

	if (trx) {
		if (remainder <= MIN_CMD_TIME) {
			/* 0b0001..TX Start Now */
			radio_start_now_cmd = next_tx_radio_cmd;
			remainder = 0;
		} else {
			/*
			 * 0b0010..TX Start @ T1 Timer Compare Match
			 *         (EVENT_TMR = T1_CMP)
			 */
			next_radio_cmd = next_tx_radio_cmd;
		}
		timer_ready = remainder + tx_warmup;

		if (driver_data->ignore_next_tx) {
			LOG_DBG("ignoring next TX command");
			return remainder;
		}
	} else {
		if (next_radio_cmd == NULL) {
			next_radio_cmd = (rfc_bleRadioOp_t *)&driver_data ->cmd_ble_generic_rx;
		}

		if (remainder <= MIN_CMD_TIME) {
			/* 0b0101..RX Start Now */
			radio_start_now_cmd = next_radio_cmd;
			remainder = 0;
		} else {
			/*
			 * 0b0110..RX Start @ T1 Timer Compare Match
			 *         (EVENT_TMR = T1_CMP)
			 */
		}
		timer_ready = remainder + rx_warmup;

		if (driver_data->ignore_next_rx) {
			return remainder;
		}
	}

	if (radio_start_now_cmd) {

		/* trigger Rx/Tx Start Now */
		radio_start_now_cmd->channel = driver_data->chan;
		if ( CMD_BLE_SLAVE == next_radio_cmd->commandNo ) {
			// timing is done in radio_set_up_slave_cmd()
		} else {
			radio_start_now_cmd->startTime = now;
			radio_start_now_cmd->startTrigger.triggerType = TRIG_ABSTIME;
			radio_start_now_cmd->startTrigger.pastTrig = true;
		}
		driver_data->active_command_handle =
			RF_postCmd(rfHandle, (RF_Op *)radio_start_now_cmd,
				   RF_PriorityNormal, rf_callback, RF_EVENT_MASK);
	} else {
		if (next_radio_cmd != NULL) {
			/* enable T1_CMP to trigger the SEQCMD */
			/* trigger Rx/Tx Start Now */
			next_radio_cmd->channel = driver_data->chan;
			if ( CMD_BLE_SLAVE == next_radio_cmd->commandNo ) {
				// timing is done in radio_set_up_slave_cmd()
			} else {
				next_radio_cmd->startTime = now + remainder;
				next_radio_cmd->startTrigger.triggerType = TRIG_ABSTIME;
				next_radio_cmd->startTrigger.pastTrig = true;
			}
			driver_data->active_command_handle =
				RF_postCmd(rfHandle, (RF_Op *)next_radio_cmd,
					   RF_PriorityNormal, rf_callback,
					   RF_EVENT_MASK);
		}
	}

	return remainder;
}

uint32_t radio_tmr_start(uint8_t trx, uint32_t ticks_start, uint32_t remainder)
{
	if ((!(remainder / 1000000UL)) || (remainder & 0x80000000)) {
		ticks_start--;
		remainder += 30517578UL;
	}
	remainder /= 1000000UL;

	return radio_tmr_start_hlp(trx, ticks_start, remainder);
}

uint32_t radio_tmr_start_tick(uint8_t trx, uint32_t tick)
{
	/* Setup compare event with min. 1 us offset */
	uint32_t remainder_us = 1;

	return radio_tmr_start_hlp(trx, tick, remainder_us);
}

uint32_t radio_tmr_start_us(uint8_t trx, uint32_t start_us)
{
	/* Setup compare event with min. 1 us offset */
    uint32_t remainder_us = 1;

	return radio_tmr_start_hlp(trx, HAL_TICKER_US_TO_TICKS(start_us), remainder_us);
}

uint32_t radio_tmr_start_now(uint8_t trx)
{
	return radio_tmr_start(trx, cntr_cnt_get(), 0);
}

uint32_t radio_tmr_start_get(void)
{
	return rtc_start;
}

void radio_tmr_stop(void)
{
	/* Deep Sleep Mode (DSM)? */
}

void radio_tmr_hcto_configure(uint32_t hcto)
{
	if (skip_hcto) {
		skip_hcto = 0;
		return;
	}

	driver_data->rat_hcto_compare.timeout = hcto;

	/* 0b1001..RX Stop @ T2 Timer Compare Match (EVENT_TMR = T2_CMP) */
	driver_data->rat_hcto_handle =
		RF_ratCompare(rfHandle, &driver_data->rat_hcto_compare, NULL);
}

void radio_tmr_aa_capture(void)
{
	timer_aa_save = 1;
}

uint32_t radio_tmr_aa_get(void)
{
	LOG_DBG("return tmr_aa (%u) - rtc_diff_start_us (%u) = %u", timer_aa, rtc_diff_start_us, timer_aa - rtc_diff_start_us);
	return timer_aa - rtc_diff_start_us;
}

static uint32_t radio_timer_aa;

void radio_tmr_aa_save(uint32_t aa)
{
	LOG_DBG("set radio_timer_aa = %u", aa);
	radio_timer_aa = aa;
}

uint32_t radio_tmr_aa_restore(void)
{
	LOG_DBG("return radio_timer_aa = %u", radio_timer_aa);
	return radio_timer_aa;
}

uint32_t radio_tmr_ready_get(void)
{
	LOG_DBG("return tmr_ready (%u) - rtc_diff_start_us (%u) = %u", timer_ready, rtc_diff_start_us, timer_ready - rtc_diff_start_us);
	return timer_ready - rtc_diff_start_us;
}

static uint32_t radio_timer_ready;

void radio_tmr_ready_save(uint32_t ready)
{
	radio_timer_ready = ready;
}

uint32_t radio_tmr_ready_restore(void)
{
	return radio_timer_ready;
}

void radio_tmr_end_capture(void)
{
	timer_end_save = 1;
}

uint32_t radio_tmr_end_get(void)
{
	return timer_end - rtc_start;
}

uint32_t radio_tmr_tifs_base_get(void)
{
	return radio_tmr_end_get();
}

#if defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)
static uint32_t timer_sample_val;
#endif /* CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */

void radio_tmr_sample(void)
{
#if defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)

#else /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */

#endif /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */
}

uint32_t radio_tmr_sample_get(void)
{
#if defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)
	return timer_sample_val;
#else /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */
	return 0;
#endif /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */
}

static void *radio_ccm_ext_rx_pkt_set(struct ccm *cnf, uint8_t phy, uint8_t pdu_type, void *pkt)
{
	return _pkt_scratch;
}

void *radio_ccm_rx_pkt_set(struct ccm *cnf, uint8_t phy, void *pkt)
{
	return radio_ccm_ext_rx_pkt_set(cnf, phy, RADIO_PKT_CONF_PDU_TYPE_DC, pkt);
}

void *radio_ccm_iso_rx_pkt_set(struct ccm *cnf, uint8_t phy, uint8_t pdu_type, void *pkt)
{
	return radio_ccm_ext_rx_pkt_set(cnf, phy, pdu_type, pkt);
}

static void *radio_ccm_ext_tx_pkt_set(struct ccm *cnf, uint8_t pdu_type, void *pkt)
{
	return _pkt_scratch;
}

void *radio_ccm_tx_pkt_set(struct ccm *cnf, void *pkt)
{
	return radio_ccm_ext_tx_pkt_set(cnf, RADIO_PKT_CONF_PDU_TYPE_DC, pkt);
}

void *radio_ccm_iso_tx_pkt_set(struct ccm *cnf, uint8_t pdu_type, void *pkt)
{
	return radio_ccm_ext_tx_pkt_set(cnf, pdu_type, pkt);
}

uint32_t radio_ccm_is_done(void)
{
	return 0;
}

uint32_t radio_ccm_mic_is_valid(void)
{
	return 0;
}

#if defined(CONFIG_BT_CTLR_PRIVACY)
void radio_ar_configure(uint32_t nirk, void *irk, uint8_t flags)
{

}

uint32_t radio_ar_match_get(void)
{
	return 0;
}

void radio_ar_status_reset(void)
{

}

uint32_t radio_ar_has_match(void)
{
	return 0;
}

uint8_t radio_ar_resolve(const uint8_t *addr)
{
	return 0;
}
#endif /* CONFIG_BT_CTLR_PRIVACY */
